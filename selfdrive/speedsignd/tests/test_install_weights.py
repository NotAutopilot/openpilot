"""Install-path helper: local copy and checksummed fetch onto /data."""
from pathlib import Path

import pytest

from openpilot.selfdrive.speedsignd.install import (
  MIN_ONNX_BYTES,
  fetch_and_install,
  install_weights,
  sha256_file,
  weights_present,
  weights_sha_matches,
  weights_status_summary,
)
from openpilot.selfdrive.speedsignd.weights_manifest import ASSET_SHA256, release_asset_url
from scripts.nap.install_speed_sign_weights import main
from scripts.nap.script_lifecycle import script_reboots_on_exit


def test_install_copies_onnx_to_dest(tmp_path):
  src = tmp_path / "speed_sign.onnx"
  src.write_bytes(b"tiny-onnx")
  dest = tmp_path / "nap" / "speed_sign.onnx"
  out = install_weights(str(src), str(dest))
  assert Path(out).read_bytes() == b"tiny-onnx"


def test_cli_install(tmp_path):
  src = tmp_path / "m.onnx"
  src.write_bytes(b"abc")
  dest = tmp_path / "out.onnx"
  assert main([str(src), "--out", str(dest)]) == 0
  assert dest.read_bytes() == b"abc"


def test_fetch_verifies_checksum(tmp_path):
  src = tmp_path / "speed_sign.onnx"
  src.write_bytes(b"compact-yolo")
  digest = sha256_file(str(src))
  dest = tmp_path / "nap" / "speed_sign.onnx"
  out = fetch_and_install(dest=str(dest), url=src.resolve().as_uri(), sha256=digest)
  assert Path(out).read_bytes() == b"compact-yolo"


def test_fetch_rejects_bad_checksum(tmp_path):
  src = tmp_path / "speed_sign.onnx"
  src.write_bytes(b"nope")
  dest = tmp_path / "nap" / "speed_sign.onnx"
  with pytest.raises(RuntimeError, match="SHA-256"):
    fetch_and_install(dest=str(dest), url=src.resolve().as_uri(), sha256="0" * 64)


def test_cli_download_file_url(tmp_path):
  src = tmp_path / "pub.onnx"
  src.write_bytes(b"from-url")
  dest = tmp_path / "installed.onnx"
  digest = sha256_file(str(src))
  assert main(["--url", src.resolve().as_uri(), "--out", str(dest), "--sha256", digest]) == 0
  assert dest.read_bytes() == b"from-url"


def test_manifest_points_at_github_release():
  url = release_asset_url()
  assert "speed-sign-onnx-v2" in url
  assert url.endswith("speed_sign.onnx")
  assert ASSET_SHA256 == "1131200b7fe2d400596e09d7680fc3a26a08dcd8952acfdf261170bdb22d3d28"


def test_weights_status_missing_and_installed(tmp_path):
  missing = tmp_path / "none.onnx"
  assert not weights_present(str(missing))
  assert weights_status_summary(str(missing)) == "Missing"

  tiny = tmp_path / "tiny.onnx"
  tiny.write_bytes(b"too-small")
  assert not weights_present(str(tiny))
  assert weights_status_summary(str(tiny)) == "Missing"

  ok = tmp_path / "speed_sign.onnx"
  ok.write_bytes(b"x" * MIN_ONNX_BYTES)
  assert weights_present(str(ok))
  # Size alone is not Installed. v1 at this path is Out of date.
  assert weights_status_summary(str(ok)) == "Out of date"
  assert not weights_sha_matches(str(ok))


def test_weights_status_installed_only_when_sha_matches(tmp_path, monkeypatch):
  ok = tmp_path / "speed_sign.onnx"
  ok.write_bytes(b"v2-weights" + (b"x" * MIN_ONNX_BYTES))
  digest = sha256_file(str(ok))
  monkeypatch.setattr("openpilot.selfdrive.speedsignd.install.ASSET_SHA256", digest)
  assert weights_sha_matches(str(ok))
  assert weights_status_summary(str(ok)).startswith("Installed")


def test_fetch_replaces_existing_file(tmp_path):
  dest = tmp_path / "nap" / "speed_sign.onnx"
  dest.parent.mkdir()
  dest.write_bytes(b"old-v1-weights-keep-me-not")
  src = tmp_path / "speed_sign.onnx"
  src.write_bytes(b"new-v2-weights")
  digest = sha256_file(str(src))
  out = fetch_and_install(dest=str(dest), url=src.resolve().as_uri(), sha256=digest)
  assert Path(out).read_bytes() == b"new-v2-weights"


def test_install_script_does_not_reboot_and_settings_wire_it():
  assert not script_reboots_on_exit("scripts.nap.install_speed_sign_weights")
  root = Path(__file__).resolve().parents[3]
  nap = (root / "selfdrive" / "ui" / "layouts" / "settings" / "nap.py").read_text(encoding="utf-8")
  mici = (root / "selfdrive" / "ui" / "mici" / "layouts" / "settings" / "nap.py").read_text(encoding="utf-8")
  content = (root / "selfdrive" / "ui" / "layouts" / "settings" / "nap_content.py").read_text(encoding="utf-8")
  assert "Install weights" in nap
  assert "install weights" in mici
  assert "scripts.nap.install_speed_sign_weights" in nap
  assert "scripts.nap.install_speed_sign_weights" in mici
  assert "INSTALL_SPEED_SIGN_WEIGHTS_INSTRUCTIONS" in content
  assert "weights_status_summary" in nap
  assert "weights_status_summary" in mici
  assert "NO WT" in content
