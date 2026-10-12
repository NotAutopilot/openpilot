"""Install-path helper: local copy and checksummed fetch onto /data."""
import os
import zipfile
from pathlib import Path

import pytest

from openpilot.selfdrive.speedsignd.install import (
  MIN_ONNX_BYTES,
  SETTINGS_RUNTIME_MISSING,
  clang_fallback_warning,
  clear_runtime_status_cache,
  fetch_and_install,
  import_vendored_runtime,
  install_runtime_wheel,
  install_status_summary,
  install_weights,
  runtime_status_summary,
  sha256_file,
  vendored_runtime_ready,
  vendored_sys_path,
  weights_present,
  weights_sha_matches,
  weights_status_summary,
)
from openpilot.selfdrive.speedsignd.weights_manifest import (
  ASSET_SHA256,
  ORT_BYTES,
  ORT_SHA256,
  ORT_URL,
  ORT_VERSION,
  ORT_WHEEL,
  release_asset_url,
)
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
  assert main([str(src), "--out", str(dest), "--skip-runtime"]) == 0
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
  assert main([
    "--url", src.resolve().as_uri(), "--out", str(dest), "--sha256", digest, "--skip-runtime",
  ]) == 0
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
  assert "runtime missing" in content
  assert "NO RT" in content
  assert "pylib" in content


def test_runtime_wheel_pin():
  assert ORT_VERSION == "1.31.0"
  assert "cp312" in ORT_WHEEL
  assert ORT_WHEEL.endswith("manylinux_2_28_aarch64.whl")
  assert ORT_SHA256 == "35758d7606d578ec5b9d65f6e8a1f488013194c3f6097038a3223cb26d35ef9a"
  assert ORT_BYTES == 21420804
  assert ORT_URL.endswith(ORT_WHEEL)
  assert "pythonhosted.org" in ORT_URL


def _fake_wheel(path: Path, payload: str = "vendored = True\n") -> str:
  with zipfile.ZipFile(path, "w") as zf:
    zf.writestr("onnxruntime/", b"")
    zf.writestr("onnxruntime/__init__.py", payload)
    zf.writestr("onnxruntime/LICENSE", b"license-text\n")
    zf.writestr("onnxruntime-9.9.9.dist-info/METADATA", "Name: onnxruntime\n")
    zf.writestr("numpy/__init__.py", "nope")
  return sha256_file(str(path))


def test_runtime_wheel_unzips_off_the_venv(tmp_path):
  wheel = tmp_path / "ort.whl"
  digest = _fake_wheel(wheel)
  pylib = tmp_path / "pylib"
  out = install_runtime_wheel(str(wheel), str(pylib), sha256=digest, version="9.9.9")
  assert out == str(pylib)
  assert (pylib / "onnxruntime").is_dir()
  assert (pylib / "onnxruntime" / "LICENSE").is_file()
  assert not (pylib / "onnxruntime" / "LICENSE").is_dir()
  assert (pylib / "onnxruntime" / "__init__.py").read_text(encoding="utf-8").startswith("vendored")
  assert (pylib / "onnxruntime-9.9.9.dist-info" / "METADATA").is_file()
  assert not (pylib / "numpy").exists()
  assert vendored_runtime_ready(str(pylib), expected_sha=digest)
  assert not vendored_runtime_ready(str(pylib))  # stamp is not the pinned 3X sha
  assert "pylib" in (pylib / "onnxruntime.stamp").read_text(encoding="utf-8")


def test_wheel_path_traversal_is_rejected(tmp_path):
  wheel = tmp_path / "bad.whl"
  with zipfile.ZipFile(wheel, "w") as zf:
    zf.writestr("onnxruntime/__init__.py", "__version__ = '0'\n")
    zf.writestr("onnxruntime/../../outside.py", "pwned")
  digest = sha256_file(str(wheel))
  pylib = tmp_path / "pylib"
  with pytest.raises(RuntimeError, match="wheel path rejected"):
    install_runtime_wheel(str(wheel), str(pylib), sha256=digest)
  assert not (tmp_path / "outside.py").exists()
  assert not pylib.exists()


_NAMELIST = Path(__file__).with_name("onnxruntime_1.31.0_aarch64_namelist.txt")
_REAL_WHEEL = Path(
  "/tmp/speedsign/onnxruntime-1.31.0-cp312-cp312-manylinux_2_28_aarch64.whl",
)


def _pinned_wheel_namelist() -> list[str]:
  """Namelist of the pinned aarch64 wheel. The zip on disk wins over the fixture."""
  if _REAL_WHEEL.is_file():
    with zipfile.ZipFile(_REAL_WHEEL) as zf:
      return zf.namelist()
  return [line for line in _NAMELIST.read_text(encoding="utf-8").splitlines() if line]


def test_real_wheel_namelist_license_is_a_file_and_imports(tmp_path):
  """Route failure: extract treated onnxruntime/LICENSE as a directory.

  The pinned wheel's namelist has `onnxruntime/` (a directory entry) and
  `onnxruntime/LICENSE` (a file). Rebuild that listing with a pure-Python
  __init__ so import works on x86, and when the real wheel is present also
  unpack it.
  """
  names = _pinned_wheel_namelist()
  assert names == [line for line in _NAMELIST.read_text(encoding="utf-8").splitlines() if line]
  assert "onnxruntime/" in names
  assert "onnxruntime/LICENSE" in names
  assert "onnxruntime/__init__.py" in names
  assert "onnxruntime/capi/" in names

  sim = tmp_path / "from-namelist.whl"
  with zipfile.ZipFile(sim, "w") as zf:
    for name in names:
      if name.endswith("/"):
        zf.writestr(name, b"")
      elif name == "onnxruntime/__init__.py":
        zf.writestr(name, "__version__ = '1.31.0'\n")
      elif name == "onnxruntime/LICENSE":
        zf.writestr(name, b"Apache-2.0\n")
      else:
        zf.writestr(name, b"x")
  digest = sha256_file(str(sim))
  pylib = tmp_path / "pylib"
  pylib.mkdir()
  # The failed 3X extract left the package name as a file.
  (pylib / "onnxruntime").write_text("not-a-directory", encoding="utf-8")
  install_runtime_wheel(str(sim), str(pylib), sha256=digest, version="1.31.0")
  assert (pylib / "onnxruntime").is_dir()
  license_path = pylib / "onnxruntime" / "LICENSE"
  assert license_path.is_file()
  assert not license_path.is_dir()
  assert license_path.read_bytes() == b"Apache-2.0\n"
  loaded = import_vendored_runtime(str(pylib))
  assert loaded.endswith(os.path.join("onnxruntime", "__init__.py"))
  assert os.path.abspath(str(pylib)) in os.path.abspath(loaded)

  if not _REAL_WHEEL.is_file():
    return
  real_dest = tmp_path / "real"
  install_runtime_wheel(str(_REAL_WHEEL), str(real_dest), sha256=ORT_SHA256)
  real_license = real_dest / "onnxruntime" / "LICENSE"
  assert (real_dest / "onnxruntime").is_dir()
  assert real_license.is_file()
  assert not real_license.is_dir()
  assert (real_dest / "onnxruntime" / "__init__.py").is_file()
  capi = real_dest / "onnxruntime" / "capi" / "onnxruntime_pybind11_state.cpython-312-aarch64-linux-gnu.so"
  assert capi.is_file()


def test_runtime_wheel_rejects_bad_sha(tmp_path):
  wheel = tmp_path / "ort.whl"
  _fake_wheel(wheel)
  with pytest.raises(RuntimeError, match="SHA-256"):
    install_runtime_wheel(str(wheel), str(tmp_path / "pylib"), sha256="0" * 64)


def test_vendored_path_is_aarch64_only(tmp_path):
  wheel = tmp_path / "ort.whl"
  digest = _fake_wheel(wheel)
  pylib = tmp_path / "pylib"
  install_runtime_wheel(str(wheel), str(pylib), sha256=digest)
  # x86 must not put the aarch64 wheel on sys.path, even when system ORT is absent.
  assert vendored_sys_path("x86_64", str(pylib), False) is None
  assert vendored_sys_path("aarch64", str(pylib), True) is None
  # Default expected sha is the pinned wheel, so a fake stamp does not load.
  assert vendored_sys_path("aarch64", str(pylib), False) is None
  (pylib / "onnxruntime.stamp").write_text("onnxruntime 1.31.0\n" + ORT_SHA256 + "\npylib\n", encoding="utf-8")
  assert vendored_sys_path("aarch64", str(pylib), False) == str(pylib)
  assert vendored_sys_path("arm64", str(pylib), False) == str(pylib)
  assert vendored_sys_path("x86_64", str(pylib), False) is None


def test_runtime_missing_label(monkeypatch):
  monkeypatch.setattr(
    "openpilot.selfdrive.speedsignd.install.ensure_vendored_runtime", lambda: False,
  )
  monkeypatch.setattr(
    "openpilot.selfdrive.speedsignd.install.cv2_dnn_importable", lambda: False,
  )
  clear_runtime_status_cache()
  assert runtime_status_summary() == "runtime missing"
  assert SETTINGS_RUNTIME_MISSING == "runtime missing — tap Install weights"
  assert install_status_summary("/no/such/speed_sign.onnx") == "Missing · " + SETTINGS_RUNTIME_MISSING
  warn = clang_fallback_warning("tinygrad-clang")
  assert "falling back to tinygrad-clang because onnxruntime is missing" in warn
  assert SETTINGS_RUNTIME_MISSING in warn
  assert clang_fallback_warning("onnxruntime-cpu") == ""
  assert clang_fallback_warning("numpy-mutcd") == ""
  clear_runtime_status_cache()


def test_cli_also_installs_runtime(tmp_path, monkeypatch):
  called = {}

  def _fake(pylib=None, url=None, sha256=None):
    called["yes"] = True
    called["sha"] = sha256
    return str(tmp_path / "pylib")

  monkeypatch.setattr(
    "scripts.nap.install_speed_sign_weights.fetch_and_install_runtime", _fake,
  )
  monkeypatch.setattr(
    "scripts.nap.install_speed_sign_weights.runtime_status_summary",
    lambda: "runtime onnxruntime 1.31.0",
  )
  src = tmp_path / "m.onnx"
  src.write_bytes(b"abc")
  dest = tmp_path / "out.onnx"
  assert main([str(src), "--out", str(dest)]) == 0
  assert called["yes"] is True
  assert called["sha"] == ORT_SHA256
