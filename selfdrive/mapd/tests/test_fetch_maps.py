import hashlib
import json
import os
import shutil
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.common.constants import CV
from openpilot.selfdrive.mapd.fetch_maps import (
  cleanup_incomplete,
  download_maps,
  fetch_and_install,
  fetch_maps_index,
  install_from_file,
  installed_db_summary,
  installed_revision_summary,
  load_installed_revision,
  main as fetch_maps_main,
  record_installed_maps,
  refresh_maps,
  required_free_bytes,
  staging_dir,
)
from openpilot.selfdrive.mapd.maps_manifest import (
  ASSET_SHA256,
  ASSET_SQLITE_BYTES,
  ASSET_ZST_BYTES,
  FREE_MARGIN_BYTES,
  MIN_FREE_BYTES,
  MIN_FREE_MIB,
  RELEASE_REVISION,
  SQLITE_SHA256,
  bundled_maps_index,
  load_committed_maps_index,
  maps_update_decision,
  parse_maps_index,
  revision_is_newer,
)
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB


def _tiny_sqlite(path: str) -> None:
  con = OsmSpeedLimitDB.create(path)
  OsmSpeedLimitDB.insert_way(
    con, 1, "Main", "primary", 35 * CV.MPH_TO_MS,
    [(37.0, -122.0), (37.001, -122.0)],
  )
  con.execute("INSERT OR REPLACE INTO meta(key, value) VALUES ('way_count', '1')")
  con.commit()
  con.close()


def _sha256(path: str) -> str:
  h = hashlib.sha256()
  with open(path, "rb") as f:
    h.update(f.read())
  return h.hexdigest()


def _make_zst(tmp_path, name="speed_limits.sqlite.zst"):
  zstd = pytest.importorskip("zstandard")
  src = str(tmp_path / "speed_limits.sqlite")
  _tiny_sqlite(src)
  zst = str(tmp_path / name)
  cctx = zstd.ZstdCompressor()
  with open(src, "rb") as inf, open(zst, "wb") as outf:
    cctx.copy_stream(inf, outf)
  return src, zst


def _track_hashed_paths(monkeypatch) -> list[str]:
  import openpilot.selfdrive.mapd.fetch_maps as fm
  hashed: list[str] = []
  orig = fm._sha256_file

  def wrap(path: str) -> str:
    hashed.append(os.path.abspath(path))
    return orig(path)

  monkeypatch.setattr(fm, "_sha256_file", wrap)
  return hashed


def test_us_pack_free_space_threshold():
  assert MIN_FREE_MIB == 854
  assert MIN_FREE_BYTES == 854 * 1024 * 1024
  assert required_free_bytes(us_pack=True) == MIN_FREE_BYTES
  assert ASSET_ZST_BYTES + ASSET_SQLITE_BYTES + FREE_MARGIN_BYTES == MIN_FREE_BYTES


def test_fetch_installs_sqlite(tmp_path):
  src = str(tmp_path / "speed_limits.sqlite")
  _tiny_sqlite(src)

  dest = str(tmp_path / "installed" / "speed_limits.sqlite")
  fetch_and_install(dest=dest, url=Path(src).as_uri(), sha256="")

  db = OsmSpeedLimitDB(dest)
  assert db.open()
  m = db.lookup(37.0004, -122.0, bearing_deg=0.0)
  assert m is not None and m.way_id == 1
  db.close()
  summary = installed_db_summary(dest)
  assert "Installed" in summary
  assert os.path.getsize(dest) > 1024
  stage = staging_dir(dest)
  assert not os.path.isdir(stage) or not os.listdir(stage)


def test_stages_beside_dest_not_tmp(tmp_path, monkeypatch):
  src = str(tmp_path / "speed_limits.sqlite")
  _tiny_sqlite(src)
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  recorded: list[str] = []

  import openpilot.selfdrive.mapd.fetch_maps as fm
  orig = fm.download_url

  def wrap(url, dest_path, progress=True):
    recorded.append(dest_path)
    return orig(url, dest_path, progress)

  monkeypatch.setattr(fm, "download_url", wrap)
  fetch_and_install(dest=dest, url=Path(src).as_uri(), sha256="")

  assert recorded
  packed = recorded[0]
  dest_dir = os.path.dirname(os.path.abspath(dest))
  assert os.path.dirname(packed) == os.path.join(dest_dir, ".download")
  assert os.path.basename(os.path.dirname(packed)) == ".download"
  # Staging lives beside dest, not a system tempfile.mkdtemp() tree.
  assert packed.startswith(dest_dir + os.sep)


def test_insufficient_space_fails_before_download(tmp_path, monkeypatch):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  downloaded = {"n": 0}

  import openpilot.selfdrive.mapd.fetch_maps as fm

  monkeypatch.setattr(fm.shutil, "disk_usage", lambda p: SimpleNamespace(free=100 * 1024 * 1024, used=0, total=0))

  def boom(url, dest_path, progress=True):
    downloaded["n"] += 1
    raise AssertionError("download should not start when space check fails")

  monkeypatch.setattr(fm, "download_url", boom)

  with pytest.raises(RuntimeError, match="Not enough free space"):
    fetch_and_install(
      dest=dest,
      url="https://github.com/jmbrunick/openpilot/releases/download/osm-us-speed-limits-v2/speed_limits_us.sqlite.zst",
      sha256=ASSET_SHA256,
    )
  assert downloaded["n"] == 0
  assert not os.path.isfile(dest)


def test_cleanup_on_failure_removes_partials_and_staging(tmp_path, monkeypatch):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  leftover_partial = dest + ".partial"
  stage = staging_dir(dest)
  os.makedirs(stage, exist_ok=True)
  leftover_zst = os.path.join(stage, "speed_limits_us.sqlite.zst")
  with open(leftover_partial, "wb") as f:
    f.write(b"old-partial")
  with open(leftover_zst, "wb") as f:
    f.write(b"old-zst")
  # Previous good sqlite must survive a failed fetch.
  _tiny_sqlite(dest)

  import openpilot.selfdrive.mapd.fetch_maps as fm

  def fail_download(url, dest_path, progress=True):
    os.makedirs(os.path.dirname(dest_path), exist_ok=True)
    with open(dest_path, "wb") as f:
      f.write(b"incomplete-zst")
    raise RuntimeError("simulated download failure")

  monkeypatch.setattr(fm, "download_url", fail_download)

  with pytest.raises(RuntimeError, match="simulated download failure"):
    fetch_and_install(dest=dest, url=Path(tmp_path / "x.sqlite").as_uri(), sha256="")

  assert os.path.isfile(dest)
  assert not os.path.isfile(leftover_partial)
  assert not os.path.isfile(leftover_zst)
  assert not os.path.isdir(stage) or not os.listdir(stage)


def test_sha256_is_of_downloaded_zst_not_sqlite(tmp_path):
  src, zst = _make_zst(tmp_path)
  zst_hash = _sha256(zst)
  sqlite_hash = _sha256(src)
  assert zst_hash != sqlite_hash

  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  with pytest.raises(RuntimeError, match="SHA-256 mismatch"):
    fetch_and_install(dest=dest, url=Path(zst).as_uri(), sha256=sqlite_hash)
  assert not os.path.isfile(dest)
  assert not os.path.isfile(dest + ".partial")

  fetch_and_install(dest=dest, url=Path(zst).as_uri(), sha256=zst_hash)
  assert os.path.isfile(dest)
  assert _sha256(dest) == sqlite_hash
  db = OsmSpeedLimitDB(dest)
  assert db.open()
  db.close()
  stage = staging_dir(dest)
  assert not os.path.isdir(stage) or not os.listdir(stage)


def test_install_from_file_does_not_hash_sqlite_against_zst(tmp_path, monkeypatch):
  """ASSET_SHA256 is the zst digest. Never hash dest sqlite against it."""
  src, zst = _make_zst(tmp_path)
  zst_hash = _sha256(zst)
  sqlite_hash = _sha256(src)
  assert zst_hash != sqlite_hash
  assert SQLITE_SHA256 == ""

  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  hashed = _track_hashed_paths(monkeypatch)
  install_from_file(zst, dest, sha256=zst_hash)
  assert os.path.isfile(dest)
  assert _sha256(dest) == sqlite_hash
  dest_abs = os.path.abspath(dest)
  assert dest_abs not in hashed
  assert all(p.endswith(".zst") for p in hashed)
  assert not any(p == dest_abs or p.endswith(".partial") for p in hashed)

  # Re-running with the zst hash on an already-installed sqlite must not fail
  # the way the old install_from_file did (hash dest vs ASSET_SHA256).
  hashed.clear()
  sqlite_copy = str(tmp_path / "already_installed.sqlite")
  shutil.copyfile(dest, sqlite_copy)
  dest2 = str(tmp_path / "osm2" / "speed_limits.sqlite")
  install_from_file(sqlite_copy, dest2, sha256=zst_hash)
  assert os.path.isfile(dest2)
  assert hashed == []


def test_fetch_and_install_never_hashes_dest_sqlite(tmp_path, monkeypatch):
  src, zst = _make_zst(tmp_path)
  zst_hash = _sha256(zst)
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  hashed = _track_hashed_paths(monkeypatch)
  fetch_and_install(dest=dest, url=Path(zst).as_uri(), sha256=zst_hash)
  dest_abs = os.path.abspath(dest)
  assert dest_abs not in hashed
  assert all(".zst" in os.path.basename(p) for p in hashed)
  assert _sha256(dest) == _sha256(src)
  assert _sha256(dest) != zst_hash


def test_empty_sha256_skips_verification(tmp_path, monkeypatch):
  src, zst = _make_zst(tmp_path)
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  hashed = _track_hashed_paths(monkeypatch)
  install_from_file(zst, dest, sha256="")
  assert hashed == []
  dest2 = str(tmp_path / "osm2" / "speed_limits.sqlite")
  src2 = str(tmp_path / "plain.sqlite")
  _tiny_sqlite(src2)
  fetch_and_install(dest=dest2, url=Path(src2).as_uri(), sha256="")
  assert hashed == []
  dest3 = str(tmp_path / "osm3" / "speed_limits.sqlite")
  rc = fetch_maps_main(["--url", Path(src2).as_uri(), "--out", dest3, "--sha256", ""])
  assert rc == 0
  assert os.path.isfile(dest3)


def test_invalid_sqlite_does_not_clobber_dest(tmp_path):
  zstd = pytest.importorskip("zstandard")
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_sqlite(dest)
  prev = _sha256(dest)
  zst = str(tmp_path / "junk.sqlite.zst")
  cctx = zstd.ZstdCompressor()
  with open(zst, "wb") as outf:
    outf.write(cctx.compress(b"not a sqlite"))
  with pytest.raises(RuntimeError, match="not a NAP OSM sqlite"):
    fetch_and_install(dest=dest, url=Path(zst).as_uri(), sha256=_sha256(zst))
  assert os.path.isfile(dest)
  assert _sha256(dest) == prev
  assert not os.path.isfile(dest + ".partial")


def test_fetch_maps_does_not_use_tempfile():
  import inspect
  import openpilot.selfdrive.mapd.fetch_maps as fm
  assert "tempfile" not in inspect.getsource(fm)


def test_cleanup_incomplete_helper(tmp_path):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_sqlite(dest)
  with open(dest + ".partial", "wb") as f:
    f.write(b"x")
  stage = staging_dir(dest)
  os.makedirs(stage, exist_ok=True)
  with open(os.path.join(stage, "foo.zst"), "wb") as f:
    f.write(b"y")
  cleanup_incomplete(dest)
  assert os.path.isfile(dest)
  assert not os.path.isfile(dest + ".partial")
  assert not os.path.isdir(stage) or not os.listdir(stage)


def test_parse_maps_index_required_and_aliases():
  idx = parse_maps_index({
    "revision": "1",
    "asset_url": "https://example.test/speed_limits_us.sqlite.zst",
    "asset_name": "speed_limits_us.sqlite.zst",
    "sha256": ASSET_SHA256,
    "bytes": 203815064,
    "notes": "v1",
  })
  assert idx.revision == "1"
  assert idx.asset_name == "speed_limits_us.sqlite.zst"
  assert idx.sha256 == ASSET_SHA256.lower()
  assert idx.bytes == 203815064
  assert idx.notes == "v1"

  aliased = parse_maps_index({
    "version": "2.0.0",
    "url": "https://example.test/v2.zst",
    "sha256_zst": "A" * 64,
    "size": 10,
  })
  assert aliased.revision == "2.0.0"
  assert aliased.asset_url.endswith("v2.zst")
  assert aliased.sha256 == "a" * 64
  assert aliased.bytes == 10

  with pytest.raises(ValueError, match="missing sha256"):
    parse_maps_index({"revision": "1", "asset_url": "https://example.test/x.zst"})
  with pytest.raises(ValueError, match="missing revision"):
    parse_maps_index({"asset_url": "https://example.test/x.zst", "sha256": "a" * 64})
  with pytest.raises(ValueError, match="64 hex"):
    parse_maps_index({"revision": "1", "asset_url": "https://example.test/x.zst", "sha256": "deadbeef"})


def test_committed_maps_index_matches_first_install_constants():
  idx = load_committed_maps_index()
  bundled = bundled_maps_index()
  assert idx.revision == RELEASE_REVISION == bundled.revision
  assert idx.sha256 == ASSET_SHA256 == bundled.sha256
  assert idx.asset_name == bundled.asset_name
  assert idx.asset_url == bundled.asset_url
  assert idx.bytes is not None and idx.bytes > 0


def test_maps_update_decision_up_to_date_vs_newer():
  remote = bundled_maps_index()
  assert maps_update_decision(False, "", "", remote) == "install"
  assert maps_update_decision(True, remote.revision, remote.sha256, remote) == "up_to_date"
  assert maps_update_decision(True, remote.revision, remote.sha256.upper(), remote) == "up_to_date"

  newer = parse_maps_index({
    "revision": "3",
    "asset_url": remote.asset_url,
    "asset_name": remote.asset_name,
    "sha256": "b" * 64,
  })
  assert maps_update_decision(True, "1", remote.sha256, newer) == "update"
  assert maps_update_decision(True, "2", remote.sha256, newer) == "update"
  assert maps_update_decision(True, "1", "b" * 64, newer) == "up_to_date"

  # Same revision but different zst SHA → republish, still download.
  republish = parse_maps_index({
    "revision": remote.revision,
    "asset_url": remote.asset_url,
    "asset_name": remote.asset_name,
    "sha256": "c" * 64,
  })
  assert maps_update_decision(True, remote.revision, remote.sha256, republish) == "update"

  # Legacy sqlite (no recorded revision): unknown pack — download.
  assert maps_update_decision(True, "", "", remote) == "update"
  assert maps_update_decision(True, "", "", newer) == "update"

  assert revision_is_newer("2", "1")
  assert revision_is_newer("1.1.0", "1.0.9")
  assert not revision_is_newer("1", "1")
  assert not revision_is_newer("1", "2")


def test_sha_mismatch_does_not_replace_sqlite(tmp_path):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_sqlite(dest)
  prev = _sha256(dest)
  _src, zst = _make_zst(tmp_path)
  with pytest.raises(RuntimeError, match="SHA-256 mismatch"):
    fetch_and_install(dest=dest, url=Path(zst).as_uri(), sha256="d" * 64)
  assert os.path.isfile(dest)
  assert _sha256(dest) == prev
  assert not os.path.isfile(dest + ".partial")


def test_refresh_up_to_date_skips_download(tmp_path, monkeypatch, capsys):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_sqlite(dest)
  remote = bundled_maps_index()
  record_installed_maps(remote, dest)

  import openpilot.selfdrive.mapd.fetch_maps as fm

  def boom(*_a, **_k):
    raise AssertionError("must not download when up to date")

  monkeypatch.setattr(fm, "fetch_maps_index", lambda url=None: remote)
  monkeypatch.setattr(fm, "fetch_and_install", boom)
  assert refresh_maps(dest=dest) is None
  out = capsys.readouterr().out
  assert "Maps are up to date" in out
  assert f"revision {remote.revision}" in out
  assert os.path.isfile(dest)


def test_refresh_newer_downloads_and_records_revision(tmp_path, monkeypatch):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_sqlite(dest)
  old = bundled_maps_index()
  record_installed_maps(old, dest)

  _src, zst = _make_zst(tmp_path)
  zst_hash = _sha256(zst)
  remote = parse_maps_index({
    "revision": "3",
    "asset_url": Path(zst).as_uri(),
    "asset_name": os.path.basename(zst),
    "sha256": zst_hash,
    "notes": "county refresh",
  })

  import openpilot.selfdrive.mapd.fetch_maps as fm
  monkeypatch.setattr(fm, "fetch_maps_index", lambda url=None: remote)
  assert refresh_maps(dest=dest) == dest
  rev, sha = load_installed_revision(dest)
  assert rev == "3"
  assert sha == zst_hash
  assert installed_revision_summary(dest) == "3"


def test_refresh_sha_mismatch_keeps_sqlite(tmp_path, monkeypatch):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_sqlite(dest)
  prev = _sha256(dest)
  record_installed_maps(bundled_maps_index(), dest)
  prev_rev, prev_sha = load_installed_revision(dest)

  _src, zst = _make_zst(tmp_path)
  remote = parse_maps_index({
    "revision": "3",
    "asset_url": Path(zst).as_uri(),
    "asset_name": os.path.basename(zst),
    "sha256": "e" * 64,
  })

  import openpilot.selfdrive.mapd.fetch_maps as fm
  monkeypatch.setattr(fm, "fetch_maps_index", lambda url=None: remote)
  with pytest.raises(RuntimeError, match="SHA-256 mismatch"):
    refresh_maps(dest=dest)
  assert os.path.isfile(dest)
  assert _sha256(dest) == prev
  rev, sha = load_installed_revision(dest)
  assert rev == prev_rev
  assert sha == prev_sha


def test_refresh_without_sqlite_does_first_install(tmp_path, monkeypatch):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  _src, zst = _make_zst(tmp_path)
  zst_hash = _sha256(zst)
  remote = parse_maps_index({
    "revision": "1",
    "asset_url": Path(zst).as_uri(),
    "asset_name": os.path.basename(zst),
    "sha256": zst_hash,
  })
  import openpilot.selfdrive.mapd.fetch_maps as fm
  monkeypatch.setattr(fm, "fetch_maps_index", lambda url=None: remote)
  assert refresh_maps(dest=dest) == dest
  assert os.path.isfile(dest)
  assert load_installed_revision(dest) == ("1", zst_hash)


def test_refresh_unversioned_sqlite_downloads_current_pack(tmp_path, monkeypatch):
  """v1 sqlite with no recorded revision must not be treated as up to date."""
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_sqlite(dest)
  _src, zst = _make_zst(tmp_path)
  zst_hash = _sha256(zst)
  remote = bundled_maps_index()
  remote = parse_maps_index({
    "revision": remote.revision,
    "asset_url": Path(zst).as_uri(),
    "asset_name": os.path.basename(zst),
    "sha256": zst_hash,
    "notes": remote.notes,
  })
  import openpilot.selfdrive.mapd.fetch_maps as fm
  monkeypatch.setattr(fm, "fetch_maps_index", lambda url=None: remote)
  assert refresh_maps(dest=dest) == dest
  assert load_installed_revision(dest) == (bundled_maps_index().revision, zst_hash)


def test_fetch_maps_index_from_file(tmp_path):
  payload = {
    "revision": "1",
    "asset_url": "https://example.test/speed_limits_us.sqlite.zst",
    "asset_name": "speed_limits_us.sqlite.zst",
    "sha256": ASSET_SHA256,
    "bytes": 1,
  }
  path = tmp_path / "maps-index.json"
  path.write_text(json.dumps(payload), encoding="utf-8")
  idx = fetch_maps_index(path.as_uri())
  assert idx.revision == "1"
  assert idx.sha256 == ASSET_SHA256.lower()


def test_download_maps_uses_shared_install(tmp_path, monkeypatch):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  _src, zst = _make_zst(tmp_path)
  zst_hash = _sha256(zst)
  remote = parse_maps_index({
    "revision": "1",
    "asset_url": Path(zst).as_uri(),
    "asset_name": os.path.basename(zst),
    "sha256": zst_hash,
  })
  import openpilot.selfdrive.mapd.fetch_maps as fm
  monkeypatch.setattr(fm, "fetch_maps_index", lambda url=None: remote)
  download_maps(dest=dest)
  assert os.path.isfile(dest)
  assert load_installed_revision(dest) == ("1", zst_hash)


def test_refresh_cli_up_to_date(tmp_path, monkeypatch, capsys):
  dest = str(tmp_path / "osm" / "speed_limits.sqlite")
  os.makedirs(os.path.dirname(dest), exist_ok=True)
  _tiny_sqlite(dest)
  remote = bundled_maps_index()
  record_installed_maps(remote, dest)
  index_path = tmp_path / "maps-index.json"
  index_path.write_text(
    json.dumps({
      "revision": remote.revision,
      "asset_url": remote.asset_url,
      "asset_name": remote.asset_name,
      "sha256": remote.sha256,
    }),
    encoding="utf-8",
  )
  import openpilot.selfdrive.mapd.fetch_maps as fm

  def boom(*_a, **_k):
    raise AssertionError("cli refresh must not download when up to date")

  monkeypatch.setattr(fm, "fetch_and_install", boom)
  rc = fetch_maps_main(["--refresh", "--out", dest, "--index-url", index_path.as_uri()])
  assert rc == 0
  assert "Maps are up to date" in capsys.readouterr().out

