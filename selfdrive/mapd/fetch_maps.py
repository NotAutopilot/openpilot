"""Download a prebuilt NAP OSM speed-limit sqlite from a GitHub Release.

Stages on the destination filesystem (not /tmp). US pack peak is the zst plus
the sqlite.partial before the zst is deleted: ~204 MiB + ~570 MiB.

Downloads retry (DOWNLOAD_ATTEMPTS, backoff) and resume with HTTP Range from
.download/<asset>.partial, also across taps. Only a SHA-256 mismatch throws
the staged bytes away.
"""
from __future__ import annotations

import argparse
import errno
import hashlib
import json
import os
import shutil
import time
import http.client
import urllib.error
import urllib.request
from urllib.parse import unquote, urlparse

from openpilot.selfdrive.mapd.db_paths import default_db_path
from openpilot.selfdrive.mapd.maps_manifest import (
  ASSET_NAME,
  ASSET_SHA256,
  ASSET_SQLITE_BYTES,
  ASSET_ZST_BYTES,
  ATTRIBUTION,
  FREE_MARGIN_BYTES,
  GITHUB_REPO,
  LICENSE,
  LICENSE_URL,
  MAPS_INDEX_URL,
  MIN_FREE_BYTES,
  MIN_FREE_MIB,
  PARAM_REVISION,
  PARAM_SHA256,
  RELEASE_TAG,
  REVISION_SIDECAR,
  SQLITE_SHA256,
  USER_AGENT,
  MapsIndex,
  bundled_maps_index,
  maps_update_decision,
  parse_maps_index,
  release_asset_url,
)
from openpilot.selfdrive.mapd.osm_db import OsmSpeedLimitDB

CHUNK = 256 * 1024
STAGING_DIRNAME = ".download"
# Wi-Fi drops / slow GitHub CDN: retry with backoff and resume via HTTP Range.
DOWNLOAD_ATTEMPTS = 8
READ_TIMEOUT_S = 60.0
BACKOFF_BASE_S = 2.0
BACKOFF_MAX_S = 30.0
# Records which zst digest a resumable partial belongs to.
EXPECT_SUFFIX = ".expect"


class HashMismatchError(RuntimeError):
  pass


class _Retryable(Exception):
  pass


def _p(msg: str) -> None:
  print(msg, flush=True)


def _sqlite_present(path: str | None) -> bool:
  if not path or not os.path.isfile(path):
    return False
  try:
    return os.path.getsize(path) >= 1024
  except OSError:
    return False


sqlite_present = _sqlite_present


def installed_db_summary(path: str | None = None) -> str:
  path = path or default_db_path()
  if not _sqlite_present(path):
    return "Not installed"
  try:
    sz = os.path.getsize(path)
  except OSError:
    return "Not installed"
  mb = sz / (1024 * 1024)
  ways = OsmSpeedLimitDB.way_count(path)
  if ways is None:
    return f"Installed ({mb:.0f} MB)"
  return f"Installed ({mb:.0f} MB, {ways:,} ways)"


def _sidecar_path(dest: str) -> str:
  return os.path.join(os.path.dirname(os.path.abspath(dest)) or ".", REVISION_SIDECAR)


def _params_get(key: str) -> str:
  try:
    from openpilot.common.params import Params
    raw = Params().get(key)
  except Exception:
    return ""
  if raw is None:
    return ""
  if isinstance(raw, bytes):
    raw = raw.decode("utf-8", errors="replace")
  return str(raw).strip()


def _params_put(key: str, value: str) -> None:
  try:
    from openpilot.common.params import Params
    Params().put(key, value)
  except Exception:
    pass


def load_installed_revision(dest: str | None = None) -> tuple[str, str]:
  """Return (revision, zst sha256) recorded after a successful install.

  Prefers the sidecar next to the sqlite so the check works without a
  params rebuild. Falls back to NAPMapSpeedDbRevision / NAPMapSpeedDbSha256.
  """
  dest = os.path.abspath(dest or default_db_path())
  sidecar = _sidecar_path(dest)
  if os.path.isfile(sidecar):
    try:
      with open(sidecar, encoding="utf-8") as f:
        data = json.load(f)
      if isinstance(data, dict):
        rev = str(data.get("revision") or "").strip()
        sha = str(data.get("sha256") or "").strip().lower()
        return rev, sha
    except (OSError, ValueError, TypeError):
      pass
  return _params_get(PARAM_REVISION), _params_get(PARAM_SHA256).lower()


def record_installed_maps(index: MapsIndex, dest: str | None = None) -> None:
  """Persist revision + zst sha after a successful fetch. Sidecar + params."""
  dest = os.path.abspath(dest or default_db_path())
  payload = {
    "revision": index.revision,
    "sha256": (index.sha256 or "").lower(),
    "asset_name": index.asset_name,
    "asset_url": index.asset_url,
  }
  sidecar = _sidecar_path(dest)
  try:
    os.makedirs(os.path.dirname(sidecar) or ".", exist_ok=True)
    tmp = sidecar + ".partial"
    with open(tmp, "w", encoding="utf-8") as f:
      json.dump(payload, f, indent=2)
      f.write("\n")
    os.replace(tmp, sidecar)
  except OSError:
    pass
  if index.revision:
    _params_put(PARAM_REVISION, index.revision)
  if index.sha256:
    _params_put(PARAM_SHA256, index.sha256.lower())


def installed_revision_summary(path: str | None = None) -> str:
  path = path or default_db_path()
  if not _sqlite_present(path):
    return "Not installed"
  rev, _sha = load_installed_revision(path)
  if rev:
    return rev
  return "Unversioned"


def fetch_maps_index(url: str | None = None) -> MapsIndex:
  """Download and parse the published maps-index JSON (not the sqlite)."""
  url = url or os.environ.get("NAP_MAPS_INDEX_URL") or MAPS_INDEX_URL
  if url.startswith("file:"):
    path = unquote(urlparse(url).path)
    try:
      with open(path, encoding="utf-8") as f:
        raw = f.read()
    except OSError as e:
      raise RuntimeError(f"Could not read maps index {url}: {e}") from e
    return parse_maps_index(raw)

  req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT, "Accept": "application/json"})
  try:
    with urllib.request.urlopen(req, timeout=30) as resp:
      raw = resp.read()
  except urllib.error.HTTPError as e:
    raise RuntimeError(
      f"Maps index failed HTTP {e.code} for {url}. Need Wi-Fi to GitHub."
    ) from e
  except urllib.error.URLError as e:
    raise RuntimeError(f"Maps index download failed: {e}. Need Wi-Fi to GitHub.") from e
  return parse_maps_index(raw)


def resolve_published_index(*, index_url: str | None = None, fallback: bool = True) -> MapsIndex:
  """Live index, or bundled first-install constants if fallback is allowed."""
  try:
    index = fetch_maps_index(index_url)
    _p(f"Maps index: revision {index.revision} ({index.asset_name})")
    return index
  except Exception as e:
    if not fallback:
      raise
    _p(f"Published maps index unavailable ({e}). Using first-install constants.")
    return bundled_maps_index()


def fetch_and_install_from_index(index: MapsIndex, dest: str | None = None) -> str:
  """Shared US-pack fetch used by Download US Maps."""
  dest = fetch_and_install(
    dest=dest,
    url=index.asset_url,
    sha256=index.sha256,
    sqlite_sha256=SQLITE_SHA256,
  )
  record_installed_maps(index, dest)
  return dest


def download_maps(dest: str | None = None, *, index_url: str | None = None) -> str:
  """Install the current published US pack (index if reachable, else bundled)."""
  index = resolve_published_index(index_url=index_url, fallback=True)
  return fetch_and_install_from_index(index, dest)


def refresh_maps(dest: str | None = None, *, index_url: str | None = None) -> str | None:
  """CLI: version-check the published US index; download only if missing or newer.

  Settings → Refresh maps is live Overpass (`local_refresh.refresh_local_maps`),
  not this function. Keep this for `python -m scripts.nap.fetch_osm_maps --refresh`.
  """
  dest = os.path.abspath(dest or default_db_path())
  index = resolve_published_index(index_url=index_url, fallback=False)
  if index.notes:
    _p(index.notes)
  has_sqlite = _sqlite_present(dest)
  installed_rev, installed_sha = load_installed_revision(dest)
  decision = maps_update_decision(has_sqlite, installed_rev, installed_sha, index)
  if decision == "up_to_date":
    if has_sqlite and (not installed_rev or not installed_sha):
      record_installed_maps(index, dest)
    _p(f"Maps are up to date (revision {index.revision})")
    return None
  if decision == "install":
    _p("No maps installed yet. Downloading the current US pack…")
  else:
    from_rev = installed_rev or "unversioned"
    _p(f"Update available: {from_rev} → {index.revision}")
  return fetch_and_install_from_index(index, dest)


def _sha256_file(path: str) -> str:
  h = hashlib.sha256()
  with open(path, "rb") as f:
    while True:
      chunk = f.read(1024 * 1024)
      if not chunk:
        break
      h.update(chunk)
  return h.hexdigest()


def _progress(prefix: str, done: int, total: int) -> None:
  if total > 0:
    pct = min(100.0, 100.0 * done / total)
    _p(f"{prefix} {done / 1e6:.1f}/{total / 1e6:.1f} MB ({pct:.0f}%)")
  else:
    _p(f"{prefix} {done / 1e6:.1f} MB")


def staging_dir(dest: str) -> str:
  return os.path.join(os.path.dirname(os.path.abspath(dest)) or ".", STAGING_DIRNAME)


def cleanup_incomplete(dest: str, *, keep_resume: bool = False) -> None:
  """Remove leftover partials/staging from a failed or interrupted fetch.

  keep_resume keeps the staged download (*.zst.partial / *.zst and its
  .expect tag) so the next Download US Maps tap resumes with HTTP Range.
  """
  victims = [dest + ".partial"]
  stage = staging_dir(dest)
  if os.path.isdir(stage):
    try:
      names = os.listdir(stage)
    except OSError:
      names = []
    for name in names:
      if keep_resume and name.endswith((".partial", EXPECT_SUFFIX, ".zst")):
        continue
      victims.append(os.path.join(stage, name))
  for p in victims:
    try:
      if os.path.isfile(p) or os.path.islink(p):
        os.remove(p)
    except OSError:
      pass
  try:
    if os.path.isdir(stage) and not os.listdir(stage):
      os.rmdir(stage)
  except OSError:
    pass


def _space_error(dest: str, free: int, need: int) -> RuntimeError:
  dest_dir = os.path.dirname(os.path.abspath(dest)) or "."
  osm = dest_dir
  return RuntimeError(
    f"Not enough free space on {dest_dir}. " +
    f"Need {need / (1024 * 1024):.0f} MiB, have {free / (1024 * 1024):.0f} MiB. " +
    f"US pack is ~{ASSET_ZST_BYTES / (1024 * 1024):.0f} MiB zst + " +
    f"~{ASSET_SQLITE_BYTES / (1024 * 1024):.0f} MiB sqlite " +
    f"(peak both on this filesystem; +{FREE_MARGIN_BYTES / (1024 * 1024):.0f} MiB margin, " +
    f"threshold {MIN_FREE_MIB} MiB / {MIN_FREE_BYTES} bytes). " +
    "Clean leftovers then routes/videos if still short:\n" +
    f"  rm -f {osm}/*.partial {osm}/{STAGING_DIRNAME}/*\n" +
    f"  df -h {dest_dir}\n" +
    "Smaller region (PC, then scp): python scripts/nap/download_osm_speed_limits.py " +
    "--lat <lat> --lon <lon> --radius-km 30"
  )


def required_free_bytes(zst_bytes: int | None = None, *, us_pack: bool = True) -> int:
  """Bytes that must be free on the dest filesystem before a fetch.

  US pack: max(zst + sqlite + margin, MIN_FREE_BYTES) = 822 MiB.
  Local/test installs (file://, empty sha256) only need a small headroom.
  """
  if not us_pack:
    extra = int(zst_bytes or 0)
    return max(extra + 8 * 1024 * 1024, 8 * 1024 * 1024)
  zst = ASSET_ZST_BYTES if not zst_bytes or zst_bytes <= 0 else int(zst_bytes)
  need = zst + ASSET_SQLITE_BYTES + FREE_MARGIN_BYTES
  return max(need, MIN_FREE_BYTES)


def assert_free_space(dest: str, zst_bytes: int | None = None, *, us_pack: bool = True,
                      already_staged: int = 0) -> int:
  dest_dir = os.path.dirname(os.path.abspath(dest)) or "."
  os.makedirs(dest_dir, exist_ok=True)
  # A resumable partial already sits on this filesystem; it counts toward the budget.
  free = int(shutil.disk_usage(dest_dir).free) + max(0, int(already_staged))
  need = required_free_bytes(zst_bytes, us_pack=us_pack)
  _p(f"Free on {dest_dir}: {free / (1024 * 1024):.0f} MiB (need {need / (1024 * 1024):.0f} MiB)")
  if free < need:
    raise _space_error(dest, free, need)
  return free


def _raise_if_enospc(exc: OSError, dest: str) -> None:
  if exc.errno == errno.ENOSPC:
    dest_dir = os.path.dirname(os.path.abspath(dest)) or "."
    free = int(shutil.disk_usage(dest_dir).free)
    raise _space_error(dest, free, required_free_bytes()) from exc
  raise exc


def _backoff_s(attempt: int) -> float:
  return min(BACKOFF_MAX_S, BACKOFF_BASE_S * (2 ** max(0, attempt - 1)))


def _content_range_start(value: str | None) -> int | None:
  # "bytes 1000-1999/213762114"
  try:
    return int(str(value).split()[1].split("-")[0])
  except (IndexError, ValueError, AttributeError):
    return None


def _content_range_total(value: str | None) -> int:
  try:
    tot = str(value).rsplit("/", 1)[1]
    return 0 if tot == "*" else int(tot)
  except (IndexError, ValueError, AttributeError):
    return 0


def _download_once(url: str, tmp: str, progress: bool, timeout: float) -> int:
  """One HTTP attempt, appending to tmp from its current size. Returns total bytes in tmp.

  Raises _Retryable on timeouts / connection drops / 5xx / 429.
  """
  have = os.path.getsize(tmp) if os.path.isfile(tmp) else 0
  headers = {"User-Agent": USER_AGENT}
  if have > 0:
    headers["Range"] = f"bytes={have}-"
  req = urllib.request.Request(url, headers=headers)
  try:
    resp_cm = urllib.request.urlopen(req, timeout=timeout)
  except urllib.error.HTTPError as e:
    if e.code == 416 and have > 0:
      # Range starts at/after EOF: the partial is already complete.
      return have
    if e.code >= 500 or e.code in (408, 429):
      raise _Retryable(f"HTTP {e.code}") from e
    raise RuntimeError(
      f"Download failed HTTP {e.code} for {url}. " +
      "Publish the OSM sqlite as a GitHub Release asset (see docs-nap/map-speed.md)."
    ) from e
  except (urllib.error.URLError, TimeoutError, ConnectionError, http.client.HTTPException) as e:
    raise _Retryable(str(getattr(e, "reason", e))) from e

  with resp_cm as resp:
    status = getattr(resp, "status", None) or resp.getcode()
    if have > 0 and status == 206 and _content_range_start(resp.headers.get("Content-Range")) == have:
      mode = "ab"
      total = _content_range_total(resp.headers.get("Content-Range"))
      if progress:
        _p(f"Resuming at {have / 1e6:.1f} MB")
    else:
      # Server ignored Range (200) or answered a different range: start over.
      mode, have = "wb", 0
      total = int(resp.headers.get("Content-Length") or 0)
    n = have
    last = n
    read = getattr(resp, "read1", None) or resp.read
    try:
      with open(tmp, mode) as f:
        while True:
          try:
            # read1: return what has arrived, so a stall keeps every received byte for the resume.
            chunk = read(CHUNK)
          except (TimeoutError, ConnectionError, http.client.HTTPException) as e:
            raise _Retryable(str(e) or type(e).__name__) from e
          except OSError as e:
            if e.errno == errno.ENOSPC:
              raise
            raise _Retryable(str(e)) from e
          if not chunk:
            break
          f.write(chunk)
          n += len(chunk)
          if progress and (n - last >= 5 * 1024 * 1024 or (total and n == total)):
            _progress("Downloading", n, total)
            last = n
    except _Retryable:
      raise
    except OSError as e:
      _raise_if_enospc(e, tmp)
    if total and n < total:
      raise _Retryable(f"connection closed at {n}/{total} bytes")
    return n


def download_url(url: str, dest: str, progress: bool = True, *, attempts: int = DOWNLOAD_ATTEMPTS,
                 timeout: float = READ_TIMEOUT_S, sleep=time.sleep) -> int:
  """Download url → dest with retries and HTTP Range resume from dest + '.partial'.

  The partial is kept on failure so a later call (second tap) resumes. Only a
  hash mismatch (caller) deletes it.
  """
  os.makedirs(os.path.dirname(dest) or ".", exist_ok=True)
  tmp = dest + ".partial"
  last_err = ""
  for attempt in range(1, max(1, attempts) + 1):
    try:
      n = _download_once(url, tmp, progress, timeout)
      os.replace(tmp, dest)
      return n
    except _Retryable as e:
      last_err = str(e) or "connection error"
      if attempt >= attempts:
        break
      wait = _backoff_s(attempt)
      have = os.path.getsize(tmp) if os.path.isfile(tmp) else 0
      _p(f"Retrying… ({attempt}) {last_err}; {have / 1e6:.1f} MB kept, next try in {wait:.0f}s")
      sleep(wait)
  raise RuntimeError(
    f"Download failed after {attempts} attempts: {last_err}. Need Wi-Fi to GitHub. " +
    "Tap Download again to resume."
  )


def _decompress_zst(src: str, dest_partial: str) -> None:
  """Stream-decompress src → dest_partial. Delete src after the stream succeeds
  so peak disk is zst+partial, then partial only. Caller replaces onto dest."""
  os.makedirs(os.path.dirname(dest_partial) or ".", exist_ok=True)
  try:
    try:
      import zstandard as zstd
    except ImportError as e:
      zstd_bin = shutil.which("zstd")
      if not zstd_bin:
        raise RuntimeError("Need the zstandard Python package (or zstd CLI) to decompress maps.") from e
      import subprocess
      try:
        subprocess.check_call([zstd_bin, "-d", "-f", src, "-o", dest_partial])
      except subprocess.CalledProcessError as cpe:
        raise RuntimeError(f"zstd decompress failed (exit {cpe.returncode})") from cpe
    else:
      dctx = zstd.ZstdDecompressor()
      with open(src, "rb") as inf, open(dest_partial, "wb") as outf:
        dctx.copy_stream(inf, outf)
    try:
      os.remove(src)
    except OSError:
      pass
  except OSError as e:
    try:
      if os.path.isfile(dest_partial):
        os.remove(dest_partial)
    except OSError:
      pass
    _raise_if_enospc(e, dest_partial)


def _sqlite_ok(path: str) -> bool:
  db = OsmSpeedLimitDB(path)
  ok = db.open()
  db.close()
  return ok


sqlite_ok = _sqlite_ok


def _hash_requested(value: str | None) -> str:
  """Non-empty stripped digest, or '' to skip. `--sha256 ''` is a valid skip."""
  if value is None:
    return ""
  return str(value).strip()


def _verify_sha256(path: str, sha256: str, *, kind: str) -> None:
  got = _sha256_file(path)
  if got.lower() != sha256.lower():
    raise HashMismatchError(f"SHA-256 mismatch for {path} ({kind}): got {got}, expected {sha256}")


def install_from_file(
  src: str,
  dest: str,
  sha256: str = ASSET_SHA256,
  sqlite_sha256: str = SQLITE_SHA256,
) -> str:
  """Install a local .zst or sqlite onto dest.

  ASSET_SHA256 / sha256 is the **zst** digest and is checked on src **before**
  decompress. Dest sqlite is never compared to that digest. SQLITE_SHA256 is
  optional and empty by default.
  """
  src = os.path.abspath(src)
  dest = os.path.abspath(dest)
  os.makedirs(os.path.dirname(dest) or ".", exist_ok=True)
  zst_hash = _hash_requested(sha256)
  sql_hash = _hash_requested(sqlite_sha256)
  partial = dest + ".partial"
  is_zst = src.lower().endswith(".zst")
  try:
    if is_zst:
      if zst_hash:
        _p("Verifying SHA-256 of downloaded zst (before decompress)…")
        _verify_sha256(src, zst_hash, kind="zst, before decompress")
      _p("Decompressing zstd…")
      _decompress_zst(src, partial)
    else:
      # Raw sqlite: ASSET_SHA256 does not apply.
      if sql_hash:
        _verify_sha256(src, sql_hash, kind="sqlite")
      shutil.copyfile(src, partial)
    if sql_hash and is_zst:
      _verify_sha256(partial, sql_hash, kind="sqlite")
    if not _sqlite_ok(partial):
      raise RuntimeError(f"Downloaded file is not a NAP OSM sqlite: {partial}")
    os.replace(partial, dest)
  except Exception:
    try:
      if os.path.isfile(partial):
        os.remove(partial)
    except OSError:
      pass
    raise
  return dest


def fetch_and_install(
  dest: str | None = None,
  url: str | None = None,
  sha256: str = ASSET_SHA256,
  sqlite_sha256: str = SQLITE_SHA256,
) -> str:
  dest = os.path.abspath(dest or default_db_path())
  url = url or os.environ.get("NAP_MAP_RELEASE_URL") or release_asset_url()
  _p(f"OpenStreetMap speed limits ({LICENSE}). {ATTRIBUTION}")
  _p(LICENSE_URL)
  _p(f"Source: {url}")
  _p(f"Install path: {dest}")

  dest_dir = os.path.dirname(dest) or "."
  os.makedirs(dest_dir, exist_ok=True)
  zst_hash = _hash_requested(sha256)
  stage = staging_dir(dest)
  packed_name = os.path.basename(url.split("?", 1)[0]) or ASSET_NAME
  if not packed_name or packed_name in (".", "/"):
    packed_name = ASSET_NAME
  packed = os.path.join(stage, packed_name)
  # Keep a staged download from an earlier tap only if it is for the same asset.
  expect = f"{url}\n{zst_hash.lower()}\n"
  expect_path = packed + EXPECT_SUFFIX
  try:
    with open(expect_path, encoding="utf-8") as f:
      same = f.read() == expect
  except OSError:
    same = False
  cleanup_incomplete(dest, keep_resume=same and bool(zst_hash))
  os.makedirs(stage, exist_ok=True)
  if os.path.isfile(packed) and not os.path.isfile(packed + ".partial"):
    # A finished zst from a run that died before install: resume (Range → 416) re-uses it.
    os.replace(packed, packed + ".partial")
  staged = os.path.getsize(packed + ".partial") if os.path.isfile(packed + ".partial") else 0
  if staged:
    _p(f"Resuming earlier download ({staged / 1e6:.1f} MB already on device)")
  with open(expect_path, "w", encoding="utf-8") as f:
    f.write(expect)
  # Default Settings path (HTTPS + ASSET_SHA256) needs the US-pack budget.
  # file:// / empty sha256 (tests, Overpass copies) skip the US-pack check.
  us_pack = not url.startswith("file:") and bool(zst_hash)
  assert_free_space(dest, us_pack=us_pack, already_staged=staged)
  prev_tmpdir = os.environ.get("TMPDIR")
  os.environ["TMPDIR"] = stage

  try:
    n = download_url(url, packed)
    _p(f"Downloaded {n / 1e6:.1f} MB → {packed}")
    if us_pack:
      # zst is now on dest fs; still need room for sqlite.partial + margin
      dest_dir = os.path.dirname(os.path.abspath(dest)) or "."
      free = int(shutil.disk_usage(dest_dir).free)
      need_decomp = ASSET_SQLITE_BYTES + FREE_MARGIN_BYTES
      _p(f"Free after zst: {free / (1024 * 1024):.0f} MiB (need {need_decomp / (1024 * 1024):.0f} MiB to decompress)")
      if free < need_decomp:
        raise _space_error(dest, free, need_decomp)
    install_from_file(packed, dest, sha256=zst_hash, sqlite_sha256=sqlite_sha256)
    if not packed.lower().endswith(".zst"):
      try:
        os.remove(packed)
      except OSError:
        pass
  except HashMismatchError:
    # Corrupt or stale bytes: the only case that throws the staged download away.
    cleanup_incomplete(dest)
    raise
  except Exception:
    cleanup_incomplete(dest, keep_resume=bool(zst_hash))
    raise
  finally:
    if prev_tmpdir is None:
      os.environ.pop("TMPDIR", None)
    else:
      os.environ["TMPDIR"] = prev_tmpdir

  cleanup_incomplete(dest)
  if not _sqlite_ok(dest):
    raise RuntimeError(f"Downloaded file is not a NAP OSM sqlite: {dest}")

  _p(installed_db_summary(dest))
  _p("mapd reloads this file within ~15s onroad. No reboot required.")
  return dest


def main(argv: list[str] | None = None) -> int:
  p = argparse.ArgumentParser(description="Fetch prebuilt US OSM speed-limit DB (ODbL) onto the device")
  p.add_argument("--url", default=None, help="Override release asset URL")
  p.add_argument("--tag", default=RELEASE_TAG)
  p.add_argument("--repo", default=GITHUB_REPO)
  p.add_argument("--asset", default=ASSET_NAME)
  p.add_argument("--out", default=None, help="sqlite destination (default: /data/media/0/osm/speed_limits.sqlite)")
  p.add_argument(
    "--sha256", default=ASSET_SHA256,
    help="SHA-256 of the zst asset (not sqlite). Empty string skips verification.",
  )
  p.add_argument(
    "--sqlite-sha256", default=SQLITE_SHA256,
    help="Optional SHA-256 of the decompressed sqlite. Empty string skips (default).",
  )
  p.add_argument("--status", action="store_true", help="Print installed DB summary and exit")
  p.add_argument(
    "--refresh", action="store_true",
    help="Check published maps-index.json; download only if missing or newer",
  )
  p.add_argument("--index-url", default=None, help="Override maps-index.json URL")
  args = p.parse_args(argv)

  if args.status:
    _p(installed_db_summary(args.out))
    _p(f"Revision: {installed_revision_summary(args.out)}")
    return 0

  try:
    if args.refresh:
      refresh_maps(dest=args.out, index_url=args.index_url)
    elif args.url or args.tag != RELEASE_TAG or args.repo != GITHUB_REPO or args.asset != ASSET_NAME:
      fetch_and_install(
        dest=args.out, url=args.url or release_asset_url(args.repo, args.tag, args.asset),
        sha256=args.sha256, sqlite_sha256=args.sqlite_sha256,
      )
    else:
      download_maps(dest=args.out, index_url=args.index_url)
  except Exception as e:
    _p(f"ERROR: {e}")
    return 1
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
