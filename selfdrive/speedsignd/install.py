"""Install the compact speed-sign ONNX onto /data. No osm.org, no sqlite.

The same offroad action installs a pinned aarch64 CPU onnxruntime wheel
into /data/media/0/nap/pylib. That directory is added to sys.path only
when the system import fails and the machine is aarch64. The system
venv is not modified.
"""
from __future__ import annotations

import hashlib
import os
import platform
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.request
import zipfile
from urllib.parse import unquote, urlparse

from openpilot.selfdrive.speedsignd.paths import default_onnx_path, default_pylib_dir
from openpilot.selfdrive.speedsignd.weights_manifest import (
  ASSET_SHA256,
  ORT_SHA256,
  ORT_URL,
  ORT_VERSION,
  USER_AGENT,
  release_asset_url,
)

CHUNK = 256 * 1024
# v2 asset is ~12 MB. v1 was ~43 MB. Reject empty / leftover .partial files.
# A large file with the wrong sha is "Out of date", not Installed — the v1
# weights must be replaced, not decoded with the v2 class map.
MIN_ONNX_BYTES = 1_000_000


def weights_present(path: str | None = None) -> bool:
  path = path or default_onnx_path()
  try:
    return os.path.isfile(path) and os.path.getsize(path) >= MIN_ONNX_BYTES
  except OSError:
    return False


def weights_sha_matches(path: str | None = None, expected: str | None = None) -> bool:
  """True only when the file is large enough and matches the published sha."""
  path = path or default_onnx_path()
  expected = ASSET_SHA256 if expected is None else expected
  if not expected or not weights_present(path):
    return False
  try:
    return sha256_file(path).lower() == expected.lower()
  except OSError:
    return False


def weights_status_summary(path: str | None = None) -> str:
  """Settings label: Installed, Out of date, or Missing."""
  path = path or default_onnx_path()
  if not weights_present(path):
    return "Missing"
  if not weights_sha_matches(path):
    return "Out of date"
  try:
    sz = os.path.getsize(path)
  except OSError:
    return "Missing"
  mb = sz / (1024 * 1024)
  return f"Installed ({mb:.0f} MB)"


def sha256_file(path: str) -> str:
  h = hashlib.sha256()
  with open(path, "rb") as f:
    while True:
      chunk = f.read(1024 * 1024)
      if not chunk:
        break
      h.update(chunk)
  return h.hexdigest()


def verify_sha256(path: str, expected: str) -> None:
  got = sha256_file(path)
  if got.lower() != expected.lower():
    raise RuntimeError(f"SHA-256 mismatch for {path}: got {got}, expected {expected}")


def install_weights(src: str, dest: str | None = None, sha256: str | None = None) -> str:
  if not os.path.isfile(src):
    raise FileNotFoundError(f"ONNX not found: {src}")
  dest = dest or default_onnx_path()
  parent = os.path.dirname(dest)
  if parent:
    os.makedirs(parent, exist_ok=True)
  if sha256:
    verify_sha256(src, sha256)
  shutil.copy2(src, dest)
  return dest


def download_url(url: str, dest: str) -> int:
  if url.startswith("file:"):
    path = unquote(urlparse(url).path)
    os.makedirs(os.path.dirname(dest) or ".", exist_ok=True)
    shutil.copy2(path, dest)
    return os.path.getsize(dest)
  req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
  try:
    resp_cm = urllib.request.urlopen(req, timeout=60)
  except urllib.error.HTTPError as e:
    raise RuntimeError(
      f"Download failed HTTP {e.code} for {url}. "
      + "Publish speed_sign.onnx as a GitHub Release asset, or export locally "
      + "(python -m scripts.nap.export_speed_sign_onnx) and pass the file."
    ) from e
  except urllib.error.URLError as e:
    raise RuntimeError(f"Download failed: {e}. Need Wi-Fi, or pass a local .onnx.") from e

  os.makedirs(os.path.dirname(dest) or ".", exist_ok=True)
  tmp = dest + ".partial"
  n = 0
  try:
    with resp_cm as resp, open(tmp, "wb") as f:
      while True:
        chunk = resp.read(CHUNK)
        if not chunk:
          break
        f.write(chunk)
        n += len(chunk)
    os.replace(tmp, dest)
  except Exception:
    try:
      if os.path.isfile(tmp):
        os.remove(tmp)
    except OSError:
      pass
    raise
  return n


def fetch_and_install(
  dest: str | None = None,
  url: str | None = None,
  sha256: str | None = ASSET_SHA256,
) -> str:
  """Download the published ONNX (checksum) onto dest."""
  dest = dest or default_onnx_path()
  url = url or os.environ.get("NAP_SPEED_SIGN_ONNX_URL") or release_asset_url()
  parent = os.path.dirname(os.path.abspath(dest)) or "."
  os.makedirs(parent, exist_ok=True)
  tmp = dest + ".download"
  try:
    download_url(url, tmp)
    if sha256:
      verify_sha256(tmp, sha256)
    os.replace(tmp, dest)
  except Exception:
    try:
      if os.path.isfile(tmp):
        os.remove(tmp)
    except OSError:
      pass
    raise
  return dest


def fetch_and_install_proposer(dest: str | None = None, url: str | None = None,
                               sha256: str | None = None) -> str:
  """Download the two-stage strip proposer ONNX (checksum) next to speed_sign.onnx."""
  from openpilot.selfdrive.speedsignd.paths import default_proposer_path
  from openpilot.selfdrive.speedsignd.weights_manifest import PROPOSER_SHA256, proposer_asset_url
  return fetch_and_install(
    dest=dest or default_proposer_path(),
    url=url or os.environ.get("NAP_SPEED_SIGN_PROPOSER_URL") or proposer_asset_url(),
    sha256=PROPOSER_SHA256 if sha256 is None else (sha256 or None),
  )


RUNTIME_STAMP = "onnxruntime.stamp"
_AARCH64 = ("aarch64", "arm64")
_status_cache: dict[str, float | str] = {"t": 0.0, "s": ""}


def host_is_aarch64(machine: str | None = None) -> bool:
  name = platform.machine() if machine is None else machine
  return name.lower() in _AARCH64


def runtime_stamp_path(pylib: str | None = None) -> str:
  return os.path.join(pylib or default_pylib_dir(), RUNTIME_STAMP)


def clear_runtime_status_cache() -> None:
  _status_cache["t"] = 0.0
  _status_cache["s"] = ""


def vendored_runtime_ready(pylib: str | None = None, expected_sha: str | None = None) -> bool:
  """True when pylib has the onnxruntime package and a matching stamp."""
  pylib = pylib or default_pylib_dir()
  expected = ORT_SHA256 if expected_sha is None else expected_sha
  init = os.path.join(pylib, "onnxruntime", "__init__.py")
  stamp = runtime_stamp_path(pylib)
  if not expected or not os.path.isfile(init) or not os.path.isfile(stamp):
    return False
  try:
    with open(stamp, encoding="utf-8") as f:
      text = f.read()
  except OSError:
    return False
  return expected.lower() in text.lower()


def _discard_path(path: str) -> None:
  if not os.path.lexists(path):
    return
  if os.path.islink(path) or os.path.isfile(path):
    os.remove(path)
    return
  shutil.rmtree(path)


def _wheel_plan(zf: zipfile.ZipFile) -> list[tuple[str, bool, zipfile.ZipInfo]]:
  """onnxruntime package and its dist-info. Dirs stay dirs.

  A directory entry in the wheel is `onnxruntime/` (trailing slash).
  Stripping that slash makes ZipInfo.is_dir() false, and extractall then
  writes a file named `onnxruntime`. The next member,
  `onnxruntime/LICENSE`, fails with ENOTDIR because its parent is a file.
  """
  out: list[tuple[str, bool, zipfile.ZipInfo]] = []
  for info in zf.infolist():
    raw = info.filename.replace("\\", "/")
    is_dir = raw.endswith("/")
    if raw.startswith(("/", "../")) or "/../" in raw:
      raise RuntimeError("wheel path rejected: " + info.filename)
    parts: list[str] = []
    for part in raw.split("/"):
      if part in ("", "."):
        continue
      if part == "..":
        raise RuntimeError("wheel path rejected: " + info.filename)
      parts.append(part)
    if not parts:
      continue
    top = parts[0]
    pkg = top == "onnxruntime"
    dist = top.startswith("onnxruntime-") and top.endswith(".dist-info")
    if not (pkg or dist):
      continue
    out.append(("/".join(parts), is_dir, info))
  files = {rel for rel, is_dir, _info in out if not is_dir}
  dirs = {rel for rel, is_dir, _info in out if is_dir}
  both = files & dirs
  if both:
    raise RuntimeError("wheel entry is both a file and a directory: " + sorted(both)[0])
  for rel in files:
    parent = rel
    while "/" in parent:
      parent = parent.rsplit("/", 1)[0]
      if parent in files:
        raise RuntimeError("wheel file is used as a directory: " + parent)
  return out


def _extract_wheel_plan(zf: zipfile.ZipFile, dest: str, plan: list[tuple[str, bool, zipfile.ZipInfo]]) -> None:
  """Write dirs with makedirs and files with open. Never extractall."""
  ordered = sorted(plan, key=lambda item: (not item[1], item[0].count("/"), item[0]))
  for rel, is_dir, info in ordered:
    target = os.path.join(dest, *rel.split("/"))
    if is_dir:
      if os.path.isfile(target):
        raise RuntimeError("wheel directory collides with a file: " + rel)
      os.makedirs(target, exist_ok=True)
      continue
    parent = os.path.dirname(target)
    if parent:
      if os.path.isfile(parent):
        raise RuntimeError("wheel parent is a file: " + rel)
      os.makedirs(parent, exist_ok=True)
    if os.path.isdir(target):
      raise RuntimeError("wheel file collides with a directory: " + rel)
    with zf.open(info, "r") as src, open(target, "wb") as out:
      shutil.copyfileobj(src, out)


def import_vendored_runtime(pylib: str) -> str:
  """Import onnxruntime from pylib in a fresh interpreter. Returns __file__."""
  code = (
    "import sys\n"
    + "sys.path.insert(0, sys.argv[1])\n"
    + "import onnxruntime\n"
    + "print(onnxruntime.__file__)\n"
  )
  proc = subprocess.run(
    [sys.executable, "-c", code, pylib],
    capture_output=True, text=True, timeout=120, cwd=os.sep, check=False,
  )
  if proc.returncode != 0:
    err = (proc.stderr or proc.stdout or "").strip()
    raise RuntimeError("import onnxruntime failed: " + err)
  lines = [line for line in (proc.stdout or "").splitlines() if line.strip()]
  if not lines:
    raise RuntimeError("import onnxruntime printed no path")
  loaded = lines[-1].strip()
  if os.path.abspath(pylib) not in os.path.abspath(loaded):
    raise RuntimeError("import onnxruntime loaded " + loaded + ", not " + pylib)
  return loaded


def install_runtime_wheel(
  wheel: str,
  pylib: str | None = None,
  sha256: str | None = ORT_SHA256,
  version: str = ORT_VERSION,
) -> str:
  """Unzip a wheel into pylib. Does not call pip and does not touch the venv.

  Extract into a sibling temp dir, delete any partial pylib, then rename.
  On aarch64, import must succeed or the pylib is removed.
  """
  if not os.path.isfile(wheel):
    raise FileNotFoundError("runtime wheel not found: " + wheel)
  if sha256:
    verify_sha256(wheel, sha256)
  pylib = os.path.abspath(pylib or default_pylib_dir())
  parent = os.path.dirname(pylib) or "."
  os.makedirs(parent, exist_ok=True)
  staging = pylib + ".partial"
  _discard_path(staging)
  os.makedirs(staging)
  try:
    with zipfile.ZipFile(wheel) as zf:
      plan = _wheel_plan(zf)
      if not any(rel == "onnxruntime" or rel.startswith("onnxruntime/") for rel, _is_dir, _info in plan):
        raise RuntimeError("wheel has no onnxruntime package: " + wheel)
      _extract_wheel_plan(zf, staging, plan)
    init = os.path.join(staging, "onnxruntime", "__init__.py")
    if not os.path.isfile(init):
      raise RuntimeError("wheel extract missed onnxruntime/__init__.py")
    license_path = os.path.join(staging, "onnxruntime", "LICENSE")
    if os.path.isdir(license_path):
      raise RuntimeError("onnxruntime/LICENSE extracted as a directory")
    stamp = os.path.join(staging, RUNTIME_STAMP)
    body = "onnxruntime " + version + "\n" + (sha256 or "") + "\n" + "pylib\n"
    tmp = stamp + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
      f.write(body)
    os.replace(tmp, stamp)
    _discard_path(pylib)
    os.rename(staging, pylib)
  except Exception:
    _discard_path(staging)
    raise
  if host_is_aarch64():
    try:
      import_vendored_runtime(pylib)
    except Exception:
      _discard_path(pylib)
      raise
  clear_runtime_status_cache()
  return pylib


def fetch_and_install_runtime(
  pylib: str | None = None,
  url: str | None = None,
  sha256: str | None = ORT_SHA256,
) -> str:
  """Download the pinned CPU wheel and unzip it under pylib.

  A matching stamp skips the download. The system venv is not modified.
  """
  pylib = pylib or default_pylib_dir()
  if sha256 and vendored_runtime_ready(pylib, expected_sha=sha256):
    return pylib
  url = url or os.environ.get("NAP_SPEED_SIGN_RUNTIME_URL") or ORT_URL
  parent = os.path.dirname(os.path.abspath(pylib)) or "."
  os.makedirs(parent, exist_ok=True)
  # Beside pylib, not inside it. install_runtime_wheel deletes a partial pylib.
  wheel = os.path.abspath(pylib) + ".whl.download"
  try:
    download_url(url, wheel)
    return install_runtime_wheel(wheel, pylib, sha256=sha256)
  finally:
    try:
      if os.path.isfile(wheel):
        os.remove(wheel)
    except OSError:
      pass


def system_ort_importable() -> bool:
  """True when onnxruntime imports without the vendored pylib on sys.path."""
  try:
    import onnxruntime  # noqa: F401
    return True
  except Exception:
    return False


def cv2_dnn_importable() -> bool:
  try:
    import cv2
  except Exception:
    return False
  dnn = getattr(cv2, "dnn", None)
  return dnn is not None and hasattr(dnn, "readNetFromONNX")


def vendored_sys_path(machine: str, pylib: str, system_ok: bool) -> str | None:
  """Directory to insert, or None. Never shadow a working system import.

  The wheel is aarch64. Putting it first on sys.path on x86 would hide
  a working CPU wheel with an ELF the host cannot load.
  """
  if system_ok:
    return None
  if not host_is_aarch64(machine):
    return None
  if not vendored_runtime_ready(pylib):
    return None
  return pylib


def ensure_vendored_runtime() -> bool:
  """Import onnxruntime, using /data pylib only as the aarch64 fallback."""
  if system_ort_importable():
    return True
  pylib = vendored_sys_path(platform.machine(), default_pylib_dir(), False)
  if pylib is None:
    return False
  if pylib not in sys.path:
    sys.path.insert(0, pylib)
  return system_ort_importable()


def _runtime_status_uncached() -> str:
  if ensure_vendored_runtime():
    try:
      import onnxruntime as ort
      ver = str(getattr(ort, "__version__", "") or ORT_VERSION)
    except Exception:
      ver = ORT_VERSION
    return "runtime onnxruntime " + ver
  if cv2_dnn_importable():
    return "runtime opencv-dnn"
  return "runtime missing"


def runtime_status_summary() -> str:
  """Settings/HUD label. 'runtime missing' until ORT or cv2.dnn can import."""
  now = time.monotonic()
  cached = _status_cache.get("s") or ""
  if cached and now - float(_status_cache.get("t") or 0.0) < 2.0:
    return str(cached)
  text = _runtime_status_uncached()
  _status_cache["t"] = now
  _status_cache["s"] = text
  return text


def runtime_missing() -> bool:
  return runtime_status_summary() == "runtime missing"


# Settings row value when neither onnxruntime nor cv2.dnn can import.
# runtime_status_summary stays the short token so HUD NO RT still matches it.
SETTINGS_RUNTIME_MISSING = "runtime missing — tap Install weights"


def install_status_summary(path: str | None = None) -> str:
  """Weights label plus the runtime. Weights strings stay on their own helper."""
  runtime = runtime_status_summary()
  if runtime == "runtime missing":
    runtime = SETTINGS_RUNTIME_MISSING
  return weights_status_summary(path) + " · " + runtime


def clang_fallback_warning(backend: str) -> str:
  """Non-empty when this process will run tinygrad CLANG because ORT is absent."""
  if backend != "tinygrad-clang":
    return ""
  return (
    "speedsignd falling back to tinygrad-clang because onnxruntime is missing. "
    + "Settings weights status: " + SETTINGS_RUNTIME_MISSING + ". "
    + "CLANG will not run while engaged."
  )
