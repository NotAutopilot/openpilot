"""Install the compact speed-sign ONNX onto /data. No osm.org, no sqlite."""
from __future__ import annotations

import hashlib
import os
import shutil
import urllib.error
import urllib.request
from urllib.parse import unquote, urlparse

from openpilot.selfdrive.speedsignd.paths import default_onnx_path
from openpilot.selfdrive.speedsignd.weights_manifest import (
  ASSET_SHA256,
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
