#!/usr/bin/env python3
"""Install the compact US MUTCD speed-sign ONNX and the CPU runtime onto /data.

Does not download maps, does not talk to osm.org, and does not write sqlite.
Does not pip-install and does not modify the system venv.

On the comma 3X (Wi-Fi), fetch the published weights (~12 MB) and the pinned
aarch64 CPU onnxruntime wheel (~21 MB). Both are SHA-256 checked. The wheel
is unzipped into /data/media/0/nap/pylib and loaded via sys.path. Settings
shows "runtime missing" until that import works.

  python -m scripts.nap.install_speed_sign_weights

Or copy a local export (from scripts.nap.export_speed_sign_onnx):

  python -m scripts.nap.install_speed_sign_weights /path/to/speed_sign.onnx

Destination (override with --out or NAP_SPEED_SIGN_ONNX):

  /data/media/0/nap/speed_sign.onnx
  /data/media/0/nap/pylib/onnxruntime

Do not commit the ONNX or the wheel to git. Enable the logger separately
(Settings → NAP → Speed Sign Logger; still default Off).
"""
from __future__ import annotations

import argparse
import os
import sys

from openpilot.selfdrive.speedsignd.install import (
  fetch_and_install,
  fetch_and_install_proposer,
  fetch_and_install_runtime,
  install_weights,
  runtime_status_summary,
  sha256_file,
)
from openpilot.selfdrive.speedsignd.paths import nap_data_dir
from openpilot.selfdrive.speedsignd.weights_manifest import ASSET_SHA256, ORT_SHA256


def main(argv: list[str] | None = None) -> int:
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument(
    "onnx",
    nargs="?",
    default=None,
    help="local speed_sign.onnx (omit to download the published asset)",
  )
  p.add_argument("--out", default=None, help="destination (default /data/media/0/nap/speed_sign.onnx)")
  p.add_argument("--url", default=None, help="override download URL")
  p.add_argument(
    "--sha256",
    default=ASSET_SHA256,
    help="expected SHA-256 of the ONNX (empty string skips)",
  )
  p.add_argument(
    "--skip-runtime",
    action="store_true",
    help="install the ONNX only (tests). The settings button does not pass this.",
  )
  args = p.parse_args(argv)
  sha = (args.sha256 or "").strip() or None
  if args.onnx:
    dest = install_weights(args.onnx, args.out, sha256=None)
  else:
    dest = fetch_and_install(dest=args.out, url=args.url, sha256=sha)
  digest = sha256_file(dest)
  print(f"installed {os.path.getsize(dest)} bytes -> {dest}")
  print(f"sha256 {digest}")
  print(f"nap data dir: {nap_data_dir()}")
  if not args.onnx:
    # Two-stage strip proposer. Not fatal: speedsignd falls back to fixed tiles.
    try:
      prop = fetch_and_install_proposer()
      print(f"proposer -> {prop}")
    except Exception as e:
      print("proposer install failed (fixed tiles stay in use): " + str(e), file=sys.stderr)
  if not args.skip_runtime:
    try:
      pylib = fetch_and_install_runtime(sha256=ORT_SHA256)
    except Exception as e:
      print("runtime install failed: " + str(e), file=sys.stderr)
      print("status: runtime missing", file=sys.stderr)
      return 1
    print(f"runtime -> {pylib}")
    print(runtime_status_summary())
  print("enable: Settings → NAP → Speed Sign Logger, or Params put NAPSpeedSignLog 1")
  print("leave Logger On while disengaged until the first session is cached, then engage")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
