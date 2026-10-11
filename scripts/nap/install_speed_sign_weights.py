#!/usr/bin/env python3
"""Install the compact US MUTCD speed-sign ONNX onto /data.

Does not download maps, does not talk to osm.org, and does not write sqlite.

On the comma 3X (Wi-Fi), fetch the published weights (~12 MB) with checksum.
An existing file at the destination is replaced. Settings shows Out of date
when the sha is not speed-sign-onnx-v2:

  python -m scripts.nap.install_speed_sign_weights

Or copy a local export (from scripts.nap.export_speed_sign_onnx):

  python -m scripts.nap.install_speed_sign_weights /path/to/speed_sign.onnx

Destination (override with --out or NAP_SPEED_SIGN_ONNX):

  /data/media/0/nap/speed_sign.onnx

Do not commit the ONNX to git. Enable the logger separately
(Settings → NAP → Speed Sign Logger; still default Off).
"""
from __future__ import annotations

import argparse
import os

from openpilot.selfdrive.speedsignd.install import fetch_and_install, install_weights, sha256_file
from openpilot.selfdrive.speedsignd.paths import nap_data_dir
from openpilot.selfdrive.speedsignd.weights_manifest import ASSET_SHA256, release_asset_url


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
  args = p.parse_args(argv)
  sha = (args.sha256 or "").strip() or None
  if args.onnx:
    dest = install_weights(args.onnx, args.out, sha256=None)
  else:
    dest = fetch_and_install(dest=args.out, url=args.url or release_asset_url(), sha256=sha)
  digest = sha256_file(dest)
  print(f"installed {os.path.getsize(dest)} bytes -> {dest}")
  print(f"sha256 {digest}")
  print(f"nap data dir: {nap_data_dir()}")
  print("enable: Settings → NAP → Speed Sign Logger, or Params put NAPSpeedSignLog 1")
  print("then go onroad — a clear 55/60 R2-1 should light the SIGN plate")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
