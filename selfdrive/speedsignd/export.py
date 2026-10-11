"""Export the US MUTCD YOLOv8 checkpoint to a compact ONNX (PC, once).

Downloads the MIT YOLOv8s weights from HuggingFace and writes speed_sign.onnx.
Needs ultralytics + torch on the machine that runs this — not on the 3X.

  python -m scripts.nap.export_speed_sign_onnx --out /tmp/speed_sign.onnx
  python -m scripts.nap.install_speed_sign_weights /tmp/speed_sign.onnx
"""
from __future__ import annotations

import os
import shutil
import tempfile

from openpilot.selfdrive.speedsignd.install import download_url, sha256_file, verify_sha256
from openpilot.selfdrive.speedsignd.weights_manifest import (
  ASSET_SHA256,
  HF_PT_SHA256,
  YOLO_IMGSZ,
  hf_pt_url,
)


def export_onnx(
  out: str,
  pt_url: str | None = None,
  pt_sha256: str = HF_PT_SHA256,
  imgsz: int = YOLO_IMGSZ,
) -> str:
  try:
    from ultralytics import YOLO
  except ImportError as e:
    raise RuntimeError(
      "export_speed_sign_onnx needs ultralytics (and torch) on this machine:\n"
      + "  pip install ultralytics onnx\n"
      + "The comma 3X does not run this — export on a PC, then install the ONNX."
    ) from e

  parent = os.path.dirname(os.path.abspath(out)) or "."
  os.makedirs(parent, exist_ok=True)
  work = tempfile.mkdtemp(prefix="nap-speed-sign-export-")
  try:
    pt = os.path.join(work, "best.pt")
    download_url(pt_url or hf_pt_url(), pt)
    if pt_sha256:
      verify_sha256(pt, pt_sha256)
    model = YOLO(pt)
    exported = model.export(format="onnx", imgsz=imgsz, simplify=True, dynamic=False, opset=12, nms=False)
    exported = str(exported)
    shutil.copy2(exported, out)
  finally:
    shutil.rmtree(work, ignore_errors=True)
  return out


def export_and_report(out: str) -> tuple[str, str, bool]:
  path = export_onnx(out)
  digest = sha256_file(path)
  matches = digest.lower() == ASSET_SHA256.lower()
  return path, digest, matches
