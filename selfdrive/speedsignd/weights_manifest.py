"""Where speedsignd fetches the compact US MUTCD speed-sign ONNX.

Weights stay off git. Clone/flash stays small;
`python -m scripts.nap.install_speed_sign_weights` pulls the ONNX onto
`/data/media/0/nap/speed_sign.onnx` and replaces any older file.

Source model: YOLOv8n, imgsz 320, opset 12. Trained on 640×640 native
crops resized to 320 (scale 0.5), which matches the 628×320 road window
letterboxed at 320/628 ≈ 0.51. Output `[1, 17, 2100]` (xywh + 13 classes).
Do not commit the ONNX. Release: speed-sign-onnx-v2 / speed_sign.onnx.
"""
from __future__ import annotations

# Host + tag for the pre-exported ONNX. Bump when republishing.
GITHUB_REPO = "jmbrunick/openpilot"
RELEASE_TAG = "speed-sign-onnx-v2"
ASSET_NAME = "speed_sign.onnx"
RELEASE_REVISION = "2"

# HuggingFace checkpoint used by scripts.nap.export_speed_sign_onnx.
# The shipped detector is the v2 Release asset, not a fresh export of this pt.
HF_REPO = "cvtechniques/JC-Traffic-Sign-Detection"
HF_PT_PATH = "trainv8/weights/best.pt"
HF_PT_SHA256 = "74089754755d904d724e07ec9d74488427decc11c1c8af323e2b4524664575ae"

# SHA-256 of speed-sign-onnx-v2 / speed_sign.onnx (opset 12, 320²).
ASSET_SHA256 = "1131200b7fe2d400596e09d7680fc3a26a08dcd8952acfdf261170bdb22d3d28"
ASSET_BYTES = 12658953

# CPU onnxruntime for the stock 3X. The image has neither onnxruntime nor
# cv2.dnn (route 00000180: onnxruntime-absent(ModuleNotFoundError),
# cv2-dnn-absent). This is the PyPI CPU wheel, not onnxruntime-gpu.
# ELF AArch64, tag cp312, manylinux_2_28, max GLIBC_2.28, NEEDED only
# libc/libstdc++/libm/libgcc_s/libdl/librt/libpthread/ld-linux-aarch64.
# openpilot requires Python >=3.12.3,<3.13. AGNOS 18.4 is newer than 2.28.
# Same 1.31.0 CPU build measured ~12 ms / ~180 GFLOP/s on one x86 thread.
ORT_VERSION = "1.31.0"
ORT_PYTHON = "cp312"
ORT_PLATFORM = "manylinux_2_28_aarch64"
ORT_WHEEL = "onnxruntime-1.31.0-cp312-cp312-manylinux_2_28_aarch64.whl"
ORT_SHA256 = "35758d7606d578ec5b9d65f6e8a1f488013194c3f6097038a3223cb26d35ef9a"
ORT_BYTES = 21420804
ORT_URL = (
  "https://files.pythonhosted.org/packages/53/1a/"
  + "561b43ca1536d9e81d1785bb8a1a260a9e314ef6d04976ba0411c652bda1/"
  + ORT_WHEEL
)

USER_AGENT = "NotAutopilot-speedsignd/1.0 (https://github.com/jmbrunick/openpilot)"

# Ultralytics YOLOv8n export at imgsz=320. ~2.2 GFLOP, about 4× cheaper than
# the v8s weights this replaced. Do not raise on-device imgsz.
YOLO_IMGSZ = 320
YOLO_INPUT_NAME = "images"
YOLO_OUTPUT_LAYOUT = (1, 17, 2100)  # 4 + 13 classes, 2100 anchors

# Class order from speed-sign-onnx-v2 metadata `names`.
# 0 sl_25 … 9 sl_70, 10 sl_unreadable, 11 neg_yellow, 12 neg_other.
# sl_unreadable may schedule the OCR second look. neg_* and sl_unreadable
# are not mph.
YOLO_CLASS_NAMES: tuple[str, ...] = (
  "sl_25",
  "sl_30",
  "sl_35",
  "sl_40",
  "sl_45",
  "sl_50",
  "sl_55",
  "sl_60",
  "sl_65",
  "sl_70",
  "sl_unreadable",
  "neg_yellow",
  "neg_other",
)

# Empty-road max is ~0. Do not lower: misreads (30/35/60 as 50, 60 as 30)
# are already common at 0.40. A lower floor would light more of them.
YOLO_MIN_CONF = 0.40
YOLO_IOU = 0.45
YOLO_MAX_DET = 3

# JSONL and the HUD both need two reads of the same mph from different
# frames. v8n on one A55 is expected around 1–4 s, so the 25% duty gap is
# about 4–16 s. 12 s covers two reads when infer is up to ~3 s. A slower
# infer can miss the window; that is a miss, not a wrong mph.
DEBOUNCE_HITS = 2
DEBOUNCE_WINDOW_S = 12.0


# Two-stage strip proposer: 1-class "sign" YOLOv8 (width 0.125), input
# [1,3,128,352] = lower-right strip x 964-1928, y 525-875, output [1,5,924].
# Trained on Justin's drives + teacher pseudo-labels + night augmentation.
PROPOSER_RELEASE_TAG = "speed-sign-proposer-v1"
PROPOSER_ASSET_NAME = "sign_proposer.onnx"
PROPOSER_SHA256 = "1ef4b7d4131678399e2a2b2049276637759f7e2e3829d1441e98a1e43be94422"
PROPOSER_BYTES = 3732311


def proposer_asset_url(repo: str = GITHUB_REPO) -> str:
  return f"https://github.com/{repo}/releases/download/{PROPOSER_RELEASE_TAG}/{PROPOSER_ASSET_NAME}"


def release_asset_url(repo: str = GITHUB_REPO, tag: str = RELEASE_TAG, asset: str = ASSET_NAME) -> str:
  return f"https://github.com/{repo}/releases/download/{tag}/{asset}"


def hf_pt_url(repo: str = HF_REPO, path: str = HF_PT_PATH) -> str:
  return f"https://huggingface.co/{repo}/resolve/main/{path}"
