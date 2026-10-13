"""On-device paths for the speed-sign logger.

Weights and JSONL live under /data (or ~/.comma on PC). Nothing is written to
the OSM sqlite, and weights are not stored in git.
"""
from __future__ import annotations

import os

PARAM_KEY = "NAPSpeedSignLog"
LOG_FILENAME = "speed_signs.jsonl"
ONNX_FILENAME = "speed_sign.onnx"
PROPOSER_FILENAME = "sign_proposer.onnx"
PYLIB_DIRNAME = "pylib"


def nap_data_dir() -> str:
  if os.path.isdir("/data/media/0"):
    return "/data/media/0/nap"
  return os.path.join(os.path.expanduser("~"), ".comma", "media", "0", "nap")


def default_log_path() -> str:
  override = os.environ.get("NAP_SPEED_SIGN_LOG")
  if override:
    return override
  return os.path.join(nap_data_dir(), LOG_FILENAME)


def default_onnx_path() -> str:
  override = os.environ.get("NAP_SPEED_SIGN_ONNX")
  if override:
    return override
  return os.path.join(nap_data_dir(), ONNX_FILENAME)


def default_pylib_dir() -> str:
  """Vendored wheels. Not the system venv and not site-packages."""
  override = os.environ.get("NAP_SPEED_SIGN_PYLIB")
  if override:
    return override
  return os.path.join(nap_data_dir(), PYLIB_DIRNAME)


def default_proposer_path() -> str:
  """Two-stage strip proposer (Release speed-sign-proposer-v1)."""
  override = os.environ.get("NAP_SPEED_SIGN_PROPOSER")
  if override:
    return override
  return os.path.join(nap_data_dir(), PROPOSER_FILENAME)
