"""Tests for MUTCD numeric speed-sign detection (no modelV2 head)."""
from __future__ import annotations

import numpy as np
import pytest

from openpilot.selfdrive.speedsignd.detect import (
  MUTCD_MPH,
  OnnxSpeedSignDetector,
  SpeedSignDetector,
  detect_mutcd_speed_signs,
  paint_mutcd_r2_1,
)
from openpilot.selfdrive.speedsignd.nv12 import y_plane_from_nv12


def _scene(h=240, w=320, seed=0) -> np.ndarray:
  rng = np.random.default_rng(seed)
  y = np.full((h, w), 70, np.uint8)
  y += rng.integers(0, 12, size=(h, w), dtype=np.uint8)
  return y


def test_mutcd_numeric_sign_is_read():
  y = _scene()
  paint_mutcd_r2_1(y, 45, x=200, y=30, w=90, h=112)
  hits = detect_mutcd_speed_signs(y)
  assert hits, "expected a MUTCD 45 mph detection"
  assert hits[0].mph == 45
  assert hits[0].conf >= 0.42


def test_several_mutcd_speeds():
  for mph in (15, 25, 30, 40, 45, 55, 60, 70, 80):
    y = _scene(seed=mph)
    paint_mutcd_r2_1(y, mph, x=190, y=24, w=96, h=120)
    hits = detect_mutcd_speed_signs(y)
    assert hits, f"missed {mph}"
    assert hits[0].mph == mph


def test_empty_road_is_not_a_sign():
  y = _scene(seed=99)
  assert detect_mutcd_speed_signs(y) == []


def test_license_plate_aspect_is_rejected():
  y = _scene()
  # Wide white plate (license-like), not R2-1.
  y[160:190, 80:220] = 230
  y[168:182, 95:110] = 20
  y[168:182, 120:140] = 20
  y[168:182, 150:170] = 20
  assert detect_mutcd_speed_signs(y) == []


def test_try_load_refuses_sha_mismatch(tmp_path, monkeypatch):
  from openpilot.selfdrive.speedsignd.install import MIN_ONNX_BYTES
  path = tmp_path / "speed_sign.onnx"
  path.write_bytes(b"v1" * MIN_ONNX_BYTES)
  opened = []

  def boom(*_a, **_k):
    opened.append(1)
    raise AssertionError("mismatched sha must not open a session")

  monkeypatch.setattr("openpilot.selfdrive.speedsignd.detect._onnx_session", boom)
  assert OnnxSpeedSignDetector.try_load(str(path)) is None
  assert opened == []


def test_detector_falls_back_without_onnx(tmp_path, monkeypatch):
  missing = str(tmp_path / "nope.onnx")
  monkeypatch.setenv("NAP_SPEED_SIGN_ONNX", missing)
  det = SpeedSignDetector(onnx_path=missing)
  assert det.onnx is None
  assert det.weights_missing()
  y = _scene()
  paint_mutcd_r2_1(y, 35, x=200, y=30, w=90, h=112)
  hits = det.detect(y)
  assert hits and hits[0].mph == 35


def test_try_reload_picks_up_onnx_after_install(tmp_path, monkeypatch):
  path = str(tmp_path / "later.onnx")
  det = SpeedSignDetector(onnx_path=path)
  assert det.weights_missing()
  assert not det.try_reload()

  class Sess:
    def get_inputs(self):
      return [type("I", (), {"name": "image", "shape": [1, 1, 32, 32]})()]

    def run(self, _out, _feed):
      return [np.array([[0.0, 0.0, 10.0, 10.0, 45.0, 0.9]], dtype=np.float32)]

  def _fake_load(cls, p=None):
    return OnnxSpeedSignDetector(path, Sess())

  monkeypatch.setattr(OnnxSpeedSignDetector, "try_load", classmethod(_fake_load))
  assert det.try_reload()
  assert not det.weights_missing()
  assert not det.try_reload()


def test_onnx_parse_and_mutcd_filter():
  class Sess:
    def get_inputs(self):
      return [type("I", (), {"name": "image", "shape": [1, 1, 64, 64]})()]

    def run(self, _out, _feed):
      return [np.array([[10.0, 10.0, 40.0, 50.0, 45.0, 0.9]], dtype=np.float32)]

  det = OnnxSpeedSignDetector("/tmp/fake.onnx", Sess())
  y = np.zeros((64, 64), np.uint8)
  hits = det.detect(y)
  assert len(hits) == 1
  assert hits[0].mph == 45
  assert hits[0].conf == pytest.approx(0.9, abs=1e-5)


def test_onnx_rejects_non_mutcd_mph():
  class Sess:
    def get_inputs(self):
      return [type("I", (), {"name": "image", "shape": [1, 1, 32, 32]})()]

    def run(self, _out, _feed):
      return [np.array([[0.0, 0.0, 10.0, 10.0, 47.0, 0.99]], dtype=np.float32)]

  hits = OnnxSpeedSignDetector("x", Sess()).detect(np.zeros((32, 32), np.uint8))
  assert hits == []


def test_y_plane_from_nv12():
  class Buf:
    width = 4
    height = 2
    stride = 4
    data = bytes([1, 2, 3, 4, 5, 6, 7, 8]) + bytes(8)

  y = y_plane_from_nv12(Buf())
  assert y is not None
  assert y.shape == (2, 4)
  assert int(y[0, 0]) == 1


def test_mutcd_set_is_us_r2_1():
  assert 15 in MUTCD_MPH and 25 in MUTCD_MPH and 70 in MUTCD_MPH
  assert 47 not in MUTCD_MPH


def test_cpu_backend_assertion_rejects_gpu_and_accepts_cpu():
  import os

  from openpilot.selfdrive.speedsignd.detect import (
    _TinyOrtSession,
    assert_cpu_backend,
    compile_allowed,
    describe_infer_runtimes,
    dnn_ids_are_cpu,
    pin_cpu_only_env,
  )

  pin_cpu_only_env()
  assert os.environ["DEV"] == "CPU"
  assert os.environ["CPU_COUNT"] == "1"
  assert os.environ["CLANG"] == "1"
  assert os.environ["BEAM"] == "0"
  assert os.environ["NOOPT"] == "0"
  assert os.environ["OMP_NUM_THREADS"] == "1"
  for key in ("GPU", "QCOM", "CUDA", "METAL", "AMD", "NV", "WEBGPU"):
    assert os.environ[key] == "0"

  class CpuOrt:
    def get_providers(self):
      return ["CPUExecutionProvider"]

  assert assert_cpu_backend(CpuOrt()) == "onnxruntime-cpu"

  class GpuOrt:
    def get_providers(self):
      return ["QNNExecutionProvider", "CPUExecutionProvider"]

  with pytest.raises(AssertionError):
    assert_cpu_backend(GpuOrt())

  class Tiny:
    _nap_backend = "tinygrad-cpu"
    _nap_device = "CPU"

  assert assert_cpu_backend(Tiny()) == "tinygrad-cpu"

  class Qcom:
    _nap_backend = "tinygrad"
    _nap_device = "QCOM"

  with pytest.raises(AssertionError):
    assert_cpu_backend(Qcom())

  assert not compile_allowed(known=True, controlling=True, ready=False)
  assert compile_allowed(known=True, controlling=False, ready=False)
  assert not compile_allowed(known=False, controlling=False, ready=False)
  assert not compile_allowed(known=False, controlling=True, ready=False)
  assert compile_allowed(known=True, controlling=True, ready=True)

  class Runner:
    def __call__(self, tensors):
      class Out:
        device = "CPU"

        def numpy(self):
          return np.zeros((1, 25, 4), np.float32)

      return {"out": Out()}

  sess = _TinyOrtSession(Runner(), lambda v: v, device="CPU", renderer="ClangJITRenderer")
  assert sess._nap_backend == "tinygrad-clang"
  with pytest.raises(RuntimeError, match="compile-blocked"):
    sess.run(None, {"images": np.zeros((1, 3, 2, 2), np.float32)})
  sess.allow_compile(True)
  out = sess.run(None, {"images": np.zeros((1, 3, 2, 2), np.float32)})
  assert sess.compiled
  assert out[0].shape == (1, 25, 4)
  sess.allow_compile(False)
  sess.run(None, {"images": np.zeros((1, 3, 2, 2), np.float32)})

  class GpuRunner:
    def __call__(self, tensors):
      class Out:
        device = "QCOM"

        def numpy(self):
          return np.zeros((1,), np.float32)

      return Out()

  gpu = _TinyOrtSession(GpuRunner(), lambda v: v, device="CPU", renderer="ClangJITRenderer")
  gpu.allow_compile(True)
  with pytest.raises(AssertionError, match="QCOM"):
    gpu.run(None, {"images": np.zeros((1, 3, 2, 2), np.float32)})
  assert not gpu.compiled

  text = describe_infer_runtimes()
  assert "onnxruntime-" in text
  assert "tinygrad-" in text
  assert "cv2-dnn-" in text

  assert dnn_ids_are_cpu(3, 0, gpu_backends={5, 4}, gpu_targets={1, 6}, cpu_target=0)
  assert not dnn_ids_are_cpu(5, 0, gpu_backends={5, 4}, gpu_targets={1, 6}, cpu_target=0)
  assert not dnn_ids_are_cpu(3, 1, gpu_backends={5, 4}, gpu_targets={1, 6}, cpu_target=0)
  assert not dnn_ids_are_cpu(3, 6, gpu_backends={5, 4}, gpu_targets={1, 6}, cpu_target=0)

  class Cv:
    _nap_backend = "opencv-dnn-cpu"
    _nap_device = "CPU"

  assert assert_cpu_backend(Cv()) == "opencv-dnn-cpu"


def test_compile_stamp_and_lazy_session_block_while_engaged(tmp_path, monkeypatch):
  """No session build until known-disengaged, unless this sha already compiled."""
  from openpilot.selfdrive.speedsignd.detect import (
    _LazyCpuSession,
    compile_cache_warm,
    write_compile_stamp,
  )
  monkeypatch.setenv("NAP_SPEED_SIGN_CACHE", str(tmp_path))
  assert not compile_cache_warm("deadbeef")
  write_compile_stamp("deadbeef", "opencv-dnn-cpu")
  assert compile_cache_warm("deadbeef")
  assert not compile_cache_warm("other")
  write_compile_stamp("clangsha", "tinygrad-clang")
  assert not compile_cache_warm("clangsha")
  db = tmp_path / "clangsha" / "tinygrad" / "cache.db"
  db.parent.mkdir(parents=True)
  db.write_bytes(b"kernels")
  assert compile_cache_warm("clangsha")

  cold = _LazyCpuSession("/no/such.onnx", "not-a-real-sha", "opencv-dnn-cpu")
  assert not cold.compiled
  with pytest.raises(RuntimeError, match="compile-blocked"):
    cold.run(None, {"images": np.zeros((1, 3, 2, 2), np.float32)})
  warm = _LazyCpuSession("/no/such.onnx", "deadbeef", "opencv-dnn-cpu")
  assert warm.compiled


def test_read_mph_washes_out_above_fixed_90():
  """ROAD plates often have ink ≥ 90. Old threshold returned None → HUD kept 65."""
  from openpilot.selfdrive.speedsignd.detect import _read_mph
  y = _scene(seed=7)
  paint_mutcd_r2_1(y, 50, x=200, y=30, w=90, h=112)
  crop = y[30:142, 200:290].copy()
  crop[crop < 90] = 100
  mph, conf = _read_mph(crop)
  assert mph == 50
  assert conf >= 0.25


def test_clang_fallback_warns_when_onnxruntime_is_missing(tmp_path, monkeypatch):
  """ONNX present, onnxruntime absent: one warning names tinygrad-clang and the settings tap."""
  from openpilot.common.swaglog import cloudlog
  from openpilot.selfdrive.speedsignd.detect import _onnx_session
  onnx = tmp_path / "speed_sign.onnx"
  onnx.write_bytes(b"not-a-real-model")
  warnings: list[str] = []
  monkeypatch.setattr(
    "openpilot.selfdrive.speedsignd.detect._select_cpu_kind", lambda: "tinygrad-clang",
  )
  monkeypatch.setattr(cloudlog, "warning", lambda text: warnings.append(text))
  _sess, backend = _onnx_session(str(onnx))
  assert backend == "tinygrad-clang"
  assert len(warnings) == 1
  assert "falling back to tinygrad-clang because onnxruntime is missing" in warnings[0]
  assert "runtime missing — tap Install weights" in warnings[0]
