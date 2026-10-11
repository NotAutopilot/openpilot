"""ONNX keeps running while OP is controlling, and while driving manually."""
from __future__ import annotations

from types import SimpleNamespace

from openpilot.selfdrive.speedsignd.detect import SpeedSign
from openpilot.selfdrive.speedsignd.hud import live_sign_view
from openpilot.selfdrive.speedsignd.jsonl import JsonlLogger
from openpilot.selfdrive.speedsignd.speedsignd import (
  SM_HZ,
  SPEEDSIGND_HZ,
  UNKNOWN_GRACE_S,
  InferSlot,
  detect_if_allowed,
  detect_skip_reason,
  drain_vision_latest,
  engaged_from_sm,
  engagement_from_sm,
  engagement_log_fields,
  format_infer_diag,
  format_refine_token,
  op_controlling,
  should_reset_detect_after_wait,
  should_run_onnx_detect,
  should_run_speed_sign_log,
)


class _HitDetect:
  def __init__(self):
    self.calls = 0
    self.nv12 = None

  def detect(self, y, min_conf=None, rgb=None, nv12=None):
    self.calls += 1
    self.nv12 = nv12
    return [SpeedSign(mph=55, conf=0.9, bbox=(0, 0, 8, 8))]


class _FakeParams:
  def __init__(self, enabled=False):
    self.enabled = enabled

  def get_bool(self, key):
    return self.enabled


def test_logger_on_still_starts_process_when_onroad():
  assert should_run_speed_sign_log(True, _FakeParams(True))
  assert not should_run_speed_sign_log(True, _FakeParams(False))


def test_onnx_runs_while_op_controlling():
  assert should_run_onnx_detect(True)
  assert should_run_onnx_detect(False)


def test_unknown_is_not_forever_controlling():
  """Soft fail-safe: cereal silence is not 'OP is driving'."""
  assert op_controlling() is False
  assert op_controlling(active=None, state=None, enabled=None) is False
  assert should_run_onnx_detect(op_controlling())


def test_active_and_controlling_states_are_the_gate():
  assert op_controlling(active=True) is True
  assert op_controlling(active=False) is False
  assert op_controlling(state="disabled") is False
  assert op_controlling(state="preEnabled", enabled=True, active=False) is False
  assert op_controlling(state="enabled", active=True) is True
  assert op_controlling(state="softDisabling") is True
  assert op_controlling(state="overriding") is True
  assert op_controlling(state=0) is False
  assert op_controlling(state=2) is True
  assert op_controlling(enabled=False) is False
  assert op_controlling(enabled=True) is True


def test_detect_if_allowed_runs_while_controlling_and_keeps_jsonl_short(tmp_path):
  det = _HitDetect()
  log = JsonlLogger(str(tmp_path / "out.jsonl"))
  y = __import__("numpy").zeros((32, 32), __import__("numpy").uint8)
  signs, written = detect_if_allowed(
    y, 45.0, -95.0, 0.0, True, det, log, now=1.0, controlling=True,
  )
  assert det.calls == 1
  assert signs and signs[0].mph == 55
  assert written and written[0]["mph"] == 55
  raw = (tmp_path / "out.jsonl").read_bytes()
  assert len(raw) <= 240


def test_detect_if_allowed_skips_jsonl_when_engaged_and_disk_stalled(tmp_path):
  det = _HitDetect()
  log = JsonlLogger(str(tmp_path / "out.jsonl"))
  y = __import__("numpy").zeros((32, 32), __import__("numpy").uint8)
  signs, written = detect_if_allowed(
    y, 45.0, -95.0, 0.0, True, det, log, now=1.0, controlling=True, disk_stalled=True,
  )
  assert det.calls == 1
  assert signs and written == []
  assert not (tmp_path / "out.jsonl").exists()
  # Manual driving still logs through a stall; the hardwared hit was engaged writes.
  signs, written = detect_if_allowed(
    y, 45.0, -95.0, 0.0, True, det, log, now=20.0, controlling=False, disk_stalled=True,
  )
  assert written and (tmp_path / "out.jsonl").exists()


def test_detect_if_allowed_passes_nv12_crop(tmp_path):
  det = _HitDetect()
  log = JsonlLogger(str(tmp_path / "out.jsonl"))
  nv12 = __import__("openpilot.selfdrive.speedsignd.nv12", fromlist=["Nv12DetectCrop"]).Nv12DetectCrop(
    y=__import__("numpy").zeros((32, 32), __import__("numpy").uint8),
    uv=None, frame_w=64, frame_h=32, crop=(32, 0, 32, 32),
  )
  signs, written = detect_if_allowed(
    nv12.y, 45.0, -95.0, 0.0, True, det, log, now=1.0, controlling=False, nv12=nv12,
  )
  assert det.calls == 1
  assert det.nv12 is nv12
  assert signs and written


def test_detect_if_allowed_runs_when_disengaged_moving(tmp_path):
  """Product: log signs while driving manually. Moving / not parked is OK."""
  det = _HitDetect()
  log = JsonlLogger(str(tmp_path / "out.jsonl"))
  y = __import__("numpy").zeros((32, 32), __import__("numpy").uint8)
  signs, written = detect_if_allowed(
    y, 45.0, -95.0, 0.0, True, det, log, now=1.0, controlling=False,
  )
  assert det.calls == 1
  assert signs and signs[0].mph == 55
  assert written and written[0]["mph"] == 55


class _Sm:
  def __init__(self, recv: dict[str, int], services: dict[str, object], *,
               seen: dict[str, bool] | None = None,
               alive: dict[str, bool] | None = None,
               valid: dict[str, bool] | None = None):
    self.recv_frame = recv
    self._services = services
    self.seen = seen if seen is not None else {k: v > 0 for k, v in recv.items()}
    if alive is not None:
      self.alive = alive
    if valid is not None:
      self.valid = valid

  def __getitem__(self, name: str):
    return self._services[name]


def _ss(*, enabled=False, active=False, state="disabled"):
  return SimpleNamespace(enabled=enabled, active=active, state=state)


def test_unknown_cereal_startup_grace_then_allow_detect():
  sm = _Sm({}, {})
  start = 10.0
  during = engagement_from_sm(sm, now=start + 0.5, started_at=start)
  assert during.known is False
  assert during.controlling is False
  assert during.allow_detect is False
  assert during.detect_paused is False
  after = engagement_from_sm(sm, now=start + UNKNOWN_GRACE_S + 0.1, started_at=start)
  assert after.allow_detect is True
  assert after.detect_paused is False
  assert after.unknown_after_grace is True
  # now=None (tests) is post-grace allow, not forever-WAIT.
  relaxed = engagement_from_sm(sm)
  assert relaxed.allow_detect is True
  assert relaxed.detect_paused is False
  assert engaged_from_sm(sm) is False


def test_stale_alive_or_invalid_uses_last_payload():
  """1 Hz poll of 100 Hz selfdriveState makes alive timeout (100 ms) fire.

  Last parsed active/state still wins — do not stuck WAIT.
  """
  sm = _Sm(
    {"selfdriveState": 9},
    {"selfdriveState": _ss(enabled=False, active=False, state="disabled")},
    seen={"selfdriveState": True},
    alive={"selfdriveState": False},
  )
  sample = engagement_from_sm(sm)
  assert sample.known is True
  assert sample.controlling is False
  assert sample.allow_detect is True
  assert sample.detect_paused is False
  assert sample.alive is False

  sm = _Sm(
    {"selfdriveState": 9},
    {"selfdriveState": _ss(enabled=False, active=False, state="disabled")},
    seen={"selfdriveState": True},
    valid={"selfdriveState": False},
  )
  sample = engagement_from_sm(sm)
  assert sample.allow_detect is True
  assert sample.detect_paused is False


def test_first_recv_frame_zero_is_seen():
  """SubMaster writes recv_frame=0 on the first update that lands a packet."""
  sm = _Sm(
    {"selfdriveState": 0},
    {"selfdriveState": _ss(enabled=False, active=False, state="disabled")},
    seen={"selfdriveState": True},
    alive={"selfdriveState": True},
    valid={"selfdriveState": True},
  )
  sample = engagement_from_sm(sm)
  assert sample.known is True
  assert sample.allow_detect is True
  assert sample.detect_paused is False


def test_engagement_from_sm_active_controlling():
  sm = _Sm(
    {"selfdriveState": 3},
    {"selfdriveState": _ss(enabled=True, active=True, state="enabled")},
    seen={"selfdriveState": True},
  )
  sample = engagement_from_sm(sm)
  assert sample.controlling is True
  assert sample.allow_detect is True
  assert sample.detect_paused is False
  assert should_run_onnx_detect(sample.controlling)

  sm = _Sm(
    {"selfdriveState": 3},
    {"selfdriveState": _ss(enabled=False, active=False, state="disabled")},
    seen={"selfdriveState": True},
  )
  sample = engagement_from_sm(sm)
  assert sample.controlling is False
  assert sample.allow_detect is True
  assert sample.detect_paused is False
  assert should_run_onnx_detect(sample.controlling)


def test_enabled_but_not_active_preenabled_allows_detect():
  sm = _Sm(
    {"selfdriveState": 4},
    {"selfdriveState": _ss(enabled=True, active=False, state="preEnabled")},
    seen={"selfdriveState": True},
  )
  sample = engagement_from_sm(sm)
  assert sample.controlling is False
  assert sample.allow_detect is True
  assert sample.detect_paused is False


def test_overriding_and_soft_disabling_still_detect():
  for state in ("overriding", "softDisabling"):
    sm = _Sm(
      {"selfdriveState": 5},
      {"selfdriveState": _ss(enabled=True, active=True, state=state)},
      seen={"selfdriveState": True},
    )
    sample = engagement_from_sm(sm)
    assert sample.controlling is True
    assert sample.detect_paused is False
    assert sample.allow_detect is True


def test_stale_flags_still_detect_when_last_payload_is_active():
  sm = _Sm(
    {"selfdriveState": 9},
    {"selfdriveState": _ss(enabled=True, active=True, state="enabled")},
    seen={"selfdriveState": True},
    alive={"selfdriveState": False},
    valid={"selfdriveState": False},
  )
  sample = engagement_from_sm(sm)
  assert sample.controlling is True
  assert sample.allow_detect is True
  assert sample.detect_paused is False


def test_stock_cruise_or_moving_does_not_block_detect():
  """Manual driving (stock CC on, moving) still runs YOLO when OP is off."""
  cruise_on = SimpleNamespace(cruiseState=SimpleNamespace(enabled=True), vEgo=22.0)
  sm = _Sm(
    {"selfdriveState": 4, "carState": 2},
    {"selfdriveState": _ss(enabled=False, active=False, state="disabled"), "carState": cruise_on},
    seen={"selfdriveState": True, "carState": True},
  )
  sample = engagement_from_sm(sm)
  assert sample.controlling is False
  assert sample.allow_detect is True
  # carState-only (no selfdriveState yet) is unknown — not WAIT, detect after grace.
  sm = _Sm({"carState": 2}, {"carState": cruise_on}, seen={"carState": True})
  sample = engagement_from_sm(sm, now=5.0, started_at=0.0)
  assert sample.known is False
  assert sample.detect_paused is False
  assert sample.allow_detect is True


def test_wait_plate_only_when_controlling():
  paused = live_sign_view(enabled=True, msg_valid=True, mph=0, valid=False, detect_paused=True)
  assert paused.plate_text == "WAIT"
  unknown = live_sign_view(enabled=True, msg_valid=True, mph=0, valid=False, detect_paused=False)
  assert unknown.plate_text == ""
  assert not unknown.show_detect_paused


def test_engagement_log_fields_include_gate_signals():
  sm = _Sm(
    {"selfdriveState": 3},
    {"selfdriveState": _ss(enabled=True, active=True, state="enabled")},
    seen={"selfdriveState": True},
    alive={"selfdriveState": True},
    valid={"selfdriveState": True},
  )
  text = engagement_log_fields(engagement_from_sm(sm))
  assert "enabled=True" in text
  assert "active=True" in text
  assert "state=enabled" in text
  assert "alive=True" in text
  assert "valid=True" in text


def test_sm_polls_faster_than_detect():
  assert SM_HZ >= 10.0
  assert SM_HZ <= 20.0
  assert SPEEDSIGND_HZ == 1.0
  assert SM_HZ > SPEEDSIGND_HZ
  # Root cause of Justin's stuck WAIT: 100 Hz selfdriveState alive window is
  # 10/freq = 100 ms. A 1 Hz sm.update leaves alive stale every tick.
  selfdrive_freq = 100.0
  alive_s = 10.0 / selfdrive_freq
  assert alive_s == 0.1
  assert 1.0 / SPEEDSIGND_HZ > alive_s
  assert 1.0 / SM_HZ < alive_s


def test_infer_slot_abandons_in_flight_on_pause():
  """Engage must not join a 300–1500 ms YOLO; drop the result and keep the loop moving."""
  import time
  slot = InferSlot()
  started = __import__("threading").Event()
  release = __import__("threading").Event()

  def slow():
    started.set()
    release.wait(2.0)
    return [SpeedSign(mph=55, conf=0.9, bbox=(0, 0, 1, 1))], [{"mph": 55}]

  t0 = time.monotonic()
  assert slot.start(slow)
  assert started.wait(1.0)
  assert slot.busy is True
  assert slot.start(slow) is False  # one at a time
  abandoned = slot.pause()
  assert abandoned is True
  assert time.monotonic() - t0 < 0.25  # did not wait for the infer
  release.set()
  time.sleep(0.05)
  assert slot.take() is None
  assert slot.busy is False
  # Still paused: refuse new work until resume.
  assert slot.start(slow) is False
  slot.resume()
  assert slot.start(lambda: ([], [])) is True
  time.sleep(0.05)
  out = slot.take()
  assert out is not None and out.signs == []


def test_infer_slot_leftover_after_resume_is_still_abandoned():
  """WAIT→manual must not adopt the abandoned infer (that applied overrun skip)."""
  import time
  slot = InferSlot()
  started = __import__("threading").Event()
  release = __import__("threading").Event()

  def slow():
    started.set()
    release.wait(2.0)
    return [SpeedSign(mph=55, conf=0.9, bbox=(0, 0, 1, 1))], [{"mph": 55}]

  assert slot.start(slow)
  assert started.wait(1.0)
  assert slot.pause() is True
  slot.resume()
  release.set()
  time.sleep(0.08)
  assert slot.take() is None
  assert slot.busy is False
  assert slot.start(lambda: ([SpeedSign(mph=60, conf=0.8, bbox=(0, 0, 1, 1))], [])) is True
  for _ in range(50):
    out = slot.take()
    if out is not None:
      assert out.signs[0].mph == 60
      return
    time.sleep(0.01)
  raise AssertionError("post-WAIT infer never arrived")


def test_post_wait_manual_detect_resets_next_detect():
  """Coverage gap after #70: WAIT cleared but leftover next_detect starved YOLO."""
  assert should_reset_detect_after_wait(False, True) is True
  assert should_reset_detect_after_wait(True, True) is False
  assert should_reset_detect_after_wait(None, True) is False
  assert should_reset_detect_after_wait(False, False) is False


def test_drain_vision_latest_is_nonblocking():
  class _Client:
    def __init__(self):
      self.calls = []

    def recv(self, timeout_ms=0):
      self.calls.append(timeout_ms)
      return None

  client = _Client()
  drain_vision_latest(client)
  assert client.calls == [0]
  drain_vision_latest(None)


def test_infer_slot_surfaces_exception():
  slot = InferSlot()

  def boom():
    raise RuntimeError("tinygrad oops")

  assert slot.start(boom)
  for _ in range(50):
    out = slot.take()
    if out is not None:
      assert out.signs == []
      assert out.error and "tinygrad oops" in out.error
      return
    __import__("time").sleep(0.01)
  raise AssertionError("exception outcome missing")


def test_infer_slot_passes_diag_tuple():
  slot = InferSlot()
  diag = {"backend": "tinygrad", "peak_conf": 0.12, "peak_name": "stop"}
  assert slot.start(lambda: ([], [], diag))
  for _ in range(50):
    out = slot.take()
    if out is not None:
      assert out.diag == diag
      assert out.error is None
      return
    __import__("time").sleep(0.01)
  raise AssertionError("diag outcome missing")


def test_format_infer_diag_has_on_car_fields():
  text = format_infer_diag(
    {
      "backend": "tinygrad",
      "frame_w": 1928,
      "frame_h": 1208,
      "letterbox": 320,
      "crop": (720, 0, 1208, 1208),
      "weights_path": "/data/media/0/nap/speed_sign.onnx",
      "weights_sha": "6ed5f87f3ad2",
      "out_shape": (1, 25, 2100),
      "peak_conf": 0.91,
      "peak_name": "stop",
      "n_over": 10,
      "sl_peak_conf": 0.12,
      "sl_peak_name": "speedLimit55",
      "n_over_sl": 0,
      "top3": (("stop", 0.91), ("yield", 0.22), ("speedLimit55", 0.12)),
      "posted": ((30, 0.01), (50, 0.02), (60, 0.03)),
      "refine": ((65, 50, 50, 0.71),),
      "luma_mean": 88.0,
      "luma_std": 22.0,
      "chroma": 1,
      "prep_ms": 12.0,
      "sess_ms": 420.0,
    },
    allow_detect=True,
  )
  assert "backend=tinygrad" in text
  assert "frame=1928x1208" in text
  assert "letterbox=320" in text
  assert "sha=6ed5f87f3ad2" in text
  assert "allow=1" in text
  assert "peak=0.91/stop" in text
  assert "n_over=10" in text
  assert "sl_peak=0.12/speedLimit55" in text
  assert "n_over_sl=0" in text
  assert "top=stop:0.91,yield:0.22,speedLimit55:0.12" in text
  assert "cls=30:0.01,50:0.02,60:0.03" in text
  assert "refine=50:0.71 class=65" in text
  assert "refine=-(" not in text
  assert "out=1x25x2100" in text
  assert "luma=88/22" in text
  assert "chroma=1" in text
  assert "prep=12" in text
  assert "sess=420" in text


def test_format_infer_diag_always_shows_sl_peak_even_when_n_over_zero():
  text = format_infer_diag(
    {
      "backend": "tinygrad",
      "peak_conf": 0.12,
      "peak_name": "speedLimit35",
      "n_over": 0,
      "sl_peak_conf": 0.12,
      "sl_peak_name": "speedLimit35",
      "n_over_sl": 0,
      "top3": (("speedLimit35", 0.12), ("speedLimit30", 0.08), ("stop", 0.03)),
      "posted": ((30, 0.08), (50, 0.04), (60, 0.02)),
    },
    allow_detect=True,
  )
  assert "peak=0.12/speedLimit35" in text
  assert "sl_peak=0.12/speedLimit35" in text
  assert "n_over=0" in text
  assert "top=speedLimit35:0.12,speedLimit30:0.08,stop:0.03" in text
  assert "cls=30:0.08,50:0.04,60:0.02" in text


def test_format_infer_diag_noise_peak_has_blank_class():
  """Do not print peak=0.00/speedLimit65 — that is not a 65 read."""
  text = format_infer_diag(
    {
      "backend": "tinygrad",
      "peak_conf": 0.00,
      "peak_name": "",
      "n_over": 0,
      "sl_peak_conf": 0.00,
      "sl_peak_name": "",
      "n_over_sl": 0,
      "top3": (),
      "posted": ((30, 0.00), (50, 0.00), (60, 0.00)),
    },
    allow_detect=True,
  )
  assert "peak=0.00/" in text
  assert "speedLimit65" not in text
  assert "cls=30:0.00,50:0.00,60:0.00" in text


def test_format_infer_diag_parked_50_shows_refine_over_confident_65():
  """Justin: top=65:0.73, cls=50:0.00, hud must log refine=50 (class=65)."""
  text = format_infer_diag(
    {
      "backend": "tinygrad",
      "peak_conf": 0.73,
      "peak_name": "speedLimit65",
      "n_over": 1,
      "sl_peak_conf": 0.73,
      "sl_peak_name": "speedLimit65",
      "n_over_sl": 1,
      "top3": (("speedLimit65", 0.73),),
      "posted": ((30, 0.00), (50, 0.00), (60, 0.00), (65, 0.73)),
      "refine": ((65, 50, 50, 0.71),),
    },
    allow_detect=True,
  )
  assert "peak=0.73/speedLimit65" in text
  assert "cls=30:0.00,50:0.00,60:0.00,65:0.73" in text
  assert "refine=50:0.71 class=65" in text
  assert "refine=-(" not in text
  assert "refine=(65)" not in text


def test_format_refine_token_fail_is_dash_not_paren_65():
  """Justin read refine=(65) from refine=-(65). Fail must be refine=- class=65."""
  assert format_refine_token(65, 65, None, 0.0) == "- class=65"
  text = format_infer_diag({"refine": ((65, 65, None, 0.0),)}, allow_detect=True)
  assert "refine=- class=65" in text
  assert "refine=-(" not in text
  assert "refine=(65)" not in text
  assert "refine=65:" not in text


def test_format_refine_token_agree_65_has_conf():
  assert format_refine_token(65, 65, 65, 0.47) == "65:0.47"
  text = format_infer_diag({"refine": ((65, 65, 65, 0.47),)}, allow_detect=True)
  assert "refine=65:0.47" in text
  assert "refine=-" not in text


def test_format_infer_diag_missing_sl_fields_stays_parseable():
  text = format_infer_diag({"backend": "tinygrad", "peak_conf": 0.05, "peak_name": "stop"}, allow_detect=True)
  assert "peak=0.05/stop" in text
  assert "sl_peak=0.00/" in text
  assert "n_over_sl=0" in text
  assert "top=" not in text


def test_detect_skip_reason_names_infer_never_ran():
  assert detect_skip_reason(connected=False, onnx=True, busy=False, holdoff=False) == "vision-disconnected"
  assert detect_skip_reason(connected=True, onnx=False, busy=False, holdoff=False) == "no-onnx"
  assert detect_skip_reason(connected=True, onnx=True, busy=False, holdoff=False, buf_empty=True) == "road-recv-empty"
  assert detect_skip_reason(connected=True, onnx=True, busy=False, holdoff=False, parse_fail=True) == "nv12-parse"
  assert detect_skip_reason(connected=True, onnx=True, busy=True, holdoff=False) == "infer-busy"
  assert detect_skip_reason(connected=True, onnx=True, busy=False, holdoff=True) == "holdoff"


def test_infer_slot_keeps_result_when_not_paused():
  slot = InferSlot()
  assert slot.start(lambda: ([SpeedSign(mph=60, conf=0.8, bbox=(0, 0, 1, 1))], []))
  for _ in range(50):
    out = slot.take()
    if out is not None:
      assert out.signs[0].mph == 60
      return
    __import__("time").sleep(0.01)
  raise AssertionError("infer result never arrived")


def test_detect_gate_is_not_parked_or_force_offroad():
  from pathlib import Path
  src = Path(__file__).resolve().parents[1] / "speedsignd.py"
  text = src.read_text(encoding="utf-8")
  assert "NAPForceOffroad" not in text
  assert "standstill" not in text
  assert "carState" not in text
  assert "selfdriveState" in text
  assert "detect_paused" in text
  assert "SM_HZ" in text
  assert "UNKNOWN_GRACE_S" in text
  assert "engagement_log_fields" in text
  assert "InferSlot" in text
  assert "abandon" in text
  assert "should_reset_detect_after_wait" in text
  assert "drain_vision_latest" in text
  assert "_gen" in text
  assert "format_infer_diag" in text
  assert "peak=" in text
  assert "luma=" in text
  assert "backend=" in text
  assert "waiting-infer" in text
  assert "detect_skip_reason" in text
  assert "copy_nv12_detect_crop" in text
  assert "rgb_from_nv12" not in text
  assert "SPEEDSIGND_CORES" in text
  assert "sched_yield" in text
  assert "crop_rgb=1" in text
  assert "detect_while_engaged=1" in text
  assert "format_read_timing" in text
  assert "cpu_share=" in text
  assert "procs_running" in text
  assert "modelV2" in text
  assert '["gpsLocationExternal", "gpsLocation", "selfdriveState", "modelV2"]' in text
