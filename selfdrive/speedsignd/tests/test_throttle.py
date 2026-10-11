"""speedsignd yields to modeld: 1 Hz cap, 25% duty cycle, one little core."""
from __future__ import annotations

import os
from types import SimpleNamespace

from openpilot.selfdrive.speedsignd.detect import (
  CAP_MS_DEFAULT,
  CAP_MS_ENV,
  THREADS_DEFAULT,
  THREADS_ENV,
  limit_infer_threads,
  parse_infer_cap_ms,
  parse_infer_threads,
)
from openpilot.selfdrive.speedsignd.speedsignd import (
  HZ_ENV,
  HZ_MAX,
  HZ_MIN,
  INFER_BUDGET_MS,
  SM_HZ,
  CPU_BUDGET,
  IDLE_FACTOR,
  SPEEDSIGND_CORE,
  SPEEDSIGND_CORES,
  SPEEDSIGND_HZ,
  SPEEDSIGND_NICE,
  CpuShareMeter,
  HostPressure,
  ModelWatch,
  backoff_extra_s,
  cpu_over_budget,
  format_backoff_timing,
  format_read_timing,
  infer_overran,
  join_reasons,
  next_detect_mono,
  parse_detect_hz,
  parse_proc_stat_jiffies,
  parse_proc_thread_count,
  pressure_reason,
  read_gap_s,
  reset_ratekeeper_if_behind,
  steady_core_duty,
  yield_to_modeld,
)


def test_default_detect_hz_is_1():
  assert SPEEDSIGND_HZ == 1.0
  assert 10.0 <= SM_HZ <= 20.0
  assert SM_HZ > SPEEDSIGND_HZ
  assert INFER_BUDGET_MS == 100.0
  assert SPEEDSIGND_NICE == 19
  assert SPEEDSIGND_CORE == 2
  assert SPEEDSIGND_CORES == (2,)
  assert abs(CPU_BUDGET - 0.25) < 1e-9
  assert IDLE_FACTOR == 3.0
  assert THREADS_DEFAULT == 1
  assert CAP_MS_DEFAULT == 800.0
  assert parse_detect_hz(None) == 1.0
  assert parse_detect_hz("") == 1.0
  assert parse_detect_hz("nope") == 1.0
  assert parse_detect_hz("nan") == 1.0


def test_detect_hz_env_clamped():
  assert parse_detect_hz("0.5") == 0.5
  assert parse_detect_hz("2") == 2.0
  assert parse_detect_hz("0.01") == HZ_MIN
  assert parse_detect_hz("99") == HZ_MAX
  assert HZ_ENV == "NAP_SPEED_SIGN_HZ"


def _start_gap(end: float, infer_s: float, period: float, budget: float, cap: float = 0.0, backoff: float = 0.0) -> float:
  nxt = next_detect_mono(end, infer_s, period, budget, cap, backoff_s=backoff)
  return nxt - (end - infer_s)


def test_cheap_infer_waits_out_the_period():
  period, budget = 1.0, 0.100
  infer_s = 0.050
  assert not infer_overran(infer_s, period, budget)
  # Period is longer than the infer, so starts are one period apart.
  assert abs(_start_gap(10.0, infer_s, period, budget) - period) < 1e-9


def test_over_budget_still_obeys_the_duty_cycle():
  period, budget = 1.0, 0.100
  infer_s = 0.350  # over the 100 ms budget, still under the 1 Hz period
  assert infer_overran(infer_s, period, budget)
  # 4× infer is longer than the period, so the duty cycle — not the budget flag — sets the gap.
  assert abs(_start_gap(10.0, infer_s, period, budget, cap=0.800) - infer_s * 4.0) < 1e-9


def test_long_infer_idles_three_times():
  period, budget = 1.0, 0.100
  infer_s = 1.5
  assert infer_overran(infer_s, period, budget)
  assert abs(_start_gap(5.0, infer_s, period, budget) - infer_s * (1.0 + IDLE_FACTOR)) < 1e-9


def test_reset_ratekeeper_drops_backlog():
  rk = SimpleNamespace(remaining=-0.4, _next_frame_time=3.0, _interval=1.0)
  assert reset_ratekeeper_if_behind(rk, now=10.0) is True
  assert rk._next_frame_time == 11.0
  rk.remaining = 0.2
  rk._next_frame_time = 12.0
  assert reset_ratekeeper_if_behind(rk, now=11.8) is False
  assert rk._next_frame_time == 12.0


def test_4hz_would_saturate_typical_onnx_budget():
  """Hypothesis: 4 Hz * ~250–400 ms tinygrad infer ≥ 100% of a core. Default stays 1 Hz."""
  infer_s = 0.30
  assert 4.0 * infer_s >= 1.0
  assert SPEEDSIGND_HZ == 1.0
  assert SPEEDSIGND_HZ * infer_s <= 0.40
  duty = steady_core_duty(infer_s, 1.0 / SPEEDSIGND_HZ)
  assert duty <= CPU_BUDGET + 1e-9


def test_yield_to_modeld_prefers_sched_idle_on_core_2(monkeypatch):
  called: dict[str, object] = {}
  fake_rt = SimpleNamespace(
    drop_realtime=lambda: called.setdefault("drop", True),
    set_core_affinity=lambda cores: called.setdefault("cores", tuple(cores)),
  )
  monkeypatch.setitem(__import__("sys").modules, "openpilot.common.realtime", fake_rt)
  monkeypatch.setattr(os, "SCHED_IDLE", 5, raising=False)

  def _sched(pid, policy, param):
    called["sched"] = (pid, policy, param.sched_priority)
    return 0

  def _nice(n):
    called["nice"] = n
    return n

  monkeypatch.setattr(os, "sched_setscheduler", _sched)
  monkeypatch.setattr(os, "nice", _nice)
  assert yield_to_modeld() == "idle"
  assert called.get("drop") is True
  assert called.get("sched")[1] == 5
  assert "nice" not in called
  assert called.get("cores") == (2,)


def test_yield_to_modeld_falls_back_to_nice_19(monkeypatch):
  called: dict[str, object] = {}
  fake_rt = SimpleNamespace(
    drop_realtime=lambda: called.setdefault("drop", True),
    set_core_affinity=lambda cores: called.setdefault("cores", tuple(cores)),
  )
  monkeypatch.setitem(__import__("sys").modules, "openpilot.common.realtime", fake_rt)

  def _sched(*_a, **_k):
    raise OSError("no idle")

  def _nice(n):
    called["nice"] = n
    return n

  monkeypatch.setattr(os, "sched_setscheduler", _sched)
  monkeypatch.setattr(os, "nice", _nice)
  assert yield_to_modeld() == "nice"
  assert called.get("nice") == SPEEDSIGND_NICE
  assert called.get("cores") == SPEEDSIGND_CORES


def test_infer_threads_default_one_and_clamped():
  assert parse_infer_threads(None) == 1
  assert parse_infer_threads("") == 1
  assert parse_infer_threads("nope") == 1
  assert parse_infer_threads("8") == 1
  assert parse_infer_threads("0") == 1
  assert THREADS_ENV == "NAP_SPEED_SIGN_THREADS"
  n = limit_infer_threads(1)
  assert n == 1
  for key in (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
    "VECLIB_MAXIMUM_THREADS",
    "BLIS_NUM_THREADS",
    "OPENCV_FOR_THREADS_NUM",
    "GOTO_NUM_THREADS",
    "TINYGRAD_NUM_THREADS",
  ):
    assert os.environ[key] == "1"


def test_infer_cap_ms_default_and_zero_disables():
  assert parse_infer_cap_ms(None) == 800.0
  assert parse_infer_cap_ms("") == 800.0
  assert parse_infer_cap_ms("nope") == 800.0
  assert parse_infer_cap_ms("0") == 0.0
  assert parse_infer_cap_ms("50") == 50.0
  assert parse_infer_cap_ms("99999") == 5000.0
  assert CAP_MS_ENV == "NAP_SPEED_SIGN_INFER_CAP_MS"


def test_duty_cycle_caps_one_core_at_a_quarter():
  """Idle is at least 3× infer, so a long read is 25% of the pinned core.

  Road test of 795d252f: ~4.3 s infer back-to-back was ~87% of a core.
  4.3 × 4 = 17.2 s start-to-start. A 50 ms infer still waits out 1 Hz.
  """
  period, budget, cap = 1.0, 0.100, 0.800
  for infer_s in (3.8, 4.3, 5.4):
    gap = read_gap_s(infer_s, period)
    assert abs(gap - infer_s * (1.0 + IDLE_FACTOR)) < 1e-9
    assert gap - infer_s >= IDLE_FACTOR * infer_s - 1e-9
    assert steady_core_duty(infer_s, period) <= CPU_BUDGET + 1e-9
    assert abs(_start_gap(100.0, infer_s, period, budget, cap) - gap) < 1e-9
  assert abs(read_gap_s(0.050, period) - period) < 1e-9
  assert steady_core_duty(0.050, period) < CPU_BUDGET
  # The old 2× infer + 1 s gap was still too hot (3.8 s → 8.6 s is 44%).
  assert read_gap_s(3.8, period) > (2.0 * 3.8 + period)


def test_period_wins_when_four_infers_fit_inside_it():
  assert abs(read_gap_s(0.20, 1.0) - 1.0) < 1e-9
  assert abs(read_gap_s(0.35, 1.0) - 1.40) < 1e-9
  assert abs(_start_gap(10.0, 0.20, 1.0, 0.100, 0.800) - 1.0) < 1e-9


def test_pressure_backoff_adds_another_three_infers():
  infer_s, period = 4.0, 1.0
  healthy = HostPressure(procs_running=6, procs_blocked=0, ncpu=8, alert_text="")
  assert pressure_reason(healthy) == ""
  assert backoff_extra_s(infer_s, period, "") == 0.0
  assert abs(steady_core_duty(infer_s, period) - CPU_BUDGET) < 1e-9

  hot = HostPressure(procs_running=8 + 2 + 1, procs_blocked=0, ncpu=8)
  assert pressure_reason(hot) == "cpu-load"
  extra = backoff_extra_s(infer_s, period, pressure_reason(hot))
  assert extra == IDLE_FACTOR * infer_s
  # 4× infer + another 3× infer → 4/28 ≈ 14% of the pinned core.
  assert abs(steady_core_duty(infer_s, period, extra) - (4.0 / 28.0)) < 1e-9
  assert abs(_start_gap(20.0, infer_s, period, 0.1, 0.8, backoff=extra) - 28.0) < 1e-9

  lag = HostPressure(alert_text="Driving Model Lagging 22.0% frames dropped", procs_running=4, ncpu=8)
  assert pressure_reason(lag) == "model-lag"
  controls = HostPressure(alert_text="Communication Issue Between Processes", procs_running=3, ncpu=8)
  assert pressure_reason(controls) == "controls-lag"
  disk = HostPressure(procs_running=4, procs_blocked=4, ncpu=8)
  assert pressure_reason(disk) == "disk-stall"


def test_timing_lines_are_greppable():
  line = format_read_timing(
    read_interval_ms=17200, infer_ms=4300, engaged=True, backoff=False, reason="pace",
    cpu_share=0.25, threads=1, backend="tinygrad-clang",
  )
  assert line.startswith("speedsignd timing ")
  assert "read_interval_ms=17200" in line
  assert "infer_ms=4300" in line
  assert "cpu_share=0.250" in line
  assert "threads=1" in line
  assert "backend=tinygrad-clang" in line
  assert "engaged=1" in line
  assert "backoff=0" in line
  missing = format_read_timing(
    read_interval_ms=1000, infer_ms=50, engaged=False, backoff=False, reason="pace",
  )
  assert "cpu_share=na" in missing
  assert "backend=unset" in missing
  event = format_backoff_timing(extra_ms=3980, infer_ms=3980, engaged=True, reason="model-lag")
  assert event.startswith("speedsignd timing ")
  assert "backoff=1" in event and "reason=model-lag" in event and "engaged=1" in event


def test_driving_cores_stay_with_modeld_and_controlsd():
  """Core 2 is the little core with no exclusive realtime owner on the 3X.

  0 UI CTRL_HIGH, 1 sensord, 3 pandad and encoderd, 4 controlsd/card/selfdrived,
  5 plannerd/radard, 6 camerad, 7 modeld FIFO. locationd's family is priority 5
  on 0–3 and still preempts SCHED_IDLE on core 2.
  """
  from pathlib import Path
  root = Path(__file__).resolve().parents[3]
  modeld = (root / "selfdrive" / "modeld" / "modeld.py").read_text(encoding="utf-8")
  controlsd = (root / "selfdrive" / "controls" / "controlsd.py").read_text(encoding="utf-8")
  ui = (root / "selfdrive" / "ui" / "ui.py").read_text(encoding="utf-8")
  sensord = (root / "system" / "sensord" / "sensord.py").read_text(encoding="utf-8")
  camerad = (root / "system" / "camerad" / "main.cc").read_text(encoding="utf-8")
  encoderd = (root / "system" / "loggerd" / "encoderd.cc").read_text(encoding="utf-8")
  locationd = (root / "selfdrive" / "locationd" / "locationd.py").read_text(encoding="utf-8")
  hardware = (root / "system" / "hardware" / "tici" / "hardware.py").read_text(encoding="utf-8")
  assert "config_realtime_process(7, 54)" in modeld
  assert "config_realtime_process(4, Priority.CTRL_HIGH)" in controlsd
  assert "config_realtime_process(0, Priority.CTRL_HIGH)" in ui
  assert "config_realtime_process([1, ], 1)" in sensord
  assert "set_core_affinity({6})" in camerad
  assert "set_core_affinity({3})" in encoderd
  assert "config_realtime_process([0, 1, 2, 3], 5)" in locationd
  assert 'taskset", "-pc", "3"' in hardware
  for core in (0, 1, 3, 4, 5, 6, 7):
    assert core not in SPEEDSIGND_CORES
  assert SPEEDSIGND_CORES == (2,)
  assert SPEEDSIGND_NICE >= 19
  assert THREADS_DEFAULT == 1


def test_model_frame_drop_and_skip_are_backoff_reasons():
  watch = ModelWatch()
  watch.observe(10, 0.0)
  watch.observe(11, 1.0)
  assert watch.reason() == ""
  watch.observe(12, 6.0)
  assert watch.reason() == "model-drop"
  watch.observe(15, 6.0)
  assert watch.reason() == "model-drop+model-skip"
  assert watch.consume() == "model-drop+model-skip"
  assert watch.reason() == "model-drop"
  watch.observe(16, 1.0)
  # A hot drop stays until the next infer consumes it, then clears once it is under 5%.
  assert watch.consume() == "model-drop"
  assert watch.reason() == ""
  skipped = ModelWatch()
  skipped.observe(1, 0.0)
  skipped.observe(4, 0.0)
  assert skipped.reason() == "model-skip"
  assert skipped.consume() == "model-skip"
  assert skipped.reason() == ""
  assert join_reasons("model-drop", "cpu-budget", "model-drop") == "model-drop+cpu-budget"


def test_model_skip_or_exec_over_50ms_pauses_at_least_10s():
  """modelExecutionTime is seconds. 24 ms is healthy. Above 50 ms pauses 10 s."""
  from openpilot.selfdrive.speedsignd.speedsignd import MODEL_GUARD_PAUSE_S, model_guard_rest_s
  healthy = ModelWatch()
  healthy.observe(1, 0.0, 0.024)
  healthy.observe(2, 1.0, 0.026)
  assert healthy.reason() == ""
  assert not healthy.hard_guard()
  assert model_guard_rest_s(4.0, 1.0, "", False) == 0.0

  slow = ModelWatch()
  slow.observe(1, 0.0, 0.024)
  slow.observe(2, 0.0, 0.050)
  assert slow.reason() == ""
  slow.observe(3, 0.0, 0.051)
  assert slow.reason() == "model-exec"
  assert slow.hard_guard()
  assert model_guard_rest_s(1.0, 1.0, slow.reason(), slow.hard_guard()) >= MODEL_GUARD_PAUSE_S
  assert model_guard_rest_s(1.0, 1.0, slow.reason(), slow.hard_guard()) >= 10.0
  assert slow.consume() == "model-exec"
  assert slow.reason() == "model-exec"
  slow.observe(4, 0.0, 0.022)
  assert slow.consume() == "model-exec"
  assert slow.reason() == ""

  skipped = ModelWatch()
  skipped.observe(1, 0.0, 0.022)
  skipped.observe(4, 0.0, 0.022)
  assert skipped.reason() == "model-skip"
  assert skipped.hard_guard()
  # A short infer's 3× backoff is under 10 s; the hard guard raises the floor.
  assert model_guard_rest_s(0.2, 1.0, skipped.reason(), True) >= 10.0

  drop = ModelWatch()
  drop.observe(1, 0.0, 0.022)
  drop.observe(2, 6.0, 0.022)
  assert drop.reason() == "model-drop"
  assert not drop.hard_guard()
  assert model_guard_rest_s(4.0, 1.0, drop.reason(), False) == 12.0


def test_cpu_share_meter_and_budget():
  text = "123 (speedsignd) R 1 1 1 0 -1 0 0 0 0 0 40 10 0 0"
  assert parse_proc_stat_jiffies(text) == 50
  assert parse_proc_thread_count("Name:\tspeedsignd\nThreads:\t1\n") == 1
  meter = CpuShareMeter()
  meter._clk = 100
  assert meter.sample(0.0, 0) is None
  # 25 jiffies in 1 s at 100 Hz is 0.25 of one core.
  assert abs(meter.sample(1.0, 25) - 0.25) < 1e-9
  assert not cpu_over_budget(0.25)
  assert not cpu_over_budget(0.27)
  assert cpu_over_budget(0.29)
  assert not cpu_over_budget(None)
