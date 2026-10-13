#!/usr/bin/env python3
"""On-drive MUTCD speed-sign logger (log-only).

Reads the ROAD camera + GNSS, appends JSONL under /data, and publishes
liveSpeedSignNAP for the on-road HUD. Does not write sqlite, does not change
vCruise / HUD MAX, and does not talk to osm.org.

Stock modelV2 has no speedSign head — this is a separate process, default off.

Safety: Logger On starts the process + HUD. Detect keeps running while
openpilot is engaged. Inference is CPU only (onnxruntime
CPUExecutionProvider, else cv2.dnn on the CPU target, else tinygrad
DEV=CPU / CLANG). It must not open the QCOM GPU modeld uses. The first
realize compiles only while known-disengaged and is cached per weights
sha. One ONNX thread, SCHED_IDLE (else nice 19), pinned to little
core 2 — not core 0 (UI), 1 (sensord), 3 (pandad/encoderd), 4 (controlsd),
5 (plannerd/radard), 6 (camerad), or 7 (modeld). While engaged the average
is at most 25% of that core: after each infer the process idles at least
3× the infer time. modelV2 frame drops add more rest. A real modelV2
frame skip, or modelExecutionTime above 50 ms, pauses at least 10 s.
A 20 Hz SubMaster poll of 20 Hz modelV2 is not that skip. HUD lights
only when two reads agree on the same mph, then holds 45 s. JSONL stays a
short line, skipped while engaged if disk stalls. SubMaster is polled at
20 Hz. Unknown cereal after a short startup allows throttled detect, but
not the compile. Not gated on park / Force Offroad. Detect copies one
right-side 628×320 ROAD window (x 1300–1928, y 520–840) and letterboxes
it to YOLO_IMGSZ. 4 Hz YOLO on a 3X starved modeld; the default stays 1 Hz.
"""
from __future__ import annotations

import math
import os

# Before numpy / tinygrad / OpenBLAS import. One compute thread.
# Package __init__ sets the same keys even earlier. Manager forks after
# numpy is already loaded, so main() also calls limit_infer_threads().
for _thread_key in (
  "OMP_NUM_THREADS",
  "OPENBLAS_NUM_THREADS",
  "MKL_NUM_THREADS",
  "NUMEXPR_NUM_THREADS",
  "VECLIB_MAXIMUM_THREADS",
  "BLIS_NUM_THREADS",
  "OPENCV_FOR_THREADS_NUM",
  "GOTO_NUM_THREADS",
  "TINYGRAD_NUM_THREADS",
  "KMP_NUM_THREADS",
):
  os.environ[_thread_key] = "1"
os.environ["OMP_DYNAMIC"] = "FALSE"
os.environ["MKL_DYNAMIC"] = "FALSE"
os.environ["OMP_WAIT_POLICY"] = "PASSIVE"
os.environ["BEAM"] = "0"
os.environ["NOOPT"] = "0"
os.environ["CPU_COUNT"] = "1"
os.environ["OPENCV_OPENCL_RUNTIME"] = "disabled"
os.environ["OPENCV_DNN_OPENCL"] = "0"

import threading
import time
from dataclasses import dataclass
from typing import Any

from openpilot.selfdrive.speedsignd.debounce import SignDebounce
from openpilot.selfdrive.speedsignd.hud import LiveSignHold, apply_live_sign, frame_reads
from openpilot.selfdrive.speedsignd.jsonl import JsonlLogger, make_record, record_line
from openpilot.selfdrive.speedsignd.detect import (
  CAP_MS_DEFAULT,
  CAP_MS_ENV,
  THREADS_DEFAULT,
  THREADS_ENV,
  SpeedSignDetector,
  assert_cpu_backend,
  compile_allowed,
  describe_infer_runtimes,
  limit_infer_threads,
  ocr_tight_y,
  parse_infer_cap_ms,
  parse_infer_threads,
)
from openpilot.selfdrive.speedsignd.nv12 import copy_nv12_detect_crop, copy_nv12_rect
from openpilot.selfdrive.speedsignd.paths import PARAM_KEY, default_log_path, default_onnx_path, default_proposer_path
from openpilot.selfdrive.speedsignd import two_stage
from openpilot.selfdrive.speedsignd.detect_types import SpeedSign
from openpilot.selfdrive.speedsignd.yolo import REFINE_OVERRIDE_CONF, SIGN_LIKE_CONF

# Safe default. 4 Hz YOLOv8s tinygrad on ROAD frames saturates a 3X CPU core
# and Ratekeeper catch-up never sleeps. Override: env NAP_SPEED_SIGN_HZ.
SPEEDSIGND_HZ = 1.0
HZ_ENV = "NAP_SPEED_SIGN_HZ"
HZ_MIN = 0.2
HZ_MAX = 4.0
# Poll selfdriveState faster than detect. 100 Hz service + 1 Hz update makes
# SubMaster.alive flap (timeout is 10/freq = 100 ms) and stuck the WAIT plate.
SM_HZ = 20.0
# modelV2 is published at 20 Hz. SubMaster keeps only the latest message.
MODEL_HZ = 20.0
# After this, unread cereal allows throttled detect instead of forever-WAIT.
UNKNOWN_GRACE_S = 2.0
# Historical tinygrad budget. A real 3X infer is several seconds, so this
# must not insert a second wait — that was the ~2× infer gap. Pressure
# backoff (below) is the CPU safety valve.
INFER_BUDGET_MS = 100.0
INFER_LOG_PERIOD_S = 15.0
# comma 3X. Host pressure uses os.cpu_count() when the device reports it.
NCPU_DEFAULT = 8
# procs_running above ncpu + this means the run queue is backed up.
CPU_OVERSUBSCRIBE_EXTRA = 2
# procs_blocked this high is the disk-stall shape that froze hardwared.
DISK_BLOCKED_MIN = 4
# One JSONL observation is a handful of numbers. Refuse anything larger.
JSONL_LINE_MAX = 240
# selfdriveState alert text. No extra subscriber: modeld lag and
# communication-issue are already published on the socket we poll.
LAG_ALERT_MARKERS = (
  ("driving model lagging", "model-lag"),
  ("modeldlagging", "model-lag"),
  ("communication issue", "controls-lag"),
  ("commissue", "controls-lag"),
  ("high cpu", "cpu-load"),
  ("highcpuusage", "cpu-load"),
)
# 3X little cores are 0–3. Core 2 is the only one without a dedicated
# realtime owner: UI is CTRL_HIGH on 0, sensord is on 1, pandad (54) and
# encoderd (52) share 3. Big cores: controlsd/card/selfdrived on 4,
# plannerd/radard on 5, camerad on 6, modeld FIFO 54 on 7. locationd's
# family is priority 5 across 0–3 and preempts SCHED_IDLE.
SPEEDSIGND_NICE = 19
SPEEDSIGND_CORE = 2
SPEEDSIGND_CORES = (SPEEDSIGND_CORE,)
# Idle at least 3× infer between reads → duty = infer / (4× infer) = 25%.
IDLE_FACTOR = 3.0
CPU_BUDGET = 1.0 / (1.0 + IDLE_FACTOR)
CPU_BUDGET_SLACK = 0.03
# Duty is averaged over this window. A 2–3 frame burst may run back-to-back;
# the idle after it brings the trailing average back to 25%.
DUTY_WINDOW_S = 10.0
# Extra YOLO reads after the frame that scored ≥ SIGN_LIKE_CONF. Two extra
# reads is three frames total, still on the tile that saw the sign.
BURST_EXTRA = 2
# modelV2.frameDropPerc. The lag alert fires near 20; back off earlier.
MODEL_DROP_PERC = 5.0
# modelV2.modelExecutionTime is seconds (perf_counter in modeld). 22–26 ms
# healthy is ~0.024. Above 50 ms means something else is on modeld's GPU
# or the model itself stalled. Pause at least 10 s. Same for any frameId skip.
MODEL_EXEC_LIMIT_S = 0.050
MODEL_GUARD_PAUSE_S = 10.0
# Route 0000017f: a 177 s infer times IDLE_FACTOR made extra_ms 531828.
# model-skip rest is capped so one slow compile cannot hide the plate for 9 min.
MODEL_SKIP_BACKOFF_CAP_S = 60.0
SECOND_LOOK_FRAMES = 2
SECOND_LOOK_DEADLINE_S = 0.30
VISION_TIMEOUT_MS = 200
SERVICE_NAME = "liveSpeedSignNAP"
# Retry ONNX after Settings → Install weights without requiring a reboot.
ONNX_RETRY_S = 15.0

# Matches selfdrive.selfdrived.state.ACTIVE_STATES (actuators on).
CONTROLLING_STATES = frozenset({"enabled", "softDisabling", "overriding"})
STATE_BY_RAW = {
  0: "disabled",
  1: "preEnabled",
  2: "enabled",
  3: "softDisabling",
  4: "overriding",
}


def should_run_speed_sign_log(started: bool, params: Any, _cp: Any = None) -> bool:
  """Manager gate. Default-off: unset/false param never starts the process."""
  try:
    enabled = params.get_bool(PARAM_KEY)
  except Exception:
    enabled = False
  return bool(started) and bool(enabled)


@dataclass(frozen=True)
class EngagementSample:
  """Last read of selfdriveState for the ONNX / WAIT gate."""
  known: bool
  controlling: bool
  allow_detect: bool
  detect_paused: bool  # HUD WAIT. Engaged driving does not set this.
  enabled: bool | None = None
  active: bool | None = None
  state: str | None = None
  alive: bool | None = None
  valid: bool | None = None
  recv_frame: int = 0
  unknown_after_grace: bool = False


def selfdrive_state_name(state: Any) -> str | None:
  """Normalize cereal OpenpilotState to a short name."""
  if state is None:
    return None
  if isinstance(state, str):
    return state.split(".")[-1]
  if isinstance(state, int):
    return STATE_BY_RAW.get(state)
  raw = getattr(state, "raw", None)
  if raw is not None:
    try:
      mapped = STATE_BY_RAW.get(int(raw))
      if mapped is not None:
        return mapped
    except (TypeError, ValueError):
      pass
  try:
    return str(state).split(".")[-1]
  except Exception:
    return None


def op_controlling(*, active: bool | None = None, state: Any = None,
                   enabled: bool | None = None) -> bool:
  """True only when openpilot is commanding actuators.

  Prefer `active`. `state` in enabled / softDisabling / overriding also
  counts. `disabled` and `preEnabled` do not. Unknown fields are not
  controlling — the caller applies a short startup grace instead of
  treating cereal silence as forever-WAIT.
  """
  name = selfdrive_state_name(state)
  if name == "disabled" or name == "preEnabled":
    return False
  if name in CONTROLLING_STATES:
    return True
  if active is True:
    return True
  if active is False:
    return False
  if enabled is False:
    return False
  if enabled is True:
    return True
  return False


def should_run_onnx_detect(controlling: bool) -> bool:
  """ONNX runs while engaged. `controlling` is logged, not a pause."""
  del controlling
  return True


def _sm_flag(sm: Any, flag_name: str, service: str) -> bool | None:
  try:
    flags = getattr(sm, flag_name, None)
    if flags is None or service not in flags:
      return None
    return bool(flags[service])
  except Exception:
    return None


def _sm_seen(sm: Any, service: str) -> bool:
  """True after a real selfdriveState payload. Do not use recv_frame <= 0.

  SubMaster starts recv_frame at 0 and writes frame 0 on the first receive.
  The old `recv_frame <= 0` check treated that first (and only) packet as
  unseen, then a 1 Hz loop left alive stale → permanent WAIT.
  """
  try:
    seen = getattr(sm, "seen", None)
    if seen is not None and service in seen:
      return bool(seen[service])
  except Exception:
    pass
  try:
    return int(sm.recv_frame.get(service, -1)) > 0
  except Exception:
    return False


def engagement_from_sm(
  sm: Any,
  *,
  now: float | None = None,
  started_at: float = 0.0,
  grace_s: float = UNKNOWN_GRACE_S,
) -> EngagementSample:
  """Read engagement without treating alive/valid flaps as 'OP is driving'.

  Last parsed active/state/enabled wins even if SubMaster marks the socket
  dead — 1 Hz poll of a 100 Hz service makes alive timeout (100 ms) fire
  every tick. Unknown cereal: no YOLO for `grace_s`, then throttled detect.
  Engaged (controlling) still allows detect. detect_paused stays false so
  the HUD shows mph instead of WAIT.
  """
  service = "selfdriveState"
  try:
    recv_frame = int(sm.recv_frame.get(service, 0) or 0)
  except Exception:
    recv_frame = 0
  alive = _sm_flag(sm, "alive", service)
  valid = _sm_flag(sm, "valid", service)
  seen = _sm_seen(sm, service)

  active = enabled = None
  state: str | None = None
  known = False
  if seen:
    try:
      obj: Any = sm[service]
      active = bool(obj.active)
      enabled = bool(obj.enabled)
      state = selfdrive_state_name(getattr(obj, "state", None))
      known = True
    except Exception:
      known = False

  controlling = op_controlling(active=active, state=state, enabled=enabled) if known else False
  if known:
    # Engaged used to set allow_detect false. That skipped ~95% of a drive.
    # Priority, core mask, and pressure backoff protect modeld instead.
    allow_detect = True
    detect_paused = False
    unknown_after = False
  else:
    # now is None (unit tests): treat as post-grace so unknown allows detect.
    in_grace = False if now is None else (float(now) - float(started_at)) < float(grace_s)
    allow_detect = not in_grace
    detect_paused = False
    unknown_after = not in_grace

  return EngagementSample(
    known=known,
    controlling=controlling,
    allow_detect=allow_detect,
    detect_paused=detect_paused,
    enabled=enabled,
    active=active,
    state=state,
    alive=alive,
    valid=valid,
    recv_frame=recv_frame,
    unknown_after_grace=unknown_after,
  )


def engagement_log_fields(sample: EngagementSample) -> str:
  return (
    f"controlling={sample.controlling} enabled={sample.enabled} "
    + f"active={sample.active} state={sample.state} "
    + f"alive={sample.alive} valid={sample.valid} known={sample.known}"
  )


def engaged_from_sm(sm: Any, **kwargs) -> bool:
  """Back-compat: True when OP is commanding actuators (not cereal-unknown)."""
  return engagement_from_sm(sm, **kwargs).controlling


def parse_detect_hz(raw: str | None, default: float = SPEEDSIGND_HZ) -> float:
  """Clamp env override. Default 1 Hz; refuse junk / out-of-range."""
  if raw is None or str(raw).strip() == "":
    return float(default)
  try:
    hz = float(raw)
  except (TypeError, ValueError):
    return float(default)
  if not math.isfinite(hz):
    return float(default)
  return min(HZ_MAX, max(HZ_MIN, hz))


def infer_overran(infer_s: float, period_s: float, budget_s: float) -> bool:
  """True when this infer used more than the period or the CPU budget.

  Diagnostic only. Spacing does not pay this back with a second infer.
  """
  return infer_s > period_s or infer_s > budget_s


def read_gap_s(infer_s: float, period_s: float, backoff_s: float = 0.0) -> float:
  """Start-to-start gap. Idle is at least 3× infer, and never faster than the period.

  A 4 s infer waits 12 s, so the next start is 16 s later and the core
  averages 25%. A 50 ms infer still waits out a 1 s period (5% of a core).
  """
  infer_s = max(0.0, float(infer_s))
  period_s = max(0.0, float(period_s))
  paced = max(period_s, infer_s * (1.0 + IDLE_FACTOR))
  return paced + max(0.0, float(backoff_s))


def steady_core_duty(infer_s: float, period_s: float, backoff_s: float = 0.0) -> float:
  """Fraction of the one core speedsignd is pinned to."""
  gap = read_gap_s(infer_s, period_s, backoff_s)
  if gap <= 0.0:
    return 1.0
  return min(1.0, max(0.0, float(infer_s)) / gap)


def next_detect_mono(infer_end: float, infer_s: float, period_s: float, budget_s: float,
                     cap_s: float = 0.0, backoff_s: float = 0.0) -> float:
  """Earliest monotonic time another ONNX infer may start.

  Start-to-start is max(period, 4× infer) plus any extra rest. The road
  test at 795d252f ran the next read as soon as the infer ended (~87% of
  core 0). `budget_s` and `cap_s` stay in the signature; they do not add
  a second wait on top of the duty cycle.
  """
  if not math.isfinite(budget_s) or not math.isfinite(cap_s):
    raise ValueError("budget and cap must be finite")
  gap = read_gap_s(infer_s, period_s, backoff_s)
  return (float(infer_end) - float(infer_s)) + gap


def window_compute(spans: list[tuple[float, float]], t: float, window: float = DUTY_WINDOW_S) -> float:
  """Infer seconds overlapping [t-window, t]. spans are (end, duration)."""
  lo = float(t) - float(window)
  total = 0.0
  for end, dur in spans:
    end_f = float(end)
    dur_f = max(0.0, float(dur))
    start = end_f - dur_f
    a = max(start, lo)
    b = min(end_f, float(t))
    if b > a:
      total += b - a
  return total


def earliest_budget_start(
  spans: list[tuple[float, float]], now: float, next_dur: float,
  window: float = DUTY_WINDOW_S, budget: float = CPU_BUDGET,
) -> float:
  """Earliest t ≥ now such that the infer [t, t+next_dur] keeps the window ≤ budget.

  Past spans age out of the trailing window. A burst that already spent the
  allowance waits here until idle has repaid it.
  """
  now_f = float(now)
  next_dur = max(0.0, float(next_dur))
  allowance = float(window) * float(budget)

  def fits(t: float) -> bool:
    finish = t + next_dur
    used = window_compute(spans, finish, window)
    return used + next_dur <= allowance + 1e-6

  if fits(now_f):
    return now_f
  candidates = [now_f]
  for end, _dur in spans:
    candidates.append(float(end) + float(window) - next_dur)
  for t in sorted(candidates):
    t = max(now_f, t)
    if fits(t):
      return t
  newest = max((float(end) for end, _dur in spans), default=now_f)
  return max(now_f, newest + float(window) - next_dur)


def sign_burst_trigger(diag: dict | None, signs=None) -> bool:
  """True when any speed-sign class scored ≥ SIGN_LIKE_CONF.

  sl_peak covers sl_NN below YOLO_MIN_CONF. A decoded sign, or an
  sl_unreadable peak, counts too. neg_* does not.
  """
  diag = diag or {}
  try:
    if float(diag.get("sl_peak_conf") or 0.0) >= SIGN_LIKE_CONF:
      return True
  except (TypeError, ValueError):
    pass
  peak_name = str(diag.get("peak_name") or "")
  try:
    peak = float(diag.get("peak_conf") or 0.0)
  except (TypeError, ValueError):
    peak = 0.0
  if peak >= SIGN_LIKE_CONF and peak_name.startswith(("sl_", "speedLimit")):
    return True
  for s in signs or []:
    try:
      if float(getattr(s, "conf", 0.0) or 0.0) >= SIGN_LIKE_CONF:
        return True
    except (TypeError, ValueError):
      continue
  return False


def schedule_after_read(
  *,
  now: float,
  infer_s: float,
  period_s: float,
  budget_s: float,
  cap_s: float,
  backoff_s: float,
  sign_like: bool,
  window_used: int,
  window_count: int,
  burst_left: int,
  burst_window: int,
  burst_cluster_s: float,
  spans: list[tuple[float, float]],
) -> tuple[float, int, int, int, float, bool]:
  """Next detect time plus burst state.

  Returns (next_mono, burst_left, burst_window, next_window, cluster_s, bursting_next).

  Steady reads keep max(period, 4× infer). A sign-like score schedules up to
  BURST_EXTRA more reads on the same tile, immediately, while that cluster
  still fits in the 10 s × 25% allowance. The following non-burst start waits
  until the trailing window is back at 25%. A model-guard backoff cancels the
  burst. `budget_s` and `cap_s` stay on next_detect_mono; they do not add a
  second wait.
  """
  infer_s = max(0.0, float(infer_s))
  spans_now = list(spans) + [(float(now), infer_s)]
  paced = next_detect_mono(float(now), infer_s, period_s, budget_s, cap_s, backoff_s)
  duty_t = earliest_budget_start(spans_now, float(now), infer_s)
  steady = max(paced, duty_t)
  nwin = max(1, int(window_count))
  next_window = (int(window_used) + 1) % nwin
  allowance = DUTY_WINDOW_S * CPU_BUDGET
  if float(backoff_s) > 0.0:
    return steady, 0, int(window_used), next_window, 0.0, False
  if int(burst_left) > 0:
    cluster = float(burst_cluster_s) + infer_s
    left = int(burst_left) - 1
    if left > 0 and cluster + infer_s <= allowance + 1e-6:
      return float(now), left, int(burst_window), int(burst_window), cluster, True
    return steady, 0, int(burst_window), next_window, 0.0, False
  if sign_like:
    cluster = infer_s
    if BURST_EXTRA > 0 and cluster + infer_s <= allowance + 1e-6:
      return float(now), BURST_EXTRA, int(window_used), int(window_used), cluster, True
  return steady, 0, int(window_used), next_window, 0.0, False


@dataclass(frozen=True)
class HostPressure:
  """CPU / disk / alert snapshot. No new cereal subscriber."""
  procs_running: int | None = None
  procs_blocked: int | None = None
  ncpu: int = NCPU_DEFAULT
  alert_text: str = ""


def parse_proc_stat_pressure(text: str) -> tuple[int | None, int | None]:
  """procs_running and procs_blocked from /proc/stat."""
  running = blocked = None
  for line in text.splitlines():
    if line.startswith("procs_running "):
      try:
        running = int(line.split()[1])
      except (IndexError, ValueError):
        running = None
    elif line.startswith("procs_blocked "):
      try:
        blocked = int(line.split()[1])
      except (IndexError, ValueError):
        blocked = None
  return running, blocked


def read_proc_stat_pressure(path: str = "/proc/stat") -> tuple[int | None, int | None]:
  try:
    with open(path, encoding="utf-8") as f:
      return parse_proc_stat_pressure(f.read())
  except OSError:
    return None, None


def lag_reason_from_alert(*parts: str | None) -> str:
  """Map a selfdriveState alert onto a backoff reason, or ''."""
  blob = " ".join(p for p in parts if p).lower()
  if not blob:
    return ""
  for needle, reason in LAG_ALERT_MARKERS:
    if needle in blob:
      return reason
  return ""


def alert_text_from_sm(sm: Any) -> str:
  try:
    obj = sm["selfdriveState"]
  except Exception:
    return ""
  parts: list[str] = []
  for name in ("alertText1", "alertText2", "alertType"):
    try:
      val = getattr(obj, name, None)
    except Exception:
      val = None
    if val:
      parts.append(str(val))
  return " ".join(parts)


def pressure_reason(pressure: HostPressure) -> str:
  """Why to rest, or '' when the host looks healthy."""
  reasons: list[str] = []
  alert = lag_reason_from_alert(pressure.alert_text)
  if alert:
    reasons.append(alert)
  ncpu = int(pressure.ncpu) if pressure.ncpu else NCPU_DEFAULT
  if pressure.procs_running is not None and pressure.procs_running > ncpu + CPU_OVERSUBSCRIBE_EXTRA:
    reasons.append("cpu-load")
  if pressure.procs_blocked is not None and pressure.procs_blocked >= DISK_BLOCKED_MIN:
    reasons.append("disk-stall")
  out: list[str] = []
  for reason in reasons:
    if reason not in out:
      out.append(reason)
  return "+".join(out)


def disk_stalled(pressure: HostPressure) -> bool:
  return pressure.procs_blocked is not None and pressure.procs_blocked >= DISK_BLOCKED_MIN


def backoff_extra_s(infer_s: float, period_s: float, reason: str) -> float:
  """Extra rest on top of the 3× idle. Empty reason → none.

  Another 3× infer drops a long infer from 25% of a core to about 14%.
  """
  if not reason:
    return 0.0
  return max(float(period_s), IDLE_FACTOR * max(0.0, float(infer_s)))


def model_guard_rest_s(infer_s: float, period_s: float, reason: str, hard: bool) -> float:
  """Frame-drop backoff, with a 10 s floor on a frame skip or modeld > 50 ms.

  Any reason that includes model-skip is also capped at 60 s, hard or not.
  model-drop and model-exec keep the uncapped 3× rest.
  """
  extra = backoff_extra_s(infer_s, period_s, reason)
  if hard:
    extra = max(extra, MODEL_GUARD_PAUSE_S)
  if reason and "model-skip" in reason:
    return min(extra, MODEL_SKIP_BACKOFF_CAP_S)
  return extra


def collect_host_pressure(sm: Any, *, stat_path: str = "/proc/stat", ncpu: int | None = None) -> HostPressure:
  running, blocked = read_proc_stat_pressure(stat_path)
  if ncpu is None:
    ncpu = os.cpu_count() or NCPU_DEFAULT
  return HostPressure(
    procs_running=running,
    procs_blocked=blocked,
    ncpu=int(ncpu),
    alert_text=alert_text_from_sm(sm),
  )


def jsonl_line_is_light(record: dict) -> bool:
  return len(record_line(record).encode("utf-8")) <= JSONL_LINE_MAX


def reset_ratekeeper_if_behind(rk, now: float) -> bool:
  """Drop Ratekeeper catch-up so an overrun does not burst more infers."""
  if rk.remaining < 0:
    rk._next_frame_time = now + rk._interval
    return True
  return False


def model_frame_gap_allowed(dt_s: float, hz: float = MODEL_HZ) -> int:
  """How far frameId may jump without modeld having dropped a frame.

  Both sides are 20 Hz, and SubMaster keeps only the latest modelV2, so a
  poll that lands between two publishes sees delta 2. A vision recv or an
  infer that blocks this poll for dt seconds misses about dt*20 publishes.
  +1 is that one coalesced message. frameDropPerc is the real modeld drop.
  """
  dt_s = max(0.0, float(dt_s))
  return max(2, math.ceil(dt_s * float(hz)) + 1)


@dataclass
class ModelWatch:
  """modelV2 frame drops, frame-id gaps, and modelExecutionTime. No new socket."""
  last_frame: int | None = None
  last_mono: float | None = None
  last_drop: float | None = None
  last_exec: float | None = None
  drop_hot: bool = False
  exec_hot: bool = False
  skipped: bool = False

  def observe(self, frame_id: int | None, drop: float | None, exec_s: float | None = None,
              now: float | None = None) -> None:
    if now is None:
      now = time.monotonic()
    if frame_id is not None and self.last_frame is not None and self.last_mono is not None:
      gap = int(frame_id) - int(self.last_frame)
      allowed = model_frame_gap_allowed(float(now) - float(self.last_mono))
      if gap > allowed:
        self.skipped = True
    if drop is not None:
      value = float(drop)
      # Jitter under 5% is not a drop. 5% is well before modeld's ~20% lag alert.
      if value >= MODEL_DROP_PERC:
        self.drop_hot = True
      self.last_drop = value
    if exec_s is not None:
      try:
        exec_value = float(exec_s)
      except (TypeError, ValueError):
        exec_value = None
      if exec_value is not None and math.isfinite(exec_value):
        self.last_exec = exec_value
        if exec_value > MODEL_EXEC_LIMIT_S:
          self.exec_hot = True
    if frame_id is not None:
      self.last_frame = int(frame_id)
      self.last_mono = float(now)

  def hard_guard(self) -> bool:
    """Any skipped frame id, or modeld execution above 50 ms."""
    return bool(self.skipped or self.exec_hot)

  def reason(self) -> str:
    parts: list[str] = []
    if self.drop_hot:
      parts.append("model-drop")
    if self.exec_hot:
      parts.append("model-exec")
    if self.skipped:
      parts.append("model-skip")
    return "+".join(parts)

  def consume(self) -> str:
    """Return the reason and clear a one-shot skip. A hot drop or exec stays hot."""
    reason = self.reason()
    self.skipped = False
    if self.last_drop is None or self.last_drop < MODEL_DROP_PERC:
      self.drop_hot = False
    if self.last_exec is None or self.last_exec <= MODEL_EXEC_LIMIT_S:
      self.exec_hot = False
    return reason


def join_reasons(*parts: str) -> str:
  out: list[str] = []
  for part in parts:
    for piece in (part or "").split("+"):
      if piece and piece not in out:
        out.append(piece)
  return "+".join(out)


def parse_proc_thread_count(text: str) -> int | None:
  for line in text.splitlines():
    if line.startswith("Threads:"):
      try:
        return int(line.split()[1])
      except (IndexError, ValueError):
        return None
  return None


def read_proc_thread_count(path: str = "/proc/self/status") -> int | None:
  try:
    with open(path, encoding="utf-8") as f:
      return parse_proc_thread_count(f.read())
  except OSError:
    return None


def parse_proc_stat_jiffies(text: str) -> int | None:
  """utime+stime from /proc/self/stat. comm may contain spaces."""
  end = text.rfind(")")
  if end < 0:
    return None
  parts = text[end + 2:].split()
  try:
    return int(parts[11]) + int(parts[12])
  except (IndexError, ValueError):
    return None


def read_proc_stat_jiffies(path: str = "/proc/self/stat") -> int | None:
  try:
    with open(path, encoding="utf-8") as f:
      return parse_proc_stat_jiffies(f.read())
  except OSError:
    return None


class CpuShareMeter:
  """CPU seconds of this process divided by wall seconds. 1.0 is one full core."""

  def __init__(self) -> None:
    self._mono: float | None = None
    self._jiffies: int | None = None
    self._clk = os.sysconf(os.sysconf_names["SC_CLK_TCK"]) if hasattr(os, "sysconf_names") else 100

  def sample(self, now: float, jiffies: int | None) -> float | None:
    if jiffies is None:
      return None
    if self._mono is None or self._jiffies is None:
      self._mono = float(now)
      self._jiffies = int(jiffies)
      return None
    dt = float(now) - self._mono
    dj = int(jiffies) - self._jiffies
    self._mono = float(now)
    self._jiffies = int(jiffies)
    if dt <= 1e-6 or self._clk <= 0:
      return None
    return (dj / float(self._clk)) / dt


def cpu_over_budget(share: float | None) -> bool:
  return share is not None and float(share) > CPU_BUDGET + CPU_BUDGET_SLACK


def yield_to_modeld() -> str:
  """SCHED_IDLE on core 2, else nice 19. Lowers speedsignd only."""
  from openpilot.common.realtime import drop_realtime, set_core_affinity
  drop_realtime()
  policy = "nice"
  try:
    idle = getattr(os, "SCHED_IDLE", None)
    if idle is None:
      raise OSError("no SCHED_IDLE")
    os.sched_setscheduler(0, idle, os.sched_param(0))
    policy = "idle"
  except OSError:
    try:
      os.nice(SPEEDSIGND_NICE)
    except OSError:
      pass
  try:
    set_core_affinity(list(SPEEDSIGND_CORES))
  except Exception:
    pass
  limit_infer_threads(1)
  return policy


def should_reset_detect_after_wait(last_allow: bool | None, allow_detect: bool) -> bool:
  """First manual tick after WAIT: do not sit out a leftover next_detect."""
  return bool(allow_detect) and last_allow is False


def drain_vision_latest(client) -> None:
  """Non-blocking ROAD recv so WAIT idle does not wedge the VisionIPC socket."""
  if client is None:
    return
  try:
    client.recv(timeout_ms=0)
  except Exception:
    pass


def process_frame(
  y,
  lat: float,
  lon: float,
  bearing: float | None,
  gps_ok: bool,
  detector: SpeedSignDetector,
  logger: JsonlLogger,
  now: float,
  rgb=None,
  debounce: SignDebounce | None = None,
  nv12=None,
  write_jsonl: bool = True,
) -> tuple[list, list[dict]]:
  """Detect on every ROAD frame. JSONL only with a GNSS fix and a short line.

  With `debounce`: HUD and JSONL both need two agreeing frames. The returned
  list is the signs that cast a vote this call, not a plate that is already
  lit. Tests omit debounce and see raw detections.
  `write_jsonl` is false while engaged during a disk stall.
  """
  signs = [] if (y is None and rgb is None and nv12 is None) else detector.detect(y, rgb=rgb, nv12=nv12)
  hud_signs = signs
  jsonl_signs = signs
  if debounce is not None:
    split = debounce.update_split(signs, now)
    hud_signs = split.hud
    jsonl_signs = split.confirmed
  written: list[dict] = []
  if gps_ok and write_jsonl:
    for sign in jsonl_signs:
      rec = make_record(now, lat, lon, bearing, sign.mph, sign.conf)
      if not jsonl_line_is_light(rec):
        continue
      if logger.write(rec):
        written.append(rec)
  return hud_signs, written


def process_observations(
  y,
  lat: float,
  lon: float,
  bearing: float | None,
  gps_ok: bool,
  detector: SpeedSignDetector,
  logger: JsonlLogger,
  now: float,
) -> list[dict]:
  """Detect + append. No GPS fix → no row (lat/lon would be junk)."""
  _signs, written = process_frame(y, lat, lon, bearing, gps_ok, detector, logger, now)
  return written


@dataclass
class InferOutcome:
  signs: list
  written: list[dict]
  infer_s: float
  error: str | None = None
  diag: dict | None = None


def format_refine_token(class_mph, hud_mph, ref_mph, ref_conf) -> str:
  """Unambiguous refine= token. Fail is `refine=- class=65`, never `-(65)`.

  Agree: `65:0.47`. Override: `50:0.43 class=65`. Parentheses around 65
  were misread as refine=(65) in Justin's journalctl paste.
  """
  if ref_mph is None:
    return f"- class={int(class_mph)}"
  token = f"{int(ref_mph)}:{float(ref_conf):.2f}"
  if int(ref_mph) == int(hud_mph) == int(class_mph):
    return token
  return f"{int(hud_mph)}:{float(ref_conf):.2f} class={int(class_mph)}"


def format_infer_diag(diag: dict | None, *, allow_detect: bool) -> str:
  """One-line on-car fields: backend, frame, letterbox, crop, sha, peak."""
  d = diag or {}
  crop = d.get("crop") or (0, 0, 0, 0)
  if isinstance(crop, (list, tuple)) and len(crop) == 4:
    crop_s = f"{int(crop[0])},{int(crop[1])} {int(crop[2])}x{int(crop[3])}"
  else:
    crop_s = str(crop)
  out = d.get("out_shape", ())
  if isinstance(out, (list, tuple)):
    out_s = "x".join(str(int(v)) for v in out) if out else "-"
  else:
    out_s = str(out)
  sl_conf = float(d.get("sl_peak_conf", 0.0) or 0.0)
  sl_name = d.get("sl_peak_name", "") or ""
  n_over = int(d.get("n_over", 0) or 0)
  # Always log sl_peak — Justin's R2-1 miss is n_over=0 with a weak speedLimit*.
  # When n_over>0 the global peak is often stop; sl_peak is the HUD-relevant head.
  sl_s = f" sl_peak={sl_conf:.2f}/{sl_name}"
  top3 = d.get("top3") or ()
  if top3:
    top_s = " top=" + ",".join(f"{n}:{c:.2f}" for n, c in list(top3)[:3])
  else:
    top_s = ""
  posted = d.get("posted") or ()
  if posted:
    posted_s = " cls=" + ",".join(f"{int(mph)}:{float(c):.2f}" for mph, c in posted)
  else:
    posted_s = ""
  refine = d.get("refine") or ()
  if refine:
    parts = []
    for row in refine:
      if not row or len(row) < 4:
        continue
      class_mph, hud_mph, ref_mph, ref_conf = row[0], row[1], row[2], row[3]
      parts.append(format_refine_token(class_mph, hud_mph, ref_mph, ref_conf))
    refine_s = " refine=" + ",".join(parts) if parts else ""
  else:
    refine_s = ""
  return (
    f"backend={d.get('backend', '?')} "
    + f"frame={int(d.get('frame_w', 0) or 0)}x{int(d.get('frame_h', 0) or 0)} "
    + f"letterbox={int(d.get('letterbox', 0) or 0)} crop={crop_s} "
    + f"onnx={d.get('weights_path', '')} sha={d.get('weights_sha', '')} "
    + f"allow={int(bool(allow_detect))} out={out_s} "
    + f"peak={float(d.get('peak_conf', 0.0) or 0.0):.2f}/{d.get('peak_name', '')} "
    + f"n_over={n_over}"
    + sl_s
    + f" n_over_sl={int(d.get('n_over_sl', 0) or 0)}"
    + top_s
    + posted_s
    + refine_s
    + " "
    + f"luma={float(d.get('luma_mean', 0.0) or 0.0):.0f}/{float(d.get('luma_std', 0.0) or 0.0):.0f} "
    + f"chroma={int(d.get('chroma', 0) or 0)} "
    + f"prep={float(d.get('prep_ms', 0.0) or 0.0):.0f} "
    + f"sess={float(d.get('sess_ms', 0.0) or 0.0):.0f}"
  )


def detect_skip_reason(
  *,
  connected: bool,
  onnx: bool,
  busy: bool,
  holdoff: bool,
  buf_empty: bool | None = None,
  parse_fail: bool = False,
) -> str:
  """Why YOLO did not start. Distinguishes infer-never-ran from raw=[]."""
  if not connected:
    return "vision-disconnected"
  if not onnx:
    return "no-onnx"
  if parse_fail:
    return "nv12-parse"
  if buf_empty:
    return "road-recv-empty"
  if busy:
    return "infer-busy"
  if holdoff:
    return "holdoff"
  return "ok"


class InferSlot:
  """At most one ONNX infer. The 20 Hz loop never joins on engage.

  Re-engage used to block speedsignd for 300–1500 ms on an in-flight YOLO
  (and leave a core hot) → Communication Issue Between Processes. pause()
  drops the result immediately and refuses new work. The leftover infer may
  still finish at nice 19; we do not start another and we do not wait.

  resume() only clears the start-gate. A generation counter keeps the
  abandoned infer from publishing after WAIT clears (that used to apply
  skip-on-overrun to a stale leftover and delay the first real manual detect).
  """

  def __init__(self):
    self._lock = threading.Lock()
    self._cancel = threading.Event()
    self._busy = False
    self._outcome: InferOutcome | None = None
    self._gen = 0

  @property
  def busy(self) -> bool:
    with self._lock:
      return self._busy

  def pause(self) -> bool:
    """Cancel in-flight work and drop results. True if an infer was busy."""
    self._cancel.set()
    with self._lock:
      was_busy = self._busy
      self._outcome = None
      self._gen += 1
      return was_busy

  def resume(self) -> None:
    self._cancel.clear()

  def take(self) -> InferOutcome | None:
    with self._lock:
      out = self._outcome
      self._outcome = None
      return out

  def start(self, fn) -> bool:
    """Run fn() in a daemon thread. fn returns (signs, written) or + diag."""
    with self._lock:
      if self._busy or self._cancel.is_set():
        return False
      self._busy = True
      self._outcome = None
      gen = self._gen

    def run():
      t0 = time.monotonic()
      signs: list = []
      written: list[dict] = []
      diag = None
      error = None
      try:
        if self._cancel.is_set():
          return
        result = fn()
        if isinstance(result, tuple) and len(result) >= 3:
          signs, written, diag = result[0], result[1], result[2]
        else:
          signs, written = result
      except Exception as e:
        error = f"{type(e).__name__}: {e}"
        signs, written = [], []
      finally:
        infer_s = time.monotonic() - t0
        with self._lock:
          if not self._cancel.is_set() and gen == self._gen:
            self._outcome = InferOutcome(
              list(signs), list(written), infer_s, error=error, diag=diag,
            )
          self._busy = False

    threading.Thread(target=run, name="speedsignd-onnx", daemon=True).start()
    return True


def detect_if_allowed(
  y,
  lat: float,
  lon: float,
  bearing: float | None,
  gps_ok: bool,
  detector: SpeedSignDetector,
  logger: JsonlLogger,
  now: float,
  *,
  controlling: bool,
  rgb=None,
  debounce: SignDebounce | None = None,
  nv12=None,
  disk_stalled: bool = False,
) -> tuple[list, list[dict]]:
  """ONNX while engaged or manual. Skip the JSONL write if engaged and disk-stalled."""
  if not should_run_onnx_detect(controlling):
    return [], []
  write_jsonl = not (bool(controlling) and bool(disk_stalled))
  return process_frame(
    y, lat, lon, bearing, gps_ok, detector, logger, now,
    rgb=rgb, debounce=debounce, nv12=nv12, write_jsonl=write_jsonl,
  )


def _connect_road_camera():
  from msgq.visionipc import VisionIpcClient, VisionStreamType
  client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
  return client


def live_sign_publish_fields(
  hold: LiveSignHold,
  signs,
  now_mono: float,
  weights_missing: bool,
  detect_paused: bool = False,
) -> tuple[bool, int, float, bool, bool]:
  """msg.valid, mph, conf, weights_missing, detect_paused for liveSpeedSignNAP.

  When ONNX is missing, keep msg.valid so the HUD can show NO WT — never a
  numpy-fallback mph. Controlling does not freeze the hold: mph stays 0
  until two different frames agree, then the plate shows that mph while
  engaged. Unknown cereal must not set detect_paused (no false WAIT).
  """
  if weights_missing:
    return True, 0, 0.0, True, False
  if detect_paused:
    return True, 0, 0.0, False, True
  live, mph, conf = hold.update(signs, now_mono)
  return live, mph if live else 0, conf if live else 0.0, False, False


def _publish_live(
  pm, hold: LiveSignHold, signs, now_mono: float, messaging,
  weights_missing: bool, detect_paused: bool = False,
) -> None:
  msg_valid, mph, conf, missing, paused = live_sign_publish_fields(
    hold, signs, now_mono, weights_missing, detect_paused,
  )
  msg = messaging.new_message(SERVICE_NAME)
  msg.valid = msg_valid
  apply_live_sign(
    getattr(msg, SERVICE_NAME),
    mph=mph, conf=conf, valid=bool(mph), weights_missing=missing, detect_paused=paused,
  )
  pm.send(SERVICE_NAME, msg)


def format_read_timing(
  *,
  read_interval_ms: float,
  infer_ms: float,
  engaged: bool,
  backoff: bool,
  reason: str,
  cpu_share: float | None = None,
  threads: int = 1,
  backend: str = "",
  os_threads: int | None = None,
) -> str:
  """One greppable line per finished read. `speedsignd timing`.

  `threads` is the pinned BLAS/OpenMP/OpenCV pool (1). `os_threads` is
  /proc/self/status Threads, which also counts the main loop, the infer
  worker, and VisionIPC. Route 0000017e logged the proc count in `threads`
  and showed 11.
  """
  share = "na" if cpu_share is None else f"{float(cpu_share):.3f}"
  os_s = "" if os_threads is None else f"os_threads={int(os_threads)} "
  return (
    "speedsignd timing "
    + f"read_interval_ms={read_interval_ms:.0f} "
    + f"infer_ms={infer_ms:.0f} "
    + f"cpu_share={share} "
    + f"threads={int(threads)} "
    + os_s
    + f"backend={backend or 'unset'} "
    + f"engaged={int(bool(engaged))} "
    + f"backoff={int(bool(backoff))} "
    + f"reason={reason or 'pace'}"
  )


def format_backoff_timing(
  *,
  extra_ms: float,
  infer_ms: float,
  engaged: bool,
  reason: str,
  backend: str = "",
) -> str:
  """Greppable back-off event. Still starts with `speedsignd timing`."""
  return (
    "speedsignd timing "
    + f"backoff=1 reason={reason or 'pressure'} "
    + f"extra_ms={extra_ms:.0f} "
    + f"infer_ms={infer_ms:.0f} "
    + f"engaged={int(bool(engaged))} "
    + f"backend={backend or 'unset'}"
  )


def _log_infer_timing(cloudlog, infer_ms: list[float], skip_count: int, hz: float, engaged: bool,
                      threads: int = 1) -> None:
  n = len(infer_ms)
  mean_ms = sum(infer_ms) / n if n else 0.0
  max_ms = max(infer_ms) if n else 0.0
  os_n = read_proc_thread_count()
  os_s = "na" if os_n is None else str(int(os_n))
  cloudlog.info(
    "speedsignd timing hz=%.2f infer_ms mean=%.1f max=%.1f n=%d skip=%d engaged=%d threads=%d os_threads=%s",
    hz, mean_ms, max_ms, n, skip_count, int(bool(engaged)), int(threads), os_s,
  )


@dataclass
class PendingSecondLook:
  bbox: tuple[int, int, int, int]
  frames_left: int
  deadline: float


def sign_like_bbox(signs, diag) -> tuple[int, int, int, int] | None:
  """Box for an immediate OCR follow-up. None when this frame already agrees."""
  for s in signs or []:
    _pending, pair = frame_reads(s)
    if pair is not None:
      return None
  for s in signs or []:
    pending, _pair = frame_reads(s)
    conf = float(getattr(s, "conf", 0.0) or 0.0)
    if pending or conf >= SIGN_LIKE_CONF:
      box = getattr(s, "bbox", None)
      if box is not None and len(box) == 4:
        return tuple(int(v) for v in box)
  raw = (diag or {}).get("sign_like_bbox") if diag else None
  if raw is not None and len(raw) == 4:
    return tuple(int(v) for v in raw)
  return None


def tight_rect(bbox, frame_w: int, frame_h: int, pad_frac: float = 0.50) -> tuple[int, int, int, int]:
  x, y, w, h = (int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3]))
  pad = int(max(w, h) * pad_frac)
  x0 = max(0, x - pad)
  y0 = max(0, y - pad)
  x1 = min(int(frame_w), x + w + pad)
  y1 = min(int(frame_h), y + h + pad)
  x0 -= x0 % 2
  y0 -= y0 % 2
  return x0, y0, max(2, x1 - x0), max(2, y1 - y0)


def second_look_sign(buf, bbox) -> SpeedSign | None:
  """OCR a tight full-res crop. Not a second YOLO."""
  width = int(getattr(buf, "width", 0) or 0)
  height = int(getattr(buf, "height", 0) or 0)
  if width < 2 or height < 2:
    return None
  crop = copy_nv12_rect(buf, tight_rect(bbox, width, height))
  if crop is None:
    return None
  mph, conf = ocr_tight_y(crop.y)
  if mph is None or float(conf) < REFINE_OVERRIDE_CONF:
    return None
  return SpeedSign(
    mph=int(mph), conf=float(conf), bbox=tuple(int(v) for v in bbox),
    refine_mph=int(mph), refine_conf=float(conf),
  )


def observe_model(watch: ModelWatch, sm, now: float | None = None) -> None:
  try:
    if not sm.updated["modelV2"]:
      return
    msg = sm["modelV2"]
  except Exception:
    return
  try:
    watch.observe(
      getattr(msg, "frameId", None),
      getattr(msg, "frameDropPerc", None),
      getattr(msg, "modelExecutionTime", None),
      now=time.monotonic() if now is None else now,
    )
  except Exception:
    return


def main():
  from cereal import messaging
  from openpilot.common.realtime import Ratekeeper
  from openpilot.common.swaglog import cloudlog
  from openpilot.selfdrive.mapd.gps_fix import gps_sample_from_sm

  # Child of manager: shrink the BLAS pool the parent already started.
  limit_infer_threads(1)
  sched_name = yield_to_modeld()

  hz = parse_detect_hz(os.environ.get(HZ_ENV))
  period_s = 1.0 / hz
  budget_s = INFER_BUDGET_MS / 1000.0
  threads = parse_infer_threads(os.environ.get(THREADS_ENV), THREADS_DEFAULT)
  cap_ms = parse_infer_cap_ms(os.environ.get(CAP_MS_ENV), CAP_MS_DEFAULT)
  cap_s = cap_ms / 1000.0 if cap_ms > 0 else 0.0

  log_path = default_log_path()
  onnx_path = default_onnx_path()
  detector = SpeedSignDetector(onnx_path=onnx_path)
  logger = JsonlLogger(log_path)
  hold = LiveSignHold()
  debounce = SignDebounce()
  # Two-stage reader: strip proposer + tracker + one sharp read per sign.
  # Missing proposer file -> legacy fixed tiles (unchanged behavior).
  proposer = two_stage.Proposer(default_proposer_path())
  tracker = two_stage.SignTracker()
  two_period_s = 1.0 / two_stage.PROPOSER_HZ
  last_proposer_try = time.monotonic()
  last_stopped_log = 0.0
  backend = detector.backend_name()
  weights_sha = detector.weights_sha_short()
  if detector.onnx is not None and detector.onnx.session is not None:
    backend = assert_cpu_backend(detector.onnx.session)
  runtimes = describe_infer_runtimes()
  os_threads = read_proc_thread_count()
  from openpilot.selfdrive.speedsignd.install import runtime_missing, runtime_status_summary
  runtime_label = runtime_status_summary()
  cloudlog.info(
    "speedsignd cpu-only asserted backend=%s runtimes=%s dev=%s qcom=%s gpu=%s cpu_count=%s",
    backend, runtimes, os.environ.get("DEV", ""), os.environ.get("QCOM", ""),
    os.environ.get("GPU", ""), os.environ.get("CPU_COUNT", ""),
  )
  cloudlog.info(
    "speedsignd starting log=%s backend=%s onnx=%s sha=%s sm_hz=%.1f detect_hz=%.2f "
    + "budget_ms=%.0f cap_ms=%.0f threads=%d os_threads=%s cores=%s nice=%d sched=%s "
    + "detect_while_engaged=1 crop_rgb=1 duty=%.2f runtime=%s",
    log_path, backend, onnx_path, weights_sha, SM_HZ, hz, INFER_BUDGET_MS, cap_ms, threads,
    "na" if os_threads is None else str(int(os_threads)),
    ",".join(str(c) for c in SPEEDSIGND_CORES), SPEEDSIGND_NICE, sched_name, CPU_BUDGET,
    runtime_label,
  )
  # tinygrad-clang already logged its own fallback inside _onnx_session.
  if runtime_missing() and backend != "tinygrad-clang":
    cloudlog.warning(
      "speedsignd runtime missing — Settings → NAP → Install weights also installs "
      + "onnxruntime into /data/media/0/nap/pylib (not the system venv). "
      + "The plate shows NO RT. CLANG will not run while engaged.",
    )
  if detector.onnx is None:
    cloudlog.warning(
      "speedsignd: no ONNX at %s — numpy fallback will not see real roadside signs. "
      + "HUD will show NO WT. Settings → NAP → Install weights, or: "
      + "python -m scripts.nap.install_speed_sign_weights",
      onnx_path,
    )

  cloudlog.info(
    "speedsignd two-stage proposer=%s path=%s err=%s hz=%.1f conf=%.2f strip=%s",
    int(proposer.ok), proposer.path, proposer.error, two_stage.PROPOSER_HZ,
    two_stage.PROPOSER_CONF, two_stage.STRIP_RECT,
  )
  sm = messaging.SubMaster(
    ["gpsLocationExternal", "gpsLocation", "selfdriveState", "modelV2"],
    frequency=SM_HZ,
  )
  pm = messaging.PubMaster([SERVICE_NAME])
  rk = Ratekeeper(SM_HZ, print_delay_threshold=None)
  client = None
  last_connect = 0.0
  last_onnx_try = time.monotonic()
  next_detect = 0.0
  infer_ms: list[float] = []
  skip_count = 0
  last_timing_log = time.monotonic()
  started_at = time.monotonic()
  last_allow: bool | None = None
  last_paused: bool | None = None
  last_unknown_warn = 0.0
  last_empty_frame_log = 0.0
  last_empty_raw_log = 0.0
  last_infer_done = 0.0
  last_gate_log = 0.0
  last_skip_reason = "init"
  in_holdoff = False
  slot = InferSlot()
  last_read_start = 0.0
  pending_interval_ms = 0.0
  pending_backoff = False
  pending_reason = "pace"
  scheduled_backoff = False
  scheduled_reason = "pace"
  model_watch = ModelWatch()
  cpu_meter = CpuShareMeter()
  second_look: PendingSecondLook | None = None
  model_extended = False
  last_infer_s = period_s
  window_i = 0
  pending_window = 0
  burst_left = 0
  burst_window = 0
  burst_cluster_s = 0.0
  duty_spans: list[tuple[float, float]] = []

  while True:
    sm.update(0)
    observe_model(model_watch, sm)
    now_mono = time.monotonic()
    sample = engagement_from_sm(sm, now=now_mono, started_at=started_at)
    # Refresh every tick, before any infer starts. The first frame of a boot
    # that is already engaged sees controlling=True and does not compile.
    # Unknown cereal is not permission either. The worker re-reads this flag
    # inside run(), so a flip before realize still aborts.
    detector.allow_compile(compile_allowed(
      known=bool(sample.known),
      controlling=bool(sample.controlling),
      ready=detector.compile_ready(),
    ))
    live_model = model_watch.reason()
    if live_model and not model_extended:
      hard_now = model_watch.hard_guard()
      extra_now = model_guard_rest_s(last_infer_s, period_s, live_model, hard_now)
      # A frame skip or modeld > 50 ms pauses even before the first infer.
      # A frameDropPerc backoff still only extends a schedule that already exists.
      if next_detect > 0.0 or hard_now:
        next_detect = max(next_detect, now_mono + extra_now)
      # A model pause is not a burst. Do not dump the extra reads when it ends.
      burst_left = 0
      burst_cluster_s = 0.0
      model_extended = True
      cloudlog.warning(
        "%s",
        format_backoff_timing(
          extra_ms=extra_now * 1000.0,
          infer_ms=last_infer_s * 1000.0,
          engaged=sample.controlling,
          reason=live_model,
          backend=detector.backend_name(),
        ),
      )
    elif not live_model:
      model_extended = False
    if not sample.allow_detect:
      abandoned = slot.pause()
      if abandoned:
        cloudlog.info("speedsignd abandon in-flight ONNX (%s)", engagement_log_fields(sample))
    else:
      if should_reset_detect_after_wait(last_allow, sample.allow_detect):
        next_detect = 0.0
        in_holdoff = False
      slot.resume()
      # Engaged + not yet compiled: drop a queued result. The worker also
      # re-reads allow_compile before realize, so boot-already-engaged never
      # enters clang. A realize that has already entered clang cannot be
      # killed; that only happens after a known-disengaged start.
      if sample.controlling and slot.busy and not detector.compile_ready():
        slot.pause()
    if last_allow != sample.allow_detect or last_paused != sample.detect_paused:
      cloudlog.info(
        "speedsignd detect %s (%s)",
        "paused" if not sample.allow_detect else "running",
        engagement_log_fields(sample),
      )
      last_allow = sample.allow_detect
      last_paused = sample.detect_paused
    if sample.unknown_after_grace and now_mono - last_unknown_warn >= INFER_LOG_PERIOD_S:
      cloudlog.warning(
        "speedsignd selfdriveState unread after %.1fs; allowing throttled detect (not WAIT) %s",
        now_mono - started_at, engagement_log_fields(sample),
      )
      last_unknown_warn = now_mono
    if now_mono - last_timing_log >= INFER_LOG_PERIOD_S:
      _log_infer_timing(cloudlog, infer_ms, skip_count, hz, sample.controlling, threads)
      infer_ms = []
      skip_count = 0
      last_timing_log = now_mono
    # Retry the weights read at nice 19, including while engaged. Install
    # itself stays an offroad settings action and does not run here.
    # Session create can compile. Do that only while not engaged.
    if (
      sample.allow_detect and not sample.controlling and detector.onnx is None
      and now_mono - last_onnx_try >= ONNX_RETRY_S
    ):
      last_onnx_try = now_mono
      if detector.try_reload():
        cloudlog.info(
          "speedsignd: ONNX loaded after retry backend=%s onnx=%s sha=%s",
          detector.backend_name(), onnx_path, detector.weights_sha_short(),
        )
    signs: list = []
    if sample.allow_detect:
      outcome = slot.take()
      if outcome is not None:
        signs = outcome.signs
        infer_ms.append(outcome.infer_s * 1000.0)
        share = cpu_meter.sample(now_mono, read_proc_stat_jiffies())
        os_threads = read_proc_thread_count()
        cloudlog.info(
          "%s",
          format_read_timing(
            read_interval_ms=pending_interval_ms,
            infer_ms=outcome.infer_s * 1000.0,
            engaged=sample.controlling,
            backoff=pending_backoff,
            reason=pending_reason,
            cpu_share=share,
            threads=threads,
            os_threads=os_threads,
            backend=detector.backend_name(),
          ),
        )
        pressure = collect_host_pressure(sm)
        model_reason = model_watch.reason()
        hard = model_watch.hard_guard()
        model_watch.consume()
        reason = join_reasons(
          pressure_reason(pressure),
          model_reason,
          "cpu-budget" if cpu_over_budget(share) else "",
        )
        extra_s = model_guard_rest_s(outcome.infer_s, period_s, reason, hard)
        scheduled_backoff = extra_s > 0.0
        scheduled_reason = reason or "pace"
        if scheduled_backoff:
          cloudlog.warning(
            "%s",
            format_backoff_timing(
              extra_ms=extra_s * 1000.0,
              infer_ms=outcome.infer_s * 1000.0,
              engaged=sample.controlling,
              reason=reason,
              backend=detector.backend_name(),
            ),
          )
        raw = list(getattr(debounce, "last_raw", []))
        diag = outcome.diag if outcome.diag is not None else detector.diag_dict()
        two = isinstance(diag, dict) and bool(diag.get("two_stage"))
        if two:
          t_frame = float(diag.get("t_frame", now_mono))
          tracker.update(diag.get("props") or [], t_frame)
          read_tid = diag.get("read_tid")
          if read_tid is not None:
            for tr in tracker.tracks:
              if tr.tid == read_tid:
                tracker.mark_read(tr, got_mph=bool(signs), neg=two_stage.neg_read(diag.get("read_diag")))
            if signs and diag.get("read_jpg_meta") is not None:
              top = max(signs, key=lambda s: float(s.conf))
              meta = dict(diag["read_jpg_meta"])
              meta.update(
                mph=int(top.mph), conf=round(float(top.conf), 3),
                refine_mph=None if getattr(top, "refine_mph", None) is None else int(top.refine_mph),
                refine_conf=round(float(getattr(top, "refine_conf", 0.0) or 0.0), 3),
                **{"class": (diag.get("read_diag") or {}).get("peak_name")},
              )
              two_stage.write_pending(meta, diag.get("read_jpg"))
        (
          next_mono, burst_left, burst_window, window_i, burst_cluster_s, bursting_next,
        ) = schedule_after_read(
          now=now_mono,
          infer_s=outcome.infer_s,
          period_s=two_period_s if two else period_s,
          budget_s=budget_s,
          cap_s=cap_s,
          backoff_s=extra_s,
          sign_like=False if two else sign_burst_trigger(diag, raw),
          window_used=pending_window,
          window_count=1 if two else 2,
          burst_left=burst_left,
          burst_window=burst_window,
          burst_cluster_s=burst_cluster_s,
          spans=duty_spans,
        )
        duty_spans.append((now_mono, outcome.infer_s))
        duty_spans = [(end, dur) for end, dur in duty_spans if now_mono - float(end) <= DUTY_WINDOW_S]
        next_detect = max(next_detect, next_mono)
        if bursting_next and not scheduled_backoff:
          scheduled_reason = "burst"
        err = f" err={outcome.error}" if outcome.error else ""
        if outcome.error and not diag.get("error"):
          diag = dict(diag)
          diag["error"] = outcome.error
        if two:
          err = err + (
            f" two=props:{len(diag.get('props') or [])}/{float(diag.get('prop_ms') or 0.0):.0f}ms"
            + f" read={diag.get('read_tid')} tracks={len(tracker.tracks)}"
          )
        (cloudlog.debug if (two and diag.get("read_tid") is None) else cloudlog.info)(
          "speedsignd infer %.0fms %s raw=%s hud=%s jsonl=%s%s",
          outcome.infer_s * 1000.0,
          format_infer_diag(diag, allow_detect=sample.allow_detect),
          [(int(s.mph), round(float(s.conf), 2)) for s in raw],
          [(int(s.mph), round(float(s.conf), 2)) for s in signs],
          [w.get("mph") for w in outcome.written],
          err,
        )
        last_infer_done = now_mono
        last_infer_s = outcome.infer_s
        last_skip_reason = "ok"
        # A burst re-reads the same tile with YOLO. The OCR second look would
        # spend those frames and push the confirm past the sign.
        if bursting_next or two:
          second_look = None
        else:
          box = sign_like_bbox(outcome.signs, diag)
          if box is not None:
            second_look = PendingSecondLook(
              bbox=box,
              frames_left=SECOND_LOOK_FRAMES,
              deadline=now_mono + SECOND_LOOK_DEADLINE_S,
            )
          else:
            second_look = None
        if not raw and now_mono - last_empty_raw_log >= INFER_LOG_PERIOD_S:
          cloudlog.info(
            "speedsignd empty-raw %s",
            format_infer_diag(diag, allow_detect=sample.allow_detect),
          )
          last_empty_raw_log = now_mono
    if sample.allow_detect and now_mono - last_gate_log >= INFER_LOG_PERIOD_S:
      if last_infer_done == 0.0 or now_mono - last_infer_done >= INFER_LOG_PERIOD_S:
        connected = client is not None and bool(getattr(client, "is_connected", lambda: False)())
        cloudlog.info(
          "speedsignd waiting-infer reason=%s connected=%s busy=%s holdoff=%s onnx=%s allow=1",
          last_skip_reason, connected, slot.busy, in_holdoff, detector.onnx is not None,
        )
      last_gate_log = now_mono
    if client is None or not client.is_connected():
      last_skip_reason = detect_skip_reason(
        connected=False, onnx=detector.onnx is not None, busy=slot.busy, holdoff=in_holdoff,
      )
      if now_mono - last_connect >= 0.5:
        last_connect = now_mono
        try:
          if client is None:
            client = _connect_road_camera()
          client.connect(False)
        except Exception:
          client = None
    elif not sample.allow_detect:
      # Do not idle the ROAD client for the whole engage — first manual recv
      # after WAIT used to timeout / return nothing while modeld stayed healthy.
      drain_vision_latest(client)
    elif (
      second_look is not None and sample.allow_detect and not slot.busy
      and now_mono <= second_look.deadline
    ):
      buf = client.recv(timeout_ms=VISION_TIMEOUT_MS)
      looked = second_look_sign(buf, second_look.bbox) if buf is not None else None
      second_look.frames_left -= 1
      if looked is not None:
        signs = list(signs) + [looked]
        now_look = time.monotonic()
        split = debounce.update_split([looked], now_look)
        cloudlog.info(
          "speedsignd second-look mph=%d conf=%.2f",
          int(looked.mph), float(looked.conf),
        )
        if split.confirmed:
          lat, lon, bearing, gps_ok = gps_sample_from_sm(sm, now=now_look)
          stalled = bool(sample.controlling) and disk_stalled(collect_host_pressure(sm))
          if gps_ok and not stalled:
            for sign in split.confirmed:
              rec = make_record(now_look, lat, lon, bearing, sign.mph, sign.conf)
              if jsonl_line_is_light(rec):
                logger.write(rec)
      if second_look.frames_left <= 0 or now_mono > second_look.deadline:
        second_look = None
    elif sample.allow_detect and now_mono >= next_detect and not slot.busy:
      in_holdoff = False
      buf = client.recv(timeout_ms=VISION_TIMEOUT_MS)
      # Engage can happen during the vision wait — re-read before ONNX.
      sm.update(0)
      observe_model(model_watch, sm)
      sample = engagement_from_sm(sm, now=time.monotonic(), started_at=started_at)
      if not sample.allow_detect:
        slot.pause()
      else:
        which = window_i
        speed_ms = two_stage.speed_from_sm(sm)
        if (
          proposer.ok is False and not sample.controlling
          and now_mono - last_proposer_try >= ONNX_RETRY_S
        ):
          last_proposer_try = now_mono
          if proposer.load():
            cloudlog.info("speedsignd two-stage proposer loaded after retry")
        use_two = proposer.ok and detector.onnx is not None and buf is not None
        stopped_skip = False
        if use_two and not two_stage.proposals_allowed(speed_ms):
          # Stopped: no proposer, no read. Holds / plate unchanged.
          next_detect = now_mono + two_period_s
          last_skip_reason = "stopped"
          if now_mono - last_stopped_log >= INFER_LOG_PERIOD_S:
            cloudlog.info("speedsignd two-stage skip stopped v=%.1f", float(speed_ms or 0.0))
            last_stopped_log = now_mono
          stopped_skip = True
          use_two = False
          nv12 = None
          two_job = None
        elif use_two:
          strip = copy_nv12_rect(buf, two_stage.STRIP_RECT)
          t_frame = time.monotonic()
          cand = tracker.capture_candidate(t_frame)
          cap = None
          cap_rect = None
          if cand is not None:
            cap_rect = two_stage.capture_rect(
              cand.predicted(t_frame), int(getattr(buf, "width", 1928) or 1928),
              int(getattr(buf, "height", 1208) or 1208),
            )
            cap = copy_nv12_rect(buf, cap_rect)
          nv12 = cap if cap is not None else strip
          two_job = (strip, cap, None if cand is None else cand.tid, cap_rect, t_frame)
        else:
          nv12 = copy_nv12_detect_crop(buf, which) if buf is not None else None
          two_job = None
        if stopped_skip:
          pass
        elif buf is None:
          last_skip_reason = detect_skip_reason(
            connected=True, onnx=detector.onnx is not None, busy=False, holdoff=False,
            buf_empty=True,
          )
          now_empty = time.monotonic()
          if now_empty - last_empty_frame_log >= INFER_LOG_PERIOD_S:
            cloudlog.info("speedsignd ROAD recv empty (timeout_ms=%d)", VISION_TIMEOUT_MS)
            last_empty_frame_log = now_empty
        elif nv12 is None:
          last_skip_reason = detect_skip_reason(
            connected=True, onnx=detector.onnx is not None, busy=False, holdoff=False,
            parse_fail=True,
          )
          now_empty = time.monotonic()
          if now_empty - last_empty_frame_log >= INFER_LOG_PERIOD_S:
            stride = getattr(buf, "stride", 0)
            uv_off = getattr(buf, "uv_offset", 0)
            cloudlog.info(
              "speedsignd NV12 parse failed w=%s h=%s stride=%s uv_offset=%s",
              getattr(buf, "width", None), getattr(buf, "height", None), stride, uv_off,
            )
            last_empty_frame_log = now_empty
        lat, lon, bearing, gps_ok = gps_sample_from_sm(sm, now=time.monotonic())
        # On-road: do not run numpy-mutcd when weights are missing (no fake mph / JSONL).
        have_frame = detector.onnx is not None and nv12 is not None
        if have_frame:
          pressure_now = collect_host_pressure(sm)
          stalled = bool(sample.controlling) and disk_stalled(pressure_now)
          engaged_now = bool(sample.controlling)

          ready = detector.compile_ready()
          # Compile only when this read is known-disengaged. Unknown cereal
          # is not that. A warm cache (ready) runs while engaged, on CPU.
          permitted = compile_allowed(
            known=bool(sample.known), controlling=engaged_now, ready=ready,
          )
          detector.allow_compile(permitted)

          def _run2(job=two_job, lat=lat, lon=lon, bearing=bearing, gps_ok=gps_ok,
                    engaged_now=engaged_now, stalled=stalled):
            if not detector.compile_allowed_now():
              raise RuntimeError("compile-blocked while driving")
            assert job is not None
            strip_c, cap_c, tid, rect, t_fr = job
            t0 = time.monotonic()
            props = proposer.propose(strip_c)
            prop_ms = (time.monotonic() - t0) * 1000.0
            signs_r: list = []
            written_r: list = []
            read_diag = None
            jpg = None
            if cap_c is not None:
              signs_r, written_r = detect_if_allowed(
                cap_c.y, lat, lon, bearing, gps_ok, detector, logger, time.monotonic(),
                controlling=engaged_now, rgb=None, debounce=debounce, nv12=cap_c,
                disk_stalled=stalled,
              )
              read_diag = detector.diag_dict()
              if signs_r and not (engaged_now and stalled):
                jpg = two_stage.crop_jpeg_bytes(cap_c)
            d = dict(read_diag or {})
            d.update(
              two_stage=1, t_frame=t_fr, props=props, prop_ms=round(prop_ms, 1),
              read_tid=tid, read_diag=read_diag, read_jpg=jpg,
              read_jpg_meta=None if cap_c is None else dict(
                t=round(time.time(), 3),  # noqa: TID251
                track_id=tid, crop=list(rect) if rect else None,
                lat=lat if gps_ok else None, lon=lon if gps_ok else None,
                bearing=bearing if gps_ok else None, gps_ok=bool(gps_ok),
              ),
            )
            return signs_r, written_r, d

          def _run(nv12=nv12, lat=lat, lon=lon, bearing=bearing, gps_ok=gps_ok,
                   engaged_now=engaged_now, stalled=stalled):
            # Live flag, not the value captured when this read was scheduled.
            if not detector.compile_allowed_now():
              raise RuntimeError("compile-blocked while driving")
            signs_w = detect_if_allowed(
              nv12.y, lat, lon, bearing, gps_ok, detector, logger, time.monotonic(),
              controlling=engaged_now, rgb=None, debounce=debounce, nv12=nv12,
              disk_stalled=stalled,
            )
            return signs_w[0], signs_w[1], detector.diag_dict()

          if not permitted:
            last_skip_reason = "compile-wait"
            next_detect = max(next_detect, time.monotonic() + 1.0)
            if now_mono - last_gate_log >= INFER_LOG_PERIOD_S:
              cloudlog.warning(
                "%s",
                format_backoff_timing(
                  extra_ms=1000.0, infer_ms=0.0, engaged=True,
                  reason="compile-wait", backend=detector.backend_name(),
                ),
              )
              last_gate_log = now_mono
          elif slot.start(_run2 if two_job is not None else _run):
            pending_window = which
            now_start = time.monotonic()
            pending_interval_ms = 0.0 if last_read_start <= 0.0 else (now_start - last_read_start) * 1000.0
            pending_backoff = scheduled_backoff
            pending_reason = scheduled_reason
            last_read_start = now_start
            # Floor at one period. The outcome handler then applies the
            # 3× idle duty cycle, which is longer whenever the infer is.
            next_detect = now_start + (two_period_s if two_job is not None else period_s)
            last_skip_reason = "ok"
          else:
            last_skip_reason = detect_skip_reason(
              connected=True, onnx=True, busy=True, holdoff=False,
            )
        elif detector.onnx is None:
          last_skip_reason = detect_skip_reason(
            connected=True, onnx=False, busy=slot.busy, holdoff=False,
          )
    elif sample.allow_detect and (slot.busy or (next_detect > 0 and now_mono < next_detect)):
      last_skip_reason = detect_skip_reason(
        connected=True, onnx=detector.onnx is not None, busy=slot.busy, holdoff=True,
      )
      if not in_holdoff:
        skip_count += 1
        in_holdoff = True
      if slot.busy:
        try:
          os.sched_yield()
        except Exception:
          pass
    _publish_live(
      pm, hold, signs, now_mono, messaging,
      detector.weights_missing(), detect_paused=sample.detect_paused,
    )
    rk.keep_time()
    reset_ratekeeper_if_behind(rk, time.monotonic())


if __name__ == "__main__":
  main()
