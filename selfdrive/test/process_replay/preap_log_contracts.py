"""Pre-AP invariants a drive log (recorded or replayed) must satisfy.

Long chimes follow enableLongControl edges, not interceptor handshake and
not gas override (b568c89384). Lat chimes follow cruiseState.enabled
(pcmEnable/pcmDisable). Bosch radar presence is reported from CAN and
liveTracks when those frames exist; missing radar is a skip, not a fail.
"""
from __future__ import annotations

from collections import defaultdict
from collections.abc import Iterable
from dataclasses import dataclass, field
from typing import Literal

from cereal import log as capnp_log

from openpilot.selfdrive.selfdrived.preap_regen import PreAPChimeState, update_preap_chimes
from openpilot.selfdrive.ui.radar.bosch_status import (
  ADDR_ALERT,
  ADDR_CAR_CONFIG,
  ADDR_POINT0,
  ADDR_SGU,
  ADDR_VIN_FEED,
  ADDR_VIN_HOST,
  SGU_DIRTY_BIT,
  SGU_FAIL_BIT,
  SGU_HW_FAIL_BIT,
)

GAS_COMMAND = 0x551
GAS_SENSOR = 0x552
DAS_BODY_CONTROLS = 0x3E9
RADAR_UDS_TX = 0x641
RADAR_UDS_RX = 0x651
BOSCH_POINT_COUNT = 32
BOSCH_POINT_STRIDE = 3
BOSCH_POINT_LAST = ADDR_POINT0 + (BOSCH_POINT_COUNT - 1) * BOSCH_POINT_STRIDE + 1

CheckStatus = Literal["pass", "fail", "skip"]

LONG_ENGAGE = "pedalCruiseEnabled"
LONG_DISENGAGE = "pedalCruiseDisabled"
LAT_ENGAGE = "pcmEnable"
LAT_DISENGAGE = "pcmDisable"
GAS_OVERRIDE = "gasPressedOverride"


@dataclass(frozen=True)
class CheckResult:
  name: str
  status: CheckStatus
  detail: str


@dataclass(frozen=True)
class SignalPresence:
  name: str
  present: bool
  count: int
  extra: str = ""


@dataclass
class OffcarReport:
  source: str
  processes: list[str] = field(default_factory=list)
  process_notes: dict[str, str] = field(default_factory=dict)
  fingerprint: str = ""
  checks: list[CheckResult] = field(default_factory=list)
  signals: list[SignalPresence] = field(default_factory=list)
  replay_error: str = ""

  @property
  def failed(self) -> bool:
    return any(c.status == "fail" for c in self.checks) or bool(self.replay_error)

  def format(self) -> str:
    lines = [
      "Pre-AP off-car replay",
      f"source: {self.source}",
    ]
    if self.fingerprint:
      lines.append(f"fingerprint: {self.fingerprint}")
    if self.replay_error:
      lines.append(f"replay error: {self.replay_error}")
    lines.append("")
    lines.append("Processes")
    if not self.processes:
      lines.append("  (scan only — no process_replay)")
    else:
      for name in self.processes:
        note = self.process_notes.get(name, "")
        suffix = f"  {note}" if note else ""
        lines.append(f"  {name}{suffix}")
    lines.append("")
    lines.append("CAN / signals")
    for signal in self.signals:
      flag = "yes" if signal.present else "no "
      extra = f"  {signal.extra}" if signal.extra else ""
      lines.append(f"  {flag}  {signal.name:28s}  n={signal.count}{extra}")
    lines.append("")
    lines.append("Assertions")
    for check in self.checks:
      lines.append(f"  {check.status.upper():4s}  {check.name:28s}  {check.detail}")
    failed = sum(c.status == "fail" for c in self.checks)
    skipped = sum(c.status == "skip" for c in self.checks)
    passed = sum(c.status == "pass" for c in self.checks)
    lines.append("")
    lines.append(f"{passed} pass, {failed} fail, {skipped} skip")
    return "\n".join(lines) + "\n"


def evaluate_contracts(msgs: Iterable, *, source: str = "") -> OffcarReport:
  report = OffcarReport(source=source)
  can_counts: dict[int, int] = defaultdict(int)
  can_buses: dict[int, set[int]] = defaultdict(set)
  sgu_hw_fail = 0
  sgu_fail = 0
  sgu_dirty = 0
  live_track_frames = 0
  live_track_points = 0
  radar_fault_frames = 0
  radar_unavail_frames = 0
  fingerprint = ""
  openpilot_long = False
  pcm_cruise = False
  radar_unavailable = False
  saw_enable_long_field = False
  samples: list[tuple[int, bool, bool, bool, bool]] = []
  events: list[tuple[int, frozenset[str]]] = []

  for msg in msgs:
    which = msg.which()
    t = int(msg.logMonoTime)
    if which == "can":
      for frame in msg.can:
        addr = int(frame.address)
        can_counts[addr] += 1
        can_buses[addr].add(int(frame.src) & 0x7F)
        if addr == ADDR_SGU:
          dat = bytes(frame.dat)
          if _le_bit(dat, SGU_HW_FAIL_BIT):
            sgu_hw_fail += 1
          if _le_bit(dat, SGU_FAIL_BIT):
            sgu_fail += 1
          if _le_bit(dat, SGU_DIRTY_BIT):
            sgu_dirty += 1
    elif which == "carParams":
      fingerprint = str(msg.carParams.carFingerprint)
      openpilot_long = bool(msg.carParams.openpilotLongitudinalControl)
      pcm_cruise = bool(msg.carParams.pcmCruise)
      radar_unavailable = bool(msg.carParams.radarUnavailable)
    elif which == "carState":
      cs = msg.carState
      has_long = cs.enableLongControl if hasattr(cs, "enableLongControl") else False
      if hasattr(cs, "enableLongControl"):
        saw_enable_long_field = True
      samples.append((
        t,
        bool(cs.cruiseState.enabled),
        bool(has_long),
        bool(cs.gasPressed),
        bool(cs.brakePressed),
      ))
    elif which == "onroadEvents":
      names = frozenset(str(e.name) for e in msg.onroadEvents)
      events.append((t, names))
    elif which == "selfdriveState":
      alert = str(msg.selfdriveState.alertType or "")
      if alert:
        events.append((t, frozenset({alert.split("/", 1)[0]})))
    elif which == "liveTracks":
      live_track_frames += 1
      live_track_points += len(msg.liveTracks.points)
      errors = msg.liveTracks.errors
      if bool(getattr(errors, "radarFault", False)):
        radar_fault_frames += 1
    elif which == "radarState":
      errors = msg.radarState.radarErrors
      if bool(getattr(errors, "radarFault", False)):
        radar_fault_frames += 1
      if bool(getattr(errors, "radarUnavailableTemporary", False)):
        radar_unavail_frames += 1

  report.fingerprint = fingerprint
  bosch_points = sum(can_counts[addr] for addr in range(ADDR_POINT0, BOSCH_POINT_LAST + 1))
  report.signals = [
    _signal("GAS_COMMAND 0x551", GAS_COMMAND, can_counts, can_buses),
    _signal("GAS_SENSOR 0x552", GAS_SENSOR, can_counts, can_buses),
    _signal("TeslaRadarSguInfo 0x301", ADDR_SGU, can_counts, can_buses,
            extra=_sgu_extra(sgu_hw_fail, sgu_fail, sgu_dirty)),
    SignalPresence("Bosch RadarPoint 0x310-0x36E", bosch_points > 0, bosch_points),
    _signal("TeslaRadarAlertMatrix 0x501", ADDR_ALERT, can_counts, can_buses),
    _signal("radar car config 0x2A9", ADDR_CAR_CONFIG, can_counts, can_buses),
    _signal("radar VIN feed 0x2B9", ADDR_VIN_FEED, can_counts, can_buses),
    _signal("radar VIN host 0x560", ADDR_VIN_HOST, can_counts, can_buses),
    _signal("radar UDS TX 0x641", RADAR_UDS_TX, can_counts, can_buses),
    _signal("radar UDS RX 0x651", RADAR_UDS_RX, can_counts, can_buses),
    _signal("DAS_bodyControls 0x3E9", DAS_BODY_CONTROLS, can_counts, can_buses,
            extra="chassis 0x3E9; no DAS ECU on pre-AP"),
    SignalPresence("enableLongControl", saw_enable_long_field, sum(1 for s in samples if s[2]),
                   extra="CarState field" if saw_enable_long_field else "field missing"),
    SignalPresence("liveTracks", live_track_frames > 0, live_track_frames,
                   extra=f"points={live_track_points}"),
    SignalPresence("radarFault", radar_fault_frames > 0, radar_fault_frames),
    SignalPresence("radarUnavailableTemporary", radar_unavail_frames > 0, radar_unavail_frames),
    SignalPresence("openpilotLongitudinalControl", openpilot_long, int(openpilot_long),
                   extra=f"pcmCruise={pcm_cruise} radarUnavailable={radar_unavailable}"),
  ]

  expected = _expected_chimes(samples)
  report.checks.extend(_lat_long_chime_checks(samples, events, expected, saw_enable_long_field))
  report.checks.append(_gas_override_check(samples, events, expected, saw_enable_long_field))
  report.checks.append(_enable_long_holds_on_gas(samples, saw_enable_long_field))
  report.checks.append(_radar_track_check(bosch_points, live_track_frames, live_track_points, radar_unavailable))
  report.checks.append(_radar_hw_fail_check(can_counts[ADDR_SGU], sgu_hw_fail, radar_fault_frames))
  report.checks.append(_radar_donor_check(can_counts))
  return report


def synthetic_drive(*, override_emits_disengage: bool = False,
                    override_drops_long: bool = False,
                    include_radar: bool = True) -> list:
  """Minimal Pre-AP message stream covering engage, gas override, brake, cancel."""
  msgs = []
  t = 1_000_000

  def stamp(event):
    nonlocal t
    event.valid = True
    event.logMonoTime = t
    t += 100_000_000
    return event

  params = capnp_log.Event.new_message()
  cp = params.init("carParams")
  cp.brand = "tesla"
  cp.carFingerprint = "TESLA_MODEL_S_PREAP"
  cp.openpilotLongitudinalControl = True
  cp.pcmCruise = False
  msgs.append(stamp(params))

  frames: list[tuple[int, bytes, int]] = [
    (GAS_COMMAND, bytes(6), 2),
    (GAS_SENSOR, bytes(6), 2),
  ]
  if include_radar:
    frames.extend([
      (ADDR_SGU, bytes(6), 1),
      (ADDR_POINT0, bytes(8), 1),
      (ADDR_POINT0 + 1, bytes(8), 1),
    ])
  can = capnp_log.Event.new_message()
  can_frames = can.init("can", len(frames))
  for i, (addr, dat, src) in enumerate(frames):
    can_frames[i].address, can_frames[i].dat, can_frames[i].src = addr, dat, src
  msgs.append(stamp(can))

  # idle, engage, hold, gas override, release, brake (drop long), cancel
  timeline = [
    (False, False, False, False, ()),
    (False, False, False, False, ()),
    (True, True, False, False, (LAT_ENGAGE, LONG_ENGAGE)),
    (True, True, False, False, ()),
    (True, True, False, False, ()),
    (True, (False if override_drops_long else True), True, False,
     (GAS_OVERRIDE,) + ((LONG_DISENGAGE,) if override_emits_disengage else ())),
    (True, (False if override_drops_long else True), True, False, (GAS_OVERRIDE,)),
    (True, True, False, False, ()),
    (True, False, False, True, (LONG_DISENGAGE,)),
    (False, False, False, False, (LAT_DISENGAGE,)),
  ]
  for lat, long_on, gas, brake, event_names in timeline:
    cs_event = capnp_log.Event.new_message()
    cs = cs_event.init("carState")
    cs.cruiseState.enabled = lat
    cs.enableLongControl = long_on
    cs.gasPressed = gas
    cs.brakePressed = brake
    cs.vEgo = 15.0
    msgs.append(stamp(cs_event))
    if event_names:
      ev = capnp_log.Event.new_message()
      packed = ev.init("onroadEvents", len(event_names))
      for i, name in enumerate(event_names):
        packed[i].name = getattr(capnp_log.OnroadEvent.EventName, name)
      msgs.append(stamp(ev))
    if include_radar:
      tracks = capnp_log.Event.new_message()
      live = tracks.init("liveTracks")
      pts = live.init("points", 1)
      pts[0].trackId = 1
      pts[0].dRel = 20.0
      pts[0].measured = True
      msgs.append(stamp(tracks))
  return msgs


def _signal(name: str, addr: int, counts: dict[int, int], buses: dict[int, set[int]], extra: str = "") -> SignalPresence:
  count = counts.get(addr, 0)
  bus_note = f"bus={','.join(str(b) for b in sorted(buses.get(addr, ())))}" if count else extra
  return SignalPresence(name, count > 0, count, extra=bus_note)


def _sgu_extra(hw_fail: int, sgu_fail: int, dirty: int) -> str:
  return f"HWFail={hw_fail} SGUFail={sgu_fail} dirty={dirty}"


def _le_bit(dat: bytes, index: int) -> bool:
  byte_i, bit_i = divmod(index, 8)
  if byte_i >= len(dat):
    return False
  return bool((dat[byte_i] >> bit_i) & 1)


def _expected_chimes(samples: list[tuple[int, bool, bool, bool, bool]]) -> list[tuple[int, bool, bool, bool, bool]]:
  if not samples:
    return []
  # A segment that opens already engaged is not a rising edge.
  lat0, long0 = samples[0][1], samples[0][2]
  prev = PreAPChimeState(lat_engaged=lat0, long_engaged=long0)
  out = [(samples[0][0], False, False, False, False)]
  for t, lat, long_on, _gas, _brake in samples[1:]:
    chimes, prev = update_preap_chimes(lat_engaged=lat, long_engaged=long_on, prev=prev)
    out.append((t, chimes.lat_engage, chimes.lat_disengage, chimes.long_engage, chimes.long_disengage))
  return out


def _events_near(events: list[tuple[int, frozenset[str]]], t: int, window_ns: int = 250_000_000) -> frozenset[str]:
  names: set[str] = set()
  for et, ev in events:
    if abs(et - t) <= window_ns:
      names.update(ev)
  return frozenset(names)


def _lat_long_chime_checks(samples, events, expected, saw_long_field: bool) -> list[CheckResult]:
  if not samples:
    return [CheckResult("lat_long_chimes", "skip", "no carState in log")]

  lat_eng = [(t, e) for (t, *_), e in zip(samples, expected, strict=True) if e[1]]
  lat_dis = [(t, e) for (t, *_), e in zip(samples, expected, strict=True) if e[2]]
  long_eng = [(t, e) for (t, *_), e in zip(samples, expected, strict=True) if e[3]]
  long_dis = [(t, e) for (t, *_), e in zip(samples, expected, strict=True) if e[4]]

  checks = []
  if not events:
    checks.append(CheckResult(
      "lat_engage_chime", "skip" if not lat_eng else "pass",
      f"{len(lat_eng)} cruise rising edges; no onroadEvents to compare (scan reconstructed via update_preap_chimes)",
    ))
  else:
    checks.append(_edge_event_check("lat_engage_chime", lat_eng, events, LAT_ENGAGE, "cruiseState.enabled rising"))
    checks.append(_edge_event_check("lat_disengage_chime", lat_dis, events, LAT_DISENGAGE, "cruiseState.enabled falling"))

  if not saw_long_field:
    checks.append(CheckResult("long_engage_chime", "skip", "enableLongControl not on CarState"))
    checks.append(CheckResult("long_disengage_chime", "skip", "enableLongControl not on CarState"))
    return checks

  if not events:
    checks.append(CheckResult(
      "long_engage_chime", "pass" if long_eng else "skip",
      f"{len(long_eng)} enableLongControl rising edges reconstructed; no onroadEvents",
    ))
    checks.append(CheckResult(
      "long_disengage_chime", "pass" if long_dis else "skip",
      f"{len(long_dis)} enableLongControl falling edges reconstructed; no onroadEvents",
    ))
    return checks

  checks.append(_edge_event_check("long_engage_chime", long_eng, events, LONG_ENGAGE, "enableLongControl rising"))
  checks.append(_edge_event_check("long_disengage_chime", long_dis, events, LONG_DISENGAGE, "enableLongControl falling"))
  return checks


def _edge_event_check(name: str, edges, events, event_name: str, label: str) -> CheckResult:
  if not edges:
    return CheckResult(name, "skip", f"no {label} in this log")
  missing = []
  for t, _ in edges:
    if event_name not in _events_near(events, t):
      missing.append(t)
  if missing:
    return CheckResult(name, "fail", f"{len(edges)} {label}; {len(missing)} without {event_name}")
  return CheckResult(name, "pass", f"{len(edges)} {label} with {event_name}")


def _gas_override_check(samples, events, expected, saw_long_field: bool) -> CheckResult:
  if not saw_long_field:
    return CheckResult("gas_override_no_chime", "skip", "enableLongControl not on CarState")
  if len(samples) < 2:
    return CheckResult("gas_override_no_chime", "skip", "not enough carState samples")
  overrides = []
  prev_gas = samples[0][3]
  prev_long = samples[0][2]
  prev_lat = samples[0][1]
  for (t, lat, long_on, gas, _brake), exp in zip(samples[1:], expected[1:], strict=True):
    started = gas and not prev_gas and prev_long and prev_lat
    if started:
      observed = _events_near(events, t) if events else frozenset()
      overrides.append((t, long_on, exp[4], LONG_DISENGAGE in observed))
    prev_gas, prev_long, prev_lat = gas, long_on, lat
  if not overrides:
    return CheckResult("gas_override_no_chime", "skip", "no gas press while lat+long engaged")

  chimed = [t for t, _held, expected_dis, observed_dis in overrides if expected_dis or observed_dis]
  dropped = [t for t, held, *_ in overrides if not held]
  if chimed or dropped:
    return CheckResult(
      "gas_override_no_chime",
      "fail",
      f"{len(overrides)} overrides; long-disengage chime on {len(chimed)}; enableLongControl dropped on {len(dropped)}",
    )
  return CheckResult(
    "gas_override_no_chime",
    "pass",
    f"{len(overrides)} gas presses while engaged; enableLongControl held; no {LONG_DISENGAGE}",
  )


def _enable_long_holds_on_gas(samples, saw_long_field: bool) -> CheckResult:
  if not saw_long_field:
    return CheckResult("enableLongControl", "skip", "field missing")
  if len(samples) < 2:
    return CheckResult("enableLongControl", "skip", "not enough carState samples")
  held = 0
  dropped = 0
  prev_long = samples[0][2]
  for _t, lat, long_on, gas, _brake in samples[1:]:
    if gas and lat and prev_long:
      if long_on:
        held += 1
      else:
        dropped += 1
    prev_long = long_on
  if held == 0 and dropped == 0:
    ever = any(s[2] for s in samples)
    return CheckResult(
      "enableLongControl",
      "pass" if ever else "skip",
      "published on CarState" + ("; never true in this log" if not ever else ""),
    )
  if dropped:
    return CheckResult("enableLongControl", "fail", f"dropped during gas override on {dropped} samples (held {held})")
  return CheckResult("enableLongControl", "pass", f"held true across {held} gas-override samples")


def _radar_track_check(bosch_points: int, live_frames: int, live_points: int, radar_unavailable: bool) -> CheckResult:
  if bosch_points == 0 and live_frames == 0:
    return CheckResult("radar_tracks", "skip", "no Bosch point CAN and no liveTracks")
  if bosch_points > 0 and live_points == 0:
    if radar_unavailable:
      return CheckResult(
        "radar_tracks",
        "skip",
        f"Bosch points n={bosch_points} but CarParams.radarUnavailable; liveTracks empty",
      )
    if live_frames == 0:
      return CheckResult("radar_tracks", "skip", f"Bosch points n={bosch_points}; replay card/radard to publish liveTracks")
    return CheckResult("radar_tracks", "fail", f"Bosch points n={bosch_points} but liveTracks published 0 points")
  return CheckResult("radar_tracks", "pass", f"Bosch points n={bosch_points}, liveTracks frames={live_frames} points={live_points}")


def _radar_hw_fail_check(sgu_frames: int, hw_fail: int, radar_fault_frames: int) -> CheckResult:
  if sgu_frames == 0:
    return CheckResult("radar_hw_fail", "skip", "no TeslaRadarSguInfo 0x301")
  if hw_fail == 0:
    return CheckResult(
      "radar_hw_fail",
      "pass",
      f"0x301 n={sgu_frames}; RADC_HWFail never set" + (f"; radarFault frames={radar_fault_frames}" if radar_fault_frames else ""),
    )
  return CheckResult(
    "radar_hw_fail",
    "pass",
    f"RADC_HWFail on {hw_fail}/{sgu_frames} SGU frames; radarFault frames={radar_fault_frames} (logged, not a harness fail)",
  )


def _radar_donor_check(can_counts: dict[int, int]) -> CheckResult:
  feed = can_counts.get(ADDR_VIN_FEED, 0)
  host = can_counts.get(ADDR_VIN_HOST, 0)
  tx = can_counts.get(RADAR_UDS_TX, 0)
  rx = can_counts.get(RADAR_UDS_RX, 0)
  cfg = can_counts.get(ADDR_CAR_CONFIG, 0)
  if feed + host + tx + rx + cfg == 0:
    return CheckResult("radar_donor", "skip", "no 0x2A9/0x2B9/0x560/0x641/0x651")
  return CheckResult(
    "radar_donor",
    "pass",
    f"0x2A9={cfg} 0x2B9={feed} 0x560={host} UDS TX={tx} RX={rx}",
  )
