import queue
import threading
import time
from dataclasses import dataclass

from cereal import log
from openpilot.system.hardware.thermal_reader import (
  STALE_AFTER_S, STALE_ESCALATE_S, ThermalCache, stale_thermal_status, read_thermal_snapshot, thermal_reader_thread,
)

ThermalStatus = log.DeviceState.ThermalStatus


class FakeZone:
  def __init__(self, name, temp, block=None):
    self.name, self.temp, self.block = name, temp, block

  def read(self):
    if self.block is not None:
      self.block()
    return self.temp


@dataclass
class FakeConfig:
  cpu: list | None = None
  pmic: list | None = None
  memory: FakeZone | None = None


class FakeClock:
  def __init__(self):
    self.t = 100.0

  def __call__(self):
    return self.t


def test_snapshot_matches_get_msg_shape_and_names_slow_zone():
  clk = FakeClock()
  def slow():
    clk.t += 13.0
  cfg = FakeConfig(cpu=[FakeZone("cpu0", 50.), FakeZone("cpu1", 51.)],
                   pmic=[FakeZone("pm8998_tz", 45., block=slow)], memory=FakeZone("ddr", 40.))
  snap = read_thermal_snapshot(cfg, clock=clk)
  assert snap.temps == {"cpuTempC": [50., 51.], "pmicTempC": [45.], "memoryTempC": 40.}
  assert list(snap.slow_zones) == ["pm8998_tz"]
  assert abs(snap.slow_zones["pm8998_tz"] - 13.0) < 1e-6


def test_stale_flag_and_holds_last_temps():
  clk = FakeClock()
  q = queue.Queue(maxsize=1)
  cache = ThermalCache(q, clock=clk)
  q.put_nowait(read_thermal_snapshot(FakeConfig(memory=FakeZone("ddr", 70.)), clock=clk))
  assert cache.poll() is False
  clk.t += STALE_AFTER_S - 0.5
  assert cache.poll() is False
  clk.t += 1.0
  assert cache.poll() is True
  assert cache.temps == {"memoryTempC": 70.}  # held, not zeroed
  q.put_nowait(read_thermal_snapshot(FakeConfig(memory=FakeZone("ddr", 71.)), clock=clk))
  assert cache.poll() is False
  assert cache.temps["memoryTempC"] == 71.


OK, OV, CRIT = ThermalStatus.ok, ThermalStatus.overheated, ThermalStatus.critical
YELLOW_T = 92.0


def status(st, stale, stale_s, temp):
  return stale_thermal_status(st, stale, stale_s, temp, YELLOW_T, OK, OV)


def test_stale_green_holds_up_to_60s_then_escalates():
  assert status(OK, False, 0.0, 76.) == OK
  assert status(OK, True, 3.0, 76.) == OK            # 13 s stall = 3 s stale
  assert status(OK, True, STALE_ESCALATE_S, 76.) == OK
  assert status(OK, True, STALE_ESCALATE_S + 0.5, 76.) == OV


def test_stale_from_yellow_escalates_immediately():
  assert status(OK, True, 0.0, 93.) == OV            # last reading in yellow zone
  assert status(OV, True, 0.0, 90.) == OV
  assert status(CRIT, True, 0.0, 99.) == CRIT        # never cooler


def test_stale_start_end_logged(monkeypatch):
  import openpilot.system.hardware.thermal_reader as tr
  events = []
  monkeypatch.setattr(tr.cloudlog, "event", lambda name, **kw: events.append((name, kw)))
  clk = FakeClock()
  q = queue.Queue(maxsize=1)
  cache = ThermalCache(q, clock=clk)
  q.put_nowait(read_thermal_snapshot(FakeConfig(memory=FakeZone("ddr", 70.)), clock=clk))
  cache.poll()
  clk.t += STALE_AFTER_S + 1
  cache.poll()
  clk.t += 70
  assert cache.stale_s > STALE_ESCALATE_S
  q.put_nowait(read_thermal_snapshot(FakeConfig(memory=FakeZone("ddr", 70.)), clock=clk))
  cache.poll()
  names = [n for n, _ in events]
  assert names == ["hardwared thermal stale start", "hardwared thermal stale end"]
  assert events[1][1]["stale_s"] == 70.0
  assert cache.stale_s == 0.0


def test_13s_thermal_read_does_not_stop_2hz_publish():
  """Real threads: one zone sleeps 13 s. The 2 Hz publish loop keeps going,
  goes stale after 10 s at green (76 C) without any alert, then recovers."""
  release = threading.Event()
  stall_once = {"armed": False}
  def stall():
    if stall_once["armed"]:
      stall_once["armed"] = False
      release.wait(13.0)
  cfg = FakeConfig(cpu=[FakeZone("cpu0", 60.)], pmic=[FakeZone("pm8998_tz", 50., block=stall)])
  q = queue.Queue(maxsize=1)
  end = threading.Event()
  th = threading.Thread(target=thermal_reader_thread, args=(end, cfg, q), daemon=True)
  th.start()
  cache = ThermalCache(q)
  time.sleep(0.6)
  cache.poll()
  assert cache.snapshot is not None
  stall_once["armed"] = True

  publishes, stale_seen, statuses = [], [], []
  t0 = time.monotonic()
  while time.monotonic() - t0 < 15.0:
    start = time.monotonic()
    stale = cache.poll()
    publishes.append(start)
    stale_seen.append((start - t0, stale))
    statuses.append(status(OK, stale, cache.stale_s, 76.))
    assert time.monotonic() - start < 0.05  # never blocks on the sensor
    time.sleep(max(0.0, 0.5 - (time.monotonic() - start)))
  end.set()
  release.set()

  gaps = [b - a for a, b in zip(publishes, publishes[1:], strict=False)]
  assert max(gaps) < 0.7
  assert len(publishes) >= 28
  assert any(s for _, s in stale_seen)
  assert not any(s for t, s in stale_seen if t < 9.0)
  assert all(st == OK for st in statuses)  # no overheat alert at green
  # read finished at ~13 s, then fresh again
  assert stale_seen[-1][1] is False
  assert statuses[-1] == ThermalStatus.ok
