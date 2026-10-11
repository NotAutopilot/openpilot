import queue
import threading
import time
from dataclasses import dataclass

from cereal import log
from openpilot.system.hardware.thermal_reader import (
  STALE_AFTER_S, ThermalCache, conservative_thermal_status, read_thermal_snapshot, thermal_reader_thread,
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


def test_conservative_status():
  ov = ThermalStatus.overheated
  assert conservative_thermal_status(ThermalStatus.ok, True, ov) == ov
  assert conservative_thermal_status(ThermalStatus.critical, True, ov) == ThermalStatus.critical
  assert conservative_thermal_status(ThermalStatus.ok, False, ov) == ThermalStatus.ok


def test_13s_thermal_read_does_not_stop_2hz_publish():
  """Real threads: one zone sleeps 13 s. The 2 Hz publish loop keeps going,
  goes stale after 10 s, reports at least overheated, then recovers."""
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
    statuses.append(conservative_thermal_status(ThermalStatus.ok, stale, ThermalStatus.overheated))
    assert time.monotonic() - start < 0.05  # never blocks on the sensor
    time.sleep(max(0.0, 0.5 - (time.monotonic() - start)))
  end.set()
  release.set()

  gaps = [b - a for a, b in zip(publishes, publishes[1:], strict=False)]
  assert max(gaps) < 0.7
  assert len(publishes) >= 28
  assert any(s for _, s in stale_seen)
  assert not any(s for t, s in stale_seen if t < 9.0)
  assert ThermalStatus.overheated in statuses
  # read finished at ~13 s, then fresh again
  assert stale_seen[-1][1] is False
  assert statuses[-1] == ThermalStatus.ok
