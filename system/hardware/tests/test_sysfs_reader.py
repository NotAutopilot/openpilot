import ast
import inspect
import textwrap
import threading
import time

from openpilot.system.hardware import sysfs_reader as sr
from openpilot.system.hardware.sysfs_reader import SysReadCache, read_once, sysfs_reader_thread


class FakeClock:
  def __init__(self):
    self.t = 100.0

  def __call__(self):
    return self.t


def test_read_once_caches_and_names_slow_read(monkeypatch):
  clk = FakeClock()
  events = []
  monkeypatch.setattr(sr.cloudlog, "event", lambda name, **kw: events.append((name, kw)))
  def slow_som():
    clk.t += 13.0
    return 4.2
  cache = SysReadCache({"power_draw": 0.0, "som_power_draw": 0.0}, clock=clk)
  slow = read_once({"power_draw": lambda: 1.5, "som_power_draw": slow_som}, cache, clock=clk)
  assert cache.get("power_draw") == 1.5
  assert cache.get("som_power_draw") == 4.2
  assert list(slow) == ["som_power_draw"]
  assert events and events[0][0] == "hardwared slow sysfs read"
  assert events[0][1]["reads_ms"] == {"som_power_draw": 13000.0}


def test_failed_read_keeps_last_value(monkeypatch):
  monkeypatch.setattr(sr.cloudlog, "exception", lambda *a, **k: None)
  cache = SysReadCache({"gpu": 7})
  def boom():
    raise OSError("EIO")
  read_once({"gpu": boom}, cache)
  assert cache.get("gpu") == 7


def test_13s_hung_power_draw_read_does_not_stop_2hz_device_state():
  """Real threads: the power-draw read hangs ~13 s (Oct 13 11:25 route 0000018e
  seg 23). The 2 Hz publish loop keeps sending deviceState with the last
  cached power values and never blocks; fresh values resume after the hang."""
  release = threading.Event()
  armed = {"on": False}
  hang_done = threading.Event()
  def som_power_draw():
    if armed["on"]:
      armed["on"] = False
      release.wait(13.0)
      hang_done.set()
      return 9.9
    return 9.9 if hang_done.is_set() else 3.3
  cache = SysReadCache({"power_draw": 0.0, "som_power_draw": 0.0, "memory": 0})
  readers = {"power_draw": lambda: 1.1, "som_power_draw": som_power_draw, "memory": lambda: 42}
  end = threading.Event()
  threading.Thread(target=sysfs_reader_thread, args=(end, readers, cache), daemon=True).start()
  time.sleep(0.6)
  assert cache.get("som_power_draw") == 3.3
  armed["on"] = True

  published = []  # (t, powerDrawW, somPowerDrawW)
  t0 = time.monotonic()
  # run through the hang plus ~1 s after it ends (bounded for slow CI)
  done_at = None
  while time.monotonic() - t0 < 20.0:
    if hang_done.is_set() and done_at is None:
      done_at = time.monotonic()
    if done_at is not None and time.monotonic() - done_at > 1.0:
      break
    start = time.monotonic()
    msg = {"powerDrawW": cache.get("power_draw", 0.0), "somPowerDrawW": cache.get("som_power_draw", 0.0),
           "memoryUsagePercent": cache.get("memory", 0)}
    published.append((start - t0, msg))
    assert time.monotonic() - start < 0.05  # never blocks on the hung read
    time.sleep(max(0.0, 0.5 - (time.monotonic() - start)))
  end.set()
  release.set()

  times = [t for t, _ in published]
  gaps = [b - a for a, b in zip(times, times[1:], strict=False)]
  assert max(gaps) < 0.7
  assert done_at is not None and done_at - t0 > 12.0  # it really hung ~13 s
  assert len(published) >= 26
  # during the hang deviceState kept the last good value (not zero)
  assert all(m["somPowerDrawW"] in (3.3, 9.9) for _, m in published)
  assert all(m["memoryUsagePercent"] == 42 for _, m in published)
  assert any(m["somPowerDrawW"] == 3.3 for t, m in published if 5.0 < t < 12.0)
  # read finished at ~13 s, value updated
  assert published[-1][1]["somPowerDrawW"] == 9.9


BLOCKING_CALLS = {
  "get_current_power_draw", "get_som_power_draw", "get_gpu_usage_percent", "get_screen_brightness",
  "booted", "get_available_percent", "virtual_memory", "cpu_percent", "get_msg",
}


def test_publish_loop_makes_no_blocking_system_file_reads():
  """Guard: inside hardware_thread's `while` loop, none of these reads may be
  called directly (they belong in sysfs_reader / thermal_reader threads)."""
  from openpilot.system.hardware import hardwared
  tree = ast.parse(textwrap.dedent(inspect.getsource(hardwared.hardware_thread)))
  loops = [n for n in ast.walk(tree) if isinstance(n, ast.While)]
  assert loops
  bad = set()
  for loop in loops:
    for n in ast.walk(loop):
      if isinstance(n, ast.Call):
        f = n.func
        name = f.attr if isinstance(f, ast.Attribute) else getattr(f, "id", None)
        if name in BLOCKING_CALLS:
          bad.add(name)
  assert not bad, f"blocking reads on the publish loop: {sorted(bad)}"
