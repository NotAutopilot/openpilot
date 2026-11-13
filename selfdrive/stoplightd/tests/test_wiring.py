"""stoplightd plumbing: message, process, param, soundd, UI, and wav."""
import re
import wave
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.stoplightd import stoplightd as sd
from openpilot.selfdrive.stoplightd.detect import find_lamps
from openpilot.selfdrive.ui.onroad.stoplight_chip import GREEN_TEXT, RED_TEXT, StoplightChip, chip_from_sm
from openpilot.selfdrive.ui.stoplight_chime import (
  STOPLIGHT_CHIME_FILE, STOPLIGHT_CHIME_ID, STOPLIGHT_CHIME_MIN_VOLUME, floor_volume, should_ding,
)

ROOT = Path(__file__).resolve().parents[3]


def text(rel):
  return (ROOT / rel).read_text(encoding="utf-8")


def test_message_and_service():
  log = text("cereal/log.capnp")
  assert re.search(r"napVision @113 :Custom\.NapVision;", log)
  assert "customReserved6" not in log
  custom = text("cereal/custom.capnp")
  assert "struct NapVision @0xf98d843bfd7004a3" in custom
  assert "stoplight :group" in custom
  group = custom.split("stoplight :group", 1)[1].split("\n  }", 1)[0]
  ords = [int(n) for n in re.findall(r"@(\d+)\s*:", group)]
  assert ords == list(range(14))
  for field in ("chime", "chimeCount", "redLight", "green", "gated", "enabled"):
    assert re.search(rf"\b{field} @\d+", group), field
  assert '"napVision": (True, 0., 1)' in text("cereal/services.py")


def test_process_and_param():
  procs = text("system/manager/process_config.py")
  line = next(ln for ln in procs.splitlines() if '"stoplightd"' in ln)
  assert "selfdrive.stoplightd.stoplightd" in line
  assert "only_onroad" in line and 'get_bool("NAPStoplightChime")' in line
  assert "optional=" not in line
  assert "restart_if_crash=True" in line
  assert '{"NAPStoplightChime", {PERSISTENT, BOOL, "0"}}' in text("common/params_keys.h")
  # Default Off: the process does not run in onroad tests, so no PROCS budget.
  assert "stoplightd" not in text("selfdrive/test/test_onroad.py")


def test_no_carstate_reader():
  src = text("selfdrive/stoplightd/stoplightd.py")
  assert 'sub_sock("livePose"' in src and 'sub_sock("onroadEvents"' in src
  assert "carState" not in re.sub(r'""".*?"""|#.*', "", src, flags=re.S)
  # Portable: do not import jmbrunick-only helpers (e.g. speedsignd.nv12).
  assert "speedsignd" not in src and "pathobstacled" not in src


def test_soundd_plays_ding():
  sound = text("selfdrive/ui/soundd.py")
  assert "sound_list[STOPLIGHT_CHIME_ID] = (STOPLIGHT_CHIME_FILE, 1, MAX_VOLUME)" in sound
  assert "'napVision'" in sound.split("def soundd_thread", 1)[0] or "'napVision'" in sound
  alert = sound.split("def get_audible_alert", 1)[1].split("\n  def ", 1)[0]
  # The ding is checked after selfdriveState and never preempts a playing alert.
  assert alert.index("selfdriveState") < alert.index("napVision")
  assert "sm['napVision'].stoplight" in alert
  assert "self.current_alert == AudibleAlert.none or self.current_alert == STOPLIGHT_CHIME_ID" in alert
  assert "stoplight_floor_volume(self.current_alert" in sound
  assert '"napVision"' in text("selfdrive/ui/ui_state.py")


def test_chime_helpers():
  assert STOPLIGHT_CHIME_ID < 0 and STOPLIGHT_CHIME_FILE == "light_green.wav"
  assert should_ding(SimpleNamespace(enabled=True, chime=True))
  assert not should_ding(SimpleNamespace(enabled=False, chime=True))
  assert not should_ding(SimpleNamespace(enabled=True, chime=False))
  assert not should_ding(object())
  assert floor_volume(STOPLIGHT_CHIME_ID, 0.1) == STOPLIGHT_CHIME_MIN_VOLUME
  assert floor_volume(STOPLIGHT_CHIME_ID, 0.9) == 0.9
  assert floor_volume(STOPLIGHT_CHIME_ID, float("nan")) == STOPLIGHT_CHIME_MIN_VOLUME
  assert floor_volume(-1, 0.1) == 0.1


def test_wav_is_plain_git_48k_mono():
  path = ROOT / "selfdrive/assets/sounds/light_green.wav"
  with wave.open(str(path), "rb") as w:
    assert w.getframerate() == 48000 and w.getnchannels() == 1 and w.getsampwidth() == 2
    assert 0.4 <= w.getnframes() / 48000 <= 1.5
  assert not path.read_bytes().startswith(b"version https://git-lfs")
  assert "selfdrive/assets/sounds/light_green.wav !filter !diff !merge -text" in text(".gitattributes")


def test_settings_toggle():
  nap = text("selfdrive/ui/layouts/settings/nap.py")
  assert "NAPStoplightChime" in nap
  assert "Green Light Chime" in nap
  assert "put_bool(NAP_STOPLIGHT_CHIME, False)" in nap
  mici = text("selfdrive/ui/mici/layouts/settings/nap.py")
  assert "NAPStoplightChime" in mici
  assert "green light chime" in mici


def test_hud_draws_chip():
  for rel in ("selfdrive/ui/onroad/hud_renderer.py", "selfdrive/ui/mici/onroad/hud_renderer.py"):
    assert "chip_from_sm" in text(rel), rel


def test_chip_states():
  chip = StoplightChip()
  msg = SimpleNamespace(enabled=True, gated=True, redLight=True, green=False)
  assert chip.update(msg, 0.1, 0.0)[0] == RED_TEXT
  msg.redLight, msg.green = False, True
  assert chip.update(msg, 0.1, 1.0)[0] == GREEN_TEXT
  assert chip.update(msg, 0.1, 3.9)[0] == GREEN_TEXT
  assert chip.update(msg, 0.1, 4.5) is None
  msg.gated = False
  assert chip.update(msg, 0.1, 5.0) is None
  msg.gated, msg.green, msg.redLight = True, False, True
  assert chip.update(msg, 9.0, 6.0) is None  # stale
  sm = SimpleNamespace(seen={"napVision": False}, recv_time={}, __getitem__=None)
  assert chip_from_sm(chip, sm, 1.0) is None


# --- daemon step with fakes ---------------------------------------------------

def _scene(red_on, green_on, width=1928, height=1208):
  """Half-res planes with one 3-lamp head: red at (1000, 300) full-res, green 64 px below."""
  hw, hh = width // 2, height // 2
  y = np.full((hh, hw), 60, np.uint8)
  u = np.full((hh, hw), 128, np.uint8)
  v = np.full((hh, hw), 128, np.uint8)
  def lamp(cx, cy, uu, vv):
    yy, xx = np.ogrid[:hh, :hw]
    m = (xx - cx) ** 2 + (yy - cy) ** 2 <= 7 ** 2
    y[m], u[m], v[m] = 200, uu, vv
  if red_on:
    lamp(500, 150, 100, 235)
  if green_on:
    lamp(500, 182, 168, 20)
  return ((y, u, v), width), ""


def test_synthetic_scene_detects():
  ((y, u, v), _), _ = _scene(True, False)
  lamps = find_lamps(y, u, v)
  assert [lp.color for lp in lamps] == ["red"]
  ((y, u, v), _), _ = _scene(False, True)
  assert [lp.color for lp in find_lamps(y, u, v)] == ["green"]


def _drive(core, t, frames, enabled=True, speed=0.0, events=()):
  outs = []
  calls = []
  for f in frames:
    core.gate.on_pose(t, speed)
    core.gate.on_events(t, list(events))
    def grab(now, f=f):
      calls.append(now)
      return f
    outs.append(core.step(t, enabled, grab))
    t += 0.25
  return outs, calls, t


def test_step_red_to_green_once():
  core = sd.Stoplight()
  idle = [(None, "no_frame")] * 4
  outs, _, t = _drive(core, 0.0, idle + [_scene(True, False)] * 8 + [_scene(False, True)] * 8)
  assert sum(o["chime"] for o in outs) == 1
  assert outs[-1]["chimeCount"] == 1
  assert any(o["redLight"] for o in outs)
  assert all(set(o) >= {"enabled", "gated", "chime", "chimeCount", "redLight", "green"} for o in outs)
  # Pull away: gate closes, tracker resets; next stop can ding again.
  outs2, calls, t = _drive(core, t, [_scene(False, True)] * 2, speed=3.0)
  assert not any(o["gated"] for o in outs2) and calls == []
  outs3, _, _ = _drive(core, t, [_scene(True, False)] * 12 + [_scene(False, True)] * 8)
  assert sum(o["chime"] for o in outs3) == 1 and outs3[-1]["chimeCount"] == 2


def test_step_off_reads_no_frames():
  core = sd.Stoplight()
  outs, calls, _ = _drive(core, 0.0, [_scene(True, False)] * 10, enabled=False)
  assert calls == [] and not any(o["gated"] for o in outs)


def test_step_park_reads_no_frames():
  core = sd.Stoplight()
  outs, calls, _ = _drive(core, 0.0, [_scene(True, False)] * 10, events=("wrongGear",))
  assert calls == [] and not any(o["redLight"] for o in outs)


def test_planes_from_nv12_uses_uv_offset():
  w, h, stride = 64, 32, 128
  uv_off = stride * h + 4096
  data = np.zeros(uv_off + stride * h // 2, np.uint8)
  data[:stride * h].reshape(h, stride)[:, :w] = 77
  uv = data[uv_off:].reshape(h // 2, stride)
  uv[:, 0:w:2] = 11
  uv[:, 1:w:2] = 222
  buf = SimpleNamespace(width=w, height=h, stride=stride, uv_offset=uv_off, data=data)
  planes = sd.planes_from_nv12(buf)
  assert planes is not None
  y, u, v = planes
  assert y.shape == (h // 2, w // 2) == u.shape == v.shape
  assert (y == 77).all() and (u == 11).all() and (v == 222).all()
  assert sd.planes_from_nv12(SimpleNamespace(width=w, height=h, stride=stride, uv_offset=uv_off, data=data[:100])) is None
