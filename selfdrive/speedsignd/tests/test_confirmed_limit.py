import json
import os
import time

from openpilot.selfdrive.speedsignd.confirmed_limit import (
  MPH_TO_KPH, ConfirmedLimitHold, ConfirmedMapData, confirmed_payload, parse_confirmed,
)
from openpilot.selfdrive.speedsignd import two_stage


def test_parse_confirmed():
  assert parse_confirmed(None) is None
  assert parse_confirmed(confirmed_payload(35, 1.0)) == (35, 1.0)
  assert parse_confirmed(json.dumps({"mph": 45, "seq": 2})) == (45, 2.0)
  assert parse_confirmed({"mph": 33, "seq": 1}) is None
  assert parse_confirmed({"mph": 120, "seq": 1}) is None
  assert parse_confirmed("junk") is None


def test_hold_until_map_zone_changes():
  h = ConfirmedLimitHold()
  assert h.update(None, 88.5) is None
  k = h.update((35, 1.0), 88.5)
  assert abs(k - 35 * MPH_TO_KPH) < 1e-6
  assert h.update((35, 1.0), 88.5) == k      # same OSM zone: held
  assert h.update((35, 1.0), None) == k      # OSM unknown: held
  assert h.update((35, 1.0), 72.4) is None   # OSM changed: released
  assert h.update((35, 1.0), 88.5) is None   # stays released for that tap
  k2 = h.update((45, 2.0), 72.4)             # new Yes applies again
  assert abs(k2 - 45 * MPH_TO_KPH) < 1e-6


def test_hold_anchor_when_no_map_and_newer_yes_replaces():
  h = ConfirmedLimitHold()
  h.update((30, 1.0), None)
  assert h.update((30, 1.0), 56.3) is not None   # first OSM value anchors
  assert h.update((30, 1.0), 56.3) is not None
  assert abs(h.update((50, 2.0), 56.3) - 50 * MPH_TO_KPH) < 1e-6
  assert h.update(None, 56.3) is None             # param cleared (offroad)
  assert not h.active


def test_map_proxy():
  class R:
    nextSpeedLimit = 20.0
    nextSpeedLimitDistance = 300.0
  m = ConfirmedMapData(56.3, R())
  assert m.speedLimitValid and abs(m.speedLimit - 56.3 / 3.6) < 1e-6
  assert m.nextSpeedLimit == 20.0 and m.nextSpeedLimitDistance == 300.0
  m = ConfirmedMapData(56.3, None)
  assert m.nextSpeedLimit == 0.0


def test_feedback_copied_to_current_segment(tmp_path):
  fb = tmp_path / "fb"
  fb.mkdir()
  root = tmp_path / "realdata"
  old = root / "0000018a--17462b65c0--3"
  cur = root / "0000018a--17462b65c0--4"
  old.mkdir(parents=True)
  cur.mkdir()
  (root / "boot").mkdir()
  now = time.time()  # noqa: TID251
  os.utime(old, (now - 120, now - 120))
  with open(fb / two_stage.PENDING_JSON, "w") as f:
    json.dump({"mph": 35, "has_jpg": True, "lat": 45.1, "lon": -95.0}, f)
  with open(fb / two_stage.PENDING_JPG, "wb") as f:
    f.write(b"jpg")
  rec = two_stage.record_answer(35, True, d=str(fb), now=now)
  assert rec["lat"] == 45.1
  seg = two_stage.current_segment_dir(str(root), now=now)
  assert seg == str(cur)
  line = two_stage.json.dumps(rec)
  assert two_stage.copy_to_realdata(rec, line, str(fb), root=str(root), now=now) == cur.name
  with open(cur / "speedsign_feedback.jsonl") as f:
    assert '"answer":"yes"' in f.read().replace(" ", "")
  assert (cur / ("speedsign_" + rec["jpg"])).read_bytes() == b"jpg"


def test_no_segment_when_stale(tmp_path):
  seg = tmp_path / "0000018a--17462b65c0--4"
  seg.mkdir()
  now = time.time()  # noqa: TID251
  os.utime(seg, (now - 3600, now - 3600))
  assert two_stage.current_segment_dir(str(tmp_path), now=now) is None
