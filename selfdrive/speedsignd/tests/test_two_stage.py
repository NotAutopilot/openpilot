import json
import os


def _lines(path):
  with open(path) as f:
    return f.read().splitlines()

import numpy as np

from openpilot.selfdrive.speedsignd import two_stage as ts
from openpilot.selfdrive.speedsignd.nv12 import Nv12DetectCrop


def test_decode_maps_strip_to_frame():
  raw = np.zeros((1, 5, 4), np.float32)
  raw[0, :, 0] = [176, 64, 20, 20, 0.9]   # strip center
  raw[0, :, 1] = [10, 10, 5, 5, 0.05]     # below threshold
  out = ts.decode_proposals(raw, ts.STRIP_RECT)
  assert len(out) == 1
  x, y, w, h, s = out[0]
  sx = 964 / 352
  assert abs((x + w / 2) - (964 + 176 * sx)) < 1.0
  assert abs((y + h / 2) - (525 + 64 * 350 / 128)) < 1.0
  assert s == np.float32(0.9)


def test_tracker_needs_two_hits_and_captures_large():
  tr = ts.SignTracker()
  tr.update([(1100, 740, 20, 30, 0.5)], 0.0)
  assert tr.capture_candidate(0.0) is None  # seen once
  tr.update([(1130, 735, 24, 34, 0.5)], 0.5)
  assert tr.capture_candidate(0.5) is None  # small, mid-frame
  tr.update([(1400, 700, 50, 70, 0.6)], 1.0)
  c = tr.capture_candidate(1.0)
  assert c is not None and c.hits == 3


def test_neg_track_suppressed():
  tr = ts.SignTracker()
  for i, t in enumerate((0.0, 0.5)):
    tr.update([(1500 + 40 * i, 700, 60, 70, 0.5)], t)
  c = tr.capture_candidate(0.5)
  assert c is not None
  tr.mark_read(c, got_mph=False, neg=ts.neg_read({"peak_name": "neg_other", "peak_conf": 0.8}))
  assert c.neg and c.done
  tr.update([(1580, 700, 64, 74, 0.5)], 1.0)
  assert tr.capture_candidate(1.0) is None


def test_capture_rect_clamped():
  x, y, w, h = ts.capture_rect((1900, 1190, 20, 20))
  assert w == h == ts.CAPTURE_MIN_SIDE
  assert x + w <= 1928 and y + h <= 1208 and x % 2 == 0 and y % 2 == 0
  _, _, w2, _ = ts.capture_rect((1000, 500, 300, 300))
  assert w2 == ts.CAPTURE_MAX_SIDE


def test_skip_when_stopped():
  assert not ts.proposals_allowed(0.5)
  assert ts.proposals_allowed(5.0)
  assert ts.proposals_allowed(None)


def test_strip_rgb_shape():
  crop = Nv12DetectCrop(y=np.full((350, 964), 100, np.uint8), uv=np.full((175, 964), 128, np.uint8),
                        frame_w=1928, frame_h=1208, crop=ts.STRIP_RECT)
  rgb = ts.strip_rgb_from_crop(crop)
  assert rgb.shape == (128, 352, 3) and rgb.dtype == np.uint8


def test_feedback_yes_no(tmp_path):
  d = str(tmp_path)
  assert ts.write_pending({"mph": 45, "conf": 0.8, "track_id": 3, "lat": 1.0, "lon": 2.0,
                           "gps_ok": True, "t": 1.0, "class": "sl_45"}, b"\xff\xd8jpg", d)
  rec = ts.record_answer(45, True, d, now=1000.0)
  assert rec["answer"] == "yes" and rec["track_id"] == 3 and rec["jpg"].endswith("_yes_45.jpg")
  assert os.path.isfile(os.path.join(d, rec["jpg"]))
  assert len(_lines(os.path.join(d, ts.CONFIRMED_JSONL))) == 1
  rec2 = ts.record_answer(30, False, d, now=1001.0)  # plate mph != pending -> stale, no jpg
  assert rec2["stale_pending"] and "jpg" not in rec2
  lines = [json.loads(x) for x in _lines(os.path.join(d, ts.FEEDBACK_JSONL))]
  assert [x["answer"] for x in lines] == ["yes", "no"]
  assert len(_lines(os.path.join(d, ts.CONFIRMED_JSONL))) == 1
