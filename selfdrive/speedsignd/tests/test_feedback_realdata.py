import json
import os
import time

from openpilot.selfdrive.speedsignd import two_stage


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
  assert two_stage.current_segment_dir(str(root), now=now) == str(cur)
  line = json.dumps(rec)
  assert two_stage.copy_to_realdata(rec, line, str(fb), root=str(root), now=now) == cur.name
  with open(cur / "speedsign_feedback.jsonl") as f:
    assert '"answer": "yes"' in f.read()
  assert (cur / ("speedsign_" + rec["jpg"])).read_bytes() == b"jpg"


def test_no_segment_when_stale(tmp_path):
  seg = tmp_path / "0000018a--17462b65c0--4"
  seg.mkdir()
  now = time.time()  # noqa: TID251
  os.utime(seg, (now - 3600, now - 3600))
  assert two_stage.current_segment_dir(str(tmp_path), now=now) is None
