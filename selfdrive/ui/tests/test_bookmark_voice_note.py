import os
import wave

from openpilot.selfdrive.ui.feedback.feedbackd import VoiceNote, active_segment_dir, VOICE_NOTE_MAX_DURATION
from openpilot.system.micd import SAMPLE_RATE, SAMPLE_BUFFER


def _mkseg(root, name, lock):
  d = root / name
  d.mkdir()
  (d / "rlog.zst").write_bytes(b"")
  if lock:
    (d / "rlog.lock").write_bytes(b"")
  return d


def test_active_segment_dir_picks_locked(tmp_path):
  _mkseg(tmp_path, "00000001--abc--0", lock=False)
  live = _mkseg(tmp_path, "00000001--abc--1", lock=True)
  assert active_segment_dir(str(tmp_path)) == str(live)


def test_active_segment_dir_none(tmp_path):
  _mkseg(tmp_path, "00000001--abc--0", lock=False)
  assert active_segment_dir(str(tmp_path)) is None
  assert active_segment_dir(str(tmp_path / "missing")) is None


def test_voice_note_writes_wav(tmp_path):
  note = VoiceNote(str(tmp_path), "20261014-070000")
  blocks = int(VOICE_NOTE_MAX_DURATION * SAMPLE_RATE / SAMPLE_BUFFER)
  for _ in range(blocks):
    note.add(bytes(SAMPLE_BUFFER * 2))
  assert note.duration() >= VOICE_NOTE_MAX_DURATION
  note.close()
  path = tmp_path / "bookmark_20261014-070000.wav"
  assert path.exists() and not os.path.exists(str(path) + ".tmp")
  with wave.open(str(path)) as w:
    assert w.getframerate() == SAMPLE_RATE
    assert w.getnframes() == blocks * SAMPLE_BUFFER


def test_voice_note_without_segment(tmp_path):
  note = VoiceNote(None, "x")
  note.add(bytes(10))
  note.close()
  assert note.path is None
