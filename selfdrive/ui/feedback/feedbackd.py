#!/usr/bin/env python3
"""Bookmark + voice note.

Every bookmarkButton press (settings sidebar "Bookmark" or the onroad flag):
  * immediately publishes userBookmark, so the segment is flagged/preserved, and
  * starts a voice note from the comma mic (rawAudioData from micd).
A second press stops the note early; otherwise it auto-stops after
VOICE_NOTE_MAX_DURATION. Audio is logged as audioFeedback blocks in the rlog and
also written as a WAV in the active segment dir:
  <log_root>/<route>--<seg>/bookmark_<YYYYmmdd-HHMMSS>.wav
"""
import os
import time
import wave
import datetime

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware.hw import Paths
from openpilot.system.micd import SAMPLE_RATE, SAMPLE_BUFFER

VOICE_NOTE_MAX_DURATION = 30.0
FEEDBACK_MAX_DURATION = VOICE_NOTE_MAX_DURATION  # used by the selfdrived audioFeedback alert
# a second tap this soon after the first is treated as a bounce, not a stop
MIN_STOP_DELAY = 0.5
# CLOCK_MONOTONIC is shared across processes. Ignore a stale dashboard request (e.g. queued before this process started)
DASH_REQUEST_MAX_AGE = 5.0


def active_segment_dir(root: str | None = None) -> str | None:
  """Newest segment dir that loggerd is still writing (has a .lock file)."""
  root = root or Paths.log_root()
  try:
    dirs = [os.path.join(root, d) for d in os.listdir(root) if "--" in d]
  except OSError:
    return None
  dirs = [d for d in dirs if os.path.isdir(d)]
  for d in sorted(dirs, key=os.path.getmtime, reverse=True):
    try:
      if any(f.endswith(".lock") for f in os.listdir(d)):
        return d
    except OSError:
      continue
  return None


class VoiceNote:
  def __init__(self, seg_dir: str | None, stamp: str):
    self.start_t = time.monotonic()
    self.block_num = 0
    self.path = None
    self._wav = None
    if seg_dir is not None:
      self.path = os.path.join(seg_dir, f"bookmark_{stamp}.wav")
      try:
        self._wav = wave.open(self.path + ".tmp", "wb")
        self._wav.setnchannels(1)
        self._wav.setsampwidth(2)
        self._wav.setframerate(SAMPLE_RATE)
      except OSError:
        cloudlog.exception("feedbackd: could not open voice note file")
        self._wav = None

  def add(self, data: bytes):
    self.block_num += 1
    if self._wav is not None:
      try:
        self._wav.writeframes(data)
      except OSError:
        cloudlog.exception("feedbackd: voice note write failed")

  def duration(self) -> float:
    return self.block_num * SAMPLE_BUFFER / SAMPLE_RATE

  def close(self):
    if self._wav is not None:
      try:
        self._wav.close()
        os.replace(self.path + ".tmp", self.path)
        cloudlog.event("bookmark_voice_note_saved", path=self.path, duration=self.duration())
      except OSError:
        cloudlog.exception("feedbackd: voice note close failed")
      self._wav = None


def main():
  pm = messaging.PubMaster(['userBookmark', 'audioFeedback'])
  # No carState here: carState is at the msgq reader limit.
  sm = messaging.SubMaster(['rawAudioData', 'bookmarkButton'])
  params = Params()
  note: VoiceNote | None = None
  last_left = -1
  params.remove("NAPBookmarkRecording")

  while True:
    sm.update()

    # Web dashboard tap (nap_dash server) arrives as a param, not a second
    # bookmarkButton publisher (msgq allows one publisher per service).
    dash_tap = False
    req = params.get("NAPBookmarkRequest")
    if req is not None:
      params.remove("NAPBookmarkRequest")
      dash_tap = abs(time.monotonic() - int(req)) < DASH_REQUEST_MAX_AGE

    if sm.updated['bookmarkButton'] or dash_tap:
      if note is None:
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        cloudlog.event("bookmark_pressed", stamp=stamp)
        pm.send('userBookmark', messaging.new_message('userBookmark', valid=True))
        note = VoiceNote(active_segment_dir(), stamp)
      elif time.monotonic() - note.start_t > MIN_STOP_DELAY:
        cloudlog.info("bookmark pressed again - stopping voice note")
        note.close()
        note = None

    if note is not None and sm.updated['rawAudioData']:
      raw_audio = sm['rawAudioData']
      msg = messaging.new_message('audioFeedback', valid=True)
      msg.audioFeedback.audio.data = raw_audio.data
      msg.audioFeedback.audio.sampleRate = raw_audio.sampleRate
      msg.audioFeedback.blockNum = note.block_num
      note.add(raw_audio.data)
      pm.send('audioFeedback', msg)
      if note.duration() >= VOICE_NOTE_MAX_DURATION:
        note.close()
        note = None

    # Recording status for the web dashboard: seconds left, absent when idle.
    left = -1 if note is None else max(0, int(VOICE_NOTE_MAX_DURATION - note.duration()))
    if left != last_left:
      if left < 0:
        params.remove("NAPBookmarkRecording")
      else:
        params.put("NAPBookmarkRecording", left)
      last_left = left


if __name__ == '__main__':
  main()
