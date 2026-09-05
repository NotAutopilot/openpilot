from openpilot.cereal import messaging
from openpilot.selfdrive.ui.radar.bosch_status import BoschRadarMonitor


def _live_tracks_reader(*, measured=True):
  msg = messaging.new_message("liveTracks")
  points = msg.liveTracks.init("points", 1)
  points[0].trackId = 3
  points[0].dRel = 8.0
  points[0].yRel = 0.0
  points[0].vRel = 0.1
  points[0].measured = measured
  return msg.as_reader().liveTracks


def test_live_tracks_reads_measured():
  live = _live_tracks_reader(measured=True)
  status = BoschRadarMonitor().update(0.0, [], live)
  assert status.tracks[0].track_id == 3
  assert status.tracks[0].measured is True


def test_unmeasured_live_track():
  live = _live_tracks_reader(measured=False)
  status = BoschRadarMonitor().update(0.0, [], live)
  assert status.tracks[0].track_id == 3
  assert status.tracks[0].measured is False
