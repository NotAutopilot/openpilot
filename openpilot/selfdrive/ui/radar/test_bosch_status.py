from types import SimpleNamespace

from openpilot.cereal import messaging
from openpilot.selfdrive.ui.radar.bosch_status import BoschRadarMonitor


def test_live_tracks_uses_deprecated_measured():
  msg = messaging.new_message("liveTracks")
  points = msg.liveTracks.init("points", 1)
  points[0].trackId = 7
  points[0].dRel = 12.0
  points[0].yRel = 0.25
  points[0].vRel = -0.5
  points[0].deprecated.measured = True

  status = BoschRadarMonitor().update(0.0, [], msg.liveTracks)
  assert len(status.tracks) == 1
  assert status.tracks[0].track_id == 7
  assert status.tracks[0].measured is True


def test_missing_measured_does_not_crash_ui():
  point = SimpleNamespace(trackId=3, dRel=8.0, yRel=0.0, vRel=0.1)
  tracks = SimpleNamespace(points=(point,), errors=None)

  status = BoschRadarMonitor().update(0.0, [], tracks)
  assert status.tracks[0].track_id == 3
  assert status.tracks[0].measured is False
