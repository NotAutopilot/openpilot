"""Production Pre-AP radar calibration display. Receive-only, no safety bypass."""
from __future__ import annotations

import time

from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.radar_decode import (
  MAX_LAT, MAX_LONG, MIN_LAT, MIN_LONG, in_calibration_window, unpack_can, update_tracks,
)
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.safety import parse_explicit_confirmation, require_preap_tool_start
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.transport import DiagnosticTransport, TransportError

DISPLAY_INTERVAL = 0.5


def p(msg=""):
  print(msg, flush=True)


def collect_calibration_points(messages, tracks: dict[int, dict] | None = None) -> list[dict]:
  tracks = tracks if tracks is not None else {}
  for msg in messages:
    unpacked = unpack_can(msg)
    if unpacked is None:
      continue
    addr, dat, _src = unpacked
    update_tracks(tracks, addr, dat)
  return [pt for pt in tracks.values() if in_calibration_window(pt)]


def run(*, confirmed: bool, params: Params | None = None, transport: DiagnosticTransport | None = None,
        duration_s: float | None = None) -> int:
  params = params or Params()
  require_preap_tool_start(params, tool="calibrate_radar", confirmed=confirmed)
  owned = transport is None
  transport = transport or DiagnosticTransport()
  try:
    transport.connect()
    p("Radar calibration (receive-only, safety mode unchanged)")
    p(f"Place a target {MIN_LONG:.1f}-{MAX_LONG:.1f}m ahead, lateral {MIN_LAT:.1f} to {MAX_LAT:.1f}m.")
    p("Ctrl+C to stop.")
    tracks: dict[int, dict] = {}
    last_display = 0.0
    deadline = None if duration_s is None else (time.monotonic() + duration_s)
    while deadline is None or time.monotonic() < deadline:
      try:
        points = collect_calibration_points(transport.can_recv(), tracks)
      except TransportError as exc:
        raise TransportError(f"radar calibration RX failed: {exc}") from exc
      now = time.monotonic()
      if now - last_display >= DISPLAY_INTERVAL:
        last_display = now
        if points:
          for pt in points:
            p(f"  d={pt['long_dist']:.2f}m y={pt['lat_dist']:.2f}m v={pt.get('long_speed', 0):.1f}")
        else:
          p("  [no tracks in calibration window]")
      time.sleep(0.05)
  except KeyboardInterrupt:
    p("\nStopped.")
  finally:
    if owned:
      transport.close()
  return 0


def main(argv=None) -> int:
  try:
    return run(confirmed=parse_explicit_confirmation(argv))
  except Exception as exc:
    p(f"ERROR: {exc}")
    return 1


if __name__ == "__main__":
  raise SystemExit(main())
