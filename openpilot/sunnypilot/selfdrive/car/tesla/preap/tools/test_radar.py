"""Production Pre-AP radar installation test. Receive-only, no safety bypass."""
from __future__ import annotations

import time

from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.radar_decode import (
  MOVING_STATES, unpack_can, update_tracks,
)
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.safety import require_offroad, require_runtime_path
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.transport import DiagnosticTransport, TransportError

DISPLAY_INTERVAL = 0.5


def p(msg=""):
  print(msg, flush=True)


def collect_tracks(messages, tracks: dict[int, dict] | None = None) -> dict[int, dict]:
  tracks = tracks if tracks is not None else {}
  for msg in messages:
    unpacked = unpack_can(msg)
    if unpacked is None:
      continue
    addr, dat, _src = unpacked
    update_tracks(tracks, addr, dat)
  return tracks


def run(*, params: Params | None = None, transport: DiagnosticTransport | None = None,
        duration_s: float | None = None) -> int:
  params = params or Params()
  require_runtime_path()
  require_offroad(params)
  owned = transport is None
  transport = transport or DiagnosticTransport()
  try:
    transport.connect()
    p("Radar test (receive-only, safety mode unchanged)")
    p("Ctrl+C to stop.")
    tracks: dict[int, dict] = {}
    last_display = 0.0
    seen = 0
    deadline = None if duration_s is None else (time.monotonic() + duration_s)
    while deadline is None or time.monotonic() < deadline:
      try:
        collect_tracks(transport.can_recv(), tracks)
      except TransportError as exc:
        raise TransportError(f"radar test RX failed: {exc}") from exc
      now = time.monotonic()
      if now - last_display >= DISPLAY_INTERVAL:
        last_display = now
        live = 0
        for idx, pt in tracks.items():
          if "long_dist" not in pt:
            continue
          live += 1
          seen += 1
          moving = MOVING_STATES.get(pt.get("moving_state", 0), "Unknown")
          p(f"  #{idx} d={pt['long_dist']:.2f}m y={pt.get('lat_dist', 0):.2f}m {moving}")
        if live == 0:
          p("  [no radar tracks yet]")
      time.sleep(0.05)
    if duration_s is not None and seen == 0:
      p("  [!!] no radar tracks")
      return 1
  except KeyboardInterrupt:
    p("\nStopped.")
  finally:
    if owned:
      transport.close()
  return 0


def main() -> int:
  try:
    return run()
  except Exception as exc:
    p(f"ERROR: {exc}")
    return 1


if __name__ == "__main__":
  raise SystemExit(main())
