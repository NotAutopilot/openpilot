#!/usr/bin/env python3
"""Render the radar HUD and monitor dialog to PNGs for visual check."""

from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("OFFSCREEN", "1")
os.environ.setdefault("BIG", "1")
os.environ.setdefault("SCALE", "1")

from dataclasses import replace

from openpilot.selfdrive.ui.radar.bosch_status import BoschRadarStatus, RadarTrack
from openpilot.selfdrive.ui.radar.radar_view import RadarHudOverlay, RadarMonitorDialog, radar_hud_rect
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app
import pyray as rl


def _status(label_tracks=True) -> BoschRadarStatus:
  tracks = (
    RadarTrack(12, 18.4, -0.31, -1.2, True),
    RadarTrack(15, 32.0, 1.10, -0.4, True),
    RadarTrack(19, 47.5, -2.80, 0.1, False),
    RadarTrack(22, 61.0, 0.20, -3.4, True),
  ) if label_tracks else ()
  return BoschRadarStatus(
    tracks=tracks,
    hw_fail=False,
    sgu_fail=True,
    dirty=False,
    alerts=("vinValidity", "xwdValidity", "radPositionMismatch"),
    table_frozen=True,
    awd=True,
    position=0,
    epas_type=2,
    vin="5YJSA1E45FF108485",
    unique_raw=2,
    gtw_live=True,
    vin_stream_complete=True,
    last_raw_age_s=0.08,
    vin_f190="5YJSA1E45FF108485",
    vin_chassis="5YJSA1H15EFP37440",
  )


def main() -> None:
  out = Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/radar-ui")
  out.mkdir(parents=True, exist_ok=True)

  gui_app.init_window("radar preview")
  ui_state.radar_hud = True
  ui_state.radar_status = _status()

  overlay = RadarHudOverlay()
  dialog = RadarMonitorDialog()
  hud = radar_hud_rect(rl.Rectangle(0, 0, gui_app.width, gui_app.height))

  rl.begin_drawing()
  rl.clear_background(rl.Color(30, 40, 50, 255))
  rl.draw_rectangle(0, 0, gui_app.width, 120, rl.Color(20, 20, 20, 255))
  overlay.render(hud)
  rl.end_drawing()
  overlay_path = str(out / "radar_hud.png")
  rl.take_screenshot(overlay_path)

  rl.begin_drawing()
  dialog.render(rl.Rectangle(0, 0, gui_app.width, gui_app.height))
  rl.end_drawing()
  dialog_path = str(out / "radar_monitor.png")
  rl.take_screenshot(dialog_path)

  ui_state.radar_status = replace(
    _status(),
    sgu_fail=False,
    alerts=(),
    table_frozen=False,
    awd=False,
    position=1,
    epas_type=0,
    vin="5YJSA1S11EFP54129",
    unique_raw=14,
    last_raw_age_s=0.04,
    vin_f190="5YJSA1E25FF106153",
    vin_chassis="5YJSA1H13EFP20460",
  )
  rl.begin_drawing()
  rl.clear_background(rl.Color(30, 40, 50, 255))
  overlay.render(hud)
  rl.end_drawing()
  live_path = str(out / "radar_hud_live.png")
  rl.take_screenshot(live_path)

  gui_app.close()
  print(overlay_path)
  print(dialog_path)
  print(live_path)


if __name__ == "__main__":
  main()
