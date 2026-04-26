#!/usr/bin/env python3
"""Replay drive-2 segment 0 through the new StockCCSpoofer + engagement FSM.

Drives the engagement FSM with the actual sequence of cruise_buttons from the
log, simulating the di_cruise_state transitions, then runs StockCCSpoofer each
frame and counts the spoof TX frames it would have produced.

If the refactor works, we should see:
  - At every driver MAIN pull from STANDBY, NAP would TX a CANCEL spoof
    after the double-pull window (assuming no second pull suppresses it).
  - At every double-pull from STANDBY where DI doesn't auto-engage, NAP
    would TX SET_ACCEL retries until DI reaches ENABLED or timeout.
"""
import os
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# openpilot.tools.lib.logreader pulls in tools.lib.api → frogpilot_utilities
# → sentry → athena.registration → common.api → frogpilot_utilities (cycle).
# Inline a minimal local-file reader to avoid the partial-init cycle.
import zstandard as zstd
from cereal import log as capnp_log

def LogReader(path):
  with open(path, 'rb') as f:
    raw = zstd.ZstdDecompressor().stream_reader(f).read()
  return list(capnp_log.Event.read_multiple_bytes(raw))

from opendbc.can import CANParser, CANPacker
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.preap.stock_cc_spoofer import StockCCSpoofer
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.car.tesla.values import CruiseButtons


# Default to drive-3 segment 0 (the no-pedal mismatch capture). LOG_DIR
# overrides the parent so the script works on any host with the logs in
# a non-default location.
_LOG_ROOT = os.environ.get("LOG_DIR",
                           "/Users/jack/projects/personal/notautopilot/logs/no-pedal/drive-3")
LOG = os.path.join(_LOG_ROOT, "d0cdc986c5d023f5_0000001d--522b06a981--0--rlog.zst")
DI_NAMES = {0: "OFF", 1: "STANDBY", 2: "ENABLED", 3: "STANDSTILL", 4: "OVERRIDE", 5: "FAULT"}
LEVER = {0: "IDLE", 1: "CANCEL", 2: "MAIN", 4: "UP_2ND", 8: "DN_2ND", 16: "UP_1ST", 32: "DN_1ST"}


def main():
  parser = CANParser("tesla_preap", [("STW_ACTN_RQ", 0), ("DI_state", 0), ("ESP_B", 0)], 0)
  eng = PreAPEngagement(double_pull_enabled=True, double_pull_window_ms=750)
  spoofer = StockCCSpoofer()

  # Real TeslaCANPreAP so the verifier can inspect actual spoofer bytes.
  packer_party = CANPacker("tesla_preap")
  packer_pt = CANPacker("tesla_preap")
  real_can = TeslaCANPreAP({0: packer_party, 2: packer_pt})

  prev_lever = 0
  start_ts = None
  frame = 0
  fired = []  # (rt, button)

  lr = LogReader(LOG)
  for msg in lr:
    if msg.which() != "can":
      continue
    t = msg.logMonoTime
    if start_ts is None:
      start_ts = t
    rt = (t - start_ts) / 1e9

    frames_can = [(c.address, c.dat, c.src) for c in msg.can]
    parser.update([(t, frames_can)])

    try:
      lever = int(parser.vl["STW_ACTN_RQ"]["SpdCtrlLvr_Stat"])
      di = int(parser.vl["DI_state"]["DI_cruiseState"])
      vkph = float(parser.vl["ESP_B"]["ESP_vehicleSpeed"])
    except Exception:
      continue

    di_name = DI_NAMES.get(di, "OFF")
    msg_stw = {"MC_STW_ACTN_RQ": 3, "DTR_Dist_Rq": 255}

    # Drive engagement FSM with this frame's button state
    eng.process_buttons(
      lever, prev_lever, int(rt * 1000),
      v_ego=vkph * 0.27778, speed_units="MPH",
      use_pedal=False, pedal_long_allowed=False,
      long_control_allowed=True, real_brake_pressed=False,
      di_cruise_state=di_name)

    # Bridge to a synthetic CarState the spoofer reads
    cs = SimpleNamespace(
      preap_cc_cancel_needed=eng.preap_cc_cancel_needed,
      preap_cc_engage_needed=eng.preap_cc_engage_needed,
      di_cruise_state=di_name,
      msg_stw_actn_req=msg_stw,
    )

    sends = spoofer.update(cs, frame, real_can, can_bus_party=2)
    for addr, dat, _bus in sends:
      if addr != 0x45:  # only STW_ACTN_RQ
        continue
      btn = dat[0] & 0x3F
      btn_name = "CANCEL" if btn == CruiseButtons.CANCEL else "SET_ACCEL" if btn == CruiseButtons.SET_ACCEL else f"btn={btn}"
      fired.append((rt, btn_name, dat[0]))

    prev_lever = lever
    frame += 1

  print(f"\n  total NAP stalk-spoof TX (would have fired): {len(fired)}")
  if fired:
    print(f"  by type: CANCEL={sum(1 for _, b, _ in fired if b=='CANCEL')}  SET_ACCEL={sum(1 for _, b, _ in fired if b=='SET_ACCEL')}")
    print(f"  first 30:")
    for rt, b, byte0 in fired[:30]:
      # gate-11 invariant: VSL_Enbl_Rq=1 → byte0 high nibble has bit 6 set
      # → 0x40 | (lever bits 0..5). CANCEL=0x41, SET_ACCEL/UP_1ST=0x50.
      print(f"    t+{rt:7.3f}  {b}  byte0=0x{byte0:02x} NAP")


if __name__ == "__main__":
  main()
