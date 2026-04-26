#!/usr/bin/env python3
"""Compare NAP-spoofed STW_ACTN_RQ frames vs driver's physical ones.

Drive-3 segment 2 has both:
  - NAP's UP_1ST spoof at t=124.97-125.5 (rejected by DI)
  - Driver's UP_1ST physical at t=127.47 (engages DI)

Diff the bytes to find why the DI accepts one but not the other.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from openpilot.tools.lib.logreader import LogReader

_LOG_ROOT = os.environ.get("LOG_DIR",
                           "/Users/jack/projects/personal/notautopilot/logs/no-pedal/drive-3")
LOG = os.path.join(_LOG_ROOT, "d0cdc986c5d023f5_0000001d--522b06a981--2--rlog.zst")


def hex_bytes(b):
  return " ".join(f"{x:02x}" for x in b)


def decode_stw(b):
  """Manually decode STW_ACTN_RQ key fields per DBC."""
  if len(b) < 8:
    return "?"
  # SpdCtrlLvr_Stat: bits 0-5 of byte 0
  lever = b[0] & 0x3F
  # MC_STW_ACTN_RQ: bits 52-55 (byte 6, top nibble per DBC bit numbering)
  mc = (b[6] >> 4) & 0x0F
  # CRC_STW_ACTN_RQ: byte 7
  crc = b[7]
  return f"lever={lever:2d} mc={mc:2d} crc={crc:02x}"


def main():
  start_ts = None
  driver_recent = []  # last few (rt, src, bytes, addr) from chassis bus
  nap_tx = []         # NAP TX of 0x45

  lr = LogReader(LOG)
  for msg in lr:
    t = msg.logMonoTime
    if start_ts is None:
      start_ts = t
    rt = (t - start_ts) / 1e9

    which = msg.which()
    if which == "can":
      for c in msg.can:
        if c.address == 69:  # 0x45
          # Source = the bus the message came in on (driver's stalk = 0)
          driver_recent.append((rt, c.src, bytes(c.dat), c.address))
    elif which == "sendcan":
      for c in msg.sendcan:
        if c.address == 69:
          nap_tx.append((rt, c.src, bytes(c.dat), c.address))

  # Find NAP TX in [124, 126] window
  nap_in_engage = [(rt, src, b) for rt, src, b, _ in nap_tx if 124 <= rt <= 126]
  print(f"=== NAP-spoofed STW_ACTN_RQ in [124, 126]s window: {len(nap_in_engage)} ===")
  for rt, src, b in nap_in_engage[:15]:
    print(f"  t+{rt:7.3f} src={src}: {hex_bytes(b)}  | {decode_stw(b)}")

  # Find driver physical UP_1ST around t=127.4
  driver_up = [(rt, src, b) for rt, src, b, _ in driver_recent
               if 127.3 <= rt <= 127.6 and (b[0] & 0x3F) == 16]
  print(f"\n=== Driver's PHYSICAL UP_1ST in [127.3, 127.6]s window: {len(driver_up)} ===")
  for rt, src, b in driver_up[:8]:
    print(f"  t+{rt:7.3f} src={src}: {hex_bytes(b)}  | {decode_stw(b)}")

  # Also sample driver's IDLE frame just before NAP's spoof for context
  driver_recent_idle = [(rt, src, b) for rt, src, b, _ in driver_recent
                        if 124.5 <= rt <= 124.95 and (b[0] & 0x3F) == 0]
  print(f"\n=== Driver's IDLE frames in [124.5, 124.95]s (just before NAP spoof): "
        f"{len(driver_recent_idle)} ===")
  for rt, src, b in driver_recent_idle[:6]:
    print(f"  t+{rt:7.3f} src={src}: {hex_bytes(b)}  | {decode_stw(b)}")


if __name__ == "__main__":
  main()
