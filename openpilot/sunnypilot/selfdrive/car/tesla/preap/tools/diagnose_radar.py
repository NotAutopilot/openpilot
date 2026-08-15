"""Production Pre-AP Bosch radar diagnosis. --panda never changes safety mode."""
from __future__ import annotations

import argparse
import time

from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.radar_decode import (
  RADAR_ADDR_END, RADAR_ADDR_START, RADAR_BUS, unpack_can,
)
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.safety import require_offroad, require_runtime_path
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.transport import DiagnosticTransport, TransportError

GTW_MESSAGES = {
  0x109: "DI_torque1",
  0x119: "DI_torque2",
  0x129: "ESP_115h",
  0x149: "ESP_145h",
  0x159: "ESP_C (Brake)",
  0x169: "ESP_wheelSpeeds",
  0x199: "STW_ANGLHP_STAT",
  0x1A9: "DI_espControl",
  0x209: "GTW_odo",
  0x219: "STW_ACTN_RQ",
  0x2A9: "GTW_carConfig",
  0x2B9: "VIP_405HS",
  0x2D9: "BC_status",
}

RADAR_STATUS_MSGS = {
  0x631: "Radar Init Sync",
  0x300: "Radar Active",
  0x501: "Radar Alert Matrix",
}


def p(msg=""):
  print(msg, flush=True)


def diagnose_panda(transport: DiagnosticTransport, sniff_s: float = 3.0, params: Params | None = None) -> dict:
  """Read-only panda health. Must not change safety mode."""
  result = {
    "safety_mode": None,
    "bus1_count": 0,
    "bus1_addrs": set(),
    "radar_enabled": None,
    "radar_behind": None,
  }
  if params is not None:
    result["radar_enabled"] = params.get_bool("NAPRadarEnabled")
    result["radar_behind"] = params.get_bool("NAPRadarBehindNosecone")

  panda = transport.connect()
  try:
    if hasattr(panda, "get_safety_mode"):
      result["safety_mode"] = panda.get_safety_mode()
    elif hasattr(panda, "safety_model"):
      result["safety_mode"] = panda.safety_model
  except Exception:
    pass

  start = time.monotonic()
  while time.monotonic() - start < sniff_s:
    try:
      for msg in transport.can_recv():
        unpacked = unpack_can(msg)
        if unpacked is None:
          continue
        addr, _dat, src = unpacked
        if src == RADAR_BUS:
          result["bus1_count"] += 1
          result["bus1_addrs"].add(addr)
    except TransportError as exc:
      raise TransportError(f"radar diagnose RX failed: {exc}") from exc
  return result


def report_health(result: dict) -> int:
  p("Panda health (safety mode unchanged)")
  if result.get("radar_enabled") is not None:
    p(f"  NAPRadarEnabled: {result['radar_enabled']}")
    p(f"  NAPRadarBehindNosecone: {result['radar_behind']}")
    if not result["radar_enabled"]:
      p("  [!!] radar not enabled; GTW emulation will not run")
  p(f"  safety_mode={result.get('safety_mode')}")
  p(f"  bus 1 messages: {result['bus1_count']}")
  addrs = result["bus1_addrs"]
  gtw_seen = addrs & set(GTW_MESSAGES)
  radar_seen = {a for a in addrs if RADAR_ADDR_START <= a <= RADAR_ADDR_END}
  status_seen = addrs & set(RADAR_STATUS_MSGS)
  p(f"  GTW emulation: {len(gtw_seen)}/{len(GTW_MESSAGES)}")
  for addr in sorted(gtw_seen):
    p(f"    {hex(addr)} {GTW_MESSAGES[addr]}")
  p(f"  radar tracks: {'YES' if radar_seen else 'NO'}")
  p(f"  radar status: {sorted(hex(a) for a in status_seen) if status_seen else 'NONE'}")
  if result["bus1_count"] == 0:
    p("  [!!] NO TRAFFIC on bus 1")
    return 1
  return 0


def run(*, panda_mode: bool, params: Params | None = None, transport: DiagnosticTransport | None = None) -> int:
  del panda_mode  # UI and CLI both use read-only panda sniff; never change safety.
  require_runtime_path()
  params = params or Params()
  require_offroad(params)
  owned = transport is None
  transport = transport or DiagnosticTransport()
  try:
    result = diagnose_panda(transport, params=params)
    return report_health(result)
  finally:
    if owned:
      transport.close()


def main(cli_args=None) -> int:
  parser = argparse.ArgumentParser(description="Diagnose Pre-AP Bosch radar")
  parser.add_argument("--panda", action="store_true", help="Direct panda health check; do not change safety mode")
  args = parser.parse_args(cli_args)
  try:
    return run(panda_mode=args.panda)
  except Exception as exc:
    p(f"ERROR: {exc}")
    return 1


if __name__ == "__main__":
  raise SystemExit(main())
