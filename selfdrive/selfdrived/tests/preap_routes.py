"""Pinned Jack Pre-AP rlog segments for off-car chime replay.

Not the upstream TESLA process_replay case (that's Model Y, and the
full-brand whitelist skips selfdrived/radard). Fingerprint is the fork
Pre-AP Model S. process_replay defaults DisengageOnAccelerator=True,
which USER_DISABLEs on gas and makes b568c89384 untestable.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from openpilot.selfdrive.test.process_replay.preap_route_index import (
  JACK_DONGLE,
  RouteIndexError,
  resolve_source,
)
from openpilot.tools.lib.logreader import LogReader, ReadMode

FINGERPRINT = "TESLA_MODEL_S_PREAP"
REPLAY_PROCS = ("selfdrived", "radard")

# Must override process_replay.generate_params_config.
CUSTOM_PARAMS: dict[str, bool | int | float] = {
  "DisengageOnAccelerator": False,
  "NAPForcePreAP": True,
  "NAPPedalEnabled": True,
  "NAPRadarEnabled": True,
}


@dataclass(frozen=True)
class PinnedSegment:
  segment_name: str
  path: Path
  note: str = ""


LOGS_ROOT = Path("/home/jack/projects/personal/notautopilot/logs")
MONTANA_ROUTE = "00000119--99de680f15"
MONTANA_DIR = LOGS_ROOT / "montana-supercharger-2026-07-22-route-00000119--99de680f15"

# Real rlogs on this VPS. Do not invent names. /data is absent here.
PINNED_SEGMENTS: tuple[PinnedSegment, ...] = (
  PinnedSegment(
    segment_name=f"{JACK_DONGLE}|{MONTANA_ROUTE}--0",
    path=MONTANA_DIR / f"{MONTANA_ROUTE}--0" / "rlog.zst",
    note="montana supercharger 2026-07-22 seg 0",
  ),
  PinnedSegment(
    segment_name=f"{JACK_DONGLE}|{MONTANA_ROUTE}--1",
    path=MONTANA_DIR / f"{MONTANA_ROUTE}--1" / "rlog.zst",
    note="montana supercharger 2026-07-22 seg 1",
  ),
  PinnedSegment(
    segment_name=f"{JACK_DONGLE}|{MONTANA_ROUTE}--10",
    path=MONTANA_DIR / f"{MONTANA_ROUTE}--10" / "rlog.zst",
    note="montana supercharger 2026-07-22 seg 10",
  ),
  PinnedSegment(
    segment_name="d0cdc986c5d023f5|0000002b--fc5c96bf2e--0",
    path=LOGS_ROOT / "pedal-engagement/drive-1/d0cdc986c5d023f5_0000002b--fc5c96bf2e--0--rlog.zst",
    note="pedal-engagement drive-1",
  ),
  PinnedSegment(
    segment_name="d0cdc986c5d023f5|00000074--113899b226--28",
    path=LOGS_ROOT / "dissengagement/d0cdc986c5d023f5_00000074--113899b226--28--rlog.zst",
    note="dissengagement",
  ),
  PinnedSegment(
    segment_name="d0cdc986c5d023f5|00000005--fb95696ac5--0",
    path=LOGS_ROOT / "vdas/drive-1/d0cdc986c5d023f5_00000005--fb95696ac5--0--rlog.zst",
    note="vdas drive-1",
  ),
)


def resolve_pinned_rlog(pinned: PinnedSegment) -> Path | None:
  """Return an existing rlog path, or None if this machine does not have it."""
  if pinned.path.is_file() and "rlog" in pinned.path.name:
    return pinned.path
  try:
    found = resolve_source(pinned.segment_name, dongle=JACK_DONGLE, prefer="rlog")
  except RouteIndexError:
    return None
  rlogs = [s.path for s in found if s.kind == "rlog"]
  return rlogs[0] if rlogs else None


def load_rlog(path: Path) -> list:
  if "qlog" in path.name:
    raise ValueError(f"rlogs only; refusing {path}")
  return list(LogReader(str(path), default_mode=ReadMode.RLOG, sort_by_time=True))


def assert_replay_contract() -> None:
  if CUSTOM_PARAMS.get("DisengageOnAccelerator") is not False:
    raise RuntimeError("DisengageOnAccelerator must be False to test gas-override chimes")
  if FINGERPRINT != "TESLA_MODEL_S_PREAP":
    raise RuntimeError(f"fingerprint must be TESLA_MODEL_S_PREAP, got {FINGERPRINT}")
  if "controlsd" in REPLAY_PROCS:
    raise RuntimeError("controlsd is out of scope until preap_regen reads carControl")
