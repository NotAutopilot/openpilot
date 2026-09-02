#!/usr/bin/env python3
"""Replay a Pre-AP route off-car and assert chime / long / radar contracts.

Uses existing process_replay. Does not start a simulator.

  python -m openpilot.selfdrive.test.process_replay.preap_offcar --list
  python -m openpilot.selfdrive.test.process_replay.preap_offcar --fixture
  python -m openpilot.selfdrive.test.process_replay.preap_offcar SEGMENT_OR_PATH
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from openpilot.selfdrive.test.process_replay.preap_log_contracts import (
  OffcarReport,
  evaluate_contracts,
  synthetic_drive,
)
from openpilot.selfdrive.test.process_replay.preap_route_index import (
  DATA_DIR_ENV,
  JACK_DONGLE,
  RouteIndexError,
  default_search_roots,
  group_routes,
  index_segments,
  resolve_source,
)
from openpilot.selfdrive.test.process_replay.process_replay import replay_process_with_name
from openpilot.selfdrive.test.process_replay.test_processes import NAP_PREAP_NO_PEDAL_PARAMS, NAP_PREAP_PEDAL_PARAMS
from openpilot.tools.lib.logreader import LogReader

DEFAULT_PROCS = ("card", "selfdrived", "radard")
FULL_PROCS = ("card", "controlsd", "selfdrived", "radard", "plannerd", "lagd")
FINGERPRINT = "TESLA_MODEL_S_PREAP"


def main(argv: list[str] | None = None) -> int:
  parser = argparse.ArgumentParser(
    description="Replay Jack Pre-AP logs off-car and assert pedal/chime/radar contracts.",
  )
  parser.add_argument("query", nargs="?", help="route, segment, log id, or path to rlog/qlog")
  parser.add_argument("--list", action="store_true", help=f"list indexed {JACK_DONGLE} segments and exit")
  parser.add_argument("--fixture", action="store_true", help="run assertions on the synthetic CI fixture")
  parser.add_argument("--scan-only", action="store_true", help="evaluate the recorded log without process_replay")
  parser.add_argument("--full", action="store_true", help=f"replay {', '.join(FULL_PROCS)}")
  parser.add_argument("--procs", default=None, help="comma-separated process_replay names (default: card,selfdrived,radard)")
  parser.add_argument("--data-dir", action="append", default=[], help="extra search root (repeatable)")
  parser.add_argument("--dongle", default=JACK_DONGLE, help="dongle filter for --list / resolve")
  parser.add_argument("--qlog", action="store_true", help="prefer qlogs when both exist")
  args = parser.parse_args(argv)

  extra_roots = [Path(p).expanduser() for p in args.data_dir]
  roots = extra_roots + default_search_roots() if extra_roots else None

  if args.list:
    return _print_index(roots, args.dongle)

  if args.fixture:
    msgs = synthetic_drive()
    report = evaluate_contracts(msgs, source="synthetic fixture")
    report.processes = []
    report.process_notes["fixture"] = "in-memory Pre-AP engage/override/brake/cancel"
    sys.stdout.write(report.format())
    return 1 if report.failed else 0

  if not args.query:
    parser.error("provide a route/path, or use --list / --fixture")

  try:
    segments = resolve_source(args.query, roots=roots, dongle=args.dongle, prefer="qlog" if args.qlog else "rlog")
  except RouteIndexError as exc:
    sys.stderr.write(f"{exc}\n")
    _print_drop_paths(roots)
    return 2

  paths = [str(s.path) for s in segments]
  source = segments[0].segment_name if len(segments) == 1 else f"{segments[0].route_name} ({len(segments)} segs)"
  msgs = list(LogReader(paths if len(paths) > 1 else paths[0], sort_by_time=True))
  if args.procs:
    procs = tuple(p.strip() for p in args.procs.split(",") if p.strip())
  else:
    procs = FULL_PROCS if args.full else DEFAULT_PROCS

  report: OffcarReport
  if args.scan_only:
    report = evaluate_contracts(msgs, source=source)
    report.processes = []
  else:
    report = _replay_and_evaluate(msgs, source=source, procs=procs)

  sys.stdout.write(report.format())
  sys.stdout.write("files:\n")
  for segment in segments:
    sys.stdout.write(f"  {segment.kind}  {segment.path}\n")
  if report.failed:
    return 1
  return 0


def _replay_and_evaluate(msgs: list, *, source: str, procs: tuple[str, ...]) -> OffcarReport:
  pedal_present = any(
    int(frame.address) in (0x551, 0x552)
    for msg in msgs if msg.which() == "can"
    for frame in msg.can
  )
  custom_params = NAP_PREAP_PEDAL_PARAMS if pedal_present else NAP_PREAP_NO_PEDAL_PARAMS
  captured: dict[str, dict[str, str]] = {}
  try:
    outputs = replay_process_with_name(
      list(procs),
      msgs,
      fingerprint=FINGERPRINT,
      custom_params=custom_params,
      return_all_logs=True,
      captured_output_store=captured,
      disable_progress=False,
    )
  except Exception as exc:
    report = evaluate_contracts(msgs, source=source)
    report.replay_error = f"{type(exc).__name__}: {exc}"
    report.processes = list(procs)
    report.process_notes = dict.fromkeys(procs, "replay failed; evaluated recorded log")
    return report

  report = evaluate_contracts(outputs, source=source)
  report.processes = list(procs)
  produced: dict[str, int] = {}
  for msg in outputs:
    produced[msg.which()] = produced.get(msg.which(), 0) + 1
  for proc in procs:
    cap = captured.get(proc, {})
    err = (cap.get("err") or "").strip().splitlines()
    tail = err[-1] if err else ""
    n = sum(produced.get(s, 0) for s in _subs_hint(proc))
    note = f"out~{n}"
    if tail:
      note += f"  {tail[:80]}"
    report.process_notes[proc] = note
  return report


def _subs_hint(proc: str) -> tuple[str, ...]:
  return {
    "card": ("carState", "liveTracks", "sendcan", "carParams"),
    "selfdrived": ("selfdriveState", "onroadEvents"),
    "radard": ("radarState",),
    "controlsd": ("carControl", "controlsState"),
    "plannerd": ("longitudinalPlan",),
    "lagd": ("liveDelay",),
  }.get(proc, ())


def _print_index(roots, dongle: str) -> int:
  segments = index_segments(roots, dongle=dongle)
  sys.stdout.write(f"dongle {dongle}\n")
  sys.stdout.write("search roots:\n")
  for root in (roots if roots is not None else default_search_roots()):
    sys.stdout.write(f"  {root}\n")
  if not segments:
    sys.stdout.write(f"no segments indexed. Drop rlogs into a search root or set {DATA_DIR_ENV}.\n")
    return 0
  grouped = group_routes(segments)
  unique_segs = {(s.dongle, s.log_id, s.segment) for s in segments}
  sys.stdout.write(f"{len(grouped)} routes, {len(unique_segs)} segments\n")
  for route_name, segs in grouped.items():
    kinds = ",".join(sorted({s.kind for s in segs}))
    nums = sorted({s.segment for s in segs})
    shown = ",".join(str(n) for n in nums[:12])
    extra = "" if len(nums) <= 12 else f"... +{len(nums) - 12}"
    sys.stdout.write(f"  {route_name}  {len(nums)} {kinds}  [{shown}{extra}]\n")
  return 0


def _print_drop_paths(roots) -> None:
  used = roots if roots is not None else default_search_roots()
  sys.stderr.write("drop rlog.zst files here:\n")
  if used:
    for root in used:
      sys.stderr.write(f"  {root}\n")
  else:
    sys.stderr.write(f"  set {DATA_DIR_ENV} to a directory of explorer-style rlogs\n")
    sys.stderr.write("  or copy from the device: /data/media/0/realdata\n")


if __name__ == "__main__":
  raise SystemExit(main())
