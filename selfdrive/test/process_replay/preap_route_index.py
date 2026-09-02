"""Where Jack's Pre-AP rlogs live on this machine.

Does not invent routes. It only indexes files already on disk under
known drop paths (workspace copies, comma cache, /data, backups) and
matches a query against those files.
"""
from __future__ import annotations

import os
import re
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from openpilot.tools.lib.helpers import RE

JACK_DONGLE = "d0cdc986c5d023f5"
DATA_DIR_ENV = "PREAP_OFFCAR_DATA_DIR"

_KIND_RE = re.compile(r"^(rlog|qlog)\.(zst|bz2)$")
_EXPLORER_RE = re.compile(RE.EXPLORER_FILE)
_SEGMENT_DIR_RE = re.compile(RE.OP_SEGMENT_DIR)
_ROUTE_RE = re.compile(fr"^{RE.ROUTE_NAME}$")
_SEGMENT_RE = re.compile(fr"^{RE.SEGMENT_NAME}$")
_LOG_ID_RE = re.compile(fr"^{RE.LOG_ID}$")
# Comma device dump without dongle in the folder name: 00000119--99de680f15--0/rlog.zst
_LOGID_SEGMENT_DIR_RE = re.compile(
  r"^(?P<log_id>(?:[0-9]{4}-[0-9]{2}-[0-9]{2}--[0-9]{2}-[0-9]{2}-[0-9]{2}|[a-f0-9]{8}--[a-z0-9]{10}))--(?P<segment_num>[0-9]+)$"
)
_DONGLE_IN_NAME_RE = re.compile(r"([a-f0-9]{16})")


@dataclass(frozen=True)
class LocatedSegment:
  dongle: str
  log_id: str
  segment: int
  path: Path
  kind: Literal["rlog", "qlog"]

  @property
  def route_name(self) -> str:
    return f"{self.dongle}|{self.log_id}"

  @property
  def segment_name(self) -> str:
    return f"{self.route_name}--{self.segment}"


class RouteIndexError(FileNotFoundError):
  """No matching rlog/qlog on the configured search roots."""


def default_search_roots() -> list[Path]:
  here = Path(__file__).resolve()
  repo = here.parents[3]
  candidates = []
  env = os.environ.get(DATA_DIR_ENV)
  if env:
    candidates.append(Path(env).expanduser())
  candidates.extend([
    repo / "logs",
    repo.parents[1] / "logs" if len(repo.parents) >= 2 else repo / "logs",
    Path.home() / "projects/personal/notautopilot/logs",
    Path.home() / ".commacache",
    Path("/data/media/0/realdata"),
    Path("/data"),
    Path.home() / f"backups/notautopilot/comma-{JACK_DONGLE}",
  ])
  roots: list[Path] = []
  seen: set[Path] = set()
  for raw in candidates:
    try:
      path = raw.expanduser().resolve()
    except OSError:
      continue
    if path in seen or not path.is_dir():
      continue
    seen.add(path)
    roots.append(path)
  return roots


def index_segments(roots: list[Path] | None = None, *, dongle: str | None = JACK_DONGLE) -> list[LocatedSegment]:
  found: dict[tuple[str, str, int, str], LocatedSegment] = {}
  for root in roots if roots is not None else default_search_roots():
    if not root.is_dir():
      continue
    for path in _iter_log_files(root):
      located = parse_log_path(path, default_dongle=dongle or JACK_DONGLE)
      if located is None:
        continue
      if dongle is not None and located.dongle != dongle:
        continue
      key = (located.dongle, located.log_id, located.segment, located.kind)
      existing = found.get(key)
      if existing is None or _prefer(located, existing):
        found[key] = located
  return sorted(found.values(), key=lambda s: (s.dongle, s.log_id, s.segment, s.kind))


def parse_log_path(path: Path, *, default_dongle: str = JACK_DONGLE) -> LocatedSegment | None:
  name = path.name
  explorer = _EXPLORER_RE.match(name)
  if explorer:
    kind = _kind_from_filename(explorer.group("file_name"))
    if kind is None:
      return None
    return LocatedSegment(
      dongle=explorer.group("dongle_id"),
      log_id=explorer.group("log_id"),
      segment=int(explorer.group("segment_num")),
      path=path,
      kind=kind,
    )

  if _KIND_RE.match(name):
    kind = _kind_from_filename(name)
    if kind is None:
      return None
    parent = path.parent.name
    seg_match = _SEGMENT_DIR_RE.match(parent)
    if seg_match:
      return LocatedSegment(
        dongle=seg_match.group("dongle_id"),
        log_id=seg_match.group("log_id"),
        segment=int(seg_match.group("segment_num")),
        path=path,
        kind=kind,
      )
    logid_match = _LOGID_SEGMENT_DIR_RE.match(parent)
    if logid_match:
      return LocatedSegment(
        dongle=_dongle_from_parents(path) or default_dongle,
        log_id=logid_match.group("log_id"),
        segment=int(logid_match.group("segment_num")),
        path=path,
        kind=kind,
      )
  return None


def resolve_source(
  query: str,
  *,
  roots: list[Path] | None = None,
  dongle: str | None = JACK_DONGLE,
  prefer: Literal["rlog", "qlog"] = "rlog",
) -> list[LocatedSegment]:
  """Resolve a path, route, segment, or log id to files already on disk."""
  as_path = Path(query).expanduser()
  if as_path.exists():
    if as_path.is_file():
      located = parse_log_path(as_path, default_dongle=dongle or JACK_DONGLE)
      if located is None:
        kind = _kind_from_filename(as_path.name)
        if kind is None:
          raise RouteIndexError(f"not an rlog/qlog: {as_path}")
        return [LocatedSegment(dongle=dongle or "local", log_id=as_path.stem, segment=0, path=as_path, kind=kind)]
      return [located]
    if as_path.is_dir():
      return _select_kind(index_segments([as_path], dongle=None), prefer)

  indexed = index_segments(roots, dongle=None)
  if dongle is not None:
    jack_first = [s for s in indexed if s.dongle == dongle]
    others = [s for s in indexed if s.dongle != dongle]
    indexed = jack_first + others

  matches = [s for s in indexed if _matches_query(s, query)]
  if not matches:
    roots_used = roots if roots is not None else default_search_roots()
    root_list = ", ".join(str(r) for r in roots_used) or "(none)"
    raise RouteIndexError(
      f"no rlog/qlog matched {query!r} under {root_list}. "
      + f"Drop Jack Pre-AP segments (dongle {JACK_DONGLE}) into one of those dirs or set {DATA_DIR_ENV}."
    )
  return _select_kind(matches, prefer)


def group_routes(segments: list[LocatedSegment]) -> dict[str, list[LocatedSegment]]:
  grouped: dict[str, list[LocatedSegment]] = defaultdict(list)
  for segment in segments:
    grouped[segment.route_name].append(segment)
  return dict(grouped)


def _iter_log_files(root: Path):
  for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
    dirnames[:] = [d for d in dirnames if d not in {".git", "__pycache__", ".venv", "node_modules"}]
    for filename in filenames:
      if _KIND_RE.match(filename) or _EXPLORER_RE.match(filename):
        yield Path(dirpath) / filename


def _dongle_from_parents(path: Path) -> str | None:
  for parent in path.parents:
    match = _DONGLE_IN_NAME_RE.search(parent.name)
    if match:
      return match.group(1)
  return None


def _kind_from_filename(name: str) -> Literal["rlog", "qlog"] | None:
  match = _KIND_RE.match(name)
  if match is None:
    return None
  return match.group(1)  # type: ignore[return-value]


def _prefer(new: LocatedSegment, old: LocatedSegment) -> bool:
  return new.kind == "rlog" and old.kind != "rlog"


def _select_kind(segments: list[LocatedSegment], prefer: Literal["rlog", "qlog"]) -> list[LocatedSegment]:
  by_seg: dict[tuple[str, str, int], list[LocatedSegment]] = defaultdict(list)
  for segment in segments:
    by_seg[(segment.dongle, segment.log_id, segment.segment)].append(segment)
  chosen: list[LocatedSegment] = []
  for group in by_seg.values():
    preferred = [s for s in group if s.kind == prefer]
    chosen.append((preferred or group)[0])
  return sorted(chosen, key=lambda s: (s.dongle, s.log_id, s.segment))


def _matches_query(segment: LocatedSegment, query: str) -> bool:
  q = query.replace("/", "|").strip()
  if q in {segment.segment_name, segment.route_name, segment.log_id, segment.path.name, str(segment.path)}:
    return True
  if _SEGMENT_RE.match(q):
    return q.replace("_", "|") == segment.segment_name or q == f"{segment.dongle}_{segment.log_id}--{segment.segment}"
  if _ROUTE_RE.match(q):
    return q.replace("_", "|") == segment.route_name
  if _LOG_ID_RE.match(q):
    return q == segment.log_id
  logid_seg = _LOGID_SEGMENT_DIR_RE.match(q)
  if logid_seg:
    return (
      logid_seg.group("log_id") == segment.log_id
      and int(logid_seg.group("segment_num")) == segment.segment
    )
  needle = q.replace("|", "_")
  return needle in str(segment.path) or needle in segment.segment_name.replace("|", "_")
