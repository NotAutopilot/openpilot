"""Offroad, confirmation, and runtime-path gates for Pre-AP production tools."""
from __future__ import annotations

from pathlib import Path

from openpilot.common.params import Params

DESTRUCTIVE_TOOLS = frozenset({
  "calibrate_pedal",
  "calibrate_radar",
  "flash_epas",
  "restore_epas",
})

TOOLS_DIR = Path(__file__).resolve().parent


class ToolSafetyError(Exception):
  """Tool refused to run because a safety gate failed."""


def require_runtime_path(module_file: str | Path | None = None) -> Path:
  """Production tools must execute from this package under BASEDIR."""
  from openpilot.common.basedir import BASEDIR
  tools_dir = TOOLS_DIR
  base = Path(BASEDIR).resolve()
  if base not in tools_dir.parents:
    raise ToolSafetyError(f"tools must run from BASEDIR ({base}), not {tools_dir}")
  if module_file is not None:
    path = Path(module_file).resolve()
    if path.parent != tools_dir and tools_dir not in path.parents:
      raise ToolSafetyError(f"tool must run from production path {tools_dir}")
    return path
  return tools_dir


def require_offroad(params: Params | None = None) -> None:
  params = params or Params()
  if not params.get_bool("IsOffroad"):
    raise ToolSafetyError("tool requires offroad")


def require_confirmation(confirmed: bool, *, tool: str) -> None:
  if tool in DESTRUCTIVE_TOOLS and not confirmed:
    raise ToolSafetyError("destructive tool requires explicit confirmation")


def require_preap_tool_start(params: Params | None = None, *, tool: str, confirmed: bool) -> None:
  require_runtime_path()
  require_offroad(params)
  require_confirmation(confirmed, tool=tool)


def parse_explicit_confirmation(argv=None) -> bool:
  """Return True only when this invocation includes an explicit --confirm flag.

  Absence, empty argv, and default argparse False cannot silently satisfy
  destructive tools. Callers must pass the flag after a real user acknowledgment.
  """
  import argparse
  parser = argparse.ArgumentParser(add_help=False)
  parser.add_argument("--confirm", action="store_true", default=False)
  args, _unknown = parser.parse_known_args(argv)
  return bool(args.confirm)
