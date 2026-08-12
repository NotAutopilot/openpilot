from __future__ import annotations

import subprocess
from pathlib import Path

import pytest


ACADOS_ROOT = Path(__file__).resolve().parents[1]


def _libraries(arch: str) -> list[Path]:
  lib_dir = ACADOS_ROOT / arch / "lib"
  if not lib_dir.is_dir():
    return []
  return sorted(
    path for path in lib_dir.iterdir()
    if path.is_file() and (path.name.endswith(".so") or ".so." in path.name)
  )


def _stack_flags(library: Path) -> str:
  result = subprocess.run(
    ["readelf", "-lW", str(library)],
    check=True,
    capture_output=True,
    text=True,
  )
  for line in result.stdout.splitlines():
    fields = line.split()
    if fields and fields[0] == "GNU_STACK":
      for field in fields[1:]:
        if set(field) <= set("RWE"):
          return field
      break
  pytest.fail(f"{library} has no GNU_STACK program header")


@pytest.mark.parametrize("arch", ("x86_64", "larch64"))
def test_tracked_libraries_request_non_executable_stack(arch: str) -> None:
  libraries = _libraries(arch)
  assert libraries, f"{arch} has no tracked acados libraries"
  for library in libraries:
    assert "E" not in _stack_flags(library), f"{library} requests an executable stack"
