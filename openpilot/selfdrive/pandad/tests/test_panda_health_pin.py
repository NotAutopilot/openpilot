"""Guard: pandad.cc health_t members must exist on the committed panda gitlink."""
from __future__ import annotations

import re
import subprocess
from pathlib import Path

HEALTH_USE_RE = re.compile(r"\bhealth\.([A-Za-z_][A-Za-z0-9_]*)")
HEALTH_T_RE = re.compile(
  r"struct\s+(?:__attribute__\(\(packed\)\)\s+)?health_t\s*\{(.*?)\n\};",
  re.S,
)
HEALTH_MEMBER_RE = re.compile(r"^\s*(?:u?int\d+_t|float|bool)\s+(\w+)\s*;", re.M)


def _repo_root() -> Path:
  return Path(subprocess.check_output(
    ["git", "rev-parse", "--show-toplevel"], text=True,
  ).strip())


def _pandad_cc(root: Path) -> Path:
  for candidate in (
    root / "openpilot/selfdrive/pandad/pandad.cc",
    root / "selfdrive/pandad/pandad.cc",
  ):
    if candidate.is_file():
      return candidate
  raise FileNotFoundError("pandad.cc not found")


def _panda_gitlink_sha(root: Path) -> str:
  line = subprocess.check_output(
    ["git", "ls-tree", "HEAD", "panda"], cwd=root, text=True,
  ).strip()
  parts = line.split()
  if len(parts) < 3 or parts[1] != "commit":
    raise AssertionError(f"HEAD panda gitlink is not a commit: {line!r}")
  return parts[2]


def _panda_checkout_sha(root: Path) -> str:
  return subprocess.check_output(
    ["git", "rev-parse", "HEAD"], cwd=root / "panda", text=True,
  ).strip()


def _health_h_at_gitlink(root: Path, sha: str) -> str:
  return subprocess.check_output(
    ["git", "show", f"{sha}:board/health.h"], cwd=root / "panda", text=True,
  )


def _health_t_members(health_h: str) -> set[str]:
  match = HEALTH_T_RE.search(health_h)
  if match is None:
    raise AssertionError("health_t struct not found in panda/board/health.h")
  return set(HEALTH_MEMBER_RE.findall(match.group(1)))


def test_checked_out_panda_matches_gitlink():
  root = _repo_root()
  gitlink = _panda_gitlink_sha(root)
  checkout = _panda_checkout_sha(root)
  msg = f"checked-out panda {checkout} != git ls-tree HEAD panda {gitlink}; run: git submodule update --init panda"
  assert checkout == gitlink, msg


def test_pandad_health_fields_exist_on_gitlink():
  root = _repo_root()
  gitlink = _panda_gitlink_sha(root)
  used = set(HEALTH_USE_RE.findall(_pandad_cc(root).read_text()))
  assert used, "pandad.cc has no health.MEMBER uses"
  members = _health_t_members(_health_h_at_gitlink(root, gitlink))
  missing = sorted(used - members)
  msg = f"pandad.cc uses health_t fields missing from panda {gitlink[:12]} board/health.h: {missing}"
  assert not missing, msg
