import re
from pathlib import Path


WORKFLOW_PATH = Path(__file__).resolve().parents[1] / "workflows" / "tests.yaml"
PROCESS_REPLAY_PATH = Path(__file__).resolve().parents[2] / "selfdrive" / "test" / "process_replay" / "process_replay.py"


def indented_block(document: str, header: str) -> str:
  lines = document.splitlines()
  header_index = lines.index(header)
  header_indent = len(header) - len(header.lstrip())
  block = []
  for line in lines[header_index + 1:]:
    line_indent = len(line) - len(line.lstrip())
    if line.strip() and line_indent <= header_indent:
      break
    block.append(line)
  return "\n".join(block)


def normalized_lines(block: str) -> set[str]:
  return {line.strip().removesuffix("\\").rstrip() for line in block.splitlines()}


def test_nap_ci_uses_fork_lfs_storage():
  workflow = WORKFLOW_PATH.read_text()

  assert "OPENPILOT_LFS_URL: https://github.com/${{ github.repository }}.git/info/lfs" in normalized_lines(workflow)
  assert 'command: git config lfs.url "$OPENPILOT_LFS_URL" && git lfs pull' in normalized_lines(workflow)


def test_nap_branches_run_on_push():
  workflow = WORKFLOW_PATH.read_text()
  push_config = indented_block(workflow, "  push:")
  push_branches = indented_block(push_config, "    branches:")

  assert re.search(r"^\s*-\s+['\"]?nap-\*['\"]?\s*$", push_branches, re.MULTILINE)


def test_named_focused_job_is_present():
  workflow = WORKFLOW_PATH.read_text()
  focused_job = indented_block(workflow, "  tesla_preap_longitudinal_regression:")

  assert "name: Tesla Pre-AP longitudinal regressions" in normalized_lines(focused_job)


def test_focused_tests_and_mutations_are_pinned():
  workflow = WORKFLOW_PATH.read_text()
  focused_job = indented_block(workflow, "  tesla_preap_longitudinal_regression:")
  required_commands = (
    "selfdrive/controls/tests/test_following_distance.py",
    "selfdrive/controls/tests/test_tesla_preap_following.py",
    "selfdrive/controls/tests/test_tesla_preap_longcontrol.py",
    "opendbc_repo/opendbc/car/tesla/preap/tests/test_longitudinal_tuning.py",
    "opendbc_repo/opendbc/car/tesla/preap/tests/test_virtual_das.py",
    "opendbc_repo/opendbc/car/tesla/preap/tests/test_vdas_grade_control.py",
  )

  for command in required_commands:
    assert command in normalized_lines(focused_job)
  assert "run: python .github/ci/tesla_preap_longitudinal_mutations.py" in normalized_lines(focused_job)


def test_focused_job_builds_generated_mpc_dependencies():
  workflow = WORKFLOW_PATH.read_text()
  focused_job = indented_block(workflow, "  tesla_preap_longitudinal_regression:")
  build_step = indented_block(focused_job, "    - name: Build focused test dependencies")

  assert "run: scons -j$(nproc)" in normalized_lines(build_step)


def test_focused_job_cannot_be_skipped_or_soft_failed():
  workflow = WORKFLOW_PATH.read_text()
  focused_job = indented_block(workflow, "  tesla_preap_longitudinal_regression:")

  assert not re.search(r"^\s*(?:if|continue-on-error)\s*:", focused_job, re.MULTILINE)


def test_additive_follow_telemetry_is_ignored_by_process_replay():
  process_replay = PROCESS_REPLAY_PATH.read_text()
  plannerd_config = re.search(r'proc_name="plannerd",(?P<body>.*?)\n  \),', process_replay, re.DOTALL)
  assert plannerd_config is not None

  assert '"longitudinalPlan.napFollowDistance"' in plannerd_config.group("body")
  assert '"longitudinalPlan.tFollow"' in plannerd_config.group("body")


def main():
  test_nap_branches_run_on_push()
  test_nap_ci_uses_fork_lfs_storage()
  test_named_focused_job_is_present()
  test_focused_tests_and_mutations_are_pinned()
  test_focused_job_builds_generated_mpc_dependencies()
  test_focused_job_cannot_be_skipped_or_soft_failed()
  test_additive_follow_telemetry_is_ignored_by_process_replay()
  print("Tesla Pre-AP longitudinal workflow contract passed")


if __name__ == "__main__":
  main()
