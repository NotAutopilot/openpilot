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
    "selfdrive/controls/tests/test_follow_distance.py",
    "selfdrive/controls/tests/test_lead_approach.py",
    "selfdrive/controls/tests/test_firm_brake_latch.py",
    "selfdrive/selfdrived/tests/test_preap_regen_cmd_pin.py",
    "selfdrive/controls/tests/test_unified_far_gap_gate.py",
    "selfdrive/controls/tests/test_unified_far_gap_catchup.py",
    "selfdrive/mapd/tests/test_map_speed_policy.py",
    "selfdrive/mapd/tests/test_map_speed_policy_2.py",
    "selfdrive/mapd/tests/test_map_match_road_class.py",
    "selfdrive/controls/tests/test_radard.py",
    "selfdrive/controls/tests/test_path_obstacle_radar.py",
    "selfdrive/controls/lib/tests/test_path_obstacle.py",
    "selfdrive/controls/tests/test_radar_path_gate.py",
    "selfdrive/controls/tests/test_rain_radar_hold.py",
    "selfdrive/controls/tests/test_radar_sensor_dirty.py",
    "selfdrive/controls/tests/test_tesla_preap_following.py",
    "selfdrive/controls/tests/test_gap_lock_gesture.py",
    "selfdrive/controls/tests/test_gap_lock_latch.py",
    "selfdrive/controls/tests/test_curve_preview.py",
    "selfdrive/controls/tests/test_curve_follow.py",
    "selfdrive/controls/tests/test_curve_follow_planner.py",
    "selfdrive/controls/tests/test_sight_cap_planner.py",
    "selfdrive/controls/tests/test_sight_cap_replay.py",
    "selfdrive/controls/tests/test_model_sight.py",
    "selfdrive/car/tests/test_curve_follow_card_max.py",
    "selfdrive/controls/tests/test_lead_leaving.py",
    "selfdrive/controls/lib/tests/test_curve_max_hold.py",
    "selfdrive/controls/lib/tests/test_lat_turn_geometry.py",
    "selfdrive/controls/lib/tests/test_lane_change_confirm_replay.py",
    "selfdrive/controls/lib/tests/test_lane_change_nudge.py",
    "selfdrive/controls/lib/tests/test_lane_change_target.py",
    "selfdrive/controls/lib/tests/test_lane_change_lock.py",
    "selfdrive/controls/lib/tests/test_lane_change_turn.py",
    "selfdrive/mapd/tests/test_roundabout.py",
    "selfdrive/mapd/tests/test_roundabout_map.py",
    "selfdrive/controls/lib/tests/test_roundabout_guide.py",
    "selfdrive/controls/lib/tests/test_roundabout_ring_hold.py",
    "selfdrive/controls/lib/tests/test_driver_lateral_handoff_early_yield.py",
    "selfdrive/controls/lib/tests/test_driver_lateral_handoff_wheel_resume.py",
    "selfdrive/controls/lib/tests/test_lat_yield_inference_guard.py",
    "selfdrive/car/tesla/tests/test_preap_lat_yield.py",
    "selfdrive/controls/tests/test_roundabout_planner.py",
    "selfdrive/controls/tests/test_tesla_preap_longcontrol.py",
    "selfdrive/controls/tests/test_tesla_preap_gas_lift_handoff.py",
    "selfdrive/controls/tests/test_tesla_preap_brake_cancel_regen.py",
    "selfdrive/controls/tests/test_tesla_preap_one_pedal_long.py",
    "opendbc_repo/opendbc/car/tesla/preap/tests/test_preap_engagement.py",
    "opendbc_repo/opendbc/car/tesla/preap/tests/test_lat_yield.py",
    "opendbc_repo/opendbc/car/tesla/preap/tests/test_longitudinal_tuning.py",
    "opendbc_repo/opendbc/car/tesla/preap/tests/test_virtual_das.py",
    "opendbc_repo/opendbc/car/tesla/preap/tests/test_vdas_grade_control.py",
  )

  for command in required_commands:
    assert command in normalized_lines(focused_job)
  assert "run: python .github/ci/tesla_preap_longitudinal_mutations.py" in normalized_lines(focused_job)
  assert "run: python .github/ci/tesla_preap_roundabout_mutations.py" in normalized_lines(focused_job)
  assert "run: python .github/ci/tesla_preap_lane_change_mutations.py" in normalized_lines(focused_job)
  assert "run: python .github/ci/tesla_preap_lat_yield_mutations.py" in normalized_lines(focused_job)


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
