import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
LONGCONTROL_TEST_PATH = "selfdrive/controls/tests/test_tesla_preap_longcontrol.py"
FOLLOWING_TEST_PATH = "selfdrive/controls/tests/test_tesla_preap_following.py"
NOISE_GATE_TEST_NODE = (
  "opendbc_repo/opendbc/car/tesla/preap/tests/test_virtual_das.py::TestInnerPID::" +
  "test_sub_deadband_sign_changing_noise_does_not_accumulate_residual_authority"
)


@dataclass(frozen=True)
class HistoricalMutation:
  name: str
  source_path: str
  original: bytes
  replacement: bytes
  test_nodes: tuple[str, ...]


MUTATIONS = (
  HistoricalMutation(
    name="historical-outer-ki",
    source_path="opendbc_repo/opendbc/car/tesla/preap/constants.py",
    original=b"PEDAL_LONG_KI_V = [0.0, 0.0, 0.0, 0.0]\n",
    replacement=b"PEDAL_LONG_KI_V = [0.05, 0.08, 0.10, 0.15]\n",
    test_nodes=(
      f"{LONGCONTROL_TEST_PATH}::test_vdas_receives_route_shaped_planner_target_trace_unchanged",
      f"{LONGCONTROL_TEST_PATH}::test_road_load_history_cannot_reverse_finite_jerk_negative_planner_target",
      f"{LONGCONTROL_TEST_PATH}::test_negative_planner_target_reaches_regen_side_of_coast_anchor",
    ),
  ),
  HistoricalMutation(
    name="adaptive-follow-cap-bypassed",
    source_path="selfdrive/controls/lib/longitudinal_planner.py",
    original=(
      b"        cap_strength = get_preap_follow_cap_strength(" +
      b"v_ego, lead.dRel, lead.vLead, self.t_follow)\n"
    ),
    replacement=b"        cap_strength = 0.0\n",
    test_nodes=(
      f"{FOLLOWING_TEST_PATH}::" +
      "test_planner_adaptive_cap_changes_the_delivered_acceleration_for_unequal_speed_lead",
    ),
  ),
  HistoricalMutation(
    name="longcontrol-feedforward-coupling-bypassed",
    source_path="selfdrive/controls/lib/longcontrol.py",
    original=b"                                     feedforward=a_target)\n",
    replacement=b"                                     feedforward=0.0)\n",
    test_nodes=(
      f"{FOLLOWING_TEST_PATH}::test_max_follow_full_closed_loop_recovers_gap_with_production_fallback",
    ),
  ),
  HistoricalMutation(
    name="hard-inner-error-deadband-restored",
    source_path="opendbc_repo/opendbc/car/tesla/preap/virtual_das.py",
    original=b"    error = self._gate_pid_error_noise(error, freeze_integrator)\n",
    replacement=(
      b"    if abs(error) < PID_ERROR_DEADBAND:\n" +
      b"      error = 0.0\n"
    ),
    test_nodes=(
      f"{FOLLOWING_TEST_PATH}::test_max_follow_full_closed_loop_recovers_gap_with_production_fallback",
    ),
  ),
  HistoricalMutation(
    name="inner-error-noise-gate-call-bypassed",
    source_path="opendbc_repo/opendbc/car/tesla/preap/virtual_das.py",
    original=b"    error = self._gate_pid_error_noise(error, freeze_integrator)\n",
    replacement=b"    error = error\n",
    test_nodes=(NOISE_GATE_TEST_NODE,),
  ),
  HistoricalMutation(
    name="negative-handoff-integral-slew-regressed",
    source_path="opendbc_repo/opendbc/car/tesla/preap/virtual_das.py",
    original=b"NEGATIVE_HANDOFF_INTEGRAL_SLEW = 0.25  # m/s\xc2\xb3\n",
    replacement=b"NEGATIVE_HANDOFF_INTEGRAL_SLEW = 0.20  # m/s\xc2\xb3\n",
    test_nodes=(
      f"{LONGCONTROL_TEST_PATH}::test_negative_planner_target_reaches_regen_side_of_coast_anchor",
    ),
  ),
  HistoricalMutation(
    name="grade-effort-compensation-removed",
    source_path="opendbc_repo/opendbc/car/tesla/preap/virtual_das.py",
    original=b"      a_limited + steady_grade_compensation + transient_pitch_compensation,\n",
    replacement=b"      a_limited,\n",
    test_nodes=(
      f"{FOLLOWING_TEST_PATH}::test_plant_aligned_full_closed_loop_grade_compensation_holds_speed",
    ),
  ),
  HistoricalMutation(
    name="grade-effort-compensation-sign-flipped",
    source_path="opendbc_repo/opendbc/car/tesla/preap/virtual_das.py",
    original=b"      a_limited + steady_grade_compensation + transient_pitch_compensation,\n",
    replacement=b"      a_limited - steady_grade_compensation - transient_pitch_compensation,\n",
    test_nodes=(
      f"{FOLLOWING_TEST_PATH}::test_plant_aligned_full_closed_loop_grade_compensation_holds_speed",
    ),
  ),
  HistoricalMutation(
    name="grade-effort-compensation-doubled",
    source_path="opendbc_repo/opendbc/car/tesla/preap/virtual_das.py",
    original=b"      a_limited + steady_grade_compensation + transient_pitch_compensation,\n",
    replacement=(
      b"      a_limited + 2.0 * steady_grade_compensation " +
      b"+ 2.0 * transient_pitch_compensation,\n"
    ),
    test_nodes=(
      f"{FOLLOWING_TEST_PATH}::test_plant_aligned_full_closed_loop_grade_compensation_holds_speed",
    ),
  ),
)


class JUnitReportError(RuntimeError):
  pass


def run_pytest(test_nodes: tuple[str, ...], junit_path: Path) -> subprocess.CompletedProcess[str]:
  environment = os.environ.copy()
  environment["PYTHONDONTWRITEBYTECODE"] = "1"
  environment["PYTHONPATH"] = str(REPO_ROOT)
  return subprocess.run(
    [
      sys.executable, "-m", "pytest", "-q", "-n", "0", "-p", "no:cacheprovider",
      f"--junitxml={junit_path}", *test_nodes,
    ],
    cwd=REPO_ROOT,
    env=environment,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    check=False,
  )


def junit_testcases(junit_path: Path) -> list[ET.Element]:
  try:
    return list(ET.parse(junit_path).iter("testcase"))
  except (OSError, ET.ParseError) as exc:
    raise JUnitReportError(f"cannot read {junit_path.name}: {exc}") from exc


def has_only_assertion_failures(testcases: list[ET.Element]) -> bool:
  failures = [failure for testcase in testcases for failure in testcase.findall("failure")]
  errors = [error for testcase in testcases for error in testcase.findall("error")]
  return (
    bool(failures)
    and not errors
    and all(
      (failure.get("type") or "").endswith("AssertionError")
      or (failure.get("message") or "").startswith("AssertionError:")
      or "AssertionError" in (failure.text or "")
      for failure in failures
    )
  )


def apply_mutation(mutation: HistoricalMutation) -> tuple[Path, bytes]:
  source_path = REPO_ROOT / mutation.source_path
  original_source = source_path.read_bytes()
  match_count = original_source.count(mutation.original)
  if match_count != 1:
    raise RuntimeError(
      f"{mutation.name}: expected one source match in {mutation.source_path}, found {match_count}"
    )
  source_path.write_bytes(original_source.replace(mutation.original, mutation.replacement, 1))
  return source_path, original_source


def main() -> int:
  with tempfile.TemporaryDirectory(prefix="tesla-preap-parent-mutation-") as temp_dir:
    temp_root = Path(temp_dir)
    baseline_xml = temp_root / "baseline.xml"
    baseline = run_pytest(
      (LONGCONTROL_TEST_PATH, FOLLOWING_TEST_PATH, NOISE_GATE_TEST_NODE),
      baseline_xml,
    )
    if baseline.returncode != 0:
      print("BASELINE FAILED: parent longitudinal regression tests did not pass")
      print(baseline.stdout)
      return 1
    try:
      baseline_testcases = junit_testcases(baseline_xml)
    except JUnitReportError as exc:
      print(f"BASELINE INVALID: {exc}")
      return 1
    print(f"BASELINE PASS: {len(baseline_testcases)} parent tests")

    survivors = []
    for mutation in MUTATIONS:
      source_path = None
      original_source = None
      mutation_result = None
      mutation_error = None
      restored = False
      try:
        source_path, original_source = apply_mutation(mutation)
        mutation_result = run_pytest(
          mutation.test_nodes,
          temp_root / f"{mutation.name}.xml",
        )
      except Exception as exc:  # pragma: no cover - failure reporting path
        mutation_error = exc
      finally:
        if source_path is not None and original_source is not None:
          source_path.write_bytes(original_source)
          restored = source_path.read_bytes() == original_source

      if not restored:
        print(f"INVALID: {mutation.name} source restoration was not byte-identical")
        return 1
      if mutation_error is not None:
        print(f"INVALID: {mutation.name} could not run: {mutation_error}")
        return 1
      if mutation_result is None:
        print(f"INVALID: {mutation.name} produced no pytest result")
        return 1

      mutation_xml = temp_root / f"{mutation.name}.xml"
      try:
        mutation_testcases = junit_testcases(mutation_xml)
      except JUnitReportError as exc:
        print(f"INVALID: {mutation.name} {exc}")
        return 1
      if mutation_result.returncode == 1 and has_only_assertion_failures(mutation_testcases):
        print(f"KILLED: {mutation.name} [{', '.join(mutation.test_nodes)}]")
      elif mutation_result.returncode == 0:
        survivors.append(mutation.name)
        print(f"SURVIVED: {mutation.name} [{', '.join(mutation.test_nodes)}]")
      else:
        print(f"INVALID: {mutation.name} exited without assertion-only test failures "
              + f"(pytest status {mutation_result.returncode})")
        print(mutation_result.stdout)
        return 1

    if survivors:
      print(f"Historical mutations survived: {', '.join(survivors)}")
      return 1
    print(f"ALL KILLED: {len(MUTATIONS)} parent longitudinal mutations")
    print("RESTORED: every mutated source is byte-identical")
    return 0


if __name__ == "__main__":
  raise SystemExit(main())
