import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SOURCE_PATH = REPO_ROOT / "opendbc_repo" / "opendbc" / "car" / "tesla" / "preap" / "constants.py"
ORIGINAL_KI = b"PEDAL_LONG_KI_V = [0.0, 0.0, 0.0, 0.0]\n"
HISTORICAL_KI = b"PEDAL_LONG_KI_V = [0.05, 0.08, 0.10, 0.15]\n"
TEST_PATH = "selfdrive/controls/tests/test_tesla_preap_longcontrol.py"
MUTATION_TEST_NODES = (
  f"{TEST_PATH}::test_vdas_receives_route_shaped_planner_target_trace_unchanged",
  f"{TEST_PATH}::test_road_load_history_cannot_reverse_finite_jerk_negative_planner_target",
  f"{TEST_PATH}::test_negative_planner_target_reaches_regen_side_of_coast_anchor",
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


def main() -> int:
  with tempfile.TemporaryDirectory(prefix="tesla-preap-parent-mutation-") as temp_dir:
    temp_root = Path(temp_dir)
    baseline_xml = temp_root / "baseline.xml"
    baseline = run_pytest((TEST_PATH,), baseline_xml)
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

    original_source = SOURCE_PATH.read_bytes()
    mutation_result = None
    mutation_error = None
    restored = False
    try:
      match_count = original_source.count(ORIGINAL_KI)
      if match_count != 1:
        raise RuntimeError(f"expected one outer-KI source match, found {match_count}")
      SOURCE_PATH.write_bytes(original_source.replace(ORIGINAL_KI, HISTORICAL_KI, 1))
      mutation_result = run_pytest(MUTATION_TEST_NODES, temp_root / "historical-outer-ki.xml")
    except Exception as exc:  # pragma: no cover - failure reporting path
      mutation_error = exc
    finally:
      SOURCE_PATH.write_bytes(original_source)
      restored = SOURCE_PATH.read_bytes() == original_source

    if not restored:
      print("INVALID: source restoration did not reproduce the original bytes")
      return 1
    if mutation_error is not None:
      print(f"INVALID: historical outer-KI mutation could not run: {mutation_error}")
      return 1
    if mutation_result is None:
      print("INVALID: historical outer-KI mutation produced no pytest result")
      return 1

    mutation_xml = temp_root / "historical-outer-ki.xml"
    try:
      mutation_testcases = junit_testcases(mutation_xml)
    except JUnitReportError as exc:
      print(f"INVALID: historical-outer-ki {exc}")
      return 1
    if mutation_result.returncode == 1 and has_only_assertion_failures(mutation_testcases):
      print(f"KILLED: historical-outer-ki [{', '.join(MUTATION_TEST_NODES)}]")
      print("RESTORED: outer-KI source is byte-identical")
      return 0
    if mutation_result.returncode == 0:
      print("SURVIVED: historical-outer-ki")
      return 1

    print("INVALID: historical-outer-ki exited without assertion-only test failures "
          + f"(pytest status {mutation_result.returncode})")
    print(mutation_result.stdout)
    return 1


if __name__ == "__main__":
  raise SystemExit(main())
