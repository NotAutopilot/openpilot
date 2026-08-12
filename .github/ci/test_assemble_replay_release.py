"""Focused tests for active process-replay release assembly.

Covers a valid 66-ref assembly and fail-closed rejects (missing/extra task,
invalid digest, executable=false). Uses local synthetic fakedata only — no
network downloads.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

CI_DIR = Path(__file__).resolve().parent
REPO_ROOT = CI_DIR.parents[1]
ASSEMBLE_PATH = REPO_ROOT / "tools" / "ci" / "assemble_replay_release.py"
ACTIVE_INVENTORY = CI_DIR / "process_replay_active_inventory.json"

SOURCE_SHA = "a" * 40
OPENDBC_SHA = "b" * 40
PREV_TAG = "process-replay/v1/" + ("c" * 40) + "-" + ("d" * 12)


def _load_assemble():
  spec = importlib.util.spec_from_file_location("assemble_replay_release", ASSEMBLE_PATH)
  assert spec is not None and spec.loader is not None
  mod = importlib.util.module_from_spec(spec)
  sys.modules[spec.name] = mod
  spec.loader.exec_module(mod)
  return mod


assemble = _load_assemble()


def _canonical(obj: object) -> bytes:
  return assemble.canonical_json(obj)


def _sha256(data: bytes) -> str:
  return hashlib.sha256(data).hexdigest()


def _load_inventory_cases() -> list[dict]:
  inventory = json.loads(ACTIVE_INVENTORY.read_text(encoding="utf-8"))
  assert inventory["inventory"] == "active"
  assert inventory["expected_cases"] == 16
  assert inventory["expected_tasks"] == 66
  cases = inventory["cases"]
  assert isinstance(cases, list) and len(cases) == 16
  return [dict(c) for c in cases]


def _write_cases(path: Path, cases: list[dict]) -> None:
  path.write_text(json.dumps(cases, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _seed_fakedata(fakedata: Path, cases: list[dict], *, skip_task: str | None = None, extra_name: str | None = None) -> list[str]:
  fakedata.mkdir(parents=True, exist_ok=True)
  written: list[str] = []
  for case in cases:
    for proc in case["processes"]:
      task_id = f"{case['case_id']}:{proc}"
      if skip_task is not None and task_id == skip_task:
        continue
      name = f"{case['case_id']}__{proc}__{SOURCE_SHA}.zst"
      (fakedata / name).write_bytes(f"{task_id}\n".encode())
      written.append(task_id)
  if extra_name is not None:
    (fakedata / extra_name).write_bytes(b"extra-ref\n")
  return written


def _run_assemble(cases_json: Path, fakedata: Path, release: Path) -> subprocess.CompletedProcess[str]:
  return subprocess.run(
    [
      sys.executable,
      str(ASSEMBLE_PATH),
      "--operation",
      "generate-active",
      "--source-sha",
      SOURCE_SHA,
      "--fakedata-dir",
      str(fakedata),
      "--release-dir",
      str(release),
      "--opendbc-sha",
      OPENDBC_SHA,
      "--cases-json",
      str(cases_json),
      "--previous-accepted-tag",
      PREV_TAG,
    ],
    check=False,
    capture_output=True,
    text=True,
  )


def test_active_inventory_matches_harness_case_record_contract():
  inventory = json.loads(ACTIVE_INVENTORY.read_text(encoding="utf-8"))
  assert inventory["schema_version"] == 1
  assert inventory["inventory"] == "active"
  assert inventory["expected_cases"] == 16
  assert inventory["expected_tasks"] == 66
  assert inventory["partition"] == {"card": 16, "controlsd": 16, "lagd": 16, "other": 18}
  assert inventory["allowed_processes"] == sorted(inventory["allowed_processes"])

  cases = inventory["cases"]
  task_ids = inventory["task_ids"]
  assert len(cases) == 16
  assert len(task_ids) == 66
  assert len(set(task_ids)) == 66

  derived = [f"{c['case_id']}:{p}" for c in cases for p in c["processes"]]
  assert derived == task_ids
  assert all(c["executable"] is True for c in cases)
  assert all(isinstance(c["source"], str) and c["source"] for c in cases)
  assert all(isinstance(c["source_bytes"], int) and c["source_bytes"] > 0 for c in cases)
  assert all(isinstance(c["source_sha256"], str) and len(c["source_sha256"]) == 64 for c in cases)
  assert all(isinstance(c["params_digest"], str) and len(c["params_digest"]) == 64 for c in cases)

  sorted_cases = sorted(cases, key=lambda c: c["case_id"])
  sources_payload = [
    {
      "case_id": c["case_id"],
      "source": c["source"],
      "source_bytes": c["source_bytes"],
      "source_sha256": c["source_sha256"],
    }
    for c in sorted_cases
  ]
  params_payload = [{"case_id": c["case_id"], "params_digest": c["params_digest"]} for c in sorted_cases]
  assert inventory["expected_sources_digest"] == _sha256(_canonical(sources_payload))
  assert inventory["expected_params_digest"] == _sha256(_canonical(params_payload))

  counts = {"card": 0, "controlsd": 0, "lagd": 0, "other": 0}
  for task_id in task_ids:
    proc = task_id.split(":", 1)[1]
    counts[proc if proc in counts else "other"] += 1
  assert counts == inventory["partition"]


def test_assemble_valid_66_ref_release(tmp_path: Path):
  cases = _load_inventory_cases()
  cases_json = tmp_path / "cases.json"
  fakedata = tmp_path / "fakedata"
  release = tmp_path / "release"
  _write_cases(cases_json, cases)
  written = _seed_fakedata(fakedata, cases)
  assert len(written) == 66

  proc = _run_assemble(cases_json, fakedata, release)
  assert proc.returncode == 0, proc.stderr + proc.stdout

  assert (release / "ref_commit").read_text(encoding="utf-8") == SOURCE_SHA + "\n"
  manifest_bytes = (release / "manifest.json").read_bytes()
  assert manifest_bytes == _canonical(json.loads(manifest_bytes.decode("utf-8")))

  manifest = json.loads(manifest_bytes.decode("utf-8"))
  assert manifest["schema_version"] == 1
  assert manifest["openpilot_sha"] == SOURCE_SHA
  assert manifest["opendbc_sha"] == OPENDBC_SHA
  assert manifest["previous_accepted_tag"] == PREV_TAG
  assert len(manifest["cases"]) == 16
  assert len(manifest["refs"]) == 66

  inventory = json.loads(ACTIVE_INVENTORY.read_text(encoding="utf-8"))
  assert manifest["sources_digest"] == inventory["expected_sources_digest"]
  assert manifest["params_digest"] == inventory["expected_params_digest"]

  by_id = {c["case_id"]: c for c in cases}
  for cid, rec in by_id.items():
    assembled = next(c for c in manifest["cases"] if c["case_id"] == cid)
    assert assembled["source"] == rec["source"]
    assert assembled["source_sha256"] == rec["source_sha256"]
    assert assembled["source_bytes"] == rec["source_bytes"]
    assert assembled["params_digest"] == rec["params_digest"]
    assert assembled["executable"] is True

  for task_id, entry in manifest["refs"].items():
    case_id, process = task_id.split(":", 1)
    rec = by_id[case_id]
    assert process in rec["processes"]
    assert entry["filename"] == f"{case_id}__{process}__{SOURCE_SHA}.zst"
    assert entry["source"] == rec["source"]
    assert entry["source_sha256"] == rec["source_sha256"]
    assert entry["source_bytes"] == rec["source_bytes"]
    assert entry["params_digest"] == rec["params_digest"]
    payload = (release / entry["filename"]).read_bytes()
    assert entry["size"] == len(payload)
    assert entry["sha256"] == _sha256(payload)

  zsts = sorted(p.name for p in release.iterdir() if p.name.endswith(".zst"))
  assert len(zsts) == 66
  assert set(release.iterdir()) == {release / "ref_commit", release / "manifest.json"} | {release / n for n in zsts}


def test_assemble_rejects_missing_task(tmp_path: Path):
  cases = _load_inventory_cases()
  cases_json = tmp_path / "cases.json"
  fakedata = tmp_path / "fakedata"
  release = tmp_path / "release"
  _write_cases(cases_json, cases)
  victim = f"{cases[0]['case_id']}:{cases[0]['processes'][0]}"
  _seed_fakedata(fakedata, cases, skip_task=victim)

  proc = _run_assemble(cases_json, fakedata, release)
  assert proc.returncode != 0
  assert "expected 66 tasks" in (proc.stderr + proc.stdout)


def test_assemble_rejects_extra_task(tmp_path: Path):
  cases = _load_inventory_cases()
  cases_json = tmp_path / "cases.json"
  fakedata = tmp_path / "fakedata"
  release = tmp_path / "release"
  _write_cases(cases_json, cases)
  # hyundai2 is core-only; selfdrived is an extra process for that case.
  extra = f"hyundai2__selfdrived__{SOURCE_SHA}.zst"
  _seed_fakedata(fakedata, cases, extra_name=extra)

  proc = _run_assemble(cases_json, fakedata, release)
  assert proc.returncode != 0
  combined = proc.stderr + proc.stdout
  assert "process selfdrived not in case hyundai2 process set" in combined or "expected 66 tasks" in combined


def test_assemble_rejects_invalid_digest(tmp_path: Path):
  cases = _load_inventory_cases()
  cases[0]["params_digest"] = "not-a-digest"
  cases_json = tmp_path / "cases.json"
  fakedata = tmp_path / "fakedata"
  release = tmp_path / "release"
  _write_cases(cases_json, cases)
  _seed_fakedata(fakedata, cases)

  proc = _run_assemble(cases_json, fakedata, release)
  assert proc.returncode != 0
  assert "params_digest" in (proc.stderr + proc.stdout)


def test_assemble_rejects_executable_false(tmp_path: Path):
  cases = _load_inventory_cases()
  cases[0]["executable"] = False
  cases_json = tmp_path / "cases.json"
  fakedata = tmp_path / "fakedata"
  release = tmp_path / "release"
  _write_cases(cases_json, cases)
  _seed_fakedata(fakedata, cases)

  proc = _run_assemble(cases_json, fakedata, release)
  assert proc.returncode != 0
  assert "must be executable" in (proc.stderr + proc.stdout)


def test_assemble_rejects_generate_staged_operation():
  proc = subprocess.run(
    [
      sys.executable,
      str(ASSEMBLE_PATH),
      "--operation",
      "generate-staged",
      "--source-sha",
      SOURCE_SHA,
      "--fakedata-dir",
      ".",
      "--release-dir",
      ".",
      "--opendbc-sha",
      OPENDBC_SHA,
      "--cases-json",
      str(ACTIVE_INVENTORY),
      "--previous-accepted-tag",
      PREV_TAG,
    ],
    check=False,
    capture_output=True,
    text=True,
  )
  assert proc.returncode != 0
  assert "generate-staged" in (proc.stderr + proc.stdout) or "invalid choice" in (proc.stderr + proc.stdout).lower()
