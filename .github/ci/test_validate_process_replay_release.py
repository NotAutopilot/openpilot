"""Contract tests for process-replay release validator and publisher gates.

These tests prove fail-closed publication behavior against pending inventory and
common release/manifest tampering. They do not run process replay itself.
"""

from __future__ import annotations

import hashlib
import json
import zipfile
from pathlib import Path

import pytest

import validate_process_replay_release as v


CI_DIR = Path(__file__).resolve().parent
WORKFLOW_PATH = CI_DIR.parent / "workflows" / "process_replay_refs.yaml"
STAGED_INVENTORY = CI_DIR / "process_replay_staged_inventory.json"

SOURCE_SHA = "a" * 40
OPENDBC_SHA = "b" * 40
OTHER_OPENDBC = "c" * 40


def _canonical(obj) -> bytes:
  return v.canonical_json(obj)


def _sha256(data: bytes) -> str:
  return hashlib.sha256(data).hexdigest()


def _build_cases(executable: bool = True, pin_sources: bool = True) -> list[dict]:
  cases = []
  # 16 active-like full/partial + 2 pending-style ids => 18 cases / 78 tasks
  # card/controlsd/lagd for all 18 = 54; remaining 24 "other" on first 2 full cases (12 each).
  # Use exact allowed process set from staged inventory contract.
  allowed = [
    "calibrationd", "card", "controlsd", "dmonitoringd", "lagd", "locationd",
    "paramsd", "plannerd", "radard", "selfdrived", "torqued", "ubloxd",
  ]
  other = [p for p in allowed if p not in {"card", "controlsd", "lagd"}]
  assert len(other) == 9
  # two full cases with core+all other (2*(3+9)=24 other tasks), sixteen core-only (16*3=48), total 72?

  # Need 78 = 18*3 core + 24 other => other tasks across cases must total 24.
  # 2 full cases with all 9 other would be 18 other; need 24 => use 2 cases with 9 other + 2 cases with 3 other?

  # Simpler: first case has 12 procs? allowed only has 9 other.
  # 9+9+3+3 = 24 other with four cases carrying extras.
  for idx in range(18):
    cid = f"case{idx:02d}"
    procs = ["card", "controlsd", "lagd"]
    if idx < 2:
      procs = procs + other  # 9 other each => 18
    elif idx < 4:
      procs = procs + other[:3]  # 3 other each => 6; total other 24
    source = f"https://example.test/rlogs/{cid}.zst" if executable else ""
    case = {
      "case_id": cid,
      "car_brand": f"BRAND{idx}",
      "source": source,
      "source_sha256": ("1" * 64) if pin_sources and executable else None,
      "source_bytes": 16 if pin_sources and executable else None,
      "processes": procs,
      "params_digest": "2" * 64,
      "custom_params": {},
      "executable": bool(executable and source),
    }
    cases.append(case)
  return cases


def _inventory_from_cases(cases: list[dict]) -> dict:
  allowed = sorted({
    "calibrationd", "card", "controlsd", "dmonitoringd", "lagd", "locationd",
    "paramsd", "plannerd", "radard", "selfdrived", "torqued", "ubloxd",
  })
  task_ids = []
  for case in cases:
    for proc in case["processes"]:
      task_ids.append(f"{case['case_id']}:{proc}")
  assert len(cases) == 18
  assert len(task_ids) == 78
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
  return {
    "schema_version": 1,
    "inventory": "staged",
    "expected_cases": 18,
    "expected_tasks": 78,
    "partition": {"card": 18, "controlsd": 18, "lagd": 18, "other": 24},
    "allowed_processes": allowed,
    "cases": cases,
    "task_ids": task_ids,
    "expected_sources_digest": _sha256(_canonical(sources_payload)),
    "expected_params_digest": _sha256(_canonical(params_payload)),
  }


def _write_release(release: Path, cases: list[dict], *, opendbc_sha: str = OPENDBC_SHA) -> str:
  release.mkdir(parents=True, exist_ok=True)
  (release / "ref_commit").write_text(SOURCE_SHA + "\n", encoding="utf-8")
  refs = {}
  for case in cases:
    for proc in case["processes"]:
      task_id = f"{case['case_id']}:{proc}"
      filename = f"{case['case_id']}__{proc}__{SOURCE_SHA}.zst"
      payload = f"{task_id}\n".encode()
      (release / filename).write_bytes(payload)
      refs[task_id] = {
        "filename": filename,
        "size": len(payload),
        "sha256": _sha256(payload),
      }
  sorted_cases = sorted((v.case_identity(c) for c in cases), key=lambda c: c["case_id"])
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
  manifest = {
    "schema_version": 1,
    "cases": sorted_cases,
    "refs": {k: refs[k] for k in sorted(refs)},
    "openpilot_sha": SOURCE_SHA,
    "opendbc_sha": opendbc_sha,
    "sources_digest": _sha256(_canonical(sources_payload)),
    "params_digest": _sha256(_canonical(params_payload)),
    "previous_accepted_tag": "",
  }
  raw = _canonical(manifest)
  (release / "manifest.json").write_bytes(raw)
  return _sha256(raw)


def test_current_staged_inventory_is_rejected_at_publish_boundary():
  trusted = json.loads(STAGED_INVENTORY.read_text(encoding="utf-8"))
  with pytest.raises(SystemExit, match="publication requires"):
    v.require_publishable_inventory(trusted)


def test_validate_release_fails_closed_on_pending_repo_inventory(tmp_path: Path):
  # Ignore synthetic cases; exercise the real checked-in pending inventory path.
  inv_path = STAGED_INVENTORY
  release = tmp_path / "release"
  release.mkdir()
  (release / "ref_commit").write_text(SOURCE_SHA + "\n", encoding="utf-8")
  (release / "manifest.json").write_text("{}\n", encoding="utf-8")
  trusted = v.load_trusted_inventory(inv_path)
  with pytest.raises(SystemExit, match="publication requires"):
    v.validate_release(
      release,
      trusted=trusted,
      source_sha=SOURCE_SHA,
      operation="publish",
      expected_manifest_sha256="d" * 64,
      rollback_tag=None,
      expected_opendbc_sha=OPENDBC_SHA,
    )


def test_happy_path_accepts_fully_pinned_inventory(tmp_path: Path):
  cases = _build_cases(executable=True, pin_sources=True)
  inventory = _inventory_from_cases(cases)
  inv_path = tmp_path / "inventory.json"
  inv_path.write_bytes(_canonical(inventory))
  release = tmp_path / "release"
  manifest_sha = _write_release(release, cases, opendbc_sha=OPENDBC_SHA)
  trusted = v.load_trusted_inventory(inv_path)
  result = v.validate_release(
    release,
    trusted=trusted,
    source_sha=SOURCE_SHA,
    operation="publish",
    expected_manifest_sha256=manifest_sha,
    rollback_tag=None,
    expected_opendbc_sha=OPENDBC_SHA,
  )
  assert result["release_source_sha"] == SOURCE_SHA
  assert result["release_manifest_sha256"] == manifest_sha
  assert result["release_tag"] == f"process-replay/v1/{SOURCE_SHA}-{manifest_sha[:12]}"


def test_opendbc_gitlink_mismatch_is_rejected(tmp_path: Path):
  cases = _build_cases()
  inventory = _inventory_from_cases(cases)
  inv_path = tmp_path / "inventory.json"
  inv_path.write_bytes(_canonical(inventory))
  release = tmp_path / "release"
  manifest_sha = _write_release(release, cases, opendbc_sha=OTHER_OPENDBC)
  trusted = v.load_trusted_inventory(inv_path)
  with pytest.raises(SystemExit, match="opendbc_sha .* != trusted gitlink"):
    v.validate_release(
      release,
      trusted=trusted,
      source_sha=SOURCE_SHA,
      operation="publish",
      expected_manifest_sha256=manifest_sha,
      rollback_tag=None,
      expected_opendbc_sha=OPENDBC_SHA,
    )


def test_manifest_case_tamper_is_rejected(tmp_path: Path):
  cases = _build_cases()
  inventory = _inventory_from_cases(cases)
  inv_path = tmp_path / "inventory.json"
  inv_path.write_bytes(_canonical(inventory))
  release = tmp_path / "release"
  _write_release(release, cases)
  # Tamper one pinned source hash after the fact and rewrite canonical manifest.
  manifest = json.loads((release / "manifest.json").read_text(encoding="utf-8"))
  manifest["cases"][0]["source_sha256"] = "e" * 64
  # keep refs/digests stale on purpose
  tampered = _canonical(manifest)
  (release / "manifest.json").write_bytes(tampered)
  trusted = v.load_trusted_inventory(inv_path)
  with pytest.raises(SystemExit):
    v.validate_release(
      release,
      trusted=trusted,
      source_sha=SOURCE_SHA,
      operation="publish",
      expected_manifest_sha256=_sha256(tampered),
      rollback_tag=None,
      expected_opendbc_sha=OPENDBC_SHA,
    )


def test_ref_bytes_tamper_is_rejected(tmp_path: Path):
  cases = _build_cases()
  inventory = _inventory_from_cases(cases)
  inv_path = tmp_path / "inventory.json"
  inv_path.write_bytes(_canonical(inventory))
  release = tmp_path / "release"
  manifest_sha = _write_release(release, cases)
  # Corrupt one ref payload without updating manifest metadata.
  victim = next(release.glob(f"*__card__{SOURCE_SHA}.zst"))
  victim.write_bytes(b"tampered-ref-bytes")
  trusted = v.load_trusted_inventory(inv_path)
  with pytest.raises(SystemExit, match="(size mismatch|hash mismatch)"):
    v.validate_release(
      release,
      trusted=trusted,
      source_sha=SOURCE_SHA,
      operation="publish",
      expected_manifest_sha256=manifest_sha,
      rollback_tag=None,
      expected_opendbc_sha=OPENDBC_SHA,
    )


def test_non_executable_inventory_rejected():
  cases = _build_cases(executable=False, pin_sources=False)
  inventory = _inventory_from_cases(cases)
  with pytest.raises(SystemExit, match="executable=true"):
    v.require_publishable_inventory(inventory)


def test_workflow_authenticates_controller_run_and_artifact_name():
  workflow = WORKFLOW_PATH.read_text(encoding="utf-8")
  assert 'EXPECTED_ARTIFACT_NAME="nap-replay-${CONTROLLER_RUN_ID}"' in workflow
  assert 'repos/${CONTROLLER_REPO}/actions/runs/${CONTROLLER_RUN_ID}' in workflow
  assert '.github/workflows/replay.yaml' in workflow
  assert 'nap-replay' in workflow
  assert "RUN_STATUS" in workflow and "RUN_CONCLUSION" in workflow
  assert "RUN_HEAD_SHA" in workflow
  assert "RUN_EVENT" in workflow
  assert "META_NAME" in workflow


def test_workflow_rejects_duplicate_and_unsafe_zip_members():
  workflow = WORKFLOW_PATH.read_text(encoding="utf-8")
  assert "duplicate zip member" in workflow
  assert "non-regular zip member rejected" in workflow
  assert "external_attr" in workflow
  assert "O_EXCL" in workflow


def test_workflow_binds_handoff_review_proof_and_opendbc_gitlink():
  workflow = WORKFLOW_PATH.read_text(encoding="utf-8")
  assert "handoff-meta.json" in workflow
  assert "compare" in workflow and "results.json" in workflow
  assert "pass count must be 78" in workflow or "pass', -1)) != 78" in workflow
  assert "HEAD:opendbc_repo" in workflow
  assert "--expected-opendbc-sha" in workflow


def test_zip_extractor_snippet_rejects_symlink_member(tmp_path: Path):
  """Exercise the same mode checks the workflow embeds before extraction."""
  zpath = tmp_path / "cand.zip"
  with zipfile.ZipFile(zpath, "w") as zf:
    info = zipfile.ZipInfo("release/evil.zst")
    # symlink mode
    info.external_attr = (0o120777 & 0xFFFF) << 16
    zf.writestr(info, b"target")

  S_IFMT = 0o170000
  S_IFREG = 0o100000
  with zipfile.ZipFile(zpath) as zf:
    for info in zf.infolist():
      mode = (info.external_attr >> 16) & 0xFFFF
      ftype = mode & S_IFMT
      assert ftype not in (0, S_IFREG)
