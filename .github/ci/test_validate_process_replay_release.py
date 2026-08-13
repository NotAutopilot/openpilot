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


def _allowed_processes() -> list[str]:
  return sorted({
    "calibrationd", "card", "controlsd", "dmonitoringd", "lagd", "locationd",
    "paramsd", "plannerd", "radard", "selfdrived", "torqued", "ubloxd",
  })


def _build_cases(
  inventory: str = "staged",
  *,
  executable: bool = True,
  pin_sources: bool = True,
) -> list[dict]:
  """Synthetic cases matching exact active (16/66) or staged (18/78) contracts."""
  allowed = _allowed_processes()
  other = [p for p in allowed if p not in {"card", "controlsd", "lagd"}]
  assert len(other) == 9
  if inventory == "active":
    # 16 cases / 66 tasks: two full (9 other each => 18), fourteen core-only.
    n_cases = 16
    extra_plan = {0: other, 1: other}
  elif inventory == "staged":
    # 18 cases / 78 tasks: two full (9 other) + two partial (3 other) => 24 other.
    n_cases = 18
    extra_plan = {0: other, 1: other, 2: other[:3], 3: other[:3]}
  else:
    raise ValueError(f"unknown inventory {inventory!r}")

  cases = []
  for idx in range(n_cases):
    cid = f"case{idx:02d}"
    procs = ["card", "controlsd", "lagd"] + list(extra_plan.get(idx, []))
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


def _inventory_from_cases(cases: list[dict], inventory: str = "staged") -> dict:
  contract = v.INVENTORY_CONTRACTS[inventory]
  expected_cases = contract["expected_cases"]
  expected_tasks = contract["expected_tasks"]
  partition = dict(contract["partition"])
  allowed = _allowed_processes()
  task_ids = []
  for case in cases:
    for proc in case["processes"]:
      task_ids.append(f"{case['case_id']}:{proc}")
  assert len(cases) == expected_cases
  assert len(task_ids) == expected_tasks
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
    "inventory": inventory,
    "expected_cases": expected_cases,
    "expected_tasks": expected_tasks,
    "partition": partition,
    "allowed_processes": allowed,
    "cases": cases,
    "task_ids": task_ids,
    "expected_sources_digest": _sha256(_canonical(sources_payload)),
    "expected_params_digest": _sha256(_canonical(params_payload)),
  }


def _write_release(
  release: Path,
  cases: list[dict],
  *,
  opendbc_sha: str = OPENDBC_SHA,
  previous_accepted_tag: str = "",
) -> str:
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
        "source": case["source"],
        "source_sha256": case["source_sha256"],
        "source_bytes": case["source_bytes"],
        "params_digest": case["params_digest"],
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
    "previous_accepted_tag": previous_accepted_tag,
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


def test_valid_active_inventory_loads(tmp_path: Path):
  cases = _build_cases("active", executable=True, pin_sources=True)
  inventory = _inventory_from_cases(cases, inventory="active")
  inv_path = tmp_path / "active_inventory.json"
  inv_path.write_bytes(_canonical(inventory))
  trusted = v.load_trusted_inventory(inv_path)
  assert trusted["inventory"] == "active"
  assert trusted["expected_cases"] == 16
  assert trusted["expected_tasks"] == 66
  assert trusted["partition"] == {"card": 16, "controlsd": 16, "lagd": 16, "other": 18}


def test_active_declared_count_or_partition_mismatch_rejected(tmp_path: Path):
  cases = _build_cases("active")
  inventory = _inventory_from_cases(cases, inventory="active")

  bad_counts = dict(inventory)
  bad_counts["expected_cases"] = 18
  bad_counts["expected_tasks"] = 78
  path_counts = tmp_path / "bad_counts.json"
  path_counts.write_bytes(_canonical(bad_counts))
  with pytest.raises(SystemExit, match="must declare 16 cases / 66 tasks for inventory=active"):
    v.load_trusted_inventory(path_counts)

  bad_partition = dict(inventory)
  bad_partition["partition"] = {"card": 18, "controlsd": 18, "lagd": 18, "other": 24}
  path_part = tmp_path / "bad_partition.json"
  path_part.write_bytes(_canonical(bad_partition))
  with pytest.raises(SystemExit, match="partition mismatch for inventory=active"):
    v.load_trusted_inventory(path_part)

  cross = dict(inventory)
  cross["inventory"] = "staged"
  path_cross = tmp_path / "cross_label.json"
  path_cross.write_bytes(_canonical(cross))
  with pytest.raises(SystemExit, match="must declare 18 cases / 78 tasks for inventory=staged"):
    v.load_trusted_inventory(path_cross)


def test_archive_previous_accepted_tag_identity_accepted(tmp_path: Path):
  cases = _build_cases("staged", executable=True, pin_sources=True)
  inventory = _inventory_from_cases(cases, inventory="staged")
  inv_path = tmp_path / "inventory.json"
  inv_path.write_bytes(_canonical(inventory))
  archive_tag = f"process-replay/archive/{'f' * 40}"
  release = tmp_path / "release"
  manifest_sha = _write_release(release, cases, previous_accepted_tag=archive_tag)
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
  assert result["release_manifest_sha256"] == manifest_sha


def test_ref_identity_fields_must_match_trusted_case(tmp_path: Path):
  cases = _build_cases("staged", executable=True, pin_sources=True)
  inventory = _inventory_from_cases(cases, inventory="staged")
  inv_path = tmp_path / "inventory.json"
  inv_path.write_bytes(_canonical(inventory))
  release = tmp_path / "release"
  _write_release(release, cases)
  manifest = json.loads((release / "manifest.json").read_text(encoding="utf-8"))
  victim = next(iter(manifest["refs"]))
  manifest["refs"][victim]["source_sha256"] = "e" * 64
  tampered = _canonical(manifest)
  (release / "manifest.json").write_bytes(tampered)
  trusted = v.load_trusted_inventory(inv_path)
  with pytest.raises(SystemExit, match="ref source_sha256 diverges from trusted case"):
    v.validate_release(
      release,
      trusted=trusted,
      source_sha=SOURCE_SHA,
      operation="publish",
      expected_manifest_sha256=_sha256(tampered),
      rollback_tag=None,
      expected_opendbc_sha=OPENDBC_SHA,
    )


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


def test_workflow_generates_only_current_active_inventory():
  workflow = WORKFLOW_PATH.read_text(encoding="utf-8")
  assert "generate-active" in workflow
  assert "generate-staged" not in workflow
  assert "CONTROLLER_REPO" not in workflow
  assert "NAP_CI_CONTROLLER_ARTIFACT_READ_TOKEN" not in workflow
  assert 'EXPECTED_TASKS: "66"' in workflow
  assert 'github.ref == \'refs/heads/nap-dev\'' in workflow
  assert 'EXPECTED_WORKFLOW_REF="${SOURCE_REPO}/.github/workflows/process_replay_refs.yaml@refs/heads/nap-dev"' in workflow


def test_workflow_rejects_unsafe_downloaded_artifact_layouts():
  workflow = WORKFLOW_PATH.read_text(encoding="utf-8")
  assert "unexpected candidate top-level entry" in workflow
  assert "path traversal in candidate" in workflow
  assert "symlink forbidden in candidate" in workflow
  assert "non-regular candidate member" in workflow
  assert "nested release path forbidden" in workflow


def test_workflow_binds_same_run_review_proof_and_opendbc_gitlink():
  workflow = WORKFLOW_PATH.read_text(encoding="utf-8")
  assert "handoff-meta.json" in workflow
  assert "compare" in workflow and "results.json" in workflow
  assert "review proof pass count must be {expected_tasks}" in workflow
  assert "workflow_run_id" in workflow
  assert "GITHUB_RUN_ID" in workflow
  assert "HEAD:opendbc_repo" in workflow
  assert "expected_ids = sorted(trusted_ids)" in workflow
  assert "len(set(trusted_ids)) != expected_tasks" in workflow
  assert "path: .process-replay-publish-release" in workflow
  assert "--release-dir .process-replay-publish-release" in workflow
  assert 'cp -a "${GITHUB_WORKSPACE}/.process-replay-publish-release/." .' in workflow
  assert "--expected-opendbc-sha" in workflow


def test_workflow_preserves_same_source_no_op_and_atomic_publication():
  workflow = WORKFLOW_PATH.read_text(encoding="utf-8")
  assert 'CURRENT_REF_COMMIT}" == "${SOURCE_SHA}' in workflow
  assert "manifest.json?ref=${PREVIOUS_HEAD}" in workflow
  assert "previous_accepted_tag" in workflow
  assert "git push --atomic" in workflow
  assert '--force-with-lease="refs/heads/${ARTIFACTS_BRANCH}:${CAPTURED_HEAD}"' in workflow


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
