#!/usr/bin/env python3
"""Fail-closed validation of a process-replay release tree.

Trusted inventory is loaded from the protected nap-dev checkout (GITHUB_SHA)
under the exact active or staged contract declared by the inventory itself.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import sys
from pathlib import Path
from typing import Any, Never

REQUIRED_MANIFEST_KEYS = (
  "schema_version",
  "cases",
  "refs",
  "openpilot_sha",
  "opendbc_sha",
  "sources_digest",
  "params_digest",
  "previous_accepted_tag",
)
REQUIRED_CASE_KEYS = (
  "case_id",
  "car_brand",
  "source",
  "source_sha256",
  "source_bytes",
  "processes",
  "params_digest",
  "custom_params",
  "executable",
)
REQUIRED_REF_KEYS = ("filename", "size", "sha256", "source", "source_sha256", "source_bytes", "params_digest")
CASE_ID_RE = re.compile(r"^[a-z0-9][a-z0-9-]*$")
PROC_RE = re.compile(r"^[a-z0-9_]+$")
SHA40_RE = re.compile(r"^[0-9a-f]{40}$")
SHA64_RE = re.compile(r"^[0-9a-f]{64}$")
TAG_RE = re.compile(r"^process-replay/v1/([0-9a-f]{40})-([0-9a-f]{12})$")
ARCHIVE_TAG_RE = re.compile(r"^process-replay/archive/([0-9a-f]{40})$")
CORE_PROCS = frozenset({"card", "controlsd", "lagd"})

# Exact trusted inventory contracts. Counts, partitions, and labels are fail-closed.
INVENTORY_CONTRACTS: dict[str, dict[str, Any]] = {
  "active": {
    "expected_cases": 16,
    "expected_tasks": 66,
    "partition": {"card": 16, "controlsd": 16, "lagd": 16, "other": 18},
  },
  "staged": {
    "expected_cases": 18,
    "expected_tasks": 78,
    "partition": {"card": 18, "controlsd": 18, "lagd": 18, "other": 24},
  },
}


def die(msg: str) -> Never:
  raise SystemExit(f"validate_process_replay_release: {msg}")


def canonical_json(obj: Any) -> bytes:
  return (json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")


def load_trusted_inventory(path: Path) -> dict[str, Any]:
  if not path.is_file() or path.is_symlink():
    die(f"trusted inventory missing or symlink: {path}")
  raw = path.read_bytes()
  try:
    inventory = json.loads(raw.decode("utf-8"))
  except json.JSONDecodeError as exc:
    die(f"trusted inventory is not valid JSON: {exc}")
  if not isinstance(inventory, dict):
    die("trusted inventory must be an object")
  if inventory.get("schema_version") != 1:
    die("trusted inventory schema_version must be 1")
  label = inventory.get("inventory")
  contract = INVENTORY_CONTRACTS.get(label) if isinstance(label, str) else None
  if contract is None:
    die(f"trusted inventory must declare inventory as one of {sorted(INVENTORY_CONTRACTS)}")
  expected_cases = contract["expected_cases"]
  expected_tasks = contract["expected_tasks"]
  expected_partition = contract["partition"]
  if inventory.get("expected_cases") != expected_cases or inventory.get("expected_tasks") != expected_tasks:
    die(f"trusted inventory must declare {expected_cases} cases / {expected_tasks} tasks for inventory={label}")
  partition = inventory.get("partition")
  if partition != expected_partition:
    die(f"trusted inventory partition mismatch for inventory={label}: {partition}")
  cases = inventory.get("cases")
  task_ids = inventory.get("task_ids")
  allowed = inventory.get("allowed_processes")
  if not isinstance(cases, list) or len(cases) != expected_cases:
    die(f"trusted inventory cases must be a list of {expected_cases}")
  if not isinstance(task_ids, list) or len(task_ids) != expected_tasks:
    die(f"trusted inventory task_ids must be a list of {expected_tasks}")
  if not isinstance(allowed, list) or not all(isinstance(p, str) for p in allowed):
    die("trusted inventory allowed_processes must be a string list")
  if list(allowed) != sorted(allowed):
    die("trusted inventory allowed_processes must be sorted")
  derived: list[str] = []
  seen_cases: set[str] = set()
  for case in cases:
    if not isinstance(case, dict):
      die("trusted case must be an object")
    if set(case) != set(REQUIRED_CASE_KEYS):
      die(f"trusted case keys must be exactly {REQUIRED_CASE_KEYS}")
    cid = case["case_id"]
    if not isinstance(cid, str) or not CASE_ID_RE.fullmatch(cid):
      die(f"trusted case_id invalid: {cid!r}")
    if cid in seen_cases:
      die(f"duplicate trusted case_id: {cid}")
    seen_cases.add(cid)
    procs = case["processes"]
    if not isinstance(procs, list) or not procs or len(set(procs)) != len(procs):
      die(f"trusted processes invalid for {cid}")
    for proc in procs:
      if not isinstance(proc, str) or not PROC_RE.fullmatch(proc) or proc not in allowed:
        die(f"trusted process rejected for {cid}: {proc!r}")
      derived.append(f"{cid}:{proc}")
  if derived != task_ids:
    die("trusted task_ids do not match derived case/process inventory")
  if len(set(task_ids)) != expected_tasks:
    die("trusted task_ids must be unique")
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
  sources_digest = hashlib.sha256(canonical_json(sources_payload)).hexdigest()
  params_digest = hashlib.sha256(canonical_json(params_payload)).hexdigest()
  if inventory.get("expected_sources_digest") != sources_digest:
    die("trusted expected_sources_digest mismatch against recomputed full sorted case records")
  if inventory.get("expected_params_digest") != params_digest:
    die("trusted expected_params_digest mismatch against recomputed full sorted case records")
  counts = {"card": 0, "controlsd": 0, "lagd": 0, "other": 0}
  for task_id in task_ids:
    process = task_id.split(":", 1)[1]
    if process in CORE_PROCS:
      counts[process] += 1
    else:
      counts["other"] += 1
  if counts != partition:
    die(f"trusted partition derived mismatch: {counts}")
  return inventory


def list_release_files(release: Path) -> list[str]:
  if release.is_symlink() or not release.is_dir():
    die("release/ must be a real directory")
  rel_files: list[str] = []
  for path in sorted(release.rglob("*")):
    rel = path.relative_to(release).as_posix()
    if ".." in Path(rel).parts or rel.startswith("/"):
      die(f"illegal path {rel}")
    if path.is_symlink():
      die(f"symlink forbidden: {rel}")
    if path.is_dir():
      continue
    if not path.is_file():
      die(f"non-regular file forbidden: {rel}")
    if "/" in rel:
      die(f"nested release paths forbidden: {rel}")
    rel_files.append(rel)
  return rel_files


def case_identity(case: dict[str, Any]) -> dict[str, Any]:
  return {key: case[key] for key in REQUIRED_CASE_KEYS}


def require_publishable_case(case: dict[str, Any], *, where: str) -> None:
  """Publication boundary: every case must be executable with pinned source bytes."""
  cid = case.get("case_id", "<unknown>")
  if case.get("executable") is not True:
    die(f"{where} case {cid}: publication requires executable=true (descriptors not finalized)")
  source = case.get("source")
  if not isinstance(source, str) or not source.strip():
    die(f"{where} case {cid}: publication requires non-empty source locator/URL")
  source_bytes = case.get("source_bytes")
  if not isinstance(source_bytes, int) or isinstance(source_bytes, bool) or source_bytes <= 0:
    die(f"{where} case {cid}: publication requires positive source_bytes")
  source_sha256 = case.get("source_sha256")
  if not isinstance(source_sha256, str) or not SHA64_RE.fullmatch(source_sha256):
    die(f"{where} case {cid}: publication requires 64 lowercase hex source_sha256")


def require_publishable_inventory(trusted: dict[str, Any]) -> None:
  """Fail closed on pending/stale inventory until fixture descriptors are finalized."""
  cases = trusted.get("cases")
  if not isinstance(cases, list):
    die("trusted inventory cases missing")
  for case in cases:
    if not isinstance(case, dict):
      die("trusted case must be an object")
    require_publishable_case(case, where="trusted inventory")


def validate_release(
  release: Path,
  *,
  trusted: dict[str, Any],
  source_sha: str,
  operation: str,
  expected_manifest_sha256: str | None,
  rollback_tag: str | None,
  expected_opendbc_sha: str | None,
) -> dict[str, str]:
  if not SHA40_RE.fullmatch(source_sha) or source_sha != source_sha.lower():
    die("source_sha must be lowercase 40-hex")
  if operation not in {"publish", "rollback"}:
    die(f"unknown operation {operation}")

  # Publication requires every trusted case to be executable with pinned source digests.
  require_publishable_inventory(trusted)

  expected_cases = int(trusted["expected_cases"])
  expected_tasks = int(trusted["expected_tasks"])
  partition = dict(trusted["partition"])
  allowed_processes = set(trusted["allowed_processes"])
  trusted_cases = {c["case_id"]: case_identity(c) for c in trusted["cases"]}
  trusted_task_ids = list(trusted["task_ids"])
  trusted_task_set = set(trusted_task_ids)
  trusted_sources_digest = trusted["expected_sources_digest"]
  trusted_params_digest = trusted["expected_params_digest"]

  rel_files = list_release_files(release)
  if "ref_commit" not in rel_files or "manifest.json" not in rel_files:
    die("release tree missing ref_commit or manifest.json")
  zsts = [name for name in rel_files if name.endswith(".zst")]
  extras = [name for name in rel_files if name not in {"ref_commit", "manifest.json"} and not name.endswith(".zst")]
  if extras:
    die(f"unexpected release files: {extras}")
  if len(zsts) != expected_tasks:
    die(f"expected {expected_tasks} zst refs, found {len(zsts)}")

  ref_commit = (release / "ref_commit").read_text(encoding="utf-8").strip()
  if ref_commit != source_sha:
    die(f"ref_commit {ref_commit} != source_sha {source_sha}")

  manifest_bytes = (release / "manifest.json").read_bytes()
  if not manifest_bytes.endswith(b"\n"):
    die("manifest.json must end with LF")
  manifest_sha = hashlib.sha256(manifest_bytes).hexdigest()
  if operation == "publish":
    if not expected_manifest_sha256 or not SHA64_RE.fullmatch(expected_manifest_sha256):
      die("publish requires expected manifest sha256")
    if manifest_sha != expected_manifest_sha256:
      die("manifest sha256 mismatch against dispatch input")

  try:
    manifest = json.loads(manifest_bytes.decode("utf-8"))
  except json.JSONDecodeError as exc:
    die(f"manifest.json is not valid JSON: {exc}")
  if not isinstance(manifest, dict):
    die("manifest must be an object")
  if canonical_json(manifest) != manifest_bytes:
    die("manifest.json is not canonical sorted UTF-8 JSON with LF")
  if set(manifest) != set(REQUIRED_MANIFEST_KEYS):
    die(f"manifest keys must be exactly {REQUIRED_MANIFEST_KEYS}; got {sorted(manifest)}")
  if manifest["schema_version"] != 1:
    die("manifest schema_version must be integer 1")
  if manifest["openpilot_sha"] != source_sha:
    die("manifest openpilot_sha must equal source_sha")
  if not isinstance(manifest["opendbc_sha"], str) or not SHA40_RE.fullmatch(manifest["opendbc_sha"]):
    die("opendbc_sha must be 40 lowercase hex")
  if not expected_opendbc_sha or not SHA40_RE.fullmatch(expected_opendbc_sha):
    die("expected_opendbc_sha must be lowercase 40-hex from trusted gitlink")
  if manifest["opendbc_sha"] != expected_opendbc_sha:
    die(f"manifest opendbc_sha {manifest['opendbc_sha']} != trusted gitlink {expected_opendbc_sha}")
  previous = manifest["previous_accepted_tag"]
  if previous != "" and not (
    isinstance(previous, str)
    and (TAG_RE.fullmatch(previous) or ARCHIVE_TAG_RE.fullmatch(previous))
  ):
    die("previous_accepted_tag must be empty, a process-replay/v1 release tag, or a process-replay/archive SHA tag")

  cases = manifest["cases"]
  refs = manifest["refs"]
  if not isinstance(cases, list) or not isinstance(refs, dict):
    die("manifest must contain cases list and refs object")
  if len(cases) != expected_cases:
    die(f"expected {expected_cases} cases, found {len(cases)}")
  if len(refs) != expected_tasks:
    die(f"expected {expected_tasks} refs, found {len(refs)}")

  case_ids: list[str] = []
  for case in cases:
    if not isinstance(case, dict):
      die("manifest case must be an object")
    if set(case) != set(REQUIRED_CASE_KEYS):
      die(f"manifest case keys must be exactly {REQUIRED_CASE_KEYS}")
    cid = case["case_id"]
    if not isinstance(cid, str) or not CASE_ID_RE.fullmatch(cid):
      die(f"invalid case_id in manifest: {cid!r}")
    require_publishable_case(case, where="manifest")
    case_ids.append(cid)
    trusted_case = trusted_cases.get(cid)
    if trusted_case is None:
      die(f"manifest case_id not in trusted inventory: {cid}")
    if case_identity(case) != trusted_case:
      die(f"manifest case identity diverges from trusted inventory: {cid}")
  if len(set(case_ids)) != expected_cases:
    die("case_id values must be unique")
  if case_ids != sorted(case_ids):
    die("cases must be sorted by case_id")
  if set(case_ids) != set(trusted_cases):
    die("manifest case set must equal trusted inventory")

  sorted_cases = sorted((case_identity(c) for c in cases), key=lambda c: c["case_id"])
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
  sources_digest = hashlib.sha256(canonical_json(sources_payload)).hexdigest()
  params_digest = hashlib.sha256(canonical_json(params_payload)).hexdigest()
  if manifest["sources_digest"] != sources_digest or sources_digest != trusted_sources_digest:
    die("sources_digest mismatch against recomputed full sorted case records / trusted inventory")
  if manifest["params_digest"] != params_digest or params_digest != trusted_params_digest:
    die("params_digest mismatch against recomputed full sorted case records / trusted inventory")

  if list(refs.keys()) != sorted(refs.keys()):
    die("refs must be sorted by task key")
  if set(refs) != trusted_task_set:
    die("manifest refs task set must equal trusted task inventory")

  counts = {"card": 0, "controlsd": 0, "lagd": 0, "other": 0}
  name_re = re.compile(rf"^[a-z0-9][a-z0-9-]*__[a-z0-9_]+__{re.escape(source_sha)}\.zst$")
  for task_id, meta in refs.items():
    if not isinstance(task_id, str) or not re.fullmatch(r"[a-z0-9][a-z0-9-]*:[a-z0-9_]+", task_id):
      die(f"bad task id {task_id}")
    if not isinstance(meta, dict) or set(meta) != set(REQUIRED_REF_KEYS):
      die(f"ref metadata keys must be exactly {REQUIRED_REF_KEYS} for {task_id}")
    case_id, process = task_id.split(":", 1)
    if case_id not in trusted_cases:
      die(f"ref case_id not in trusted cases: {case_id}")
    if process not in allowed_processes:
      die(f"unknown process rejected: {process}")
    if process not in trusted_cases[case_id]["processes"]:
      die(f"process {process} not in trusted case {case_id}")
    trusted_case = trusted_cases[case_id]
    if meta.get("source") != trusted_case["source"]:
      die(f"ref source diverges from trusted case for {task_id}")
    if meta.get("source_sha256") != trusted_case["source_sha256"]:
      die(f"ref source_sha256 diverges from trusted case for {task_id}")
    if meta.get("source_bytes") != trusted_case["source_bytes"]:
      die(f"ref source_bytes diverges from trusted case for {task_id}")
    if meta.get("params_digest") != trusted_case["params_digest"]:
      die(f"ref params_digest diverges from trusted case for {task_id}")
    if process in CORE_PROCS:
      counts[process] += 1
    else:
      counts["other"] += 1
    filename = meta["filename"]
    size = meta["size"]
    digest = meta["sha256"]
    if not isinstance(filename, str) or filename not in zsts:
      die(f"ref filename missing from tree: {filename}")
    if not name_re.fullmatch(filename):
      die(f"ref filename must encode source_sha: {filename}")
    if filename != f"{case_id}__{process}__{source_sha}.zst":
      die(f"filename does not match task identity: {filename}")
    if not isinstance(size, int) or isinstance(size, bool) or size < 0:
      die(f"invalid size for {filename}")
    if not isinstance(digest, str) or not SHA64_RE.fullmatch(digest):
      die(f"invalid sha256 for {filename}")
    data = (release / filename).read_bytes()
    if size != len(data):
      die(f"size mismatch for {filename}")
    actual = hashlib.sha256(data).hexdigest()
    if digest != actual:
      die(f"hash mismatch for {filename}")

  if counts != partition:
    die(f"partition mismatch: {counts} != {partition}")
  if set(zsts) != {refs[task_id]["filename"] for task_id in refs}:
    die("zst set does not equal manifest filenames")

  release_tag = f"process-replay/v1/{source_sha}-{manifest_sha[:12]}"
  if operation == "rollback":
    if not rollback_tag:
      die("rollback requires rollback_tag")
    if not TAG_RE.fullmatch(rollback_tag):
      die("rollback_tag must match process-replay/v1/<40-hex>-<12-hex>")
    if rollback_tag != release_tag:
      die(f"rollback_tag {rollback_tag} is not cryptographically bound to fetched manifest {release_tag}")

  return {
    "release_source_sha": source_sha,
    "release_manifest_sha256": manifest_sha,
    "release_tag": release_tag,
    "release_previous_accepted_tag": previous,
  }


def write_github_output(result: dict[str, str]) -> None:
  out_path = os.environ.get("GITHUB_OUTPUT")
  if not out_path:
    return
  with open(out_path, "a", encoding="utf-8") as handle:
    for key, value in result.items():
      handle.write(f"{key}={value}\n")


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--release-dir", required=True, type=Path)
  parser.add_argument("--trusted-inventory", required=True, type=Path)
  parser.add_argument("--source-sha", required=True)
  parser.add_argument("--operation", required=True, choices=("publish", "rollback"))
  parser.add_argument("--expected-manifest-sha256", default="")
  parser.add_argument("--rollback-tag", default="")
  parser.add_argument(
    "--expected-opendbc-sha",
    required=True,
    help="Trusted opendbc_repo gitlink SHA for source_sha (lowercase 40-hex)",
  )
  args = parser.parse_args()

  trusted = load_trusted_inventory(args.trusted_inventory)
  result = validate_release(
    args.release_dir,
    trusted=trusted,
    source_sha=args.source_sha.strip().lower(),
    operation=args.operation,
    expected_manifest_sha256=(args.expected_manifest_sha256 or None),
    rollback_tag=(args.rollback_tag or None),
    expected_opendbc_sha=args.expected_opendbc_sha.strip().lower(),
  )
  write_github_output(result)
  print(f"validated release tree source_sha={result['release_source_sha']} tasks={trusted['expected_tasks']} tag={result['release_tag']}")
  return 0


if __name__ == "__main__":
  sys.exit(main())
