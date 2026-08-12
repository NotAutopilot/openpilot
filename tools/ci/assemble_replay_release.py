#!/usr/bin/env python3
"""Assemble a process-replay release/ tree from harness fakedata candidates.

Layout (public consumer contract):
  release/ref_commit
  release/manifest.json  (canonical sorted JSON + LF)
  release/{case_id}__{process}__{source_sha}.zst

This tool only assembles the active inventory (16 cases / 66 refs). Staged /
pending descriptors are rejected so non-executable sources cannot be published
through this path.

Manifest is NOT final unless every case/ref identity carries harness
params_digest + source_sha256 (and aggregate sources_digest/params_digest),
plus openpilot/opendbc SHAs, previous_accepted_tag, filenames/sizes/hashes.

results.json / timing / run metadata must stay outside release/.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path

NAME_RE_TMPL = r"^[a-z0-9][a-z0-9-]*__[a-z0-9_]+__{sha}\.zst$"
CASE_RE = re.compile(r"^[a-z0-9][a-z0-9-]*$")
PROC_RE = re.compile(r"^[a-z0-9_]+$")
SHA_RE = re.compile(r"^[0-9a-f]{40}$")
DIGEST_RE = re.compile(r"^[0-9a-f]{64}$")
TAG_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._/-]{0,127}$")

# Active inventory only: 16 cases / 66 tasks.
EXPECTED_CASES = 16
EXPECTED_TASKS = 66
EXPECTED_PARTITION = {"card": 16, "controlsd": 16, "lagd": 16, "other": 18}


def die(msg: str) -> None:
  raise SystemExit(f"assemble_replay_release: {msg}")


def canonical_json(obj: object) -> bytes:
  return (json.dumps(obj, sort_keys=True, separators=(",", ":"), ensure_ascii=False) + "\n").encode("utf-8")


def sha256_bytes(data: bytes) -> str:
  return hashlib.sha256(data).hexdigest()


def require_digest(label: str, value: object) -> str:
  text = str(value or "")
  if not DIGEST_RE.fullmatch(text):
    die(f"{label} must be 64 lowercase hex (harness digest); omitted/invalid values reject the manifest")
  return text


def validate_case_record(rec: dict) -> dict:
  cid = str(rec.get("case_id", ""))
  if not CASE_RE.fullmatch(cid):
    die(f"invalid case_id {cid!r}")
  for key in (
    "car_brand",
    "source",
    "processes",
    "custom_params",
    "params_digest",
    "executable",
    "source_sha256",
    "source_bytes",
  ):
    if key not in rec:
      die(f"case {cid} missing {key}")
  if rec["executable"] is not True:
    die(f"case {cid} must be executable for release assembly")
  if not isinstance(rec["source"], str) or not rec["source"]:
    die(f"case {cid} source must be non-empty string")
  if not isinstance(rec["processes"], list) or not rec["processes"]:
    die(f"case {cid} processes must be non-empty list")
  if not isinstance(rec["custom_params"], dict):
    die(f"case {cid} custom_params must be object")
  params = require_digest(f"case {cid} params_digest", rec["params_digest"])
  source_sha256 = require_digest(f"case {cid} source_sha256", rec["source_sha256"])
  try:
    source_bytes = int(rec["source_bytes"])
  except (TypeError, ValueError):
    die(f"case {cid} source_bytes must be int")
  if source_bytes <= 0:
    die(f"case {cid} source_bytes must be > 0")
  # Preserve harness record fields; normalize digests for identity.
  out = dict(rec)
  out["case_id"] = cid
  out["params_digest"] = params
  out["source_sha256"] = source_sha256
  out["source_bytes"] = source_bytes
  return out


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument(
    "--operation",
    required=True,
    choices=("generate-active",),
    help="Only generate-active is supported (staged publication is rejected)",
  )
  parser.add_argument("--source-sha", required=True)
  parser.add_argument("--fakedata-dir", required=True, type=Path)
  parser.add_argument("--release-dir", required=True, type=Path)
  parser.add_argument("--opendbc-sha", required=True)
  parser.add_argument("--cases-json", required=True, type=Path, help="Harness --list-cases JSON for exact records")
  parser.add_argument(
    "--previous-accepted-tag",
    required=True,
    help="Immutable previous-accepted tag/ref identity (required provenance; no empty default)",
  )
  args = parser.parse_args()

  source_sha = args.source_sha.strip()
  if source_sha != source_sha.lower() or not SHA_RE.fullmatch(source_sha):
    die("source_sha must be 40 lowercase hex")
  opendbc_sha = args.opendbc_sha.strip()
  if opendbc_sha != opendbc_sha.lower() or not SHA_RE.fullmatch(opendbc_sha):
    die("opendbc_sha must be 40 lowercase hex")

  previous_accepted_tag = str(args.previous_accepted_tag).strip()
  if not TAG_RE.fullmatch(previous_accepted_tag):
    die("previous_accepted_tag missing or invalid")

  if not args.cases_json.is_file():
    die(f"cases json missing: {args.cases_json}")
  case_records = json.loads(args.cases_json.read_text(encoding="utf-8"))
  if not isinstance(case_records, list):
    die("cases json must be a list")
  if len(case_records) != EXPECTED_CASES:
    die(f"cases json count {len(case_records)} != {EXPECTED_CASES}")

  by_id: dict[str, dict] = {}
  for raw in case_records:
    if not isinstance(raw, dict):
      die("case record must be object")
    rec = validate_case_record(raw)
    cid = rec["case_id"]
    if cid in by_id:
      die(f"duplicate case_id {cid}")
    by_id[cid] = rec

  fakedata = args.fakedata_dir
  if not fakedata.is_dir():
    die(f"fakedata dir missing: {fakedata}")

  release = args.release_dir
  if release.exists():
    for p in sorted(release.rglob("*"), reverse=True):
      if p.is_symlink():
        die(f"symlink forbidden in release: {p}")
      if p.is_file():
        p.unlink()
      elif p.is_dir():
        p.rmdir()
  release.mkdir(parents=True, exist_ok=True)

  name_re = re.compile(NAME_RE_TMPL.format(sha=re.escape(source_sha)))
  refs: dict[str, dict[str, object]] = {}
  case_ids: set[str] = set()
  counts = {"card": 0, "controlsd": 0, "lagd": 0, "other": 0}

  for path in sorted(fakedata.glob(f"*__*__{source_sha}.zst")):
    if path.is_symlink() or not path.is_file():
      die(f"invalid candidate: {path}")
    name = path.name
    if not name_re.fullmatch(name):
      die(f"candidate name rejected: {name}")
    case_id, process, _rest = name.split("__", 2)
    if not CASE_RE.fullmatch(case_id) or not PROC_RE.fullmatch(process):
      die(f"bad case/process in {name}")
    if case_id not in by_id:
      die(f"candidate case {case_id} not in harness case list")
    rec = by_id[case_id]
    if process not in set(rec["processes"]):
      die(f"process {process} not in case {case_id} process set")
    task_id = f"{case_id}:{process}"
    if task_id in refs:
      die(f"duplicate task {task_id}")
    data = path.read_bytes()
    digest = sha256_bytes(data)
    dest = release / name
    dest.write_bytes(data)
    # Per-ref identity: filename/size/hash + harness source/params digests.
    refs[task_id] = {
      "filename": name,
      "size": len(data),
      "sha256": digest,
      "source": rec["source"],
      "source_sha256": rec["source_sha256"],
      "source_bytes": rec["source_bytes"],
      "params_digest": rec["params_digest"],
    }
    case_ids.add(case_id)
    counts[process if process in counts else "other"] += 1

  if len(refs) != EXPECTED_TASKS:
    die(f"expected {EXPECTED_TASKS} tasks, found {len(refs)}")
  if len(case_ids) != EXPECTED_CASES:
    die(f"expected {EXPECTED_CASES} cases, found {len(case_ids)}")
  if case_ids != set(by_id):
    die(f"assembled cases {sorted(case_ids)} != harness cases {sorted(by_id)}")
  if counts != EXPECTED_PARTITION:
    die(f"partition mismatch: {counts} != {EXPECTED_PARTITION}")

  # Exact task identity set: every process in every case record must be present.
  expected_task_ids = sorted(
    f"{cid}:{proc}" for cid, rec in by_id.items() for proc in rec["processes"]
  )
  if sorted(refs) != expected_task_ids:
    die("assembled task identities do not match harness case process sets")

  ordered_cases = [by_id[cid] for cid in sorted(by_id)]
  ordered_refs = {k: refs[k] for k in sorted(refs)}

  # Match validator/trusted-inventory aggregate digests exactly: sorted arrays of
  # records (not case-id-keyed dicts).
  sources_payload = [
    {
      "case_id": cid,
      "source": by_id[cid]["source"],
      "source_bytes": by_id[cid]["source_bytes"],
      "source_sha256": by_id[cid]["source_sha256"],
    }
    for cid in sorted(by_id)
  ]
  params_payload = [
    {"case_id": cid, "params_digest": by_id[cid]["params_digest"]}
    for cid in sorted(by_id)
  ]
  sources_digest = sha256_bytes(canonical_json(sources_payload))
  params_digest_all = sha256_bytes(canonical_json(params_payload))

  manifest: dict[str, object] = {
    "cases": ordered_cases,
    "openpilot_sha": source_sha,
    "opendbc_sha": opendbc_sha,
    "params_digest": params_digest_all,
    "previous_accepted_tag": previous_accepted_tag,
    "refs": ordered_refs,
    "schema_version": 1,
    "sources_digest": sources_digest,
  }

  # Fail closed: every case/ref identity still carries digests after assembly.
  for cid, rec in by_id.items():
    require_digest(f"assembled case {cid} params_digest", rec["params_digest"])
    require_digest(f"assembled case {cid} source_sha256", rec["source_sha256"])
  for tid, entry in ordered_refs.items():
    require_digest(f"assembled ref {tid} sha256", entry["sha256"])
    require_digest(f"assembled ref {tid} params_digest", entry["params_digest"])
    require_digest(f"assembled ref {tid} source_sha256", entry["source_sha256"])
  require_digest("manifest sources_digest", sources_digest)
  require_digest("manifest params_digest", params_digest_all)

  manifest_bytes = canonical_json(manifest)
  (release / "manifest.json").write_bytes(manifest_bytes)
  (release / "ref_commit").write_text(source_sha + "\n", encoding="utf-8")

  rel_files = sorted(p.name for p in release.iterdir() if p.is_file())
  zsts = [n for n in rel_files if n.endswith(".zst")]
  if "ref_commit" not in rel_files or "manifest.json" not in rel_files:
    die("release missing ref_commit or manifest.json")
  if len(zsts) != EXPECTED_TASKS:
    die("zst count self-check failed")
  extras = [n for n in rel_files if n not in {"ref_commit", "manifest.json"} and not n.endswith(".zst")]
  if extras:
    die(f"unexpected release files: {extras}")

  # Independent rebuild check: re-canonicalize must be byte-identical.
  rebuild = canonical_json(json.loads(manifest_bytes.decode("utf-8")))
  if rebuild != manifest_bytes:
    die("manifest failed independent byte-identical rebuild check")

  manifest_sha = sha256_bytes(manifest_bytes)
  print(f"assembled release/ inventory=active tasks={len(zsts)} cases={len(case_ids)}")
  print(f"sha={source_sha} previous_accepted_tag={previous_accepted_tag}")
  print(f"manifest_sha256={manifest_sha}")
  print(f"sources_digest={sources_digest}")
  print(f"params_digest={params_digest_all}")
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
