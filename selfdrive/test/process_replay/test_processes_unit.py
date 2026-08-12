#!/usr/bin/env python3
import concurrent.futures
import json
import os
import tempfile
import pickle
from pathlib import Path
import pytest

from openpilot.common.params import ParamKeyType, Params
from openpilot.common.prefix import OpenpilotPrefix
from openpilot.selfdrive.test.process_replay import test_processes as tp


def test_active_inventory_counts():
  cases = tp.select_cases("active")
  tasks = tp.build_tasks(cases)
  assert len(cases) == 16
  assert len(tasks) == 66
  assert len({c.case_id for c in cases}) == 16
  assert len({t.task_id for t in tasks}) == 66
  assert len(tp.shard_task_ids(tasks, "card")) == 16
  assert len(tp.shard_task_ids(tasks, "controlsd")) == 16
  assert len(tp.shard_task_ids(tasks, "lagd")) == 16
  assert len(tp.shard_task_ids(tasks, "other")) == 18
  tp.assert_inventory_invariants(cases)


def test_staged_inventory_counts_and_pending_not_executable():
  cases = tp.select_cases("staged")
  tasks = tp.build_tasks(cases)
  assert len(cases) == 18
  assert len(tasks) == 78
  assert len(tp.PENDING_CASES) == 2
  assert all(not c.executable for c in tp.PENDING_CASES)
  assert len(tp.shard_task_ids(tasks, "card")) == 18
  assert len(tp.shard_task_ids(tasks, "controlsd")) == 18
  assert len(tp.shard_task_ids(tasks, "lagd")) == 18
  assert len(tp.shard_task_ids(tasks, "other")) == 24
  union: set[str] = set()
  for shard in ("card", "controlsd", "lagd", "other"):
    ids = tp.shard_task_ids(tasks, shard)
    assert union.isdisjoint(ids)
    union.update(ids)
  assert union == {t.task_id for t in tasks}
  tp.assert_inventory_invariants(cases)


def test_source_segments_compat():
  assert len(tp.source_segments) == 17
  assert tp.source_segments[0][0] == "HYUNDAI"
  brands = [c for c, _ in tp.source_segments]
  assert "TOYOTA2" in brands
  assert len(tp.segments) == 16
  assert "TOYOTA2" not in [c for c, _ in tp.segments]


def test_filter_whitelist_blacklist_semantics():
  tasks = tp.build_tasks(tp.ACTIVE_CASES)
  only_card = tp.filter_tasks(tasks, whitelist_procs=["card"], blacklist_procs=[], whitelist_cars=None, blacklist_cars=[])
  assert {t.process for t in only_card} == {"card"}
  assert len(only_card) == 16

  no_core = tp.filter_tasks(
    tasks,
    whitelist_procs=None,
    blacklist_procs=["card", "controlsd", "lagd"],
    whitelist_cars=None,
    blacklist_cars=[],
  )
  assert len(no_core) == 18
  assert {"card", "controlsd", "lagd"}.isdisjoint({t.process for t in no_core})

  honda = tp.filter_tasks(tasks, whitelist_procs=["card"], blacklist_procs=[], whitelist_cars=["honda"], blacklist_cars=[])
  assert {t.case.car_brand for t in honda} == {"HONDA"}
  assert [t.task_id for t in honda] == ["honda:card"]


def test_empty_and_fixed_digests():
  assert tp.params_digest({}) == tp.EMPTY_PARAMS_DIGEST
  assert tp.params_digest(tp.NAP_PREAP_PEDAL_PARAMS) == tp.NAP_PREAP_PEDAL_PARAMS_DIGEST
  assert tp.params_digest(tp.NAP_PREAP_NO_PEDAL_PARAMS) == tp.NAP_PREAP_NO_PEDAL_PARAMS_DIGEST
  manifest = tp.canonical_params_manifest(tp.NAP_PREAP_PEDAL_PARAMS)
  assert manifest.endswith(b"\n")
  assert b" " not in manifest


def test_header_type_validation_and_rejection():
  tp.validate_params_against_header(tp.NAP_PREAP_PEDAL_PARAMS)
  with pytest.raises(TypeError):
    tp.validate_params_against_header({"NAPFollowDistance": 1.5})
  with pytest.raises(TypeError):
    tp.validate_params_against_header({"NAPBrakeFactor": 1})
  with pytest.raises(TypeError):
    tp.validate_params_against_header({"NAPForcePreAP": 1})


def test_params_round_trip_native_types():
  with OpenpilotPrefix():
    params = Params()
    for key, value in tp.NAP_PREAP_PEDAL_PARAMS.items():
      declared = params.get_type(key)
      if declared == ParamKeyType.BOOL:
        assert isinstance(value, bool)
        params.put_bool(key, value)
        assert params.get_bool(key) is value
      elif declared == ParamKeyType.INT:
        assert isinstance(value, int)
        assert not isinstance(value, bool)
        params.put(key, value)
        assert params.get(key) == value
      elif declared == ParamKeyType.FLOAT:
        assert isinstance(value, float)
        params.put(key, value)
        assert params.get(key) == value
      else:
        raise AssertionError(f"unexpected type for {key}: {declared}")


def test_ref_filename_and_case_id_validation():
  sha = "a" * 40
  assert tp.ref_filename("hyundai", "card", sha) == f"hyundai__card__{sha}.zst"
  with pytest.raises(ValueError):
    tp.ref_filename("HYUNDAI", "card", sha)
  with pytest.raises(ValueError):
    tp.validate_case_id("bad_id")
  with pytest.raises(ValueError):
    tp.ref_filename("hyundai", "card", "abc")


def test_pin_and_artifacts_url_require_full_sha():
  class FakeResp:
    def __enter__(self):
      return self

    def __exit__(self, *args):
      return False

    def read(self, *args, **kwargs):
      return json.dumps({"object": {"sha": "a" * 40}}).encode()

  sha = tp.pin_artifacts_commit(opener=lambda *a, **k: FakeResp())
  assert sha == "a" * 40
  url = tp.artifacts_file_url(sha, "ref_commit")
  assert url == f"https://raw.githubusercontent.com/NotAutopilot/ci-artifacts/{sha}/ref_commit"
  with pytest.raises(ValueError):
    tp.artifacts_file_url("process-replay", "ref_commit")


def test_resolve_ref_path_prefers_local_new_name_then_legacy():
  case = tp.ACTIVE_CASES[0]
  task = tp.ReplayTask(case=case, process="card")
  ref = "d" * 40
  with tempfile.TemporaryDirectory() as td:
    new_name = tp.ref_filename(case.case_id, "card", ref)
    legacy = tp.legacy_ref_filename(case.source, "card", ref)
    Path(td, new_name).write_bytes(b"new")
    path = tp.resolve_ref_path(task, ref_commit=ref, artifacts_commit="b" * 40, reference_dir=td, update_refs=False)
    assert Path(path).resolve() == (Path(td) / new_name).resolve()
    os.remove(os.path.join(td, new_name))
    Path(td, legacy).write_bytes(b"legacy")
    path = tp.resolve_ref_path(task, ref_commit=ref, artifacts_commit="b" * 40, reference_dir=td, update_refs=False)
    assert Path(path).resolve() == (Path(td) / legacy).resolve()


def test_resolve_ref_path_remote_legacy_for_route_cases(monkeypatch):
  case = tp.ACTIVE_CASES[0]
  task = tp.ReplayTask(case=case, process="card")
  commit = "c" * 40
  ref = "d" * 40
  monkeypatch.setattr(tp, "LEGACY_ARTIFACTS_COMMITS", frozenset({commit}))
  path = tp.resolve_ref_path(task, ref_commit=ref, artifacts_commit=commit, reference_dir=None, update_refs=False)
  legacy = tp.legacy_ref_filename(case.source, "card", ref)
  assert path == tp.artifacts_file_url(commit, legacy)


def test_resolve_ref_path_rejects_missing_manifest_outside_legacy():
  case = tp.ACTIVE_CASES[0]
  task = tp.ReplayTask(case=case, process="card")
  with pytest.raises(ValueError, match="LEGACY_ARTIFACTS_COMMITS"):
    tp.resolve_ref_path(
      task,
      ref_commit="d" * 40,
      artifacts_commit="c" * 40,
      reference_dir=None,
      update_refs=False,
      artifacts_manifest=None,
    )


def test_incremental_diagnostics_and_nonzero_incomplete():
  with tempfile.TemporaryDirectory() as td:
    expected = ["hyundai:card", "hyundai:controlsd"]
    settled = {
      "hyundai:card": tp.TaskResult("pass", "hyundai:card", 0.1, "", "ref", "new", {"carState": 1}),
    }
    doc = tp.build_results_document(
      tested_commit="t" * 40,
      ref_commit="a" * 40,
      artifacts_commit="a" * 40,
      inventory="active",
      cases=tp.ACTIVE_CASES[:1],
      expected_task_ids=expected,
      results=settled,
    )
    assert doc["schema_version"] == tp.RESULTS_SCHEMA_VERSION
    assert doc["status_counts"]["incomplete"] == 1
    assert tp.exit_code_for(doc) == 1
    tp.write_diagnostics(td, doc, list(settled.values()), "r" * 40)
    assert os.path.exists(os.path.join(td, "results.json"))
    assert os.path.exists(os.path.join(td, "diff.txt"))
    loaded = json.loads(Path(td, "results.json").read_text())
    assert loaded["incomplete_task_ids"] == ["hyundai:controlsd"]


def test_injected_worker_error_then_later_success():
  payloads = [
    {"task_id": "a:card", "ref_log_path": "", "new_log_path": ""},
    {"task_id": "b:card", "ref_log_path": "", "new_log_path": ""},
    {"task_id": "c:card", "ref_log_path": "", "new_log_path": ""},
  ]

  def worker(payload):
    if payload["task_id"] == "a:card":
      raise RuntimeError("injected worker boom")
    return tp.TaskResult(
      status="pass",
      task_id=payload["task_id"],
      elapsed_s=0.01,
      diff_or_traceback="",
      ref_path="",
      new_path="",
      message_counts={},
    )

  settled: list[tp.TaskResult] = []
  results = tp.run_task_pool(
    payloads,
    jobs=2,
    worker=worker,
    on_settled=settled.append,
    executor_factory=concurrent.futures.ThreadPoolExecutor,
  )
  by_id = {r.task_id: r for r in results}
  assert set(by_id) == {"a:card", "b:card", "c:card"}
  assert by_id["a:card"].status == "error"
  assert "injected worker boom" in by_id["a:card"].diff_or_traceback
  assert by_id["b:card"].status == "pass"
  assert by_id["c:card"].status == "pass"
  assert len(settled) == 3


def test_run_test_process_serializes_exceptions():
  payload = {
    "task_id": "hyundai:card",
    "case_id": "hyundai",
    "car_brand": "HYUNDAI",
    "source": "not-a-real-segment|--0",
    "processes": ["card"],
    "custom_params": {},
    "process": "card",
    "lr_dat": b"not-a-log",
    "ref_log_path": "",
    "new_log_path": os.path.join(tempfile.gettempdir(), "nap-replay-unit-new.zst"),
    "ignore_fields": [],
    "ignore_msgs": [],
    "update_refs": True,
  }
  result = tp.run_test_process(payload)
  assert isinstance(result, tp.TaskResult)
  assert result.status == "error"
  assert result.task_id == "hyundai:card"
  assert result.diff_or_traceback


def test_download_digest_mismatch_is_error(monkeypatch):
  case = tp.ReplayCase(
    case_id="fixture-case",
    car_brand="TESLA",
    source="https://example.invalid/rlog.zst",
    processes=("card",),
    source_sha256="0" * 64,
    source_bytes=4,
  )

  class FakeFile:
    def __enter__(self):
      return self

    def __exit__(self, *args):
      return False

    def read(self, *args, **kwargs):
      return b"abcd"

  monkeypatch.setattr(tp, "FileReader", lambda *a, **k: FakeFile())
  with pytest.raises(ValueError, match="sha256 mismatch"):
    tp.download_case_bytes(case)


def test_duplicate_case_id_rejected():
  case = tp.ACTIVE_CASES[0]
  with pytest.raises(AssertionError):
    tp.assert_inventory_invariants([case, case])

def test_case_dedup_does_not_hash_custom_params():
  case = tp.ReplayCase(
    case_id="dedup-case",
    car_brand="TESLA",
    source="",
    processes=("card",),
    custom_params={"NAPForcePreAP": True},
  )
  tasks = [tp.ReplayTask(case=case, process="card"), tp.ReplayTask(case=case, process="card")]
  deduped = tp.unique_cases_by_id(tasks)
  assert deduped == {"dedup-case": case}


def test_executable_source_metadata_is_exact():
  with pytest.raises(ValueError, match="requires both"):
    tp.ReplayCase(case_id="missing-metadata", car_brand="TESLA", source="https://example.invalid/rlog.zst",
                  processes=("card",))
  with pytest.raises(ValueError, match="omit"):
    tp.ReplayCase(case_id="pending-metadata", car_brand="TESLA", source="",
                  processes=("card",), source_bytes=1, source_sha256="0" * 64)


def test_reference_dir_owns_ref_commit_and_candidate_metadata():
  with tempfile.TemporaryDirectory() as td:
    ref_commit = "a" * 40
    tp.write_candidate_ref_commit(ref_commit, td)
    assert tp.resolve_ref_commit(reference_dir=td) == (ref_commit, None)
    assert Path(td, "ref_commit").read_text() == f"{ref_commit}\n"


def test_manifest_requires_canonical_remote_filename():
  case = tp.ACTIVE_CASES[0]
  task = tp.ReplayTask(case=case, process="card")
  with pytest.raises(ValueError, match="non-canonical filename"):
    tp.resolve_ref_path(
      task,
      ref_commit="a" * 40,
      artifacts_commit="a" * 40,
      reference_dir=None,
      update_refs=False,
      artifacts_manifest={"refs": {task.task_id: {"filename": "legacy.zst"}}},
    )


def test_main_records_all_tasks_when_setup_fails():
  with tempfile.TemporaryDirectory() as td:
    output_dir = os.path.join(td, "diagnostics")
    result = tp.main(
      ["--whitelist-procs", "card", "--output-dir", output_dir],
      get_commit_fn=lambda: (_ for _ in ()).throw(RuntimeError("setup failed")),
      clear_stale_fakedata_fn=lambda *_: pytest.fail("cleanup must follow initial diagnostics"),
    )
    assert result == 1
    document = json.loads(Path(output_dir, "results.json").read_text())
    assert document["status_counts"]["error"] == 16
    assert document["status_counts"]["incomplete"] == 0

def test_generated_log_is_saved_before_reference_read(monkeypatch):
  from types import SimpleNamespace

  case = tp.ACTIVE_CASES[0]
  task = tp.ReplayTask(case=case, process="card")
  cfg = SimpleNamespace(proc_name="card", pubs=set(), subs={"testMsg"}, ignore=[], tolerance=0)
  message = SimpleNamespace(which=lambda: "testMsg")
  saved: list[str] = []
  monkeypatch.setattr(tp, "replay_process", lambda *args, **kwargs: [message])
  monkeypatch.setattr(tp, "check_most_messages_valid", lambda messages: True)
  monkeypatch.setattr(tp, "save_log", lambda path, messages: saved.append(path))
  monkeypatch.setattr(tp, "load_ref_log_messages", lambda path: (_ for _ in ()).throw(OSError("missing ref")))

  status, detail, _ = tp.test_process(cfg, [], task, "/missing/ref.zst", "/candidate/new.zst")
  assert status == "error"
  assert "missing ref" in detail
  assert saved == ["/candidate/new.zst"]


def test_task_result_is_pickleable():
  result = tp.TaskResult("pass", "case:card", 1.25, "", "/ref.zst", "/new.zst", {"carState": 3})
  restored = pickle.loads(pickle.dumps(result))
  assert restored == result
  assert restored.to_record()["message_counts"] == {"carState": 3}


def test_validate_ref_commit_rejects_malformed_identities():
  good = "ab" * 20
  assert tp.validate_ref_commit(good) == good
  for bad in ("abc", "A" * 40, "ab" * 19, "ab" * 21, "g" * 40, "a" * 39 + "/", "a" * 39 + "?", "a" * 39 + "#", "a" * 39 + '\n', "a" * 39 + " "):
    with pytest.raises(ValueError):
      tp.validate_ref_commit(bad)


def test_same_dir_collision_prefers_canonical_over_legacy():
  case = tp.ACTIVE_CASES[0]
  task = tp.ReplayTask(case=case, process="card")
  ref = "e" * 40
  with tempfile.TemporaryDirectory() as td:
    new_name = tp.ref_filename(case.case_id, "card", ref)
    legacy = tp.legacy_ref_filename(case.source, "card", ref)
    Path(td, new_name).write_bytes(b"canonical")
    Path(td, legacy).write_bytes(b"legacy")
    path = tp.resolve_ref_path(task, ref_commit=ref, artifacts_commit="b" * 40, reference_dir=td, update_refs=False)
    assert Path(path).name == new_name
    assert Path(path).read_bytes() == b"canonical"


def test_symlink_and_outside_local_refs_are_rejected():
  case = tp.ACTIVE_CASES[0]
  task = tp.ReplayTask(case=case, process="card")
  ref = "f" * 40
  with tempfile.TemporaryDirectory() as td:
    root = Path(td)
    outside = root / "outside.txt"
    outside.write_text("secret" + '\n', encoding="utf-8")
    (root / "ref_commit").symlink_to(outside)
    with pytest.raises(ValueError, match="symlink"):
      tp.resolve_ref_commit(reference_dir=str(root))
    (root / "ref_commit").unlink()
    (root / "ref_commit").write_text(ref + '\n', encoding="utf-8")
    new_name = tp.ref_filename(case.case_id, "card", ref)
    (root / new_name).symlink_to(outside)
    with pytest.raises(ValueError, match="symlink"):
      tp.resolve_ref_path(task, ref_commit=ref, artifacts_commit="b" * 40, reference_dir=str(root), update_refs=False)
    (root / new_name).unlink()
    with pytest.raises(ValueError, match="basename"):
      tp.resolve_regular_member(str(root), "../outside.txt")


def test_per_task_ref_failure_continues_other_tasks(monkeypatch):
  with tempfile.TemporaryDirectory() as td:
    output_dir = os.path.join(td, "out")
    ref_dir = os.path.join(td, "refs")
    os.makedirs(ref_dir)
    Path(ref_dir, "ref_commit").write_text(("a" * 40) + '\n', encoding="utf-8")

    real_resolve = tp.resolve_ref_path

    def fake_resolve(task, **kwargs):
      if task.process == "controlsd":
        raise ValueError(f"injected ref failure for {task.task_id}")
      return real_resolve(task, **kwargs)

    monkeypatch.setattr(tp, "resolve_ref_path", fake_resolve)

    def fake_pool(payloads, jobs, on_settled=None, **kwargs):
      results = []
      for payload in payloads:
        result = tp.TaskResult("pass", payload["task_id"], 0.01, "", payload.get("ref_log_path", ""), payload.get("new_log_path", ""), {})
        results.append(result)
        if on_settled is not None:
          on_settled(result)
      return results

    case = tp.ACTIVE_CASES[0]

    def fake_download(payload):
      return payload["case_id"], b"not-used", None

    monkeypatch.setattr(tp.concurrent.futures, "ProcessPoolExecutor", concurrent.futures.ThreadPoolExecutor)

    code = tp.main(
      ["--whitelist-procs", "card", "controlsd", "--whitelist-cars", case.car_brand,
       "--output-dir", output_dir, "--reference-dir", ref_dir, "-j", "1"],
      get_commit_fn=lambda: "b" * 40,
      download_case_by_id_fn=fake_download,
      run_task_pool_fn=fake_pool,
      clear_stale_fakedata_fn=lambda *_: None,
    )
    assert code == 1
    document = json.loads(Path(output_dir, "results.json").read_text())
    by_id = {row["task_id"]: row for row in document["results"]}
    assert by_id[f"{case.case_id}:controlsd"]["status"] == "error"
    assert "injected ref failure" in by_id[f"{case.case_id}:controlsd"]["diff_or_traceback"]
    assert by_id[f"{case.case_id}:card"]["status"] == "pass"


def test_on_settled_flushes_diagnostics_each_callback(monkeypatch):
  with tempfile.TemporaryDirectory() as td:
    output_dir = os.path.join(td, "out")
    snapshots = []

    def fake_pool(payloads, jobs, on_settled=None, **kwargs):
      assert on_settled is not None
      first = tp.TaskResult("pass", payloads[0]["task_id"], 0.01, "", "", "", {})
      on_settled(first)
      snapshots.append({row["task_id"] for row in json.loads(Path(output_dir, "results.json").read_text())["results"]})
      assert Path(output_dir, "diff.txt").exists()
      second = tp.TaskResult("pass", payloads[1]["task_id"], 0.01, "", "", "", {})
      on_settled(second)
      snapshots.append({row["task_id"] for row in json.loads(Path(output_dir, "results.json").read_text())["results"]})
      return [first, second]

    case = tp.ACTIVE_CASES[0]
    monkeypatch.setattr(tp.concurrent.futures, "ProcessPoolExecutor", concurrent.futures.ThreadPoolExecutor)
    code = tp.main(
      ["--whitelist-procs", "card", "controlsd", "--whitelist-cars", case.car_brand,
       "--output-dir", output_dir, "--update-refs", "-j", "1"],
      get_commit_fn=lambda: "c" * 40,
      download_case_by_id_fn=lambda payload: (payload["case_id"], b"abc", None),
      run_task_pool_fn=fake_pool,
      clear_stale_fakedata_fn=lambda *_: None,
      write_candidate_ref_commit_fn=lambda *_: "",
    )
    assert code == 0
    assert len(snapshots[0]) == 1
    assert len(snapshots[1]) == 2
    assert snapshots[0].issubset(snapshots[1])


def test_manifest_missing_on_non_legacy_is_error():
  class Missing:
    def __init__(self, *a, **k):
      pass

    def read(self, *a, **k):
      raise tp.URLFileException("Remote file is empty or doesn't exist: x")

  with pytest.raises(ValueError, match="non-legacy"):
    tp.load_artifacts_manifest("a" * 40, expected_ref_commit="b" * 40, url_file_cls=Missing)


def test_manifest_empty_body_is_malformed_not_legacy(monkeypatch):
  monkeypatch.setattr(tp, "LEGACY_ARTIFACTS_COMMITS", frozenset({"a" * 40}))

  class Empty:
    def __init__(self, *a, **k):
      pass

    def read(self, *a, **k):
      return b""

  with pytest.raises(ValueError, match="empty body"):
    tp.load_artifacts_manifest("a" * 40, expected_ref_commit="b" * 40, url_file_cls=Empty)
