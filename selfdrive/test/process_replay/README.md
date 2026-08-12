# Process replay

Process replay is a regression test that replays recorded source logs through individual openpilot processes and compares each process output against a known reference.

## Cases and tasks

A **case** is one source input: car brand, source locator, selected processes, and optional typed custom params.
A **task** is one process on one case. Task IDs are canonical: `{case_id}:{process}`.

The active inventory is the legacy 16 sources / 66 tasks:
- `card`, `controlsd`, and `lagd` on every case
- the other nine non-model processes only on `HYUNDAI` and `TOYOTA`

`--inventory staged` also lists two temporary Pre-AP descriptors (`PENDING_CASES`) for a total of 18 cases / 78 tasks. Those pending cases are listing/generation descriptors only until sanitized fixture URLs and digests are filled in; they are not executable in normal active runs.

`source_segments` remains the regen compatibility list used by `regen_all.py`.

## Running locally

```bash
# full active suite
./test_processes.py -j$(nproc)

# shard-style filters (diagnostics go to --output-dir)
./test_processes.py -j$(nproc) --whitelist-procs card --output-dir /tmp/replay/card
./test_processes.py -j$(nproc) --whitelist-procs controlsd --output-dir /tmp/replay/controlsd
./test_processes.py -j$(nproc) --whitelist-procs lagd --output-dir /tmp/replay/lagd
./test_processes.py -j$(nproc) --blacklist-procs card controlsd lagd --output-dir /tmp/replay/other

# inventories
./test_processes.py --list-cases
./test_processes.py --list-tasks
./test_processes.py --inventory staged --list-tasks
```

Log downloads are cached by default. Disable with `DISABLE_FILEREADER_CACHE=1`.

### CLI

```
Usage: test_processes.py [-h]
                         [--whitelist-procs PROCS] [--whitelist-cars CARS]
                         [--blacklist-procs PROCS] [--blacklist-cars CARS]
                         [--ignore-fields FIELDS] [--ignore-msgs MSGS]
                         [--update-refs] [--inventory {active,staged}]
                         [--list-cases] [--list-tasks]
                         [--output-dir DIR] [--reference-dir DIR] [-j JOBS]
```

- `--output-dir` only changes where `diff.txt` and `results.json` are written.
- `--reference-dir` selects an already-generated local candidate tree for comparison.
- `--update-refs` writes candidate refs for the selected tasks. It does not mark those candidates as accepted; acceptance is a second normal run against `--reference-dir`.

## References and diagnostics

Remote references are resolved from one pinned `NotAutopilot/ci-artifacts` commit on `process-replay`, not from a moving branch URL. Filenames use `{case_id}__{process}__{ref_commit}.zst`. Manifest-era trees require `manifest.json` and canonical filenames. Pre-bootstrap segment filenames are allowed only when the pinned artifacts commit is listed in `LEGACY_ARTIFACTS_COMMITS`.

Each selected case is downloaded once. When a case declares size/SHA-256, both are checked before replay. A digest failure becomes a task error for every task on that case; other cases continue.

Workers return serializable `TaskResult` values. One task failure does not abort the pool. `diff.txt` and schema-versioned `results.json` are created before downloads start and rewritten after every settled task. The process exits nonzero on any diff, error, or incomplete inventory. After a normal compare run, only failed/different generated logs are kept; stale `fakedata` is cleared at start.

## Params

Runtime custom params stay native Python types (`bool`, `int`, `float`) and are validated against `common/params_keys.h` before scheduling. Manifest digests hash a separate canonical JSON array sorted by key with explicit `type` tags.

## API

Process replay also exposes programmatic helpers for replaying processes on provided logs:

```py
def replay_process_with_name(name: Union[str, Iterable[str]], lr: LogIterable, *args, **kwargs) -> List[capnp._DynamicStructReader]:

def replay_process(
  cfg: Union[ProcessConfig, Iterable[ProcessConfig]], lr: LogIterable, frs: Optional[Dict[str, Any]] = None,
  fingerprint: Optional[str] = None, return_all_logs: bool = False, custom_params: Optional[Dict[str, Any]] = None, disable_progress: bool = False
) -> List[capnp._DynamicStructReader]:
```

Example:

```py
from openpilot.selfdrive.test.process_replay import replay_process_with_name
from openpilot.tools.lib.logreader import LogReader

lr = LogReader(...)
output_logs = replay_process_with_name('locationd', lr)
output_logs = replay_process_with_name(['ubloxd', 'locationd'], lr)
```

Supported processes include `controlsd`, `radard`, `plannerd`, `calibrationd`, `dmonitoringd`, `locationd`, `paramsd`, `ubloxd`, `torqued`, `card`, `lagd`, `selfdrived`, `modeld`, and `dmonitoringmodeld`.

Use `custom_params` to seed `Params` with typed values. `get_custom_params_from_lr` can recover meaningful values from a previous segment. VisionIPC processes need an `frs` map of camera state names to `FrameReader` objects. Pass `captured_output_store` to collect stdout/stderr per process.
