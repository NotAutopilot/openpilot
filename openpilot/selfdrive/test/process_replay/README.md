# Process replay

Process replay is a regression test designed to identify any changes in the output of a process. This test replays a segment through individual processes and compares the output to a known good replay. Each make is represented in the test with a segment.

If the test fails, make sure that you didn't unintentionally change anything. If there are intentional changes, the reference logs will be updated.

Use `test_processes.py` to run the test locally.
Log files are cached by default. Use `DISABLE_FILEREADER_CACHE='1' test_processes.py` to disable caching.

Currently the following processes are tested:

* controlsd
* radard
* plannerd
* calibrationd
* dmonitoringd
* locationd
* paramsd
* ubloxd
* torqued

### Usage
```
Usage: test_processes.py [-h] [--whitelist-procs PROCS] [--whitelist-cars CARS] [--blacklist-procs PROCS]
                         [--blacklist-cars CARS] [--ignore-fields FIELDS] [--ignore-msgs MSGS] [--update-refs]
Regression test to identify changes in a process's output
optional arguments:
  -h, --help            show this help message and exit
  --whitelist-procs PROCS               Whitelist given processes from the test (e.g. controlsd)
  --whitelist-cars WHITELIST_CARS       Whitelist given cars from the test (e.g. HONDA)
  --blacklist-procs BLACKLIST_PROCS     Blacklist given processes from the test (e.g. controlsd)
  --blacklist-cars BLACKLIST_CARS       Blacklist given cars from the test (e.g. HONDA)
  --ignore-fields IGNORE_FIELDS         Extra fields or msgs to ignore (e.g. driverMonitoringState.events)
  --ignore-msgs IGNORE_MSGS             Msgs to ignore (e.g. onroadEvents)
  --update-refs                         Updates reference logs using current commit
```

## Fork references

`naponsp-dev` reads references from [`NotAutopilot/ci-artifacts@process-replay-naponsp`](https://github.com/NotAutopilot/ci-artifacts/tree/process-replay-naponsp). The checked-in `ref_commit` pins the parent revision used to generate them, independently of the artifact branch's latest commit.

The initial reference is pre-feature parent `b688860729d5c9c1510f4902963ee119d6082040`, with opendbc `f9c9d0e3bc6f5f0c44aabefa056772146e375537`. Its 66 replay cases matched candidate `5758742aa2f7f8bb11b4ae594ef3960ccbd30319` using the unchanged comparison settings. This preserves the fork's `carStateSP` output rather than hiding it to match upstream references. The ordinary NAP `process-replay` artifact branch is separate and unchanged.

For a reference update, run the complete matrix on x86-64 Linux at the proposed reference revision, review the output differences, and obtain maintainer approval. Publish the resulting revision-named `.zst` files and `ref_commit` to `process-replay-naponsp`, then update this repository's pin and rerun the complete matrix against the published files. Do not narrow the process set, remove message streams, or loosen tolerances to accept a change.

The existing matrix uses 16 public CI routes. Its Tesla route is Model Y; it does not replace Pre-AP-specific replay or vehicle validation.

Other forks can use their own artifact URL, or keep revision-named logs in the local `fakedata` directory. `test_processes.py` saves generated logs there. Keep the logs and `ref_commit` aligned.

## API

Process replay test suite exposes programmatic APIs for simultaneously running processes or groups of processes on provided logs.

```py
def replay_process_with_name(name: Union[str, Iterable[str]], lr: LogIterable, *args, **kwargs) -> List[capnp._DynamicStructReader]:

def replay_process(
  cfg: Union[ProcessConfig, Iterable[ProcessConfig]], lr: LogIterable, frs: Optional[Dict[str, Any]] = None,
  fingerprint: Optional[str] = None, return_all_logs: bool = False, custom_params: Optional[Dict[str, Any]] = None, disable_progress: bool = False
) -> List[capnp._DynamicStructReader]:
```

Example usage:
```py
from openpilot.selfdrive.test.process_replay import replay_process_with_name
from openpilot.tools.lib.logreader import LogReader

lr = LogReader(...)

# provide a name of the process to replay
output_logs = replay_process_with_name('locationd', lr)

# or list of names
output_logs = replay_process_with_name(['ubloxd', 'locationd'], lr)
```

Supported processes:
* controlsd
* radard
* plannerd
* calibrationd
* dmonitoringd
* locationd
* paramsd
* ubloxd
* torqued
* modeld
* dmonitoringmodeld

Certain processes may require an initial state, which is usually supplied within `Params` and persisting from segment to segment (e.g CalibrationParams, LiveParameters). The `custom_params` is dictionary  used to prepopulate `Params` with arbitrary values. The `get_custom_params_from_lr` helper is provided to fetch meaningful values from log files.

```py
from openpilot.selfdrive.test.process_replay import get_custom_params_from_lr

previous_segment_lr = LogReader(...)
current_segment_lr = LogReader(...)

custom_params = get_custom_params_from_lr(previous_segment_lr, 'last')

output_logs = replay_process_with_name('calibrationd', lr, custom_params=custom_params)
```

Replaying processes that use VisionIPC (e.g. modeld, dmonitoringmodeld) require additional `frs` dictionary with camera states as keys and `FrameReader` objects as values.

```py
from openpilot.tools.lib.framereader import FrameReader

frs = {
  'roadCameraState': FrameReader(...),
  'wideRoadCameraState': FrameReader(...),
  'driverCameraState': FrameReader(...),
}

output_logs = replay_process_with_name(['modeld', 'dmonitoringmodeld'], lr, frs=frs)
```

To capture stdout/stderr of the replayed process, `captured_output_store` can be provided.

```py
output_store = dict()
# pass dictionary by reference, it will be filled with standard outputs - even if process replay fails
output_logs = replay_process_with_name(['radard', 'plannerd'], lr, captured_output_store=output_store)

# entries with captured output in format { 'out': '...', 'err': '...' } will be added to provided dictionary for each replayed process
print(output_store['radard']['out']) # radard stdout
print(output_store['radard']['err']) # radard stderr
```
