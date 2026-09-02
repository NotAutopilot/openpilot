# Off-car Pre-AP replay

Replay a recorded Pre-AP route through openpilot `process_replay` and assert the pedal / chime / radar contracts we actually hit on the road. This is log replay, not a driving simulator. CARLA is gone from comma openpilot; MetaDrive is a later phase.

Jack's car: dongle `d0cdc986c5d023f5`, Comma Pedal `0x551` / `0x552`, no DAS.

## One command

From the repo root, with the venv active:

```bash
python -m pytest selfdrive/selfdrived/tests/test_preap_chimes.py -q
python -m openpilot.selfdrive.test.process_replay.preap_offcar --list
python -m openpilot.selfdrive.test.process_replay.preap_offcar --fixture
python -m openpilot.selfdrive.test.process_replay.preap_offcar 00000074--113899b226--28
```

The product test is `test_preap_chimes.py`. It is not the upstream TESLA process_replay case (that's Model Y, and that whitelist skips selfdrived/radard). Replay procs are `selfdrived` and `radard`. `DisengageOnAccelerator=False` is required so gas override stays engaged. rlogs only; qlogs drop 100Hz carState edges.

`--list` only prints routes already on disk. It will not invent a route name.

`--fixture` runs the synthetic engage / gas-override / brake / cancel stream. That is the CI path when no rlogs are present.

A query can be a segment name, a log id, a dongle|route, or a path to an `rlog.zst`.

Scan the recorded messages without rerunning processes:

```bash
python -m openpilot.selfdrive.test.process_replay.preap_offcar --scan-only /path/to/rlog.zst
```

`--scan-only` is what the device logged (possibly old code). Default replay runs current `selfdrived` / `radard` on logged `carState` (card/0x551 decode comes later).

## Where logs go

The harness searches, in order:

1. `PREAP_OFFCAR_DATA_DIR`
2. `--data-dir` (repeatable)
3. `openpilot-nap/logs` if present
4. `~/projects/personal/notautopilot/logs` (VPS copies of Jack's routes)
5. `~/.commacache`
6. `/data/media/0/realdata` and `/data` (device)
7. `~/backups/notautopilot/comma-d0cdc986c5d023f5`

Drop explorer-style files:

```
d0cdc986c5d023f5_<log_id>--<seg>--rlog.zst
```

or device directories:

```
d0cdc986c5d023f5|<log_id>--<seg>/rlog.zst
```

If this machine has no rlogs, `--list` prints empty and `--fixture` still passes. Copy a segment into one of the paths above (or set `PREAP_OFFCAR_DATA_DIR`) and rerun the query.

Existing openpilot tools still work on the same files:

```bash
python selfdrive/debug/run_process_on_route.py <route> --whitelist-procs selfdrived
tools/replay/replay <route> --data_dir=/path/to/parent
```

## What it asserts

- Lat engage / disengage chimes on `cruiseState.enabled` (`pcmEnable` / `pcmDisable`, `selfdriveState.alertSound`)
- Long engage / disengage chimes on `enableLongControl` (`pedalCruiseEnabled` / `pedalCruiseDisabled`, `alertType` / `alertSound`)
- Gas override must not chime and must not drop `enableLongControl` (`update_preap_chimes`, `b568c89384`)
- Pedal CAN `0x551` / `0x552` presence
- Bosch radar: `0x301` SGU (HWFail / SGUFail / dirty), point CAN `0x310+`, `liveTracks` if those frames exist
- Donor / GTW identity if `0x2A9` / `0x2B9` / `0x560` / UDS `0x641`/`0x651` are in the log
- DAS `0x3E9` presence (pre-AP should not have a DAS)

Missing radar or a route with no gas-override is a skip, not a fail. A skip means this log cannot speak to that bug.

The printed report lists processes that ran, each assertion, and which CAN / cereal signals showed up.

## Tests

```bash
python -m pytest selfdrive/selfdrived/tests/test_preap_chimes.py -q
python -m pytest selfdrive/test/process_replay/test_preap_offcar.py -q
```
