# iBooster bench session 1

This procedure is only for Ray's and pod's SGH-retrofit Pre-AP Model S bench cars. Run it parked, with ignition on, and with the car under direct supervision.

The session sends only zero-position iBooster commands on `0x553` bus 0. It does not characterize braking force or travel. It records how the ECU behaves during mode changes, one deliberate counter skip, and short transmit gaps.

## Do not use ALLOUTPUT

Do not run this session with ALLOUTPUT, all-output mode, or any tool that bypasses panda safety.

This bench tool uses `SAFETY_TESLA_PREAP` with the iBooster bench flag. Panda permits only `0x553` on bus 0 with length 6, valid CRC, mode 0 or 2, and 0 mm position. Steering, pedal, stalk, and every other transmit message stay blocked.

## Prerequisites

- SGH-retrofit Pre-AP Model S test car: Ray's car or pod's car.
- Car parked on level ground, ignition on, wheels chocked if available.
- Comma/panda connected normally to the car.
- Current `nap-ibooster-dev` build installed.
- Driver seat occupied or a second person watching the car.
- Foot off the brake and accelerator before pressing Enter on the command.

Stop if the car is not parked, if the ignition is off, if any warning appears, or if anyone is unsure what the command is about to do.

## Connect

1. Park the car and turn ignition on.
2. Connect the comma/panda as usual.
3. Open a terminal on the comma or SSH into it.
4. Stop openpilot so the bench tool can talk to the panda directly. It
   holds the panda's USB interface while running. The device screen goes
   dark until the reboot at the end — that is normal:

```bash
sudo systemctl stop comma
```

5. Go to openpilot:

```bash
cd /data/openpilot
```

## Run

Use the car name that matches the car in front of you.

Ray's car:

```bash
cd /data/openpilot && source .venv/bin/activate && python -m opendbc.car.tesla.preap.ibooster_session1_bench --car ray --output-dir /data/openpilot/ibooster-bench-runs
```

pod's car:

```bash
cd /data/openpilot && source .venv/bin/activate && python -m opendbc.car.tesla.preap.ibooster_session1_bench --car pod --output-dir /data/openpilot/ibooster-bench-runs
```

Do not run any other command for this session.

## Healthy output

Healthy startup looks like this:

```text
iBooster session 1 bench
Car: ray
Safety: SAFETY_TESLA_PREAP with iBooster bench flag; ALLOUTPUT is not used
Healthy: 0x554 Status=NO_FAULT BrakeOK=1 DriverBrakeApplied=0 0x39D readiness=7
Running: mode_0_zero_hold
Running: mode_2_zero_hold
Running: transition_0_to_2_zero
Running: transition_2_to_0_zero
Running: counter_skip_zero
Running: tx_gap_sweep_mode_2_zero
Complete. Output file: /data/openpilot/ibooster-bench-runs/ray-YYYYMMDDTHHMMSSZ-ibooster-session1.json
```

The readiness number may be different on some firmware. It must stay stable for the whole run.

A normal run takes about 30 seconds. Leave the car parked and do not press brake or accelerator while it runs.

## Stop now output

Stop immediately if the screen prints `STOP NOW:`. Do not restart the session until the output file has been reviewed.

Stop-now lines are exact and look like one of these:

```text
STOP NOW: 0x554 Status != NO_FAULT
STOP NOW: 0x554 BrakeOK == 0
STOP NOW: 0x554 DriverBrakeApplied == 1
STOP NOW: 0x39D readiness changed
STOP NOW: RX loss
```

If this happens:

1. Leave the car parked.
2. Do not press the accelerator.
3. If the car shows a brake or stability warning, turn ignition off and wait before trying anything else.
4. Save the output file path shown below the stop message.
5. Send the JSON file back with a short note describing what the car did, if anything.

## FAULT OBSERVED output

If the screen prints `FAULT OBSERVED`, let the session finish. In the counter-skip and transmit-gap checks, a fault can be the thing being measured, not a reason to stop the run. Losing the status stream during those checks also shows `FAULT OBSERVED`, and the tool keeps sending mode 0 at 0 mm through the observation window. The JSON will say `result: fault_observed` and `cleared_by_mode_0: true` or `false`.

If `cleared_by_mode_0` is `false`, turn ignition off for 10 seconds, turn it back on, and note whether any brake warning appears or stays on. Send that note with the JSON file.

## Output file

The run writes one JSON file:

```text
/data/openpilot/ibooster-bench-runs/<car>-YYYYMMDDTHHMMSSZ-ibooster-session1.json
```

The file contains raw transmitted `0x553` bytes with timestamps, all accepted `0x554` Status frames, all accepted `0x39D` readiness frames, the mode-transition results, the counter-skip observation, the transmit-gap results, and any fault-observation window with whether mode 0 cleared it.

Send the JSON file back exactly as written. Do not edit it.

From a laptop, one way to copy it is:

```bash
scp comma:/data/openpilot/ibooster-bench-runs/<file>.json .
```

Then send that JSON file through the usual channel.

## After the session

Reboot the device to bring openpilot back:

```bash
sudo reboot
```

Then switch back to your normal branch if you changed branches for this session.
