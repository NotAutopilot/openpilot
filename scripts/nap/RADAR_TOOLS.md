# NAP Radar Diagnostic Tools

Tools for diagnosing and validating the Pre-AP Tesla Bosch radar pipeline.

## diagnose_radar.py

Real-time diagnostic tool with two modes. Run on-device while openpilot is active.

### Cereal Monitor (default)

```bash
python3 scripts/nap/diagnose_radar.py
```

Monitors both CAN buses via cereal messaging (non-invasive, GTW emulation stays active):

- **Bus 0 source rates** — Measures every chassis bus message that triggers GTW forwarding. Since CAN TX doesn't loopback to RX, we can't see our own GTW messages on bus 1. Instead, bus 0 source rates directly equal the GTW forwarding rates.
- **Timing gap detection** — Tracks max inter-message gap and counts gaps >2.5x the expected period. Detects burstiness that could cause the radar to drop Doppler tracking.
- **Rate trend** — Per-second rate history (last 10s) for the 3 critical sources: 0x118 (wheel speed, 100Hz), 0x0E (steering, 100Hz), 0x115 (ESP control, 50Hz).
- **Vehicle speed decode** — Live speed from DI_torque2 (0x118), the source the firmware uses to synthesize 0x169 wheel speeds.
- **Radar status transitions** — Catches ACTIVE (0x300) to INIT (0x631) flip-flops.
- **Doppler tracking** — Per-interval percentage of radar tracks with non-zero vRel.

Press Ctrl+C to stop. A final summary with timing statistics is printed.

### Panda Health Check

```bash
python3 scripts/nap/diagnose_radar.py --panda
```

Quick one-shot check (does NOT change safety mode):

- Reads NAPRadarEnabled param
- Verifies safety_mode=36 (teslaLegacy) and all param flags (PREAP, RADAR_EMULATION, etc.)
- Shows CAN bus health (rx/tx/error counts per bus)
- 3-second bus 1 sniff to check for radar traffic

### GTW Source-to-Destination Map

Source of truth: `tesla_preap_gtw_emulation()` in `opendbc/safety/modes/tesla_preap.h`.

| Source (bus 0) | Dest (bus 1) | Name | Expected Rate |
|---|---|---|---|
| 0x108 | 0x109 | DI_torque1 | 100 Hz |
| 0x118 | 0x119 | DI_torque2 | 100 Hz |
| 0x0E | 0x199 | STW_ANGLHP_STAT | 100 Hz |
| 0x115 | 0x129 | ESP_115h | 50 Hz |
| 0x145 | 0x149 | ESP_145h | 50 Hz |
| 0x20A | 0x159 | BrakeMessage → ESP_C | 50 Hz |
| 0x308 | 0x209 | GTW_odo | 50 Hz |
| 0x45 | 0x219 | STW_ACTN_RQ | 10 Hz |
| 0x398 | 0x2A9 | GTW_carConfig | 1 Hz |
| 0x405 | 0x2B9 | VIN_VIP_405HS | 5 Hz |
| 0x30A | 0x2D9 | BC_status | 10 Hz |

Synthesized (no direct source on bus 0):
- 0x169 (ESP_wheelSpeeds) — built from 0x118 data, sent at 100 Hz
- 0x1A9 (DI_espControl) — built from 0x115 data, sent at 50 Hz

0x398 (GTW_carConfig) is not a straight re-address: the firmware patches the
country, radar type, radar position and EPAS type bitfields on the way through.
Those are the fields the radar checks against what it was programmed with, so a
missing or wrong 0x2A9 looks exactly like a VIN mismatch — tracks for five
seconds, then frozen.

---

## vin_learn_radar.py

Teaches a used Bosch radar the VIN of the car it is now installed in. Run from
**Settings → NAP → Radar → Radar VIN Learn**, or on-device directly.

A radar pulled from another Tesla keeps that car's VIN in its own memory. It
sends tracks for ~5 seconds after power-up regardless, then stops updating them
once it sees a VIN, radar position or EPAS type on the bus that doesn't match
what it was programmed with.

```bash
python3 scripts/nap/vin_learn_radar.py
```

There is no VIN to type in — the car's VIN is read off 0x405 on bus 0 and the
radar learns it from the live bus. What the tool does:

1. reads this car's VIN from the chassis bus
2. reads the VIN stored in the radar (DID 0xF190); if it already matches, stops
   without writing anything
3. extended diagnostic session + Tesla SecurityAccess level 1
4. `routineControl` start / stop / requestResults on routine 0x0A03
5. reads the stored VIN back to confirm it changed

Requires the car awake and in PARK with the brake held — key fob inside, press
the brake pedal to bring the car up (a Pre-AP Model S has no start button; the
brake is what wakes it and holding it is what keeps it awake). The car may chime
during the routine. Reboot the device afterwards.

The panda is put in `teslaPreap` safety mode with `PREAP_FLAG_RADAR_VIN_LEARN`
(8) — GTW emulation has to keep feeding the radar this car's VIN and position
while it learns, and the flag is what opens 0x641 on the radar bus in the TX
whitelist. If the panda rejects the sends, it is running firmware from before
that flag existed: reboot so openpilot reflashes it.

State machine and protocol handling live in
`opendbc/car/tesla/preap/radar_vin.py` (unit tested, no hardware needed).

---

## radar_replay.py

Offline replay harness for validating radar pipeline changes against drive logs.

### Usage

```bash
# Replay a specific log file
python3 scripts/nap/radar_replay.py path/to/log.zst

# Replay first .zst in logs/crappy-radar/
python3 scripts/nap/radar_replay.py
```

Runs the radar pipeline twice on each log:
1. **Baseline** (dt=0.05, old DT_MDL) — measures behavior with the original KF time step
2. **Fixed** (dt=0.125, 8Hz radar) — measures behavior with the corrected time step

### What It Measures

- **Lead ID changes/s** — How often the selected lead vehicle changes. Target: <1.0/s
- **Unique track IDs** — Total distinct radar tracks seen during the drive
- **Average track lifespan** — Mean frames a track survives before disappearing
- **Max detection range** — Farthest object detected
- **Radar-fused vs vision-only** — Percentage of lead selections backed by radar data

### Output

- CSV file per run (one row per radar cycle) with columns: timestamp, num_tracks, lead_id, lead_dRel, lead_vRel, lead_aLeadK, lead_radar, v_ego, track_ids
- Summary statistics printed to console

### Important Note

The replay uses a simplified lead selection (closest in-lane track) without the vision model. Lead ID change rates from replay will be higher than the live system, which uses vision-radar fusion with Laplacian matching and hysteresis. To measure actual live lead stability, analyze the `radarState` events in the log (leadOne.radarTrackId).

### Dependencies

- `zstandard` — for .zst decompression
- `pycapnp` — for capnp message parsing (fallback when cereal is unavailable)
- Works both on-device (with cereal) and locally (with pycapnp + log.capnp schema)

### Probing a radar that won't answer

```bash
python3 scripts/nap/vin_learn_radar.py --probe
```

Read-only — it sends nothing but TesterPresent and writes nothing. Use it when
the learn reports that requests went out and nothing came back. It separates
three failures that otherwise look identical:

1. **Nothing on bus 1** — either the radar is unpowered (on the common Pre-AP
   install radar power is tapped off the EPAS fuse, so the car must be awake) or
   bus 1 RX isn't reaching the tool, which would be a tool bug.
2. **Track frames but no diagnostic reply** — the radar is powered and running
   but ignores diagnostics. Not a wiring problem.
3. **A reply at an unexpected address** — the learn is targeting the wrong one.

It probes 0x641 (what the DBC and Tinkla's tooling use) and 0x671, and reports
any frame that appears on the radar bus after the request which wasn't part of
the steady-state background — so a response at a third address still shows up.
