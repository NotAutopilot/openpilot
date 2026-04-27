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

| Source (bus 0) | Dest (bus 1) | Name | Expected Rate |
|---|---|---|---|
| 0x108 | 0x109 | DI_torque1 | 100 Hz |
| 0x118 | 0x119 | DI_torque2 | 100 Hz |
| 0x0E | 0x199 | STW_ANGLHP_STAT | 100 Hz |
| 0x115 | 0x129 | ESP_115h | 50 Hz |
| 0x145 | 0x149 | ESP_145h | 50 Hz |
| 0x20A | 0x209 | GTW_odo | 50 Hz |
| 0x308 | 0x219 | STW_ACTN_RQ | 10 Hz |
| 0x405 | 0x2B9 | VIP_405HS | 5 Hz |
| 0x398 | 0x2D9 | BC_status | 1 Hz |

Synthesized (no direct source on bus 0):
- 0x169 (ESP_wheelSpeeds) — built from 0x118 data, sent at 100 Hz
- 0x1A9 (DI_espControl) — built from 0x115 data, sent at 50 Hz

Messages not present on Pre-AP (expected missing):
- 0x158 (ESP_C/Brake), 0x2A8 (GTW_carConfig — event-triggered only)

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
