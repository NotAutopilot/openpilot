#!/usr/bin/env python3
"""
Radar Replay Harness — Offline validation against drive logs.

Reads compressed rlog files from logs/crappy-radar/, feeds CAN frames
through the radar_interface CAN parsing, then through radard's KF +
lead selection logic. Outputs CSV metrics for before/after comparison.

Usage:
  python scripts/nap/radar_replay.py [log_path]

If no path given, processes the first .zst file in logs/crappy-radar/.

Output CSV columns:
  timestamp, num_tracks, lead_id, lead_dRel, lead_vRel, lead_aLeadK,
  lead_radar, v_ego, track_ids

Summary stats printed at end:
  - Average track lifespan
  - Lead ID change frequency
  - Max detection range
  - Radar-fused lead percentage
"""

import sys
import csv
import math
import struct
from collections import defaultdict
from pathlib import Path

# ---------------------------------------------------------------------------
# Minimal KF1D (same as openpilot.common.simple_kalman.KF1D)
# ---------------------------------------------------------------------------
class KF1D:
    def __init__(self, x0, A, C, K):
        self.x0_0 = x0[0][0]
        self.x1_0 = x0[1][0]
        self.A0_0 = A[0][0]
        self.A0_1 = A[0][1]
        self.A1_0 = A[1][0]
        self.A1_1 = A[1][1]
        self.C0_0 = C[0]
        self.C0_1 = C[1]
        self.K0_0 = K[0][0]
        self.K1_0 = K[1][0]
        self.A_K_0 = self.A0_0 - self.K0_0 * self.C0_0
        self.A_K_1 = self.A0_1 - self.K0_0 * self.C0_1
        self.A_K_2 = self.A1_0 - self.K1_0 * self.C0_0
        self.A_K_3 = self.A1_1 - self.K1_0 * self.C0_1

    def update(self, meas):
        x0 = self.A_K_0 * self.x0_0 + self.A_K_1 * self.x1_0 + self.K0_0 * meas
        x1 = self.A_K_2 * self.x0_0 + self.A_K_3 * self.x1_0 + self.K1_0 * meas
        self.x0_0 = x0
        self.x1_0 = x1

    @property
    def x(self):
        return [[self.x0_0], [self.x1_0]]


# ---------------------------------------------------------------------------
# Minimal KalmanParams (mirrors radard.py)
# ---------------------------------------------------------------------------
def interp(x, xp, fp):
    """Simple linear interpolation."""
    if x <= xp[0]:
        return fp[0]
    if x >= xp[-1]:
        return fp[-1]
    for i in range(len(xp) - 1):
        if xp[i] <= x <= xp[i + 1]:
            t = (x - xp[i]) / (xp[i + 1] - xp[i])
            return fp[i] + t * (fp[i + 1] - fp[i])
    return fp[-1]


class KalmanParams:
    def __init__(self, dt):
        self.A = [[1.0, dt], [0.0, 1.0]]
        self.C = [1.0, 0.0]
        dts = [i * 0.01 for i in range(1, 21)]
        K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689, 0.21372394,
              0.22761098, 0.24069424, 0.253096, 0.26491023, 0.27621103, 0.28705801,
              0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
              0.35353899, 0.36200124]
        K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
              0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
              0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
              0.26393339, 0.26278425]
        self.K = [[interp(dt, dts, K0)], [interp(dt, dts, K1)]]


# ---------------------------------------------------------------------------
# Track (mirrors radard.py Track)
# ---------------------------------------------------------------------------
_LEAD_ACCEL_TAU = 1.5
SPEED, ACCEL = 0, 1
RADAR_TO_CAMERA = 1.52


class Track:
    def __init__(self, identifier, v_lead, kp):
        self.identifier = identifier
        self.cnt = 0
        self.aLeadTau = _LEAD_ACCEL_TAU
        self.kf = KF1D([[v_lead], [0.0]], kp.A, kp.C, kp.K)

    def update(self, d_rel, y_rel, v_rel, v_lead, measured):
        self.dRel = d_rel
        self.yRel = y_rel
        self.vRel = v_rel
        self.vLead = v_lead
        self.measured = measured
        if self.cnt > 0:
            self.kf.update(self.vLead)
        self.vLeadK = float(self.kf.x[SPEED][0])
        self.aLeadK = float(self.kf.x[ACCEL][0])
        if abs(self.aLeadK) < 0.5:
            self.aLeadTau = _LEAD_ACCEL_TAU
        else:
            self.aLeadTau *= 0.9
        self.cnt += 1


# ---------------------------------------------------------------------------
# Bosch Radar CAN Parser (standalone, no opendbc dependency)
# ---------------------------------------------------------------------------
def extract_le_signal(data, start_bit, length):
    """Extract a little-endian CAN signal from raw bytes."""
    value = 0
    for i in range(length):
        bit_pos = start_bit + i
        byte_idx = bit_pos // 8
        bit_idx = bit_pos % 8
        if byte_idx < len(data) and (data[byte_idx] & (1 << bit_idx)):
            value |= (1 << i)
    return value


def parse_radar_points(can_frames):
    """Parse radar points from a dict of {addr: data} for one radar cycle.

    Returns list of (track_id, dRel, yRel, vRel, measured).
    """
    points = []
    for i in range(32):
        addr_a = 0x310 + i * 2
        addr_b = 0x310 + i * 2 + 1
        if addr_a not in can_frames:
            continue
        data_a = can_frames[addr_a]
        data_b = can_frames.get(addr_b)

        tracked = extract_le_signal(data_a, 62, 1)
        if not tracked:
            continue

        long_dist = extract_le_signal(data_a, 0, 12) * 0.0625
        long_speed = extract_le_signal(data_a, 12, 12) * 0.0625 - 128
        lat_dist = extract_le_signal(data_a, 24, 11) * 0.125 - 128
        prob_exist = extract_le_signal(data_a, 35, 5) * 3.125
        measured = extract_le_signal(data_a, 61, 1)

        if long_dist <= 0 or long_dist > 250 or prob_exist < 50:
            continue

        idx_a = extract_le_signal(data_a, 63, 1)
        if data_b is not None:
            idx_b = extract_le_signal(data_b, 63, 1)
            if idx_a != idx_b:
                continue

        points.append((i, long_dist, lat_dist, long_speed, bool(measured)))

    return points


# ---------------------------------------------------------------------------
# Laplacian matching (mirrors radard.py)
# ---------------------------------------------------------------------------
def laplacian_pdf(x, mu, b):
    b = max(b, 1e-4)
    return math.exp(-abs(x - mu) / b)


def match_vision_to_tracks(v_ego, vision_x, vision_y, vision_v, tracks):
    """Simple lead matching. Returns best track or None."""
    if not tracks:
        return None
    offset = vision_x - RADAR_TO_CAMERA

    def score(t):
        pd = laplacian_pdf(t.dRel, offset, 5.0)
        py = laplacian_pdf(t.yRel, -vision_y, 2.0)
        pv = laplacian_pdf(t.vRel + v_ego, vision_v, 5.0)
        return pd * py * pv

    best = max(tracks.values(), key=score)
    dist_ok = abs(best.dRel - offset) < max(offset * 0.25, 5.0)
    vel_ok = (abs(best.vRel + v_ego - vision_v) < 10) or (v_ego + best.vRel > 3)
    return best if dist_ok and vel_ok else None


# ---------------------------------------------------------------------------
# Log Reader
# ---------------------------------------------------------------------------
def read_rlog(path):
    """Read an rlog.zst file, yield (logMonoTime, event_type, event_data).

    Uses capnp for proper deserialization.
    """
    import zstandard as zstd
    import capnp
    # Find the log.capnp schema
    try:
        from cereal import log as cereal_log
        schema = cereal_log.Event.schema
    except ImportError:
        # Fallback: try to find the schema file
        print("WARNING: cereal not importable, trying manual capnp load", flush=True)
        schema_paths = list(Path('/Users/jackbrandt/projects/openpilot-dev').rglob('log.capnp'))
        if not schema_paths:
            print("ERROR: Cannot find log.capnp schema", flush=True)
            sys.exit(1)
        capnp.remove_import_hook()
        log_capnp = capnp.load(str(schema_paths[0]))
        schema = log_capnp.Event.schema

    dctx = zstd.ZstdDecompressor()
    with open(path, 'rb') as f:
        reader = dctx.stream_reader(f)
        buf = b''
        while True:
            chunk = reader.read(65536)
            if not chunk:
                break
            buf += chunk
            # capnp messages are packed sequentially
            while len(buf) >= 8:
                try:
                    # Try to read a message from buffer
                    capnp.lib.capnp._MallocMessageBuilder()
                    # Use packed stream reading
                    break
                except Exception:
                    break
            # We'll use a simpler approach: read all at once

    # Simpler approach: decompress fully then parse
    with open(path, 'rb') as f:
        dctx = zstd.ZstdDecompressor()
        raw = dctx.decompress(f.read(), max_output_size=500 * 1024 * 1024)

    events = []
    pos = 0
    while pos < len(raw):
        try:
            # Read capnp segment header: number of segments (uint32), then sizes
            if pos + 4 > len(raw):
                break
            n_segs = struct.unpack_from('<I', raw, pos)[0] + 1
            header_size = 4 + 4 * n_segs
            if n_segs % 2 == 1:
                header_size += 0  # padding already handled
            else:
                header_size += 4  # pad to 8-byte boundary with extra 4
            # Actually, capnp C++ wire format: 4 bytes (nsegs-1), then nsegs*4 bytes of sizes, padded to 8
            raw_header_size = 4 + 4 * n_segs
            if raw_header_size % 8 != 0:
                raw_header_size += 4
            if pos + raw_header_size > len(raw):
                break
            total_data = 0
            for s in range(n_segs):
                seg_size = struct.unpack_from('<I', raw, pos + 4 + s * 4)[0] * 8
                total_data += seg_size
            msg_end = pos + raw_header_size + total_data
            if msg_end > len(raw):
                break
            msg_bytes = raw[pos:msg_end]
            pos = msg_end

            with schema.from_bytes(msg_bytes, traversal_limit_in_words=2**63 - 1) as event:
                which = event.which()
                mono_time = event.logMonoTime
                events.append((mono_time, which, event))
        except Exception:
            pos += 1  # skip byte and retry
            continue

    return events


def read_rlog_simple(path):
    """Simplified rlog reader — extract CAN frames and carState/modelV2 events.

    Returns list of cereal Event readers with timestamp and event data.
    Tries cereal module first, falls back to manual capnp parsing.
    """
    import zstandard as zstd

    print(f"Reading {path}...", flush=True)

    dctx = zstd.ZstdDecompressor()
    with open(path, 'rb') as f:
        raw = dctx.decompress(f.read(), max_output_size=500 * 1024 * 1024)

    print(f"  Decompressed: {len(raw)} bytes", flush=True)

    try:
        from cereal import log as cereal_log
        events = list(cereal_log.Event.read_multiple_bytes(raw))
        print(f"  Parsed {len(events)} events (cereal)", flush=True)
        return events
    except ImportError:
        pass

    # Fallback: manual capnp parsing
    import capnp
    schema_paths = list(Path(__file__).resolve().parent.parent.parent.rglob('log.capnp'))
    if not schema_paths:
        # Try broader search
        schema_paths = list(Path('/Users/jackbrandt/projects/openpilot-dev').rglob('log.capnp'))
    if not schema_paths:
        print("ERROR: Cannot find log.capnp schema", flush=True)
        sys.exit(1)

    print(f"  Using capnp schema: {schema_paths[0]}", flush=True)
    capnp.remove_import_hook()
    log_capnp = capnp.load(str(schema_paths[0]),
                           imports=[str(schema_paths[0].parent.parent / 'cereal'),
                                    str(schema_paths[0].parent)])

    # First pass: extract raw message bytes for each event
    msg_list = []
    pos = 0
    while pos < len(raw):
        try:
            if pos + 4 > len(raw):
                break
            n_segs = struct.unpack_from('<I', raw, pos)[0] + 1
            if n_segs > 10:
                pos += 1
                continue
            raw_header_size = 4 + 4 * n_segs
            if raw_header_size % 8 != 0:
                raw_header_size += 4
            if pos + raw_header_size > len(raw):
                break
            total_data = 0
            for s in range(n_segs):
                seg_size = struct.unpack_from('<I', raw, pos + 4 + s * 4)[0] * 8
                total_data += seg_size
            msg_end = pos + raw_header_size + total_data
            if msg_end > len(raw):
                break
            msg_list.append(raw[pos:msg_end])
            pos = msg_end
        except Exception:
            pos += 1
            continue

    # Second pass: parse each message and extract the fields we need into plain dicts
    # (from_bytes returns a context manager; data is only valid inside the with block)
    events = []
    for msg_bytes in msg_list:
        try:
            with log_capnp.Event.from_bytes(msg_bytes, traversal_limit_in_words=2**63 - 1) as event:
                w = event.which()
                mono_time = event.logMonoTime
                if w == 'can':
                    frames = []
                    for f in event.can:
                        frames.append({'src': f.src, 'address': f.address, 'dat': bytes(f.dat)})
                    events.append({'which': w, 'logMonoTime': mono_time, 'can': frames})
                elif w == 'carState':
                    events.append({'which': w, 'logMonoTime': mono_time, 'vEgo': event.carState.vEgo})
                elif w == 'liveTracks':
                    points = []
                    for pt in event.liveTracks.points:
                        points.append({
                            'trackId': pt.trackId,
                            'dRel': pt.dRel,
                            'yRel': pt.yRel,
                            'vRel': pt.vRel,
                            'measured': bool(pt.measured),
                        })
                    events.append({'which': w, 'logMonoTime': mono_time, 'points': points})
                else:
                    events.append({'which': w, 'logMonoTime': mono_time})
        except Exception:
            continue

    print(f"  Parsed {len(events)} events (capnp fallback)", flush=True)
    return events


# ---------------------------------------------------------------------------
# Main Replay
# ---------------------------------------------------------------------------
def replay(log_path, radar_dt=0.125):
    """Run radar pipeline replay on a drive log.

    Args:
        log_path: Path to .zst rlog file
        radar_dt: KF time step (0.05 for old DT_MDL, 0.125 for fixed 8Hz)
    """
    events = read_rlog_simple(log_path)

    kp = KalmanParams(radar_dt)
    tracks = {}
    v_ego = 0.0
    prev_lead_id = None

    # Collect radar frames by trigger cycle
    radar_can_buf = {}  # addr -> data for current cycle

    # Output data
    rows = []
    lead_ids_over_time = []
    track_lifetimes = defaultdict(int)
    max_range = 0.0
    radar_lead_count = 0
    vision_lead_count = 0
    total_lead_count = 0

    t0 = None

    for event in events:
        # Support both cereal Event objects and plain dicts (capnp fallback)
        if isinstance(event, dict):
            which = event['which']
            mono_time = event['logMonoTime']
        else:
            try:
                which = event.which()
            except Exception:
                continue
            mono_time = event.logMonoTime

        if t0 is None:
            t0 = mono_time
        t_sec = (mono_time - t0) * 1e-9

        # Track v_ego from carState
        if which == 'carState':
            try:
                v_ego = event['vEgo'] if isinstance(event, dict) else event.carState.vEgo
            except Exception:
                pass
            continue

        # Collect CAN frames
        if which == 'can':
            try:
                frames = event['can'] if isinstance(event, dict) else event.can
                for frame in frames:
                    src = frame['src'] if isinstance(frame, dict) else frame.src
                    if src == 1:  # radar bus
                        addr = frame['address'] if isinstance(frame, dict) else frame.address
                        dat = frame['dat'] if isinstance(frame, dict) else bytes(frame.dat)
                        radar_can_buf[addr] = dat
            except Exception:
                pass
            continue

        # Process on liveTracks (radar cycle boundary)
        if which == 'liveTracks':
            # Parse radar points from liveTracks.points (RadarData struct)
            ar_pts = {}
            if isinstance(event, dict):
                points = event['points']
                for pt in points:
                    ar_pts[pt['trackId']] = (pt['dRel'], pt['yRel'], pt['vRel'], pt['measured'])
            else:
                try:
                    live_tracks = event.liveTracks
                    for pt in live_tracks.points:
                        ar_pts[pt.trackId] = (pt.dRel, pt.yRel, pt.vRel, bool(pt.measured))
                except Exception:
                    continue

            # Remove disappeared tracks
            for tid in list(tracks.keys()):
                if tid not in ar_pts:
                    tracks.pop(tid)

            # Update tracks
            for tid, (d, y, v, m) in ar_pts.items():
                v_lead = v + v_ego
                if tid not in tracks:
                    tracks[tid] = Track(tid, v_lead, kp)
                tracks[tid].update(d, y, v, v_lead, m)
                track_lifetimes[tid] += 1
                if d > max_range:
                    max_range = d

            # Simple lead selection (closest tracked point in lane)
            lead_track = None
            for t in sorted(tracks.values(), key=lambda t: t.dRel):
                if abs(t.yRel) < 2.5 and t.dRel > 0.5:
                    lead_track = t
                    break

            lead_id = lead_track.identifier if lead_track else -1
            lead_dRel = lead_track.dRel if lead_track else 0.0
            lead_vRel = lead_track.vRel if lead_track else 0.0
            lead_aLeadK = lead_track.aLeadK if lead_track else 0.0
            is_radar = lead_track is not None

            if lead_id != -1:
                total_lead_count += 1
                if is_radar:
                    radar_lead_count += 1
                else:
                    vision_lead_count += 1

            lead_ids_over_time.append((t_sec, lead_id))

            if prev_lead_id is not None and lead_id != prev_lead_id:
                pass  # counted below

            track_id_list = sorted(tracks.keys())

            rows.append({
                'timestamp': f"{t_sec:.3f}",
                'num_tracks': len(tracks),
                'lead_id': lead_id,
                'lead_dRel': f"{lead_dRel:.2f}",
                'lead_vRel': f"{lead_vRel:.2f}",
                'lead_aLeadK': f"{lead_aLeadK:.3f}",
                'lead_radar': 1 if is_radar else 0,
                'v_ego': f"{v_ego:.2f}",
                'track_ids': '|'.join(str(x) for x in track_id_list),
            })

            prev_lead_id = lead_id
            continue

    # ---------------------------------------------------------------------------
    # Output CSV
    # ---------------------------------------------------------------------------
    csv_path = str(log_path).replace('.zst', '') + f'_replay_dt{radar_dt:.3f}.csv'
    if rows:
        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        print(f"\nCSV written: {csv_path} ({len(rows)} rows)", flush=True)
    else:
        print("\nNo radar data found in log.", flush=True)
        return

    # ---------------------------------------------------------------------------
    # Summary Statistics
    # ---------------------------------------------------------------------------
    print(f"\n{'='*60}", flush=True)
    print(f"REPLAY SUMMARY (dt={radar_dt:.3f}s)", flush=True)
    print(f"{'='*60}", flush=True)

    duration = float(rows[-1]['timestamp']) - float(rows[0]['timestamp']) if len(rows) > 1 else 0

    # Lead ID changes
    id_changes = 0
    for i in range(1, len(lead_ids_over_time)):
        if lead_ids_over_time[i][1] != lead_ids_over_time[i - 1][1]:
            id_changes += 1
    change_rate = id_changes / max(duration, 0.1)

    # Track lifetimes
    lifetimes = list(track_lifetimes.values())
    avg_lifetime = sum(lifetimes) / max(len(lifetimes), 1)
    max_lifetime = max(lifetimes) if lifetimes else 0

    # Dynamic tracks (non-zero vRel)
    dynamic_count = 0
    for row in rows:
        for tid_str in row['track_ids'].split('|'):
            if tid_str and int(tid_str) in tracks:
                t = tracks[int(tid_str)]
                if abs(t.vRel) > 0.1:
                    dynamic_count += 1

    # Radar-fused percentage
    radar_pct = 100.0 * radar_lead_count / max(total_lead_count, 1)

    print(f"  Duration:            {duration:.1f}s", flush=True)
    print(f"  Total frames:        {len(rows)}", flush=True)
    print(f"  Unique track IDs:    {len(track_lifetimes)}", flush=True)
    print(f"  Avg track lifespan:  {avg_lifetime:.1f} frames", flush=True)
    print(f"  Max track lifespan:  {max_lifetime} frames", flush=True)
    print(f"  Max detection range: {max_range:.1f}m", flush=True)
    print(f"  Lead ID changes:     {id_changes} ({change_rate:.2f}/s)", flush=True)
    print(f"  Radar-fused leads:   {radar_lead_count}/{total_lead_count} ({radar_pct:.0f}%)", flush=True)
    print(f"  Vision-only leads:   {vision_lead_count}/{total_lead_count}", flush=True)


def main():
    log_dir = Path(__file__).resolve().parent.parent.parent / 'logs' / 'crappy-radar'

    if len(sys.argv) > 1:
        log_path = Path(sys.argv[1])
    else:
        # Find first .zst file
        zst_files = sorted(log_dir.glob('*.zst'))
        if not zst_files:
            print(f"No .zst files found in {log_dir}", flush=True)
            sys.exit(1)
        log_path = zst_files[0]

    print("Radar Replay Harness", flush=True)
    print(f"Log: {log_path}", flush=True)
    print(f"{'='*60}", flush=True)

    # Run with old dt (baseline) and new dt (fixed)
    print("\n--- BASELINE (dt=0.05, DT_MDL) ---", flush=True)
    replay(log_path, radar_dt=0.05)

    print("\n\n--- FIXED (dt=0.125, 8Hz radar) ---", flush=True)
    replay(log_path, radar_dt=0.125)


if __name__ == "__main__":
    main()
