#!/usr/bin/env python3
"""
Vision vs Radar Lead Distance Analysis

Reads drive logs and extracts per-frame:
  - Vision model lead distance (modelV2.leadsV3[0].x[0])
  - Fused radar lead distance (radarState.leadOne.dRel)
  - Whether lead is radar-matched or vision-only
  - The delta between vision and radar distances
  - Vision lead standard deviation (xStd) and probability

Outputs CSV and prints summary statistics showing the disagreement pattern.

Usage:
  python scripts/nap/vision_radar_delta.py [log_dir_or_file]

Default: processes all .zst files in logs/crappy-radar/route-5/
"""

import sys
import csv
import struct
from pathlib import Path

RADAR_TO_CAMERA = 1.52  # Same as radard.py

def read_events(path):
    """Read a .zst cereal log file and return list of parsed events."""
    import zstandard as zstd

    print(f"  Reading {Path(path).name}...", end="", flush=True)

    dctx = zstd.ZstdDecompressor()
    with open(path, 'rb') as f:
        raw = dctx.decompress(f.read(), max_output_size=500 * 1024 * 1024)

    # Try cereal module first (faster)
    try:
        from cereal import log as cereal_log
        events = list(cereal_log.Event.read_multiple_bytes(raw))
        print(f" {len(events)} events (cereal)", flush=True)
        return events
    except ImportError:
        pass

    # Fallback: manual capnp parsing
    import capnp
    schema_paths = list(Path(__file__).resolve().parent.parent.parent.rglob('log.capnp'))
    if not schema_paths:
        print(" ERROR: Cannot find log.capnp schema", flush=True)
        sys.exit(1)
    capnp.remove_import_hook()
    log_capnp = capnp.load(str(schema_paths[0]),
                           imports=[str(schema_paths[0].parent.parent / 'cereal'),
                                    str(schema_paths[0].parent)])

    # Parse raw capnp messages
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

    events = []
    for msg_bytes in msg_list:
        try:
            with log_capnp.Event.from_bytes(msg_bytes, traversal_limit_in_words=2**63 - 1) as event:
                events.append(event)
        except Exception:
            continue
    print(f" {len(events)} events (capnp)", flush=True)
    return events


def extract_frames(events):
    """Extract modelV2 and radarState data, paired by timestamp."""
    # Collect all modelV2 and radarState events
    model_frames = {}    # logMonoTime -> model data
    radar_frames = []    # list of (logMonoTime, radar data, mdMonoTime)
    car_state = {}       # logMonoTime -> vEgo

    for event in events:
        try:
            which = event.which()
        except Exception:
            continue
        mono = event.logMonoTime

        if which == 'modelV2':
            mv = event.modelV2
            leads = mv.leadsV3
            if len(leads) > 0:
                lead0 = leads[0]
                x_vals = list(lead0.x)
                xstd_vals = list(lead0.xStd)
                v_vals = list(lead0.v)
                model_frames[mono] = {
                    'x0': x_vals[0] if x_vals else 0.0,
                    'xStd0': xstd_vals[0] if xstd_vals else 0.0,
                    'prob': float(lead0.prob),
                    'v0': v_vals[0] if v_vals else 0.0,
                }

        elif which == 'radarState':
            rs = event.radarState
            l1 = rs.leadOne
            radar_frames.append((mono, {
                'dRel': float(l1.dRel),
                'yRel': float(l1.yRel),
                'vRel': float(l1.vRel),
                'vLead': float(l1.vLead),
                'vLeadK': float(l1.vLeadK),
                'aLeadK': float(l1.aLeadK),
                'status': bool(l1.status),
                'radar': bool(l1.radar),
                'radarTrackId': int(l1.radarTrackId),
                'modelProb': float(l1.modelProb),
            }, int(rs.mdMonoTime)))

        elif which == 'carState':
            car_state[mono] = float(event.carState.vEgo)

    return model_frames, radar_frames, car_state


def find_nearest_model(md_mono_time, model_frames):
    """Find the model frame closest to the given mdMonoTime."""
    if md_mono_time in model_frames:
        return model_frames[md_mono_time]
    # Find nearest
    best_t = None
    best_dt = float('inf')
    for t in model_frames:
        dt = abs(t - md_mono_time)
        if dt < best_dt:
            best_dt = dt
            best_t = t
    if best_t is not None and best_dt < 1e9:  # within 1 second
        return model_frames[best_t]
    return None


def find_nearest_vego(mono_time, car_state):
    """Find vEgo closest to the given timestamp."""
    best_t = None
    best_dt = float('inf')
    for t in car_state:
        dt = abs(t - mono_time)
        if dt < best_dt:
            best_dt = dt
            best_t = t
    if best_t is not None:
        return car_state[best_t]
    return 0.0


def analyze(log_paths):
    """Run the full analysis on a set of log files."""
    all_model_frames = {}
    all_radar_frames = []
    all_car_state = {}

    for path in sorted(log_paths):
        events = read_events(path)
        mf, rf, cs = extract_frames(events)
        all_model_frames.update(mf)
        all_radar_frames.extend(rf)
        all_car_state.update(cs)

    # Sort radar frames by time
    all_radar_frames.sort(key=lambda x: x[0])

    if not all_radar_frames:
        print("No radarState events found in logs.")
        return

    # Pair each radarState with its corresponding modelV2
    rows = []
    t0 = all_radar_frames[0][0]

    for mono_time, radar, md_mono in all_radar_frames:
        t_sec = (mono_time - t0) * 1e-9
        model = find_nearest_model(md_mono, all_model_frames)
        v_ego = find_nearest_vego(mono_time, all_car_state)

        if model is None:
            continue

        vision_dist_cam = model['x0']              # distance from camera
        vision_dist_radar = vision_dist_cam - RADAR_TO_CAMERA  # distance from radar
        radar_dist = radar['dRel']
        is_radar = radar['radar']
        has_status = radar['status']

        if has_status:
            delta = vision_dist_radar - radar_dist
        else:
            delta = 0.0

        # Compute what dist_sane would be (25% original, 35% updated)
        if vision_dist_radar > 0:
            threshold_25 = max(vision_dist_radar * 0.25, 5.0)
            threshold_35 = max(vision_dist_radar * 0.35, 8.0)
            dist_sane_25 = abs(delta) < threshold_25 if is_radar or has_status else True
            dist_sane_35 = abs(delta) < threshold_35 if is_radar or has_status else True
        else:
            threshold_25 = 5.0
            threshold_35 = 8.0
            dist_sane_25 = True
            dist_sane_35 = True

        rows.append({
            'time_s': f"{t_sec:.3f}",
            'v_ego_mps': f"{v_ego:.2f}",
            'vision_dist_cam': f"{vision_dist_cam:.2f}",
            'vision_dist_radar_frame': f"{vision_dist_radar:.2f}",
            'radar_dRel': f"{radar_dist:.2f}",
            'delta_m': f"{delta:.2f}",
            'abs_delta_m': f"{abs(delta):.2f}",
            'is_radar_matched': 1 if is_radar else 0,
            'lead_status': 1 if has_status else 0,
            'radar_track_id': radar['radarTrackId'],
            'vision_prob': f"{model['prob']:.3f}",
            'vision_xStd': f"{model['xStd0']:.2f}",
            'threshold_25pct': f"{threshold_25:.2f}",
            'threshold_35pct': f"{threshold_35:.2f}",
            'dist_sane_25': 1 if dist_sane_25 else 0,
            'dist_sane_35': 1 if dist_sane_35 else 0,
        })

    # Write CSV
    csv_path = str(log_paths[0].parent / 'vision_radar_delta.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nCSV written: {csv_path} ({len(rows)} rows)")

    # -----------------------------------------------------------------------
    # Summary statistics
    # -----------------------------------------------------------------------
    print(f"\n{'='*70}")
    print("VISION vs RADAR LEAD DISTANCE ANALYSIS")
    print(f"{'='*70}")

    # Filter to frames with active lead
    active = [r for r in rows if int(r['lead_status']) == 1]
    radar_matched = [r for r in active if int(r['is_radar_matched']) == 1]
    vision_only = [r for r in active if int(r['is_radar_matched']) == 0]

    print(f"\n  Total frames:           {len(rows)}")
    print(f"  Active lead frames:     {len(active)}")
    print(f"  Radar-matched:          {len(radar_matched)} ({100*len(radar_matched)/max(len(active),1):.1f}%)")
    print(f"  Vision-only:            {len(vision_only)} ({100*len(vision_only)/max(len(active),1):.1f}%)")

    if active:
        deltas = [float(r['delta_m']) for r in active]
        abs_deltas = [abs(d) for d in deltas]
        import statistics
        print("\n  Delta (vision - radar) statistics:")
        print(f"    Mean:                 {statistics.mean(deltas):+.2f} m")
        print(f"    Median:               {statistics.median(deltas):+.2f} m")
        print(f"    Std dev:              {statistics.stdev(deltas):.2f} m")
        print(f"    Min:                  {min(deltas):+.2f} m")
        print(f"    Max:                  {max(deltas):+.2f} m")
        print(f"    Abs mean:             {statistics.mean(abs_deltas):.2f} m")

        # Percentiles
        sorted_abs = sorted(abs_deltas)
        p50 = sorted_abs[len(sorted_abs)//2]
        p90 = sorted_abs[int(len(sorted_abs)*0.9)]
        p95 = sorted_abs[int(len(sorted_abs)*0.95)]
        p99 = sorted_abs[int(len(sorted_abs)*0.99)]
        print(f"    P50 abs delta:        {p50:.2f} m")
        print(f"    P90 abs delta:        {p90:.2f} m")
        print(f"    P95 abs delta:        {p95:.2f} m")
        print(f"    P99 abs delta:        {p99:.2f} m")

    # dist_sane failure analysis
    if active:
        fail_25 = [r for r in active if int(r['dist_sane_25']) == 0]
        fail_35 = [r for r in active if int(r['dist_sane_35']) == 0]
        print("\n  dist_sane failure rate:")
        print(f"    At 25% threshold:     {len(fail_25)}/{len(active)} ({100*len(fail_25)/len(active):.1f}%)")
        print(f"    At 35% threshold:     {len(fail_35)}/{len(active)} ({100*len(fail_35)/len(active):.1f}%)")

    # Vision model noise analysis
    if active:
        xstds = [float(r['vision_xStd']) for r in active]
        print("\n  Vision model xStd (positional uncertainty):")
        print(f"    Mean:                 {statistics.mean(xstds):.2f} m")
        print(f"    Max:                  {max(xstds):.2f} m")

    # Radar-matched vs vision-only lead distance comparison
    if radar_matched and vision_only:
        rm_dists = [float(r['radar_dRel']) for r in radar_matched]
        vo_dists = [float(r['radar_dRel']) for r in vision_only]
        print("\n  Lead distance by source:")
        print(f"    Radar-matched mean:   {statistics.mean(rm_dists):.1f} m")
        print(f"    Vision-only mean:     {statistics.mean(vo_dists):.1f} m")

    # Transition analysis: how often does it flip between radar and vision?
    if len(active) > 1:
        flips = 0
        for i in range(1, len(active)):
            if int(active[i]['is_radar_matched']) != int(active[i-1]['is_radar_matched']):
                flips += 1
        duration = float(active[-1]['time_s']) - float(active[0]['time_s'])
        flip_rate = flips / max(duration, 0.1)
        print("\n  Radar/vision source flips:")
        print(f"    Total:                {flips}")
        print(f"    Rate:                 {flip_rate:.2f}/s")

    # Delta by distance range (is the disagreement worse at range?)
    if active:
        ranges = [(0, 30, "0-30m"), (30, 60, "30-60m"), (60, 100, "60-100m"), (100, 300, "100m+")]
        print("\n  Delta by distance range:")
        print(f"    {'Range':<12} {'Count':>6} {'Mean delta':>12} {'Abs mean':>10} {'Std':>8}")
        for lo, hi, label in ranges:
            in_range = [r for r in active if lo <= float(r['vision_dist_radar_frame']) < hi]
            if in_range:
                d = [float(r['delta_m']) for r in in_range]
                ad = [abs(x) for x in d]
                std = statistics.stdev(d) if len(d) > 1 else 0
                print(f"    {label:<12} {len(in_range):>6} {statistics.mean(d):>+12.2f} {statistics.mean(ad):>10.2f} {std:>8.2f}")

    # Check for consistent offset (sign of delta)
    if active:
        pos = sum(1 for r in active if float(r['delta_m']) > 0)
        neg = sum(1 for r in active if float(r['delta_m']) < 0)
        zero = sum(1 for r in active if float(r['delta_m']) == 0)
        print("\n  Delta sign distribution:")
        print(f"    Vision > Radar:       {pos} ({100*pos/len(active):.1f}%)")
        print(f"    Vision < Radar:       {neg} ({100*neg/len(active):.1f}%)")
        print(f"    Equal:                {zero}")

    # Vision-only frames: what does the delta look like there?
    # (delta is vision_dist - radarState.dRel; when vision-only, dRel IS the vision dist)
    # So delta should be ~0 for vision-only frames. If not, there's a timing mismatch.

    # Frame-to-frame vision lead jitter
    if len(active) > 1:
        vdists = [float(r['vision_dist_cam']) for r in active]
        jitter = [abs(vdists[i] - vdists[i-1]) for i in range(1, len(vdists))]
        print("\n  Vision lead frame-to-frame jitter:")
        print(f"    Mean:                 {statistics.mean(jitter):.2f} m")
        print(f"    Max:                  {max(jitter):.2f} m")
        print(f"    P90:                  {sorted(jitter)[int(len(jitter)*0.9)]:.2f} m")
        print(f"    P99:                  {sorted(jitter)[int(len(jitter)*0.99)]:.2f} m")
        big_jitter = sum(1 for j in jitter if j > 5.0)
        print(f"    Frames with >5m jump: {big_jitter} ({100*big_jitter/len(jitter):.1f}%)")


def main():
    if len(sys.argv) > 1:
        p = Path(sys.argv[1])
        if p.is_dir():
            log_paths = sorted(p.glob('*.zst'))
        else:
            log_paths = [p]
    else:
        log_dir = Path(__file__).resolve().parent.parent.parent / 'logs' / 'crappy-radar' / 'route-5'
        log_paths = sorted(log_dir.glob('*.zst'))

    if not log_paths:
        print("No .zst log files found.")
        sys.exit(1)

    print("Vision vs Radar Lead Distance Analysis")
    print(f"Processing {len(log_paths)} log files")
    print(f"{'='*70}")

    analyze(log_paths)


if __name__ == '__main__':
    main()
