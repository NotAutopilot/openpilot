#!/usr/bin/env python3
"""
Deep Vision vs Radar Analysis — compares raw liveTracks radar data
against modelV2 vision leads to find WHY matching fails.

For each radarState frame, also looks at the liveTracks (raw radar points)
to see if a suitable radar track EXISTS but wasn't matched.

Outputs:
  1. Per-frame CSV with vision lead, best radar track, and match diagnostics
  2. Summary of WHY 89.7% of frames are vision-only

Usage:
  python scripts/nap/vision_radar_deep.py [log_dir]
"""

import math
import sys
import csv
import struct
from pathlib import Path
from collections import defaultdict
import statistics

RADAR_TO_CAMERA = 1.52


def read_events(path):
    """Read a .zst cereal log file."""
    import zstandard as zstd
    print(f"  Reading {Path(path).name}...", end="", flush=True)

    dctx = zstd.ZstdDecompressor()
    with open(path, 'rb') as f:
        raw = dctx.decompress(f.read(), max_output_size=500 * 1024 * 1024)

    try:
        from cereal import log as cereal_log
        events = list(cereal_log.Event.read_multiple_bytes(raw))
        print(f" {len(events)} events (cereal)", flush=True)
        return events
    except ImportError:
        pass

    import capnp
    schema_paths = list(Path(__file__).resolve().parent.parent.parent.rglob('log.capnp'))
    if not schema_paths:
        schema_paths = list(Path('/Users/jackbrandt/projects/openpilot-dev').rglob('log.capnp'))
    if not schema_paths:
        print(" ERROR: Cannot find log.capnp schema", flush=True)
        sys.exit(1)
    capnp.remove_import_hook()
    log_capnp = capnp.load(str(schema_paths[0]),
                           imports=[str(schema_paths[0].parent.parent / 'cereal'),
                                    str(schema_paths[0].parent)])

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
                w = event.which()
                mono = event.logMonoTime
                if w == 'modelV2':
                    leads = event.modelV2.leadsV3
                    if len(leads) > 0:
                        lead0 = leads[0]
                        events.append({
                            'which': 'modelV2', 'logMonoTime': mono,
                            'x0': list(lead0.x)[0] if len(lead0.x) > 0 else 0,
                            'xStd0': list(lead0.xStd)[0] if len(lead0.xStd) > 0 else 0,
                            'y0': list(lead0.y)[0] if len(lead0.y) > 0 else 0,
                            'yStd0': list(lead0.yStd)[0] if len(lead0.yStd) > 0 else 0,
                            'v0': list(lead0.v)[0] if len(lead0.v) > 0 else 0,
                            'vStd0': list(lead0.vStd)[0] if len(lead0.vStd) > 0 else 0,
                            'prob': float(lead0.prob),
                        })
                elif w == 'radarState':
                    rs = event.radarState
                    l1 = rs.leadOne
                    events.append({
                        'which': 'radarState', 'logMonoTime': mono,
                        'mdMonoTime': int(rs.mdMonoTime),
                        'dRel': float(l1.dRel), 'radar': bool(l1.radar),
                        'status': bool(l1.status), 'radarTrackId': int(l1.radarTrackId),
                    })
                elif w == 'liveTracks':
                    lt = event.liveTracks
                    points = []
                    for pt in lt.points:
                        points.append({
                            'trackId': int(pt.trackId), 'dRel': float(pt.dRel),
                            'yRel': float(pt.yRel), 'vRel': float(pt.vRel),
                            'measured': bool(pt.measured),
                        })
                    events.append({
                        'which': 'liveTracks', 'logMonoTime': mono,
                        'points': points,
                    })
                elif w == 'carState':
                    events.append({
                        'which': 'carState', 'logMonoTime': mono,
                        'vEgo': float(event.carState.vEgo),
                    })
                else:
                    pass
        except Exception:
            continue
    print(f" {len(events)} events (capnp)", flush=True)
    return events


def laplacian_pdf(x, mu, b):
    b = max(b, 1e-4)
    return math.exp(-abs(x - mu) / b)


def analyze(log_paths):
    """Run deep analysis."""
    # Collect all events
    all_events = []
    for path in sorted(log_paths):
        all_events.extend(read_events(path))

    # Sort by timestamp
    all_events.sort(key=lambda e: e['logMonoTime'] if isinstance(e, dict) else e.logMonoTime)

    # Index events by type for pairing
    model_by_time = {}
    live_tracks_latest = []  # most recent liveTracks points
    v_ego = 0.0
    t0 = None

    rows = []

    # Reason counters
    reasons = defaultdict(int)

    for evt in all_events:
        if isinstance(evt, dict):
            which = evt['which']
            mono = evt['logMonoTime']
        else:
            which = evt.which()
            mono = evt.logMonoTime

        if t0 is None:
            t0 = mono

        if which == 'carState':
            v_ego = evt['vEgo'] if isinstance(evt, dict) else evt.carState.vEgo

        elif which == 'modelV2':
            model_by_time[mono] = evt

        elif which == 'liveTracks':
            pts = evt['points'] if isinstance(evt, dict) else []
            live_tracks_latest = pts

        elif which == 'radarState':
            rs = evt if isinstance(evt, dict) else None
            if rs is None:
                continue

            t_sec = (mono - t0) * 1e-9
            md_mono = rs['mdMonoTime']

            # Find corresponding model frame
            model = model_by_time.get(md_mono)
            if model is None:
                # Find nearest
                best_t = None
                best_dt = float('inf')
                for t in model_by_time:
                    dt = abs(t - md_mono)
                    if dt < best_dt:
                        best_dt = dt
                        best_t = t
                if best_t is not None and best_dt < 1e9:
                    model = model_by_time[best_t]

            if model is None:
                continue

            vision_x = model['x0']
            vision_y = model['y0']
            vision_v = model['v0']
            vision_prob = model['prob']
            vision_xStd = model['xStd0']
            vision_yStd = model['yStd0']
            vision_vStd = model['vStd0']
            offset_vision_dist = vision_x - RADAR_TO_CAMERA

            is_radar = rs['radar']
            has_status = rs['status']
            radar_dRel = rs['dRel']
            rs['radarTrackId']

            # Analyze what liveTracks has
            n_tracks = len(live_tracks_latest)

            # Find the best-matching track manually (replicate radard logic)
            best_track = None
            best_score = -1
            best_d_delta = None
            best_y_delta = None
            best_v_delta = None

            for pt in live_tracks_latest:
                pd = laplacian_pdf(pt['dRel'], offset_vision_dist,
                                   max(vision_xStd, 1e-4))
                py = laplacian_pdf(pt['yRel'], -vision_y,
                                   max(vision_yStd, 1e-4))
                pv = laplacian_pdf(pt['vRel'] + v_ego, vision_v,
                                   max(vision_vStd, 1e-4))
                score = pd * py * pv
                if score > best_score:
                    best_score = score
                    best_track = pt
                    best_d_delta = pt['dRel'] - offset_vision_dist
                    best_y_delta = pt['yRel'] - (-vision_y)
                    best_v_delta = (pt['vRel'] + v_ego) - vision_v

            # Check dist_sane and vel_sane on best track
            if best_track is not None:
                dist_sane = abs(best_track['dRel'] - offset_vision_dist) < max(offset_vision_dist * 0.25, 5.0)
                dist_sane_35 = abs(best_track['dRel'] - offset_vision_dist) < max(offset_vision_dist * 0.35, 8.0)
                vel_sane = (abs(best_track['vRel'] + v_ego - vision_v) < 10) or (v_ego + best_track['vRel'] > 3)
            else:
                dist_sane = False
                dist_sane_35 = False
                vel_sane = False

            # Determine WHY matching failed (if it did)
            reason = "matched"
            if not is_radar and has_status:
                if n_tracks == 0:
                    reason = "no_radar_tracks"
                elif not (vision_prob > 0.5):
                    reason = "low_vision_prob"
                elif best_track is not None and not dist_sane:
                    reason = "dist_sane_fail"
                elif best_track is not None and not vel_sane:
                    reason = "vel_sane_fail"
                else:
                    reason = "unknown"
            elif not has_status:
                reason = "no_lead"

            reasons[reason] += 1

            row = {
                'time_s': f"{t_sec:.3f}",
                'v_ego': f"{v_ego:.2f}",
                'vision_x_cam': f"{vision_x:.2f}",
                'vision_x_radar': f"{offset_vision_dist:.2f}",
                'vision_prob': f"{vision_prob:.3f}",
                'vision_xStd': f"{vision_xStd:.2f}",
                'n_radar_tracks': n_tracks,
                'is_radar_matched': 1 if is_radar else 0,
                'lead_status': 1 if has_status else 0,
                'fused_dRel': f"{radar_dRel:.2f}",
                'best_track_id': best_track['trackId'] if best_track else -1,
                'best_track_dRel': f"{best_track['dRel']:.2f}" if best_track else "",
                'best_track_yRel': f"{best_track['yRel']:.2f}" if best_track else "",
                'best_track_vRel': f"{best_track['vRel']:.2f}" if best_track else "",
                'd_delta': f"{best_d_delta:.2f}" if best_d_delta is not None else "",
                'y_delta': f"{best_y_delta:.2f}" if best_y_delta is not None else "",
                'v_delta': f"{best_v_delta:.2f}" if best_v_delta is not None else "",
                'dist_sane_25': 1 if dist_sane else 0,
                'dist_sane_35': 1 if dist_sane_35 else 0,
                'vel_sane': 1 if vel_sane else 0,
                'match_score': f"{best_score:.6f}" if best_score > 0 else "",
                'reason': reason,
            }
            rows.append(row)

    # Write CSV
    csv_path = str(log_paths[0].parent / 'vision_radar_deep.csv')
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nCSV: {csv_path} ({len(rows)} rows)")

    # -----------------------------------------------------------------------
    # Summary
    # -----------------------------------------------------------------------
    print(f"\n{'='*70}")
    print("DEEP VISION vs RADAR ANALYSIS")
    print(f"{'='*70}")

    total = len(rows)
    print(f"\n  Total radarState frames: {total}")
    print("\n  Match failure reasons:")
    for reason, count in sorted(reasons.items(), key=lambda x: -x[1]):
        print(f"    {reason:<25} {count:>6} ({100*count/total:.1f}%)")

    # Focus on "no_radar_tracks" — when vision sees a lead but radar has zero tracks
    no_tracks_frames = [r for r in rows if r['reason'] == 'no_radar_tracks']
    if no_tracks_frames:
        dists = [float(r['vision_x_cam']) for r in no_tracks_frames]
        print("\n  'no_radar_tracks' analysis (vision sees lead, radar has nothing):")
        print(f"    Count:                {len(no_tracks_frames)}")
        print(f"    Vision dist mean:     {statistics.mean(dists):.1f} m")
        print(f"    Vision dist median:   {statistics.median(dists):.1f} m")
        print(f"    Vision dist range:    {min(dists):.1f} - {max(dists):.1f} m")

    # Focus on "dist_sane_fail" — radar tracks exist but distance doesn't match
    dist_fail = [r for r in rows if r['reason'] == 'dist_sane_fail']
    if dist_fail:
        d_deltas = [float(r['d_delta']) for r in dist_fail if r['d_delta']]
        abs_d = [abs(d) for d in d_deltas]
        y_deltas = [float(r['y_delta']) for r in dist_fail if r['y_delta']]
        print("\n  'dist_sane_fail' analysis (radar tracks exist, distance mismatch):")
        print(f"    Count:                {len(dist_fail)}")
        if d_deltas:
            print(f"    d_delta mean:         {statistics.mean(d_deltas):+.2f} m")
            print(f"    d_delta median:       {statistics.median(d_deltas):+.2f} m")
            print(f"    d_delta std:          {statistics.stdev(d_deltas):.2f} m")
            print(f"    abs d_delta mean:     {statistics.mean(abs_d):.2f} m")
            print(f"    abs d_delta P50:      {sorted(abs_d)[len(abs_d)//2]:.2f} m")
            print(f"    abs d_delta P90:      {sorted(abs_d)[int(len(abs_d)*0.9)]:.2f} m")
        vision_dists = [float(r['vision_x_radar']) for r in dist_fail]
        print(f"    Vision dist mean:     {statistics.mean(vision_dists):.1f} m")
        if y_deltas:
            print(f"    y_delta mean:         {statistics.mean(y_deltas):+.2f} m")

    # When radar IS matched, what do the deltas look like?
    matched = [r for r in rows if int(r['is_radar_matched']) == 1]
    if matched:
        md = [float(r['d_delta']) for r in matched if r['d_delta']]
        if md:
            print("\n  Radar-matched frames delta analysis:")
            print(f"    Count:                {len(matched)}")
            print(f"    d_delta mean:         {statistics.mean(md):+.2f} m")
            print(f"    d_delta std:          {statistics.stdev(md):.2f} m")
            print(f"    abs d_delta mean:     {statistics.mean([abs(d) for d in md]):.2f} m")

    # Frames with vision lead but NO radar tracks at all — by distance bins
    if no_tracks_frames:
        print("\n  'no_radar_tracks' by distance:")
        ranges = [(0, 20, "0-20m"), (20, 40, "20-40m"), (40, 60, "40-60m"),
                  (60, 100, "60-100m"), (100, 300, "100m+")]
        for lo, hi, label in ranges:
            in_range = [r for r in no_tracks_frames if lo <= float(r['vision_x_cam']) < hi]
            pct_of_total = 100 * len(in_range) / len(no_tracks_frames) if no_tracks_frames else 0
            print(f"    {label:<12} {len(in_range):>6} ({pct_of_total:.1f}%)")

    # Radar track availability: for ALL active-lead frames, how many have tracks?
    active = [r for r in rows if int(r['lead_status']) == 1]
    has_tracks = [r for r in active if int(r['n_radar_tracks']) > 0]
    print("\n  Radar track availability (active lead frames):")
    print(f"    With radar tracks:    {len(has_tracks)}/{len(active)} ({100*len(has_tracks)/max(len(active),1):.1f}%)")
    print(f"    Without radar tracks: {len(active)-len(has_tracks)}/{len(active)} ({100*(len(active)-len(has_tracks))/max(len(active),1):.1f}%)")

    # When tracks exist but aren't matched — is it dist_sane or vel_sane?
    has_tracks_not_matched = [r for r in active if int(r['n_radar_tracks']) > 0 and int(r['is_radar_matched']) == 0]
    if has_tracks_not_matched:
        ds_fail = sum(1 for r in has_tracks_not_matched if int(r['dist_sane_25']) == 0)
        vs_fail = sum(1 for r in has_tracks_not_matched if int(r['vel_sane']) == 0)
        both_pass = sum(1 for r in has_tracks_not_matched if int(r['dist_sane_25']) == 1 and int(r['vel_sane']) == 1)
        print(f"\n  Tracks exist but not matched ({len(has_tracks_not_matched)} frames):")
        print(f"    dist_sane fail:       {ds_fail} ({100*ds_fail/len(has_tracks_not_matched):.1f}%)")
        print(f"    vel_sane fail:        {vs_fail} ({100*vs_fail/len(has_tracks_not_matched):.1f}%)")
        print(f"    Both pass (mystery):  {both_pass} ({100*both_pass/len(has_tracks_not_matched):.1f}%)")


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

    print("Deep Vision vs Radar Analysis")
    print(f"Processing {len(log_paths)} log files")
    print(f"{'='*70}")

    analyze(log_paths)


if __name__ == '__main__':
    main()
