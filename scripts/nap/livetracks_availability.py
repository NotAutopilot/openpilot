#!/usr/bin/env python3
"""
LiveTracks Availability Analysis

Investigate why liveTracks has zero points 90% of the time.
Checks:
  1. liveTracks update rate vs radarState rate
  2. When liveTracks has points vs when it doesn't
  3. Number of points per liveTracks message
  4. Timing gaps in liveTracks

Usage:
  python scripts/nap/livetracks_availability.py [log_dir]
"""

import sys
import struct
from pathlib import Path
from collections import defaultdict
import statistics

def read_events_all(path):
    """Read ALL events (not just filtered) to check timing."""
    import zstandard as zstd
    print(f"  Reading {Path(path).name}...", end="", flush=True)

    dctx = zstd.ZstdDecompressor()
    with open(path, 'rb') as f:
        raw = dctx.decompress(f.read(), max_output_size=500 * 1024 * 1024)

    try:
        from cereal import log as cereal_log
        events = list(cereal_log.Event.read_multiple_bytes(raw))
        print(f" {len(events)} events", flush=True)
        return events
    except ImportError:
        pass

    import capnp
    schema_paths = list(Path('/Users/jackbrandt/projects/openpilot-dev').rglob('log.capnp'))
    if not schema_paths:
        print(" ERROR", flush=True)
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

    events = []
    for msg_bytes in msg_list:
        try:
            with log_capnp.Event.from_bytes(msg_bytes, traversal_limit_in_words=2**63 - 1) as event:
                events.append({
                    'which': event.which(),
                    'logMonoTime': event.logMonoTime,
                    '_raw': event,
                })
        except Exception:
            continue
    print(f" {len(events)} events", flush=True)
    return events


def analyze(log_paths):
    # Process first 2 files for detailed timing analysis
    paths_to_analyze = sorted(log_paths)[:3]

    lt_times = []      # liveTracks timestamps
    lt_n_points = []   # number of points per liveTracks message
    rs_times = []      # radarState timestamps
    mv_times = []      # modelV2 timestamps
    all_event_types = defaultdict(int)

    for path in paths_to_analyze:
        events = read_events_all(path)
        for evt in events:
            if isinstance(evt, dict):
                which = evt['which']
                mono = evt['logMonoTime']
            else:
                which = evt.which()
                mono = evt.logMonoTime

            all_event_types[which] += 1

            if which == 'liveTracks':
                lt_times.append(mono)
                try:
                    if isinstance(evt, dict) and '_raw' in evt:
                        lt = evt['_raw'].liveTracks
                    else:
                        lt = evt.liveTracks
                    pts = list(lt.points)
                    lt_n_points.append(len(pts))
                except Exception:
                    lt_n_points.append(-1)

            elif which == 'radarState':
                rs_times.append(mono)

            elif which == 'modelV2':
                mv_times.append(mono)

    print(f"\n{'='*70}")
    print("LIVETRACKS AVAILABILITY ANALYSIS")
    print(f"{'='*70}")

    print(f"\n  Analyzed {len(paths_to_analyze)} log files")

    # Event type distribution
    print(f"\n  Event types in first {len(paths_to_analyze)} files:")
    for etype, count in sorted(all_event_types.items(), key=lambda x: -x[1])[:15]:
        print(f"    {etype:<25} {count:>6}")

    # liveTracks timing
    if lt_times:
        lt_times.sort()
        lt_intervals = [(lt_times[i] - lt_times[i-1]) * 1e-9 for i in range(1, len(lt_times))]
        duration = (lt_times[-1] - lt_times[0]) * 1e-9
        rate = len(lt_times) / max(duration, 0.1)
        print("\n  liveTracks:")
        print(f"    Count:                {len(lt_times)}")
        print(f"    Duration:             {duration:.1f}s")
        print(f"    Rate:                 {rate:.1f} Hz")
        if lt_intervals:
            print(f"    Interval mean:        {statistics.mean(lt_intervals)*1000:.1f} ms")
            print(f"    Interval median:      {statistics.median(lt_intervals)*1000:.1f} ms")
            print(f"    Interval max:         {max(lt_intervals)*1000:.1f} ms")
            gaps_1s = sum(1 for i in lt_intervals if i > 1.0)
            gaps_500ms = sum(1 for i in lt_intervals if i > 0.5)
            print(f"    Gaps > 500ms:         {gaps_500ms}")
            print(f"    Gaps > 1s:            {gaps_1s}")

    # liveTracks points per message
    if lt_n_points:
        zero_pts = sum(1 for n in lt_n_points if n == 0)
        nonzero_pts = sum(1 for n in lt_n_points if n > 0)
        print("\n  liveTracks points per message:")
        print(f"    Zero points:          {zero_pts}/{len(lt_n_points)} ({100*zero_pts/len(lt_n_points):.1f}%)")
        print(f"    Non-zero points:      {nonzero_pts}/{len(lt_n_points)} ({100*nonzero_pts/len(lt_n_points):.1f}%)")
        if nonzero_pts > 0:
            nz = [n for n in lt_n_points if n > 0]
            print(f"    When non-zero, mean:  {statistics.mean(nz):.1f}")
            print(f"    When non-zero, max:   {max(nz)}")

    # radarState timing
    if rs_times:
        rs_times.sort()
        [(rs_times[i] - rs_times[i-1]) * 1e-9 for i in range(1, len(rs_times))]
        rs_duration = (rs_times[-1] - rs_times[0]) * 1e-9
        rs_rate = len(rs_times) / max(rs_duration, 0.1)
        print("\n  radarState:")
        print(f"    Count:                {len(rs_times)}")
        print(f"    Rate:                 {rs_rate:.1f} Hz")

    # modelV2 timing
    if mv_times:
        mv_times.sort()
        mv_duration = (mv_times[-1] - mv_times[0]) * 1e-9
        mv_rate = len(mv_times) / max(mv_duration, 0.1)
        print("\n  modelV2:")
        print(f"    Count:                {len(mv_times)}")
        print(f"    Rate:                 {mv_rate:.1f} Hz")

    # Key ratio
    if lt_times and rs_times:
        lt_dur = (lt_times[-1] - lt_times[0]) * 1e-9
        rs_dur = (rs_times[-1] - rs_times[0]) * 1e-9
        lt_rate = len(lt_times) / max(lt_dur, 0.1)
        rs_rate = len(rs_times) / max(rs_dur, 0.1)
        print("\n  Rate comparison:")
        print(f"    liveTracks:           {lt_rate:.1f} Hz")
        print(f"    radarState:           {rs_rate:.1f} Hz")
        print(f"    Ratio (RS/LT):        {rs_rate/max(lt_rate, 0.1):.1f}x")
        print()
        print("  CRITICAL: radard polls on modelV2 (~20Hz) and only calls")
        print("  RD.update() when sm.updated['liveTracks'] is True.")
        print(f"  If liveTracks rate is {lt_rate:.0f}Hz but radarState is {rs_rate:.0f}Hz,")
        print(f"  then {rs_rate/max(lt_rate, 0.1):.1f}x radarState frames run with STALE liveTracks.")
        print("  radard ALWAYS publishes on every modelV2 cycle, but only")
        print("  calls RD.update() when liveTracks is updated.")
        print()
        print("  BUT: if liveTracks messages have zero points, RD.update()")
        print("  will clear self.tracks (line 258-260 in radard.py) on every")
        print("  radar cycle where no points exist.")


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

    print("LiveTracks Availability Analysis")
    print(f"{'='*70}")
    analyze(log_paths)


if __name__ == '__main__':
    main()
