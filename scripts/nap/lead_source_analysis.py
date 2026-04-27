#!/usr/bin/env python3
"""
Lead Source Analysis — check if leadOne.radar flips between True/False,
and correlate with longitudinal plan accel changes.

Usage: python scripts/nap/lead_source_analysis.py [log_dir]
"""

import sys
import struct
from pathlib import Path
from collections import defaultdict


def read_events(path):
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
    # capnp fallback
    import capnp
    schema_paths = list(Path('/Users/jackbrandt/projects/openpilot-dev').rglob('log.capnp'))
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
                total_data += struct.unpack_from('<I', raw, pos + 4 + s * 4)[0] * 8
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
                events.append(event)
        except Exception:
            continue
    print(f" {len(events)} events", flush=True)
    return events


def analyze(log_paths):
    paths = sorted(log_paths)[:5]

    rs_data = []      # (time, leadOne.status, leadOne.radar, leadOne.dRel, leadOne.radarTrackId, valid)
    lp_data = []      # (time, hasLead, source, aDesired)
    lt_data = []      # (time, n_points)

    for path in paths:
        events = read_events(path)
        for evt in events:
            which = evt.which()
            mono = evt.logMonoTime

            if which == 'radarState':
                rs = evt.radarState
                l1 = rs.leadOne
                rs_data.append((mono, l1.status, l1.radar, l1.dRel, l1.radarTrackId, evt.valid))

            elif which == 'longitudinalPlan':
                lp = evt.longitudinalPlan
                lp_data.append((mono, lp.hasLead, str(lp.longitudinalPlanSource),
                               lp.accels[0] if len(lp.accels) > 0 else 0.0, evt.valid))

            elif which == 'liveTracks':
                lt = evt.liveTracks
                lt_data.append((mono, len(lt.points)))

    print(f"\n{'='*70}")
    print("LEAD SOURCE ANALYSIS")
    print(f"{'='*70}")

    # radarState analysis
    if rs_data:
        n_status = sum(1 for d in rs_data if d[1])
        n_radar = sum(1 for d in rs_data if d[1] and d[2])
        n_vision = sum(1 for d in rs_data if d[1] and not d[2])
        n_valid = sum(1 for d in rs_data if d[5])
        n_invalid = sum(1 for d in rs_data if not d[5])

        print(f"\n  radarState frames: {len(rs_data)}")
        print(f"    valid:                {n_valid} ({100*n_valid/len(rs_data):.1f}%)")
        print(f"    invalid:              {n_invalid} ({100*n_invalid/len(rs_data):.1f}%)")
        print(f"    leadOne.status=True:  {n_status} ({100*n_status/len(rs_data):.1f}%)")
        print(f"    leadOne.radar=True:   {n_radar} ({100*n_status/len(rs_data) if n_status else 0:.1f}% of total)")
        print(f"    leadOne.radar=False:  {n_vision} (vision-only)")

        if n_status > 0:
            print(f"    Of active leads: {100*n_radar/n_status:.1f}% radar, {100*n_vision/n_status:.1f}% vision-only")

        # Count radar↔vision transitions
        transitions = 0
        prev_radar = None
        for d in rs_data:
            if d[1]:  # status=True
                if prev_radar is not None and d[2] != prev_radar:
                    transitions += 1
                prev_radar = d[2]
        duration = (rs_data[-1][0] - rs_data[0][0]) * 1e-9
        print(f"\n    Radar↔Vision transitions: {transitions} in {duration:.1f}s = {transitions/max(duration,0.1):.2f}/s")

        # Track ID changes
        prev_tid = None
        tid_changes = 0
        for d in rs_data:
            if d[1]:  # status=True
                if prev_tid is not None and d[4] != prev_tid:
                    tid_changes += 1
                prev_tid = d[4]
        print(f"    Track ID changes:         {tid_changes} in {duration:.1f}s = {tid_changes/max(duration,0.1):.2f}/s")

        # valid↔invalid transitions
        valid_transitions = 0
        prev_valid = None
        for d in rs_data:
            if prev_valid is not None and d[5] != prev_valid:
                valid_transitions += 1
            prev_valid = d[5]
        print(f"    Valid↔Invalid transitions: {valid_transitions}")

    # liveTracks analysis
    if lt_data:
        duration = (lt_data[-1][0] - lt_data[0][0]) * 1e-9
        rate = len(lt_data) / max(duration, 0.1)
        n_zero = sum(1 for d in lt_data if d[1] == 0)
        n_nonzero = sum(1 for d in lt_data if d[1] > 0)
        print("\n  liveTracks:")
        print(f"    Count: {len(lt_data)}, Rate: {rate:.1f} Hz")
        print(f"    Zero points:    {n_zero} ({100*n_zero/len(lt_data):.1f}%)")
        print(f"    Non-zero:       {n_nonzero} ({100*n_nonzero/len(lt_data):.1f}%)")

    # longitudinalPlan analysis
    if lp_data:
        n_has_lead = sum(1 for d in lp_data if d[1])
        n_valid = sum(1 for d in lp_data if d[4])
        sources = defaultdict(int)
        for d in lp_data:
            sources[d[2]] += 1

        print("\n  longitudinalPlan:")
        print(f"    Count: {len(lp_data)}")
        print(f"    valid: {n_valid} ({100*n_valid/len(lp_data):.1f}%)")
        print(f"    hasLead=True: {n_has_lead} ({100*n_has_lead/len(lp_data):.1f}%)")
        print(f"    Sources: {dict(sources)}")

        # Check accel oscillation
        accels = [d[3] for d in lp_data]
        sign_changes = 0
        for i in range(1, len(accels)):
            if accels[i] * accels[i-1] < 0:
                sign_changes += 1
        duration = (lp_data[-1][0] - lp_data[0][0]) * 1e-9
        print(f"    Accel sign changes: {sign_changes} in {duration:.1f}s = {sign_changes/max(duration,0.1):.2f}/s")


def main():
    if len(sys.argv) > 1:
        p = Path(sys.argv[1])
        if p.is_dir():
            log_paths = sorted(p.glob('*.zst'))
        else:
            log_paths = [p]
    else:
        log_dir = Path('/Users/jackbrandt/projects/openpilot-dev/logs/crappy-radar/route-7')
        log_paths = sorted(log_dir.glob('*.zst'))

    if not log_paths:
        print("No .zst log files found.")
        sys.exit(1)

    analyze(log_paths)


if __name__ == '__main__':
    main()
