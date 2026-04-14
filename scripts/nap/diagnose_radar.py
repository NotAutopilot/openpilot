#!/usr/bin/env python3
"""
Tesla Pre-AP Radar Diagnostic Tool

Two modes of operation:
  --panda    Direct panda health check (reads safety mode, bus config).
             Does NOT change safety mode. Quick one-shot check.
  (default)  Cereal CAN monitor. Reads CAN bus via cereal messaging while
             openpilot runs normally, so GTW emulation stays active.

Monitors CAN bus 1 for:
  - All GTW emulation messages (0x109, 0x119, 0x129, 0x149, 0x159,
    0x169, 0x199, 0x1A9, 0x209, 0x219, 0x2A9, 0x2B9, 0x2D9)
  - Radar status: 0x631 (init) -> 0x300 (active)
  - Radar alert matrix (0x501) for error codes
  - Radar tracks (0x310-0x36F) for non-zero vRel

Run on-device:
  # With openpilot running (preferred — sees GTW emulation live):
  python3 scripts/nap/diagnose_radar.py

  # Quick panda health check (can run anytime):
  python3 scripts/nap/diagnose_radar.py --panda
"""

import time
import sys

# ============================================
# GTW Emulation Messages Expected on Bus 1
# ============================================
GTW_MESSAGES = {
    0x109: "DI_torque1",
    0x119: "DI_torque2",
    0x129: "ESP_115h",
    0x149: "ESP_145h",
    0x159: "ESP_C (Brake)",
    0x169: "ESP_wheelSpeeds",
    0x199: "STW_ANGLHP_STAT",
    0x1A9: "DI_espControl",
    0x209: "GTW_odo",
    0x219: "STW_ACTN_RQ",
    0x2A9: "GTW_carConfig",
    0x2B9: "VIP_405HS",
    0x2D9: "BC_status",
}

# Bus 0 source address → Bus 1 GTW destination address mapping
# The firmware re-addresses these on forwarding. We monitor the BUS 0 sources
# because CAN TX doesn't loopback to RX (we can't see our own GTW TX on bus 1).
GTW_SOURCE_MAP = {
    # src_addr: (dest_addr, name, expected_hz)
    0x108: (0x109, "DI_torque1", 100),
    0x118: (0x119, "DI_torque2", 100),    # also source for 0x169 wheel speed synthesis
    0x115: (0x129, "ESP_115h", 50),       # re-addressed; also source for 0x1A9 synthesis
    0x145: (0x149, "ESP_145h", 50),
    0x158: (0x159, "ESP_C (Brake)", 50),
    # 0x169 is SYNTHESIZED from 0x118 — no direct source
    0x0E:  (0x199, "STW_ANGLHP_STAT", 100),
    # 0x1A9 is SYNTHESIZED from 0x115 — no direct source
    0x20A: (0x209, "GTW_odo", 10),
    0x308: (0x219, "STW_ACTN_RQ", 10),    # re-addressed 0x308 → 0x219
    0x2A8: (0x2A9, "GTW_carConfig", 1),
    0x405: (0x2B9, "VIP_405HS", 2),       # re-addressed 0x405 → 0x2B9
    0x398: (0x2D9, "BC_status", 10),      # re-addressed 0x398 → 0x2D9
}

# Critical messages for radar tracking (without these the radar can't compute Doppler)
CRITICAL_SOURCES = {0x118, 0x0E, 0x115}  # wheel speed, steering, ESP control

# Radar control/status messages
RADAR_STATUS_MSGS = {
    0x631: "Radar Init Sync",
    0x300: "Radar Active",
    0x501: "Radar Alert Matrix",
}

# Radar track messages
RADAR_ADDR_START = 0x310
RADAR_ADDR_END = 0x36F
NUM_RADAR_POINTS = 32

RADAR_BUS = 1
DISPLAY_INTERVAL = 1.0

# Safety mode constants
SAFETY_TESLA_LEGACY = 36
SAFETY_FLAG_PREAP = 32
SAFETY_FLAG_RADAR_EMULATION = 256
SAFETY_FLAG_RADAR_BEHIND_NOSECONE = 128
SAFETY_FLAG_LONG_CONTROL = 1
SAFETY_FLAG_ENABLE_PEDAL = 64


def p(msg):
    """Print with flush for real-time display."""
    print(msg, flush=True)


def extract_signal(data, start_bit, length):
    """Extract a little-endian CAN signal from raw bytes."""
    value = 0
    for i in range(length):
        bit_pos = start_bit + i
        byte_idx = bit_pos // 8
        bit_idx = bit_pos % 8
        if byte_idx < len(data) and (data[byte_idx] & (1 << bit_idx)):
            value |= (1 << i)
    return value


def decode_wheel_speed_0x169(data):
    """Decode ESP_wheelSpeeds (0x169) — 4 identical 13-bit speeds."""
    if len(data) < 8:
        return None
    b = data
    rdlr = b[0] | (b[1] << 8) | (b[2] << 16) | (b[3] << 24)
    rdhr = b[4] | (b[5] << 8) | (b[6] << 16) | (b[7] << 24)

    ws0 = rdlr & 0x1FFF
    ws1 = (rdlr >> 13) & 0x1FFF
    ws2 = ((rdlr >> 26) | ((rdhr & 0x7F) << 6)) & 0x1FFF
    ws3 = (rdhr >> 7) & 0x1FFF
    counter = (rdhr >> 20) & 0x0F
    checksum = (rdhr >> 24) & 0xFF

    speeds_kph = [v * 0.04 for v in [ws0, ws1, ws2, ws3]]
    sna = any(v == 0x1FFF for v in [ws0, ws1, ws2, ws3])

    return {
        'speeds_kph': speeds_kph,
        'counter': counter,
        'checksum': checksum,
        'sna': sna,
        'raw': [ws0, ws1, ws2, ws3],
    }


def decode_car_config_0x2A9(data):
    """Decode GTW_carConfig (0x2A9) — check autopilot/radar config bits."""
    if len(data) < 8:
        return None
    rdlr = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)
    rdhr = data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24)

    park_assist = (rdlr >> 8) & 0x03
    fwd_radar_hw = (rdlr >> 6) & 0x03
    das_hw = (rdlr >> 10) & 0x03
    autopilot = (rdhr >> 28) & 0x03
    radar_pos = (rdhr >> 4) & 0x0F
    epas_type = (rdhr >> 12) & 0x0F

    return {
        'park_assist': park_assist,
        'fwd_radar_hw': fwd_radar_hw,
        'das_hw': das_hw,
        'autopilot': autopilot,
        'radar_position': radar_pos,
        'epas_type': epas_type,
    }


# ============================================
# Mode 1: Panda Health Check (--panda)
# ============================================
def panda_health_check():
    """Quick panda diagnostic — reads safety mode, param, bus config.
    Does NOT change safety mode or interfere with openpilot.
    """
    p("=" * 65)
    p("PANDA HEALTH CHECK")
    p("=" * 65)

    # Check NAPRadarEnabled param first
    p("\n--- OPENPILOT PARAMS ---")
    try:
        from openpilot.common.params import Params
        params = Params()
        radar_enabled = params.get_bool("NAPRadarEnabled")
        p(f"  NAPRadarEnabled: {radar_enabled}")
        if not radar_enabled:
            p("  !! RADAR NOT ENABLED — GTW emulation will NOT run")
            p("  !! Set NAPRadarEnabled=True in tesla_setup or params")

        radar_behind = params.get_bool("NAPRadarBehindNosecone")
        p(f"  NAPRadarBehindNosecone: {radar_behind}")
    except Exception as e:
        p(f"  Could not read params: {e}")
        p("  (This is OK if running outside openpilot environment)")

    p("\n--- PANDA CONNECTION ---")
    try:
        from panda import Panda
        panda = Panda()
        p("  Connected to Panda")
    except Exception as e:
        p(f"  FAILED to connect: {e}")
        return 1

    try:
        health = panda.health()
        p("\n--- PANDA HEALTH ---")
        safety_mode = health.get('safety_mode', health.get('safety_model', 'unknown'))
        safety_param = health.get('safety_param', 'unknown')
        p(f"  Safety mode:  {safety_mode} (expect {SAFETY_TESLA_LEGACY} = teslaLegacy)")
        p(f"  Safety param: {safety_param}")

        if safety_mode == SAFETY_TESLA_LEGACY:
            p("  [OK] Safety mode is teslaLegacy")
        else:
            p("  [!!] Safety mode is NOT teslaLegacy!")
            p("       GTW emulation is NOT running!")

        if isinstance(safety_param, int):
            flags = []
            if safety_param & SAFETY_FLAG_PREAP:
                flags.append("PREAP")
            if safety_param & SAFETY_FLAG_LONG_CONTROL:
                flags.append("LONG_CONTROL")
            if safety_param & SAFETY_FLAG_ENABLE_PEDAL:
                flags.append("ENABLE_PEDAL")
            if safety_param & SAFETY_FLAG_RADAR_EMULATION:
                flags.append("RADAR_EMULATION")
            if safety_param & SAFETY_FLAG_RADAR_BEHIND_NOSECONE:
                flags.append("RADAR_BEHIND_NOSECONE")
            p(f"  Flags: {' | '.join(flags) if flags else 'NONE'}")
            p(f"  Param binary: {safety_param:016b}")

            if not (safety_param & SAFETY_FLAG_RADAR_EMULATION):
                p("  [!!] RADAR_EMULATION flag (256) is NOT SET!")
                p("       GTW emulation codepath is disabled!")
                p("       => This is likely the root cause of frozen radar data")
            else:
                p("  [OK] RADAR_EMULATION flag is set")

            if not (safety_param & SAFETY_FLAG_PREAP):
                p("  [!!] PREAP flag (32) is NOT SET!")
            else:
                p("  [OK] PREAP flag is set")

        # Check CAN bus health
        p("\n--- CAN BUS STATUS ---")
        for _bus_key in ['can0', 'can1', 'can2']:
            bus_health_fn = getattr(panda, 'can_health', None)
            if bus_health_fn:
                break
        if bus_health_fn:
            for bus_num in range(3):
                try:
                    bh = bus_health_fn(bus_num)
                    p(f"  Bus {bus_num}: " +
                      f"rx={bh.get('total_rx_cnt', bh.get('receive_count', '?'))}, " +
                      f"tx={bh.get('total_tx_cnt', bh.get('transmit_count', '?'))}, " +
                      f"err={bh.get('total_error_cnt', bh.get('bus_error', '?'))}, " +
                      f"fwd_bus={bh.get('bus_fwd', bh.get('forwarding_bus', '?'))}")
                except Exception as e:
                    p(f"  Bus {bus_num}: error reading health: {e}")
        else:
            p("  (can_health not available on this panda version)")

        # Quick CAN sniff — count bus 1 messages for 3 seconds
        p("\n--- BUS 1 TRAFFIC (3s sniff) ---")
        panda.can_clear(0xFFFF)
        bus1_count = 0
        bus1_addrs = set()
        sniff_start = time.monotonic()
        while time.monotonic() - sniff_start < 3.0:
            for msg in panda.can_recv():
                if len(msg) == 4:
                    addr, _, dat, src = msg
                elif len(msg) == 3:
                    addr, dat, src = msg
                else:
                    continue
                if src == RADAR_BUS:
                    bus1_count += 1
                    bus1_addrs.add(addr)
            time.sleep(0.005)

        p(f"  Total bus 1 messages: {bus1_count}")
        p(f"  Unique addresses: {len(bus1_addrs)}")
        if bus1_count == 0:
            p("  [!!] NO TRAFFIC on bus 1!")
            p("       Radar is not connected, or CAN bus not configured")
        else:
            gtw_seen = bus1_addrs & set(GTW_MESSAGES.keys())
            radar_seen = {a for a in bus1_addrs if RADAR_ADDR_START <= a <= RADAR_ADDR_END}
            status_seen = bus1_addrs & set(RADAR_STATUS_MSGS.keys())
            p(f"  GTW emulation msgs: {len(gtw_seen)}/{len(GTW_MESSAGES)} present")
            if gtw_seen:
                for a in sorted(gtw_seen):
                    p(f"    [OK] 0x{a:03X} {GTW_MESSAGES[a]}")
            missing_gtw = set(GTW_MESSAGES.keys()) - gtw_seen
            for a in sorted(missing_gtw):
                p(f"    [!!] 0x{a:03X} {GTW_MESSAGES[a]} MISSING")
            p(f"  Radar track msgs:   {'YES' if radar_seen else 'NO'} ({len(radar_seen)} addrs)")
            p(f"  Radar status msgs:  {sorted(f'0x{a:03X}' for a in status_seen) if status_seen else 'NONE'}")

        panda.close()
        p("\nPanda closed (safety mode unchanged).")

    except Exception as e:
        p(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        try:
            panda.close()
        except Exception:
            pass
        return 1

    return 0


# ============================================
# Mode 2: Cereal CAN Monitor (default)
# ============================================
def cereal_monitor():
    """Monitor CAN bus via cereal messaging while openpilot runs.
    GTW emulation stays active because we don't touch the panda.

    Monitors BOTH buses:
      - Bus 0: source messages that trigger GTW forwarding (measures input rates)
      - Bus 1: radar output (tracks, status, any GTW loopback)

    Since CAN TX doesn't loopback to RX, we can't directly see GTW messages
    on bus 1. But by measuring bus 0 source rates, we know exactly what the
    radar is receiving (every source message triggers an event-driven forward).
    """
    p("=" * 65)
    p("RADAR DIAGNOSTIC TOOL (Cereal CAN Monitor)")
    p("=" * 65)
    p("")
    p("Monitors bus 0 SOURCE rates (input to GTW emulation)")
    p("and bus 1 radar output (tracks + status).")
    p("CAN TX doesn't loopback, so GTW msgs won't appear on bus 1.")
    p("Instead, bus 0 source rates = GTW forwarding rates.")
    p("Ctrl+C to stop.")
    p("=" * 65)

    try:
        from cereal import messaging
    except ImportError:
        p("\nERROR: cereal not available. Run on-device with openpilot environment.")
        p("Alternatively, use --panda mode for direct panda access.")
        return 1

    sm = messaging.SubMaster(['can'])

    # --- Bus 0 source message tracking ---
    src_total_counts = dict.fromkeys(GTW_SOURCE_MAP, 0)
    src_interval_counts = dict.fromkeys(GTW_SOURCE_MAP, 0)
    src_last_timestamps = dict.fromkeys(GTW_SOURCE_MAP)
    # Per-source: track max gap between consecutive messages (detect burstiness)
    src_max_gap_ms = dict.fromkeys(GTW_SOURCE_MAP, 0.0)
    src_min_gap_ms = {addr: float('inf') for addr in GTW_SOURCE_MAP}
    src_gap_count = dict.fromkeys(GTW_SOURCE_MAP, 0)  # gaps > 2x expected interval
    # Per-second history for rate trend (last 10 seconds)
    src_rate_history = {addr: [] for addr in GTW_SOURCE_MAP}

    # --- Bus 1 radar output tracking ---
    radar_status_counts = dict.fromkeys(RADAR_STATUS_MSGS, 0)
    radar_status = "UNKNOWN"
    radar_status_transitions = []  # list of (timestamp, old_status, new_status)
    tracks_with_velocity = 0
    total_tracked = 0
    last_car_config = None
    alert_data = None

    # --- Vehicle speed from bus 0 source (0x118 DI_torque2) ---
    last_vehicle_speed = None

    # --- Bus 1 GTW messages (shouldn't appear, but check anyway) ---
    gtw_bus1_counts = dict.fromkeys(GTW_MESSAGES, 0)

    start_time = time.monotonic()
    last_display = 0
    last_interval_reset = start_time
    total_bus0_msgs = 0
    total_bus1_msgs = 0
    interval_num = 0

    p("\nListening on CAN buses 0+1 via cereal...")

    try:
        while True:
            sm.update(100)  # 100ms timeout

            if sm.updated['can']:
                now = time.monotonic()
                for msg in sm['can']:
                    addr = msg.address
                    bus = msg.src
                    data = bytes(msg.dat)

                    # ---- Bus 0: GTW source messages ----
                    if bus == 0 and addr in GTW_SOURCE_MAP:
                        total_bus0_msgs += 1
                        src_total_counts[addr] += 1
                        src_interval_counts[addr] += 1

                        # Track inter-message timing
                        prev_ts = src_last_timestamps[addr]
                        if prev_ts is not None:
                            gap_ms = (now - prev_ts) * 1000.0
                            if gap_ms > src_max_gap_ms[addr]:
                                src_max_gap_ms[addr] = gap_ms
                            if gap_ms < src_min_gap_ms[addr]:
                                src_min_gap_ms[addr] = gap_ms
                            # Flag gaps > 2x expected interval
                            _, _, expected_hz = GTW_SOURCE_MAP[addr]
                            if expected_hz > 0:
                                expected_ms = 1000.0 / expected_hz
                                if gap_ms > expected_ms * 2.5:
                                    src_gap_count[addr] += 1
                        src_last_timestamps[addr] = now

                        # Decode vehicle speed from DI_torque2 (0x118)
                        # This is the source for 0x169 wheel speed synthesis
                        if addr == 0x118 and len(data) >= 4:
                            rdlr = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)
                            raw_speed = (rdlr >> 16) & 0xFFF  # bits [27:16], 12-bit
                            speed_mph = raw_speed * 0.05 - 25.0
                            speed_kph = speed_mph * 1.609344
                            last_vehicle_speed = {'mph': speed_mph, 'kph': speed_kph, 'raw': raw_speed}

                    # ---- Bus 1: Radar output ----
                    if bus == RADAR_BUS:
                        total_bus1_msgs += 1

                        # Check for GTW messages on bus 1 (shouldn't appear due to no TX loopback)
                        if addr in GTW_MESSAGES:
                            gtw_bus1_counts[addr] += 1

                        # Radar status messages
                        if addr in RADAR_STATUS_MSGS:
                            radar_status_counts[addr] += 1
                            new_status = None
                            if addr == 0x631:
                                new_status = "INIT (0x631)"
                            elif addr == 0x300:
                                new_status = "ACTIVE (0x300)"
                            elif addr == 0x501:
                                alert_data = data

                            if new_status and new_status != radar_status:
                                radar_status_transitions.append(
                                    (now - start_time, radar_status, new_status))
                                radar_status = new_status

                        # Radar track messages
                        if RADAR_ADDR_START <= addr <= RADAR_ADDR_END:
                            is_b = (addr - RADAR_ADDR_START) % 2 == 1
                            if not is_b:
                                tracked = extract_signal(data, 62, 1)
                                if tracked:
                                    total_tracked += 1
                                    vrel = extract_signal(data, 12, 12) * 0.0625 - 128
                                    if abs(vrel) > 0.1:
                                        tracks_with_velocity += 1

            now = time.monotonic()
            if now - last_display >= DISPLAY_INTERVAL:
                interval_dt = now - last_interval_reset
                last_interval_reset = now
                interval_num += 1
                last_display = now
                elapsed = now - start_time

                # Record per-second rates in history
                for addr in GTW_SOURCE_MAP:
                    rate = src_interval_counts[addr] / max(interval_dt, 0.01)
                    src_rate_history[addr].append(rate)
                    if len(src_rate_history[addr]) > 10:
                        src_rate_history[addr].pop(0)

                p(f"\n{'='*65}")
                p(f"[{elapsed:.0f}s] Bus 0: {total_bus0_msgs} src msgs | Bus 1: {total_bus1_msgs} radar msgs")
                p(f"{'='*65}")

                # ===== Section 1: Bus 0 Source Message Rates =====
                p("\n--- BUS 0 SOURCE RATES (GTW emulation input) ---")
                p(f"  {'Source':>6s} → {'Dest':>6s}  {'Name':20s}  {'Rate':>7s}  {'Expect':>7s}  {'MaxGap':>8s}  {'Gaps':>5s}")
                p(f"  {'-'*6:>6s}   {'-'*6:>6s}  {'-'*20:20s}  {'-'*7:>7s}  {'-'*7:>7s}  {'-'*8:>8s}  {'-'*5:>5s}")

                any_critical_missing = False
                for src_addr in sorted(GTW_SOURCE_MAP.keys()):
                    dest_addr, name, expected_hz = GTW_SOURCE_MAP[src_addr]
                    count = src_total_counts[src_addr]
                    rate = count / max(elapsed, 0.1)
                    max_gap = src_max_gap_ms[src_addr]
                    gaps = src_gap_count[src_addr]

                    # Status indicator
                    if count == 0:
                        status = "!!"
                    elif expected_hz > 0 and rate < expected_hz * 0.5:
                        status = "LO"
                    else:
                        status = "OK"

                    is_critical = src_addr in CRITICAL_SOURCES
                    marker = " *" if is_critical else "  "

                    if count > 0:
                        gap_str = f"{max_gap:6.0f}ms"
                        gap_warn = f" {gaps:4d}" if gaps > 0 else "    0"
                    else:
                        gap_str = "     N/A"
                        gap_warn = "  N/A"
                        if is_critical:
                            any_critical_missing = True

                    p(f"  [{status}] 0x{src_addr:03X} → 0x{dest_addr:03X}  {name:20s}  {rate:5.1f}/s  {expected_hz:5d}/s  {gap_str}  {gap_warn}{marker}")

                p("\n  * = critical for radar Doppler (vehicle speed, steering, ESP)")
                p("  Gaps = count of intervals > 2.5x expected period")

                if any_critical_missing:
                    p("  !! CRITICAL SOURCES MISSING — radar CANNOT compute Doppler!")

                # ===== Section 1b: Synthesized messages =====
                p("\n  Synthesized messages (derived from sources above):")
                ws_rate = src_total_counts.get(0x118, 0) / max(elapsed, 0.1)
                esp_rate = src_total_counts.get(0x115, 0) / max(elapsed, 0.1)
                p(f"    0x169 ESP_wheelSpeeds: synthesized from 0x118 at {ws_rate:.1f}/s (expect 100/s)")
                p(f"    0x1A9 DI_espControl:   synthesized from 0x115 at {esp_rate:.1f}/s (expect  50/s)")

                # ===== Section 2: Rate Trend (last 10s) for critical messages =====
                p("\n--- CRITICAL SOURCE RATE TREND (last 10s, per-second) ---")
                for src_addr in sorted(CRITICAL_SOURCES):
                    dest_addr, name, expected_hz = GTW_SOURCE_MAP[src_addr]
                    history = src_rate_history.get(src_addr, [])
                    if history:
                        hist_str = " ".join(f"{r:5.0f}" for r in history[-10:])
                        min_r = min(history[-10:])
                        max_r = max(history[-10:])
                        p(f"  0x{src_addr:03X} ({name[:16]:16s}): [{hist_str}]  range=[{min_r:.0f}-{max_r:.0f}]  expect={expected_hz}")
                    else:
                        p(f"  0x{src_addr:03X} ({name[:16]:16s}): no data yet")

                # ===== Section 3: Vehicle Speed (from bus 0 source) =====
                p("\n--- VEHICLE SPEED (from 0x118 DI_torque2 on bus 0) ---")
                if last_vehicle_speed:
                    vs = last_vehicle_speed
                    p(f"  Speed: {vs['kph']:.1f} KPH / {vs['mph']:.1f} MPH  (raw={vs['raw']})")
                    p("  This is used by firmware to synthesize 0x169 wheel speeds")
                else:
                    p("  No 0x118 data seen on bus 0")

                # ===== Section 4: Car Config Decode =====
                p("\n--- CAR CONFIG (0x2A9) ---")
                if last_car_config:
                    cc = last_car_config
                    p(f"  Autopilot: {cc['autopilot']}  RadarPos: {cc['radar_position']}  EpasType: {cc['epas_type']}")
                    p(f"  ParkAssist: {cc['park_assist']}  FwdRadarHW: {cc['fwd_radar_hw']}  DAS_HW: {cc['das_hw']}")
                else:
                    p("  No 0x2A9 data received")

                # ===== Section 5: Radar Status & Transitions =====
                p("\n--- RADAR STATUS ---")
                p(f"  Current: {radar_status}")
                for addr, name in sorted(RADAR_STATUS_MSGS.items()):
                    count = radar_status_counts[addr]
                    if count > 0:
                        rate = count / max(elapsed, 0.1)
                        p(f"  0x{addr:03X} {name}: {count} msgs ({rate:.1f}/s)")

                if radar_status_transitions:
                    p(f"\n  Status transitions ({len(radar_status_transitions)} total):")
                    for ts, old, new in radar_status_transitions[-5:]:
                        p(f"    [{ts:.1f}s] {old} → {new}")
                    if len(radar_status_transitions) > 5:
                        p(f"    ... ({len(radar_status_transitions) - 5} earlier transitions omitted)")

                if alert_data and len(alert_data) >= 2:
                    p(f"  Alert data: {alert_data.hex()}")

                # ===== Section 6: Track Velocity Check =====
                p("\n--- RADAR TRACKING (this interval) ---")
                if total_tracked > 0:
                    p(f"  Tracked points: {total_tracked}")
                    p(f"  With non-zero vRel: {tracks_with_velocity}")
                    if tracks_with_velocity == 0:
                        p("  WARNING: ALL tracks have vRel=0 (Doppler not working)")
                        p("  => Radar is in degraded 'raw echo' mode")
                    else:
                        pct = 100.0 * tracks_with_velocity / total_tracked
                        p(f"  Doppler: {pct:.0f}% of tracks have velocity")
                else:
                    p("  No tracked radar points")

                # ===== Section 7: Bus 1 GTW check (should be empty) =====
                gtw_on_bus1 = sum(gtw_bus1_counts.values())
                if gtw_on_bus1 > 0:
                    p("\n--- UNEXPECTED: GTW messages seen on bus 1 RX! ---")
                    for addr, count in sorted(gtw_bus1_counts.items()):
                        if count > 0:
                            p(f"  0x{addr:03X} {GTW_MESSAGES[addr]}: {count}")
                    p("  (CAN TX normally doesn't loopback — this is unusual)")

                # Reset per-interval counters
                total_tracked = 0
                tracks_with_velocity = 0
                src_interval_counts = dict.fromkeys(GTW_SOURCE_MAP, 0)

    except KeyboardInterrupt:
        p("\nStopped.")
        # Print final summary
        elapsed = time.monotonic() - start_time
        p(f"\n{'='*65}")
        p(f"FINAL SUMMARY ({elapsed:.0f}s)")
        p(f"{'='*65}")
        p(f"  Bus 0 source msgs: {total_bus0_msgs}")
        p(f"  Bus 1 radar msgs:  {total_bus1_msgs}")
        p(f"  Status transitions: {len(radar_status_transitions)}")
        p("\n  Source message timing:")
        for src_addr in sorted(GTW_SOURCE_MAP.keys()):
            dest_addr, name, expected_hz = GTW_SOURCE_MAP[src_addr]
            count = src_total_counts[src_addr]
            if count > 0:
                rate = count / max(elapsed, 0.1)
                max_gap = src_max_gap_ms[src_addr]
                min_gap = src_min_gap_ms[src_addr]
                gaps = src_gap_count[src_addr]
                p(f"    0x{src_addr:03X} → 0x{dest_addr:03X}: {rate:.1f}/s avg, gaps={gaps}, " +
                  f"timing=[{min_gap:.1f}-{max_gap:.1f}ms]")
            else:
                p(f"    0x{src_addr:03X} → 0x{dest_addr:03X}: NOT SEEN")

    p("\nDiagnostic ended.")
    return 0


def main():
    if '--panda' in sys.argv:
        return panda_health_check()
    else:
        return cereal_monitor()


if __name__ == "__main__":
    sys.exit(main())
