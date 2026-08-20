#!/usr/bin/env python3
"""
Tesla Pre-AP Radar VIN Learn (GUI version)

A Bosch radar pulled from another Tesla keeps that car's VIN in its own memory.
It streams tracks for about five seconds after power-up no matter what, then
stops updating them once it decides the VIN, radar position or EPAS type on the
bus don't match what it was programmed with. The symptom is a radar that "works"
for five seconds and then freezes.

This tool runs the radar's UDS VIN-learn routine so it adopts the VIN this car
broadcasts. It does not ask for a VIN: the car's own VIN is read off the chassis
bus (0x405) and the radar learns it from the live bus during the routine.

Sequence (see opendbc/car/tesla/preap/radar_vin.py for the state machine):
  1. read this car's VIN from 0x405 on bus 0
  2. read the VIN currently stored in the radar (DID 0xF190) — if it already
     matches, stop and report success without touching the radar
  3. extended diagnostic session + Tesla SecurityAccess level 1
  4. routineControl start/stop/requestResults on routine 0x0A03 (VIN learn)
  5. read the stored VIN back and confirm it changed

The panda must be in teslaPreap safety mode with PREAP_FLAG_RADAR_VIN_LEARN set:
GTW emulation has to keep feeding the radar this car's VIN, position and EPAS
type while it learns, and the flag is what opens 0x641 on the radar bus in the
TX whitelist. Nothing else in NAP ever sets that flag.

Uses direct Panda access (stops pandad via ScriptRunner).
"""

import argparse
import os
import sys
import time

# ============================================
# Constants
# ============================================
VIN_SOURCE_ADDR = 0x405     # VIN_VIP_405HS on the chassis bus
RADAR_POINT_RANGE = range(0x310, 0x37E)   # track frames, background noise for probing
# 0x641 is what the DBC and Tinkla's tooling use. 0x671 appears in a Tinkla
# source comment as an alternate RADC diagnostic address; probe both.
PROBE_TX_ADDRESSES = (0x641, 0x671)
VIN_SNIFF_TIMEOUT = 20.0    # 0x405 is a ~5 Hz three-part multiplex
LEARN_TIMEOUT = 60.0        # backstop; the learner has its own 30s budget
POLL_INTERVAL = 0.005

PANDA_CONNECT_RETRIES = 5
PANDA_CONNECT_DELAY = 2.0
PANDAD_RELEASE_DELAY = 2.0
PANDAD_STOP_TIMEOUT = 15.0
CONTROL_READ_RETRIES = 3
CONTROL_READ_RETRY_DELAY = 0.2

# Bus numbers above these are panda TX metadata, not received traffic
BUS_ECHO = 128
BUS_REJECTED = 192


def p(msg="", end="\n"):
  """Print with flush for real-time ScriptRunner display."""
  print(msg, end=end, flush=True)


def connect_panda():
  """Connect to Panda with retries."""
  from panda import Panda

  for attempt in range(PANDA_CONNECT_RETRIES):
    try:
      return Panda()
    except Exception as e:
      if attempt < PANDA_CONNECT_RETRIES - 1:
        p(f"  Panda not ready ({e}), retrying in {PANDA_CONNECT_DELAY}s...")
        time.sleep(PANDA_CONNECT_DELAY)
      else:
        raise
  return None


def recv(panda):
  """Drain the panda RX queue.

  Returns (packets, rejected). Echoes of our own sends come back with the bus
  offset by 128 and must not be fed to the learner as if the radar had answered;
  a 192 offset means the safety layer refused the send, which is the signature
  of a panda running firmware without PREAP_FLAG_RADAR_VIN_LEARN.
  """
  from opendbc.car.can_definitions import CanData

  packets = []
  rejected = False
  for msg in panda.can_recv():
    if len(msg) == 4:
      addr, _, dat, src = msg
    elif len(msg) == 3:
      addr, dat, src = msg
    else:
      continue

    if src >= BUS_ECHO:
      if src >= BUS_REJECTED:
        rejected = True
      continue

    packets.append(CanData(addr, bytes(dat), src))
  return packets, rejected


def pandad_running():
  """True if openpilot's pandad still holds the panda.

  manager.py stops pandad when NAPScriptRunning is set, but that takes a manager
  loop plus process teardown. If it is still up when we start, it re-sends
  heartbeats (re-enabling the checks we just disabled) and resets the safety mode
  out from under us — and on a C3X both processes are driving the same SPI bus.

  pandad is a PythonProcess, so /proc/<pid>/comm is the interpreter; match on the
  command line instead.
  """
  try:
    pids = [d for d in os.listdir("/proc") if d.isdigit()]
  except OSError:
    return False   # not Linux / no procfs — can't tell, don't cry wolf

  for pid in pids:
    try:
      with open(f"/proc/{pid}/cmdline", "rb") as f:
        cmdline = f.read().replace(b"\x00", b" ").decode(errors="ignore")
    except OSError:
      continue     # process exited between listing and reading
    if "selfdrive.pandad.pandad" in cmdline:
      return True
  return False


def wait_for_pandad_to_stop(timeout=PANDAD_STOP_TIMEOUT):
  """Wait for pandad to exit. Returns True if it is gone."""
  deadline = time.monotonic() + timeout
  while time.monotonic() < deadline:
    if not pandad_running():
      return True
    time.sleep(0.5)
  return not pandad_running()


def _control_read(panda, method, *args):
  """Run a panda control-transfer read, returning None instead of raising.

  On a C3X the panda is on SPI, and control transfers there can come back NACKed
  at the header-ACK stage — transient, and unrelated to whether CAN itself works
  (the bulk CAN endpoints use a different path, which is why the other NAP tools
  never hit this). These reads are diagnostics: losing one must degrade the
  report, never take down the run.
  """
  for attempt in range(CONTROL_READ_RETRIES):
    try:
      # Resolve the attribute inside the guard: a panda build without this
      # method should degrade the report like any other failed read.
      return getattr(panda, method)(*args)
    except Exception:
      if attempt < CONTROL_READ_RETRIES - 1:
        time.sleep(CONTROL_READ_RETRY_DELAY)
  return None


def bus_health(panda, bus):
  """Per-bus CAN controller counters, or None if the panda won't report them."""
  return _control_read(panda, "can_health", bus)


def panda_health(panda):
  """Overall panda health, or None if the read didn't come back."""
  return _control_read(panda, "health")


def report_tx_health(panda, bus, at_start, before_send):
  """Did our frames physically make it onto the wire?

  safety_tx_blocked only counts frames the safety layer refused. A frame that
  passes safety still has to be transmitted by the CAN peripheral — if nothing
  acknowledges it the controller retries, racks up errors and eventually goes
  bus-off, and none of that shows up as a blocked TX. This separates "the panda
  never sent it" from "the radar ignored it".
  """
  p(f"\n[4/4] Bus {bus} transmit health")
  after = bus_health(panda, bus)
  if after is None or before_send is None or at_start is None:
    p("  panda did not report per-bus CAN health; skipping")
    return

  idle_tx = before_send["total_tx_cnt"] - at_start["total_tx_cnt"]
  send_tx = after["total_tx_cnt"] - before_send["total_tx_cnt"]
  errors = after["total_error_cnt"] - before_send["total_error_cnt"]
  lost = after["total_tx_lost_cnt"] - before_send["total_tx_lost_cnt"]

  p(f"  frames sent while idle:      {idle_tx}  (GTW emulation)")
  p(f"  frames sent during probe:    {send_tx}")
  p(f"  new TX errors during probe:  {errors}")
  p(f"  TX frames lost during probe: {lost}")
  p(f"  last error: {after['last_error']}   bus_off: {after['bus_off']}   "
    + f"error_passive: {after['error_passive']}")
  p(f"  speed: {after['can_speed']} kbps")

  if idle_tx == 0:
    p("\n  Nothing is leaving the panda on this bus, not even GTW emulation.")
    p("  Look at the panda and the bus wiring, not the radar.")
  elif after["bus_off"] or after["last_error"] == "AckError":
    p("\n  Frames are being transmitted but not acknowledged. Nothing on this")
    p("  bus is receiving them — an ACK needs at least one other live node.")
  elif errors == 0 and lost == 0:
    p("\n  Our requests transmitted cleanly and were acknowledged. The radar")
    p("  received them and chose not to reply — this is not a wiring or panda")
    p("  problem, it is the radar's diagnostic layer.")


def probe_radar(panda, listen=3.0, reply_window=1.5):
  """Read-only reachability check. Sends nothing but TesterPresent.

  Answers three questions in order, so a failure lands on a specific one:
    1. can this tool hear bus 1 at all?  (proves the RX path, not just TX)
    2. is the radar transmitting?        (proves it is powered and running)
    3. does it answer a diagnostic request, and at what address?
  """
  from opendbc.car.tesla.preap.radar_vin import RADAR_BUS, RADAR_TX_ADDRESS

  p("\n" + "=" * 40)
  p("PROBE: radar diagnostic reachability")
  p("=" * 40)

  health_start = bus_health(panda, RADAR_BUS)

  p(f"\n[1/4] Listening on bus {RADAR_BUS} for {listen:.0f}s...")
  baseline = {}
  deadline = time.monotonic() + listen
  while time.monotonic() < deadline:
    packets, _ = recv(panda)
    for packet in packets:
      if packet.src == RADAR_BUS:
        baseline[packet.address] = baseline.get(packet.address, 0) + 1
    time.sleep(POLL_INTERVAL)

  total = sum(baseline.values())
  if total == 0:
    p("  NOTHING on bus 1.")
    p("  Either the radar is not powered/transmitting, or bus 1 RX is not")
    p("  reaching this tool. Since the radar works in normal driving, suspect")
    p("  the latter — that would be a tool bug, so report this result.")
    return False

  tracks = sum(count for addr, count in baseline.items() if addr in RADAR_POINT_RANGE)
  p(f"  {total} frames from {len(baseline)} addresses ({tracks} radar track frames)")
  p("  Non-track addresses seen:")
  others = [a for a in sorted(baseline) if a not in RADAR_POINT_RANGE]
  if others:
    for addr in others:
      p(f"    0x{addr:03X}  x{baseline[addr]}")
  else:
    p("    (none)")

  p(f"\n[2/4] Radar is transmitting: {'yes' if tracks else 'NO TRACK FRAMES'}")
  if not tracks:
    p("  The radar is not sending tracks. It may be unpowered — on this install")
    p("  radar power comes off the EPAS fuse, so the car has to be awake.")

  p("\n[3/4] TesterPresent probe")
  health_before_send = bus_health(panda, RADAR_BUS)
  answered = []
  for tx_addr in PROBE_TX_ADDRESSES:
    p(f"\n  → 0x{tx_addr:03X}  02 3E 00")
    panda.can_send(tx_addr, b"\x02\x3e\x00\x00\x00\x00\x00\x00", RADAR_BUS)

    replies = {}
    blocked = False
    deadline = time.monotonic() + reply_window
    while time.monotonic() < deadline:
      packets, rejected = recv(panda)
      blocked = blocked or rejected
      for packet in packets:
        # Anything on the radar bus that wasn't part of the steady-state
        # background is a candidate response, whatever address it uses.
        if packet.src != RADAR_BUS or packet.address in RADAR_POINT_RANGE:
          continue
        if packet.address not in baseline:
          replies.setdefault(packet.address, packet.dat)
      time.sleep(POLL_INTERVAL)

    if blocked:
      p("    panda REFUSED the send — firmware is missing PREAP_FLAG_RADAR_VIN_LEARN")
    elif replies:
      for addr, dat in sorted(replies.items()):
        note = ""
        if len(dat) >= 3 and dat[1] == 0x7F:
          note = f"  (negative response, NRC 0x{dat[3]:02X})" if len(dat) >= 4 else "  (negative response)"
        elif len(dat) >= 2 and dat[1] == 0x7E:
          note = "  (TesterPresent OK)"
        p(f"    ← 0x{addr:03X}: {dat.hex()}{note}")
      answered.append(tx_addr)
    else:
      p("    no reply")

  report_tx_health(panda, RADAR_BUS, health_start, health_before_send)

  p("\n" + "=" * 60)
  if answered:
    p("RADAR ANSWERS DIAGNOSTICS")
    p("=" * 60)
    p(f"Responding to: {', '.join(f'0x{a:03X}' for a in answered)}")
    if RADAR_TX_ADDRESS not in answered:
      p(f"But NOT to 0x{RADAR_TX_ADDRESS:03X}, which is what the learn uses.")
      p("That is the bug — the learn needs to target the address above.")
  else:
    p("RADAR DOES NOT ANSWER DIAGNOSTICS")
    p("=" * 60)
    p("It is on the bus and transmitting, but ignores TesterPresent at every")
    p("address tried. That points at the radar's diagnostic layer, not wiring:")
    p("some firmware builds only accept diagnostics in a particular state.")
  return bool(answered)


def check_safety_mode(panda, expected_mode):
  """Confirm the panda is in the mode we asked for, and say so plainly if not.

  A panda that has fallen back to SAFETY_SILENT blocks every send and stops GTW
  emulation, which looks exactly like a radar that isn't wired up. Catch it here
  instead of blaming the harness later.
  """
  health = panda_health(panda)
  if health is None:
    # Can't read it, so can't rule it out — but this check is a courtesy, not a
    # gate. Carry on; a wrong mode will still surface as blocked or ignored TX.
    p("  (could not read panda health to confirm the mode; continuing)")
    return True
  if health["safety_mode"] == expected_mode:
    return True

  p(f"\nERROR: Panda is in safety mode {health['safety_mode']}, expected {expected_mode}.")
  if health["heartbeat_lost"]:
    p("The panda dropped to SILENT after losing the openpilot heartbeat.")
    p("This is a bug in this tool, not your wiring — please report it.")
  else:
    p("Something else changed the safety mode. Is pandad still running?")
  return False


def sniff_car_vin(panda, timeout=VIN_SNIFF_TIMEOUT):
  """Read this car's VIN from the chassis bus. Returns the VIN or None."""
  from opendbc.car.tesla.preap.radar_vin import RadarVinAssembler

  assembler = RadarVinAssembler()
  deadline = time.monotonic() + timeout
  seen_source = False

  while time.monotonic() < deadline:
    packets, _ = recv(panda)
    for packet in packets:
      if packet.address == VIN_SOURCE_ADDR and packet.src == 0:
        seen_source = True

    vin = assembler.update(packets, time.monotonic())
    if vin is not None:
      return vin
    time.sleep(POLL_INTERVAL)

  if not seen_source:
    p(f"  No 0x{VIN_SOURCE_ADDR:03X} traffic on bus 0 — is the car awake?")
  else:
    p(f"  Saw 0x{VIN_SOURCE_ADDR:03X} but could not assemble a valid VIN")
  return None


def run_learn(panda, target_vin, timeout=LEARN_TIMEOUT):
  """Drive the VIN-learn state machine. Returns (result, failure)."""
  from opendbc.car.tesla.preap.radar_vin import RadarVinLearner, RadarVinLearnerState

  learner = RadarVinLearner()
  learner.start(target_vin, time.monotonic())

  last_state = None
  warned_rejected = False
  deadline = time.monotonic() + timeout

  while learner.state not in (RadarVinLearnerState.COMPLETE, RadarVinLearnerState.FAILED):
    if time.monotonic() > deadline:
      p("\n  Timed out waiting for the radar to finish")
      return None, None

    packets, rejected = recv(panda)
    if rejected and not warned_rejected:
      warned_rejected = True
      p("\n  WARNING: the panda refused to send on the radar bus.")
      p("  The panda firmware predates PREAP_FLAG_RADAR_VIN_LEARN — reboot the")
      p("  device so it reflashes the panda, then run this again.")

    output = learner.update(packets, time.monotonic())
    for can_send in output.can_sends:
      panda.can_send(can_send.address, can_send.dat, can_send.src)

    if learner.state != last_state:
      last_state = learner.state
      p(f"  {learner.state.name}")

    time.sleep(POLL_INTERVAL)

  return learner.result, learner.failure


def parse_args(cli_args=None):
  parser = argparse.ArgumentParser(
    description="Teach a used Tesla Bosch radar this car's VIN",
  )
  parser.add_argument("--probe", action="store_true",
                      help="Read-only reachability check; sends only TesterPresent, learns nothing")
  parser.add_argument("--vin", type=str, default=None,
                      help="Override the target VIN instead of reading it from bus 0")
  parser.add_argument("--sniff-timeout", type=float, default=VIN_SNIFF_TIMEOUT,
                      help=f"Seconds to wait for the car's VIN (default: {VIN_SNIFF_TIMEOUT})")
  parser.add_argument("--timeout", type=float, default=LEARN_TIMEOUT,
                      help=f"Seconds to wait for the learn routine (default: {LEARN_TIMEOUT})")
  return parser.parse_args(cli_args)


def main(cli_args=None):
  from cereal import car
  from opendbc.car.tesla.preap.nap_conf import nap_conf
  from opendbc.car.tesla.preap.radar_vin import RadarVinLearnerResult
  from opendbc.car.tesla.preap.safety_flags import TeslaPreAPSafetyFlags
  SafetyModel = car.CarParams.SafetyModel
  args = parse_args(cli_args)

  p("=" * 60)
  p("RADAR VIN LEARN")
  p("=" * 60)
  p("")
  p("Teaches a used Bosch radar the VIN of this car, so it keeps")
  p("tracking instead of freezing five seconds after power-up.")
  p("")
  p("The car must be awake and in PARK, with your foot on the brake.")
  p("Key fob inside; pressing the brake brings the car up and keeps it there.")
  p("The car may chime or show warnings while this runs — that is normal.")
  p("=" * 60)

  if args.probe:
    p("")
    p("PROBE MODE — read-only. Nothing is written to the radar.")

  if not nap_conf.radar_enabled:
    p("\nERROR: Radar is not enabled.")
    p("Turn on 'Radar Enabled' in NAP settings, reboot, then run this again.")
    return 1

  # The radar validates its mounting position against GTW_carConfig while it
  # learns, so a wrong nosecone setting bakes in a mismatch.
  p("")
  p(f"Radar behind nosecone: {'yes' if nap_conf.radar_behind_nosecone else 'no'}")
  p("If that is wrong, exit now, fix it in NAP settings and reboot first.")

  flags = TeslaPreAPSafetyFlags.RADAR_EMULATION | TeslaPreAPSafetyFlags.RADAR_VIN_LEARN
  if nap_conf.radar_behind_nosecone:
    flags |= TeslaPreAPSafetyFlags.RADAR_BEHIND_NOSECONE

  panda = None
  try:
    p("\nWaiting for pandad to release the panda...")
    if wait_for_pandad_to_stop():
      p("  pandad stopped")
    else:
      p("  WARNING: pandad is STILL RUNNING after "
        + f"{PANDAD_STOP_TIMEOUT:.0f}s.")
      p("  It will fight this tool for the panda — on a C3X both drive the same")
      p("  SPI bus — and its heartbeats re-enable the checks disabled below.")
      p("  Expect SPI errors and no radar replies. Results are not trustworthy.")
    time.sleep(PANDAD_RELEASE_DELAY)

    p("Connecting to Panda...")
    panda = connect_panda()
    p("  Connected")

    # The panda drops to SAFETY_SILENT and enables power save after 5s without a
    # USB heartbeat while the ignition is on (main.c: HEARTBEAT_IGNITION_CNT_ON).
    # Only pandad sends heartbeats and the ScriptRunner has stopped it, so the
    # checks have to go off or every UDS frame — and GTW emulation with it — dies
    # a few seconds in.
    #
    # Order matters: the firmware ignores this request while a car safety mode is
    # active (is_car_safety_mode), and teslaPreap is one. Disable first, then set
    # the mode. Starting pandad again re-enables the checks on its first heartbeat.
    panda.set_heartbeat_disabled()
    panda.set_power_save(False)
    p("  Heartbeat checks disabled for this session")

    panda.set_safety_mode(SafetyModel.teslaPreap, param=int(flags))
    p(f"  Safety mode: teslaPreap (flags={int(flags)}, GTW emulation + VIN learn)")

    if not check_safety_mode(panda, int(SafetyModel.teslaPreap)):
      return 1

    p("  Flushing CAN buffers...")
    panda.can_clear(0xFFFF)
    time.sleep(0.1)
    panda.can_recv()

    if args.probe:
      return 0 if probe_radar(panda) else 1

    # Step 1: this car's VIN
    p("\n" + "=" * 40)
    p("STEP 1: Read this car's VIN")
    p("=" * 40)

    if args.vin is not None:
      target_vin = args.vin.strip().upper()
      p(f"  Using VIN from command line: {target_vin}")
    else:
      p(f"  Listening on bus 0 for 0x{VIN_SOURCE_ADDR:03X}...")
      target_vin = sniff_car_vin(panda, args.sniff_timeout)
      if target_vin is None:
        p("\nERROR: Could not read this car's VIN.")
        p("Make sure the car is awake: key fob inside, press the brake pedal.")
        return 1
      p(f"  Car VIN: {target_vin}")

    # Step 2: the learn routine
    p("\n" + "=" * 40)
    p("STEP 2: Program the radar")
    p("=" * 40)
    p("  Keep your foot on the brake until this finishes.")
    p("")

    health_before = panda_health(panda)
    result, failure = run_learn(panda, target_vin, args.timeout)

    p("")
    p("=" * 60)
    if result == RadarVinLearnerResult.ALREADY_MATCHED:
      p("RADAR ALREADY MATCHES THIS CAR")
      p("=" * 60)
      p("")
      p("The radar was already programmed with this VIN, so nothing changed.")
      p("If it still stops tracking after five seconds, the mismatch is not the")
      p("VIN — check 'Radar Behind Nosecone' and run Test Radar to read the")
      p("radar's alert matrix (0x501).")
      return 0

    if result == RadarVinLearnerResult.LEARNED:
      p("VIN LEARN COMPLETE")
      p("=" * 60)
      p("")
      p(f"The radar now answers to {target_vin}.")
      p("Let the car go to 'CAR OFF', then reboot the device.")
      p("Run the radar calibration next if you have not already.")
      return 0

    p("VIN LEARN FAILED")
    p("=" * 60)
    p("")
    if failure is not None:
      p(f"Reason: {failure.name}")
    p("Nothing was written to the radar.")
    p("")

    # Distinguish "the radar never answered" from "we never got to ask it".
    health = panda_health(panda)
    if health is None or health_before is None:
      p("Could not read panda health, so this could not be narrowed down.")
      p("Run Probe Radar for a fuller picture of what the bus is doing.")
      return 1

    tx_blocked = health["safety_tx_blocked"] - health_before["safety_tx_blocked"]
    if health["safety_mode"] != int(SafetyModel.teslaPreap):
      p(f"The panda left teslaPreap mid-session (now {health['safety_mode']}), so")
      p("the requests never reached the bus. This is a tool bug, not your wiring.")
    elif tx_blocked > 0:
      p(f"The panda blocked {tx_blocked} of our sends. The firmware is missing")
      p("PREAP_FLAG_RADAR_VIN_LEARN — reboot so openpilot reflashes the panda.")
    else:
      p("The requests went out on bus 1 but the radar never answered. Check that")
      p("the radar has 12V and that its CAN pair reaches the radar port on the")
      p("adapter — the car does not carry this traffic, the panda talks to the")
      p("radar directly. Then try again with the car awake and in park.")
    return 1

  except KeyboardInterrupt:
    p("\n\nCancelled — no changes were written to the radar.")
    return 1
  except Exception as e:
    import traceback
    # Through p() rather than print_exc() so it shares the flushed stdout stream
    # and can't interleave, and so the summary below is genuinely last: the
    # ScriptRunner has no scrollback, so whatever prints last is what is read.
    p("\n" + traceback.format_exc())
    p("=" * 60)
    p("STOPPED BY AN UNEXPECTED ERROR")
    p("=" * 60)
    p(f"{type(e).__name__}: {e}")
    p("")
    p("Nothing was written to the radar.")
    if "PandaSpi" in type(e).__name__:
      p("This is a panda SPI transfer failure, not a radar fault. It usually")
      p("means something else is talking to the panda — make sure openpilot is")
      p("not running, then try again.")
    return 1
  finally:
    if panda is not None:
      try:
        panda.set_safety_mode(SafetyModel.silent)
        p("\nSafety mode restored to silent. Reboot the device to resume driving.")
      except Exception:
        pass


if __name__ == "__main__":
  sys.exit(main())
