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
import sys
import time

# ============================================
# Constants
# ============================================
VIN_SOURCE_ADDR = 0x405     # VIN_VIP_405HS on the chassis bus
VIN_SNIFF_TIMEOUT = 20.0    # 0x405 is a ~5 Hz three-part multiplex
LEARN_TIMEOUT = 60.0        # backstop; the learner has its own 30s budget
POLL_INTERVAL = 0.005

PANDA_CONNECT_RETRIES = 5
PANDA_CONNECT_DELAY = 2.0
PANDAD_RELEASE_DELAY = 2.0

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


def check_safety_mode(panda, expected_mode):
  """Confirm the panda is in the mode we asked for, and say so plainly if not.

  A panda that has fallen back to SAFETY_SILENT blocks every send and stops GTW
  emulation, which looks exactly like a radar that isn't wired up. Catch it here
  instead of blaming the harness later.
  """
  health = panda.health()
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
    p("\nWaiting for pandad to release Panda USB...")
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

    tx_blocked_before = panda.health()["safety_tx_blocked"]
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
    health = panda.health()
    tx_blocked = health["safety_tx_blocked"] - tx_blocked_before
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
    p(f"\nERROR: {e}")
    import traceback
    traceback.print_exc()
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
