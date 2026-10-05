"""Shared NAP settings content.

Constants and user-facing text used by the NAP settings panels. Both UI trees
import from here so the safety-critical instruction strings (EPAS flash,
calibration, restore) have a single source.
"""

from openpilot.system.ui.lib.multilang import tr_noop
from openpilot.selfdrive.car.nap_profiles import PROFILE_LABELS

# Preset values for float/int params exposed as multiple-button selectors.
BRAKE_FACTOR_PRESETS = [0.5, 1.0, 1.5, 2.0]
PEDAL_CAN_BUS_VALUES = [0, 2]

# Follow distance is a 1..7 enum; values match the tap-spoof step on the stalk.
FOLLOW_DISTANCE_MIN = 1
FOLLOW_DISTANCE_MAX = 7

# Radar lateral offset bounds (meters). Added to radar yRel in
# radar_interface.py. Negative = shift toward left; positive = toward right.
# ~0.27 is typical for the 3D-printed factory-location mount.
RADAR_OFFSET_MIN = -2.0
RADAR_OFFSET_MAX = 2.0

# NAPCarType options (see opendbc/car/tesla/nap_detect.py)
CAR_TYPE_VALUES = [0, 1, 2]
CAR_TYPE_LABELS = ["Auto", "Pre-AP", "AP1"]


def car_type_index(value) -> int:
  return CAR_TYPE_VALUES.index(value) if value in CAR_TYPE_VALUES else 0


def active_car_text(active_profile: str | None, car_type) -> str:
  """Which car's calibration and prefs are loaded, and how it was chosen."""
  label = PROFILE_LABELS.get(active_profile or "")
  if label is None:
    return "None yet"
  return f"{label} ({'auto-detected' if car_type_index(car_type) == 0 else 'forced'})"


CALIBRATE_PEDAL_INSTRUCTIONS = tr_noop("""Keep the car ON for calibration.

1. Stop the vehicle and disengage assistance.
2. Select Neutral and hold the brake.
3. Release the accelerator.
4. Check that the pedal connector is seated.
5. Select the pedal's CAN bus below.

Start pauses driving assistance. Cancel stops calibration.
Settings are saved only after calibration succeeds.

When finished, turn the car off to Exit,
or choose Restart device. No automatic restart.""")



CALIBRATE_RADAR_INSTRUCTIONS = """\
NAP Radar Calibrator

This script displays filtered radar points to help align the Bosch radar.

PRECONDITIONS:
  1. Vehicle must be safely parked
  2. Radar must be properly mounted and connected
  3. Place a calibration target 3-10m ahead, centered on the vehicle axis

The calibration display will show:
  - Radar points within 2.5-14.5m ahead
  - Lateral offset from center
  - Adjust radar aim until target shows ~0.0m lateral

Press START when ready to begin."""


TEST_RADAR_INSTRUCTIONS = """\
NAP Radar Test

This script displays live radar data for testing and verification.

The test will show:
  - All detected radar points
  - Distance and relative velocity
  - Track status and confidence

This is useful for:
  - Verifying radar installation
  - Checking radar alignment
  - Debugging radar issues

Press START to begin the radar test."""


FLASH_EPAS_INSTRUCTIONS = """\
EPAS Firmware Flash

STEP 1 — POWER THE CAR ON NOW:
  - Key fob inside the car
  - Foot on the brake
  - Car should be in Park and stay there
  - Do not drive
  (The EPAS ECU only responds when the car is on.)

WARNING: This will modify your steering system firmware!

POWER REQUIREMENTS:
  - 12V battery must be healthy and at a normal charge
  - Do NOT start the flash if the car has been sitting cold
    with marginal voltage
  - Power loss DURING the flash will brick the EPAS module

RISKS:
  - Incorrect firmware can disable power steering
  - Interrupted flash can brick the EPAS module
  - This modification may void warranties

Only proceed if you:
  - Fully understand the implications
  - Have backup EPAS firmware available
  - Are comfortable with the risks involved

Press START only if you accept these risks."""


BACKUP_EPAS_INSTRUCTIONS = """\
EPAS Firmware Backup

STEP 1 — POWER THE CAR ON NOW:
  - Key fob inside the car
  - Foot on the brake
  - Car should be in Park and stay there
  - Do not drive
  (The EPAS ECU only responds when the car is on. If the car
   is not on, the script will time out.)

This action only reads the stock EPAS firmware and saves it.
No flashing or firmware modifications are performed.

Run this before any Flash operation so you have a local backup.

PRECONDITIONS:
  - Stable 12V power
  - Do not power-cycle during extraction

Press START to extract the EPAS firmware backup."""


RESTORE_EPAS_INSTRUCTIONS = """\
EPAS Firmware Restore

STEP 1 — POWER THE CAR ON NOW:
  - Key fob inside the car
  - Foot on the brake
  - Car should be in Park and stay there
  - Do not drive
  (The EPAS ECU only responds when the car is on.)

WARNING: This will reflash your steering system firmware!

POWER REQUIREMENTS:
  - 12V battery must be healthy and at a normal charge
  - Do NOT start the restore if the car has been sitting cold
    with marginal voltage
  - Power loss DURING the restore will brick the EPAS module

This operation:
  - Uses the extracted stock EPAS firmware image
  - May take several minutes to complete
  - Should NOT be interrupted once started

RISKS:
  - Interrupted flash can brick the EPAS module
  - Incorrect image can disable power steering

Only proceed if you:
  - Need to return to stock EPAS firmware
  - Understand and accept the risks

Press START only if you accept these risks."""


ACKNOWLEDGMENTS_INTRO = "Special thanks to the following members. This project wouldn't be possible without you:"

ACKNOWLEDGMENTS_NAMES = [
  "Boggyver and the Tinkla Project",
  "Lukas Loetkolben",
  "Johnmr1",
  "SeriouslySerious",
  "Pod042",
  "1FrostlySlime",
]


def acknowledgments_html() -> str:
  """Render the acknowledgments block as HTML for HtmlRenderer."""
  names_html = "<br>".join(f"<b>{n}</b>" for n in ACKNOWLEDGMENTS_NAMES)
  return f"<p>{ACKNOWLEDGMENTS_INTRO}</p><p>{names_html}</p>"


def acknowledgments_text() -> str:
  """Render the acknowledgments block as plain text."""
  bullets = "\n".join(f"  {n}" for n in ACKNOWLEDGMENTS_NAMES)
  return f"{ACKNOWLEDGMENTS_INTRO}\n\n{bullets}"


def find_preset_index(presets: list, value, default: int = 0) -> int:
  """Find the closest matching preset index for a given value."""
  try:
    return presets.index(value)
  except ValueError:
    return min(range(len(presets)), key=lambda i: abs(presets[i] - value))


def pedal_calibration_entry_block_reason() -> str | None:
  """Pedal-only exception: parked offroad, or ignition-on stationary/disengaged."""
  from openpilot.selfdrive.ui.ui_state import ui_state
  from scripts.nap.pedal_session import admission_from_submaster
  return admission_from_submaster(ui_state.sm)


def pedal_calibration_entry_enabled() -> bool:
  """True when Calibrate Pedal may open the in-process wizard.

  Engaged or moving stay blocked. Waiting for vehicle state still opens
  the wizard so Start can show the reason instead of hiding the entry.
  """
  reason = pedal_calibration_entry_block_reason()
  if reason is None:
    return True
  return "waiting" in reason
