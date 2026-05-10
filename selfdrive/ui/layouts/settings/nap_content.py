"""Shared NAP settings content.

Constants and user-facing text used by the NAP settings panels. Both UI trees
import from here so the safety-critical instruction strings (EPAS flash,
calibration, restore) have a single source.
"""

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


CALIBRATE_PEDAL_INSTRUCTIONS = """\
NAP Pedal Calibrator

This script calibrates the comma pedal interceptor for your pre-AP Tesla Model S.

PRECONDITIONS:
  1. Car must be ON
  2. Gear must be in NEUTRAL
  3. Brake pedal must be PRESSED and held
  4. Do NOT press the accelerator pedal during calibration

The calibration process will:
  - Detect pedal zero position
  - Detect pedal maximum position
  - Fine-tune the scale factor
  - Validate the calibration values
  - Save calibration to params

Press START when ready to begin calibration."""


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

WARNING: This will modify your steering system firmware!

This operation:
  - Requires the vehicle to be safely parked
  - May take several minutes to complete
  - Should NOT be interrupted once started

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

This action only extracts and saves the stock EPAS firmware image.
No flashing or firmware modifications are performed.

Use this before any flash operation so you have a local backup.

PRECONDITIONS:
  - Vehicle safely parked
  - Stable 12V power
  - Do not power-cycle during extraction

Press START to extract the EPAS firmware backup."""


RESTORE_EPAS_INSTRUCTIONS = """\
EPAS Firmware Restore

WARNING: This will reflash your steering system firmware!

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
