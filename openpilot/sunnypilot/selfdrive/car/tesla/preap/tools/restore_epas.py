"""Restore stock Tesla Pre-AP EPAS firmware image."""
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.flash_epas import main

if __name__ == "__main__":
  raise SystemExit(main(["--restore"]))
