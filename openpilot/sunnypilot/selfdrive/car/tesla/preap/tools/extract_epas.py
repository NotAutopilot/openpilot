"""Extract stock Tesla Pre-AP EPAS firmware image only."""
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.flash_epas import main

if __name__ == "__main__":
  raise SystemExit(main(["--extract-only"]))
