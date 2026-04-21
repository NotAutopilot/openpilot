#!/usr/bin/env bash
# First-time setup for MagZu/openpilot nap-C3-preap
# Tesla Model S 2014 pre-AP on Comma 3
#
# Run once after cloning onto the device:
#   bash /data/openpilot/setup_c3_preap.sh

MARKER="/data/c3_first_run"

if [ ! -f /AGNOS ]; then
  echo "ERROR: not running on AGNOS — this script is for the Comma 3 only"
  exit 1
fi

echo "=== C3 pre-AP first-time setup ==="

# Init submodules (panda + opendbc)
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
echo "Initialising submodules..."
cd "$DIR"
git submodule update --init --depth 1 panda opendbc_repo

touch "$MARKER"
echo ""
echo "Setup complete. On first boot the device will update to AGNOS 12.8 if not already on it."
