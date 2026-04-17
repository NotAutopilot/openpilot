#!/usr/bin/env bash
# First-time setup for MagZu/openpilot nap-C3-preap
# Tesla Model S 2014 pre-AP on Comma 3
#
# Run once after cloning onto the device:
#   bash /data/openpilot/setup_c3_preap.sh
#
# This script installs pip packages that are missing on AGNOS 12.8
# but required by openpilot: jeepney and kaitaistruct.
# The root filesystem is read-only on AGNOS, so it is temporarily
# remounted rw for the install.

VENV_PIP="/usr/local/venv/bin/pip"
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

# Install missing pip packages
MISSING=()
"$VENV_PIP" show jeepney > /dev/null 2>&1 || MISSING+=("jeepney")
"$VENV_PIP" show kaitaistruct > /dev/null 2>&1 || MISSING+=("kaitaistruct")

if [ ${#MISSING[@]} -gt 0 ]; then
  echo "Installing: ${MISSING[*]}"
  sudo mount -o remount,rw /
  sudo "$VENV_PIP" install -q "${MISSING[@]}"
  sudo mount -o remount,ro /
  echo "Installed: ${MISSING[*]}"
else
  echo "jeepney and kaitaistruct already installed"
fi

touch "$MARKER"
echo ""
echo "Setup complete. On first boot the device will update to AGNOS 12.8 if not already on it."
