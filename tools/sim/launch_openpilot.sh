#!/usr/bin/env bash

export PASSIVE="0"
export NOBOARD="1"
export SIMULATION="1"
export SKIP_FW_QUERY="1"
export FINGERPRINT="TESLA_MODEL_S_PREAP"
export VIN="5YJSA1H13EFP20460"

export BLOCK="${BLOCK},camerad,loggerd,encoderd,micd,logmessaged,manage_athenad"
if [[ "$CI" ]] || [[ -z "${DISPLAY:-}" ]]; then
  # TODO: offscreen UI should work
  export BLOCK="${BLOCK},ui"
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
OPENPILOT_DIR="$( cd "$SCRIPT_DIR/../.." >/dev/null && pwd )"
export BASEDIR="${BASEDIR:-$OPENPILOT_DIR}"
export PYTHONPATH="${BASEDIR}${PYTHONPATH:+:$PYTHONPATH}"

python3 -c "from openpilot.tools.sim.lib.preap_params import configure_preap_sim; configure_preap_sim()"

cd "$BASEDIR/system/manager" && exec ./manager.py
