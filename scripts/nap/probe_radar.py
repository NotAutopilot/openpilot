#!/usr/bin/env python3
"""
Tesla Pre-AP Radar Probe (GUI entry point)

Read-only reachability check for the Bosch radar's diagnostic channel. Sends
nothing but TesterPresent and writes nothing to the radar.

This is a thin wrapper around `vin_learn_radar --probe`: the ScriptRunner
launches scripts with `python -m <module>` and has no way to pass arguments,
so the probe needs its own module to be reachable from a settings button.
"""

import sys

from scripts.nap.vin_learn_radar import main

if __name__ == "__main__":
  sys.exit(main(["--probe"]))
