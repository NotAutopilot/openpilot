#!/usr/bin/env python3
"""
Restore stock Tesla Pre-AP EPAS firmware image.

Thin wrapper around flash_epas.py so ScriptRunner can launch restore mode
without passing command-line flags.
"""

from scripts.nap.flash_epas import main


if __name__ == "__main__":
  raise SystemExit(main(["--restore"]))
