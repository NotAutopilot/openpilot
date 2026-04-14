#!/usr/bin/env python3
"""
Extract stock Tesla Pre-AP EPAS firmware image only.

Thin wrapper around flash_epas.py so ScriptRunner can launch extraction mode
without passing command-line flags.
"""

from scripts.nap.flash_epas import main


if __name__ == "__main__":
  raise SystemExit(main(["--extract-only"]))
