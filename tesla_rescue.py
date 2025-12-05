#!/usr/bin/env python3
"""
Tesla Pre-AP Rescue Script
===========================
Emergency script to disable the buggy pedal code path and restore system stability.

The crash was caused by `carcontroller.py` attempting to send a CAN message
with an `int` payload instead of `bytes` when `use_pedal=True`.

This script:
1. Disables pedal flags in /data/tinkla_params.json
2. Restarts system services to apply the fix

Usage:
  python tesla_rescue.py
"""

import json
import os
import sys
import subprocess
import tempfile

CONFIG_FILE = "/data/tinkla_params.json"

BANNER = """
╔══════════════════════════════════════════════════════════════════╗
║                    🚨 TESLA RESCUE SCRIPT 🚨                      ║
╠══════════════════════════════════════════════════════════════════╣
║                                                                  ║
║  This script will:                                               ║
║    1. Disable pedal in config (use_pedal = false)                ║
║    2. Mark pedal as uncalibrated (pedal_calibrated = false)      ║
║    3. Restart system services                                    ║
║                                                                  ║
║  This should fix the crash in carcontroller/card.py:             ║
║    TypeError: 'int' object is not iterable                       ║
║                                                                  ║
╚══════════════════════════════════════════════════════════════════╝
"""


def load_config():
    """Load existing config or return empty dict."""
    try:
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, 'r') as f:
                return json.load(f)
    except Exception as e:
        print(f"  Warning: Could not load config: {e}")
    return {}


def save_config(config):
    """Atomically save config to JSON file."""
    try:
        os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
        
        # Write to temp file first
        fd, tmp_path = tempfile.mkstemp(
            dir=os.path.dirname(CONFIG_FILE),
            prefix='.tinkla_params_rescue_',
            suffix='.tmp'
        )
        try:
            with os.fdopen(fd, 'w') as f:
                json.dump(config, f, indent=2)
            # Atomic rename
            os.replace(tmp_path, CONFIG_FILE)
            return True
        except Exception:
            try:
                os.unlink(tmp_path)
            except Exception:
                pass
            raise
    except Exception as e:
        print(f"  Error saving config: {e}")
        return False


def restart_services():
    """Kill OpenPilot services to force restart with new config."""
    print("\n  Restarting services...")
    
    services_to_kill = [
        "controlsd",
        "card",
        "boardd",
        "pandad",
    ]
    
    for service in services_to_kill:
        try:
            subprocess.run(
                ["pkill", "-f", service],
                capture_output=True,
                timeout=5
            )
            print(f"    Killed {service}")
        except Exception:
            pass
    
    # Nuclear option: kill all openpilot python processes
    try:
        subprocess.run(
            ["pkill", "-f", "selfdrive"],
            capture_output=True,
            timeout=5
        )
        print("    Killed selfdrive processes")
    except Exception:
        pass
    
    print("  ✓ Services will restart automatically")


def main():
    print(BANNER)
    
    # Step 1: Load current config
    print("Step 1: Loading config...")
    config = load_config()
    print(f"  Current config: {CONFIG_FILE}")
    print(f"  use_pedal: {config.get('use_pedal', 'not set')}")
    print(f"  pedal_calibrated: {config.get('pedal_calibrated', 'not set')}")
    
    # Step 2: Disable pedal
    print("\nStep 2: Disabling pedal...")
    config['use_pedal'] = False
    config['pedal_calibrated'] = False
    print("  use_pedal = False")
    print("  pedal_calibrated = False")
    
    # Step 3: Save config
    print("\nStep 3: Saving config...")
    if save_config(config):
        print(f"  ✓ Saved to {CONFIG_FILE}")
    else:
        print("  ❌ Failed to save config!")
        return 1
    
    # Step 4: Verify
    print("\nStep 4: Verifying...")
    verify_config = load_config()
    if verify_config.get('use_pedal') == False and verify_config.get('pedal_calibrated') == False:
        print("  ✓ Config verified")
    else:
        print("  ⚠️  Config verification failed - please check manually")
    
    # Step 5: Restart services
    print("\nStep 5: Restarting services...")
    restart_services()
    
    # Done
    print("\n" + "=" * 60)
    print("✅ RESCUE COMPLETE")
    print("=" * 60)
    print("""
The pedal has been disabled. OpenPilot should now start without crashing.

To re-enable the pedal after fixing the carcontroller bug:
  1. Fix the bug in carcontroller.py (int vs bytes issue)
  2. Run: python tesla_setup.py
  3. Enable pedal option

Or manually edit /data/tinkla_params.json:
  "use_pedal": true,
  "pedal_calibrated": true
""")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

