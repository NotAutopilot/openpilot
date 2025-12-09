#!/usr/bin/env python3
import random
import string
import threading
import time

from pathlib import Path

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.time_helpers import system_time_valid
from openpilot.system.hardware import HARDWARE

from openpilot.frogpilot.common.frogpilot_utilities import run_cmd
from openpilot.frogpilot.common.frogpilot_variables import (
  FrogPilotVariables
)


def frogpilot_boot_functions(params):
  params_memory = Params(memory=True)

  FrogPilotVariables()

  def boot_thread():
    while not system_time_valid():
      print("Waiting for system time to become valid...")
      time.sleep(1)

  threading.Thread(target=boot_thread, daemon=True).start()


def install_frogpilot(params):
  paths = [
  ]
  for path in paths:
    path.mkdir(parents=True, exist_ok=True)

  if params.get("FrogPilotDongleId") is None:
    params.put("FrogPilotDongleId", "".join(random.choices(string.ascii_lowercase + string.digits, k=16)))

  update_boot_logo(frogpilot=True)


def uninstall_frogpilot():
  update_boot_logo(stock=True)

  HARDWARE.uninstall()


def update_boot_logo(frogpilot=False, stock=False):
  boot_logo_location = Path("/usr/comma/bg.jpg")

  if frogpilot:
    target_logo = Path(BASEDIR) / "frogpilot/assets/other_images/frogpilot_boot_logo.jpg"
  elif stock:
    target_logo = Path(BASEDIR) / "frogpilot/assets/other_images/stock_bg.jpg"
  else:
    print(f'Error: Must specify either "frogpilot=True" or "stock=True"')
    return

  if not target_logo.is_file():
    print(f"Error: Target logo file not found at {target_logo}")
    return

  if boot_logo_location.read_bytes() != target_logo.read_bytes():
    mount_options = run_cmd(["findmnt", "-n", "-o", "OPTIONS", "/"], "Successfully retrieved mount options", "Failed to retrieve mount options")
    run_cmd(["sudo", "mount", "-o", "remount,rw", "/"], "Successfully remounted / as read-write", "Failed to remount /")
    run_cmd(["sudo", "cp", target_logo, boot_logo_location], "Successfully replaced boot logo", "Failed to replace boot logo")
    run_cmd(["sudo", "mount", "-o", f"remount,{mount_options}", "/"], "Successfully restored / mount options", "Failed to restore / mount options")
