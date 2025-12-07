#!/usr/bin/env python3
import random
import string
import threading
import time

from pathlib import Path

from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.time_helpers import system_time_valid
from openpilot.system.athena.registration import register
from openpilot.system.hardware import HARDWARE

from openpilot.frogpilot.assets.theme_manager import ThemeManager
from openpilot.frogpilot.common.frogpilot_backups import backup_frogpilot
from openpilot.frogpilot.common.frogpilot_utilities import is_FrogsGoMoo, run_cmd, use_konik_server
from openpilot.frogpilot.common.frogpilot_variables import (
  ERROR_LOGS_PATH, FROGS_GO_MOO_PATH, HD_LOGS_PATH, KONIK_LOGS_PATH, THEME_SAVE_PATH,
  FrogPilotVariables, get_frogpilot_toggles
)


def frogpilot_boot_functions(build_metadata, params):
  params_memory = Params(memory=True)

  FrogPilotVariables()
  ThemeManager(params, params_memory, boot_run=True).update_active_theme(time_validated=system_time_valid(), frogpilot_toggles=get_frogpilot_toggles(), boot_run=True)

  if use_konik_server():
    if params.get("KonikDongleId") is not None:
      params.put("DongleId", params.get("KonikDongleId"))
    else:
      params.put("KonikDongleId", register(show_spinner=True, register_konik=True))
      params.put("DongleId", params.get("KonikDongleId"))
  elif params.get("DongleId") == params.get("KonikDongleId"):
    params.put("DongleId", params.get("StockDongleId"))

  def boot_thread():
    while not system_time_valid():
      print("Waiting for system time to become valid...")
      time.sleep(1)

    backup_frogpilot(build_metadata, params)

  threading.Thread(target=boot_thread, daemon=True).start()


def install_frogpilot(build_metadata, params):
  paths = [
    ERROR_LOGS_PATH,
    HD_LOGS_PATH,
    KONIK_LOGS_PATH,
    THEME_SAVE_PATH
  ]
  for path in paths:
    path.mkdir(parents=True, exist_ok=True)

  if params.get("FrogPilotDongleId") is None:
    params.put("FrogPilotDongleId", "".join(random.choices(string.ascii_lowercase + string.digits, k=16)))

  update_boot_logo(frogpilot=True)

  if build_metadata.channel == "FrogPilot-Development" and is_FrogsGoMoo():
    mount_options = run_cmd(["findmnt", "-n", "-o", "OPTIONS", "/persist"], "Successfully retrieved mount options", "Failed to retrieve mount options")
    run_cmd(["sudo", "mount", "-o", "remount,rw", "/persist"], "Successfully remounted /persist as read-write", "Failed to remount /persist")
    run_cmd(["sudo", "python3", FROGS_GO_MOO_PATH], "Successfully ran frogsgomoo.py", "Failed to run frogsgomoo.py")
    run_cmd(["sudo", "mount", "-o", f"remount,{mount_options}", "/persist"], "Successfully restored /persist mount options", "Failed to restore /persist mount options")


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


def update_openpilot(thread_manager, params):
  def update_available():
    run_cmd(["pkill", "-SIGUSR1", "-f", "system.updated.updated"], "Checking for updates...", "Failed to check for update...", report=False)

    while params.get("UpdaterState") != "checking...":
      time.sleep(1)

    while params.get("UpdaterState") == "checking...":
      time.sleep(1)

    if not params.get_bool("UpdaterFetchAvailable"):
      return False

    while params.get_bool("IsOnroad") or thread_manager.is_thread_alive("lock_doors"):
      time.sleep(60)

    run_cmd(["pkill", "-SIGHUP", "-f", "system.updated.updated"], "Update available, downloading...", "Failed to download update...", report=False)

    while not params.get_bool("UpdateAvailable"):
      time.sleep(60)

    return True

  if params.get("UpdaterState") != "idle":
    return

  while params.get_bool("IsOnroad") or thread_manager.is_thread_alive("lock_doors"):
    time.sleep(60)

  if not update_available():
    return

  while True:
    if not update_available():
      break

  HARDWARE.reboot()
