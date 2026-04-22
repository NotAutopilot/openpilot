#!/usr/bin/env python3
import dataclasses
import json
import requests
import threading
import time

from pathlib import Path

from cereal import messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.time_helpers import system_time_valid
from openpilot.system.athena.registration import register
from openpilot.system.hardware import HARDWARE

from openpilot.frogpilot.assets.theme_manager import ThemeManager
from openpilot.frogpilot.common.frogpilot_backups import backup_frogpilot
from openpilot.frogpilot.common.frogpilot_utilities import get_frogpilot_api_info, is_FrogsGoMoo, is_url_pingable, run_cmd, use_konik_server
from openpilot.frogpilot.common.frogpilot_variables import (
  ERROR_LOGS_PATH, FROGPILOT_API, FROGS_GO_MOO_PATH, HD_LOGS_PATH, KONIK_LOGS_PATH, MAPS_PATH, THEME_SAVE_PATH,
  FrogPilotVariables, get_frogpilot_toggles
)


def capture_report(discord_user, report, params, frogpilot_toggles):
  if not is_url_pingable(FROGPILOT_API):
    return

  api_token, build_metadata, device_type, dongle_id = get_frogpilot_api_info()

  error_file_path = ERROR_LOGS_PATH / "error.txt"
  error_content = "No error log found."
  if error_file_path.exists():
    error_content = error_file_path.read_text()[:1000]

  payload = {
    "api_token": api_token,
    "build_metadata": build_metadata,
    "device": device_type,
    "discord_user": discord_user,
    "error_content": error_content,
    "frogpilot_dongle_id": dongle_id,
    "frogpilot_toggles": frogpilot_toggles,
    "report": report,
  }

  try:
    response = requests.post(f"{FROGPILOT_API}/discord/report", json=payload, headers={"Content-Type": "application/json", "User-Agent": "frogpilot-api/1.0"}, timeout=30)
    response.raise_for_status()
    print("Successfully sent error report!")
  except requests.exceptions.RequestException as exception:
    print(f"Error sending report: {exception}")


def frogpilot_boot_functions(build_metadata, params):
  params_memory = Params(memory=True)

  maps_selected = params.get("MapsSelected")
  if maps_selected:
    try:
      data = json.loads(maps_selected)
      if isinstance(data, dict):
        new_items = []
        for nation in data.get("nations", []):
          new_items.append(f"nation.{nation}")
        for state in data.get("states", []):
          new_items.append(f"us_state.{state}")
        new_items.sort()
        params.put("MapsSelected", ",".join(new_items))
    except (json.JSONDecodeError, TypeError, ValueError):
      pass

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

  register_device(build_metadata, params)

  update_boot_logo(frogpilot=True)

  if build_metadata.channel == "FrogPilot-Development" and is_FrogsGoMoo():
    mount_options = run_cmd(["findmnt", "-n", "-o", "OPTIONS", "/persist"], "Successfully retrieved mount options", "Failed to retrieve mount options")
    run_cmd(["sudo", "mount", "-o", "remount,rw", "/persist"], "Successfully remounted /persist as read-write", "Failed to remount /persist")
    run_cmd(["sudo", "python3", FROGS_GO_MOO_PATH], "Successfully ran frogsgomoo.py", "Failed to run frogsgomoo.py")
    run_cmd(["sudo", "mount", "-o", f"remount,{mount_options}", "/persist"], "Successfully restored /persist mount options", "Failed to restore /persist mount options")


def register_device(build_metadata, params):
  def register_thread():
    while not is_url_pingable(FROGPILOT_API):
      time.sleep(60)

    payload = {
      "api_token": params.get("FrogPilotApiToken"),
      "build_metadata": dataclasses.asdict(build_metadata),
      "device": HARDWARE.get_device_type(),
      "dongle_id": params.get("DongleId"),
      "frogpilot_dongle_id": params.get("FrogPilotDongleId"),
    }

    try:
      response = requests.post(f"{FROGPILOT_API}/register", json=payload, headers={"Content-Type": "application/json", "User-Agent": "frogpilot-api/1.0"}, timeout=10)
      response.raise_for_status()

      data = response.json()
      params.put("FrogPilotApiToken", data.get("api_token", ""))
      params.put("FrogPilotDongleId", data.get("frogpilot_dongle_id"))
    except Exception:
      pass

  threading.Thread(target=register_thread, daemon=True).start()


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


def update_maps(now, params, params_memory, manual_update=False):
  maps_selected = params.get("MapsSelected")
  if not maps_selected:
    return

  day = now.day
  is_first = day == 1
  is_sunday = now.weekday() == 6
  schedule = params.get("PreferredSchedule")

  maps_downloaded = MAPS_PATH.exists()
  if maps_downloaded and (schedule == 0 or (schedule == 1 and not is_sunday) or (schedule == 2 and not is_first)) and not manual_update:
    return

  suffix = "th" if 11 <= day <= 13 else {1: "st", 2: "nd", 3: "rd"}.get(day % 10, "th")
  todays_date = now.strftime(f"%B {day}{suffix}, %Y")

  if maps_downloaded and params.get("LastMapsUpdate") == todays_date and not manual_update:
    return

  pm = messaging.PubMaster(["mapdIn"])
  sm = messaging.SubMaster(["mapdExtendedOut"])

  time.sleep(1)

  msg = messaging.new_message("mapdIn")
  msg.mapdIn.type = 0
  msg.mapdIn.str = maps_selected
  pm.send("mapdIn", msg)

  started = False
  while True:
    sm.update(1000)

    if params_memory.get_bool("CancelDownloadMaps"):
      msg = messaging.new_message("mapdIn")
      msg.mapdIn.type = 27
      pm.send("mapdIn", msg)

      params_memory.remove("CancelDownloadMaps")
      params_memory.remove("DownloadMaps")
      return

    if sm.updated["mapdExtendedOut"]:
      progress = sm["mapdExtendedOut"].downloadProgress

      if progress.active:
        started = True

      if not progress.active and started:
        break

  params.put("LastMapsUpdate", todays_date)
  params_memory.remove("DownloadMaps")


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
