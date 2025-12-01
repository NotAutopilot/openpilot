#!/usr/bin/env python3
import datetime
import json
import time

from cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, Ratekeeper, config_realtime_process
from openpilot.common.time_helpers import system_time_valid

from openpilot.frogpilot.common.frogpilot_backups import backup_toggles
from openpilot.frogpilot.common.frogpilot_functions import update_openpilot
from openpilot.frogpilot.common.frogpilot_utilities import ThreadManager, is_url_pingable
from openpilot.frogpilot.common.frogpilot_variables import FrogPilotVariables
from openpilot.frogpilot.controls.frogpilot_planner import FrogPilotPlanner
from openpilot.frogpilot.system.frogpilot_stats import send_stats
from openpilot.frogpilot.system.frogpilot_tracking import FrogPilotTracking

ASSET_CHECK_RATE = (1 / DT_MDL)

def check_assets(thread_manager, params_memory, frogpilot_toggles):

def transition_offroad(frogpilot_planner, thread_manager, time_validated, sm, params, frogpilot_toggles):
  params.put("LastGPSPosition", json.dumps(frogpilot_planner.gps_position))

  if time_validated:
    thread_manager.run_with_lock(send_stats, (params, frogpilot_toggles))

def transition_onroad():

def update_checks(now, thread_manager, params, params_memory, frogpilot_toggles, boot_run=False):
  while not (is_url_pingable("https://github.com") or is_url_pingable("https://gitlab.com")):
    time.sleep(60)

  if frogpilot_toggles.automatic_updates:
    thread_manager.run_with_lock(update_openpilot, (thread_manager, params))

  time.sleep(1)

def update_toggles(frogpilot_variables, started, thread_manager, time_validated, params):
  frogpilot_variables.update(started)
  frogpilot_toggles = frogpilot_variables.frogpilot_toggles

  if time_validated:
    thread_manager.run_with_lock(backup_toggles, (params))

  return frogpilot_toggles

def frogpilot_thread():
  rate_keeper = Ratekeeper(1 / DT_MDL, None)

  config_realtime_process(5, Priority.CTRL_LOW)

  pm = messaging.PubMaster(["frogpilotPlan"])
  sm = messaging.SubMaster(["carControl", "carState", "controlsState", "deviceState", "driverMonitoringState",
                            "gpsLocation", "gpsLocationExternal", "liveParameters", "managerState", "modelV2",
                            "onroadEvents", "pandaStates", "radarState", "selfdriveState", "frogpilotCarState",
                            "frogpilotSelfdriveState", "frogpilotModelV2", "frogpilotOnroadEvents"],
                            poll="modelV2")

  params = Params(return_defaults=True)
  params_memory = Params(memory=True)

  frogpilot_variables = FrogPilotVariables()
  thread_manager = ThreadManager()

  frogpilot_toggles = frogpilot_variables.frogpilot_toggles

  run_update_checks = False
  started_previously = False
  time_validated = False

  while True:
    sm.update()

    now = datetime.datetime.now(datetime.timezone.utc)

    started = sm["deviceState"].started

    if not started and started_previously:
      frogpilot_toggles = update_toggles(frogpilot_variables, started, thread_manager, time_validated, params)
      transition_offroad(frogpilot_planner, thread_manager, time_validated, sm, params, frogpilot_toggles)

      run_update_checks = True
    elif started and not started_previously:
      frogpilot_planner = FrogPilotPlanner()
      frogpilot_tracking = FrogPilotTracking(frogpilot_planner, frogpilot_toggles)

      transition_onroad()

    if started and sm.updated["modelV2"]:
      frogpilot_planner.update(now, time_validated, sm, frogpilot_toggles)
      frogpilot_planner.publish(sm, pm, frogpilot_toggles)

      frogpilot_tracking.update(now, time_validated, sm, frogpilot_toggles)
    elif not started:
      frogpilot_plan_send = messaging.new_message("frogpilotPlan")
      frogpilot_plan_send.frogpilotPlan.frogpilotToggles = json.dumps(vars(frogpilot_toggles))
      pm.send("frogpilotPlan", frogpilot_plan_send)

    started_previously = started

    if rate_keeper.frame % ASSET_CHECK_RATE == 0:
      check_assets(thread_manager, params_memory, frogpilot_toggles)

    if params_memory.get_bool("FrogPilotTogglesUpdated"):
      frogpilot_toggles = update_toggles(frogpilot_variables, started, thread_manager, time_validated, params)

    run_update_checks |= params_memory.get_bool("ManualUpdateInitiated")
    run_update_checks |= now.second == 0 and (now.minute % 60 == 0 or (now.minute % 5 == 0 and frogpilot_variables.frogs_go_moo))
    run_update_checks &= time_validated

    if run_update_checks:
      thread_manager.run_with_lock(update_checks, (now, thread_manager, params, params_memory, frogpilot_toggles))

      run_update_checks = False
    elif not time_validated:
      time_validated = system_time_valid()
      if not time_validated:
        continue

      thread_manager.run_with_lock(backup_toggles, (params, True))
      thread_manager.run_with_lock(send_stats, (params, frogpilot_toggles))
      thread_manager.run_with_lock(update_checks, (now, thread_manager, params, params_memory, frogpilot_toggles, True))

    rate_keeper.keep_time()

def main():
  frogpilot_thread()

if __name__ == "__main__":
  main()
