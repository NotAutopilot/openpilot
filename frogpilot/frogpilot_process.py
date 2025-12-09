#!/usr/bin/env python3
import datetime
import json
import time

from cereal import messaging
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, Ratekeeper, config_realtime_process
from openpilot.common.time_helpers import system_time_valid

from openpilot.frogpilot.common.frogpilot_utilities import ThreadManager, is_url_pingable
from openpilot.frogpilot.controls.frogpilot_planner import FrogPilotPlanner
from openpilot.frogpilot.system.frogpilot_stats import send_stats
from openpilot.frogpilot.system.frogpilot_tracking import FrogPilotTracking

ASSET_CHECK_RATE = (1 / DT_MDL)

def check_assets(thread_manager, params_memory):

def transition_offroad(frogpilot_planner, thread_manager, time_validated, sm, params):
  params.put("LastGPSPosition", json.dumps(frogpilot_planner.gps_position))

  if time_validated:
    thread_manager.run_with_lock(send_stats, (params))

def transition_onroad():

def update_checks(now, thread_manager, params, params_memory, boot_run=False):
  while not (is_url_pingable("https://github.com") or is_url_pingable("https://gitlab.com")):
    time.sleep(60)

  time.sleep(1)

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

  thread_manager = ThreadManager()

  run_update_checks = False
  started_previously = False
  time_validated = False

  while True:
    sm.update()

    now = datetime.datetime.now(datetime.timezone.utc)

    started = sm["deviceState"].started

    if not started and started_previously:
      transition_offroad(frogpilot_planner, thread_manager, time_validated, sm, params)

      run_update_checks = True
    elif started and not started_previously:
      frogpilot_planner = FrogPilotPlanner()
      frogpilot_tracking = FrogPilotTracking(frogpilot_planner, frogpilot_toggles)

      transition_onroad()

    if started and sm.updated["modelV2"]:
      frogpilot_planner.update(now, time_validated, sm)
      frogpilot_planner.publish(sm, pm)

      frogpilot_tracking.update(now, time_validated, sm)
    elif not started:
      frogpilot_plan_send = messaging.new_message("frogpilotPlan")
      pm.send("frogpilotPlan", frogpilot_plan_send)

    started_previously = started

    if rate_keeper.frame % ASSET_CHECK_RATE == 0:
      check_assets(thread_manager, params_memory)

    run_update_checks |= now.second == 0 and (now.minute % 60 == 0)
    run_update_checks &= time_validated

    if run_update_checks:
      thread_manager.run_with_lock(update_checks, (now, thread_manager, params, params_memory))

      run_update_checks = False
    elif not time_validated:
      time_validated = system_time_valid()
      if not time_validated:
        continue

      thread_manager.run_with_lock(send_stats, (params))
      thread_manager.run_with_lock(update_checks, (now, thread_manager, params, params_memory, True))

    rate_keeper.keep_time()

def main():
  frogpilot_thread()

if __name__ == "__main__":
  main()
