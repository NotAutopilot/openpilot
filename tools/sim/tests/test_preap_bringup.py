"""Bring up selfdrived with launch_openpilot.sh + plant bridge. No MetaDrive."""
import os
import subprocess
import time

from cereal import messaging
from openpilot.common.basedir import BASEDIR
from openpilot.tools.sim.lib.preap_params import PREAP_FINGERPRINT, PREAP_VIN
from openpilot.tools.sim.run_bridge import create_bridge

SIM_DIR = os.path.join(BASEDIR, "tools/sim")
LAUNCH = os.path.join(SIM_DIR, "launch_openpilot.sh")
PLANT = os.path.join(SIM_DIR, "lib/plant_world.py")
BRINGUP_S = 45


def test_launch_brings_up_selfdrived():
  assert os.path.isfile(LAUNCH), (
    f"launch_openpilot.sh missing; BASEDIR={BASEDIR!r} cwd={os.getcwd()!r}"
  )
  assert os.path.isfile(PLANT), (
    f"plant_world.py missing; BASEDIR={BASEDIR!r} cwd={os.getcwd()!r}"
  )
  env = os.environ.copy()
  env["CI"] = "1"
  env["BASEDIR"] = BASEDIR
  env["PYTHONPATH"] = BASEDIR + ((":" + env["PYTHONPATH"]) if env.get("PYTHONPATH") else "")
  manager = subprocess.Popen(["./launch_openpilot.sh"], cwd=SIM_DIR, env=env)
  queue = None
  bridge = None
  proc = None
  try:
    queue, proc, bridge = create_bridge(False, False, plant=True)

    deadline = time.monotonic() + BRINGUP_S
    while not bridge.started.value and time.monotonic() < deadline:
      time.sleep(0.1)
    assert proc.exitcode is None
    assert bridge.started.value, "plant bridge did not start"

    sm = messaging.SubMaster(["managerState", "carParams", "selfdriveState", "deviceState"])
    selfdrived_running = False
    fingerprint = ""
    vin = ""
    started = False
    running: dict[str, bool] = {}
    while time.monotonic() < deadline:
      sm.update(100)
      if sm.seen["deviceState"]:
        started = sm["deviceState"].started
      if sm.seen["managerState"]:
        running = {p.name: p.running for p in sm["managerState"].processes}
        selfdrived_running = bool(running.get("selfdrived"))
      if sm.seen["carParams"]:
        fingerprint = sm["carParams"].carFingerprint
        vin = sm["carParams"].carVin
      if selfdrived_running and fingerprint == PREAP_FINGERPRINT:
        break
      time.sleep(0.1)

    assert started, f"deviceState.started is false; running={running}"
    assert selfdrived_running, f"selfdrived did not start; running={running}"
    assert fingerprint == PREAP_FINGERPRINT, fingerprint
    assert vin == PREAP_VIN, vin
  finally:
    if bridge is not None:
      bridge.shutdown()
    if proc is not None:
      proc.join(timeout=5)
      if proc.is_alive():
        proc.terminate()
    manager.terminate()
    try:
      manager.wait(timeout=8)
    except subprocess.TimeoutExpired:
      manager.kill()
    if queue is not None:
      queue.close()
