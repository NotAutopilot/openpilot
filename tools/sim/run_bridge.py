#!/usr/bin/env python3
import argparse
import os
import sys

from typing import Any
from multiprocessing import Queue


def create_bridge(dual_camera, high_quality, plant=False):
  queue: Any = Queue()

  if not plant:
    try:
      if os.environ.get("METADRIVE_CAMERAS") == "1":
        from openpilot.tools.sim.bridge.metadrive.metadrive_process import ensure_display
        ensure_display()
      from openpilot.tools.sim.bridge.metadrive.metadrive_bridge import MetaDriveBridge
      simulator_bridge = MetaDriveBridge(dual_camera, high_quality)
    except ImportError:
      plant = True

  if plant:
    from openpilot.tools.sim.bridge.plant_bridge import PlantBridge
    simulator_bridge = PlantBridge(dual_camera, high_quality)

  simulator_process = simulator_bridge.run(queue)
  return queue, simulator_process, simulator_bridge

def main():
  _, simulator_process, _ = create_bridge(True, False)
  simulator_process.join()

def parse_args(add_args=None):
  parser = argparse.ArgumentParser(description='Bridge between the simulator and openpilot.')
  parser.add_argument('--joystick', action='store_true')
  parser.add_argument('--high_quality', action='store_true')
  parser.add_argument('--dual_camera', action='store_true')
  parser.add_argument('--plant', action='store_true',
                      help='Pre-AP CAN plant without MetaDrive (selfdrived bring-up)')
  parser.add_argument('--no-keyboard', action='store_true',
                      help='Do not take the terminal; wait on the bridge process')

  return parser.parse_args(add_args)

if __name__ == "__main__":
  args = parse_args()

  queue, simulator_process, simulator_bridge = create_bridge(args.dual_camera, args.high_quality, plant=args.plant)

  if args.joystick:
    from openpilot.tools.sim.lib.manual_ctrl import wheel_poll_thread
    wheel_poll_thread(queue)
  elif args.no_keyboard or not sys.stdin.isatty():
    simulator_process.join()
  else:
    from openpilot.tools.sim.lib.keyboard_ctrl import keyboard_poll_thread
    keyboard_poll_thread(queue)

  simulator_bridge.shutdown()
  simulator_process.join()
