from multiprocessing import Queue

from openpilot.tools.sim.bridge.common import SimulatorBridge
from openpilot.tools.sim.lib.plant_world import PlantWorld


class PlantBridge(SimulatorBridge):
  def spawn_world(self, q: Queue):
    return PlantWorld(self.dual_camera)
