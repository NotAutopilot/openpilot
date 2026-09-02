"""World that packs Pre-AP CAN without MetaDrive or a GPU.

Used when metadrive is not installed (this VPS, CI) so launch_openpilot.sh
+ run_bridge.py can still bring up selfdrived.
"""
from openpilot.tools.sim.lib.common import SimulatorState, World, vec3

CRUISE_SPEED_MS = 20.0


class PlantWorld(World):
  def __init__(self, dual_camera=False):
    super().__init__(dual_camera)
    self.speed_ms = CRUISE_SPEED_MS
    self.steering_angle = 0.0

  def apply_controls(self, steer_sim, throttle_out, brake_out):
    self.steering_angle = float(steer_sim)
    if throttle_out:
      self.speed_ms = min(40.0, self.speed_ms + 0.05 * float(throttle_out))
    if brake_out:
      self.speed_ms = max(0.0, self.speed_ms - 0.1 * float(brake_out))

  def tick(self):
    # Unblock SimulatedSensors.send_camera_images (black frames).
    self.image_lock.release()

  def read_state(self):
    pass

  def read_sensors(self, state: SimulatorState):
    state.velocity = vec3(self.speed_ms, 0.0, 0.0)
    state.steering_angle = self.steering_angle
    state.bearing = 0.0
    state.gps.from_xy((0.0, 0.0))
    state.valid = True

  def read_cameras(self):
    pass

  def close(self, reason: str):
    self.exit_event.set()
    try:
      self.image_lock.release()
    except ValueError:
      pass

  def reset(self):
    self.speed_ms = CRUISE_SPEED_MS
    self.steering_angle = 0.0
