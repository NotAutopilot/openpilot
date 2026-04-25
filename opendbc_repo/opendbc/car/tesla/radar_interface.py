from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.tesla.values import DBC, CANBUS, CAR
from opendbc.car.interfaces import RadarInterfaceBase

# Optional NAP config import (available on device/runtime)
try:
  from opendbc.car.tesla.preap.nap_conf import nap_conf
except ImportError:
  nap_conf = None


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CP = CP

    self.continental_radar = CP.carFingerprint in (CAR.TESLA_MODEL_S_HW3, )
    self.bosch_radar = CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, CAR.TESLA_MODEL_S_HW2, CAR.TESLA_MODEL_S_PREAP)

    messages = []
    if self.continental_radar:
      messages.append(('RadarStatus', 16))
      self.num_points = 40
      self.trigger_msg = 1119
      self.radar_point_frq = 16
    elif self.bosch_radar:
      messages.append(('TeslaRadarSguInfo', 8))

      self.num_points = 32
      self.trigger_msg = 878
      self.radar_point_frq = 8

    if self.bosch_radar or self.continental_radar:
      for i in range(self.num_points):
        messages.extend([
          (f'RadarPoint{i}_A', self.radar_point_frq),
          (f'RadarPoint{i}_B', self.radar_point_frq),
        ])

    self.radar_off_can = CP.radarUnavailable
    if not  CP.radarUnavailable:
      self.rcp = CANParser(DBC[CP.carFingerprint][Bus.radar], messages, CANBUS.radar)
    else:
      self.rcp = None
    print(f"[NAP] RadarInterface: radarUnavailable={CP.radarUnavailable}, radar_off_can={self.radar_off_can}, rcp={'active' if self.rcp else 'None'}")

    self.updated_messages = set()
    self.track_id = 0
    # Keep parity with Tinkla radar lateral alignment behavior.
    # For behind-nosecone installs, users can configure horizontal offset in meters.
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_PREAP and nap_conf is not None:
      self.radar_offset = float(nap_conf.radar_offset)
    else:
      self.radar_offset = 0.0

  def update(self, can_msgs):

    if self.radar_off_can or (self.rcp is None):
      return super().update(None)

    values = self.rcp.update(can_msgs)
    self.updated_messages.update(values)

    if self.trigger_msg not in self.updated_messages:
      return None

    ret = structs.RadarData()

    if self.rcp is None:
      return ret

    # Errors
    if not self.rcp.can_valid:
      ret.errors.canError = True

    ret.errors.radarFault = False
    ret.errors.radarUnavailableTemporary = False
    if self.continental_radar:
      radar_status = self.rcp.vl['RadarStatus']
      if radar_status['shortTermUnavailable']:
        ret.errors.radarUnavailableTemporary = True
      if radar_status['sensorBlocked'] or radar_status['vehDynamicsError']:
        ret.errors.radarFault = True
    elif self.bosch_radar:
      radar_status = self.rcp.vl['TeslaRadarSguInfo']
      if radar_status['RADC_HWFail']:
        ret.errors.radarFault = True

    # Radar tracks
    for i in range(self.num_points):
      msg_a = self.rcp.vl[f'RadarPoint{i}_A']
      msg_b = self.rcp.vl[f'RadarPoint{i}_B']

      # Make sure msg A and B are together
      if msg_a['Index'] != msg_b['Index2']:
        continue

      # Check if it's a valid track
      if not msg_a['Tracked']:
        if i in self.pts:
          del self.pts[i]
        continue

      # Check if it's a valid point
      if self.bosch_radar and (msg_a["LongDist"] > 250.0 or msg_a["LongDist"] <= 0 or msg_a["ProbExist"] < 50.0):
        if i in self.pts:
          del self.pts[i]
        continue

      # New track!
      if i not in self.pts:
        self.pts[i] = structs.RadarData.RadarPoint()
        self.pts[i].trackId = self.track_id
        self.track_id += 1

      # Parse track data
      self.pts[i].dRel = msg_a['LongDist']
      self.pts[i].yRel = msg_a['LatDist'] + self.radar_offset
      self.pts[i].vRel = msg_a['LongSpeed']
      self.pts[i].aRel = msg_a['LongAccel']
      self.pts[i].yvRel = msg_b['LatSpeed']
      self.pts[i].measured = bool(msg_a['Meas'])

    ret.points = list(self.pts.values())
    self.updated_messages.clear()
    return ret
