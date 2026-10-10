from typing import Any

import pytest

from openpilot.cereal import messaging
from opendbc.car import gen_empty_fingerprint
from opendbc.car.car_helpers import interfaces
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.preap.sp.radar_interface import RadarInterface as PreAPRadarInterface
from opendbc.car.tesla.values import CAR
from opendbc.sunnypilot.car.tesla.values import TeslaFlagsSP
from openpilot.selfdrive.controls.radard import RADAR_MEASUREMENT_TIMEOUT, RADAR_TO_CAMERA, RadarD


# Verified lead_to_silence_reuse fields from the private route proof.
# The private route fixture itself is not read by this public test.
ROUTE_RADARD_PROOF = {
  "coverage_kind": "retirement_only",
  "slot_reuse_acceptance": "blocked_no_post_silence_fused_radar_lead",
}

# Minimal verified numeric vectors from the same window: contemporaneous
# carState/modelV2 plus the first nonempty radarInterface cycle that fused.
ROUTE_LIFECYCLE_EVENTS: list[dict[str, Any]] = [
  {
    "window": "lead_to_silence_reuse",
    "rel_ns": 580018953926,
    "type": "carState",
    "carState": {"vEgo": 23.465662002563477},
  },
  {
    "window": "lead_to_silence_reuse",
    "rel_ns": 580021045522,
    "type": "modelV2",
    "modelV2": {
      "velocity": {"x": [
        22.963895797729492, 22.973785400390625, 22.95918846130371, 22.95679473876953,
        22.951433181762695, 22.938968658447266, 22.947612762451172, 22.941211700439453,
        22.950061798095703, 22.943572998046875, 22.967647552490234, 22.979249954223633,
        23.002952575683594, 23.02733039855957, 23.059120178222656, 23.075864791870117,
        23.097864151000977, 23.142148971557617, 23.177152633666992, 23.191314697265625,
        23.205516815185547, 23.226856231689453, 23.244415283203125, 23.26291275024414,
        23.259672164916992, 23.263410568237305, 23.275007247924805, 23.276805877685547,
        23.269046783447266, 23.258718490600586, 23.2353515625, 23.23967170715332,
        23.22171401977539,
      ]},
      "leadsV3": [
        {
          "prob": 0.9971914887428284,
          "x": [49.005393981933594, 94.78558349609375, 140.82968139648438, 186.9625244140625, 232.89833068847656, 278.8450622558594],
          "xStd": [2.2456231117248535, 2.7717936038970947, 4.023665904998779, 5.787389278411865, 7.847999095916748, 10.253961563110352],
          "y": [-0.05704158544540405, -1.105865716934204, -6.773068904876709, -1.2280083894729614, -3.9621055126190186, 5.680412769317627],
          "yStd": [0.2611179053783417, 4.048710823059082, 0.18303829431533813, 0.2759804427623749, 0.31099560856819153, 5.960740089416504],
          "v": [23.0461368560791, 23.00564956665039, 23.0158748626709, 23.040847778320312, 23.020801544189453, 23.074792861938477],
          "vStd": [0.7168713212013245, 0.8361306190490723, 0.9884262681007385, 1.1433857679367065, 1.3105676174163818, 1.4741801023483276],
          "a": [-0.010735484771430492, -0.0010877299355342984, 0.01420368067920208, 0.02312212437391281, 0.025280984118580818, 0.021487433463335037],
        },
        {
          "prob": 0.998005211353302,
          "x": [48.92068862915039, 95.05860900878906, 140.8334197998047, 186.67922973632812, 232.91998291015625, 279.2420349121094],
          "xStd": [2.2957000732421875, 2.7931711673736572, 4.026915550231934, 5.756524562835693, 7.875762939453125, 10.273011207580566],
          "y": [-0.05547323450446129, -6.60338830947876, 0.0681857168674469, 4.851256370544434, 2.0368893146514893, 2.309739112854004],
          "yStd": [0.2662146985530853, 0.2564559578895569, 0.29523220658302307, 0.00027661010972224176, 0.008618931286036968, 0.050472378730773926],
          "v": [23.073501586914062, 23.014354705810547, 22.993310928344727, 23.032100677490234, 23.052854537963867, 23.11680030822754],
          "vStd": [0.7339189052581787, 0.8453420996665955, 0.990349292755127, 1.1518993377685547, 1.3188618421554565, 1.47184157371521],
          "a": [-0.009970172308385372, -0.0012622429057955742, 0.013552956283092499, 0.022676169872283936, 0.024889102205634117, 0.022328007966279984],
        },
        {
          "prob": 0.9958066940307617,
          "x": [48.96788024902344, 94.9964828491211, 140.9610595703125, 186.86744689941406, 232.8910369873047, 279.17584228515625],
          "xStd": [2.443636178970337, 2.916327476501465, 4.154557704925537, 5.883960247039795, 7.94464111328125, 10.307344436645508],
          "y": [-0.06017473340034485, -0.31537696719169617, 2.9058327674865723, 1.5139975547790527, -0.9226375222206116, -1.0760862827301025],
          "yStd": [0.27871614694595337, 0.6938527226448059, 0.05035999044775963, 4.301486015319824, 0.46176743507385254, 0.01931501179933548],
          "v": [23.038835525512695, 23.023040771484375, 22.975631713867188, 23.032320022583008, 23.07149887084961, 23.123825073242188],
          "vStd": [0.753650426864624, 0.8533411026000977, 0.9971219301223755, 1.1522274017333984, 1.3250501155853271, 1.485109806060791],
          "a": [-0.009704401716589928, -0.0018602388445287943, 0.01385495439171791, 0.022614378482103348, 0.02495618537068367, 0.021333495154976845],
        },
      ],
    },
  },
  {
    "window": "lead_to_silence_reuse",
    "rel_ns": 580025238295,
    "type": "radarInterface",
    "radarInterface": {"points": [
      {"trackId": 0, "dRel": 46.375, "yRel": 0.875, "vRel": -0.1875},
    ]},
  },
]


BOSCH_POINT_A_ADDRESS = 0x310
BOSCH_POINT_B_ADDRESS = 0x311
BOSCH_STATUS_ADDRESS = 0x301
BOSCH_TRIGGER_ADDRESS = 0x36E


class FakeRadarParser:
  def __init__(self):
    self.can_valid = True
    self.updated_addresses: set[int] = set()
    self.vl: dict[str, dict[str, Any]] = {
      "TeslaRadarSguInfo": {
        "RADC_HWFail": 0,
        "RADC_SGUFail": 0,
        "RADC_SensorDirty": 0,
        "RADC_SGUInfoConsistBit": 0,
      },
    }
    for slot in range(32):
      self.vl[f"RadarPoint{slot}_A"] = {
        "Index": 0, "Tracked": False, "LongDist": 0.0, "LatDist": 0.0,
        "LongSpeed": 0.0, "LongAccel": 0.0, "ProbExist": 0.0, "Meas": 0,
      }
      self.vl[f"RadarPoint{slot}_B"] = {"Index2": 0, "LatSpeed": 0.0}

  def update(self, _can_msgs):
    return self.updated_addresses


def _cp_pair():
  CP = CarInterface.get_params(CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_S_PREAP, gen_empty_fingerprint(), [], False, False, False)
  return CP, CP_SP


def _bosch_interface():
  CP, CP_SP = _cp_pair()
  radar = interfaces[CAR.TESLA_MODEL_S_PREAP].RadarInterface(CP, CP_SP)
  assert type(radar) is PreAPRadarInterface
  radar.radar_off_can = False
  radar.rcp = FakeRadarParser()
  return radar


def _run_raw(radar, *, tracked=True, d_rel=30.0, v_rel=0.0, y_rel=0.0):
  radar.rcp.vl["RadarPoint0_A"] = {
    "Index": 0, "Tracked": tracked, "LongDist": d_rel, "LatDist": y_rel,
    "LongSpeed": v_rel, "LongAccel": 0.0, "ProbExist": 90.0, "Meas": 1,
  }
  radar.rcp.vl["RadarPoint0_B"] = {"Index2": 0, "LatSpeed": 0.0}
  radar.rcp.updated_addresses = {BOSCH_TRIGGER_ADDRESS, BOSCH_STATUS_ADDRESS, BOSCH_POINT_A_ADDRESS, BOSCH_POINT_B_ADDRESS}
  return radar.update([])


class RadarHarness:
  def __init__(self, v_ego=20.0):
    services = ["modelV2", "carState", "liveTracks"]
    self.sm = messaging.SubMaster(services, ignore_alive=services, ignore_avg_freq=services)
    CP, CP_SP = _cp_pair()
    CP_SP.flags |= int(TeslaFlagsSP.PREAP_RADAR_PRESENT)
    self.radar = RadarD(CP, CP_SP)
    self.v_ego = v_ego
    self.frame = 0

  def step(self, time_s, vision_d_rel, points, vision_v_rel=0.0):
    messages = [self._model(time_s, vision_d_rel, vision_v_rel)]
    if self.frame == 0:
      car_state = messaging.new_message("carState", valid=True)
      car_state.logMonoTime = int(time_s * 1e9)
      car_state.carState.vEgo = self.v_ego
      messages.append(car_state)
    if points is not None:
      live = messaging.new_message("liveTracks", valid=True)
      live.logMonoTime = int(time_s * 1e9)
      packed = live.liveTracks.init("points", len(points))
      for slot, point in zip(packed, points, strict=True):
        slot.trackId = int(point.trackId)
        slot.dRel = float(point.dRel)
        slot.yRel = float(point.yRel)
        slot.vRel = float(point.vRel)
        slot.measured = bool(point.measured)
      messages.append(live)
    self.sm.update_msgs(time_s, [message.as_reader() for message in messages])
    self.radar.update(self.sm, self.sm["liveTracks"])
    self.frame += 1
    return self.radar.radar_state.leadOne

  def _model(self, time_s, vision_d_rel, vision_v_rel=0.0):
    message = messaging.new_message("modelV2", valid=True)
    message.logMonoTime = int(time_s * 1e9)
    message.modelV2.velocity.x = [self.v_ego]
    leads = message.modelV2.init("leadsV3", 2)
    for lead in leads:
      lead.prob = 0.9
      lead.x = [vision_d_rel + RADAR_TO_CAMERA]
      lead.xStd = [3.0]
      lead.y = [0.0]
      lead.yStd = [1.0]
      lead.v = [self.v_ego + vision_v_rel]
      lead.vStd = [2.0]
      lead.a = [0.0]
    return message


class TestPreAPRadarInterfaceToRadard:
  def test_slot_reuse_does_not_inherit_incumbent(self):
    raw = _bosch_interface()
    harness = RadarHarness()
    first_raw = _run_raw(raw, d_rel=30.0, v_rel=0.0)
    first = harness.step(1.0, 30.0, first_raw.points)
    assert first.radar
    first_id = first.radarTrackId

    reused_raw = _run_raw(raw, d_rel=41.0, v_rel=11.0)
    reused = harness.step(1.1, 41.0, reused_raw.points, vision_v_rel=11.0)
    assert reused.radar
    assert reused.radarTrackId != first_id
    assert reused_raw.points[0].trackId != first_raw.points[0].trackId
    assert reused.radarTrackId == reused_raw.points[0].trackId

  def test_silence_clears_fused_lead_within_timeout(self):
    raw = _bosch_interface()
    harness = RadarHarness()
    first_raw = _run_raw(raw, d_rel=30.0)
    lead = harness.step(1.0, 30.0, first_raw.points)
    assert lead.radar

    model_dt = 0.05
    expiration_frame = int(RADAR_MEASUREMENT_TIMEOUT / model_dt) + 1
    for frame in range(1, expiration_frame):
      lead = harness.step(1.0 + frame * model_dt, 36.0, None)
    assert lead.radar
    lead = harness.step(1.0 + expiration_frame * model_dt, 36.0, None)
    assert not lead.radar
    assert lead.present
    assert lead.dRel == pytest.approx(36.0)


class TestRouteDerivedReplay:
  def test_route_fixture_is_retirement_only_and_is_not_claimed(self):
    proof = ROUTE_RADARD_PROOF
    coverage = proof.get("coverage_kind") or proof.get("execution", {}).get("coverage_kind")
    assert coverage == "retirement_only"
    assert proof["slot_reuse_acceptance"] != "proved_non_inheritance"

    services = ["modelV2", "carState", "liveTracks"]
    sm = messaging.SubMaster(services, ignore_alive=services, ignore_avg_freq=services)
    CP, CP_SP = _cp_pair()
    CP_SP.flags |= int(TeslaFlagsSP.PREAP_RADAR_PRESENT)
    radar = RadarD(CP, CP_SP)
    fused_ids = []
    saw_radar = False
    for event in ROUTE_LIFECYCLE_EVENTS:
      if event.get("window") != "lead_to_silence_reuse":
        continue
      messages = []
      time_s = event["rel_ns"] * 1e-9
      if event["type"] == "modelV2":
        message = messaging.new_message("modelV2", valid=True)
        message.logMonoTime = event["rel_ns"]
        message.modelV2.velocity.x = list(event["modelV2"]["velocity"]["x"])
        leads = message.modelV2.init("leadsV3", len(event["modelV2"]["leadsV3"]))
        for dst, src in zip(leads, event["modelV2"]["leadsV3"], strict=True):
          dst.prob = src["prob"]
          dst.x = list(src["x"])
          dst.xStd = list(src["xStd"])
          dst.y = list(src["y"])
          dst.yStd = list(src["yStd"])
          dst.v = list(src["v"])
          dst.vStd = list(src["vStd"])
          dst.a = list(src["a"])
        messages.append(message)
      elif event["type"] == "carState":
        message = messaging.new_message("carState", valid=True)
        message.logMonoTime = event["rel_ns"]
        message.carState.vEgo = event["carState"]["vEgo"]
        messages.append(message)
      elif event["type"] == "radarInterface":
        message = messaging.new_message("liveTracks", valid=True)
        message.logMonoTime = event["rel_ns"]
        points = event["radarInterface"]["points"]
        packed = message.liveTracks.init("points", len(points))
        for dst, src in zip(packed, points, strict=True):
          dst.trackId = src["trackId"]
          dst.dRel = src["dRel"]
          dst.yRel = src["yRel"]
          dst.vRel = src["vRel"]
          dst.measured = True
        messages.append(message)
      if not messages:
        continue
      sm.update_msgs(time_s, [message.as_reader() for message in messages])
      radar.update(sm, sm["liveTracks"])
      if radar.radar_state is None:
        continue
      lead = radar.radar_state.leadOne
      if lead.radar:
        saw_radar = True
        fused_ids.append(lead.radarTrackId)

    assert saw_radar
    # Route evidence still lacks a post-silence fused association to a new raw ID.
    assert coverage != "retirement_and_slot_reuse"
