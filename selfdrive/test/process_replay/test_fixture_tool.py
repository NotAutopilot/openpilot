from __future__ import annotations

from typing import Any
import hashlib
import json
import os
from pathlib import Path

import pytest
import zstandard
from cereal import car, log as capnp_log
from openpilot.tools.lib.logreader import LogReader, save_log

from openpilot.selfdrive.test.process_replay import fixture_tool


POLICY = Path(__file__).with_name("nap_fixture_policy.json")
PRIVATE_VIN = "1HGBH41JXMN109186"


def gas_sensor(counter: int) -> tuple[int, bytes, int]:
  data = bytearray((1, 0xD6, 0, 0xFA, counter & 0x0F, 0))
  data[5] = (sum(data[:5]) + 0x52 + 0x05) & 0xFF
  return 0x552, bytes(data), 2


def _set_xyz(msg, x=0.0, y=0.0, z=0.0):
  msg.x, msg.y, msg.z = x, y, z
  msg.xStd = msg.yStd = msg.zStd = 0.0


def make_car_params(*, pedal: bool = True, vin: str = PRIVATE_VIN, fw_secret: str | bytes | None = None) -> Any:
  event = capnp_log.Event.new_message()
  event.valid = True
  event.logMonoTime = 1
  cp = event.init("carParams")
  cp.brand = "tesla"
  cp.carFingerprint = "TESLA_MODEL_S_PREAP"
  cp.carVin = vin
  cp.pcmCruise = not pedal
  cp.openpilotLongitudinalControl = pedal
  if fw_secret is not None:
    fw = cp.init("carFw", 1)[0]
    fw.ecu = car.CarParams.Ecu.unknown
    fw.fwVersion = fw_secret if isinstance(fw_secret, (bytes, bytearray)) else fw_secret.encode()
    fw.address = 0x7E0
  safety = cp.init("safetyConfigs", 1)[0]
  safety.safetyModel = car.CarParams.SafetyModel.teslaPreap
  safety.safetyParam = 7 if pedal else 6
  return event


def compact_stream(*, pedal: bool = True, private_vin: str = PRIVATE_VIN,
                   embed_private_text: bool = False, embed_private_data: bool = False,
                   split_vin_frames: bool = False, split_vin_events: bool = False,
                   interleaved_vin_frames: bool = False, undeclared_field: bool = False,
                   nonzero_vned: bool = False, wrong_safety_model: bool = False,
                   include_forbidden: bool = False, invalid_model: bool = False,
                   fw_secret: str | bytes | None = None,
                   duplicate_fw_in_can: bool = False) -> bytes:
  events = []
  mono = 10

  def stamp(event):
    nonlocal mono
    event.valid = True
    event.logMonoTime = mono
    mono += 10
    return event

  params = make_car_params(pedal=pedal, vin=private_vin, fw_secret=fw_secret)
  if wrong_safety_model:
    params.carParams.safetyConfigs[0].safetyModel = car.CarParams.SafetyModel.tesla
  events.append(stamp(params))

  can = capnp_log.Event.new_message()
  frame_count = 10 if pedal else 1
  extras = 0
  if split_vin_frames:
    extras += 2
  if interleaved_vin_frames:
    extras += 3
  if embed_private_data:
    extras += 1
  if duplicate_fw_in_can and fw_secret is not None:
    extras += 1
  frames = can.init("can", frame_count + extras)
  if pedal:
    for index in range(10):
      address, data, bus = gas_sensor(index)
      frames[index].address, frames[index].dat, frames[index].src = address, data, bus
  else:
    frames[0].address, frames[0].dat, frames[0].src = 0x100, b"\x01\x02\x03\x04", 0
  cursor = frame_count
  if split_vin_frames:
    frames[cursor].address, frames[cursor].dat, frames[cursor].src = 0x100, private_vin[:3].encode(), 0
    frames[cursor + 1].address, frames[cursor + 1].dat, frames[cursor + 1].src = 0x101, private_vin[3:6].encode(), 0
    cursor += 2
  if interleaved_vin_frames:
    frames[cursor].address, frames[cursor].dat, frames[cursor].src = 0x100, private_vin[:3].encode(), 0
    frames[cursor + 1].address, frames[cursor + 1].dat, frames[cursor + 1].src = 0x110, b"XX", 0
    frames[cursor + 2].address, frames[cursor + 2].dat, frames[cursor + 2].src = 0x101, private_vin[3:6].encode(), 0
    cursor += 3
  if embed_private_data:
    frames[cursor].address, frames[cursor].dat, frames[cursor].src = 0x123, b"SECRETDATA_TOKEN", 0
    cursor += 1
  if duplicate_fw_in_can and fw_secret is not None:
    fw_bytes = fw_secret if isinstance(fw_secret, (bytes, bytearray)) else fw_secret.encode()
    frames[cursor].address, frames[cursor].dat, frames[cursor].src = 0x321, bytes(fw_bytes), 0
    cursor += 1
  events.append(stamp(can))

  if split_vin_events:
    first = capnp_log.Event.new_message()
    f = first.init("can", 1)[0]
    f.address, f.dat, f.src = 0x200, private_vin[:3].encode(), 1
    events.append(stamp(first))
    second = capnp_log.Event.new_message()
    f = second.init("can", 1)[0]
    f.address, f.dat, f.src = 0x201, private_vin[3:6].encode(), 1
    events.append(stamp(second))

  # required services
  cs = stamp(capnp_log.Event.new_message())
  state = cs.init("carState")
  state.vEgo = 20.0
  state.aEgo = 0.1
  state.vCruise = 30.0
  state.standstill = False
  state.steeringAngleDeg = 1.0
  state.canValid = True
  state.cruiseState.enabled = True
  state.leftBlindspot = True
  state.rightBlindspot = False
  state.teslaCCEngaged = False
  state.teslaCCDisengaged = False
  state.teslaCCNotArmed = False
  state.pedalAuthorityFailed = False
  events.append(cs)

  cc = stamp(capnp_log.Event.new_message())
  control = cc.init("carControl")
  control.enabled = True
  control.latActive = True
  control.longActive = pedal
  control.actuators.steeringAngleDeg = 1.0
  control.actuators.accel = 0.2 if pedal else 0.0
  control.orientationNED = [0.0, 0.01, 0.0]
  control.leftBlinker = True
  control.rightBlinker = False
  events.append(cc)

  co = stamp(capnp_log.Event.new_message())
  output = co.init("carOutput")
  output.actuatorsOutput.steeringAngleDeg = 1.0
  output.actuatorsOutput.torque = 0.1
  events.append(co)

  controls = stamp(capnp_log.Event.new_message())
  cst = controls.init("controlsState")
  cst.desiredCurvature = 0.001
  cst.forceDecel = False
  pid = cst.lateralControlState.init("pidState")
  pid.saturated = False
  pid.active = True
  events.append(controls)

  sds = stamp(capnp_log.Event.new_message())
  ss = sds.init("selfdriveState")
  ss.enabled = True
  ss.active = True
  ss.engageable = True
  ss.state = capnp_log.SelfdriveState.OpenpilotState.enabled
  if embed_private_text:
    ss.alertText1 = "leak-token-value"
  events.append(sds)

  model = stamp(capnp_log.Event.new_message())
  if invalid_model:
    model.valid = False
  mv = model.init("modelV2")
  mv.frameId = 1
  mv.frameDropPerc = 0.0
  mv.position.x = [1.0, 2.0]
  mv.velocity.x = [20.0, 20.0]
  mv.acceleration.x = [0.1, 0.1]
  mv.laneLineProbs = [0.9, 0.8, 0.7, 0.6]
  lines = mv.init("laneLines", 4)
  for line in lines:
    line.y = [0.1, 0.2]
  leads = mv.init("leadsV3", 2)
  for lead in leads:
    lead.prob = 0.5
    lead.x = [10.0]
    lead.xStd = [1.0]
    lead.y = [0.0]
    lead.yStd = [0.1]
    lead.v = [15.0]
    lead.vStd = [0.5]
    lead.a = [0.0]
  mv.meta.laneChangeState = 0
  mv.meta.hardBrakePredicted = False
  mv.meta.desirePrediction = [0.0] * 8
  mv.meta.disengagePredictions.gasPressProbs = [0.0, 0.01]
  mv.action.desiredCurvature = 0.001
  mv.action.desiredAcceleration = 0.1
  mv.action.shouldStop = False
  events.append(model)

  pose = stamp(capnp_log.Event.new_message())
  lp = pose.init("livePose")
  lp.posenetOK = True
  lp.inputsOK = True
  _set_xyz(lp.orientationNED)
  _set_xyz(lp.velocityDevice, x=20.0)
  _set_xyz(lp.accelerationDevice)
  _set_xyz(lp.angularVelocityDevice)
  lp.angularVelocityDevice.valid = True
  events.append(pose)

  calib = stamp(capnp_log.Event.new_message())
  lc = calib.init("liveCalibration")
  lc.calStatus = capnp_log.LiveCalibrationData.Status.calibrated
  lc.rpyCalib = [0.0, 0.0, 0.0]
  events.append(calib)

  live_params = stamp(capnp_log.Event.new_message())
  lpd = live_params.init("liveParameters")
  lpd.valid = True
  lpd.angleOffsetDeg = 0.1
  lpd.steerRatio = 15.0
  lpd.stiffnessFactor = 1.0
  lpd.roll = 0.0
  events.append(live_params)

  torque = stamp(capnp_log.Event.new_message())
  lt = torque.init("liveTorqueParameters")
  lt.useParams = False
  events.append(torque)

  delay = stamp(capnp_log.Event.new_message())
  delay.init("liveDelay").lateralDelay = 0.05
  events.append(delay)

  tracks = stamp(capnp_log.Event.new_message())
  ltks = tracks.init("liveTracks")
  pts = ltks.init("points", 1)
  pts[0].trackId = 1
  pts[0].dRel = 20.0
  pts[0].yRel = 0.0
  pts[0].vRel = -1.0
  pts[0].measured = True
  events.append(tracks)

  radar = stamp(capnp_log.Event.new_message())
  rs = radar.init("radarState")
  rs.leadOne.status = True
  rs.leadOne.dRel = 20.0
  rs.leadOne.vLead = 19.0
  rs.leadOne.aLeadK = 0.0
  rs.leadOne.aLeadTau = 1.5
  rs.leadOne.modelProb = 0.8
  rs.leadTwo.status = False
  events.append(radar)

  plan = stamp(capnp_log.Event.new_message())
  lpplan = plan.init("longitudinalPlan")
  lpplan.aTarget = 0.1
  lpplan.shouldStop = False
  lpplan.hasLead = True
  events.append(plan)

  panda = stamp(capnp_log.Event.new_message())
  ps = panda.init("pandaStates", 1)[0]
  ps.pandaType = capnp_log.PandaState.PandaType.dos
  ps.controlsAllowed = True
  ps.safetyModel = car.CarParams.SafetyModel.teslaPreap
  ps.safetyParam = 7 if pedal else 6
  ps.safetyRxChecksInvalid = False
  events.append(panda)

  peri = stamp(capnp_log.Event.new_message())
  peri.init("peripheralState").pandaType = capnp_log.PandaState.PandaType.dos
  events.append(peri)

  device = stamp(capnp_log.Event.new_message())
  ds = device.init("deviceState")
  ds.freeSpacePercent = 50.0
  ds.memoryUsagePercent = 20
  ds.started = True
  if undeclared_field:
    ds.networkInfo.operator = "SECRET-OP"
  events.append(device)

  onroad = stamp(capnp_log.Event.new_message())
  ev = onroad.init("onroadEvents", 1)[0]
  ev.name = capnp_log.OnroadEvent.EventName.buttonEnable
  events.append(onroad)

  dms = stamp(capnp_log.Event.new_message())
  dms.init("driverMonitoringState").awarenessStatus = 1.0
  events.append(dms)

  assist = stamp(capnp_log.Event.new_message())
  assist.init("driverAssistance")
  events.append(assist)

  acc = stamp(capnp_log.Event.new_message())
  acc.init("accelerometer")
  events.append(acc)

  gyro = stamp(capnp_log.Event.new_message())
  gyro.init("gyroscope")
  events.append(gyro)

  gps = stamp(capnp_log.Event.new_message())
  loc = gps.init("gpsLocation")
  if nonzero_vned:
    loc.vNED = [1.0, 2.0, 3.0]
  events.append(gps)

  if include_forbidden:
    forbidden = stamp(capnp_log.Event.new_message())
    forbidden.init("initData")
    events.append(forbidden)

  return b"".join(event.to_bytes() for event in events)


# fix typing import

def read_messages(raw: bytes):
  return list(LogReader.from_bytes(raw))


def deny_token(label: str, value: str | bytes, encoding: str = "utf8") -> dict:
  if encoding == "utf8":
    encoded = value if isinstance(value, str) else value.decode("utf-8")
  elif encoding == "hex":
    raw_bytes = value if isinstance(value, (bytes, bytearray)) else value.encode("utf-8")
    encoded = bytes(raw_bytes).hex()
  elif encoding == "base64":
    import base64
    raw_bytes = value if isinstance(value, (bytes, bytearray)) else value.encode("utf-8")
    encoded = base64.b64encode(bytes(raw_bytes)).decode("ascii")
  else:
    raise AssertionError(encoding)
  return {"label": label, "encoding": encoding, "value": encoded}


def write_deny(path: Path, raw: bytes, tokens: list[dict] | None = None, sha: str | None = None,
               sanitized_output_sha256: str | None = None) -> Path:
  payload = {
    "raw_input_sha256": sha if sha is not None else hashlib.sha256(raw).hexdigest(),
    "tokens": tokens if tokens is not None else [deny_token("vin", PRIVATE_VIN)],
  }
  if sanitized_output_sha256 is not None:
    payload["sanitized_output_sha256"] = sanitized_output_sha256
  path.write_text(json.dumps(payload), encoding="utf-8")
  return path


def run_sanitize(tmp_path: Path, raw: bytes, deny: dict | None = None, case: str = "nap-preap-pedal-v1"):
  source = tmp_path / "source.bin"
  output = tmp_path / "output.zst"
  report = tmp_path / "report.json"
  source.write_bytes(raw)
  deny_path = None
  if deny is None:
    deny_path = write_deny(tmp_path / "deny.json", raw)
  else:
    deny_path = tmp_path / "deny.json"
    deny_path.write_text(json.dumps(deny), encoding="utf-8")
  status = fixture_tool.process(str(source), str(output), str(report), str(POLICY), case, str(deny_path), True)
  return output.read_bytes() if output.exists() else None, json.loads(report.read_text()), status


def test_sanitize_is_byte_deterministic_and_scrubs_vin_and_fw(tmp_path: Path):
  raw = compact_stream()
  first, report, status = run_sanitize(tmp_path, raw)
  second, _, _ = run_sanitize(tmp_path, raw)
  assert first == second
  assert status["scope"] == "private"
  assert status["private_token_scan"]["performed"] is True
  messages = list(LogReader.from_bytes(zstandard.ZstdDecompressor().decompress(first)))
  params = next(message.carParams for message in messages if message.which() == "carParams")
  assert params.carVin == "00000000000000000"
  assert len(params.carFw) == 0
  assert fixture_tool.safety_model_ordinal(params.safetyConfigs[0].safetyModel) == int(car.CarParams.SafetyModel.teslaPreap)
  assert report["can_evidence"]["valid_count"] == 10
  assert "token_matches" not in status
  assert "raw_input_sha256" not in status
  assert "raw_input_sha256" not in report
  assert report["sanitized_output_sha256"] == hashlib.sha256(first).hexdigest()
  assert params.safetyConfigs[0].safetyParam == 7


def test_nested_list_fields_are_copied(tmp_path: Path):
  raw = compact_stream()
  output, _, _ = run_sanitize(tmp_path, raw)
  messages = list(LogReader.from_bytes(zstandard.ZstdDecompressor().decompress(output)))
  model = next(message.modelV2 for message in messages if message.which() == "modelV2")
  assert list(model.velocity.x) == [20.0, 20.0]
  assert len(model.leadsV3) == 2
  assert list(model.leadsV3[0].x) == [10.0]
  tracks = next(message.liveTracks for message in messages if message.which() == "liveTracks")
  assert tracks.points[0].dRel == 20.0


def test_required_service_rejection(tmp_path: Path):
  messages = [event for event in read_messages(compact_stream()) if event.which() != "modelV2"]
  with pytest.raises(fixture_tool.FixtureError, match="required valid service"):
    fixture_tool.structural_checks(messages, fixture_tool.parse_policy(str(POLICY)), "nap-preap-pedal-v1")


def test_sanitize_skips_forbidden_and_unknown_source_services(tmp_path: Path):
  raw = compact_stream(include_forbidden=True)
  # also append an undeclared-but-not-forbidden service via raw event if possible: thumbnail is forbidden
  output, report, _ = run_sanitize(tmp_path, raw)
  assert output is not None
  assert "initData" not in report["message_counts"]
  assert "initData" in report["source_message_counts"]


@pytest.mark.parametrize(
  "mutate",
  (
    lambda p: p.__setitem__("schema_version", 99),
    lambda p: p.__setitem__("placeholder_vin", "11111111111111111"),
    lambda p: p["allowed_services"].__setitem__("sendcan", ["[].address"]),
    lambda p: p["forbidden_services"].remove("clocks"),
    lambda p: p["cases"].__setitem__("nap-preap-pedal-v1", {"mode": "weird", "fingerprint": "TESLA_MODEL_S_PREAP"}),
    lambda p: p.__setitem__("extra", 1),
  ),
)
def test_invalid_policy_classes_are_rejected(tmp_path: Path, mutate):
  policy = json.loads(POLICY.read_text())
  mutate(policy)
  path = tmp_path / "bad_policy.json"
  path.write_text(json.dumps(policy), encoding="utf-8")
  with pytest.raises(fixture_tool.FixtureError):
    fixture_tool.parse_policy(str(path))


def test_undeclared_service_rejected_on_validate(tmp_path: Path):
  raw = compact_stream()
  output, _, _ = run_sanitize(tmp_path, raw)
  messages = list(LogReader.from_bytes(zstandard.ZstdDecompressor().decompress(output)))
  unexpected = capnp_log.Event.new_message()
  unexpected.valid = True
  unexpected.init("sendcan", 1)
  path = tmp_path / "tampered.zst"
  save_log(str(path), messages + [unexpected.as_reader()])
  report = tmp_path / "validate.json"
  with pytest.raises(fixture_tool.FixtureError, match="undeclared service"):
    fixture_tool.process(str(path), None, str(report), str(POLICY), "nap-preap-pedal-v1", None, False)


def test_undeclared_field_rejected(tmp_path: Path):
  raw = compact_stream(undeclared_field=True)
  # sanitize strips networkInfo because not allowed; validate against unsanitized tampered output
  messages = read_messages(raw)
  # force a sanitized-looking stream that still has networkInfo by manually building
  policy = fixture_tool.parse_policy(str(POLICY))
  built = [fixture_tool.build_event(event, policy) for event in messages]
  built = [event for event in built if event is not None]
  # inject undeclared field into deviceState after sanitize rebuild
  events = []
  for event in built:
    if event.which() != "deviceState":
      events.append(event)
      continue
    msg = event.as_builder()
    msg.deviceState.networkInfo.operator = "SECRET-OP"
    events.append(msg.as_reader())
  path = tmp_path / "field.zst"
  save_log(str(path), events)
  with pytest.raises(fixture_tool.FixtureError, match="undeclared populated field"):
    fixture_tool.process(str(path), None, str(tmp_path / "report.json"), str(POLICY), "nap-preap-pedal-v1", None, False)


def test_nonzero_vned_rejected(tmp_path: Path):
  raw = compact_stream(nonzero_vned=True)
  policy = fixture_tool.parse_policy(str(POLICY))
  messages = []
  for event in read_messages(raw):
    built = fixture_tool.build_event(event, policy)
    if built is None:
      continue
    if built.which() == "gpsLocation":
      msg = built.as_builder()
      msg.gpsLocation.vNED = [1.0, 0.0, 0.0]
      messages.append(msg.as_reader())
    else:
      messages.append(built)
  path = tmp_path / "vned.zst"
  save_log(str(path), messages)
  with pytest.raises(fixture_tool.FixtureError, match="sensitive field is not zero"):
    fixture_tool.process(str(path), None, str(tmp_path / "report.json"), str(POLICY), "nap-preap-pedal-v1", None, False)


def test_exact_safety_model_required(tmp_path: Path):
  output, _, _ = run_sanitize(tmp_path, compact_stream())
  messages = list(LogReader.from_bytes(zstandard.ZstdDecompressor().decompress(output)))
  rebuilt = []
  for event in messages:
    if event.which() != "carParams":
      rebuilt.append(event)
      continue
    msg = event.as_builder()
    msg.carParams.safetyConfigs[0].safetyModel = car.CarParams.SafetyModel.tesla
    rebuilt.append(msg.as_reader())
  with pytest.raises(fixture_tool.FixtureError, match="safetyModel"):
    fixture_tool.structural_checks(rebuilt, fixture_tool.parse_policy(str(POLICY)), "nap-preap-pedal-v1")


def test_structural_checks_require_tesla_brand():
  messages = read_messages(compact_stream())
  rebuilt = []
  for event in messages:
    if event.which() != "carParams":
      rebuilt.append(event)
      continue
    msg = event.as_builder()
    msg.carParams.carVin = "00000000000000000"
    msg.carParams.carFw = []
    msg.carParams.brand = "private-brand"
    rebuilt.append(msg.as_reader())

  with pytest.raises(fixture_tool.FixtureError, match=r"carParams\.brand must be tesla"):
    fixture_tool.structural_checks(rebuilt, fixture_tool.parse_policy(str(POLICY)), "nap-preap-pedal-v1")

def test_text_token_outside_carparams_is_rejected(tmp_path: Path):
  raw = compact_stream()
  deny = {
    "raw_input_sha256": hashlib.sha256(raw).hexdigest(),
    "tokens": [
      deny_token("vin", PRIVATE_VIN),
      deny_token("retained_public_text", "TESLA_MODEL_S_PREAP"),
    ],
  }
  with pytest.raises(fixture_tool.FixtureError, match="private token"):
    run_sanitize(tmp_path, raw, deny)
  # ensure no final output
  output = tmp_path / "output.zst"
  # run_sanitize may have left nothing; check report exists with matches
  report = json.loads((tmp_path / "report.json").read_text())
  assert report["private_token_scan"]["performed"] is True
  assert report["private_token_scan"]["token_matches"]
  assert not output.exists()


def test_data_token_is_rejected(tmp_path: Path):
  raw = compact_stream(embed_private_data=True)
  deny = {"raw_input_sha256": hashlib.sha256(raw).hexdigest(),
          "tokens": [deny_token("vin", PRIVATE_VIN), deny_token("secret", "SECRETDATA_TOKEN")]}
  with pytest.raises(fixture_tool.FixtureError, match="private token"):
    run_sanitize(tmp_path, raw, deny)
  assert not (tmp_path / "output.zst").exists()
  report = json.loads((tmp_path / "report.json").read_text())
  assert any(row["label"] == "secret" for row in report["private_token_scan"]["token_matches"])


def test_split_vin_across_frames_is_rejected(tmp_path: Path):
  raw = compact_stream(split_vin_frames=True)
  with pytest.raises(fixture_tool.FixtureError, match="private token"):
    run_sanitize(tmp_path, raw)
  assert not (tmp_path / "output.zst").exists()


def test_split_vin_across_events_is_rejected(tmp_path: Path):
  raw = compact_stream(split_vin_events=True)
  with pytest.raises(fixture_tool.FixtureError, match="private token"):
    run_sanitize(tmp_path, raw)
  assert not (tmp_path / "output.zst").exists()


def test_missing_empty_wrong_hash_deny(tmp_path: Path):
  raw = compact_stream()
  source = tmp_path / "source.bin"
  source.write_bytes(raw)
  output = tmp_path / "out.zst"
  report = tmp_path / "report.json"
  with pytest.raises(fixture_tool.FixtureError, match="deny"):
    fixture_tool.process(str(source), str(output), str(report), str(POLICY), "nap-preap-pedal-v1", None, True)
  empty = tmp_path / "empty.json"
  empty.write_text(json.dumps({"raw_input_sha256": hashlib.sha256(raw).hexdigest(), "tokens": []}), encoding="utf-8")
  with pytest.raises(fixture_tool.FixtureError, match="nonempty"):
    fixture_tool.process(str(source), str(output), str(report), str(POLICY), "nap-preap-pedal-v1", str(empty), True)
  wrong = write_deny(tmp_path / "wrong.json", raw, sha="0" * 64)
  with pytest.raises(fixture_tool.FixtureError, match="SHA-256"):
    fixture_tool.process(str(source), str(output), str(report), str(POLICY), "nap-preap-pedal-v1", str(wrong), True)
  # conflicting --input-sha256 must not override deny
  deny = write_deny(tmp_path / "deny.json", raw)
  with pytest.raises(fixture_tool.FixtureError, match="conflicts"):
    fixture_tool.process(str(source), str(output), str(report), str(POLICY), "nap-preap-pedal-v1", str(deny), True, "1" * 64)


def test_public_and_private_validate_scopes(tmp_path: Path):
  raw = compact_stream()
  output, private_report, _ = run_sanitize(tmp_path, raw)
  public_report = tmp_path / "public.json"
  status = fixture_tool.process(str(tmp_path / "output.zst"), None, str(public_report), str(POLICY), "nap-preap-pedal-v1", None, False)
  assert status["scope"] == "public"
  assert status["private_token_scan"]["performed"] is False
  loaded = json.loads(public_report.read_text())
  assert loaded["scope"] == "public"
  assert loaded["private_token_scan"]["performed"] is False
  assert "token_matches" not in loaded
  assert "token_matches" not in loaded.get("private_token_scan", {})

  sanitized_deny = {
    "raw_input_sha256": hashlib.sha256(raw).hexdigest(),
    "sanitized_output_sha256": hashlib.sha256(output).hexdigest(),
    "tokens": [deny_token("vin", PRIVATE_VIN)],
  }
  deny_path = tmp_path / "san_deny.json"
  deny_path.write_text(json.dumps(sanitized_deny), encoding="utf-8")
  private_path = tmp_path / "private.json"
  status = fixture_tool.process(str(tmp_path / "output.zst"), None, str(private_path), str(POLICY), "nap-preap-pedal-v1", str(deny_path), False)
  assert status["scope"] == "private"
  assert status["private_token_scan"]["performed"] is True
  loaded = json.loads(private_path.read_text())
  assert loaded["private_token_scan"]["performed"] is True
  assert loaded["private_token_scan"]["token_matches"]
  assert all(row["count"] == 0 for row in loaded["private_token_scan"]["token_matches"])


def test_symlink_hardlink_and_same_path_collisions(tmp_path: Path):
  raw = compact_stream()
  source = tmp_path / "source.bin"
  source.write_bytes(raw)
  deny = write_deny(tmp_path / "deny.json", raw)
  report = tmp_path / "report.json"
  # same path output/report
  with pytest.raises(fixture_tool.FixtureError, match="distinct|collision"):
    fixture_tool.process(str(source), str(report), str(report), str(POLICY), "nap-preap-pedal-v1", str(deny), True)
  # symlink output
  link = tmp_path / "link.zst"
  target = tmp_path / "target.zst"
  target.write_bytes(b"x")
  link.symlink_to(target)
  with pytest.raises(fixture_tool.FixtureError, match="symlink|alias|collision"):
    fixture_tool.process(str(source), str(link), str(report), str(POLICY), "nap-preap-pedal-v1", str(deny), True)
  # hardlink alias between output and report destinations that already exist as same inode
  hard_a = tmp_path / "hard_a.json"
  hard_b = tmp_path / "hard_b.json"
  hard_a.write_text("{}", encoding="utf-8")
  os.link(hard_a, hard_b)
  with pytest.raises(fixture_tool.FixtureError, match="inode|alias|collision"):
    fixture_tool.process(str(source), str(hard_a), str(hard_b), str(POLICY), "nap-preap-pedal-v1", str(deny), True)


def test_cli_malformed_input_returns_2(tmp_path: Path):
  bad = tmp_path / "bad.bin"
  bad.write_bytes(b"not-a-log")
  deny = write_deny(tmp_path / "deny.json", b"not-a-log", tokens=[deny_token("tokenx", "abcdef")])
  code = fixture_tool.main([
    "sanitize", "--policy", str(POLICY), "--case", "nap-preap-pedal-v1",
    "--input", str(bad), "--output", str(tmp_path / "out.zst"), "--report", str(tmp_path / "r.json"),
    "--deny-token-file", str(deny),
  ])
  assert code == 2


def test_no_pedal_rejects_sensor_frames(tmp_path: Path):
  # Build a no-pedal fixture then inject pedal sensor frames.
  output, _, _ = run_sanitize(tmp_path, compact_stream(pedal=False, private_vin=PRIVATE_VIN), case="nap-preap-no-pedal-v1")
  messages = list(LogReader.from_bytes(zstandard.ZstdDecompressor().decompress(output)))
  can = capnp_log.Event.new_message()
  can.valid = True
  can.logMonoTime = 999
  frame = can.init("can", 1)[0]
  address, data, bus = gas_sensor(0)
  frame.address, frame.dat, frame.src = address, data, bus
  path = tmp_path / "nopedal_tampered.zst"
  save_log(str(path), messages + [can.as_reader()])
  with pytest.raises(fixture_tool.FixtureError, match="no-pedal"):
    fixture_tool.process(str(path), None, str(tmp_path / "nop.json"), str(POLICY), "nap-preap-no-pedal-v1", None, False)


def test_cli_public_validation_returns_failure_for_tampering(tmp_path: Path):
  output, _, _ = run_sanitize(tmp_path, compact_stream())
  path = tmp_path / "fixture.zst"
  path.write_bytes(output[:-1] + bytes((output[-1] ^ 1,)))
  assert fixture_tool.main([
    "validate", "--policy", str(POLICY), "--case", "nap-preap-pedal-v1",
    "--input", str(path), "--report", str(tmp_path / "rep.json"),
  ]) != 0


def test_interleaved_vin_across_frames_is_rejected(tmp_path: Path):
  raw = compact_stream(interleaved_vin_frames=True)
  with pytest.raises(fixture_tool.FixtureError, match="private token"):
    run_sanitize(tmp_path, raw)
  assert not (tmp_path / "output.zst").exists()


def test_deny_requires_exact_source_vin(tmp_path: Path):
  raw = compact_stream()
  source = tmp_path / "source.bin"
  source.write_bytes(raw)
  deny = write_deny(tmp_path / "deny.json", raw, tokens=[deny_token("route", "abcdefgh")])
  with pytest.raises(fixture_tool.FixtureError, match="exactly one vin token"):
    fixture_tool.process(str(source), str(tmp_path / "output.zst"), str(tmp_path / "report.json"), str(POLICY), "nap-preap-pedal-v1", str(deny), True)

  deny = write_deny(tmp_path / "deny2.json", raw, tokens=[deny_token("vin", "1HGBH41JXMN109187")])
  with pytest.raises(fixture_tool.FixtureError, match="does not match source"):
    fixture_tool.process(str(source), str(tmp_path / "output2.zst"), str(tmp_path / "report2.json"), str(POLICY), "nap-preap-pedal-v1", str(deny), True)


def test_private_validate_reuses_raw_bound_deny(tmp_path: Path):
  raw = compact_stream()
  output, report, _ = run_sanitize(tmp_path, raw)
  assert "raw_input_sha256" not in report
  deny = write_deny(
    tmp_path / "bound_deny.json",
    raw,
    sanitized_output_sha256=hashlib.sha256(output).hexdigest(),
  )
  status = fixture_tool.process(
    str(tmp_path / "output.zst"), None, str(tmp_path / "private.json"), str(POLICY),
    "nap-preap-pedal-v1", str(deny), False,
  )
  assert status["scope"] == "private"
  loaded = json.loads((tmp_path / "private.json").read_text())
  assert "raw_input_sha256" not in loaded
  assert all(row["count"] == 0 for row in loaded["private_token_scan"]["token_matches"])


def test_stale_output_removed_on_rejection(tmp_path: Path):
  raw = compact_stream()
  output, _, _ = run_sanitize(tmp_path, raw)
  assert output is not None
  out_path = tmp_path / "output.zst"
  assert out_path.exists()
  # Re-run with VIN leak against the same destination.
  with pytest.raises(fixture_tool.FixtureError, match="private token"):
    run_sanitize(tmp_path, compact_stream(split_vin_frames=True))
  assert not out_path.exists()


def test_required_valid_service_minima(tmp_path: Path):
  messages = read_messages(compact_stream(invalid_model=True))
  with pytest.raises(fixture_tool.FixtureError, match="required valid service"):
    fixture_tool.structural_checks(messages, fixture_tool.parse_policy(str(POLICY)), "nap-preap-pedal-v1")


def test_exact_safety_param_required(tmp_path: Path):
  output, _, _ = run_sanitize(tmp_path, compact_stream())
  messages = list(LogReader.from_bytes(zstandard.ZstdDecompressor().decompress(output)))
  rebuilt = []
  for message in messages:
    if message.which() != "carParams":
      rebuilt.append(message)
      continue
    event = message.as_builder()
    event.carParams.safetyConfigs[0].safetyParam = 1
    rebuilt.append(event.as_reader())
  with pytest.raises(fixture_tool.FixtureError, match="safetyParam"):
    fixture_tool.structural_checks(rebuilt, fixture_tool.parse_policy(str(POLICY)), "nap-preap-pedal-v1")


def test_retained_consumer_fields_survive_sanitize(tmp_path: Path):
  output, _, _ = run_sanitize(tmp_path, compact_stream())
  messages = list(LogReader.from_bytes(zstandard.ZstdDecompressor().decompress(output)))
  state = next(message.carState for message in messages if message.which() == "carState")
  assert state.leftBlindspot is True
  assert state.pedalAuthorityFailed is False
  control = next(message.carControl for message in messages if message.which() == "carControl")
  assert control.leftBlinker is True
  assert control.rightBlinker is False
  panda = next(message.pandaStates for message in messages if message.which() == "pandaStates")
  assert panda[0].safetyRxChecksInvalid is False


def test_accept_six_process_outputs_helper(tmp_path: Path):
  policy = fixture_tool.parse_policy(str(POLICY))
  raw = compact_stream()
  output, _, _ = run_sanitize(tmp_path, raw)
  fixture_messages = list(LogReader.from_bytes(zstandard.ZstdDecompressor().decompress(output)))

  def with_service(service: str, list_size: int | None = None, **fields):
    event = capnp_log.Event.new_message()
    event.valid = True
    event.logMonoTime = 1
    payload = event.init(service) if list_size is None else event.init(service, list_size)
    for key, value in fields.items():
      setattr(payload, key, value)
    return event.as_reader()

  # Synthetic per-process outputs using fixture carParams for card mode evidence.
  car_params = next(message for message in fixture_messages if message.which() == "carParams")
  process_outputs = {
    "card": [
      car_params,
      next(message for message in fixture_messages if message.which() == "carState"),
      next(message for message in fixture_messages if message.which() == "carOutput"),
      next(message for message in fixture_messages if message.which() == "liveTracks"),
      with_service("sendcan", list_size=1),
    ],
    "controlsd": [
      next(message for message in fixture_messages if message.which() == "carControl"),
      next(message for message in fixture_messages if message.which() == "controlsState"),
    ],
    "selfdrived": [
      next(message for message in fixture_messages if message.which() == "selfdriveState"),
      next(message for message in fixture_messages if message.which() == "onroadEvents"),
    ],
    "radard": [next(message for message in fixture_messages if message.which() == "radarState")],
    "plannerd": [
      next(message for message in fixture_messages if message.which() == "longitudinalPlan"),
      next(message for message in fixture_messages if message.which() == "driverAssistance"),
    ],
    "lagd": [next(message for message in fixture_messages if message.which() == "liveDelay")],
  }
  case_params = {
    "NAPPedalEnabled": True,
    "NAPRadarEnabled": True,
    "NAPRadarBehindNosecone": True,
  }
  accepted = fixture_tool.accept_six_process_outputs(
    case="nap-preap-pedal-v1",
    policy=policy,
    process_outputs=process_outputs,
    case_params=case_params,
  )
  assert accepted["status"] == "ok"
  assert accepted["card_mode"] == "pedal"
  assert accepted["card_safety_param"] == 7
  assert set(accepted["processes"]) == set(fixture_tool.SIX_PROCESS_OUTPUTS)

  with pytest.raises(fixture_tool.FixtureError, match="missing process outputs"):
    fixture_tool.accept_six_process_outputs(
      case="nap-preap-pedal-v1",
      policy=policy,
      process_outputs={k: v for k, v in process_outputs.items() if k != "lagd"},
      case_params=case_params,
    )


def test_selfdrived_replay_ignores_stripped_services():
  source = Path(__file__).resolve().parents[2] / "selfdrived" / "selfdrived.py"
  text = source.read_text(encoding="utf-8")
  assert "driverCameraState" in text
  assert "managerState" in text
  # REPLAY ignore list must include both stripped fixture services.
  assert "ignore += ['roadCameraState', 'wideRoadCameraState', 'driverCameraState', 'managerState']" in text


def test_policy_attestation_rejects_weakened_contract(tmp_path: Path):
  policy = json.loads(POLICY.read_text(encoding="utf-8"))
  policy["pedal"]["minimum_frames"] = 0
  policy["pedal"]["minimum_counters"] = 0
  path = tmp_path / "weak.json"
  path.write_text(json.dumps(policy), encoding="utf-8")
  with pytest.raises(fixture_tool.FixtureError, match="attested release contract|pedal evidence minima"):
    fixture_tool.parse_policy(str(path))

  policy = json.loads(POLICY.read_text(encoding="utf-8"))
  policy["allowed_services"]["carState"] = list(policy["allowed_services"]["carState"]) + ["cruiseState.speedOffline"]
  path = tmp_path / "extra_leaf.json"
  path.write_text(json.dumps(policy), encoding="utf-8")
  with pytest.raises(fixture_tool.FixtureError, match="attested release contract"):
    fixture_tool.parse_policy(str(path))


def test_deny_inventory_completeness_requires_scrubbed_secrets(tmp_path: Path):
  raw = compact_stream(fw_secret="SECRET_FW_VALUE")
  source = tmp_path / "source.bin"
  source.write_bytes(raw)
  deny = write_deny(tmp_path / "deny.json", raw)  # vin only; missing fw secret
  with pytest.raises(fixture_tool.FixtureError, match="deny tokens incomplete"):
    fixture_tool.process(str(source), str(tmp_path / "output.zst"), str(tmp_path / "report.json"), str(POLICY), "nap-preap-pedal-v1", str(deny), True)

  deny = write_deny(
    tmp_path / "deny2.json",
    raw,
    tokens=[deny_token("vin", PRIVATE_VIN), deny_token("fw", "SECRET_FW_VALUE")],
  )
  status = fixture_tool.process(str(source), str(tmp_path / "output2.zst"), str(tmp_path / "report2.json"), str(POLICY), "nap-preap-pedal-v1", str(deny), True)
  assert status["status"] == "ok"


def test_deny_inventory_keeps_fw_when_duplicated_in_retained_can(tmp_path: Path):
  # NAP-FIXTURE-001: private fwVersion bytes also present in retained can[].dat must
  # still require an explicit deny token; set-subtraction must not exempt them.
  secret = "SECRET_FW_VALUE"
  raw = compact_stream(fw_secret=secret, duplicate_fw_in_can=True)
  source = tmp_path / "source.bin"
  source.write_bytes(raw)
  out = tmp_path / "output.zst"
  deny = write_deny(tmp_path / "deny.json", raw)  # vin only
  with pytest.raises(fixture_tool.FixtureError, match="deny tokens incomplete"):
    fixture_tool.process(str(source), str(out), str(tmp_path / "report.json"), str(POLICY), "nap-preap-pedal-v1", str(deny), True)
  assert not out.exists()

  deny = write_deny(
    tmp_path / "deny2.json",
    raw,
    tokens=[deny_token("vin", PRIVATE_VIN), deny_token("fw", secret)],
  )
  # Inventory complete, but retained can[].dat still carries the secret — scan rejects.
  with pytest.raises(fixture_tool.FixtureError, match="private token"):
    fixture_tool.process(str(source), str(tmp_path / "output2.zst"), str(tmp_path / "report2.json"), str(POLICY), "nap-preap-pedal-v1", str(deny), True)
  assert not (tmp_path / "output2.zst").exists()

  # Explicit fw deny succeeds when the secret is scrubbed and not retained in CAN.
  clean = compact_stream(fw_secret=secret, duplicate_fw_in_can=False)
  clean_source = tmp_path / "clean.bin"
  clean_source.write_bytes(clean)
  deny = write_deny(
    tmp_path / "deny3.json",
    clean,
    tokens=[deny_token("vin", PRIVATE_VIN), deny_token("fw", secret)],
  )
  status = fixture_tool.process(
    str(clean_source), str(tmp_path / "output3.zst"), str(tmp_path / "report3.json"),
    str(POLICY), "nap-preap-pedal-v1", str(deny), True,
  )
  assert status["status"] == "ok"
  assert (tmp_path / "output3.zst").exists()


def test_binary_deny_token_hex_covers_non_utf8_fw(tmp_path: Path):
  secret = b"BIN\xffFW\x00SECRETS"
  raw = compact_stream(fw_secret=secret)
  source = tmp_path / "source.bin"
  source.write_bytes(raw)
  deny = write_deny(
    tmp_path / "deny.json",
    raw,
    tokens=[deny_token("vin", PRIVATE_VIN), deny_token("fw", secret, encoding="hex")],
  )
  status = fixture_tool.process(str(source), str(tmp_path / "output.zst"), str(tmp_path / "report.json"), str(POLICY), "nap-preap-pedal-v1", str(deny), True)
  assert status["status"] == "ok"

  legacy = {
    "raw_input_sha256": hashlib.sha256(raw).hexdigest(),
    "tokens": [{"label": "vin", "value": PRIVATE_VIN}],
  }
  legacy_path = tmp_path / "legacy.json"
  legacy_path.write_text(json.dumps(legacy), encoding="utf-8")
  with pytest.raises(fixture_tool.FixtureError, match="label, encoding, and value"):
    fixture_tool.load_deny(str(legacy_path))


def test_zstd_trailing_data_is_rejected(tmp_path: Path):
  raw = compact_stream()
  frame = zstandard.ZstdCompressor().compress(raw)
  path = tmp_path / "trail.zst"
  path.write_bytes(frame + b"TRAILING_JUNK")
  with pytest.raises(fixture_tool.FixtureError, match="malformed"):
    fixture_tool.process(str(path), None, str(tmp_path / "report.json"), str(POLICY), "nap-preap-pedal-v1", None, False)

  multi = frame + zstandard.ZstdCompressor().compress(b"NEXTFRAME")
  path.write_bytes(multi)
  with pytest.raises(fixture_tool.FixtureError, match="malformed"):
    fixture_tool.process(str(path), None, str(tmp_path / "report2.json"), str(POLICY), "nap-preap-pedal-v1", None, False)


def test_nonexistent_canonical_path_aliases_are_rejected(tmp_path: Path):
  missing = tmp_path / "out.zst"
  aliased = tmp_path / "sub" / ".." / "out.zst"
  with pytest.raises(fixture_tool.FixtureError, match="path collision"):
    fixture_tool.validate_path_separation({"output": missing, "report": aliased})


def test_bounded_zstd_expansion_rejects_bomb(tmp_path: Path):
  # Tiny compressed payload that would expand past the fixture tool ceiling.
  bomb = zstandard.ZstdCompressor(level=1).compress(b"A" * (fixture_tool.MAX_DECOMPRESSED_INPUT_BYTES + 1024))
  assert len(bomb) < fixture_tool.MAX_COMPRESSED_INPUT_BYTES
  path = tmp_path / "bomb.zst"
  path.write_bytes(bomb)
  report = tmp_path / "report.json"
  with pytest.raises(fixture_tool.FixtureError, match="decompressed size limit|malformed"):
    fixture_tool.process(str(path), None, str(report), str(POLICY), "nap-preap-pedal-v1", None, False)
