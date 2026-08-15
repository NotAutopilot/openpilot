import hashlib
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import MagicMock
import re
from typing import cast

import pytest
from opendbc.car import structs

from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools import epas_integrity
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.calibrate_pedal import (
  PedalCalibrationError,
  build_pedal_command,
  parse_configured_pedal_bus,
)
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.epas_integrity import (
  BOOTLOADER_SIZE,
  BootloaderIntegrityError,
  verify_bootloader,
)
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.runner import APPROVED_TOOLS, approved_module, start_tool
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.safety import (
  ToolSafetyError,
  require_confirmation,
  require_offroad,
  require_preap_tool_start,
)
from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.transport import (
  SAFETY_ALLOUTPUT,
  SAFETY_ELM327,
  SAFETY_SILENT,
  SAFETY_TESLA_PREAP,
  DiagnosticTransport,
  TransportError,
)
from opendbc.car.tesla.preap.constants import PREAP_FLAG_ENABLE_PEDAL, PREAP_FLAG_PEDAL_BUS_ZERO, PREAP_FLAG_PEDAL_CALIBRATION, PREAP_MODE_INVALID


class FakeParams:
  def __init__(self, offroad=True):
    self.store = {"IsOffroad": offroad}

  def get_bool(self, key):
    return bool(self.store.get(key, False))

  def put_bool(self, key, value, block=True):
    self.store[key] = bool(value)


def test_offroad_and_confirmation_gates():
  require_offroad(FakeParams(offroad=True))
  with pytest.raises(ToolSafetyError):
    require_offroad(FakeParams(offroad=False))
  require_confirmation(True, tool="flash_epas")
  with pytest.raises(ToolSafetyError):
    require_confirmation(False, tool="flash_epas")
  require_confirmation(False, tool="diagnose_radar")
  with pytest.raises(ToolSafetyError):
    require_preap_tool_start(FakeParams(offroad=False), tool="calibrate_pedal", confirmed=True)
  with pytest.raises(ToolSafetyError):
    require_preap_tool_start(FakeParams(offroad=True), tool="calibrate_pedal", confirmed=False)


def test_transport_rejects_alloutput():
  panda = MagicMock()
  transport = DiagnosticTransport(panda=panda)
  with pytest.raises(TransportError, match="bypass"):
    transport._set_safety_mode(SAFETY_ALLOUTPUT)
  panda.set_safety_mode.assert_not_called()

def test_transport_selects_cereal_elm327_mode():
  panda = MagicMock()
  transport = DiagnosticTransport(panda=panda)
  transport.set_diagnostic_session()
  assert SAFETY_ELM327 == int(structs.CarParams.SafetyModel.elm327)
  panda.set_safety_mode.assert_called_once_with(SAFETY_ELM327, 0)


def test_unapproved_tool_rejected():
  with pytest.raises(ValueError, match="unapproved"):
    approved_module("radar_replay")
  with pytest.raises(ValueError, match="unapproved"):
    start_tool("lead_source_analysis", confirmed=True, params=FakeParams())
  with pytest.raises(ValueError, match="unapproved"):
    approved_module("vision_radar_delta")
  assert "vision_radar_delta" not in APPROVED_TOOLS
  assert "radar_replay" not in APPROVED_TOOLS


def test_start_tool_clears_script_running_on_spawn_fail(monkeypatch):
  params = FakeParams()

  def boom(*_args, **_kwargs):
    raise OSError("spawn failed")

  monkeypatch.setattr("openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.runner.subprocess.Popen", boom)
  with pytest.raises(OSError):
    start_tool("diagnose_radar", confirmed=True, params=params)
  assert params.get_bool("NAPScriptRunning") is False
  assert params.get_bool("NAPEpasRiskAccepted") is False


def test_bootloader_integrity_before_connect(monkeypatch):
  payload = b"x" * BOOTLOADER_SIZE
  md5 = hashlib.md5(payload).hexdigest()
  sha = hashlib.sha256(payload).hexdigest()
  monkeypatch.setattr(epas_integrity, "BOOTLOADER_MD5SUM", md5)
  monkeypatch.setattr(epas_integrity, "BOOTLOADER_SHA256", sha)
  with TemporaryDirectory() as tmp:
    path = Path(tmp) / "bl.bin"
    path.write_bytes(payload)
    assert verify_bootloader(path) == payload
    path.write_bytes(b"y" * BOOTLOADER_SIZE)
    with pytest.raises(BootloaderIntegrityError):
      verify_bootloader(path)


def test_flash_and_restore_verify_bootloader_before_connect(monkeypatch):
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools import flash_epas

  called = []

  def boom(_path):
    called.append("verify")
    raise BootloaderIntegrityError("bad bootloader")

  class BoomTransport:
    def __init__(self, *args, **kwargs):
      called.append("transport_init")

    def connect(self):
      called.append("connect")
      raise AssertionError("must not connect after integrity failure")

  monkeypatch.setattr(flash_epas, "verify_bootloader", boom)
  monkeypatch.setattr(flash_epas, "require_preap_tool_start", lambda **_k: None)
  monkeypatch.setattr(flash_epas, "_consume_ui_risk_ack", lambda: True)
  monkeypatch.setattr(flash_epas, "DiagnosticTransport", BoomTransport)
  monkeypatch.setattr(flash_epas, "bootloader_path", lambda _name: Path("/tmp/missing.bin"))

  assert flash_epas.main(["--accept-risk"]) == 1
  assert called == ["verify"]
  called.clear()
  assert flash_epas.main(["--restore", "--accept-risk"]) == 1
  assert called == ["verify"]


def test_pedal_command_rejects_invalid_bus():
  with pytest.raises(PedalCalibrationError, match="invalid pedal bus"):
    build_pedal_command(0.0, enable=0, bus=1)
  addr, dat, bus = build_pedal_command(0.0, enable=0, bus=2)
  assert bus == 2
  assert len(dat) == 6
  assert addr == 0x551


def test_start_tool_flash_sets_risk_ack(monkeypatch):
  params = FakeParams()
  monkeypatch.setattr(
    "openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.runner.subprocess.Popen",
    lambda *_args, **_kwargs: MagicMock(),
  )
  monkeypatch.setattr(
    "openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.runner.threading.Thread",
    lambda **_kwargs: MagicMock(),
  )
  start_tool("flash_epas", confirmed=True, params=params)
  assert params.get_bool("NAPScriptRunning") is True
  assert params.get_bool("NAPEpasRiskAccepted") is True

  params = FakeParams()
  start_tool("diagnose_radar", confirmed=True, params=params)
  assert params.get_bool("NAPScriptRunning") is True
  assert params.get_bool("NAPEpasRiskAccepted") is False


def test_flash_requires_risk_ack_before_connect(monkeypatch):
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools import flash_epas

  called = []

  class BoomTransport:
    def __init__(self, *args, **kwargs):
      called.append("transport_init")

    def connect(self):
      called.append("connect")
      raise AssertionError("must not connect without risk ack")

  monkeypatch.setattr(flash_epas, "_consume_ui_risk_ack", lambda: False)
  monkeypatch.setattr(flash_epas, "DiagnosticTransport", BoomTransport)
  monkeypatch.setattr(flash_epas, "verify_bootloader", lambda *_a, **_k: called.append("verify") or b"x")
  assert flash_epas.main([]) == 1
  assert called == []
  assert flash_epas.main(["--restore"]) == 1
  assert called == []


def test_diagnose_radar_does_not_change_safety():
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.diagnose_radar import diagnose_panda

  panda = MagicMock()
  panda.can_recv.return_value = []
  transport = DiagnosticTransport(panda=panda)
  diagnose_panda(transport, sniff_s=0.0)
  panda.set_safety_mode.assert_not_called()
  transport.close()
  panda.set_safety_mode.assert_not_called()


def test_manager_ignore_list_stops_required_daemons():
  src = Path(__file__).resolve().parents[7] / "system" / "manager" / "manager.py"
  text = src.read_text()
  assert 'params.get_bool("NAPScriptRunning")' in text
  ignore_block = text.split('params.get_bool("NAPScriptRunning")', 1)[1].split("ensure_running", 1)[0]
  for name in ("pandad", "card", "selfdrived", "plannerd", "radard"):
    assert name in ignore_block


def test_approved_tools_include_diagnose_radar():
  assert "diagnose_radar" in APPROVED_TOOLS


def test_native_panel_keyboard_and_diagnose():
  src = Path(__file__).resolve().parents[7] / "selfdrive" / "ui" / "sunnypilot" / "layouts" / "settings" / "vehicle" / "brands" / "tesla.py"
  text = src.read_text()
  assert "Keyboard" in text
  assert "option_item_sp" not in text
  assert "Diagnose Radar" in text
  assert "NAPBrakeFactor" not in text
  assert "Emergency Disable" not in text
  assert "ConfirmDialog" in text
  assert "start_tool" in text
  assert "coop_steering_toggle.set_visible(not is_preap)" in text or "set_visible(not is_preap)" in text


def test_mads_parent_reachable_while_forced():
  src = Path(__file__).resolve().parents[7] / "selfdrive" / "ui" / "sunnypilot" / "layouts" / "settings" / "steering.py"
  text = src.read_text()
  assert "ui_state.is_offroad() and self._mads_toggle.action_item.get_state()" not in text
  assert "mads_required" in text
  assert "self._mads_toggle.set_visible(True)" in text
  assert "set_visible(not is_preap)" not in text
  assert "mads_required or is_preap" in text

def test_negative_response_maps_to_transport_error():
  from opendbc.car.uds import NegativeResponseError
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.transport import fail_closed_negative_response

  with pytest.raises(TransportError, match="negative response"):
    fail_closed_negative_response(NegativeResponseError("NRC 0x22", 0x10, 0x22))


def test_flash_negative_response_fails_closed_before_write(monkeypatch):
  from opendbc.car.uds import NegativeResponseError
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools import flash_epas

  called = []

  class FakeTransport:
    def __init__(self, *args, **kwargs):
      called.append("transport_init")

    def connect(self, panda_factory=None):
      called.append("connect")
      panda = MagicMock()
      panda.can_recv.return_value = []
      return panda

    def set_diagnostic_session(self):
      called.append("diag")

    def close(self):
      called.append("close")

  def boom(*_args, **_kwargs):
    called.append("extract")
    raise NegativeResponseError("NRC 0x7F service not supported", 0x10, 0x7F)

  monkeypatch.setattr(flash_epas, "verify_bootloader", lambda *_a, **_k: called.append("verify") or b"x")
  monkeypatch.setattr(flash_epas, "require_preap_tool_start", lambda **_k: None)
  monkeypatch.setattr(flash_epas, "_consume_ui_risk_ack", lambda: True)
  monkeypatch.setattr(flash_epas, "DiagnosticTransport", FakeTransport)
  monkeypatch.setattr(flash_epas, "extract_firmware", boom)
  monkeypatch.setattr(flash_epas, "flash_bootloader", lambda *_a, **_k: called.append("flash_bl"))
  monkeypatch.setattr(flash_epas, "flash_firmware", lambda *_a, **_k: called.append("flash_fw"))
  monkeypatch.setattr(flash_epas.os.path, "exists", lambda *_a, **_k: False)

  assert flash_epas.main(["--accept-risk"]) == 1
  assert "verify" in called
  assert "connect" in called
  assert "extract" in called
  assert "flash_bl" not in called
  assert "flash_fw" not in called
  assert "close" in called


PREAP_TRANSLATION_MSGIDS = (
  "Lateral Engagement Mode",
  "Radar Lateral Offset",
  "Radar Lateral Offset (m)",
  "Calibrate Pedal",
  "Calibrate Radar",
  "Diagnose Radar",
  "Test Radar",
  "Backup EPAS",
  "Flash EPAS",
  "Restore EPAS",
  "Cruise Coupled",
  "Longitudinal Only",
  "Pedal Interceptor",
  "Bosch Radar",
  "Radar Behind Nosecone",
  "Follow Distance",
  "Active Engagement Mode",
  "Longitudinal Path",
  "Pedal Health",
  "Radar Health",
  "Pedal Unavailable",
  "Regen Limit Reached",
  "Stock cruise required",
  "Press Brake to Slow Down",
)


def test_preap_translation_msgids_in_pot():
  trans = Path(__file__).resolve().parents[7] / "selfdrive" / "ui" / "translations"
  pot = (trans / "app.pot").read_text()
  for msgid in PREAP_TRANSLATION_MSGIDS + ("Pedal Calibration",):
    assert msgid.split("\n", 1)[0] in pot, msgid
  assert "#: openpilot/selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/tesla.py" in pot
  assert "#: openpilot/sunnypilot/selfdrive/selfdrived/preap_alerts.py" in pot
  assert "#: openpilot/sunnypilot/selfdrive/car/tesla/preap/tools/instructions.py" in pot
  for line in pot.splitlines():
    if "preap_alerts.py" in line or "tesla/preap/tools/instructions.py" in line or "brands/tesla.py" in line:
      assert line.startswith("#: openpilot/")
      assert not re.search(r"\.py:\d+", line)


def test_preap_po_catalogs_english_filled_others_empty():
  trans = Path(__file__).resolve().parents[7] / "selfdrive" / "ui" / "translations"
  catalogs = sorted(trans.glob("app_*.po"))
  assert any(p.name == "app_en.po" for p in catalogs)
  assert len(catalogs) >= 10
  for po in catalogs:
    body = po.read_text()
    for msgid in PREAP_TRANSLATION_MSGIDS:
      match = re.search(rf'msgid "{re.escape(msgid)}"\nmsgstr "(.*)"', body)
      assert match is not None, f"{po.name} missing {msgid}"
      if po.name == "app_en.po":
        assert match.group(1) == msgid, po.name
      else:
        assert match.group(1) == "", f"{po.name} should leave {msgid} untranslated"


def test_runtime_path_requires_basedir(monkeypatch):
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.safety import require_runtime_path, ToolSafetyError
  require_runtime_path()
  monkeypatch.setattr("openpilot.common.basedir.BASEDIR", "/tmp/not-openpilot")
  with pytest.raises(ToolSafetyError, match="BASEDIR"):
    require_runtime_path()


def test_runtime_path_rejects_copied_module(tmp_path):
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.safety import require_runtime_path, ToolSafetyError
  copied = tmp_path / "flash_epas.py"
  copied.write_text("# copied")
  with pytest.raises(ToolSafetyError, match="production path"):
    require_runtime_path(copied)


def test_run_script_rejects_unapproved_and_requires_offroad():
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.run_script import prepare_run, APPROVED_MODULES
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.runner import RUN_SCRIPT_MODULE, APPROVED_TOOLS
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.safety import ToolSafetyError
  assert RUN_SCRIPT_MODULE.endswith("run_script")
  assert "run_script" not in APPROVED_TOOLS
  with pytest.raises(ValueError, match="unapproved"):
    prepare_run("scripts.nap.radar_replay", FakeParams(offroad=True))
  with pytest.raises(ToolSafetyError, match="offroad"):
    prepare_run(next(iter(APPROVED_MODULES)), FakeParams(offroad=False))
  assert prepare_run(next(iter(APPROVED_MODULES)), FakeParams(offroad=True)) in APPROVED_MODULES


def test_negative_response_fail_closed():
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.transport import (
    TransportError,
    fail_closed_negative_response,
    uds_fail_closed,
  )
  class NegativeResponseError(Exception):
    error_code = 0x7F
  with pytest.raises(TransportError, match="negative response"):
    fail_closed_negative_response(NegativeResponseError("0x22"))
  wrapped = uds_fail_closed(NegativeResponseError("0x22"))
  assert isinstance(wrapped, TransportError)
  assert "fail closed" in str(wrapped)

def test_run_script_not_in_yaml():
  src = Path(__file__).resolve().parents[6] / "sunnylink" / "settings_ui_src" / "pages" / "vehicle.yaml"
  text = src.read_text()
  assert "run_script" not in text
  assert "NAPRadarOffset" not in text


def test_direct_destructive_main_requires_confirm(monkeypatch):
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools import calibrate_pedal, calibrate_radar
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.safety import parse_explicit_confirmation

  assert parse_explicit_confirmation([]) is False
  assert parse_explicit_confirmation(None) is False
  assert parse_explicit_confirmation(["--confirm"]) is True
  assert parse_explicit_confirmation(["--other"]) is False

  seen = {}

  def fake_run(*, confirmed, **kwargs):
    seen["confirmed"] = confirmed
    return 0

  monkeypatch.setattr(calibrate_pedal, "run", fake_run)
  assert calibrate_pedal.main([]) == 0
  assert seen["confirmed"] is False
  assert calibrate_pedal.main(["--confirm"]) == 0
  assert seen["confirmed"] is True

  seen.clear()
  monkeypatch.setattr(calibrate_radar, "run", fake_run)
  assert calibrate_radar.main([]) == 0
  assert seen["confirmed"] is False
  assert calibrate_radar.main(["--confirm"]) is not None
  assert seen["confirmed"] is True


def test_start_tool_passes_confirm_flag_for_destructive(monkeypatch):
  captured = {}

  def fake_popen(cmd, **kwargs):
    captured["cmd"] = cmd
    return MagicMock()

  monkeypatch.setattr(
    "openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.runner.subprocess.Popen",
    fake_popen,
  )
  monkeypatch.setattr(
    "openpilot.sunnypilot.selfdrive.car.tesla.preap.tools.runner.threading.Thread",
    lambda **_kwargs: MagicMock(),
  )
  start_tool("calibrate_pedal", confirmed=True, params=FakeParams())
  assert "--confirm" in captured["cmd"]
  start_tool("diagnose_radar", confirmed=True, params=FakeParams())
  assert "--confirm" not in captured["cmd"]


def test_start_tool_clears_runtime_flags_when_child_exits(monkeypatch):
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools import runner

  params = FakeParams()
  process = MagicMock()
  captured = {}

  class FakeThread:
    def __init__(self, *, target, args, daemon):
      captured.update(target=target, args=args, daemon=daemon)

    def start(self):
      captured["started"] = True

  monkeypatch.setattr(runner.subprocess, "Popen", lambda *_args, **_kwargs: process)
  monkeypatch.setattr(runner.threading, "Thread", FakeThread)

  assert runner.start_tool("flash_epas", confirmed=True, params=params) is process
  assert params.get_bool("NAPScriptRunning") is True
  assert params.get_bool("NAPEpasRiskAccepted") is True
  assert captured["started"] is True
  assert captured["daemon"] is True

  captured["target"](*captured["args"])
  process.wait.assert_called_once_with()
  assert params.get_bool("NAPScriptRunning") is False
  assert params.get_bool("NAPEpasRiskAccepted") is False


def test_start_tool_rejects_concurrent_owner(monkeypatch):
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools import runner

  params = FakeParams()
  params.put_bool("NAPScriptRunning", True)
  monkeypatch.setattr(runner.subprocess, "Popen", MagicMock())

  with pytest.raises(ToolSafetyError, match="already running"):
    runner.start_tool("diagnose_radar", confirmed=True, params=params)
  runner.subprocess.Popen.assert_not_called()
  assert params.get_bool("NAPScriptRunning") is True


def test_start_tool_constructs_default_params_for_native_ui(monkeypatch):
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools import runner

  params = FakeParams()
  process = MagicMock()
  monkeypatch.setattr(runner, "Params", lambda: params)
  monkeypatch.setattr(runner, "require_preap_tool_start", lambda *_args, **_kwargs: None)
  monkeypatch.setattr(runner.subprocess, "Popen", lambda *_args, **_kwargs: process)
  monkeypatch.setattr(runner.threading, "Thread", lambda **_kwargs: MagicMock())

  assert runner.start_tool("diagnose_radar", confirmed=True) is process
  assert params.get_bool("NAPScriptRunning") is True


def test_flash_parser_accepts_runner_confirmation_flag():
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools import flash_epas

  assert flash_epas.parse_args(["--confirm"]).accept_risk is True


def test_napbrakefactor_is_not_registered():
  src = Path(__file__).resolve().parents[7] / "common" / "params_keys.h"
  assert '"NAPBrakeFactor"' not in src.read_text()



class PedalCalibParams:
  def __init__(self, offroad=True, enabled=True, bus=2):
    self.store = {"IsOffroad": offroad, "NAPPedalEnabled": enabled, "NAPPedalCanBus": bus}

  def get_bool(self, key):
    return bool(self.store.get(key, False))

  def get(self, key):
    return self.store.get(key)


def test_parse_configured_pedal_bus_preserves_zero_and_defaults_empty():
  assert parse_configured_pedal_bus(0) == 0
  assert parse_configured_pedal_bus("0") == 0
  assert parse_configured_pedal_bus(b"0") == 0
  assert parse_configured_pedal_bus(None) == 2
  assert parse_configured_pedal_bus("") == 2
  assert parse_configured_pedal_bus(b"") == 2
  assert parse_configured_pedal_bus(2) == 2


class FirmwareShapedPanda:
  """Replica of panda 0xdc/0xdf: 0xdf is ignored while in a car safety mode."""

  def __init__(self, mode=SAFETY_TESLA_PREAP, param_sp=0):
    self.current_safety_mode = mode
    self.current_safety_param_sp = param_sp
    self.calls = []

  @staticmethod
  def _is_car_safety_mode(mode):
    # panda/board/main.c is_car_safety_mode
    return mode not in (SAFETY_SILENT, SAFETY_ALLOUTPUT, SAFETY_ELM327) and mode != int(structs.CarParams.SafetyModel.noOutput)

  def set_safety_mode(self, mode, param=0):
    self.calls.append(("0xdc", int(mode), int(param)))
    self.current_safety_mode = int(mode)

  def set_alternative_experience(self, alternative_experience, safety_param_sp=0):
    self.calls.append(("0xdf", int(alternative_experience), int(safety_param_sp)))
    if not self._is_car_safety_mode(self.current_safety_mode):
      self.current_safety_param_sp = int(safety_param_sp)


def test_transport_pedal_calibration_uses_tesla_preap_not_elm327():
  panda = MagicMock()
  transport = DiagnosticTransport(panda=panda)
  transport.set_pedal_calibration_session(2)
  panda.set_safety_mode.assert_any_call(SAFETY_SILENT, 0)
  panda.set_safety_mode.assert_any_call(SAFETY_TESLA_PREAP, PREAP_FLAG_PEDAL_CALIBRATION)
  panda.set_alternative_experience.assert_called_with(0, PREAP_MODE_INVALID)
  transport.set_pedal_calibration_session(0)
  panda.set_safety_mode.assert_called_with(
    SAFETY_TESLA_PREAP, PREAP_FLAG_PEDAL_CALIBRATION | PREAP_FLAG_PEDAL_BUS_ZERO,
  )


def test_pedal_calibration_session_0xdf_before_car_safety():
  panda = FirmwareShapedPanda(mode=SAFETY_TESLA_PREAP, param_sp=0)
  transport = DiagnosticTransport(panda=panda)
  transport.set_pedal_calibration_session(2)
  assert panda.calls == [
    ("0xdc", SAFETY_SILENT, 0),
    ("0xdf", 0, PREAP_MODE_INVALID),
    ("0xdc", SAFETY_TESLA_PREAP, PREAP_FLAG_PEDAL_CALIBRATION),
  ]
  assert panda.current_safety_param_sp == PREAP_MODE_INVALID
  # Car-first ordering would ignore 0xdf and leave Independent (0).
  ignored = FirmwareShapedPanda(mode=SAFETY_TESLA_PREAP, param_sp=0)
  ignored.set_safety_mode(SAFETY_TESLA_PREAP, PREAP_FLAG_PEDAL_CALIBRATION)
  ignored.set_alternative_experience(0, PREAP_MODE_INVALID)
  assert ignored.current_safety_param_sp == 0


def test_transport_rejects_elm327_and_alloutput_for_pedal_frames():
  panda = MagicMock()
  transport = DiagnosticTransport(panda=panda)
  transport.set_diagnostic_session()
  with pytest.raises(TransportError, match="teslaPreap"):
    transport.can_send(0x551, bytes(6), 2)
  panda.can_send.assert_not_called()
  with pytest.raises(TransportError, match="bypass"):
    transport._set_safety_mode(SAFETY_ALLOUTPUT)
  with pytest.raises(TransportError, match="longitudinal"):
    transport._set_safety_mode(SAFETY_TESLA_PREAP, PREAP_FLAG_PEDAL_CALIBRATION | PREAP_FLAG_ENABLE_PEDAL)
  with pytest.raises(TransportError, match="calibration flag"):
    transport._set_safety_mode(SAFETY_TESLA_PREAP, 0)


def test_calibrate_pedal_run_selects_preap_calibration_session(monkeypatch):
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools import calibrate_pedal

  calls = []

  class FakeTransport:
    def __init__(self, panda=None):
      self.panda = panda

    def connect(self, panda_factory=None):
      calls.append("connect")
      return self.panda

    def set_diagnostic_session(self):
      calls.append("elm327")

    def set_pedal_calibration_session(self, bus=2):
      calls.append(("preap", bus))

    def set_silent(self):
      calls.append("silent")

    def close(self):
      calls.append("close")

  class FakeCalibrator:
    def __init__(self, params, transport, bus):
      calls.append(("calibrator", bus))

    def run(self):
      calls.append("run")
      return 0

    def cleanup(self):
      calls.append("cleanup")

  monkeypatch.setattr(calibrate_pedal, "DiagnosticTransport", FakeTransport)
  monkeypatch.setattr(calibrate_pedal, "PedalCalibrator", FakeCalibrator)
  assert calibrate_pedal.run(
    confirmed=True,
    params=PedalCalibParams(bus=2),
    transport=cast(calibrate_pedal.DiagnosticTransport, FakeTransport()),
  ) == 0
  assert "elm327" not in calls
  assert ("preap", 2) in calls
  assert "run" in calls


def test_calibrate_pedal_run_preserves_configured_bus_zero(monkeypatch):
  from openpilot.sunnypilot.selfdrive.car.tesla.preap.tools import calibrate_pedal

  calls = []

  class FakeTransport:
    def __init__(self, panda=None):
      self.panda = panda

    def connect(self, panda_factory=None):
      return self.panda

    def set_diagnostic_session(self):
      calls.append("elm327")

    def set_pedal_calibration_session(self, bus=2):
      calls.append(("preap", bus))

    def close(self):
      calls.append("close")

  class FakeCalibrator:
    def __init__(self, params, transport, bus):
      calls.append(("calibrator", bus))

    def run(self):
      calls.append("run")
      return 0

    def cleanup(self):
      calls.append("cleanup")

  monkeypatch.setattr(calibrate_pedal, "DiagnosticTransport", FakeTransport)
  monkeypatch.setattr(calibrate_pedal, "PedalCalibrator", FakeCalibrator)
  assert calibrate_pedal.run(
    confirmed=True,
    params=PedalCalibParams(bus=0),
    transport=cast(calibrate_pedal.DiagnosticTransport, FakeTransport()),
  ) == 0
  assert "elm327" not in calls
  assert ("preap", 0) in calls
  assert ("calibrator", 0) in calls
