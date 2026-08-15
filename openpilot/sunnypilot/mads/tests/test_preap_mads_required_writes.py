import unittest
from unittest.mock import MagicMock

from opendbc.car import structs
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.selfdrive.car.helpers import convert_to_capnp
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.mads.helpers import coerce_mads_write, is_mads_required, persist_required_mads
from openpilot.sunnypilot.mads.mads import ModularAssistiveDrivingSystem


class FakeParams:
  def __init__(self, initial=None):
    self.store = dict(initial or {})
    self.puts = []

  def get_bool(self, key):
    return bool(self.store.get(key, False))

  def get(self, key, return_default=False):
    return self.store.get(key)

  def put_bool(self, key, value, block=True):
    self.puts.append((key, bool(value)))
    self.store[key] = bool(value)

  def put(self, key, value, block=True):
    self.puts.append((key, value))
    self.store[key] = value


def _required_cp_sp():
  CP_SP = structs.CarParamsSP()
  CP_SP.madsCapabilityContractVersion = 1
  CP_SP.madsRequired = True
  return CP_SP


def _modern_cp_sp():
  CP_SP = structs.CarParamsSP()
  CP_SP.madsCapabilityContractVersion = 1
  CP_SP.madsRequired = False
  return CP_SP


class TestRequiredMadsWrites(unittest.TestCase):
  def test_stale_persisted_false_is_healed(self):
    params = FakeParams({"Mads": False})
    self.assertTrue(persist_required_mads(params, _required_cp_sp()))
    self.assertTrue(params.get_bool("Mads"))
    assert ("Mads", True) in params.puts

  def test_reboot_uses_persistent_capabilities(self):
    params = FakeParams({
      "CarParamsSPPersistent": convert_to_capnp(_required_cp_sp()).to_bytes(),
      "Mads": False,
    })
    self.assertTrue(persist_required_mads(params))
    self.assertTrue(params.get_bool("Mads"))

  def test_runtime_false_write_rejected(self):
    params = FakeParams({"Mads": True})
    self.assertEqual(coerce_mads_write(params, "Mads", False, _required_cp_sp()), True)
    self.assertTrue(params.get_bool("Mads"))

  def test_sunnylink_false_payload_coerced(self):
    params = FakeParams({"Mads": True})
    self.assertEqual(coerce_mads_write(params, "Mads", "false", _required_cp_sp()), True)
    self.assertEqual(coerce_mads_write(params, "MadsMainCruiseAllowed", False, _required_cp_sp()), False)

  def test_modern_platform_allows_false(self):
    params = FakeParams({"Mads": True})
    self.assertFalse(is_mads_required(_modern_cp_sp(), params))
    self.assertEqual(coerce_mads_write(params, "Mads", False, _modern_cp_sp()), False)
    self.assertFalse(persist_required_mads(params, _modern_cp_sp()))
    self.assertTrue(params.get_bool("Mads"))

  def test_mads_runtime_heals_without_rereading_toggle(self):
    CP = structs.CarParams()
    CP.brand = "tesla"
    CP.carFingerprint = "TESLA_MODEL_S_PREAP"
    CP_SP = _required_cp_sp()
    CP_SP.madsMainCruiseInputKind = structs.CarParamsSP.MadsMainCruiseInputKind.momentary
    params = FakeParams({"Mads": False, "MadsMainCruiseAllowed": False})
    sd = MagicMock()
    sd.CP = CP
    sd.CP_SP = CP_SP
    sd.params = params
    sd.events = Events()
    sd.events_sp = EventsSP()
    sd.enabled = False
    sd.enabled_prev = False
    sd.initialized = True
    sd.CS_prev = structs.CarState()
    sd.sm = {"pandaStates": []}
    mads = ModularAssistiveDrivingSystem(sd)
    self.assertTrue(mads.enabled_toggle)
    self.assertTrue(params.get_bool("Mads"))
    params.store["Mads"] = False
    mads.read_params()
    self.assertTrue(mads.enabled_toggle)
    self.assertTrue(params.get_bool("Mads"))
    params.store["Mads"] = False
    mads.update(structs.CarState())
    self.assertTrue(mads.enabled_toggle)
    self.assertTrue(params.get_bool("Mads"))


if __name__ == "__main__":
  unittest.main()
