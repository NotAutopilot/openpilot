import capnp
import unittest
from pathlib import Path

from openpilot.cereal import custom
from openpilot.selfdrive.car.helpers import convert_to_capnp
from opendbc.car import structs


FIXTURES = Path(__file__).parent / "fixtures"


def _ordinals(schema):
  return {field.proto.name: getattr(field.proto.ordinal, field.proto.ordinal.which()) for field in schema.fields_list}


class TestPreAPSchemaContract(unittest.TestCase):
  def test_enum_numeric_pins(self):
    self.assertEqual(int(custom.CarParamsSP.MadsMainCruiseInputKind.none), 0)
    self.assertEqual(int(custom.CarParamsSP.MadsMainCruiseInputKind.stateful), 1)
    self.assertEqual(int(custom.CarParamsSP.MadsMainCruiseInputKind.momentary), 2)
    self.assertEqual(int(custom.CarParamsSP.MadsSteeringMode.remainActive), 0)
    self.assertEqual(int(custom.CarParamsSP.MadsSteeringMode.pause), 1)
    self.assertEqual(int(custom.CarParamsSP.MadsSteeringMode.disengage), 2)
    self.assertEqual(int(custom.CarParamsSP.PreapLateralEngagementMode.independent), 0)
    self.assertEqual(int(custom.CarParamsSP.PreapLateralEngagementMode.cruiseCoupled), 1)
    self.assertEqual(int(custom.CarParamsSP.PreapLateralEngagementMode.longitudinalOnly), 2)
    self.assertEqual(int(custom.CarStateSP.PreapLateralIntent.none), 0)
    self.assertEqual(int(custom.CarStateSP.PreapLateralIntent.mainCruiseRequest), 1)
    self.assertEqual(int(custom.CarStateSP.PreapLateralIntent.forceDisable), 2)
    self.assertEqual(int(custom.CarStateSP.PreapLongitudinalIntent.none), 0)
    self.assertEqual(int(custom.CarStateSP.PreapLongitudinalIntent.enable), 1)
    self.assertEqual(int(custom.CarStateSP.PreapLongitudinalIntent.disable), 2)
    self.assertEqual(int(custom.CarStateSP.PreapStockCcTransactionState.idle), 0)
    self.assertEqual(int(custom.CarStateSP.PreapStockCcTransactionState.cancelRequested), 1)
    self.assertEqual(int(custom.CarStateSP.PreapStockCcTransactionState.awaitingCancelConfirmation), 2)
    self.assertEqual(int(custom.CarStateSP.PreapStockCcTransactionState.awaitingSecondPull), 3)
    self.assertEqual(int(custom.CarStateSP.PreapStockCcTransactionState.reengageRequested), 4)
    self.assertEqual(int(custom.CarStateSP.PreapStockCcTransactionState.awaitingDiConfirmation), 5)
    self.assertEqual(int(custom.CarStateSP.PreapStockCcTransactionState.confirmed), 6)
    self.assertEqual(int(custom.CarStateSP.PreapStockCcTransactionState.cancelledOrFailed), 7)
    enum = custom.CarStateSP.PreapStockCcTransactionState
    self.assertEqual([int(enum.idle), int(enum.cancelRequested), int(enum.awaitingCancelConfirmation),
                      int(enum.awaitingSecondPull), int(enum.reengageRequested), int(enum.awaitingDiConfirmation),
                      int(enum.confirmed), int(enum.cancelledOrFailed)], list(range(8)))

  def test_field_ordinal_pins(self):
    cp = _ordinals(custom.CarParamsSP.schema)
    self.assertEqual(cp["flags"], 0)
    self.assertEqual(cp["safetyParam"], 1)
    self.assertEqual(cp["neuralNetworkLateralControl"], 2)
    self.assertEqual(cp["pcmCruiseSpeed"], 3)
    self.assertEqual(cp["intelligentCruiseButtonManagementAvailable"], 4)
    self.assertEqual(cp["enableGasInterceptor"], 5)
    self.assertEqual(cp["madsFullSettingsAvailable"], 6)
    self.assertEqual(cp["madsMainCruiseInputKind"], 7)
    self.assertEqual(cp["madsMainCruiseAllowed"], 8)
    self.assertEqual(cp["madsRequired"], 9)
    self.assertEqual(cp["teslaCoopSteeringAvailable"], 10)
    self.assertEqual(cp["madsUnifiedEngagementMode"], 11)
    self.assertEqual(cp["madsSteeringMode"], 12)
    self.assertEqual(cp["madsCapabilityContractVersion"], 13)
    self.assertEqual(cp["madsHandsOnPauseAvailable"], 14)
    self.assertEqual(cp["preapLateralEngagementMode"], 15)
    self.assertEqual(cp["radarOffset"], 16)

    cs = _ordinals(custom.CarStateSP.schema)
    self.assertEqual(cs["speedLimit"], 0)
    self.assertEqual(cs["preapLateralIntent"], 1)
    self.assertEqual(cs["preapLongitudinalIntent"], 2)
    self.assertEqual(cs["preapIntentSequence"], 3)
    self.assertEqual(cs["preapIntentEpoch"], 4)
    self.assertEqual(cs["preapStockCcState"], 5)
    self.assertEqual(cs["preapStockCcBoundCounter"], 6)
    self.assertEqual(cs["preapStockCcHostDiConfirmed"], 7)
    self.assertEqual(cs["preapStockCcEnablePending"], 8)

    car_state = _ordinals(structs.CarState.schema)
    self.assertEqual(car_state["turnSignalStalkState"], 61)
    self.assertEqual(car_state["handsOnLevel"], 62)

  def test_version0_bytes_keep_old_fields(self):
    raw = (FIXTURES / "carparams_sp_v0_tesla_vehicle_bus.bin").read_bytes()
    with custom.CarParamsSP.from_bytes(raw) as msg:
      self.assertEqual(msg.flags, 1)
      self.assertEqual(msg.safetyParam, 1)
      self.assertTrue(msg.pcmCruiseSpeed)
      self.assertEqual(msg.neuralNetworkLateralControl.model.name, "FOO")
      self.assertEqual(msg.madsCapabilityContractVersion, 0)
      self.assertFalse(msg.madsRequired)
      self.assertEqual(msg.madsMainCruiseInputKind, custom.CarParamsSP.MadsMainCruiseInputKind.none)

    raw_cs = (FIXTURES / "carstate_sp_v0_speed_limit.bin").read_bytes()
    with custom.CarStateSP.from_bytes(raw_cs) as msg:
      self.assertAlmostEqual(msg.speedLimit, 25.0, places=4)
      self.assertEqual(msg.preapLateralIntent, custom.CarStateSP.PreapLateralIntent.none)
      self.assertEqual(msg.preapIntentSequence, 0)

  def test_version1_round_trip_and_old_schema_reads_old_fields(self):
    cp = structs.CarParamsSP()
    cp.flags = 7
    cp.safetyParam = 2
    cp.pcmCruiseSpeed = False
    cp.madsCapabilityContractVersion = 1
    cp.madsRequired = True
    cp.madsMainCruiseInputKind = structs.CarParamsSP.MadsMainCruiseInputKind.momentary
    cp.preapLateralEngagementMode = structs.CarParamsSP.PreapLateralEngagementMode.cruiseCoupled
    cp.radarOffset = 1.25
    cp.neuralNetworkLateralControl.model.name = "BAR"
    raw = convert_to_capnp(cp).to_bytes()
    with custom.CarParamsSP.from_bytes(raw) as msg:
      self.assertEqual(msg.flags, 7)
      self.assertEqual(msg.safetyParam, 2)
      self.assertFalse(msg.pcmCruiseSpeed)
      self.assertEqual(msg.madsCapabilityContractVersion, 1)
      self.assertTrue(msg.madsRequired)
      self.assertEqual(msg.neuralNetworkLateralControl.model.name, "BAR")
      self.assertAlmostEqual(msg.radarOffset, 1.25, places=4)

    capnp.remove_import_hook()
    old = capnp.load(str(FIXTURES / "custom_v0.capnp"))
    with old.CarParamsSP.from_bytes(raw) as old_msg:
      self.assertEqual(old_msg.flags, 7)
      self.assertEqual(old_msg.safetyParam, 2)
      self.assertFalse(old_msg.pcmCruiseSpeed)
      self.assertEqual(old_msg.neuralNetworkLateralControl.model.name, "BAR")
      self.assertFalse(hasattr(old_msg, "madsRequired"))

    cs = structs.CarStateSP()
    cs.speedLimit = 11.0
    cs.preapLateralIntent = structs.CarStateSP.PreapLateralIntent.forceDisable
    cs.preapLongitudinalIntent = structs.CarStateSP.PreapLongitudinalIntent.disable
    cs.preapIntentSequence = 9
    cs.preapIntentEpoch = 99
    raw_cs = convert_to_capnp(cs).to_bytes()
    with custom.CarStateSP.from_bytes(raw_cs) as msg:
      self.assertAlmostEqual(msg.speedLimit, 11.0, places=4)
      self.assertEqual(msg.preapLateralIntent, custom.CarStateSP.PreapLateralIntent.forceDisable)
      self.assertEqual(msg.preapLongitudinalIntent, custom.CarStateSP.PreapLongitudinalIntent.disable)
      self.assertEqual(msg.preapIntentSequence, 9)
      self.assertEqual(msg.preapIntentEpoch, 99)
      self.assertEqual(msg.preapStockCcState, custom.CarStateSP.PreapStockCcTransactionState.idle)
      self.assertFalse(msg.preapStockCcEnablePending)

    with old.CarStateSP.from_bytes(raw_cs) as old_cs:
      self.assertAlmostEqual(old_cs.speedLimit, 11.0, places=4)
      self.assertFalse(hasattr(old_cs, "preapLateralIntent"))


if __name__ == "__main__":
  unittest.main()
