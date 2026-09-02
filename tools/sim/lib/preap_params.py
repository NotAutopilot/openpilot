"""Identity and Params for the Pre-AP MetaDrive SimulatedCar."""
import os

from openpilot.common.params import Params
from openpilot.selfdrive.test.helpers import set_params_enabled

PREAP_VIN = "5YJSA1H13EFP20460"
PREAP_FINGERPRINT = "TESLA_MODEL_S_PREAP"

# Jack's Comma Pedal calib from process_replay. Defaults put gasPressed in
# the rest-noise band; these values keep rest below PEDAL_DI_PRESSED.
PREAP_PEDAL_CALIB = {
  "NAPPedalCalibMin": 0.73299810159,
  "NAPPedalCalibMax": 110.23299810159,
  "NAPPedalCalibFactor": 0.9478672985781991,
  "NAPPedalCalibZero": 4.732998101589998,
}


def configure_preap_sim():
  """Fingerprint TESLA_MODEL_S_PREAP, pedal+radar on, override not a disengage."""
  set_params_enabled()
  os.environ["FINGERPRINT"] = PREAP_FINGERPRINT
  os.environ["VIN"] = PREAP_VIN

  params = Params()
  # Fresh OPENPILOT_PREFIX params have no DongleId; manager_init would hang in
  # register() talking to comma. Sim is not a registered device.
  if not params.get("DongleId"):
    params.put("DongleId", "UnregisteredDevice", block=True)
  params.put_bool("DisengageOnAccelerator", False, block=True)
  params.put_bool("NAPForcePreAP", True, block=True)
  params.put_bool("NAPPedalEnabled", True, block=True)
  params.put_bool("NAPPedalCalibDone", True, block=True)
  params.put_bool("NAPRadarEnabled", True, block=True)
  params.put("NAPRadarDonorVin", PREAP_VIN, block=True)
  params.put("NAPPedalCanBus", 2, block=True)
  for key, value in PREAP_PEDAL_CALIB.items():
    params.put(key, float(value), block=True)
  return params
