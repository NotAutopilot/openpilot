import struct
import crcmod
from ctypes import create_string_buffer
from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven
from opendbc.car.tesla.values import CANBUS

# Comma Pedal protocol constants
PEDAL_M1 = 0.050796813    # Primary scaling factor
PEDAL_M2 = 0.101593626    # Secondary scaling (2x M1 for redundancy)
PEDAL_D = -22.85856576    # Offset
GAS_COMMAND_ID = 0x551

# Default STW_ACTN_RQ signal values (all signals except counter, CRC, and button)
_STW_DEFAULTS = {
  "VSL_Enbl_Stat": 0, "DTR_Dist_Rq": 0, "TurnIndLvr_Stat": 0,
  "HiBmLvr_Stat": 0, "WprWashSw_Psd": 0, "WprWash_R_Sw_Posn_V2": 0,
  "StW_Lvr_Stat": 0, "StW_Cond_Flt": 0, "StW_Cond_Psd": 0,
  "HrnSw_Psd": 0, "StW_Sw00_Psd": 0, "StW_Sw01_Psd": 0,
  "StW_Sw02_Psd": 0, "StW_Sw03_Psd": 0, "StW_Sw04_Psd": 0,
  "StW_Sw05_Psd": 0, "StW_Sw06_Psd": 0,
}


class TeslaCANPreAP(TeslaCANRaven):
  def __init__(self, packers):
    super().__init__(packers)
    self.pedal_can_bus = 2
    # Pedal firmware watchdog requires consecutive counter values
    self.pedal_idx = 0
    # STW_ACTN_RQ uses CRC-8 (poly 0x1D), not the byte-sum checksum
    self.stw_crc = crcmod.mkCrcFun(0x11d, initCrc=0x00, rev=False, xorOut=0xff)

  @staticmethod
  def pedal_checksum(msg_id, dat):
    """Comma Pedal CAN checksum: addr bytes + data bytes, truncated to 8 bits."""
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_pedal_command(self, accel_command, enable=1, pedal_can_bus=None):
    """Build GAS_COMMAND (0x551) using raw struct packing for firmware byte-compatibility."""
    if pedal_can_bus is None:
      pedal_can_bus = self.pedal_can_bus

    idx = self.pedal_idx
    self.pedal_idx = (self.pedal_idx + 1) % 16

    if enable == 1:
      int_cmd1 = max(0, min(65534, int((accel_command - PEDAL_D) / PEDAL_M1)))
      int_cmd2 = max(0, min(65534, int((accel_command - PEDAL_D) / PEDAL_M2)))
    else:
      int_cmd1 = 0
      int_cmd2 = 0

    msg = create_string_buffer(6)
    struct.pack_into("BBBBB", msg, 0,
                     (int_cmd1 >> 8) & 0xFF, int_cmd1 & 0xFF,
                     (int_cmd2 >> 8) & 0xFF, int_cmd2 & 0xFF,
                     ((enable << 7) + idx) & 0xFF)
    struct.pack_into("B", msg, 5, self.pedal_checksum(GAS_COMMAND_ID, msg.raw))

    return (GAS_COMMAND_ID, bytes(msg.raw), pedal_can_bus)

  def create_epas_control(self, counter, mode):
    values = {
      "EPB_epasEACAllow": mode,
      "EPB_epasControlCounter": counter,
      "EPB_epasControlChecksum": 0,
    }
    data = self.packers[CANBUS.party].make_can_msg("EPB_epasControl", CANBUS.party, values)[1]
    values["EPB_epasControlChecksum"] = self.checksum(0x214, data)
    return self.packers[CANBUS.party].make_can_msg("EPB_epasControl", CANBUS.party, values)

  def create_action_request(self, button_to_press, bus, counter, msg_stw=None):
    """Build STW_ACTN_RQ to simulate cruise stalk button press."""
    values = {"MC_STW_ACTN_RQ": counter, "CRC_STW_ACTN_RQ": 0, "SpdCtrlLvr_Stat": button_to_press}
    if msg_stw is not None:
      for key, default in _STW_DEFAULTS.items():
        values[key] = msg_stw.get(key, default)
    else:
      values.update(_STW_DEFAULTS)

    data = self.packers[CANBUS.party].make_can_msg("STW_ACTN_RQ", bus, values)[1]
    values["CRC_STW_ACTN_RQ"] = self.stw_crc(data[:7])
    return self.packers[CANBUS.party].make_can_msg("STW_ACTN_RQ", bus, values)
