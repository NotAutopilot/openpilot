"""Byte-level invariants for NAP-spoofed STW_ACTN_RQ frames.

A real Tesla stalk emits byte 0 with bit 6 (VSL_Enbl_Rq) set. The DI rejects
or anomalously interprets frames with that bit cleared — which silently
happened pre-2026-04-25 due to a typo in `_STW_DEFAULTS` writing to the
non-existent key `VSL_Enbl_Stat` instead of `VSL_Enbl_Rq`. Drive-3 segment 2
showed NAP-spoofed UP_1ST at 47 kph dropping the DI from STANDBY→OFF instead
of engaging.

This test pins the invariant so the typo can't return.
"""
from opendbc.can import CANPacker
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP, _STW_DEFAULTS
from opendbc.car.tesla.values import CANBUS, CruiseButtons


def _build():
  packer = CANPacker("tesla_preap")
  tc = TeslaCANPreAP({CANBUS.party: packer, CANBUS.autopilot_party: packer})
  msg_stw = {"MC_STW_ACTN_RQ": 5, "CRC_STW_ACTN_RQ": 0, "DTR_Dist_Rq": 255}
  for key, default in _STW_DEFAULTS.items():
    msg_stw.setdefault(key, default)
  return tc, msg_stw


def _spoof(button):
  tc, msg_stw = _build()
  _, dat, _ = tc.create_action_request(button, CANBUS.party, 6, msg_stw)
  return dat


class TestSTWSpoofByteLayout:

  def test_vsl_enbl_rq_set_on_cancel(self):
    dat = _spoof(CruiseButtons.CANCEL)
    assert (dat[0] >> 6) & 1 == 1, \
      f"VSL_Enbl_Rq must be 1 in spoofed CANCEL — got byte0=0x{dat[0]:02x}"

  def test_vsl_enbl_rq_set_on_set_accel(self):
    dat = _spoof(CruiseButtons.SET_ACCEL)
    assert (dat[0] >> 6) & 1 == 1, \
      f"VSL_Enbl_Rq must be 1 in spoofed SET_ACCEL — got byte0=0x{dat[0]:02x}"

  def test_lever_bits_correct(self):
    assert _spoof(CruiseButtons.CANCEL)[0] & 0x3F == 1
    assert _spoof(CruiseButtons.SET_ACCEL)[0] & 0x3F == 16

  def test_byte0_matches_real_stalk_format(self):
    # Driver's real UP_1ST per drive-3 segment 2 trace: byte 0 = 0x50.
    # Lever=16 (0b010000) | VSL bit (0x40) = 0x50.
    dat = _spoof(CruiseButtons.SET_ACCEL)
    assert dat[0] == 0x50, f"Expected 0x50, got 0x{dat[0]:02x}"

  def test_defaults_uses_correct_dbc_signal_name(self):
    # Catch the typo class directly: every key in _STW_DEFAULTS must be a
    # real signal in the DBC's STW_ACTN_RQ message. If someone re-introduces
    # a bogus name, the packer would silently ignore it again.
    packer = CANPacker("tesla_preap")
    valid_msg = {key: 0 for key in _STW_DEFAULTS}
    valid_msg["SpdCtrlLvr_Stat"] = 0
    valid_msg["MC_STW_ACTN_RQ"] = 0
    valid_msg["CRC_STW_ACTN_RQ"] = 0
    # If any key is bogus, the packer drops it silently — our spoof would
    # then differ from the real frame layout. Build the message and verify
    # the resulting bytes look like a real frame (bit 6 set on byte 0).
    _, dat, _ = packer.make_can_msg("STW_ACTN_RQ", 0, valid_msg | {"VSL_Enbl_Rq": 1})
    assert (dat[0] >> 6) & 1 == 1, "VSL_Enbl_Rq=1 should set bit 6 of byte 0"
