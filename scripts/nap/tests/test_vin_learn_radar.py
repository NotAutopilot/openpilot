#!/usr/bin/env python3
"""Tests for the VIN-learn runner's CAN plumbing and panda-state checks.

Not under pyproject's `testpaths`, so run it explicitly:
    pytest scripts/nap/tests/test_vin_learn_radar.py
"""
import importlib.util
import os

import pytest

from openpilot.common.basedir import BASEDIR
from opendbc.car.can_definitions import CanData
from opendbc.car.tesla.preap.radar_vin import RADAR_BUS, RADAR_RX_ADDRESS, RADAR_TX_ADDRESS
from scripts.nap.vin_learn_radar import check_safety_mode, parse_args, probe_radar, recv

PROBE_MODULE = "scripts.nap.probe_radar"

TESLA_PREAP = 37
UDS_REQUEST = b"\x02\x10\x03\x00\x00\x00\x00\x00"


class FakePanda:
  def __init__(self, msgs=(), health=None):
    self._msgs = list(msgs)
    self._health = health or {}

  def can_recv(self):
    return self._msgs

  def health(self):
    return self._health


def test_recv_passes_through_received_traffic():
  panda = FakePanda([
    (0x405, b"\x10\x00\x00\x00\x00ABC", 0),
    (RADAR_RX_ADDRESS, b"\x03\x62\xf1\x90\x00\x00\x00\x00", RADAR_BUS),
  ])
  packets, rejected = recv(panda)
  assert not rejected
  assert packets == [
    CanData(0x405, b"\x10\x00\x00\x00\x00ABC", 0),
    CanData(RADAR_RX_ADDRESS, b"\x03\x62\xf1\x90\x00\x00\x00\x00", RADAR_BUS),
  ]


def test_recv_drops_loopback_echo_of_our_own_sends():
  # The panda returns what we transmitted with the bus offset by 128. Feeding
  # that to the learner would look like the radar answering its own request.
  panda = FakePanda([(RADAR_TX_ADDRESS, UDS_REQUEST, RADAR_BUS + 128)])
  packets, rejected = recv(panda)
  assert packets == []
  assert not rejected


@pytest.mark.parametrize("offset", [192, 128 + 192])
def test_recv_flags_safety_rejected_sends(offset):
  # A 192 offset means the safety layer refused the TX, i.e. panda firmware
  # without PREAP_FLAG_RADAR_VIN_LEARN.
  panda = FakePanda([(RADAR_TX_ADDRESS, UDS_REQUEST, RADAR_BUS + offset)])
  packets, rejected = recv(panda)
  assert packets == []
  assert rejected


def test_recv_accepts_four_tuple_shape():
  panda = FakePanda([(0x405, None, b"\x11\x00\x00\x00\x00\x00\x00\x00", 0)])
  packets, _ = recv(panda)
  assert packets == [CanData(0x405, b"\x11\x00\x00\x00\x00\x00\x00\x00", 0)]


def test_recv_skips_unknown_tuple_shapes():
  panda = FakePanda([(0x405, 0), (0x405, 0, b"\x00", 0, 0)])
  packets, rejected = recv(panda)
  assert packets == []
  assert not rejected


# The panda drops to SAFETY_SILENT once its heartbeat times out. That blocks
# every send and stops GTW emulation, which is indistinguishable from an
# unwired radar unless we look at health.

class ScriptedPanda(FakePanda):
  """Background traffic always; reply frames only after something is sent."""

  def __init__(self, background=(), reply=()):
    super().__init__()
    self._background = list(background)
    self._reply = list(reply)
    self.sent = []

  def can_send(self, addr, dat, bus):
    self.sent.append((addr, dat, bus))

  def can_recv(self):
    return self._background + (self._reply if self.sent else [])


# One radar track frame, i.e. a live radar minding its own business.
BACKGROUND = [(0x310, b"\x00" * 8, RADAR_BUS)]


def test_probe_reports_failure_when_bus_is_silent():
  panda = ScriptedPanda()
  assert probe_radar(panda, listen=0.01, reply_window=0.01) is False


def test_probe_reports_failure_when_radar_ignores_tester_present():
  panda = ScriptedPanda(background=BACKGROUND)
  assert probe_radar(panda, listen=0.01, reply_window=0.01) is False
  # Probed every candidate address, and only ever sent TesterPresent.
  assert [addr for addr, _, _ in panda.sent] == [0x641, 0x671]
  assert {dat[:3] for _, dat, _ in panda.sent} == {b"\x02\x3e\x00"}
  assert {bus for _, _, bus in panda.sent} == {RADAR_BUS}


def test_probe_detects_a_responding_radar():
  reply = [(RADAR_RX_ADDRESS, b"\x02\x7e\x00\x00\x00\x00\x00\x00", RADAR_BUS)]
  panda = ScriptedPanda(background=BACKGROUND, reply=reply)
  assert probe_radar(panda, listen=0.01, reply_window=0.01) is True


def test_probe_ignores_track_frames_as_replies():
  # Track frames are background noise, never a diagnostic response.
  reply = [(0x340, b"\xff" * 8, RADAR_BUS)]
  panda = ScriptedPanda(background=BACKGROUND, reply=reply)
  assert probe_radar(panda, listen=0.01, reply_window=0.01) is False


def test_probe_flag_parses():
  # probe_radar.py hands main() exactly this; if the flag is renamed the GUI
  # button silently runs a full VIN learn instead of a read-only probe.
  assert parse_args(["--probe"]).probe is True
  assert parse_args([]).probe is False


def test_probe_entry_point_exists_and_is_wired_to_both_layouts():
  assert importlib.util.find_spec(PROBE_MODULE) is not None
  for layout in ("selfdrive/ui/layouts/settings/nap.py",
                 "selfdrive/ui/mici/layouts/settings/nap.py"):
    with open(os.path.join(BASEDIR, layout)) as f:
      assert PROBE_MODULE in f.read(), f"{layout} does not launch {PROBE_MODULE}"


def test_check_safety_mode_accepts_expected_mode():
  panda = FakePanda(health={"safety_mode": TESLA_PREAP, "heartbeat_lost": False})
  assert check_safety_mode(panda, TESLA_PREAP)


def test_check_safety_mode_rejects_silent_fallback():
  panda = FakePanda(health={"safety_mode": 0, "heartbeat_lost": True})
  assert not check_safety_mode(panda, TESLA_PREAP)


def test_check_safety_mode_rejects_unexpected_mode_without_heartbeat_loss():
  panda = FakePanda(health={"safety_mode": 17, "heartbeat_lost": False})
  assert not check_safety_mode(panda, TESLA_PREAP)
