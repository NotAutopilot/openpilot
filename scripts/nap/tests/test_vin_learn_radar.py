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
from scripts.nap.vin_learn_radar import (
  check_safety_mode, parse_args, probe_radar, recv, report_tx_health,
)

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


def can_health(tx=0, errors=0, lost=0, last_error="No error", bus_off=False):
  return {"total_tx_cnt": tx, "total_error_cnt": errors, "total_tx_lost_cnt": lost,
          "last_error": last_error, "bus_off": bus_off, "error_passive": False,
          "can_speed": 500}


class HealthPanda(FakePanda):
  def __init__(self, healths):
    super().__init__()
    self._healths = list(healths)

  def can_health(self, bus):
    return self._healths.pop(0) if len(self._healths) > 1 else self._healths[0]


def test_tx_health_flags_a_bus_nothing_is_leaving(capsys):
  # GTW emulation should be pouring frames onto this bus; a flat counter means
  # nothing is being transmitted at all.
  panda = HealthPanda([can_health(tx=1000)])
  report_tx_health(panda, 1, can_health(tx=1000), can_health(tx=1000))
  assert "Nothing is leaving the panda on this bus" in capsys.readouterr().out


def test_tx_health_flags_unacknowledged_frames(capsys):
  panda = HealthPanda([can_health(tx=1002, errors=2, last_error="AckError")])
  report_tx_health(panda, 1, can_health(tx=0), can_health(tx=1000))
  assert "not acknowledged" in capsys.readouterr().out


def test_tx_health_reports_clean_transmit(capsys):
  panda = HealthPanda([can_health(tx=1002)])
  report_tx_health(panda, 1, can_health(tx=0), can_health(tx=1000))
  out = capsys.readouterr().out
  assert "transmitted cleanly" in out
  assert "frames sent during probe:    2" in out


def test_tx_health_skips_when_panda_reports_nothing(capsys):
  panda = FakePanda()  # no can_health at all
  report_tx_health(panda, 1, None, None)
  assert "did not report per-bus CAN health" in capsys.readouterr().out


class NackingPanda(FakePanda):
  """A panda whose control-transfer reads always fail, as SPI NACKs do."""

  def __init__(self):
    super().__init__()
    self.health_calls = 0

  def health(self):
    self.health_calls += 1
    raise RuntimeError("PandaSpiNackResponse")

  def can_health(self, bus):
    raise RuntimeError("PandaSpiNackResponse")


def test_health_reads_retry_then_give_up_instead_of_raising():
  from scripts.nap.vin_learn_radar import CONTROL_READ_RETRIES, bus_health, panda_health
  panda = NackingPanda()
  assert panda_health(panda) is None
  assert panda.health_calls == CONTROL_READ_RETRIES
  assert bus_health(panda, 1) is None


def test_safety_mode_check_does_not_block_when_health_is_unreadable(capsys):
  # A courtesy check must not gate the run just because a diagnostic read failed.
  assert check_safety_mode(NackingPanda(), TESLA_PREAP) is True
  assert "could not read panda health" in capsys.readouterr().out


def test_probe_survives_unreadable_health(capsys):
  # The probe's real job is listening and sending; health is a bonus.
  class ProbePanda(ScriptedPanda, NackingPanda):
    pass

  panda = ProbePanda(background=BACKGROUND)
  assert probe_radar(panda, listen=0.01, reply_window=0.01) is False
  out = capsys.readouterr().out
  assert "did not report per-bus CAN health" in out
  assert "TesterPresent probe" in out


def alert_payload(*bits):
  dat = bytearray(8)
  for bit in bits:
    dat[bit // 8] |= 1 << (bit % 8)
  return bytes(dat)


def test_alert_matrix_decodes_the_three_config_mismatches():
  from scripts.nap.vin_learn_radar import decode_alert_matrix
  names = dict(decode_alert_matrix(alert_payload(36, 60, 61)))
  assert names == {36: "vinValidity", 60: "radPositionMismatch", 61: "strRackMismatch"}


def test_alert_matrix_decodes_nothing_when_clean():
  from scripts.nap.vin_learn_radar import decode_alert_matrix
  assert decode_alert_matrix(bytes(8)) == []


def test_alert_report_calls_out_each_config_mismatch(capsys):
  from scripts.nap.vin_learn_radar import report_alert_matrix
  report_alert_matrix(alert_payload(36, 60, 61))
  out = capsys.readouterr().out
  assert "CONFIG MISMATCH" in out
  assert "not the one this radar was programmed with" in out
  assert "donor car's model" in out
  assert "donor car's steering rack" in out


def test_alert_report_separates_unrelated_alerts(capsys):
  from scripts.nap.vin_learn_radar import report_alert_matrix
  report_alert_matrix(alert_payload(6, 36))   # sensorBlinded + vinValidity
  out = capsys.readouterr().out
  assert "CONFIG MISMATCH" in out
  assert "Other alerts set (1)" in out
  assert "sensorBlinded" in out


def test_alert_report_says_so_when_radar_is_happy(capsys):
  from scripts.nap.vin_learn_radar import report_alert_matrix
  report_alert_matrix(bytes(8))
  assert "No alerts set" in capsys.readouterr().out


def test_alert_report_handles_never_seeing_the_message(capsys):
  from scripts.nap.vin_learn_radar import report_alert_matrix
  report_alert_matrix(None)
  assert "cannot read the" in capsys.readouterr().out


def test_probe_surfaces_the_alert_matrix(capsys):
  alert = [(0x501, alert_payload(36), RADAR_BUS)]
  panda = ScriptedPanda(background=BACKGROUND + alert)
  probe_radar(panda, listen=0.01, reply_window=0.01)
  assert "vinValidity" in capsys.readouterr().out


def test_wait_for_pandad_returns_as_soon_as_it_is_gone(monkeypatch):
  import scripts.nap.vin_learn_radar as tool
  calls = []

  def fake_running():
    calls.append(1)
    return len(calls) < 3

  monkeypatch.setattr(tool, "pandad_running", fake_running)
  assert tool.wait_for_pandad_to_stop(timeout=5.0) is True


def test_wait_for_pandad_gives_up_and_reports_it(monkeypatch):
  import scripts.nap.vin_learn_radar as tool
  monkeypatch.setattr(tool, "pandad_running", lambda: True)
  assert tool.wait_for_pandad_to_stop(timeout=0.01) is False


def test_pandad_detection_never_raises():
  # Runs on dev machines with no procfs and on-device mid-teardown, where pids
  # vanish between listing and reading. Must degrade, not blow up the run.
  from scripts.nap.vin_learn_radar import pandad_running
  assert pandad_running() in (True, False)


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
