"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import pytest

from openpilot.common.params import Params
from openpilot.sunnypilot.sunnylink.athena import sunnylinkd


class TestSunnylinkdMethods:
  @pytest.fixture(autouse=True)
  def setup(self, monkeypatch):
    # Bind module-level Params to this test's isolated OpenpilotPrefix.
    monkeypatch.setattr(sunnylinkd, "params", Params())
    self.saved_params = []

    def mock_save_param(key, value, compression=False):
      self.saved_params.append((key, value, compression))

    monkeypatch.setattr(sunnylinkd, "save_param_from_base64_encoded_string", mock_save_param)

  def test_saveParams_blocked(self):
    blocked_params = {
      "GithubUsername": "attacker",
      "GithubSshKeys": "ssh-rsa attacker_key",
    }

    sunnylinkd.saveParams(blocked_params)

    assert len(self.saved_params) == 0

  def test_saveParams_allowed(self):
    allowed_params = {
      "SpeedLimitOffset": "5",
      "MyCustomParam": "123"
    }

    sunnylinkd.saveParams(allowed_params)

    # verify content
    assert len(self.saved_params) == 2
    keys_saved = [p[0] for p in self.saved_params]
    assert "SpeedLimitOffset" in keys_saved
    assert "MyCustomParam" in keys_saved

  def test_required_mads_cannot_be_modified(self, monkeypatch):
    monkeypatch.setattr(sunnylinkd, "_mads_required", lambda: True)
    sunnylinkd.saveParams({"Mads": "0", "SpeedLimitOffset": "5"})
    assert [item[0] for item in self.saved_params] == ["SpeedLimitOffset"]

  def test_optional_mads_remains_writable(self, monkeypatch):
    monkeypatch.setattr(sunnylinkd, "_mads_required", lambda: False)
    sunnylinkd.saveParams({"Mads": "0"})
    assert [item[0] for item in self.saved_params] == ["Mads"]

  def test_saveParams_mixed(self):
    mixed_params = {
      "GithubUsername": "attacker",
      "SpeedLimitOffset": "10"
    }

    sunnylinkd.saveParams(mixed_params)

    # should save allowed one
    assert len(self.saved_params) == 1
    assert self.saved_params[0][0] == "SpeedLimitOffset"
    assert self.saved_params[0][1] == "10"
