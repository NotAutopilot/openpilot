"""Defaults-sanity rule for NAPConf.pedal_calibrated.

A calibration is trusted only when the stored params differ from defaults. If both
pedal_calib_zero and pedal_calib_factor read back as their DEFAULT_CONFIG values, the
getter reports False no matter what PEDAL_CALIB_DONE says. This closes the gap where
a partial / failed calibration leaves the done-flag set but the values unchanged.
"""
import json
import os
import tempfile
from unittest.mock import patch

import pytest

from opendbc.car.tesla.preap import nap_conf as nap_conf_module


@pytest.fixture
def tmp_conf():
  with tempfile.TemporaryDirectory() as d:
    path = os.path.join(d, "nap_params.json")
    with patch.object(nap_conf_module, "CONFIG_FILE", path), \
         patch.object(nap_conf_module, "_PARAMS_AVAILABLE", False):
      yield path


def _write(path, **overrides):
  data = dict(nap_conf_module.DEFAULT_CONFIG)
  data.update(overrides)
  with open(path, "w") as f:
    json.dump(data, f)


def test_calibrated_false_when_values_are_defaults(tmp_conf):
  _write(tmp_conf, pedal_calibrated=True, pedal_calib_zero=0.0, pedal_calib_factor=1.0)
  conf = nap_conf_module.NAPConf()
  assert conf.pedal_calibrated is False


def test_calibrated_true_when_zero_differs_from_default(tmp_conf):
  _write(tmp_conf, pedal_calibrated=True, pedal_calib_zero=0.25, pedal_calib_factor=1.0)
  conf = nap_conf_module.NAPConf()
  assert conf.pedal_calibrated is True


def test_calibrated_true_when_factor_differs_from_default(tmp_conf):
  _write(tmp_conf, pedal_calibrated=True, pedal_calib_zero=0.0, pedal_calib_factor=0.035)
  conf = nap_conf_module.NAPConf()
  assert conf.pedal_calibrated is True


def test_calibrated_false_when_flag_unset_even_if_values_differ(tmp_conf):
  _write(tmp_conf, pedal_calibrated=False, pedal_calib_zero=0.25, pedal_calib_factor=0.035)
  conf = nap_conf_module.NAPConf()
  assert conf.pedal_calibrated is False
