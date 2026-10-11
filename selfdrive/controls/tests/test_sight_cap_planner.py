"""Sight-distance cap wiring in the longitudinal planner: shadow only, never the command."""
import json

import pytest

from openpilot.selfdrive.controls.lib import longitudinal_planner as lp
from openpilot.selfdrive.controls.tests.test_curve_preview import _set_model_path
from openpilot.selfdrive.controls.tests.test_unified_lead_planner import _own_lead, _planner

DT = 0.05
BEND = (120.0, 60.0, 1.0 / 45.0)


def _drive(mode, v, n, lead=None, shrink_at=None, hook=None):
  planner, inputs, params = _planner(v, accel=0.4)
  params.sight_cap = mode
  if lead is not None:
    _own_lead(inputs, *lead)
  s = 0.0
  out = []
  for k in range(n):
    _set_model_path(inputs["modelV2"], v, s, BEND)
    pos = inputs["modelV2"].position
    n_pts = len(pos.x)
    if shrink_at is not None and k >= shrink_at:
      pos.yStd = [min(6.0, 0.4 * i) for i in range(n_pts)]
    else:
      pos.yStd = [0.02 * i for i in range(n_pts)]
    if hook is not None:
      hook(k, planner, inputs)
    planner.update(inputs)
    out.append(float(planner.output_a_target))
    s += v * DT
  return planner, out


@pytest.mark.parametrize("lead", [None, (45.0, 14.0)])
def test_shadow_never_changes_the_command(lead):
  _, off = _drive(0, 18.0, 400, lead=lead, shrink_at=300)
  p_on, on = _drive(1, 18.0, 400, lead=lead, shrink_at=300)
  assert off == on
  assert not p_on._sc_faulted


def test_shadow_computes_sight_and_logs_without_capnp_fields(monkeypatch):
  lines = []
  monkeypatch.setattr(lp.cloudlog, "error", lambda msg, *a, **k: lines.append(msg))
  planner, _ = _drive(1, 18.0, 400, shrink_at=300)
  assert not planner._sc_faulted
  assert planner._sight_cap.sight is not None
  for ln in [x for x in lines if isinstance(x, str) and x.startswith("sightcap ")]:
    rec = json.loads(ln[len("sightcap "):])
    for key in ("lead", "pitch", "crest", "night", "llp", "bl", "cn", "sd", "vc", "cmb"):
      assert key in rec


def test_off_does_not_compute():
  planner, _ = _drive(0, 18.0, 60)
  assert planner._sight_cap.sight is None
