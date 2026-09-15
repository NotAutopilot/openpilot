import numpy as np

from openpilot.sunnypilot.modeld_v2.constants import ModelConstants
from openpilot.sunnypilot.modeld_v2.parse_model_outputs import Parser

LEAD_SELECTION = ModelConstants.LEAD_MHP_SELECTION
LEAD_TRAJ_LEN = ModelConstants.LEAD_TRAJ_LEN
LEAD_WIDTH = ModelConstants.LEAD_WIDTH
LEAD_MHP_N = ModelConstants.LEAD_MHP_N


def _unique_xyva(shape, origin, step):
  idx = np.arange(int(np.prod(shape)), dtype=np.float32).reshape(shape)
  return origin + step * idx


def test_lead_144_concatenated_means_then_logstds():
  batch = 2
  shape = (batch, LEAD_SELECTION, LEAD_TRAJ_LEN, LEAD_WIDTH)
  means = _unique_xyva(shape,
                       np.array([38.0, -1.1, 18.5, -0.15], dtype=np.float32),
                       np.array([0.07, 0.003, 0.025, 0.002], dtype=np.float32))
  stds = _unique_xyva(shape,
                      np.array([1.1, 0.22, 0.55, 0.28], dtype=np.float32),
                      np.array([0.012, 0.0015, 0.006, 0.0025], dtype=np.float32))

  raw = np.concatenate([means.reshape(batch, -1), np.log(stds.reshape(batch, -1))], axis=1)
  parsed = Parser(ignore_missing=True).parse_outputs({'lead': raw})

  np.testing.assert_allclose(parsed['lead'], means, rtol=1e-5, atol=1e-6)
  np.testing.assert_allclose(parsed['lead_stds'], stds, rtol=1e-5, atol=1e-6)


def test_lead_102_weighted_selection_pairs_means_and_stds():
  batch = 2
  hyp_shape = (batch, LEAD_MHP_N, LEAD_TRAJ_LEN, LEAD_WIDTH)
  means = _unique_xyva(hyp_shape,
                       np.array([32.0, -0.9, 14.0, -0.25], dtype=np.float32),
                       np.array([0.11, 0.004, 0.03, 0.003], dtype=np.float32))
  stds = _unique_xyva(hyp_shape,
                      np.array([0.9, 0.18, 0.45, 0.24], dtype=np.float32),
                      np.array([0.01, 0.0012, 0.005, 0.002], dtype=np.float32))

  # Each batch selects both hypotheses, with different winners per batch.
  logits = np.array([
    [[8.0, -8.0, 7.0], [-8.0, 8.0, -7.0]],
    [[-8.0, 8.0, -7.0], [8.0, -8.0, 7.0]],
  ], dtype=np.float32)
  winners = np.array([[0, 1, 0], [1, 0, 1]])

  raw = np.concatenate([
    means.reshape(batch, LEAD_MHP_N, -1),
    np.log(stds.reshape(batch, LEAD_MHP_N, -1)),
    logits,
  ], axis=-1).reshape(batch, -1)

  parsed = Parser(ignore_missing=True).parse_outputs({'lead': raw})
  expected_means = means[np.arange(batch)[:, None], winners]
  expected_stds = stds[np.arange(batch)[:, None], winners]

  np.testing.assert_allclose(parsed['lead'], expected_means, rtol=1e-5, atol=1e-6)
  np.testing.assert_allclose(parsed['lead_stds'], expected_stds, rtol=1e-5, atol=1e-6)
