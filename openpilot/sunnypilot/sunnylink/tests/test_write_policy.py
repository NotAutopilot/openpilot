from openpilot.sunnypilot.sunnylink.write_policy import (
  DEVICE_OWNED_IDENTITY_KEYS,
  evaluate_ordered_writes,
  evaluate_param_write,
)


PREAP = {"tesla_preap": True, "tesla_preap_independent_brake": True}
PREAP_COUPLED = {"tesla_preap": True, "tesla_preap_independent_brake": False}
MODERN = {"tesla_preap": False}


def test_live_follow_distance():
  assert evaluate_param_write("NAPFollowDistance", 4, PREAP).allow
  assert evaluate_param_write("NAPFollowDistance", 1, PREAP).allow
  assert evaluate_param_write("NAPFollowDistance", 7, PREAP).allow
  assert not evaluate_param_write("NAPFollowDistance", 0, PREAP).allow
  assert not evaluate_param_write("NAPFollowDistance", 8, PREAP).allow


def test_next_drive_engagement_and_brake():
  assert evaluate_param_write("NAPLateralEngagementMode", 0, PREAP).allow
  assert evaluate_param_write("NAPLateralEngagementMode", 2, PREAP).allow
  assert not evaluate_param_write("NAPLateralEngagementMode", 3, PREAP).allow
  assert evaluate_param_write("MadsSteeringMode", 1, PREAP).allow
  assert not evaluate_param_write("MadsSteeringMode", 1, PREAP_COUPLED).allow


def test_rejects_hardware_and_hidden_preap_keys():
  for key in ("NAPPedalEnabled", "NAPRadarOffset", "NAPForcePreAP", "NAPPedalCalibFactor",
              "NAPAdaptiveAccel", "NAPBrakeFactor", "NAPScriptRunning", "NAPEpasRiskAccepted"):
    assert not evaluate_param_write(key, 1, PREAP).allow
  for key in ("TeslaCoopSteering", "TeslaMadsScreenButton", "Mads", "MadsMainCruiseAllowed", "MadsUnifiedEngagementMode"):
    assert not evaluate_param_write(key, True, PREAP).allow


def test_modern_tesla_unchanged_for_coop_and_mads():
  assert evaluate_param_write("TeslaCoopSteering", True, MODERN).allow
  assert evaluate_param_write("Mads", True, MODERN, mads_required=False).allow
  assert not evaluate_param_write("Mads", False, MODERN, mads_required=True).allow


def test_preap_only_keys_rejected_on_modern():
  assert not evaluate_param_write("NAPFollowDistance", 4, MODERN).allow
  assert not evaluate_param_write("NAPLateralEngagementMode", 0, MODERN).allow
  assert evaluate_param_write("MadsSteeringMode", 1, MODERN).allow


def test_hardware_rejected_on_modern_and_preap():
  for caps in (PREAP, MODERN):
    assert not evaluate_param_write("NAPRadarOffset", 0.1, caps).allow
    assert not evaluate_param_write("NAPPedalEnabled", True, caps).allow


def test_hostile_batch_mode_then_brake_fails_closed():
  results = evaluate_ordered_writes(
    [("NAPLateralEngagementMode", 1), ("MadsSteeringMode", 1)],
    PREAP,
    staged_mode=0,
  )
  assert results[0][2].allow
  assert not results[1][2].allow
  assert results[1][2].reason == "brake_mode_not_independent"


def test_hostile_batch_brake_then_mode_is_order_independent():
  results = evaluate_ordered_writes(
    [("MadsSteeringMode", 1), ("NAPLateralEngagementMode", 1)],
    PREAP,
    staged_mode=0,
  )
  assert not results[0][2].allow
  assert results[0][2].reason == "brake_mode_not_independent"
  assert results[1][2].allow


def test_device_owned_identity_keys_are_rejected():
  for key in DEVICE_OWNED_IDENTITY_KEYS:
    decision = evaluate_param_write(key, b"spoof", PREAP)
    assert not decision.allow, key
    assert decision.reason == "device_owned_identity"


def test_read_only_status_keys_are_local_only():
  for key in ("tesla_preap_active_mode", "tesla_preap_longitudinal_path",
              "tesla_preap_pedal_health", "tesla_preap_radar_health"):
    assert not evaluate_param_write(key, "ok", PREAP).allow
