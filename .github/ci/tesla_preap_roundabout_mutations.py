"""Roundabout map + Roundabout Steering Assist mutations for the Tesla Pre-AP gate.

Kept separate from tesla_preap_longitudinal_mutations.py (which stays the parent list).
Reuses that file's HistoricalMutation / apply / pytest / JUnit helpers so a mutation is
KILLED only by assertion failures, sources are restored byte-identically, and any survivor
or invalid run fails the job.
"""
import importlib.util
import sys
import tempfile
from pathlib import Path


_PARENT_PATH = Path(__file__).resolve().parent / "tesla_preap_longitudinal_mutations.py"
_spec = importlib.util.spec_from_file_location("tesla_preap_longitudinal_mutations", _PARENT_PATH)
assert _spec is not None and _spec.loader is not None
parent = importlib.util.module_from_spec(_spec)
sys.modules[_spec.name] = parent
_spec.loader.exec_module(parent)

HistoricalMutation = parent.HistoricalMutation
RB_GUIDE_TEST_PATH = "selfdrive/controls/lib/tests/test_roundabout_guide.py"
RB_MAP_TEST_PATH = "selfdrive/mapd/tests/test_roundabout_map.py"
RB_HOLD_TEST_PATH = "selfdrive/controls/lib/tests/test_roundabout_ring_hold.py"
RB_MAPD_TEST_PATH = "selfdrive/mapd/tests/test_roundabout.py"
RH_PATH = "selfdrive/controls/lib/roundabout_ring_hold.py"
RG_PATH = "selfdrive/controls/lib/roundabout_guide.py"
EXPECTED_COUNT = 110


MUTATIONS = (
  HistoricalMutation(
    name="rb-guide-early-left-before-turn-distance",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'      k_raw = max(model_k, k_pp)   # hold / add right only; never an early left\n',
    replacement=b'      k_raw = k_pp\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_right_entry_held_until_turn_distance",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-correction-not-slewed",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    self.dk_out += min(max(dk - self.dk_out, -oslew * dt), oslew * dt)\n',
    replacement=b'    self.dk_out = dk\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_left_transition_within_jerk_limit",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-output-exceeds-jerk-limit",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'      allowed = max(oslew * dt, abs(model_k - self.prev_model))\n',
    replacement=b'      allowed = 1.0\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_left_transition_within_jerk_limit",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-no-preview",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    px, py, ppsi = _arc_ahead(x, y, psi, v, yaw_rate, PREVIEW_S)\n',
    replacement=b'    px, py, ppsi = x, y, psi\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_ring_follow_keeps_the_passes_in_the_ring_band",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-circle-follow-wrong-sign",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'  return -sense * k_center\n',
    replacement=b'  return sense * k_center\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_circle_follow_sign_convention",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-no-map-bias",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    bn += BIAS_GAIN * (ex - bn)\n',
    replacement=b'    bn += 0.0\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_map_match_bias_removes_gnss_offset_beyond_a_lane",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-ignores-fix-age",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    c_fix = _interp(t - self.last_fix_t, (FIX_FRESH_S, FIX_STALE_S), (1.0, 0.0))\n',
    replacement=b'    c_fix = 1.0\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_pose_error_lowers_confidence_and_fades",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-ignores-camera-edges",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    dy = max(dy, min(0.0, allowed))\n',
    replacement=b'    dy = dy\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_edge_limit_keeps_camera_authority",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-never-hands-back",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    if self.phase == EXITING and self.phase_w <= 0.0 and abs(self.dk_out) < HANDOFF_DK:\n',
    replacement=b'    if False:\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_hands_back_to_model_after_exit",
    ),
  ),
  HistoricalMutation(
    name="rb-assist-ignores-toggle",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'          on = bool(self.params.get_bool(PARAM_ROUNDABOUT_ASSIST))\n',
    replacement=b'          on = True\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_identity_when_off_or_not_preap",
    ),
  ),
  HistoricalMutation(
    name="rb-assist-not-preap-only",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    if not self.preap:\n      return float(model_k)\n',
    replacement=b'    if False:\n      return float(model_k)\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_identity_when_off_or_not_preap",
    ),
  ),
  HistoricalMutation(
    name="rb-assist-runs-during-maneuver",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    if maneuver_active or lane_change_active:\n      on = False\n',
    replacement=b'    if False:\n      on = False\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_identity_when_driver_has_wheel_or_maneuver",
    ),
  ),
  HistoricalMutation(
    name="rb-assist-without-hint",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    on = (t - self._last_hint_t) <= HINT_HOLD_S or busy\n',
    replacement=b'    on = True\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_stale_ring_does_not_engage",
    ),
  ),
  HistoricalMutation(
    name="rb-controlsd-legacy-bias-stacks-on-assist",
    source_path="selfdrive/controls/controlsd.py",
    original=b'(0.0 if self.rb_assist.active else rb_bias)',
    replacement=b'rb_bias',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_controlsd_applies_assist_before_legacy_bias_and_pins_nothing_else",
    ),
  ),
  HistoricalMutation(
    name="rb-assist-default-on",
    source_path="common/params_keys.h",
    original=b'{"NAPRoundaboutAssist", {PERSISTENT, BOOL, "0"}}',
    replacement=b'{"NAPRoundaboutAssist", {PERSISTENT, BOOL, "1"}}',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_params_and_menu_wiring",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-nan-odometry-poisons-pose",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    if not (math.isfinite(t) and math.isfinite(v) and math.isfinite(yaw_rate)):\n',
    replacement=b'    if False:\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_nan_odometry_sample_does_not_poison_the_pose",
    ),
  ),
  HistoricalMutation(
    name="rb-map-ring-geometry-searches-around-car",
    source_path="selfdrive/mapd/osm_db.py",
    original=b'      lat, lon = 0.5 * (box[0] + box[1]), 0.5 * (box[2] + box[3])\n',
    replacement=b'      lat, lon = float(lat), float(lon)\n',
    test_nodes=(
      f"{RB_MAP_TEST_PATH}::test_v4_pack_ring_geometry_from_first_hint_distance",
    ),
  ),
  HistoricalMutation(
    name="rb-map-find-roundabout-skips-rb-ways",
    source_path="selfdrive/mapd/osm_db.py",
    original=b'    rows += [r for r in self._rb_candidates(float(lat), float(lon), RB_SEARCH_PAD_DEG, role=ROLE_RING)\n',
    replacement=b'    rows += [r for r in []\n',
    test_nodes=(
      f"{RB_MAP_TEST_PATH}::test_v4_pack_find_roundabout_sees_untagged_speed_ring",
    ),
  ),
  HistoricalMutation(
    name="rb-map-overpass-drops-rings",
    source_path="selfdrive/mapd/overpass.py",
    original=b'  rb_union = "" if not include_roundabouts else "\\n  .rb;\\n  .ap;"\n',
    replacement=b'  rb_union = ""\n',
    test_nodes=(
      f"{RB_MAP_TEST_PATH}::test_overpass_queries_include_rings_and_approaches",
    ),
  ),
  HistoricalMutation(
    name="rb-map-refresh-drops-rb-rows",
    source_path="selfdrive/mapd/local_refresh.py",
    original=b'      for row in rb_rows:\n        OsmSpeedLimitDB.insert_rb_way(con, row)\n',
    replacement=b'      for row in []:\n        OsmSpeedLimitDB.insert_rb_way(con, row)\n',
    test_nodes=(
      f"{RB_MAP_TEST_PATH}::test_refresh_merge_replaces_rb_rows_in_bbox",
    ),
  ),
  HistoricalMutation(
    name="rb-map-approach-direction-lost",
    source_path="selfdrive/mapd/roundabout_map.py",
    original=b'    fwd = list(reversed(trimmed)) if inbound else trimmed\n',
    replacement=b'    fwd = trimmed\n',
    test_nodes=(
      f"{RB_MAP_TEST_PATH}::test_approaches_are_stored_in_travel_direction",
    ),
  ),
  HistoricalMutation(
    name="rb-map-comfort-speed-outer-lane",
    source_path="selfdrive/mapd/roundabout_map.py",
    original=b'  r_inner = max(4.0, float(radius_m) - 0.5 * (lanes_n - 1) * LANE_WIDTH_M)\n',
    replacement=b'  r_inner = max(4.0, float(radius_m) + 0.5 * (lanes_n - 1) * LANE_WIDTH_M)\n',
    test_nodes=(
      f"{RB_MAP_TEST_PATH}::test_comfort_speed_soco_about_15_mph",
    ),
  ),
  HistoricalMutation(
    name="rb-map-builder-skips-roundabouts",
    source_path="scripts/nap/build_osm_speed_limits.py",
    original=b'      rb_rows = rb_rows_from_overpass(payload)\n',
    replacement=b'      rb_rows = []\n',
    test_nodes=(
      f"{RB_MAP_TEST_PATH}::test_builder_from_json_writes_pack_v4",
    ),
  ),
)


CARD = "selfdrive/car/card.py"
MAPD = "selfdrive/mapd/mapd.py"
RMAP = "selfdrive/mapd/roundabout_status.py"
T = RB_MAP_TEST_PATH

MUTATIONS += (
  HistoricalMutation(
    name="rb-card-ring-limit-needs-speed-match",
    source_path=CARD,
    original=b"    if rb_hint is not None:\n      posted_ms =",
    replacement=b"    if rb_hint is not None and map_valid and md is not None:\n      posted_ms =",
    test_nodes=(
      f"{T}::test_card_ring_hint_drops_hud_max_without_speed_match",
      f"{T}::test_card_ring_hint_same_max_with_or_without_speed_match",
    ),
  ),
  HistoricalMutation(
    name="rb-card-ring-hint-ignores-message-validity",
    source_path=CARD,
    original=b"    rb_md = self.sm['liveMapDataNAP'] if self.sm.valid.get('liveMapDataNAP', False) else None\n",
    replacement=b"    rb_md = self.sm['liveMapDataNAP']\n",
    test_nodes=(
      f"{T}::test_card_ring_hint_needs_a_valid_map_message",
    ),
  ),
  HistoricalMutation(
    name="rb-card-ring-limit-read-from-speed-match-only",
    source_path=CARD,
    original=b"    rb_hint = live_map_roundabout_hint(rb_md)\n",
    replacement=b"    rb_hint = live_map_roundabout_hint(md)\n",
    test_nodes=(
      f"{T}::test_card_ring_hint_drops_hud_max_without_speed_match",
    ),
  ),
  HistoricalMutation(
    name="rb-card-ring-slowdown-gated-on-assist-toggle",
    source_path=CARD,
    original=b"    rb_hint = live_map_roundabout_hint(rb_md)\n",
    replacement=b"    rb_hint = live_map_roundabout_hint(rb_md) if Params().get_bool('NAPRoundaboutAssist') else None\n",
    test_nodes=(
      f"{T}::test_card_and_mapd_never_read_the_assist_toggle",
    ),
  ),
  HistoricalMutation(
    name="rb-mapd-ring-hint-leaves-message-invalid",
    source_path=MAPD,
    original=b"matched=match is not None, ring_hint=True)",
    replacement=b"matched=match is not None, ring_hint=False)",
    test_nodes=(
      f"{T}::test_mapd_ring_hint_keeps_message_valid_without_speed_match",
    ),
  ),
  HistoricalMutation(
    name="rb-mapd-valid-without-speed-match-off-ring",
    source_path=MAPD,
    original=b"matched=match is not None, ring_hint=False)",
    replacement=b"matched=match is not None, ring_hint=True)",
    test_nodes=(
      f"{T}::test_mapd_ring_hint_keeps_message_valid_without_speed_match",
    ),
  ),
  HistoricalMutation(
    name="rb-map-valid-ignores-ring-hint",
    source_path=RMAP,
    original=b"  return bool(gps_ok and db_loaded and (matched or ring_hint))\n",
    replacement=b"  return bool(gps_ok and db_loaded and matched)\n",
    test_nodes=(
      f"{T}::test_map_msg_valid_only_changes_with_a_ring_hint",
    ),
  ),
  HistoricalMutation(
    name="rb-map-valid-ring-hint-skips-gps-check",
    source_path=RMAP,
    original=b"  return bool(gps_ok and db_loaded and (matched or ring_hint))\n",
    replacement=b"  return bool(db_loaded and (matched or ring_hint))\n",
    test_nodes=(
      f"{T}::test_map_msg_valid_only_changes_with_a_ring_hint",
    ),
  ),
  HistoricalMutation(
    name="rb-map-ring-count-v3-pack-reads-as-none",
    source_path=RMAP,
    original=b"      if row is None:\n        return 0\n",
    replacement=b"      if row is None:\n        return None\n",
    test_nodes=(
      f"{T}::test_pack_ring_count_and_summary",
    ),
  ),
  HistoricalMutation(
    name="rb-map-ring-count-counts-approach-rows",
    source_path=RMAP,
    original=b'"SELECT COUNT(*) FROM rb_ways WHERE role = ?", (ROLE_RING,)',
    replacement=b'"SELECT COUNT(*) FROM rb_ways WHERE role != ?", ("",)',
    test_nodes=(
      f"{T}::test_pack_ring_count_and_summary",
    ),
  ),
  HistoricalMutation(
    name="rb-map-ring-summary-zero-rings-not-flagged",
    source_path=RMAP,
    original=b"  if count <= 0:\n    return RING_MISSING_TEXT\n",
    replacement=b"  if count < 0:\n    return RING_MISSING_TEXT\n",
    test_nodes=(
      f"{T}::test_pack_ring_count_and_summary",
    ),
  ),
  HistoricalMutation(
    name="rb-map-ring-watch-logs-every-check",
    source_path=RMAP,
    original=b"    if key == self._key:\n      return None\n",
    replacement=b"    if key is None:\n      return None\n",
    test_nodes=(
      f"{T}::test_ring_data_watch_logs_once_per_pack_state",
    ),
  ),
  HistoricalMutation(
    name="rb-mapd-never-logs-missing-ring-rows",
    source_path=MAPD,
    original=b"          getattr(cloudlog, note[0])(note[1])\n",
    replacement=b"          pass\n",
    test_nodes=(
      f"{T}::test_mapd_logs_missing_ring_rows_once",
    ),
  ),
  HistoricalMutation(
    name="rb-map-ring-summary-ignores-path",
    source_path=RMAP,
    original=b"pack_ring_count(path or default_db_path())",
    replacement=b"pack_ring_count(default_db_path())",
    test_nodes=(
      f"{T}::test_pack_ring_count_and_summary",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-gps-latency-back-to-0-9",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'GPS_LATENCY_S = 0.66 ',
    replacement=b'GPS_LATENCY_S = 0.9  ',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_gps_latency_is_the_measured_one",
      f"{RB_GUIDE_TEST_PATH}::test_sep29_left_curl_no_longer_doubles_the_model",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-left-transition-a-lat-3-0",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'A_LAT_CIRC_MAX = 2.2 ',
    replacement=b'A_LAT_CIRC_MAX = 3.0 ',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_left_transition_cap_uses_actual_speed",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-output-slew-back-to-0-07",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'OUT_SLEW = 0.035 ',
    replacement=b'OUT_SLEW = 0.07  ',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_slews_are_0_035_for_a_poor_pose",
      f"{RB_GUIDE_TEST_PATH}::test_poor_pose_uses_the_slow_slews",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-bearing-accuracy-never-unreliable",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'  unreliable = bearing_acc_deg > BEARING_ACC_UNRELIABLE_DEG or \\\n',
    replacement=b'  unreliable = False or \\\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_pose_quality_thresholds",
      f"{RB_GUIDE_TEST_PATH}::test_unreliable_or_unsettled_pose_does_not_latch",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-large-bias-never-unreliable",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    (bias_m > BIAS_UNRELIABLE_M and bearing_acc_deg > BEARING_ACC_POOR_DEG)\n',
    replacement=b'    False\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_pose_quality_thresholds",
      f"{RB_GUIDE_TEST_PATH}::test_unreliable_or_unsettled_pose_does_not_latch",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-latches-on-unreliable-pose",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    can_latch = not unreliable and drift <= BIAS_LATCH_DRIFT_M\n',
    replacement=b'    can_latch = True\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_unreliable_or_unsettled_pose_does_not_latch",
      f"{RB_GUIDE_TEST_PATH}::test_sep29_left_curl_no_longer_doubles_the_model",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-latches-before-bias-settles",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    can_latch = not unreliable and drift <= BIAS_LATCH_DRIFT_M\n',
    replacement=b'    can_latch = not unreliable\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_unreliable_or_unsettled_pose_does_not_latch",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-fades-nothing-on-unreliable-pose",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'      phase_target = 0.0 if (unreliable and not self.latched) else 1.0\n',
    replacement=b'      phase_target = 1.0\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_sep29_left_curl_no_longer_doubles_the_model",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-curl-not-held-over-ring-speed",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'      k_raw = hold_curl(k_raw, model_k, sense)\n',
    replacement=b'      pass\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_curl_held_while_over_ring_speed_on_the_fixtures",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-hold-curl-wrong-side",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'  return max(model_k, k_raw) if sense > 0.0 else min(model_k, k_raw)\n',
    replacement=b'  return min(model_k, k_raw) if sense > 0.0 else max(model_k, k_raw)\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_curl_is_held_until_within_3_mph_of_ring_speed",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-hold-speed-window-removed",
    source_path="selfdrive/controls/lib/roundabout_guide.py",
    original=b'    holding = v > self.ring_v + SPEED_HOLD_MS\n',
    replacement=b'    holding = False\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_curl_held_while_over_ring_speed_on_the_fixtures",
      f"{RB_GUIDE_TEST_PATH}::test_curl_is_held_until_within_3_mph_of_ring_speed",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-left-cap-ignores-circulation-side",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'  k_circ, k_other = A_LAT_CIRC_MAX / v2, A_LAT_MAX / v2\n',
    replacement=b'  k_circ, k_other = A_LAT_MAX / v2, A_LAT_MAX / v2\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_left_transition_cap_uses_actual_speed",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-poor-pose-uses-fast-output-slew",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'    oslew = RING_SLEW if self.latched else (OUT_SLEW if poor else OUT_SLEW_GOOD)\n',
    replacement=b'    oslew = RING_SLEW if self.latched else OUT_SLEW_GOOD\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_poor_pose_uses_the_slow_slews",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-poor-pose-uses-fast-target-slew",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'    slew = RING_SLEW if self.latched else (TARGET_SLEW if poor else TARGET_SLEW_GOOD)\n',
    replacement=b'    slew = RING_SLEW if self.latched else TARGET_SLEW_GOOD\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_poor_pose_target_slew",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-inner-lane-on-poor-pose",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'      inner = r <= R and not (poor or bias_m > BIAS_UNRELIABLE_M)           # doubtful pose: never the inner lane\n',
    replacement=b'      inner = r <= R\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_poor_pose_latch_never_aims_at_the_inner_side",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-ring-state-not-logged",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'      cloudlog.error(f"roundabout_assist {e}")\n',
    replacement=b'      pass\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_ring_state_is_logged_via_cloudlog_without_capnp",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-ring-state-logged-below-qlog-level",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'      cloudlog.error(f"roundabout_assist {e}")\n',
    replacement=b'      cloudlog.warning(f"roundabout_assist {e}")\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_ring_state_is_logged_via_cloudlog_without_capnp",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-raw-model-summary-not-logged",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'      cloudlog.error("roundabout_raw " + raw_ring_summary(g, t, v_ego, model_k, out, model_v2, driver))\n',
    replacement=b'      pass\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_raw_model_summary_logged_near_and_in_the_ring",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-raw-model-summary-logged-far-from-ring",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'    near = on and g.d_edge is not None and g.d_edge < RAW_LOG_EDGE_M and g.phase != DONE\n',
    replacement=b'    near = on\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_raw_model_summary_logged_near_and_in_the_ring",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-ring-cap-ratio-1-5",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'RING_CAP_RATIO = 1.15 ',
    replacement=b'RING_CAP_RATIO = 1.5  ',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_ring_cap_is_1_15x_the_model_and_2_2_mps2",
      f"{RB_GUIDE_TEST_PATH}::test_ring_output_never_above_1_15x_the_model_or_2_2_after_the_latch",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-ring-a-lat-cap-3-0",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'RING_A_LAT_MAX = 2.2 ',
    replacement=b'RING_A_LAT_MAX = 3.0 ',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_ring_cap_is_1_15x_the_model_and_2_2_mps2",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-ring-slew-0-07",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'RING_SLEW = 0.035 ',
    replacement=b'RING_SLEW = 0.07  ',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_ring_cap_is_1_15x_the_model_and_2_2_mps2",
      f"{RB_GUIDE_TEST_PATH}::test_ring_output_slews_at_most_0_035_after_the_latch",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-ring-floor-removed",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'RING_FLOOR_RATIO = 0.85 ',
    replacement=b'RING_FLOOR_RATIO = 0.0  ',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_ring_cap_is_1_15x_the_model_and_2_2_mps2",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-ring-cap-not-lateral-limited",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'  return max(s_model, min(RING_CAP_RATIO * max(ref_k, s_model) + RING_CAP_FLOOR, RING_A_LAT_MAX / max(v * v, 1.0)))\n',
    replacement=b'  return max(s_model, RING_CAP_RATIO * max(ref_k, s_model) + RING_CAP_FLOOR)\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_ring_cap_is_1_15x_the_model_and_2_2_mps2",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-ring-cap-steers-against-the-model",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'    return s_model          # model not curving with the circulation: the assist never steers against it\n',
    replacement=b'    return 0.0\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_ring_cap_is_1_15x_the_model_and_2_2_mps2",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-ring-cap-skipped-on-poor-pose",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'      base = -sense * model_k if holding else ring_cap(self.k_ref, model_k, v, sense)\n',
    replacement=b'      base = -sense * model_k if holding else (1e9 if poor else ring_cap(self.k_ref, model_k, v, sense))\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_ring_output_never_above_1_15x_the_model_or_2_2_after_the_latch",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-ring-hard-clamp-removed",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'      out = -sense * min(-sense * (model_k + self.dk_out), cap)\n',
    replacement=b'      out = model_k + self.dk_out\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_ring_output_never_above_1_15x_the_model_or_2_2_after_the_latch",
      f"{RB_GUIDE_TEST_PATH}::test_curl_held_while_over_ring_speed_on_the_fixtures",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-lane-circle-is-the-center-line",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'        self.r_lane = R - LANE_CIRCLE_FRAC * hw\n',
    replacement=b'        self.r_lane = R\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_ring_assist_follows_the_lane_circle_once_latched",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-latched-confidence-weight-only",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'    w = self.phase_w if self.latched else W_MAX * conf * self.phase_w     # latched: the cap, not the confidence, bounds the assist\n',
    replacement=b'    w = W_MAX * conf * self.phase_w\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_latched_weight_does_not_depend_on_pose_confidence",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-stalk-releases-wrong-side",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'    exit_dir = 2 if sense > 0.0 else 1                      # exit side: right on a CCW ring\n',
    replacement=b'    exit_dir = 1 if sense > 0.0 else 2\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_held_right_stalk_releases_the_ring_and_hands_back_at_once",
      f"{RB_GUIDE_TEST_PATH}::test_held_left_stalk_does_not_release_but_the_guide_weight_is_zero",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-stalk-never-releases",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'    if drv.stalk_held and drv.stalk_dir == exit_dir:\n',
    replacement=b'    if False:\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_held_right_stalk_releases_the_ring_and_hands_back_at_once",
      f"{RB_GUIDE_TEST_PATH}::test_latched_param_and_stalk_release_through_the_assist",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-any-stalk-releases",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'    if drv.stalk_held and drv.stalk_dir == exit_dir:\n',
    replacement=b'    if drv.stalk_held:\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_held_left_stalk_does_not_release_but_the_guide_weight_is_zero",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-pull-wrong-direction",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'    self.pull_s = self.pull_s + dt if -sense * drv.torque >= PULL_TORQUE_NM else 0.0\n',
    replacement=b'    self.pull_s = self.pull_s + dt if sense * drv.torque >= PULL_TORQUE_NM else 0.0\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_steering_pull_toward_the_exit_releases_after_0_3_s",
      f"{RB_GUIDE_TEST_PATH}::test_steering_pull_toward_the_circulation_does_not_release_but_wins",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-pull-never-releases",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'    if self.pull_s >= PULL_RELEASE_S:\n',
    replacement=b'    if False:\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_steering_pull_toward_the_exit_releases_after_0_3_s",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-override-never-releases",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'    if self.override_s >= OVERRIDE_S:\n',
    replacement=b'    if False:\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_steering_press_zeroes_the_weight_after_0_3_s_and_overrides_after_0_5_s",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-map-only-exit",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'    if v > V_MAX + 2.0:\n      return "speed"\n',
    replacement=b'    if self.travel > 60.0:\n      return "map_exit"\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_no_blinker_no_map_exit_the_car_stays_on_the_ring",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-driver-torque-does-not-win",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'    drv_active, self.drv_s = RH.driver_wins(drv.stalk_held, drv.pressed, drv.torque, sense, self.drv_s, dt)\n',
    replacement=b'    drv_active, self.drv_s = bool(drv.stalk_held or drv.pressed), 0.0\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_steering_pull_toward_the_exit_releases_after_0_3_s",
      f"{RB_GUIDE_TEST_PATH}::test_steering_pull_toward_the_circulation_does_not_release_but_wins",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-held-stalk-keeps-the-assist",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'    if not lat_active or drv_active:\n',
    replacement=b'    if not lat_active:\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_held_left_stalk_does_not_release_but_the_guide_weight_is_zero",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-held-stalk-weight-not-zero",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'    if drv_active:\n      w = 0.0\n',
    replacement=b'    if False:\n      w = 0.0\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_held_left_stalk_does_not_release_but_the_guide_weight_is_zero",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-latched-param-not-written",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'      self.params.put_bool(PARAM_RING_LATCHED, latched)\n',
    replacement=b'      pass\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_latched_param_and_stalk_release_through_the_assist",
      f"{RB_GUIDE_TEST_PATH}::test_off_while_latched_resets_and_clears_the_param",
    ),
  ),
  HistoricalMutation(
    name="rb-guide-off-while-latched-keeps-the-ring",
    source_path='selfdrive/controls/lib/roundabout_guide.py',
    original=b'      if self.guide.latched:\n        self.guide.reset_state()      # toggled Off on the ring: nothing carried over\n',
    replacement=b'      if False:\n        self.guide.reset_state()\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_off_while_latched_resets_and_clears_the_param",
    ),
  ),
  HistoricalMutation(
    name="rb-desire-helper-tips-not-suppressed-on-ring",
    source_path='selfdrive/controls/lib/desire_helper.py',
    original=b'    tip_event = self._tip_turn.tip_event and not self._suppress_next_tip and not self.suppress_tips\n',
    replacement=b'    tip_event = self._tip_turn.tip_event and not self._suppress_next_tip\n',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_tips_are_suppressed_while_latched_and_modeld_controlsd_are_wired",
    ),
  ),
  HistoricalMutation(
    name="rb-modeld-ring-latch-not-read",
    source_path='selfdrive/modeld/modeld.py',
    original=b'        DH.suppress_tips = params.get_bool(PARAM_RING_LATCHED)',
    replacement=b'        DH.suppress_tips = False',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_tips_are_suppressed_while_latched_and_modeld_controlsd_are_wired",
    ),
  ),
  HistoricalMutation(
    name="rb-controlsd-driver-stalk-not-passed",
    source_path='selfdrive/controls/controlsd.py',
    original=b"stalk_state=int(getattr(CS, 'turnSignalStalkState', 0) or 0)",
    replacement=b'stalk_state=0',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_tips_are_suppressed_while_latched_and_modeld_controlsd_are_wired",
    ),
  ),
  HistoricalMutation(
    name="rb-controlsd-driver-torque-not-passed",
    source_path='selfdrive/controls/controlsd.py',
    original=b'steering_pressed=bool(CS.steeringPressed), steering_torque=float(CS.steeringTorque)',
    replacement=b'steering_pressed=False, steering_torque=0.0',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_tips_are_suppressed_while_latched_and_modeld_controlsd_are_wired",
    ),
  ),
  HistoricalMutation(
    name="rb-latched-param-persists-across-drives",
    source_path='common/params_keys.h',
    original=b'{"NAPRoundaboutLatched", {CLEAR_ON_MANAGER_START | CLEAR_ON_ONROAD_TRANSITION, BOOL}}',
    replacement=b'{"NAPRoundaboutLatched", {PERSISTENT, BOOL}}',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_tips_are_suppressed_while_latched_and_modeld_controlsd_are_wired",
    ),
  ),
  HistoricalMutation(
    name="rb-ring-speed-back-to-15-8-mph",
    source_path='selfdrive/mapd/roundabout.py',
    original=b'RB_RING_SPEED_MPH = 18.0',
    replacement=b'RB_RING_SPEED_MPH = 15.8',
    test_nodes=(
      f"{RB_GUIDE_TEST_PATH}::test_ring_speed_constant_is_18_mph_and_planner_uses_it",
      f"{RB_GUIDE_TEST_PATH}::test_ring_speed_is_18_mph_and_lateral_cap",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-cap-never-widens",
    source_path=RH_PATH,
    original=b'  if s_circle <= 0.0 or s_model >= CIRCLE_CAP_FRAC * s_circle:\n',
    replacement=b'  if True:\n',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_widen_cap_only_below_half_the_circle_and_bounded",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-cap-widens-above-half",
    source_path=RH_PATH,
    original=b'  if s_circle <= 0.0 or s_model >= CIRCLE_CAP_FRAC * s_circle:\n',
    replacement=b'  if s_circle <= 0.0 or s_model >= 2.0 * s_circle:\n',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_widen_cap_only_below_half_the_circle_and_bounded",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-cap-extra-unbounded",
    source_path=RH_PATH,
    original=b's_model + CIRCLE_CAP_EXTRA * fade, a_lat_max',
    replacement=b's_model + 9.0 * fade, a_lat_max',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_widen_cap_only_below_half_the_circle_and_bounded",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-cap-ignores-lateral-accel-cap",
    source_path=RH_PATH,
    original=b'  return max(cap, min(s_circle, s_model + CIRCLE_CAP_EXTRA * fade, a_lat_max / max(v * v, 1.0)))\n',
    replacement=b'  return max(cap, min(s_circle, s_model + CIRCLE_CAP_EXTRA * fade))\n',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_widen_cap_only_below_half_the_circle_and_bounded",
      f"{RB_HOLD_TEST_PATH}::test_cap_against_the_circle_when_the_model_is_flat_but_still_2_2_and_slew_bound",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-cap-widening-steps-at-half-circle",
    source_path=RH_PATH,
    original=b'  fade = min(max(1.0 - s_model / (CIRCLE_CAP_FRAC * s_circle), 0.0), 1.0)\n',
    replacement=b'  fade = 1.0\n',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_widen_cap_only_below_half_the_circle_and_bounded",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-exit-floor-removed",
    source_path=RH_PATH,
    original=b'    floor = max(floor, min(s_circle, a_lat_max / max(v * v, 1.0)))\n',
    replacement=b'    floor = floor\n',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_ring_limits_floor_only_in_the_exit_window_without_a_stalk",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-exit-floor-ignores-stalk",
    source_path=RG_PATH,
    original=b'      plain = drv.stalk_dir == (2 if sense > 0.0 else 1) or d_edge',
    replacement=b'      plain = d_edge',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_exit_window_floor_keeps_the_circle_target_without_a_stalk_and_not_with_one",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-exit-window-never-applied",
    source_path=RG_PATH,
    original=b'RH.in_exit_window(theta, sense, self.exits), plain)',
    replacement=b'False, plain)',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_exit_window_floor_keeps_the_circle_target_without_a_stalk_and_not_with_one",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-exit-window-too-short",
    source_path=RH_PATH,
    original=b'EXIT_BEFORE_DEG = 20.0 ',
    replacement=b'EXIT_BEFORE_DEG = 5.0 ',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_exit_windows_20_deg_before_and_5_after_for_both_senses",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-exit-window-wrong-side",
    source_path=RH_PATH,
    original=b'  return _wrap_deg(sense * (exit_deg - math.degrees(theta_rad)))\n',
    replacement=b'  return _wrap_deg(exit_deg - math.degrees(theta_rad))\n',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_exit_windows_20_deg_before_and_5_after_for_both_senses",
      f"{RB_HOLD_TEST_PATH}::test_cw_ring_mirror_of_the_window_and_the_widening",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-exits-not-computed",
    source_path=RG_PATH,
    original=b'    self.exits = RH.exit_angles(ring)\n',
    replacement=b'    self.exits = []\n',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_exit_angles_are_the_three_soco_branches",
      f"{RB_HOLD_TEST_PATH}::test_exit_window_floor_keeps_the_circle_target_without_a_stalk_and_not_with_one",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-exit-blinker-never-releases",
    source_path=RG_PATH,
    original=b'    if drv.stalk_dir == exit_dir and RH.on_exit_arm(',
    replacement=b'    if False and RH.on_exit_arm(',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_exit_blinker_releases_only_on_the_exit_arm_and_only_to_the_model",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-exit-blinker-releases-either-side",
    source_path=RG_PATH,
    original=b'    if drv.stalk_dir == exit_dir and RH.on_exit_arm(',
    replacement=b'    if drv.stalk_dir != 0 and RH.on_exit_arm(',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_exit_blinker_releases_only_on_the_exit_arm_and_only_to_the_model",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-exit-blinker-releases-anywhere",
    source_path=RH_PATH,
    original=b'  return near_exit(theta_rad, sense, exits) and d_edge >= -EXIT_ARM_EDGE_M\n',
    replacement=b'  return True\n',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_exit_blinker_releases_only_on_the_exit_arm_and_only_to_the_model",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-press-not-debounced",
    source_path=RH_PATH,
    original=b'return bool(stalk_held or exit_pull), drv_s',
    replacement=b'return bool(stalk_held or exit_pull or counts_as_press(pressed, torque, sense)), drv_s',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_press_debounce_and_light_torque",
      f"{RB_HOLD_TEST_PATH}::test_press_flicker_keeps_the_correction_but_a_held_press_and_hard_torque_still_win",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-light-torque-not-ignored",
    source_path=RH_PATH,
    original=b'  return bool(pressed) and not (sense * torque > 0.0)\n',
    replacement=b'  return bool(pressed)\n',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_press_debounce_and_light_torque",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-light-torque-ignored-toward-exit",
    source_path=RH_PATH,
    original=b'not (sense * torque > 0.0)',
    replacement=b'not (-sense * torque > 0.0)',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_press_debounce_and_light_torque",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-hard-override-removed",
    source_path=RH_PATH,
    original=b'bool(stalk_held or exit_pull)',
    replacement=b'bool(stalk_held)',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_press_debounce_and_light_torque",
      f"{RB_HOLD_TEST_PATH}::test_press_flicker_keeps_the_correction_but_a_held_press_and_hard_torque_still_win",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-guide-override-uses-raw-press",
    source_path=RG_PATH,
    original=b'    hands = RH.counts_as_press(drv.pressed, drv.torque, sense) or',
    replacement=b'    hands = drv.pressed or',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_press_flicker_keeps_the_correction_but_a_held_press_and_hard_torque_still_win",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-latch-waits-for-tangent",
    source_path=RH_PATH,
    original=b'  return d_edge < LATCH_EDGE_M and align_deg < max(tangent_deg, LATCH_ALIGN_EDGE_DEG)\n',
    replacement=b'  return d_edge < LATCH_EDGE_M and align_deg < tangent_deg\n',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_latch_ready_earlier_at_the_ring_edge",
      f"{RB_HOLD_TEST_PATH}::test_early_latch_at_45_deg_alignment_and_not_beyond",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-cap-widening-given-up-at-once",
    source_path=RH_PATH,
    original=b'  rate = 1.0 if plain or base + extra > a_cap else fall\n',
    replacement=b'  rate = 1.0\n',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_step_cap_rises_at_once_falls_slowly_and_is_bound_by_a_lat",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-cap-widening-kept-over-ring-speed",
    source_path=RH_PATH,
    original=b'  if holding:\n    return 0.0, base\n',
    replacement=b'  if False:\n    return 0.0, base\n',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_step_cap_rises_at_once_falls_slowly_and_is_bound_by_a_lat",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-guide-caps-against-live-model",
    source_path=RG_PATH,
    original=b'      s_cmd = max(min(-sense * k_pp, lim[0]), lim[1])\n',
    replacement=b'      s_cmd = max(min(-sense * k_pp, ring_cap(self.k_ref, model_k, v, sense)), ring_floor(self.k_ref, v))\n',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_0000013c_holds_the_left_turn_at_the_ring_edge_in_band",
      f"{RB_HOLD_TEST_PATH}::test_cap_against_the_circle_when_the_model_is_flat_but_still_2_2_and_slew_bound",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-guide-widening-past-lateral-accel",
    source_path=RG_PATH,
    original=b'    if self.latched and self.phase != EXITING and self.cap_extra > 0.0:\n',
    replacement=b'    if False:\n',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_0000013c_holds_the_left_turn_at_the_ring_edge_in_band",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-guide-exit-blinker-not-handed-back",
    source_path=RG_PATH,
    original=b'("stalk", "pull", "override", "exit_blinker")',
    replacement=b'("stalk", "pull", "override")',
    test_nodes=(
      f"{RB_HOLD_TEST_PATH}::test_exit_blinker_releases_only_on_the_exit_arm_and_only_to_the_model",
    ),
  ),
  HistoricalMutation(
    name="rb-hold-decel-onset-back-to-200m",
    source_path="selfdrive/mapd/roundabout.py",
    original=b'RB_ONSET_MIN_M = 140.0\n',
    replacement=b'RB_ONSET_MIN_M = 200.0\n',
    test_nodes=(
      f"{RB_MAPD_TEST_PATH}::test_funnel_eases_speed",
      f"{RB_HOLD_TEST_PATH}::test_decel_onset_is_a_named_constant_near_the_ring",
      f"{RB_MAPD_TEST_PATH}::test_funnel_detects_farther_out",
    ),
  ),
)


def main() -> int:
  if len(MUTATIONS) != EXPECTED_COUNT:
    print(f"INVALID: expected {EXPECTED_COUNT} roundabout mutations, found {len(MUTATIONS)}")
    return 1
  with tempfile.TemporaryDirectory(prefix="tesla-preap-roundabout-mutation-") as temp_dir:
    temp_root = Path(temp_dir)
    baseline_xml = temp_root / "baseline.xml"
    baseline = parent.run_pytest((RB_GUIDE_TEST_PATH, RB_MAP_TEST_PATH, RB_HOLD_TEST_PATH, RB_MAPD_TEST_PATH), baseline_xml)
    if baseline.returncode != 0:
      print("BASELINE FAILED: roundabout tests did not pass")
      print(baseline.stdout)
      return 1
    try:
      baseline_testcases = parent.junit_testcases(baseline_xml)
    except parent.JUnitReportError as exc:
      print(f"BASELINE INVALID: {exc}")
      return 1
    print(f"BASELINE PASS: {len(baseline_testcases)} roundabout tests")

    survivors = []
    for mutation in MUTATIONS:
      source_path = None
      original_source = None
      mutation_result = None
      mutation_error = None
      restored = False
      mutation_xml = temp_root / f"{mutation.name}.xml"
      try:
        source_path, original_source = parent.apply_mutation(mutation)
        mutation_result = parent.run_pytest(mutation.test_nodes, mutation_xml)
      except Exception as exc:  # pragma: no cover - failure reporting path
        mutation_error = exc
      finally:
        if source_path is not None and original_source is not None:
          source_path.write_bytes(original_source)
          restored = source_path.read_bytes() == original_source

      if not restored:
        print(f"INVALID: {mutation.name} source restoration was not byte-identical")
        return 1
      if mutation_error is not None:
        print(f"INVALID: {mutation.name} could not run: {mutation_error}")
        return 1
      if mutation_result is None:
        print(f"INVALID: {mutation.name} produced no pytest result")
        return 1
      try:
        mutation_testcases = parent.junit_testcases(mutation_xml)
      except parent.JUnitReportError as exc:
        print(f"INVALID: {mutation.name} {exc}")
        return 1
      if mutation_result.returncode == 1 and parent.has_only_assertion_failures(mutation_testcases):
        print(f"KILLED: {mutation.name} [{', '.join(mutation.test_nodes)}]")
      elif mutation_result.returncode == 0:
        survivors.append(mutation.name)
        print(f"SURVIVED: {mutation.name} [{', '.join(mutation.test_nodes)}]")
      else:
        print(f"INVALID: {mutation.name} exited without assertion-only test failures "
              + f"(pytest status {mutation_result.returncode})")
        print(mutation_result.stdout)
        return 1

    if survivors:
      print(f"Roundabout mutations survived: {', '.join(survivors)}")
      return 1
    print(f"ALL KILLED: {len(MUTATIONS)} roundabout mutations")
    print("RESTORED: every mutated source is byte-identical")
    return 0


if __name__ == "__main__":
  raise SystemExit(main())
