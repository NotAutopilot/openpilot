#!/usr/bin/env python3
import numpy as np

from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import COMFORT_BRAKE, LEAD_DANGER_FACTOR, STOP_DISTANCE, get_jerk_factor, get_T_FOLLOW

class FrogPilotFollowing:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.following_lead = False

    self.acceleration_jerk = 0
    self.danger_jerk = 0
    self.speed_jerk = 0
    self.t_follow = 0

  def update(self, long_control_active, v_ego, sm, frogpilot_toggles):
    if long_control_active:
      if sm["carState"].aEgo >= 0:
        self.base_acceleration_jerk, self.base_danger_jerk, self.base_speed_jerk = get_jerk_factor(
          frogpilot_toggles.aggressive_jerk_acceleration, frogpilot_toggles.aggressive_jerk_danger, frogpilot_toggles.aggressive_jerk_speed,
          frogpilot_toggles.standard_jerk_acceleration, frogpilot_toggles.standard_jerk_danger, frogpilot_toggles.standard_jerk_speed,
          frogpilot_toggles.relaxed_jerk_acceleration, frogpilot_toggles.relaxed_jerk_danger, frogpilot_toggles.relaxed_jerk_speed,
          frogpilot_toggles.custom_personalities, sm["selfdriveState"].personality
        )
      else:
        self.base_acceleration_jerk, self.base_danger_jerk, self.base_speed_jerk = get_jerk_factor(
          frogpilot_toggles.aggressive_jerk_deceleration, frogpilot_toggles.aggressive_jerk_danger, frogpilot_toggles.aggressive_jerk_speed_decrease,
          frogpilot_toggles.standard_jerk_deceleration, frogpilot_toggles.standard_jerk_danger, frogpilot_toggles.standard_jerk_speed_decrease,
          frogpilot_toggles.relaxed_jerk_deceleration, frogpilot_toggles.relaxed_jerk_danger, frogpilot_toggles.relaxed_jerk_speed_decrease,
          frogpilot_toggles.custom_personalities, sm["selfdriveState"].personality
        )

      self.t_follow = get_T_FOLLOW(
        frogpilot_toggles.aggressive_follow,
        frogpilot_toggles.standard_follow,
        frogpilot_toggles.relaxed_follow,
        frogpilot_toggles.custom_personalities, sm["selfdriveState"].personality
      )
    else:
      self.base_acceleration_jerk = 0
      self.base_danger_jerk = 0
      self.base_speed_jerk = 0
      self.t_follow = 0

    self.acceleration_jerk = self.base_acceleration_jerk
    self.danger_factor = LEAD_DANGER_FACTOR
    self.danger_jerk = self.base_danger_jerk
    self.speed_jerk = self.base_speed_jerk

    self.following_lead = self.frogpilot_planner.tracking_lead and self.frogpilot_planner.lead_one.dRel < (self.t_follow * 2) * v_ego

    if long_control_active and self.frogpilot_planner.tracking_lead:
      if frogpilot_toggles.human_following:
        self.update_follow_values(self.frogpilot_planner.lead_one.dRel, v_ego, self.frogpilot_planner.lead_one.vLead)

  def update_follow_values(self, lead_distance, v_ego, v_lead):
    # Offset by FrogAi for FrogPilot for a more natural approach to a faster lead
    if v_lead > v_ego:
      distance_factor = max(lead_distance - (v_ego * self.t_follow), 1)
      accelerating_offset = np.clip(STOP_DISTANCE - v_ego, 1, distance_factor)

      self.acceleration_jerk /= accelerating_offset
      self.danger_factor -= ((v_lead - v_ego) / 100)
      self.speed_jerk /= accelerating_offset
      self.t_follow /= accelerating_offset

    # Offset by FrogAi for FrogPilot for a more natural approach to a slower lead
    if v_lead < v_ego:
      distance_factor = max(lead_distance - (v_lead * self.t_follow), 1)
      braking_offset = np.clip(min(v_ego - v_lead, v_lead) - COMFORT_BRAKE, 1, distance_factor)

      if lead_distance >= 100:
        far_lead_offset = max(lead_distance - (v_ego * self.t_follow) - STOP_DISTANCE, 0)
        braking_offset += far_lead_offset

      if self.frogpilot_planner.tracking_lead_filter.x >= 0.9:
        self.danger_factor += ((v_ego - v_lead) / 100)

      self.t_follow /= braking_offset
