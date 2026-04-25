# Pre-AP pedal accel envelopes, mapped to openpilot Driving Personality toggle.
# Breakpoints are speed in m/s; values are max accel in m/s².
# Based on Tinkla Pedal profiles but reduced for feedforward architecture:
# Tinkla uses PID which naturally dampens the ramp to the ceiling; our kf=1.0
# feedforward passes MPC targets straight through, so the ceilings must be
# lower to get the same feel.
#   aggressive(0) → spirited but controlled
#   standard(1)   → smooth daily driver
#   relaxed(2)    → gentle, minimal push
ACCEL_PREAP_BP = [0.0, 1.3, 7.5, 15.0, 25.0, 40.0]  # m/s
#                  0    3    17    33    56    90  mph
ACCEL_PREAP_PROFILES = {
  0: [0.3, 0.8, 1.1, 1.0, 0.85, 0.7],   # aggressive
  1: [0.3, 0.7, 1.0, 0.9, 0.8, 0.65],   # standard
  2: [0.3, 0.6, 0.9, 0.8, 0.7, 0.55],    # relaxed (former standard values)
}

# When following a lead car, cap positive accel to these values regardless
# of personality. Prevents overshoot → regen → overshoot oscillation.
# Open road uses the full profile above; this only limits follow mode.
ACCEL_PREAP_FOLLOW = [0.3, 0.6, 0.9, 0.8, 0.7, 0.55]

# Feedforward-dominant longitudinal tune (FrogPilot/OPGM Bolt-inspired).
# kp=0: no proportional term — eliminates aEgo sensor noise amplification.
# kf=1.0: full a_target passthrough — the MPC plan (jerk-constrained, smooth)
#   is the dominant control signal.
# ki low: slow integral trim handles steady-state offset (hills, wind, regen).
#   Lower than FP Bolt (0.125-0.33) because our kf=1.0 vs their kf=0.25.
PEDAL_LONG_K_BP = [0.0, 3.0, 6.0, 35.0]
PEDAL_LONG_KP_V = [0.0, 0.0, 0.0, 0.0]
PEDAL_LONG_KI_V = [0.05, 0.08, 0.10, 0.15]

# Virtual DAS inner PID — sits inside carcontroller, corrects the
# feedforward output by tracking (a_cmd - a_ego_future). Conservative
# initial tune: no kp (same philosophy as outer loop), ki faster than
# outer so inner loop settles before outer reacts.
VDAS_INNER_K_BP = [0.0, 5.0, 35.0]
VDAS_INNER_KP_V = [0.0, 0.0, 0.0]
VDAS_INNER_KI_V = [0.3, 0.2, 0.15]

# Delay compensation: predict a_ego this far into the future using
# estimated jerk. Longer at highway speed where powertrain is slower.
VDAS_FUTURE_T_BP = [2.0, 5.0]
VDAS_FUTURE_T_V = [0.30, 0.55]

# a_ego low-pass filter time constant (seconds). Smooths IMU noise
# without adding too much phase lag. Matches Toyota's 0.25s RC.
VDAS_AEGO_FILTER_RC = 0.25
