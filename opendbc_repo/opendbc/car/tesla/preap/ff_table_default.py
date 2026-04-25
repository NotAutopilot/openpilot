"""Default feedforward lookup table for VirtualDAS.

Generated from the existing 3-breakpoint linear interpolation at a grid of
(speed, accel) points. This is the fallback when no data-driven table is
available. The generate_ff_table.py script produces a refined version from
real drive logs.

Table format: SPEED_BP × ACCEL_BP → pedal_di
Zero-torque offset is applied at runtime (not baked into the table).
"""

# Speed breakpoints (m/s) — matches PEDAL_BP from nap_conf
SPEED_BP = [0.0, 5.0, 12.0, 20.0, 30.0, 40.0]

# Acceleration breakpoints (m/s²) — from REGEN_MAX to ACCEL_MAX
ACCEL_BP = [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0, 2.5]

# pedal_di values: DEFAULT_TABLE[speed_idx][accel_idx]
# Computed from: interp(accel, [REGEN_MAX, 0, ACCEL_MAX], [DI_MIN, 0, max_pedal])
# where max_pedal = interp(speed, PEDAL_BP, PEDAL_MAX_VALUES)
# and zero_torque_di = 0 (applied as offset at runtime)
#
# Each row is a speed, each column is an accel request.
#         -1.5   -1.0   -0.5    0.0    0.5    1.0    1.5    2.0    2.5
DEFAULT_TABLE = [
    [-5.00, -3.33, -1.67,  0.00, 10.00, 20.00, 30.00, 40.00, 50.00],  # 0 m/s
    [-5.00, -3.33, -1.67,  0.00, 11.60, 23.20, 34.80, 46.40, 58.00],  # 5 m/s
    [-5.00, -3.33, -1.67,  0.00, 13.20, 26.40, 39.60, 52.80, 66.00],  # 12 m/s
    [-5.00, -3.33, -1.67,  0.00, 14.80, 29.60, 44.40, 59.20, 74.00],  # 20 m/s
    [-5.00, -3.33, -1.67,  0.00, 16.40, 32.80, 49.20, 65.60, 82.00],  # 30 m/s
    [-5.00, -3.33, -1.67,  0.00, 18.00, 36.00, 54.00, 72.00, 90.00],  # 40 m/s
]
