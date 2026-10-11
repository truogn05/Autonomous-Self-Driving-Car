"""Physical parameters, PID gains, and speed limits for the BMW X5 ego vehicle."""

# --- Simulation Timing Defaults ---
DEFAULT_TIMESTEP_MS = 16
DEFAULT_DT_SEC = DEFAULT_TIMESTEP_MS / 1000.0

# --- BMW X5 Physical Geometry (matching BmwX5.proto & BmwX5Wheel.proto) ---
WHEELBASE_M = 2.995
TRACK_FRONT_M = 1.628
TRACK_REAR_M = 1.628
TIRE_RADIUS_M = 0.374
VEHICLE_MASS_KG = 2000.0

# --- Urban Map Speed Limits (calibrated for 576m x 1068m 1:1 city map) ---
MAX_FORWARD_SPEED_KMH = 60.0      # 16.67 m/s urban arterial cruising speed
MAX_REVERSE_SPEED_KMH = -18.0     # -5.00 m/s realistic reverse speed
STOP_SPEED_THRESHOLD_KMH = 0.65   # Threshold below which vehicle is considered stopped

# --- Longitudinal Acceleration & Braking Rates ---
ACCEL_RATE_KMH_S = 18.0           # +5.0 m/s^2 forward throttle ramp (Key UP)
REVERSE_ACCEL_RATE_KMH_S = 10.8   # +3.0 m/s^2 reverse throttle ramp (Key DOWN in R)
DECEL_RATE_KMH_S = 26.0           # -7.2 m/s^2 active deceleration (Key DOWN/UP opposite to motion)
COAST_DECEL_RATE_KMH_S = 3.8      # -1.05 m/s^2 natural rolling + aero drag when coasting
SPACE_BRAKE_DECEL_RATE_KMH_S = 38.0  # -10.5 m/s^2 progressive braking (Key SPACE)
SPACE_BRAKE_RAMP_PER_SEC = 2.8    # Brake pedal pressure ramp rate [0..1 / s]
GEAR_SHIFT_HOLD_SEC = 0.16        # Symmetric pause at 0 km/h in Neutral before shifting D <-> R

# --- Error-Proportional Service & Coast Braking ---
COAST_BRAKE_DEADBAND_KMH = 1.0    # Speed excess deadband before coast engine-braking engages
BRAKE_KP_PER_KMH = 0.08           # Proportional brake intensity per km/h of speed excess
MAX_COAST_BRAKE_INTENSITY = 0.25  # Max gentle brake intensity during coasting
MAX_DECEL_BRAKE_INTENSITY = 0.65  # Max service brake intensity during active DOWN/UP deceleration

# --- Longitudinal Velocity Trim PID Gains (Derivative on Measurement + Anti-Windup) ---
PID_KP = 0.25
PID_KI = 0.06
PID_KD = 0.015
PID_INTEGRAL_LIMIT = 8.0

# --- Ackermann Lateral Steering & Self-Centering Parameters ---
MAX_STEERING_ANGLE_RAD = 0.50     # ~28.6 deg max steering at low speed (R_min ~ 5.48m)
SPEED_SENSITIVITY_KV = 0.011      # delta_max(v) = delta_0 / (1 + k_v * v^2)
STEER_KP = 9.0                    # PD proportional gain for steering rack
STEER_KD = 0.35                   # PD derivative damping gain on steering rack rate
MAX_STEER_RATE_RAD_S = 1.20       # Power steering rack angular rate limit (turning in)
SELF_CENTER_BASE_RATE_RAD_S = 1.40  # Automatic self-centering rate at standstill/low speed
SELF_CENTER_SPEED_GAIN = 1.35     # Caster self-centering rate increase with speed
STEER_DEADBAND_RAD = 0.0015       # Snap-to-zero deadband (~0.08 deg)
INDICATOR_HOLD_SEC = 0.45         # Turn signal latch hold duration after releasing key

# --- Sensor Health & Fail-safe Parameters ---
STARTUP_GRACE_STEPS = 15          # Grace period at startup while sensors initialize
MAX_STALE_SENSOR_STEPS = 10       # Trigger fail-safe brake if critical sensors freeze > 10 steps

