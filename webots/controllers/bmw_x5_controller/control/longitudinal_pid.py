"""Longitudinal speed control with anti-windup PID trim, symmetric D/N/R state machine, and error-based braking."""

from dataclasses import dataclass
from config.vehicle_params import (
    ACCEL_RATE_KMH_S,
    BRAKE_KP_PER_KMH,
    COAST_BRAKE_DEADBAND_KMH,
    COAST_DECEL_RATE_KMH_S,
    DECEL_RATE_KMH_S,
    DEFAULT_DT_SEC,
    GEAR_SHIFT_HOLD_SEC,
    MAX_COAST_BRAKE_INTENSITY,
    MAX_DECEL_BRAKE_INTENSITY,
    MAX_FORWARD_SPEED_KMH,
    MAX_REVERSE_SPEED_KMH,
    PID_INTEGRAL_LIMIT,
    PID_KD,
    PID_KI,
    PID_KP,
    REVERSE_ACCEL_RATE_KMH_S,
    SPACE_BRAKE_DECEL_RATE_KMH_S,
    SPACE_BRAKE_RAMP_PER_SEC,
    STOP_SPEED_THRESHOLD_KMH,
)


@dataclass
class LongitudinalOutput:
    """Output commands from the longitudinal controller."""
    target_speed_kmh: float
    commanded_cruising_speed_kmh: float
    brake_intensity: float
    gear: str  # "D", "N", "R"
    pid_correction_kmh: float


class LongitudinalPIDController:
    """Longitudinal controller with derivative-on-measurement PID, symmetric D/N/R hold, and error-proportional braking."""

    def __init__(self) -> None:
        self.target_speed_kmh: float = 0.0
        self.brake_intensity: float = 0.0
        self.gear: str = "N"
        self.integral_error: float = 0.0
        self.prev_meas: float = 0.0
        self.pid_active: bool = False
        self.shift_hold_timer: float = 0.0

    def reset(self) -> None:
        self.target_speed_kmh = 0.0
        self.brake_intensity = 0.0
        self.gear = "N"
        self.integral_error = 0.0
        self.prev_meas = 0.0
        self.pid_active = False
        self.shift_hold_timer = 0.0

    def update(
        self,
        dt_sec: float,
        up_pressed: bool,
        down_pressed: bool,
        space_pressed: bool,
        measured_speed_kmh: float,
    ) -> LongitudinalOutput:
        if dt_sec <= 0.0:
            dt_sec = DEFAULT_DT_SEC

        # 1. Progressive Space Brake (highest priority)
        if space_pressed:
            self.brake_intensity = min(
                1.0, self.brake_intensity + SPACE_BRAKE_RAMP_PER_SEC * dt_sec
            )
            decel = SPACE_BRAKE_DECEL_RATE_KMH_S * (0.4 + 0.6 * self.brake_intensity) * dt_sec
            if self.target_speed_kmh > 0.0:
                self.target_speed_kmh = max(0.0, self.target_speed_kmh - decel)
            elif self.target_speed_kmh < 0.0:
                self.target_speed_kmh = min(0.0, self.target_speed_kmh + decel)

            self.integral_error = 0.0
            self.pid_active = False
            self.shift_hold_timer = 0.0

            if abs(measured_speed_kmh) <= STOP_SPEED_THRESHOLD_KMH and abs(self.target_speed_kmh) < 0.1:
                self.target_speed_kmh = 0.0
                self.gear = "N"

            return LongitudinalOutput(
                target_speed_kmh=self.target_speed_kmh,
                commanded_cruising_speed_kmh=0.0,
                brake_intensity=self.brake_intensity,
                gear=self.gear,
                pid_correction_kmh=0.0,
            )

        # 2. Forward Throttle / Braking Reverse (Key UP)
        if up_pressed and not down_pressed:
            if measured_speed_kmh < -STOP_SPEED_THRESHOLD_KMH or self.target_speed_kmh < -0.05:
                # Currently moving in reverse -> ramp target toward 0 and brake proportionally to excess speed
                self.target_speed_kmh = min(0.0, self.target_speed_kmh + DECEL_RATE_KMH_S * dt_sec)
                excess = max(0.0, abs(measured_speed_kmh) - abs(self.target_speed_kmh))
                self.brake_intensity = min(
                    MAX_DECEL_BRAKE_INTENSITY,
                    max(0.20, 0.16 + BRAKE_KP_PER_KMH * excess),
                )
                self.integral_error = 0.0
                self.pid_active = False
                self.shift_hold_timer = 0.0
            else:
                # At standstill or moving forward -> symmetric Neutral pause if shifting from R -> D
                self.target_speed_kmh = max(0.0, self.target_speed_kmh)
                if self.gear == "R":
                    self.gear = "N"
                    self.shift_hold_timer = 0.0
                if self.gear != "D":
                    if abs(measured_speed_kmh) > STOP_SPEED_THRESHOLD_KMH:
                        self.gear = "D"
                        self.brake_intensity = 0.0
                        self.shift_hold_timer = 0.0
                    else:
                        self.shift_hold_timer += dt_sec
                        if self.shift_hold_timer >= GEAR_SHIFT_HOLD_SEC:
                            self.gear = "D"
                            self.brake_intensity = 0.0
                        else:
                            self.gear = "N"
                            self.brake_intensity = 0.30
                            self.pid_active = False
                if self.gear == "D":
                    self.brake_intensity = 0.0
                    self.target_speed_kmh = min(
                        MAX_FORWARD_SPEED_KMH,
                        self.target_speed_kmh + ACCEL_RATE_KMH_S * dt_sec,
                    )

        # 3. Decelerate Forward / Engage Reverse (Key DOWN)
        elif down_pressed and not up_pressed:
            if measured_speed_kmh > STOP_SPEED_THRESHOLD_KMH or self.target_speed_kmh > 0.05:
                # Currently moving forward -> ramp target toward 0 and brake proportionally to excess speed
                self.target_speed_kmh = max(0.0, self.target_speed_kmh - DECEL_RATE_KMH_S * dt_sec)
                excess = max(0.0, abs(measured_speed_kmh) - abs(self.target_speed_kmh))
                self.brake_intensity = min(
                    MAX_DECEL_BRAKE_INTENSITY,
                    max(0.20, 0.16 + BRAKE_KP_PER_KMH * excess),
                )
                self.integral_error = 0.0
                self.pid_active = False
                self.shift_hold_timer = 0.0
            else:
                # At standstill -> symmetric Neutral pause before shifting into Reverse (R)
                self.target_speed_kmh = min(0.0, self.target_speed_kmh)
                if self.gear == "D":
                    self.gear = "N"
                    self.shift_hold_timer = 0.0
                if self.gear != "R":
                    self.shift_hold_timer += dt_sec
                    if self.shift_hold_timer >= GEAR_SHIFT_HOLD_SEC:
                        self.gear = "R"
                        self.brake_intensity = 0.0
                    else:
                        self.gear = "N"
                        self.brake_intensity = 0.30
                        self.pid_active = False
                if self.gear == "R":
                    self.brake_intensity = 0.0
                    self.target_speed_kmh = max(
                        MAX_REVERSE_SPEED_KMH,
                        self.target_speed_kmh - REVERSE_ACCEL_RATE_KMH_S * dt_sec,
                    )

        # 4. Coasting (neither UP nor DOWN pressed)
        else:
            self.shift_hold_timer = 0.0
            coast_step = COAST_DECEL_RATE_KMH_S * dt_sec
            if self.target_speed_kmh > 0.0:
                self.target_speed_kmh = max(0.0, self.target_speed_kmh - coast_step)
            elif self.target_speed_kmh < 0.0:
                self.target_speed_kmh = min(0.0, self.target_speed_kmh + coast_step)

            # Apply gentle error-proportional engine braking if actual speed exceeds coasting target
            excess = abs(measured_speed_kmh) - abs(self.target_speed_kmh)
            if excess > COAST_BRAKE_DEADBAND_KMH:
                self.brake_intensity = min(
                    MAX_COAST_BRAKE_INTENSITY,
                    BRAKE_KP_PER_KMH * (excess - COAST_BRAKE_DEADBAND_KMH),
                )
            else:
                self.brake_intensity = 0.0

            if abs(self.target_speed_kmh) < 0.05 and abs(measured_speed_kmh) <= STOP_SPEED_THRESHOLD_KMH:
                self.target_speed_kmh = 0.0
                self.gear = "N"
                self.brake_intensity = 0.0
                self.integral_error = 0.0
                self.pid_active = False

        # 5. Velocity Tracking with Derivative-on-Measurement & Clamping Anti-Windup
        pid_correction = 0.0
        if self.brake_intensity == 0.0 and abs(self.target_speed_kmh) > 0.05:
            error = self.target_speed_kmh - measured_speed_kmh
            if not self.pid_active:
                self.pid_active = True
                self.prev_meas = measured_speed_kmh
                self.integral_error = 0.0

            d_meas = (measured_speed_kmh - self.prev_meas) / dt_sec
            self.prev_meas = measured_speed_kmh

            pid_correction = PID_KP * error + PID_KI * self.integral_error - PID_KD * d_meas
            raw_cmd = self.target_speed_kmh + pid_correction
            lo, hi = (
                (0.5, MAX_FORWARD_SPEED_KMH)
                if self.target_speed_kmh > 0.0
                else (MAX_REVERSE_SPEED_KMH, -0.5)
            )
            commanded_speed = max(lo, min(hi, raw_cmd))
            # Anti-windup: only integrate when output is not saturated
            if abs(raw_cmd - commanded_speed) < 1e-6:
                self.integral_error = max(
                    -PID_INTEGRAL_LIMIT,
                    min(PID_INTEGRAL_LIMIT, self.integral_error + error * dt_sec),
                )
        else:
            self.pid_active = False
            self.prev_meas = measured_speed_kmh
            # Never drive wheel motors while braking or in Neutral
            commanded_speed = 0.0

        return LongitudinalOutput(
            target_speed_kmh=self.target_speed_kmh,
            commanded_cruising_speed_kmh=commanded_speed,
            brake_intensity=self.brake_intensity,
            gear=self.gear,
            pid_correction_kmh=pid_correction,
        )

