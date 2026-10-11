"""Longitudinal speed PID, automatic D/N/R gear state machine, and progressive Space braking."""

from dataclasses import dataclass
from config.vehicle_params import (
    ACCEL_RATE_KMH_S,
    COAST_DECEL_RATE_KMH_S,
    DECEL_RATE_KMH_S,
    GEAR_SHIFT_HOLD_SEC,
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
    """Realistic longitudinal controller with PID speed tracking, gear logic, and progressive brake."""

    def __init__(self) -> None:
        self.target_speed_kmh: float = 0.0
        self.brake_intensity: float = 0.0
        self.gear: str = "N"
        self.integral_error: float = 0.0
        self.prev_error: float = 0.0
        self.reverse_hold_timer: float = 0.0

    def reset(self) -> None:
        self.target_speed_kmh = 0.0
        self.brake_intensity = 0.0
        self.gear = "N"
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.reverse_hold_timer = 0.0

    def update(
        self,
        dt_sec: float,
        up_pressed: bool,
        down_pressed: bool,
        space_pressed: bool,
        measured_speed_kmh: float,
    ) -> LongitudinalOutput:
        if dt_sec <= 0.0:
            dt_sec = 0.032

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
            self.prev_error = 0.0
            self.reverse_hold_timer = 0.0

            if abs(measured_speed_kmh) <= STOP_SPEED_THRESHOLD_KMH and abs(self.target_speed_kmh) < 0.1:
                self.target_speed_kmh = 0.0
                self.gear = "N"

            return LongitudinalOutput(
                target_speed_kmh=self.target_speed_kmh,
                commanded_cruising_speed_kmh=0.0 if self.brake_intensity > 0.25 else self.target_speed_kmh,
                brake_intensity=self.brake_intensity,
                gear=self.gear,
                pid_correction_kmh=0.0,
            )

        # 2. Forward Throttle / Braking Reverse (Key UP)
        if up_pressed and not down_pressed:
            self.reverse_hold_timer = 0.0
            if measured_speed_kmh < -STOP_SPEED_THRESHOLD_KMH or self.target_speed_kmh < -0.05:
                # Currently moving in reverse -> apply service brake to bring speed to 0 first
                self.target_speed_kmh = min(0.0, self.target_speed_kmh + DECEL_RATE_KMH_S * dt_sec)
                self.brake_intensity = 0.60
                self.integral_error = 0.0
            else:
                self.gear = "D"
                self.brake_intensity = 0.0
                self.target_speed_kmh = min(
                    MAX_FORWARD_SPEED_KMH,
                    max(0.0, self.target_speed_kmh) + ACCEL_RATE_KMH_S * dt_sec,
                )

        # 3. Decelerate Forward / Engage Reverse (Key DOWN)
        elif down_pressed and not up_pressed:
            if measured_speed_kmh > STOP_SPEED_THRESHOLD_KMH or self.target_speed_kmh > 0.05:
                # Currently moving forward -> decelerate smoothly toward 0 km/h
                self.target_speed_kmh = max(0.0, self.target_speed_kmh - DECEL_RATE_KMH_S * dt_sec)
                self.brake_intensity = 0.50
                self.integral_error = 0.0
                self.reverse_hold_timer = 0.0
            else:
                # At standstill -> brief pause before shifting into Reverse (R)
                self.target_speed_kmh = min(0.0, self.target_speed_kmh)
                if self.gear != "R":
                    self.reverse_hold_timer += dt_sec
                    if self.reverse_hold_timer >= GEAR_SHIFT_HOLD_SEC:
                        self.gear = "R"
                        self.brake_intensity = 0.0
                    else:
                        self.gear = "N"
                        self.brake_intensity = 0.30
                if self.gear == "R":
                    self.brake_intensity = 0.0
                    self.target_speed_kmh = max(
                        MAX_REVERSE_SPEED_KMH,
                        self.target_speed_kmh - REVERSE_ACCEL_RATE_KMH_S * dt_sec,
                    )

        # 4. Coasting (neither UP nor DOWN pressed)
        else:
            self.reverse_hold_timer = 0.0
            self.brake_intensity = 0.0
            coast_step = COAST_DECEL_RATE_KMH_S * dt_sec
            if self.target_speed_kmh > 0.0:
                self.target_speed_kmh = max(0.0, self.target_speed_kmh - coast_step)
            elif self.target_speed_kmh < 0.0:
                self.target_speed_kmh = min(0.0, self.target_speed_kmh + coast_step)

            if abs(self.target_speed_kmh) < 0.05 and abs(measured_speed_kmh) <= STOP_SPEED_THRESHOLD_KMH:
                self.target_speed_kmh = 0.0
                self.gear = "N"
                self.integral_error = 0.0

        # 5. PID Velocity Tracking Compensation
        pid_correction = 0.0
        commanded_speed = self.target_speed_kmh
        if self.brake_intensity == 0.0 and abs(self.target_speed_kmh) > 0.05:
            error = self.target_speed_kmh - measured_speed_kmh
            self.integral_error = max(
                -PID_INTEGRAL_LIMIT,
                min(PID_INTEGRAL_LIMIT, self.integral_error + error * dt_sec),
            )
            derivative = (error - self.prev_error) / dt_sec
            self.prev_error = error
            pid_correction = PID_KP * error + PID_KI * self.integral_error + PID_KD * derivative
            raw_cmd = self.target_speed_kmh + pid_correction
            if self.target_speed_kmh > 0.0:
                commanded_speed = max(0.5, min(MAX_FORWARD_SPEED_KMH * 1.12, raw_cmd))
            else:
                commanded_speed = min(-0.5, max(MAX_REVERSE_SPEED_KMH * 1.12, raw_cmd))
        else:
            self.prev_error = 0.0

        return LongitudinalOutput(
            target_speed_kmh=self.target_speed_kmh,
            commanded_cruising_speed_kmh=commanded_speed,
            brake_intensity=self.brake_intensity,
            gear=self.gear,
            pid_correction_kmh=pid_correction,
        )
