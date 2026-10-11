"""Ackermann steering dynamics with speed-sensitive angle limit, PD damping, and automatic self-centering."""

from dataclasses import dataclass
import math
from config.vehicle_params import (
    DEFAULT_DT_SEC,
    INDICATOR_HOLD_SEC,
    MAX_STEER_RATE_RAD_S,
    MAX_STEERING_ANGLE_RAD,
    SELF_CENTER_BASE_RATE_RAD_S,
    SELF_CENTER_SPEED_GAIN,
    SPEED_SENSITIVITY_KV,
    STEER_DEADBAND_RAD,
    STEER_KD,
    STEER_KP,
    TRACK_FRONT_M,
    WHEELBASE_M,
)


@dataclass
class SteeringOutput:
    """Output state from the Ackermann steering dynamics controller."""
    steering_angle_rad: float
    target_angle_rad: float
    max_allowed_angle_rad: float
    left_wheel_angle_rad: float
    right_wheel_angle_rad: float
    turning_radius_m: float
    indicator_dir: int  # 0: OFF, 1: RIGHT, -1: LEFT


class AckermannSteeringController:
    """Second-order PD steering actuator with speed-sensitive Ackermann limit and caster self-centering."""

    def __init__(self) -> None:
        self.steering_angle_rad: float = 0.0
        self.steering_rate_rad_s: float = 0.0
        self.latched_indicator: int = 0
        self.indicator_timer_sec: float = 0.0

    def reset(self) -> None:
        self.steering_angle_rad = 0.0
        self.steering_rate_rad_s = 0.0
        self.latched_indicator = 0
        self.indicator_timer_sec = 0.0

    @staticmethod
    def compute_max_angle(speed_kmh: float) -> float:
        """Speed-sensitive maximum steering angle to prevent rollover/skidding at high speed."""
        v_ms = abs(speed_kmh) / 3.6
        return MAX_STEERING_ANGLE_RAD / (1.0 + SPEED_SENSITIVITY_KV * (v_ms ** 2))

    @staticmethod
    def compute_ackermann_wheels(center_angle_rad: float) -> tuple[float, float, float]:
        """Compute exact left wheel angle, right wheel angle, and turning radius R (m).

        Webots Car/Driver convention:
        - Positive steering_angle_rad (+) = Right turn
        - Negative steering_angle_rad (-) = Left turn
        """
        if abs(center_angle_rad) < 1e-5:
            return 0.0, 0.0, float("inf")
        cot = 1.0 / math.tan(center_angle_rad)
        half_ratio = TRACK_FRONT_M / (2.0 * WHEELBASE_M)
        right_angle = math.atan(1.0 / (cot - half_ratio))
        left_angle = math.atan(1.0 / (cot + half_ratio))
        turning_radius = WHEELBASE_M / math.tan(abs(center_angle_rad))
        return left_angle, right_angle, turning_radius

    def update(
        self,
        dt_sec: float,
        left_pressed: bool,
        right_pressed: bool,
        measured_speed_kmh: float,
    ) -> SteeringOutput:
        if dt_sec <= 0.0:
            dt_sec = DEFAULT_DT_SEC

        max_angle = self.compute_max_angle(measured_speed_kmh)
        v_ms = abs(measured_speed_kmh) / 3.6

        # Webots Driver convention: negative = LEFT, positive = RIGHT
        if left_pressed and not right_pressed:
            target_angle = -max_angle
            self.latched_indicator = -1
            self.indicator_timer_sec = INDICATOR_HOLD_SEC
        elif right_pressed and not left_pressed:
            target_angle = +max_angle
            self.latched_indicator = 1
            self.indicator_timer_sec = INDICATOR_HOLD_SEC
        else:
            # Keys released -> automatic self-centering to 0.0 rad
            target_angle = 0.0
            if self.indicator_timer_sec > 0.0:
                self.indicator_timer_sec = max(0.0, self.indicator_timer_sec - dt_sec)
            if self.indicator_timer_sec <= 0.0 and abs(self.steering_angle_rad) < 0.06:
                self.latched_indicator = 0

        if target_angle != 0.0:
            # Active PD steering into turn with derivative damping and rate limit
            error = target_angle - self.steering_angle_rad
            pd_cmd = STEER_KP * error - STEER_KD * self.steering_rate_rad_s
            raw_rate = max(-MAX_STEER_RATE_RAD_S, min(MAX_STEER_RATE_RAD_S, pd_cmd))
            alpha = min(1.0, dt_sec * 12.0)
            desired_rate = (1.0 - alpha) * self.steering_rate_rad_s + alpha * raw_rate
            next_angle = self.steering_angle_rad + desired_rate * dt_sec
            # Prevent overshooting target_angle in a single step
            if (target_angle > 0.0 and self.steering_angle_rad <= target_angle and next_angle > target_angle) or (
                target_angle < 0.0 and self.steering_angle_rad >= target_angle and next_angle < target_angle
            ):
                next_angle = target_angle
        else:
            # Self-centering caster return when LEFT/RIGHT keys are released
            speed_factor = min(1.0, v_ms / 14.0)
            max_return_rate = SELF_CENTER_BASE_RATE_RAD_S + SELF_CENTER_SPEED_GAIN * speed_factor
            if abs(self.steering_angle_rad) <= STEER_DEADBAND_RAD:
                next_angle = 0.0
            else:
                sign = -1.0 if self.steering_angle_rad > 0.0 else 1.0
                prop_rate = STEER_KP * abs(self.steering_angle_rad)
                mag = min(max_return_rate, max(0.35, prop_rate))
                desired_rate = sign * mag
                next_angle = self.steering_angle_rad + desired_rate * dt_sec
                # Prevent zero-crossing overshoot during self-centering
                if self.steering_angle_rad * next_angle <= 0.0:
                    next_angle = 0.0

        # Smoothly rate-limit reduction if max_angle shrinks as vehicle accelerates (avoid instantaneous snap)
        max_step_delta = MAX_STEER_RATE_RAD_S * dt_sec
        if next_angle > max_angle:
            next_angle = max(max_angle, self.steering_angle_rad - max_step_delta)
        elif next_angle < -max_angle:
            next_angle = min(-max_angle, self.steering_angle_rad + max_step_delta)

        next_angle = max(-MAX_STEERING_ANGLE_RAD, min(MAX_STEERING_ANGLE_RAD, next_angle))
        if target_angle == 0.0 and abs(next_angle) <= STEER_DEADBAND_RAD:
            next_angle = 0.0

        self.steering_rate_rad_s = (next_angle - self.steering_angle_rad) / dt_sec
        self.steering_angle_rad = next_angle

        left_w, right_w, radius = self.compute_ackermann_wheels(self.steering_angle_rad)
        return SteeringOutput(
            steering_angle_rad=self.steering_angle_rad,
            target_angle_rad=target_angle,
            max_allowed_angle_rad=max_angle,
            left_wheel_angle_rad=left_w,
            right_wheel_angle_rad=right_w,
            turning_radius_m=radius,
            indicator_dir=self.latched_indicator,
        )

