"""Verification suite for the East Dead-End road, BmwX5 Ego Vehicle node, and PID/Ackermann controller."""

import json
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CTRL_DIR = ROOT / "controllers" / "bmw_x5_controller"
if str(CTRL_DIR) not in sys.path:
    sys.path.insert(0, str(CTRL_DIR))

from config.vehicle_params import (
    MAX_FORWARD_SPEED_KMH,
    MAX_REVERSE_SPEED_KMH,
    MAX_STEERING_ANGLE_RAD,
)
from control.longitudinal_pid import LongitudinalPIDController
from control.steering_dynamics import AckermannSteeringController


def verify_world_bmwx5() -> dict:
    wbt_path = ROOT / "worlds" / "my_world.wbt"
    wbt = wbt_path.read_text(encoding="utf-8")

    assert 'EXTERNPROTO "../protos/vendor/projects/vehicles/protos/bmw/BmwX5.proto"' in wbt, "Missing BmwX5 EXTERNPROTO"
    match = re.search(r"DEF\s+EGO_VEHICLE\s+BmwX5\s*\{(.*?)# END EGO_VEHICLE_BMWX5", wbt, re.S)
    assert match, "Missing DEF EGO_VEHICLE BmwX5 block"
    block = match.group(1)

    assert 'controller "bmw_x5_controller"' in block, "Wrong controller on BmwX5"
    assert "supervisor FALSE" in block, "Ego vehicle must have supervisor FALSE per AGENT.md"
    assert "translation 273 1.8 0.4" in block, "Unexpected BmwX5 spawn translation"

    required_devices = [
        "front_camera",
        "top_lidar",
        "front_radar",
        "front_center_ds",
        "front_left_ds",
        "front_right_ds",
        "rear_center_ds",
        "rear_left_ds",
        "rear_right_ds",
        "gps",
        "inertial_unit",
        "gyro",
        "accelerometer",
        "compass",
        "hud_display",
    ]
    for dev in required_devices:
        assert f'name "{dev}"' in block, f"Missing required device '{dev}' in BmwX5"

    return {
        "ego_vehicle_def": "EGO_VEHICLE",
        "proto": "BmwX5",
        "controller": "bmw_x5_controller",
        "supervisor": False,
        "spawn_xyz": [273.0, 1.8, 0.4],
        "spawn_yaw_rad": 3.141593,
        "devices_verified": len(required_devices),
        "device_names": required_devices,
    }


def verify_controller_physics() -> dict:
    dt = 0.032
    lon = LongitudinalPIDController()
    lat = AckermannSteeringController()

    # 1. Forward acceleration (UP held for 4.0 seconds)
    v_meas = 0.0
    for _ in range(int(4.0 / dt)):
        out = lon.update(dt, up_pressed=True, down_pressed=False, space_pressed=False, measured_speed_kmh=v_meas)
        # First-order vehicle plant response
        v_meas += (out.commanded_cruising_speed_kmh - v_meas) * min(1.0, 3.5 * dt)
    assert out.gear == "D", f"Expected gear D, got {out.gear}"
    assert abs(out.target_speed_kmh - MAX_FORWARD_SPEED_KMH) < 1e-3, f"Expected max forward speed {MAX_FORWARD_SPEED_KMH}, got {out.target_speed_kmh}"
    assert v_meas > 52.0, f"Vehicle did not accelerate properly: {v_meas}"
    peak_forward_speed = round(v_meas, 2)

    # 2. Progressive Space braking to full stop (0 km/h)
    brake_steps = 0
    while v_meas > 0.05 and brake_steps < 200:
        brake_steps += 1
        out = lon.update(dt, up_pressed=False, down_pressed=False, space_pressed=True, measured_speed_kmh=v_meas)
        decel = 38.0 * out.brake_intensity * dt
        v_meas = max(0.0, v_meas - decel)
    out = lon.update(dt, up_pressed=False, down_pressed=False, space_pressed=True, measured_speed_kmh=0.0)
    assert out.target_speed_kmh == 0.0 and out.gear == "N", f"Space brake did not stop car cleanly: {out}"
    assert out.brake_intensity > 0.8, f"Brake intensity did not ramp up: {out.brake_intensity}"

    # 3. Accelerate forward to 25 km/h, then hold DOWN -> decelerate to 0 -> shift to Reverse (R)
    lon.reset()
    v_meas = 25.0
    lon.target_speed_kmh = 25.0
    lon.gear = "D"
    saw_zero_pause = False
    for _ in range(int(3.5 / dt)):
        out = lon.update(dt, up_pressed=False, down_pressed=True, space_pressed=False, measured_speed_kmh=v_meas)
        if out.gear == "N" and abs(v_meas) <= 0.65:
            saw_zero_pause = True
        if out.brake_intensity > 0.0:
            v_meas = max(0.0, v_meas - 26.0 * dt)
        else:
            v_meas += (out.commanded_cruising_speed_kmh - v_meas) * min(1.0, 3.5 * dt)
    assert saw_zero_pause, "Controller did not pause at Neutral before engaging Reverse"
    assert out.gear == "R" and out.target_speed_kmh < -10.0 and v_meas < -8.0, f"Reverse failed: {out}, v={v_meas}"
    peak_reverse_speed = round(v_meas, 2)

    # 4. Steering Ackermann + Speed-dependent limit + Automatic Self-Centering
    max_low = lat.compute_max_angle(5.0)
    max_high = lat.compute_max_angle(60.0)
    assert max_low > 0.45 and max_high < 0.15, f"Speed-sensitive steering limit invalid: low={max_low}, high={max_high}"

    # Steer Left for 0.6s at 25 km/h
    lat.reset()
    for _ in range(int(0.6 / dt)):
        s_out = lat.update(dt, left_pressed=True, right_pressed=False, measured_speed_kmh=25.0)
    assert s_out.steering_angle_rad < -0.20, f"Left steering failed: {s_out.steering_angle_rad}"
    assert s_out.left_wheel_angle_rad < s_out.right_wheel_angle_rad < 0.0, "Ackermann inner/outer wheel geometry invalid for Left turn"

    # Release Left key -> verify automatic self-centering back to 0.0 rad!
    return_steps = 0
    prev_abs = abs(s_out.steering_angle_rad)
    while abs(s_out.steering_angle_rad) > 0.0 and return_steps < 60:
        return_steps += 1
        s_out = lat.update(dt, left_pressed=False, right_pressed=False, measured_speed_kmh=25.0)
        assert abs(s_out.steering_angle_rad) <= prev_abs + 1e-9, "Self-centering oscillated instead of monotonic return"
        prev_abs = abs(s_out.steering_angle_rad)
    assert s_out.steering_angle_rad == 0.0, f"Steering did not self-center to 0.0: {s_out.steering_angle_rad}"

    return {
        "max_forward_speed_kmh": MAX_FORWARD_SPEED_KMH,
        "max_reverse_speed_kmh": MAX_REVERSE_SPEED_KMH,
        "simulated_peak_forward_kmh": peak_forward_speed,
        "simulated_peak_reverse_kmh": peak_reverse_speed,
        "space_brake_stop_steps": brake_steps,
        "reverse_shift_verified": True,
        "max_steer_angle_5kmh_deg": round(math.degrees(max_low), 2),
        "max_steer_angle_60kmh_deg": round(math.degrees(max_high), 2),
        "self_center_steps_at_25kmh": return_steps,
        "self_center_final_angle_rad": s_out.steering_angle_rad,
        "status": "PASS",
    }


def main() -> int:
    report = {
        "world_bmwx5": verify_world_bmwx5(),
        "controller_physics": verify_controller_physics(),
        "overall_status": "PASS",
    }
    out_file = ROOT / "validation" / "bmwx5-verification.json"
    out_file.parent.mkdir(exist_ok=True)
    out_file.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
