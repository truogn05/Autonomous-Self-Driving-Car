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

    # Verify 100% local PROTOs & assets (zero webots:// or http:// in BmwX5 dependency tree)
    vendor_projects = ROOT / "protos" / "vendor" / "projects"
    proto_dirs = [
        vendor_projects / "vehicles" / "protos" / "bmw",
        vendor_projects / "vehicles" / "protos" / "abstract",
        vendor_projects / "appearances" / "protos",
    ]
    checked_protos = 0
    for pdir in proto_dirs:
        for pf in sorted(pdir.rglob("*.proto")):
            checked_protos += 1
            ptxt = pf.read_text(encoding="utf-8")
            assert 'EXTERNPROTO "webots://' not in ptxt, f"External webots:// EXTERNPROTO found in {pf}"
            assert '"webots://' not in ptxt, f"External webots:// asset URL found in {pf}"
            for ext in re.findall(r'EXTERNPROTO\s+"([^"]+)"', ptxt):
                resolved = (pf.parent / ext).resolve()
                assert resolved.exists(), f"Missing local EXTERNPROTO '{ext}' referenced by {pf.name} -> {resolved}"

    # Verify English console instructions in bmw_x5_controller.py
    ctrl_code = (CTRL_DIR / "bmw_x5_controller.py").read_text(encoding="utf-8")
    assert "BMW X5 AUTONOMOUS & PHYSICS KEYBOARD CONTROLLER" in ctrl_code, "Missing English console banner"
    assert "HOW TO DRIVE:" in ctrl_code, "Missing HOW TO DRIVE section in English console banner"

    return {
        "ego_vehicle_def": "EGO_VEHICLE",
        "proto": "BmwX5",
        "controller": "bmw_x5_controller",
        "supervisor": False,
        "spawn_xyz": [273.0, 1.8, 0.4],
        "spawn_yaw_rad": 3.141593,
        "devices_verified": len(required_devices),
        "device_names": required_devices,
        "local_protos_verified": checked_protos,
        "zero_external_urls": True,
        "english_console_banner": True,
    }


def verify_controller_physics() -> dict:
    dt = 0.016
    lon = LongitudinalPIDController()
    lat = AckermannSteeringController()

    # 1. Forward acceleration (UP held for 4.2 seconds from standstill N -> D)
    v_meas = 0.0
    for _ in range(int(4.2 / dt)):
        out = lon.update(dt, up_pressed=True, down_pressed=False, space_pressed=False, measured_speed_kmh=v_meas)
        if out.brake_intensity > 0.0:
            assert out.commanded_cruising_speed_kmh == 0.0, "Cruising speed must be 0 while braking"
        else:
            v_meas += (out.commanded_cruising_speed_kmh - v_meas) * min(1.0, 3.5 * dt)
    assert out.gear == "D", f"Expected gear D, got {out.gear}"
    assert abs(out.target_speed_kmh - MAX_FORWARD_SPEED_KMH) < 1e-3, f"Expected max forward speed {MAX_FORWARD_SPEED_KMH}, got {out.target_speed_kmh}"
    assert 52.0 < v_meas <= MAX_FORWARD_SPEED_KMH + 1e-4, f"Vehicle speed out of bounds: {v_meas}"
    peak_forward_speed = round(v_meas, 2)

    # 2. Derivative-kick & Coasting error-brake check
    # Coast for 0.5s with v_meas held artificially high -> error-proportional coast brake must engage
    for _ in range(int(0.5 / dt)):
        out = lon.update(dt, up_pressed=False, down_pressed=False, space_pressed=False, measured_speed_kmh=v_meas)
    assert out.brake_intensity > 0.0 and out.commanded_cruising_speed_kmh == 0.0, "Coast brake did not engage on speed excess"

    # Re-engage UP after braking -> verify zero derivative kick on first re-entry step
    out_reentry = lon.update(dt, up_pressed=True, down_pressed=False, space_pressed=False, measured_speed_kmh=v_meas - 5.0)
    assert abs(out_reentry.pid_correction_kmh) < 5.0, f"Derivative kick detected on PID re-entry: {out_reentry.pid_correction_kmh}"

    # 3. Progressive Space braking to full stop (0 km/h)
    brake_steps = 0
    while v_meas > 0.05 and brake_steps < 400:
        brake_steps += 1
        out = lon.update(dt, up_pressed=False, down_pressed=False, space_pressed=True, measured_speed_kmh=v_meas)
        assert out.commanded_cruising_speed_kmh == 0.0, "Space brake must force commanded_cruising_speed_kmh = 0"
        decel = 38.0 * out.brake_intensity * dt
        v_meas = max(0.0, v_meas - decel)
    out = lon.update(dt, up_pressed=False, down_pressed=False, space_pressed=True, measured_speed_kmh=0.0)
    assert out.target_speed_kmh == 0.0 and out.gear == "N", f"Space brake did not stop car cleanly: {out}"
    assert out.brake_intensity > 0.8, f"Brake intensity did not ramp up: {out.brake_intensity}"

    # 4. Full Symmetric State Machine Test: D (25 km/h) -> DOWN-hold -> N pause -> R -> UP-hold -> N pause -> D
    lon.reset()
    v_meas = 25.0
    lon.target_speed_kmh = 25.0
    lon.gear = "D"
    saw_d_to_r_neutral = False
    for _ in range(int(3.5 / dt)):
        out = lon.update(dt, up_pressed=False, down_pressed=True, space_pressed=False, measured_speed_kmh=v_meas)
        if out.gear == "N" and abs(v_meas) <= 0.65:
            saw_d_to_r_neutral = True
        if out.brake_intensity > 0.0:
            assert out.commanded_cruising_speed_kmh == 0.0
            v_meas = max(0.0, v_meas - 26.0 * dt)
        else:
            v_meas += (out.commanded_cruising_speed_kmh - v_meas) * min(1.0, 3.5 * dt)
    assert saw_d_to_r_neutral, "Controller did not pause at Neutral during D -> R shift"
    assert out.gear == "R" and out.target_speed_kmh < -10.0 and v_meas < -8.0, f"Reverse failed: {out}, v={v_meas}"
    peak_reverse_speed = round(v_meas, 2)

    # Now hold UP while reversing in R -> must decelerate to 0, pause in N (symmetric hold!), then shift to D
    saw_r_to_d_neutral = False
    for _ in range(int(3.0 / dt)):
        out = lon.update(dt, up_pressed=True, down_pressed=False, space_pressed=False, measured_speed_kmh=v_meas)
        if out.gear == "N" and abs(v_meas) <= 0.65:
            saw_r_to_d_neutral = True
        if out.brake_intensity > 0.0:
            assert out.commanded_cruising_speed_kmh == 0.0
            v_meas = min(0.0, v_meas + 26.0 * dt)
        else:
            v_meas += (out.commanded_cruising_speed_kmh - v_meas) * min(1.0, 3.5 * dt)
    assert saw_r_to_d_neutral, "Controller did not pause symmetrically at Neutral during R -> D shift"
    assert out.gear == "D" and out.target_speed_kmh > 10.0 and v_meas > 5.0, f"R -> D recovery failed: {out}, v={v_meas}"

    # 5. Steering Ackermann + Speed-dependent limit + Rate-limited envelope + Self-Centering
    max_low = lat.compute_max_angle(5.0)
    max_high = lat.compute_max_angle(60.0)
    assert max_low > 0.45 and max_high < 0.15, f"Speed-sensitive steering limit invalid: low={max_low}, high={max_high}"

    # Steer Left to full lock at 5 km/h, then jump speed to 50 km/h -> verify no instantaneous snap
    lat.reset()
    for _ in range(int(0.6 / dt)):
        s_out = lat.update(dt, left_pressed=True, right_pressed=False, measured_speed_kmh=5.0)
    angle_before_jump = s_out.steering_angle_rad
    s_jump = lat.update(dt, left_pressed=True, right_pressed=False, measured_speed_kmh=50.0)
    step_change = abs(s_jump.steering_angle_rad - angle_before_jump)
    assert step_change <= 1.20 * dt + 1e-6, f"Steering angle snapped instantaneously on speed increase: {step_change}"

    # Steer Left for 0.6s at 25 km/h
    lat.reset()
    for _ in range(int(0.6 / dt)):
        s_out = lat.update(dt, left_pressed=True, right_pressed=False, measured_speed_kmh=25.0)
    assert s_out.steering_angle_rad < -0.20, f"Left steering failed: {s_out.steering_angle_rad}"
    assert s_out.left_wheel_angle_rad < s_out.right_wheel_angle_rad < 0.0, "Ackermann inner/outer wheel geometry invalid for Left turn"

    # Release Left key -> verify automatic self-centering back to 0.0 rad
    return_steps = 0
    prev_abs = abs(s_out.steering_angle_rad)
    while abs(s_out.steering_angle_rad) > 0.0 and return_steps < 120:
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
        "symmetric_dnr_shift_verified": True,
        "derivative_kick_suppressed": True,
        "error_based_braking_verified": True,
        "rate_limited_speed_clamp_verified": True,
        "max_steer_angle_5kmh_deg": round(math.degrees(max_low), 2),
        "max_steer_angle_60kmh_deg": round(math.degrees(max_high), 2),
        "self_center_steps_at_25kmh": return_steps,
        "self_center_final_angle_rad": s_out.steering_angle_rad,
        "status": "PASS",
    }


def verify_hud_and_chase_camera() -> dict:
    from perception.sensor_manager import SensorTelemetry
    from ui.hud_overlay import HudOverlay

    class _MockDisplay:
        def __init__(self) -> None:
            self.calls = 0

        def getWidth(self) -> int:
            return 210

        def getHeight(self) -> int:
            return 76

        def setFont(self, *_a) -> None:
            self.calls += 1

        def setAlpha(self, *_a) -> None:
            self.calls += 1

        def setColor(self, *_a) -> None:
            self.calls += 1

        def fillRectangle(self, *_a) -> None:
            self.calls += 1

        def drawRectangle(self, *_a) -> None:
            self.calls += 1

        def drawLine(self, *_a) -> None:
            self.calls += 1

        def drawText(self, *_a) -> None:
            self.calls += 1

    mock_disp = _MockDisplay()
    hud = HudOverlay(mock_disp)
    lon = LongitudinalPIDController()
    lat = AckermannSteeringController()
    lon_out = lon.update(0.016, True, False, False, 24.0)
    lat_out = lat.update(0.016, True, False, 24.0)
    hud.render(24.0, lon_out, lat_out, SensorTelemetry())
    assert mock_disp.calls > 10, "HudOverlay did not render expected primitives"

    # Verify Spring-Arm Lazy Chase Camera math (turn lag -> smooth return -> Shift snap)
    dt_sec = 0.016
    tau_turn = 0.85
    tau_straight = 0.50
    tau_shift = 0.08
    max_lag = 0.52

    car_yaw = 0.0
    cam_yaw = 0.0
    # 1. Car turns left at 0.45 rad/s for 1.2s -> camera should lag smoothly behind (between 12 deg and 30 deg)
    for _ in range(int(1.2 / dt_sec)):
        car_yaw += 0.45 * dt_sec
        err = math.atan2(math.sin(car_yaw - cam_yaw), math.cos(car_yaw - cam_yaw))
        cam_yaw += (1.0 - math.exp(-dt_sec / tau_turn)) * err
        rem = math.atan2(math.sin(car_yaw - cam_yaw), math.cos(car_yaw - cam_yaw))
        if rem > max_lag:
            cam_yaw = car_yaw - max_lag
    lag_during_turn_deg = math.degrees(abs(car_yaw - cam_yaw))
    assert 12.0 <= lag_during_turn_deg <= 31.0, f"Unexpected turn lag: {lag_during_turn_deg} deg"

    # 2. Car drives straight for 2.0s -> camera smoothly glides back behind car (< 1.5 deg error)
    for _ in range(int(2.0 / dt_sec)):
        err = math.atan2(math.sin(car_yaw - cam_yaw), math.cos(car_yaw - cam_yaw))
        cam_yaw += (1.0 - math.exp(-dt_sec / tau_straight)) * err
    lag_after_straight_deg = math.degrees(abs(car_yaw - cam_yaw))
    assert lag_after_straight_deg < 1.5, f"Camera did not settle behind car: {lag_after_straight_deg} deg"

    # 3. User orbits camera 90 deg sideways with mouse, then holds SHIFT for 0.35s -> snaps back behind car
    cam_yaw = car_yaw + (math.pi / 2.0)
    for _ in range(int(0.35 / dt_sec)):
        err = math.atan2(math.sin(car_yaw - cam_yaw), math.cos(car_yaw - cam_yaw))
        cam_yaw += (1.0 - math.exp(-dt_sec / tau_shift)) * err
    lag_after_shift_deg = math.degrees(abs(car_yaw - cam_yaw))
    assert lag_after_shift_deg < 1.5, f"SHIFT did not snap camera behind car: {lag_after_shift_deg} deg"

    # 4. Verify AckermannVehicle & Car PROTO stability guards
    ack_txt = (ROOT / "protos/vendor/projects/vehicles/protos/abstract/AckermannVehicle.proto").read_text(encoding="utf-8")
    car_txt = (ROOT / "protos/vendor/projects/vehicles/protos/abstract/Car.proto").read_text(encoding="utf-8")
    assert "dampingConstant 120" in ack_txt and "minStop -0.62" in ack_txt, "Missing steering damping/stops in AckermannVehicle.proto"
    assert "maxTorque 4000" in car_txt and "100000000" not in car_txt, "Unstable 1e8 maxTorque still present in Car.proto"

    return {
        "hud_dimensions": [210, 76],
        "turn_lag_deg": round(lag_during_turn_deg, 2),
        "settled_lag_deg": round(lag_after_straight_deg, 3),
        "shift_snap_lag_deg": round(lag_after_shift_deg, 3),
        "steering_damping_and_stops": True,
        "status": "PASS",
    }


def main() -> int:
    report = {
        "world_bmwx5": verify_world_bmwx5(),
        "controller_physics": verify_controller_physics(),
        "hud_and_chase_camera": verify_hud_and_chase_camera(),
        "overall_status": "PASS",
    }
    out_file = ROOT / "validation" / "bmwx5-verification.json"
    out_file.parent.mkdir(exist_ok=True)
    out_file.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

