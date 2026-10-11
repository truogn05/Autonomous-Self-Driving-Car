"""Main Webots controller for the BMW X5 ego vehicle (supervisor FALSE).

Features:
- Full Autonomous Sensor Suite (Front RGB Camera w/ top-left overlay, 4-layer 3D LiDAR,
  Front Radar, GPS, IMU, Gyro, Accelerometer, Compass, 6x Ultrasonic DistanceSensors).
- Real-time Minimalist Telemetry HUD on Webots Display ('hud_display').
- Realistic Longitudinal PID speed control + symmetric D/N/R gear state machine + progressive Space braking.
- Speed-sensitive Ackermann steering with PD damping and automatic caster self-centering when arrow keys are released.
"""

import math
import sys
from pathlib import Path

# Ensure controller subpackages are importable
CONTROLLER_DIR = Path(__file__).resolve().parent
if str(CONTROLLER_DIR) not in sys.path:
    sys.path.insert(0, str(CONTROLLER_DIR))

from controller import Keyboard
from vehicle import Driver

from config.vehicle_params import DEFAULT_TIMESTEP_MS
from control.longitudinal_pid import LongitudinalPIDController
from control.steering_dynamics import AckermannSteeringController
from perception.sensor_manager import SensorManager
from ui.hud_overlay import HudOverlay

_indicator_warned = False


def set_indicator_safe(driver: Driver, indicator_dir: int, prev_dir: int) -> int:
    """Safely update turn indicators on Driver, bypassing the typo in Webots's driver.py."""
    global _indicator_warned
    if indicator_dir == prev_dir:
        return prev_dir
    try:
        # Driver.INDICATOR_OFF = 0, INDICATOR_RIGHT = 1, INDICATOR_LEFT = 2
        if indicator_dir > 0:
            state = Driver.INDICATOR_RIGHT
        elif indicator_dir < 0:
            state = Driver.INDICATOR_LEFT
        else:
            state = Driver.INDICATOR_OFF
        if Driver.api is not None and hasattr(Driver.api, "wbu_driver_set_indicator"):
            Driver.api.wbu_driver_set_indicator(int(state))
    except Exception as exc:
        if not _indicator_warned:
            print(f"[indicator] failed to set turn indicator: {exc}", flush=True)
            _indicator_warned = True
    return indicator_dir


def print_console_instructions() -> None:
    """Print detailed English driving and sensor instructions to the Webots Console."""
    banner = (
        "\n"
        "======================================================================\n"
        "  BMW X5 AUTONOMOUS & PHYSICS KEYBOARD CONTROLLER (MANUAL DRIVE MODE) \n"
        "======================================================================\n"
        "  HOW TO DRIVE:\n"
        "  1. Click anywhere inside the 3D Scene View to focus the keyboard.\n"
        "  2. Use the following keys to control the vehicle:\n"
        "     - [UP ARROW]    : Accelerate forward smoothly (PID up to 60 km/h)\n"
        "     - [DOWN ARROW]  : Decelerate to 0 km/h; hold at 0 km/h to shift\n"
        "                       into Reverse (R, up to -18 km/h)\n"
        "     - [LEFT / RIGHT]: Ackermann steering with speed-sensitive limit;\n"
        "                       release key to auto self-center wheels to 0 deg\n"
        "     - [SPACEBAR]    : Hold for progressive smooth braking to 0 km/h\n"
        "     - [SHIFT]       : Hold/Press to snap & lock 3rd-person camera behind car\n"
        "  3. OVERLAYS & SENSORS (Top-Left of 3D View):\n"
        "     - Camera Overlay : 'front_camera' (320x180 clean RGB view)\n"
        "     - HUD Overlay    : 'hud_display'  (Minimal Speed, Gear, Steer, Brake)\n"
        "     - Note: If overlays are hidden, enable via top menu:\n"
        "       Overlays -> Camera Devices -> 'front_camera'\n"
        "       Overlays -> Display Devices -> 'hud_display'\n"
        "     - Active Sensors (15): Camera, 4-Layer Lidar, Radar, 6x Sonar,\n"
        "                            GPS, IMU, Gyro, Accelerometer, Compass\n"
        "======================================================================"
    )
    print(banner, flush=True)


def main() -> int:
    driver = Driver()
    basic_step_ms = int(driver.getBasicTimeStep())
    if basic_step_ms <= 0:
        basic_step_ms = DEFAULT_TIMESTEP_MS
    dt_sec = basic_step_ms / 1000.0

    # Initialize keyboard
    keyboard = Keyboard()
    keyboard.enable(basic_step_ms)

    # Initialize sensors & HUD display
    sensors = SensorManager(driver, basic_step_ms)
    hud = HudOverlay(sensors.hud_display)

    # Initialize physics-based longitudinal PID & lateral Ackermann controllers
    lon_ctrl = LongitudinalPIDController()
    lat_ctrl = AckermannSteeringController()

    # Initial vehicle actuator state
    driver.setSteeringAngle(0.0)
    driver.setCruisingSpeed(0.0)
    driver.setBrakeIntensity(0.0)
    driver.setDippedBeams(True)
    driver.setCustomData("SHIFT:0")

    # Print English driving guide immediately and after first step
    print_console_instructions()

    # Warm-up one step before reading sensors (per webots-controller-programming skill)
    if driver.step() == -1:
        return 0

    prev_indicator = 0
    prev_shift_state = False
    failsafe_logged = False
    step_idx = 0

    while driver.step() != -1:
        step_idx += 1

        # 1. Read all sensors
        telemetry = sensors.read()

        # 2. Poll all simultaneously held keys in the current control step
        up_pressed = False
        down_pressed = False
        left_pressed = False
        right_pressed = False
        space_pressed = False
        shift_pressed = False

        while True:
            key = keyboard.getKey()
            if key == -1:
                break
            if key == Keyboard.SHIFT or (key & Keyboard.SHIFT) != 0:
                shift_pressed = True
            base_key = key & Keyboard.KEY
            if base_key in (Keyboard.UP, ord("W"), ord("w")):
                up_pressed = True
            elif base_key in (Keyboard.DOWN, ord("S"), ord("s")):
                down_pressed = True
            elif base_key in (Keyboard.LEFT, ord("A"), ord("a")):
                left_pressed = True
            elif base_key in (Keyboard.RIGHT, ord("D"), ord("d")):
                right_pressed = True
            elif base_key == ord(" "):
                space_pressed = True

        if shift_pressed != prev_shift_state:
            driver.setCustomData("SHIFT:1" if shift_pressed else "SHIFT:0")
            prev_shift_state = shift_pressed

        # 3. Fail-safe check (AGENT.md Section 3.4: emergency brake if critical sensors freeze)
        if not telemetry.sensors_healthy:
            if not failsafe_logged:
                print(
                    f"[FAIL-SAFE] Critical sensors stale for {sensors.stale_steps} steps -> engaging emergency brake.",
                    flush=True,
                )
                failsafe_logged = True
            space_pressed = True
            up_pressed = False
            down_pressed = False
        else:
            failsafe_logged = False

        # 4. Determine accurately signed vehicle speed (km/h)
        raw_speed_kmh = driver.getCurrentSpeed()
        gps_signed_kmh = telemetry.signed_gps_speed_ms * 3.6
        if not math.isfinite(raw_speed_kmh) or step_idx <= 2:
            sign = -1.0 if lon_ctrl.gear == "R" else 1.0
            measured_speed_kmh = sign * (telemetry.gps_speed_ms * 3.6)
        else:
            measured_speed_kmh = float(raw_speed_kmh)
            # Ensure reverse motion always carries negative sign even if encoder speed is unsigned
            if lon_ctrl.gear == "R" and measured_speed_kmh > 0.0 and (
                gps_signed_kmh < -0.1 or lon_ctrl.target_speed_kmh < 0.0
            ):
                measured_speed_kmh = -abs(measured_speed_kmh)

        # 5. Update Longitudinal PID & Gear / Progressive Brake Controller
        lon_out = lon_ctrl.update(
            dt_sec=dt_sec,
            up_pressed=up_pressed,
            down_pressed=down_pressed,
            space_pressed=space_pressed,
            measured_speed_kmh=measured_speed_kmh,
        )

        # 6. Update Speed-Sensitive Ackermann Steering & Self-Centering Controller
        lat_out = lat_ctrl.update(
            dt_sec=dt_sec,
            left_pressed=left_pressed,
            right_pressed=right_pressed,
            measured_speed_kmh=measured_speed_kmh,
        )

        # 7. Commit buffered commands to Webots Driver actuators
        driver.setSteeringAngle(lat_out.steering_angle_rad)
        driver.setBrakeIntensity(lon_out.brake_intensity)
        driver.setCruisingSpeed(lon_out.commanded_cruising_speed_kmh)
        prev_indicator = set_indicator_safe(driver, lat_out.indicator_dir, prev_indicator)

        # 8. Render HUD telemetry every 2 steps for smooth FPS
        if step_idx % 2 == 0:
            hud.render(measured_speed_kmh, lon_out, lat_out, telemetry)

    return 0



if __name__ == "__main__":
    raise SystemExit(main())
