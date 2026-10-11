"""Multi-sensor manager for BMW X5: Camera, 4-Layer 3D LiDAR, Radar, GPS, IMU, Gyro, Accel, Compass, and 6x Sonar."""

from dataclasses import dataclass, field
import math
from config.vehicle_params import DEFAULT_DT_SEC, MAX_STALE_SENSOR_STEPS, STARTUP_GRACE_STEPS


@dataclass
class SensorTelemetry:
    """Consolidated telemetry snapshot read each simulation step."""
    gps_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    gps_speed_ms: float = 0.0
    signed_gps_speed_ms: float = 0.0
    roll_pitch_yaw_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)
    heading_deg: float = 180.0
    gyro_rad_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    accel_m_s2: tuple[float, float, float] = (0.0, 0.0, 0.0)
    front_sonar_m: dict[str, float] = field(default_factory=lambda: {"L": 8.0, "C": 8.0, "R": 8.0})
    rear_sonar_m: dict[str, float] = field(default_factory=lambda: {"L": 8.0, "C": 8.0, "R": 8.0})
    front_min_dist_m: float = 8.0
    rear_min_dist_m: float = 8.0
    radar_nearest_m: float = 120.0
    radar_targets_count: int = 0
    lidar_front_min_m: float = 100.0
    camera_objects_count: int = 0
    sensors_healthy: bool = True
    active_sensor_count: int = 0


class SensorManager:
    """Initializes, enables, and reads the full autonomous sensor suite on BmwX5."""

    SONAR_FRONT_NAMES = {
        "L": "front_left_ds",
        "C": "front_center_ds",
        "R": "front_right_ds",
    }
    SONAR_REAR_NAMES = {
        "L": "rear_left_ds",
        "C": "rear_center_ds",
        "R": "rear_right_ds",
    }

    def __init__(self, robot, timestep: int) -> None:
        self.robot = robot
        self.timestep = max(16, int(timestep))
        self.dt_sec = self.timestep / 1000.0 if self.timestep > 0 else DEFAULT_DT_SEC
        self.lidar_period = self.timestep * 2  # Run 4-layer 3D LiDAR at half rate for high FPS
        self.stale_steps = 0
        self.step_count = 0
        self.last_lidar_front_min_m = 100.0
        self.prev_gps_xyz: tuple[float, float, float] | None = None

        self.camera = self._enable_device("front_camera", self.timestep)

        self.lidar = self._enable_device("top_lidar", self.lidar_period)
        self.radar = self._enable_device("front_radar", self.timestep)
        self.gps = self._enable_device("gps", self.timestep)
        self.imu = self._enable_device("inertial_unit", self.timestep)
        self.gyro = self._enable_device("gyro", self.timestep)
        self.accel = self._enable_device("accelerometer", self.timestep)
        self.compass = self._enable_device("compass", self.timestep)

        self.front_sonars = {
            k: self._enable_device(name, self.timestep)
            for k, name in self.SONAR_FRONT_NAMES.items()
        }
        self.rear_sonars = {
            k: self._enable_device(name, self.timestep)
            for k, name in self.SONAR_REAR_NAMES.items()
        }

        self.hud_display = self._get_device("hud_display")

    def _get_device(self, name: str):
        try:
            return self.robot.getDevice(name)
        except Exception:
            return None

    def _enable_device(self, name: str, period: int):
        dev = self._get_device(name)
        if dev is not None and hasattr(dev, "enable"):
            dev.enable(period)
        return dev

    def read(self) -> SensorTelemetry:
        """Read all enabled sensors after at least one simulation step."""
        self.step_count += 1
        telemetry = SensorTelemetry()
        active = 0
        gps_ok = True
        imu_ok = True

        # 1. GPS
        if self.gps is not None:
            active += 1
            vals = self.gps.getValues()
            spd = self.gps.getSpeed()
            if len(vals) >= 3 and all(math.isfinite(v) for v in vals):
                telemetry.gps_xyz = (float(vals[0]), float(vals[1]), float(vals[2]))
            else:
                gps_ok = False
            if math.isfinite(spd):
                telemetry.gps_speed_ms = float(spd)

        # 2. IMU (Roll, Pitch, Yaw in ENU)
        if self.imu is not None:
            active += 1
            rpy = self.imu.getRollPitchYaw()
            if len(rpy) >= 3 and all(math.isfinite(v) for v in rpy):
                telemetry.roll_pitch_yaw_rad = (float(rpy[0]), float(rpy[1]), float(rpy[2]))
                telemetry.heading_deg = math.degrees(float(rpy[2]))
            else:
                imu_ok = False

        # Compute signed longitudinal speed along vehicle heading from GPS displacement
        if gps_ok and self.prev_gps_xyz is not None:
            dx = telemetry.gps_xyz[0] - self.prev_gps_xyz[0]
            dy = telemetry.gps_xyz[1] - self.prev_gps_xyz[1]
            yaw = telemetry.roll_pitch_yaw_rad[2]
            forward_proj = dx * math.cos(yaw) + dy * math.sin(yaw)
            sign = -1.0 if forward_proj < -1e-4 else 1.0
            telemetry.signed_gps_speed_ms = sign * telemetry.gps_speed_ms
        else:
            telemetry.signed_gps_speed_ms = telemetry.gps_speed_ms
        if gps_ok:
            self.prev_gps_xyz = telemetry.gps_xyz

        # Debounce critical sensor health with startup grace period
        if gps_ok and imu_ok:
            self.stale_steps = 0
        elif self.step_count > STARTUP_GRACE_STEPS:
            self.stale_steps += 1

        # 3. Compass fallback/validation
        if self.compass is not None:
            active += 1

        # 4. Gyroscope & Accelerometer
        if self.gyro is not None:
            active += 1
            gvals = self.gyro.getValues()
            if len(gvals) >= 3 and all(math.isfinite(v) for v in gvals):
                telemetry.gyro_rad_s = (float(gvals[0]), float(gvals[1]), float(gvals[2]))

        if self.accel is not None:
            active += 1
            avals = self.accel.getValues()
            if len(avals) >= 3 and all(math.isfinite(v) for v in avals):
                telemetry.accel_m_s2 = (float(avals[0]), float(avals[1]), float(avals[2]))

        # 5. Front & Rear Ultrasonic DistanceSensors
        for k, dev in self.front_sonars.items():
            if dev is not None:
                active += 1
                v = dev.getValue()
                if math.isfinite(v):
                    telemetry.front_sonar_m[k] = float(v)
        telemetry.front_min_dist_m = min(telemetry.front_sonar_m.values())

        for k, dev in self.rear_sonars.items():
            if dev is not None:
                active += 1
                v = dev.getValue()
                if math.isfinite(v):
                    telemetry.rear_sonar_m[k] = float(v)
        telemetry.rear_min_dist_m = min(telemetry.rear_sonar_m.values())

        # 6. Front Radar
        if self.radar is not None:
            active += 1
            targets = self.radar.getTargets()
            telemetry.radar_targets_count = len(targets)
            if targets:
                nearest = min(
                    (t.distance for t in targets if math.isfinite(t.distance) and t.distance > 0.1),
                    default=120.0,
                )
                telemetry.radar_nearest_m = float(nearest)

        # 7. Roof 3D LiDAR (sample horizontal middle layer every 2 steps, bypassing R2025a getLayerRangeImage typo)
        if self.lidar is not None:
            active += 1
            if self.step_count % 2 == 0 or self.step_count == 1:
                try:
                    from controller.wb import wb

                    n_layers = max(1, int(self.lidar.getNumberOfLayers()))
                    mid_layer = n_layers // 2
                    h_res = max(1, int(self.lidar.getHorizontalResolution()))
                    ptr = wb.wb_lidar_get_layer_range_image(self.lidar._tag, mid_layer)
                    if ptr:
                        c0 = int(h_res * 0.40)
                        c1 = int(h_res * 0.60)
                        valid = [
                            float(ptr[i])
                            for i in range(c0, c1)
                            if math.isfinite(ptr[i]) and 0.6 < ptr[i] < 100.0
                        ]
                        self.last_lidar_front_min_m = min(valid) if valid else 100.0
                except Exception:
                    pass
            telemetry.lidar_front_min_m = self.last_lidar_front_min_m

        # 8. Front Camera
        if self.camera is not None:
            active += 1
            if hasattr(self.camera, "hasRecognition") and self.camera.hasRecognition():
                try:
                    telemetry.camera_objects_count = int(self.camera.getRecognitionNumberOfObjects())
                except Exception:
                    pass

        telemetry.active_sensor_count = active
        telemetry.sensors_healthy = (
            self.step_count <= STARTUP_GRACE_STEPS or self.stale_steps <= MAX_STALE_SENSOR_STEPS
        )
        return telemetry

