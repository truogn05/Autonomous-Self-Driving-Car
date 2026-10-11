"""Real-time HUD telemetry overlay rendered on the Webots Display node."""

import math
from control.longitudinal_pid import LongitudinalOutput
from control.steering_dynamics import SteeringOutput
from perception.sensor_manager import SensorTelemetry


class HudOverlay:
    """Draws speed, gear, steering angle bar, brake bar, GPS, and sensor health onto Display."""

    def __init__(self, display) -> None:
        self.display = display
        self.width = display.getWidth() if display is not None else 320
        self.height = display.getHeight() if display is not None else 140

    def render(
        self,
        measured_speed_kmh: float,
        lon_out: LongitudinalOutput,
        lat_out: SteeringOutput,
        telemetry: SensorTelemetry,
    ) -> None:
        if self.display is None:
            return

        d = self.display
        w, h = self.width, self.height

        # Dark slate dashboard background
        d.setAlpha(0.88)
        d.setColor(0x0F172A)
        d.fillRectangle(0, 0, w, h)

        # Top header bar
        d.setAlpha(1.0)
        d.setColor(0x1E293B)
        d.fillRectangle(0, 0, w, 24)
        d.setColor(0x38BDF8)
        d.setFont("Lucida Console", 10, True)
        d.drawText("BMW X5 ADAS TELEHUD", 8, 6)

        # Gear badge (D / N / R)
        gear_colors = {"D": 0x22C55E, "N": 0x64748B, "R": 0xF59E0B}
        d.setColor(gear_colors.get(lon_out.gear, 0x64748B))
        d.fillRectangle(w - 64, 3, 56, 18)
        d.setColor(0x0F172A)
        d.drawText(f"GEAR {lon_out.gear}", w - 58, 6)

        # Speed & Target Speed
        d.setColor(0xF8FAFC)
        d.setFont("Lucida Console", 12, True)
        d.drawText(f"SPD: {measured_speed_kmh:+5.1f} km/h", 8, 30)
        d.setFont("Lucida Console", 9, True)
        d.setColor(0x94A3B8)
        d.drawText(f"TGT: {lon_out.target_speed_kmh:+5.1f} km/h", 185, 32)

        # Steering angle & visual center-zero bar
        steer_deg = math.degrees(lat_out.steering_angle_rad)
        max_deg = max(1.0, math.degrees(lat_out.max_allowed_angle_rad))
        d.setColor(0xE2E8F0)
        d.drawText(f"STEER: {steer_deg:+5.1f} deg (Max {max_deg:4.1f})", 8, 52)

        bar_x, bar_y, bar_w, bar_h = 8, 66, 190, 10
        d.setColor(0x334155)
        d.fillRectangle(bar_x, bar_y, bar_w, bar_h)
        mid_x = bar_x + bar_w // 2
        d.setColor(0x94A3B8)
        d.drawLine(mid_x, bar_y - 2, mid_x, bar_y + bar_h + 2)

        # In Webots Driver: negative = Left, positive = Right
        norm_steer = max(-1.0, min(1.0, lat_out.steering_angle_rad / max(0.05, lat_out.max_allowed_angle_rad)))
        fill_w = int(abs(norm_steer) * (bar_w // 2))
        if fill_w > 0:
            d.setColor(0x38BDF8 if abs(norm_steer) < 0.85 else 0xF97316)
            if norm_steer < 0:
                d.fillRectangle(mid_x - fill_w, bar_y + 1, fill_w, bar_h - 2)
            else:
                d.fillRectangle(mid_x, bar_y + 1, fill_w, bar_h - 2)

        # Brake intensity bar
        brk_pct = int(round(lon_out.brake_intensity * 100))
        d.setColor(0xF43F5E if brk_pct > 0 else 0x94A3B8)
        d.drawText(f"BRK: {brk_pct:3d}%", 210, 52)
        d.setColor(0x334155)
        d.fillRectangle(210, 66, 98, 10)
        if brk_pct > 0:
            d.setColor(0xEF4444)
            d.fillRectangle(210, 67, int(0.98 * brk_pct), 8)

        # GPS Position & Heading
        gx, gy, _ = telemetry.gps_xyz
        d.setColor(0xCBD5E1)
        d.drawText(
            f"GPS: ({gx:+6.1f}, {gy:+6.1f})m  HDG:{telemetry.heading_deg:+6.1f}d",
            8,
            84,
        )

        # Obstacle distances (Sonar / Radar / LiDAR)
        d.setColor(0x34D399 if telemetry.front_min_dist_m > 2.5 else 0xFBBF24)
        d.drawText(
            f"SONAR F:{telemetry.front_min_dist_m:4.1f}m R:{telemetry.rear_min_dist_m:4.1f}m | LIDAR:{telemetry.lidar_front_min_m:4.1f}m",
            8,
            102,
        )

        # Footer controls hint
        d.setColor(0x64748B)
        d.drawText("[UP/DN] Accel/Rev  [L/R] Steer  [SPACE] Brake", 8, 122)
