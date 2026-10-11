"""Real-time HUD telemetry overlay rendered on the Webots Display node."""

import math
from control.longitudinal_pid import LongitudinalOutput
from control.steering_dynamics import SteeringOutput
from perception.sensor_manager import SensorTelemetry


class HudOverlay:
    """Minimalist 3-row HUD overlay: Speed & Gear, Center-Zero Steering Bar, and Brake/Obstacle status."""

    def __init__(self, display) -> None:
        self.display = display
        self.width = display.getWidth() if display is not None else 210
        self.height = display.getHeight() if display is not None else 76
        if self.display is not None:
            # Set font ONCE at initialization; calling setFont() every frame leaks memory in Webots R2025a
            self.display.setFont("Lucida Console", 10, True)

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

        # Clean dark translucent background + subtle border
        d.setAlpha(0.82)
        d.setColor(0x0F172A)
        d.fillRectangle(0, 0, w, h)
        d.setAlpha(1.0)
        d.setColor(0x334155)
        d.drawRectangle(0, 0, w - 1, h - 1)

        # Row 1: Speed (km/h) & Gear badge [D / N / R]
        spd_val = abs(int(round(measured_speed_kmh)))
        d.setColor(0xF8FAFC)
        d.drawText(f"SPD: {spd_val:3d} km/h", 8, 7)

        gear_colors = {"D": 0x22C55E, "N": 0x64748B, "R": 0xF59E0B}
        d.setColor(gear_colors.get(lon_out.gear, 0x64748B))
        d.fillRectangle(w - 42, 5, 34, 17)
        d.setColor(0x0F172A)
        d.drawText(f"[{lon_out.gear}]", w - 35, 7)

        # Row 2: Center-zero Steering Bar & Angle in degrees
        steer_deg = int(round(math.degrees(lat_out.steering_angle_rad)))
        d.setColor(0xCBD5E1)
        d.drawText("STR", 8, 30)

        bar_x, bar_y, bar_w, bar_h = 40, 32, 108, 10
        d.setColor(0x1E293B)
        d.fillRectangle(bar_x, bar_y, bar_w, bar_h)
        mid_x = bar_x + bar_w // 2

        norm_steer = max(-1.0, min(1.0, lat_out.steering_angle_rad / max(0.05, lat_out.max_allowed_angle_rad)))
        fill_w = int(abs(norm_steer) * (bar_w // 2))
        if fill_w > 0:
            d.setColor(0x38BDF8 if abs(norm_steer) < 0.85 else 0xF97316)
            if norm_steer < 0:
                d.fillRectangle(mid_x - fill_w, bar_y + 1, fill_w, bar_h - 2)
            else:
                d.fillRectangle(mid_x, bar_y + 1, fill_w, bar_h - 2)

        d.setColor(0x94A3B8)
        d.drawLine(mid_x, bar_y - 1, mid_x, bar_y + bar_h)
        d.setColor(0xE2E8F0)
        d.drawText(f"{steer_deg:+3d}d", w - 54, 30)

        # Row 3: Brake & Front Obstacle Status
        brk_pct = int(round(lon_out.brake_intensity * 100))
        if brk_pct > 0:
            d.setColor(0xEF4444)
            d.drawText(f"BRK: {brk_pct:3d}%", 8, 54)
        else:
            d.setColor(0x94A3B8)
            d.drawText("BRK:  OFF", 8, 54)

        nearest_front = min(telemetry.front_min_dist_m, telemetry.lidar_front_min_m)
        if nearest_front < 3.0:
            d.setColor(0xFBBF24 if nearest_front > 1.5 else 0xEF4444)
            d.drawText(f"WARN {nearest_front:3.1f}m", w - 92, 54)
        else:
            d.setColor(0x34D399)
            d.drawText("CLEAR", w - 60, 54)

