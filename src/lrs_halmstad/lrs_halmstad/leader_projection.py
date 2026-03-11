from __future__ import annotations

import math
from typing import Optional

import numpy as np

from lrs_halmstad.leader_types import CameraModel, Detection2D, PoseEstimate, ProjectedTarget


class LeaderProjector:
    def __init__(
        self,
        *,
        range_mode: str,
        use_depth_range: bool,
        depth_scale: float,
        depth_min_m: float,
        depth_max_m: float,
        constant_range_m: float,
        ground_min_range_m: float,
        ground_max_range_m: float,
        target_ground_z_m: float,
        cam_yaw_offset_deg: float,
        cam_pitch_offset_deg: float,
        cam_x_offset_m: float,
        cam_y_offset_m: float,
        cam_z_offset_m: float,
        smooth_alpha: float,
        xy_smooth_alpha: float,
        max_xy_jump_m: float,
        max_target_speed_mps: float,
        max_bearing_jump_deg: float,
    ) -> None:
        self.range_mode = str(range_mode).strip().lower() or "auto"
        self.use_depth_range = bool(use_depth_range)
        self.depth_scale = float(depth_scale)
        self.depth_min_m = float(depth_min_m)
        self.depth_max_m = float(depth_max_m)
        self.constant_range_m = float(constant_range_m)
        self.ground_min_range_m = float(ground_min_range_m)
        self.ground_max_range_m = float(ground_max_range_m)
        self.target_ground_z_m = float(target_ground_z_m)
        self.cam_yaw_offset_rad = math.radians(float(cam_yaw_offset_deg))
        self.cam_pitch_offset_rad = math.radians(float(cam_pitch_offset_deg))
        self.cam_x_offset_m = float(cam_x_offset_m)
        self.cam_y_offset_m = float(cam_y_offset_m)
        self.cam_z_offset_m = float(cam_z_offset_m)
        self.smooth_alpha = float(smooth_alpha)
        self.xy_smooth_alpha = float(xy_smooth_alpha)
        self.max_xy_jump_m = float(max_xy_jump_m)
        self.max_target_speed_mps = float(max_target_speed_mps)
        self.max_bearing_jump_deg = float(max_bearing_jump_deg)

        self.smoothed_bearing: Optional[float] = None
        self.smoothed_range_m: Optional[float] = None
        self.smoothed_xy: Optional[tuple[float, float]] = None
        self.prev_estimate_xy: Optional[tuple[float, float]] = None
        self.prev_estimate_stamp_ns: Optional[int] = None
        self.prev_heading_yaw: Optional[float] = None
        self.last_bearing_rad: Optional[float] = None
        self.last_range_used_m = -1.0
        self.last_bearing_used_deg = 0.0

    @staticmethod
    def camera_model_from_info(msg) -> Optional[CameraModel]:
        if len(msg.k) < 9:
            return None
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        cx = float(msg.k[2])
        cy = float(msg.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None
        return CameraModel(fx=fx, fy=fy, cx=cx, cy=cy, width=int(msg.width), height=int(msg.height))

    def reset(self) -> None:
        self.smoothed_bearing = None
        self.smoothed_range_m = None
        self.smoothed_xy = None

    def note_track_reset(self) -> None:
        self.reset()
        self.prev_heading_yaw = None

    def _depth_to_array_m(self, msg) -> Optional[np.ndarray]:
        try:
            h = int(msg.height)
            w = int(msg.width)
            if h <= 0 or w <= 0:
                return None
            if msg.encoding in ("32FC1", "32FC"):
                return np.frombuffer(msg.data, dtype=np.float32).reshape((h, w)).copy()
            if msg.encoding in ("16UC1", "16UC"):
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w)).astype(np.float32)
                arr *= self.depth_scale
                return arr
        except Exception:
            return None
        return None

    def _sample_depth_range(self, det: Detection2D, depth_msg) -> Optional[float]:
        if not self.use_depth_range or depth_msg is None:
            return None
        depth = self._depth_to_array_m(depth_msg)
        if depth is None:
            return None
        u = int(round(det.u))
        v = int(round(det.v))
        h, w = depth.shape[:2]
        if u < 0 or u >= w or v < 0 or v >= h:
            return None
        radius = 2
        patch = depth[max(0, v - radius):min(h, v + radius + 1), max(0, u - radius):min(w, u + radius + 1)]
        patch = patch[np.isfinite(patch)]
        patch = patch[(patch >= self.depth_min_m) & (patch <= self.depth_max_m)]
        if patch.size == 0:
            return None
        return float(np.median(patch))

    def _ground_range_from_pixel(self, det: Detection2D, cam: CameraModel, uav_pose: PoseEstimate) -> Optional[float]:
        x_n = (det.u - cam.cx) / cam.fx
        y_n = (det.v - cam.cy) / cam.fy
        pitch_img = math.atan2(y_n, math.sqrt(1.0 + x_n * x_n))
        cam_world_z = uav_pose.z + self.cam_z_offset_m
        dz = self.target_ground_z_m - cam_world_z
        elev = self.cam_pitch_offset_rad + pitch_img
        tan_elev = math.tan(elev)
        if abs(tan_elev) < 1e-3:
            return None
        horiz_range = dz / tan_elev
        if not math.isfinite(horiz_range) or horiz_range <= 0.0:
            return None
        if horiz_range < self.ground_min_range_m:
            return None
        if self.ground_max_range_m > 0.0 and horiz_range > self.ground_max_range_m:
            return None
        return float(horiz_range)

    def estimate(self, det: Detection2D, cam: CameraModel, uav_pose: PoseEstimate, depth_msg, stamp_ns: int) -> ProjectedTarget:
        x_n = (det.u - cam.cx) / cam.fx
        bearing = self.cam_yaw_offset_rad + math.atan2(x_n, 1.0)

        depth_range = self._sample_depth_range(det, depth_msg)
        ground_range = self._ground_range_from_pixel(det, cam, uav_pose)

        range_m = self.constant_range_m
        range_source = "const"
        if self.range_mode == "depth":
            if depth_range is not None:
                range_m = depth_range
                range_source = "depth"
        elif self.range_mode == "ground":
            if ground_range is not None:
                range_m = ground_range
                range_source = "ground"
        elif self.range_mode == "auto":
            if depth_range is not None:
                range_m = depth_range
                range_source = "depth"
            elif ground_range is not None:
                range_m = ground_range
                range_source = "ground"

        if self.smoothed_bearing is None or self.smooth_alpha >= 1.0:
            self.smoothed_bearing = bearing
        else:
            self.smoothed_bearing = self.smooth_alpha * bearing + (1.0 - self.smooth_alpha) * self.smoothed_bearing

        if self.smoothed_range_m is None or self.smooth_alpha >= 1.0:
            self.smoothed_range_m = range_m
        else:
            self.smoothed_range_m = self.smooth_alpha * range_m + (1.0 - self.smooth_alpha) * self.smoothed_range_m

        bearing = self.smoothed_bearing
        range_m = self.smoothed_range_m

        cam_world_x = uav_pose.x + self.cam_x_offset_m * math.cos(uav_pose.yaw) - self.cam_y_offset_m * math.sin(uav_pose.yaw)
        cam_world_y = uav_pose.y + self.cam_x_offset_m * math.sin(uav_pose.yaw) + self.cam_y_offset_m * math.cos(uav_pose.yaw)
        x = cam_world_x + range_m * math.cos(uav_pose.yaw + bearing)
        y = cam_world_y + range_m * math.sin(uav_pose.yaw + bearing)

        if self.smoothed_xy is None or self.xy_smooth_alpha >= 1.0:
            self.smoothed_xy = (x, y)
        else:
            self.smoothed_xy = (
                self.xy_smooth_alpha * x + (1.0 - self.xy_smooth_alpha) * self.smoothed_xy[0],
                self.xy_smooth_alpha * y + (1.0 - self.xy_smooth_alpha) * self.smoothed_xy[1],
            )
        x, y = self.smoothed_xy

        yaw = self.prev_heading_yaw if self.prev_heading_yaw is not None else (uav_pose.yaw + bearing)
        if self.prev_estimate_xy is not None and self.prev_estimate_stamp_ns is not None:
            dt = max(1e-9, (stamp_ns - self.prev_estimate_stamp_ns) * 1e-9)
            dx = x - self.prev_estimate_xy[0]
            dy = y - self.prev_estimate_xy[1]
            if math.hypot(dx, dy) > 0.05:
                yaw = math.atan2(dy, dx)

        self.last_range_used_m = float(range_m)
        self.last_bearing_used_deg = math.degrees(bearing)
        return ProjectedTarget(
            pose=PoseEstimate(x=x, y=y, z=self.target_ground_z_m, yaw=yaw),
            range_m=float(range_m),
            range_source=range_source,
            bearing_rad=float(bearing),
        )

    def is_sane(self, pose: PoseEstimate, bearing_rad: float, stamp_ns: int) -> tuple[bool, str]:
        if self.prev_estimate_xy is None or self.prev_estimate_stamp_ns is None:
            return (True, "none")
        dt = max(1e-6, (stamp_ns - self.prev_estimate_stamp_ns) * 1e-9)
        dx = pose.x - self.prev_estimate_xy[0]
        dy = pose.y - self.prev_estimate_xy[1]
        jump = math.hypot(dx, dy)
        if self.max_xy_jump_m > 0.0 and jump > self.max_xy_jump_m:
            return (False, "jump_xy")
        speed = jump / dt
        if self.max_target_speed_mps > 0.0 and speed > self.max_target_speed_mps:
            return (False, "speed")
        if self.last_bearing_rad is not None and self.max_bearing_jump_deg > 0.0:
            delta = abs((bearing_rad - self.last_bearing_rad + math.pi) % (2.0 * math.pi) - math.pi)
            if math.degrees(delta) > self.max_bearing_jump_deg:
                return (False, "bearing_jump")
        return (True, "none")

    def commit(self, pose: PoseEstimate, bearing_rad: float, stamp_ns: int) -> None:
        self.prev_estimate_xy = (pose.x, pose.y)
        self.prev_estimate_stamp_ns = stamp_ns
        self.prev_heading_yaw = pose.yaw
        self.last_bearing_rad = bearing_rad
