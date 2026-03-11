#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_path
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from lrs_halmstad.follow_math import coerce_bool, quat_from_yaw, yaw_from_quat
from lrs_halmstad.leader_projection import LeaderProjector
from lrs_halmstad.leader_tracking import UltralyticsTargetTracker
from lrs_halmstad.leader_types import CameraModel, Detection2D, PoseEstimate

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    cv2 = None


def _workspace_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _default_models_root() -> str:
    configured_root = os.environ.get("LRS_HALMSTAD_MODELS_ROOT", "").strip()
    if configured_root:
        return os.path.expanduser(configured_root)
    try:
        pkg_share = get_package_share_path("lrs_halmstad").resolve()
        for parent in pkg_share.parents:
            candidate = parent / "src" / "lrs_halmstad"
            if candidate.is_dir():
                return str(parent / "models")
    except Exception:
        pass
    return str((_workspace_root() / "models").resolve())


class LeaderEstimator(Node):
    def __init__(self) -> None:
        super().__init__("leader_estimator")
        dyn_num = ParameterDescriptor(dynamic_typing=True)
        self._declare_parameters(dyn_num)
        self._read_parameters()
        self._validate_parameters()
        self._init_state()
        self._init_ros()
        self.emit_event("ESTIMATOR_NODE_START")
        self.get_logger().info(
            "[leader_estimator] Started: "
            f"image={self.camera_topic}, camera_info={self.camera_info_topic}, depth={self.depth_topic or 'disabled'}, "
            f"uav_pose={self.uav_pose_topic}, out={self.out_topic}, status={self.status_topic}, "
            f"weights={self.tracker.weights_path or '<unset>'}, tracking={self.tracker.tracker_enable}, "
            f"tracker_name={self.tracker.tracker_name}, tracker_yaml={self.tracker.tracker_config_path}, "
            f"range_mode={self.range_mode}, target_class_name={self.target_class_name or '<any>'}, "
            f"target_class_id={self.target_class_id}"
        )

    def _declare_parameters(self, dyn_num: ParameterDescriptor) -> None:
        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("camera_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("depth_topic", "")
        self.declare_parameter("uav_pose_topic", "")
        self.declare_parameter("out_topic", "/coord/leader_estimate")
        self.declare_parameter("status_topic", "/coord/leader_estimate_status")
        self.declare_parameter("estimate_error_topic", "/coord/leader_estimate_error")
        self.declare_parameter("leader_actual_pose_enable", True)
        self.declare_parameter("leader_actual_pose_topic", "/a201_0000/amcl_pose_odom")
        self.declare_parameter("publish_status", True)
        self.declare_parameter("debug_image_topic", "/coord/leader_debug_image")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("debug_image_max_width", 960)
        self.declare_parameter("fault_status_topic", "/coord/leader_estimate_fault")
        self.declare_parameter("publish_fault_status", True)
        self.declare_parameter("event_topic", "/coord/events")
        self.declare_parameter("publish_events", True)

        self.declare_parameter("est_hz", 20.0, dyn_num)
        self.declare_parameter("image_timeout_s", 1.0)
        self.declare_parameter("uav_pose_timeout_s", 1.0)
        self.declare_parameter("leader_actual_pose_timeout_s", 2.0)

        self.declare_parameter("yolo_weights", "")
        self.declare_parameter("models_root", _default_models_root())
        self.declare_parameter("device", "cpu")
        self.declare_parameter("conf_threshold", 0.12, dyn_num)
        self.declare_parameter("iou_threshold", 0.45, dyn_num)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("target_class_id", -1)
        self.declare_parameter("target_class_name", "")
        self.declare_parameter("tracker_enable", True)
        self.declare_parameter("tracker_name", "botsort")
        self.declare_parameter("tracker_yaml", "")
        self.declare_parameter("tracker_persist", True)

        self.declare_parameter("smooth_alpha", 0.5, dyn_num)
        self.declare_parameter("xy_smooth_alpha", 0.8, dyn_num)
        self.declare_parameter("max_xy_jump_m", 20.0, dyn_num)
        self.declare_parameter("max_target_speed_mps", 20.0, dyn_num)
        self.declare_parameter("max_bearing_jump_deg", 180.0, dyn_num)
        self.declare_parameter("ok_debounce_frames", 1, dyn_num)
        self.declare_parameter("bad_debounce_frames", 1, dyn_num)

        self.declare_parameter("constant_range_m", 5.0, dyn_num)
        self.declare_parameter("range_mode", "auto")
        self.declare_parameter("use_depth_range", True)
        self.declare_parameter("depth_scale", 0.001, dyn_num)
        self.declare_parameter("depth_min_m", 0.2, dyn_num)
        self.declare_parameter("depth_max_m", 100.0, dyn_num)
        self.declare_parameter("target_ground_z_m", 0.0, dyn_num)
        self.declare_parameter("ground_min_range_m", 2.0, dyn_num)
        self.declare_parameter("ground_max_range_m", 50.0, dyn_num)
        self.declare_parameter("cam_yaw_offset_deg", 0.0, dyn_num)
        self.declare_parameter("cam_pitch_offset_deg", 0.0, dyn_num)
        self.declare_parameter("cam_x_offset_m", 0.0, dyn_num)
        self.declare_parameter("cam_y_offset_m", 0.0, dyn_num)
        self.declare_parameter("cam_z_offset_m", 0.0, dyn_num)

        self.declare_parameter("bootstrap_uav_pose_enabled", True)
        self.declare_parameter("bootstrap_uav_x", 0.0, dyn_num)
        self.declare_parameter("bootstrap_uav_y", 0.0, dyn_num)
        self.declare_parameter("bootstrap_uav_yaw", 0.0, dyn_num)

    def _read_parameters(self) -> None:
        self.uav_name = str(self.get_parameter("uav_name").value)
        self.camera_topic = str(self.get_parameter("camera_topic").value) or f"/{self.uav_name}/camera0/image_raw"
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value) or f"/{self.uav_name}/camera0/camera_info"
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.uav_pose_topic = str(self.get_parameter("uav_pose_topic").value) or f"/{self.uav_name}/pose"
        self.out_topic = str(self.get_parameter("out_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.estimate_error_topic = str(self.get_parameter("estimate_error_topic").value)
        self.leader_actual_pose_enable = coerce_bool(self.get_parameter("leader_actual_pose_enable").value)
        self.leader_actual_pose_topic = str(self.get_parameter("leader_actual_pose_topic").value)
        self.publish_status = coerce_bool(self.get_parameter("publish_status").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.publish_debug_image = coerce_bool(self.get_parameter("publish_debug_image").value)
        self.debug_image_max_width = int(self.get_parameter("debug_image_max_width").value)
        self.fault_status_topic = str(self.get_parameter("fault_status_topic").value)
        self.publish_fault_status = coerce_bool(self.get_parameter("publish_fault_status").value)
        self.event_topic = str(self.get_parameter("event_topic").value)
        self.publish_events = coerce_bool(self.get_parameter("publish_events").value)

        self.est_hz = float(self.get_parameter("est_hz").value)
        self.image_timeout_s = float(self.get_parameter("image_timeout_s").value)
        self.uav_pose_timeout_s = float(self.get_parameter("uav_pose_timeout_s").value)
        self.leader_actual_pose_timeout_s = float(self.get_parameter("leader_actual_pose_timeout_s").value)

        self.models_root = str(self.get_parameter("models_root").value)
        self.yolo_weights = str(self.get_parameter("yolo_weights").value).strip()
        self.device = str(self.get_parameter("device").value).strip() or "cpu"
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.target_class_id = int(self.get_parameter("target_class_id").value)
        self.target_class_name = str(self.get_parameter("target_class_name").value).strip()
        self.tracker_enable = coerce_bool(self.get_parameter("tracker_enable").value)
        self.tracker_name = str(self.get_parameter("tracker_name").value).strip().lower() or "botsort"
        self.tracker_yaml = str(self.get_parameter("tracker_yaml").value).strip()
        self.tracker_persist = coerce_bool(self.get_parameter("tracker_persist").value)

        self.smooth_alpha = float(self.get_parameter("smooth_alpha").value)
        self.xy_smooth_alpha = float(self.get_parameter("xy_smooth_alpha").value)
        self.max_xy_jump_m = float(self.get_parameter("max_xy_jump_m").value)
        self.max_target_speed_mps = float(self.get_parameter("max_target_speed_mps").value)
        self.max_bearing_jump_deg = float(self.get_parameter("max_bearing_jump_deg").value)
        self.ok_debounce_frames = int(self.get_parameter("ok_debounce_frames").value)
        self.bad_debounce_frames = int(self.get_parameter("bad_debounce_frames").value)

        self.constant_range_m = float(self.get_parameter("constant_range_m").value)
        self.range_mode = str(self.get_parameter("range_mode").value).strip().lower() or "auto"
        self.use_depth_range = coerce_bool(self.get_parameter("use_depth_range").value)
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.depth_min_m = float(self.get_parameter("depth_min_m").value)
        self.depth_max_m = float(self.get_parameter("depth_max_m").value)
        self.target_ground_z_m = float(self.get_parameter("target_ground_z_m").value)
        self.ground_min_range_m = float(self.get_parameter("ground_min_range_m").value)
        self.ground_max_range_m = float(self.get_parameter("ground_max_range_m").value)
        self.cam_yaw_offset_deg = float(self.get_parameter("cam_yaw_offset_deg").value)
        self.cam_pitch_offset_deg = float(self.get_parameter("cam_pitch_offset_deg").value)
        self.cam_x_offset_m = float(self.get_parameter("cam_x_offset_m").value)
        self.cam_y_offset_m = float(self.get_parameter("cam_y_offset_m").value)
        self.cam_z_offset_m = float(self.get_parameter("cam_z_offset_m").value)

        self.bootstrap_uav_pose_enabled = coerce_bool(self.get_parameter("bootstrap_uav_pose_enabled").value)
        self.bootstrap_uav_x = float(self.get_parameter("bootstrap_uav_x").value)
        self.bootstrap_uav_y = float(self.get_parameter("bootstrap_uav_y").value)
        self.bootstrap_uav_yaw = float(self.get_parameter("bootstrap_uav_yaw").value)

    def _validate_parameters(self) -> None:
        if self.est_hz <= 0.0:
            raise ValueError("est_hz must be > 0")
        if self.ok_debounce_frames < 1 or self.bad_debounce_frames < 1:
            raise ValueError("debounce frames must be >= 1")
        if not (0.0 <= self.smooth_alpha <= 1.0):
            raise ValueError("smooth_alpha must be in [0,1]")
        if not (0.0 <= self.xy_smooth_alpha <= 1.0):
            raise ValueError("xy_smooth_alpha must be in [0,1]")
        if self.range_mode not in {"auto", "depth", "ground", "const"}:
            raise ValueError("range_mode must be one of auto|depth|ground|const")
        if self.debug_image_max_width < 0:
            raise ValueError("debug_image_max_width must be >= 0")

    def _init_state(self) -> None:
        self.camera_model: Optional[CameraModel] = None
        self.last_image_msg: Optional[Image] = None
        self.last_image_stamp: Optional[Time] = None
        self.last_depth_msg: Optional[Image] = None
        self.last_image_recv_walltime: Optional[float] = None

        self.uav_pose = PoseEstimate(self.bootstrap_uav_x, self.bootstrap_uav_y, 0.0, self.bootstrap_uav_yaw)
        self.have_uav_pose = self.bootstrap_uav_pose_enabled
        self.have_real_uav_pose = False
        self.last_uav_pose_stamp: Optional[Time] = None
        self.uav_pose_source = "bootstrap" if self.bootstrap_uav_pose_enabled else "none"

        self.last_actual_leader_pose: Optional[PoseEstimate] = None
        self.last_actual_leader_pose_stamp: Optional[Time] = None

        self.last_status: Optional[str] = None
        self.last_fault_state = "none"
        self.last_fault_reason = "none"
        self.last_fault_stamp: Optional[Time] = None
        self.last_debug_state = "WAITING"

        self.last_det_conf = -1.0
        self.last_latency_ms = -1.0
        self.last_range_source = "none"
        self.last_reject_reason = "none"
        self.last_estimate_pose: Optional[PoseEstimate] = None
        self.last_estimate_error_dx_m: Optional[float] = None
        self.last_estimate_error_dy_m: Optional[float] = None
        self.last_estimate_error_m: Optional[float] = None
        self.last_selected_detection: Optional[Detection2D] = None

        self.good_det_streak = 0
        self.bad_det_streak = 0

        self.tracker = UltralyticsTargetTracker(
            models_root=self.models_root,
            yolo_weights=self.yolo_weights,
            device=self.device,
            conf_threshold=self.conf_threshold,
            iou_threshold=self.iou_threshold,
            imgsz=self.imgsz,
            target_class_id=self.target_class_id,
            target_class_name=self.target_class_name,
            tracker_enable=self.tracker_enable,
            tracker_name=self.tracker_name,
            tracker_yaml=self.tracker_yaml,
            tracker_persist=self.tracker_persist,
        )
        self.projector = LeaderProjector(
            range_mode=self.range_mode,
            use_depth_range=self.use_depth_range,
            depth_scale=self.depth_scale,
            depth_min_m=self.depth_min_m,
            depth_max_m=self.depth_max_m,
            constant_range_m=self.constant_range_m,
            ground_min_range_m=self.ground_min_range_m,
            ground_max_range_m=self.ground_max_range_m,
            target_ground_z_m=self.target_ground_z_m,
            cam_yaw_offset_deg=self.cam_yaw_offset_deg,
            cam_pitch_offset_deg=self.cam_pitch_offset_deg,
            cam_x_offset_m=self.cam_x_offset_m,
            cam_y_offset_m=self.cam_y_offset_m,
            cam_z_offset_m=self.cam_z_offset_m,
            smooth_alpha=self.smooth_alpha,
            xy_smooth_alpha=self.xy_smooth_alpha,
            max_xy_jump_m=self.max_xy_jump_m,
            max_target_speed_mps=self.max_target_speed_mps,
            max_bearing_jump_deg=self.max_bearing_jump_deg,
        )

    def _init_ros(self) -> None:
        sensor_qos = qos_profile_sensor_data
        self.image_sub = self.create_subscription(Image, self.camera_topic, self.on_image, sensor_qos)
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, sensor_qos)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.on_depth, sensor_qos) if self.depth_topic else None
        self.uav_pose_sub = self.create_subscription(PoseStamped, self.uav_pose_topic, self.on_uav_pose, 10)
        self.actual_leader_pose_sub = (
            self.create_subscription(Odometry, self.leader_actual_pose_topic, self.on_actual_leader_pose, 10)
            if self.leader_actual_pose_enable
            else None
        )

        self.estimate_pub = self.create_publisher(PoseStamped, self.out_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.events_pub = self.create_publisher(String, self.event_topic, 10)
        self.estimate_error_pub = (
            self.create_publisher(Vector3Stamped, self.estimate_error_topic, 10)
            if self.leader_actual_pose_enable
            else None
        )
        debug_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.debug_image_pub = self.create_publisher(Image, self.debug_image_topic, debug_qos) if self.publish_debug_image else None

        fault_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.fault_status_pub = (
            self.create_publisher(String, self.fault_status_topic, fault_qos)
            if self.publish_fault_status
            else None
        )

        self.timer = self.create_timer(1.0 / self.est_hz, self.on_tick)
        self.status_timer = self.create_timer(1.0, self.on_status_tick)

    def emit_event(self, value: str) -> None:
        if not self.publish_events:
            return
        msg = String()
        msg.data = value
        self.events_pub.publish(msg)

    def publish_status_msg(self, value: str) -> None:
        if not self.publish_status:
            return
        msg = String()
        msg.data = value
        self.status_pub.publish(msg)

    def publish_fault_status_msg(self, value: str) -> None:
        if not self.publish_fault_status or self.fault_status_pub is None:
            return
        msg = String()
        msg.data = value
        self.fault_status_pub.publish(msg)

    def _fault_age_ms(self, now: Time) -> float:
        if self.last_fault_stamp is None:
            return -1.0
        return max(0.0, (now - self.last_fault_stamp).nanoseconds * 1e-6)

    def _fault_line(self, state: str, reason: str, now: Time) -> str:
        return (
            f"state={state} reason={reason} fault_age_ms={self._fault_age_ms(now):.1f} "
            f"yolo={'enabled' if self.tracker.ready else 'disabled'} tracker={'enabled' if self.tracker.tracker_enable else 'disabled'} "
            f"yolo_reason={self.tracker.error or 'ok'} track_id={self._track_id_text(now)} "
            f"track_state={self._track_state_text()} conf={self.last_det_conf:.3f} "
            f"latency_ms={self.last_latency_ms:.1f} reject_reason={self.last_reject_reason}"
        )

    def _record_fault(self, state: str, reason: str, now: Time) -> None:
        self.last_fault_state = str(state).strip() or "unknown"
        self.last_fault_reason = str(reason).strip() or "none"
        self.last_fault_stamp = now
        self.publish_fault_status_msg(self._fault_line(self.last_fault_state, self.last_fault_reason, now))

    def on_image(self, msg: Image) -> None:
        self.last_image_msg = msg
        self.last_image_recv_walltime = time.monotonic()
        try:
            self.last_image_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_image_stamp = self.get_clock().now()

    def on_depth(self, msg: Image) -> None:
        self.last_depth_msg = msg

    def on_camera_info(self, msg: CameraInfo) -> None:
        camera_model = self.projector.camera_model_from_info(msg)
        if camera_model is not None:
            self.camera_model = camera_model

    def on_uav_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.uav_pose = PoseEstimate(float(p.x), float(p.y), float(p.z), yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)))
        self.have_uav_pose = True
        self.have_real_uav_pose = True
        self.uav_pose_source = "pose"
        try:
            self.last_uav_pose_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_uav_pose_stamp = self.get_clock().now()

    def on_actual_leader_pose(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.last_actual_leader_pose = PoseEstimate(float(p.x), float(p.y), float(p.z), yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)))
        try:
            self.last_actual_leader_pose_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_actual_leader_pose_stamp = self.get_clock().now()

    def _is_fresh(self, last_stamp: Optional[Time], timeout_s: float, now: Time) -> bool:
        if last_stamp is None:
            return False
        return (now - last_stamp).nanoseconds * 1e-9 <= timeout_s

    def image_fresh(self, now: Time) -> bool:
        return self._is_fresh(self.last_image_stamp, self.image_timeout_s, now)

    def uav_pose_fresh(self, now: Time) -> bool:
        if self.have_real_uav_pose:
            return self.have_uav_pose and self._is_fresh(self.last_uav_pose_stamp, self.uav_pose_timeout_s, now)
        if self.bootstrap_uav_pose_enabled and self.have_uav_pose:
            self.last_uav_pose_stamp = now
            self.uav_pose_source = "bootstrap"
            return True
        return False

    def actual_leader_pose_fresh(self, now: Time) -> bool:
        if not self.leader_actual_pose_enable:
            return False
        return self._is_fresh(self.last_actual_leader_pose_stamp, self.leader_actual_pose_timeout_s, now)

    def _image_to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        try:
            h = int(msg.height)
            w = int(msg.width)
            if h <= 0 or w <= 0:
                return None
            if msg.encoding == "bgr8":
                return np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
            if msg.encoding == "rgb8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
                if cv2 is not None:
                    return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                return arr[:, :, ::-1].copy()
            if msg.encoding == "bgra8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 4))
                return arr[:, :, :3]
            if msg.encoding == "rgba8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 4))
                rgb = arr[:, :, :3]
                if cv2 is not None:
                    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                return rgb[:, :, ::-1].copy()
            if msg.encoding == "mono8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w))
                if cv2 is not None:
                    return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
                return np.stack([arr, arr, arr], axis=-1)
        except Exception as exc:
            self.get_logger().warn(f"[leader_estimator] Failed image decode: {exc}")
        return None

    def _bgr_to_image_msg(self, img_bgr: np.ndarray) -> Optional[Image]:
        try:
            h, w = img_bgr.shape[:2]
            msg = Image()
            if self.last_image_msg is not None:
                msg.header.stamp = self.last_image_msg.header.stamp
                msg.header.frame_id = self.last_image_msg.header.frame_id
            else:
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = ""
            msg.height = int(h)
            msg.width = int(w)
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = int(w * 3)
            msg.data = np.ascontiguousarray(img_bgr).tobytes()
            return msg
        except Exception:
            return None

    def _debug_color(self, state: str) -> tuple[int, int, int]:
        if state == "OK":
            return (0, 220, 0)
        if state == "REACQUIRE":
            return (0, 215, 255)
        if state in ("NO_DET", "YOLO_DISABLED", "STALE", "DECODE_FAIL"):
            return (0, 165, 255)
        if state == "REJECT":
            return (0, 0, 255)
        return (255, 200, 0)

    def _track_id_text(self, now: Time) -> str:
        return self.tracker.track_summary(int(now.nanoseconds))[0]

    def _track_state_text(self) -> str:
        if self.last_selected_detection is None:
            return "none"
        return self.last_selected_detection.track_state

    def _maybe_resize_debug_image(self, img_bgr: np.ndarray) -> np.ndarray:
        if cv2 is None or self.debug_image_max_width <= 0:
            return img_bgr
        h, w = img_bgr.shape[:2]
        if w <= self.debug_image_max_width:
            return img_bgr
        scale = float(self.debug_image_max_width) / float(w)
        out_h = max(1, int(round(h * scale)))
        return cv2.resize(img_bgr, (self.debug_image_max_width, out_h), interpolation=cv2.INTER_AREA)

    def _publish_debug_image(self, img_bgr: Optional[np.ndarray], state: str, det: Optional[Detection2D]) -> None:
        self.last_debug_state = state
        if not self.publish_debug_image or self.debug_image_pub is None or img_bgr is None:
            return
        if cv2 is None:
            msg = self._bgr_to_image_msg(img_bgr)
            if msg is not None:
                self.debug_image_pub.publish(msg)
            return

        vis = img_bgr.copy()
        color = self._debug_color(state)
        if det is not None:
            x1, y1, x2, y2 = [int(round(v)) for v in det.bbox]
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            cls_txt = det.cls_name or (str(det.cls_id) if det.cls_id is not None else "na")
            track_txt = f" id={det.track_id}" if det.track_id is not None else ""
            cv2.putText(vis, f"{cls_txt} conf={det.conf:.2f}{track_txt}", (x1, max(24, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)

        lines = [
            f"state={state}",
            f"track_id={self._track_id_text(self.get_clock().now())} hits={self.tracker.active_track_hits} reject={self.last_reject_reason}",
            f"conf={self.last_det_conf:.2f} latency_ms={self.last_latency_ms:.1f}",
        ]
        if self.last_estimate_pose is not None:
            lines.append(f"est=({self.last_estimate_pose.x:.2f},{self.last_estimate_pose.y:.2f},{math.degrees(self.last_estimate_pose.yaw):.1f}deg)")
        y = 26
        for line in lines:
            cv2.putText(vis, line, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
            y += 24
        msg = self._bgr_to_image_msg(self._maybe_resize_debug_image(vis))
        if msg is not None:
            self.debug_image_pub.publish(msg)

    def _publish_estimate(self, pose: PoseEstimate, now: Time, bearing_rad: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(pose.x)
        msg.pose.position.y = float(pose.y)
        msg.pose.position.z = float(pose.z)
        qx, qy, qz, qw = quat_from_yaw(pose.yaw)
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)
        self.estimate_pub.publish(msg)
        self.last_estimate_pose = pose
        self.projector.commit(pose, bearing_rad, int(now.nanoseconds))
        self._publish_estimate_error(pose, now)

    def _publish_estimate_error(self, pose: PoseEstimate, now: Time) -> None:
        if not self.leader_actual_pose_enable or self.estimate_error_pub is None:
            self.last_estimate_error_dx_m = None
            self.last_estimate_error_dy_m = None
            self.last_estimate_error_m = None
            return
        if not self.actual_leader_pose_fresh(now) or self.last_actual_leader_pose is None:
            self.last_estimate_error_dx_m = None
            self.last_estimate_error_dy_m = None
            self.last_estimate_error_m = None
            return
        dx = pose.x - self.last_actual_leader_pose.x
        dy = pose.y - self.last_actual_leader_pose.y
        planar = math.hypot(dx, dy)
        self.last_estimate_error_dx_m = dx
        self.last_estimate_error_dy_m = dy
        self.last_estimate_error_m = planar
        msg = Vector3Stamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"
        msg.vector.x = float(dx)
        msg.vector.y = float(dy)
        msg.vector.z = float(planar)
        self.estimate_error_pub.publish(msg)

    def _status_line(self, state: str, now: Time, *, reason: str = "none", det: Optional[Detection2D] = None) -> str:
        image_age_s = float("inf")
        pose_age_s = float("inf")
        actual_pose_age_s = float("inf")
        if self.last_image_stamp is not None:
            image_age_s = (now - self.last_image_stamp).nanoseconds * 1e-9
        if self.last_uav_pose_stamp is not None:
            pose_age_s = (now - self.last_uav_pose_stamp).nanoseconds * 1e-9
        if self.last_actual_leader_pose_stamp is not None:
            actual_pose_age_s = (now - self.last_actual_leader_pose_stamp).nanoseconds * 1e-9
        img_wall_age_ms = -1.0
        if self.last_image_recv_walltime is not None:
            img_wall_age_ms = (time.monotonic() - self.last_image_recv_walltime) * 1000.0

        track_id_txt, track_hits_txt, track_age_txt = self.tracker.track_summary(int(now.nanoseconds))
        cls_txt = "na"
        bbox_u = "na"
        bbox_v = "na"
        track_state = self._track_state_text()
        if det is not None:
            cls_txt = det.cls_name or (str(det.cls_id) if det.cls_id is not None else "na")
            bbox_u = f"{det.u:.1f}"
            bbox_v = f"{det.v:.1f}"
            track_state = det.track_state
        err_dx_txt = "na" if self.last_estimate_error_dx_m is None else f"{self.last_estimate_error_dx_m:.2f}"
        err_dy_txt = "na" if self.last_estimate_error_dy_m is None else f"{self.last_estimate_error_dy_m:.2f}"
        err_norm_txt = "na" if self.last_estimate_error_m is None else f"{self.last_estimate_error_m:.2f}"
        return (
            f"state={state} state_reason={reason} "
            f"yolo={'enabled' if self.tracker.ready else 'disabled'} tracker={'enabled' if self.tracker.tracker_enable else 'disabled'} "
            f"yolo_reason={self.tracker.error or 'ok'} device={self.device} "
            f"track_id={track_id_txt} track_hits={track_hits_txt} track_age_s={track_age_txt} track_state={track_state} "
            f"class={cls_txt} bbox_u={bbox_u} bbox_v={bbox_v} conf={self.last_det_conf:.3f} latency_ms={self.last_latency_ms:.1f} "
            f"range_src={self.last_range_source} range_mode={self.range_mode} range_m={self.projector.last_range_used_m:.2f} "
            f"bearing_deg={self.projector.last_bearing_used_deg:.1f} reject_reason={self.last_reject_reason} "
            f"last_img_age_ms={(image_age_s * 1000.0) if math.isfinite(image_age_s) else 'inf'} last_img_recv_wall_age_ms={img_wall_age_ms:.1f} "
            f"img_age_s={image_age_s if math.isfinite(image_age_s) else 'inf'} uav_pose_age_s={pose_age_s if math.isfinite(pose_age_s) else 'inf'} "
            f"uav_pose_src={self.uav_pose_source} actual_pose_age_s={actual_pose_age_s if math.isfinite(actual_pose_age_s) else 'inf'} "
            f"err_dx_m={err_dx_txt} err_dy_m={err_dy_txt} err_planar_m={err_norm_txt}"
        )

    def _current_state(self, now: Time) -> str:
        if self.last_image_msg is None:
            return "waiting_for_image"
        if self.camera_model is None:
            return "waiting_for_camera_info"
        if not self.uav_pose_fresh(now):
            return "waiting_for_uav_pose"
        if not self.image_fresh(now):
            return "stale_image"
        if not self.tracker.ready:
            return "YOLO_DISABLED"
        return self.last_status or "running"

    def _publish_state_only(self, state: str, now: Time, img_bgr: Optional[np.ndarray], reason: str, det: Optional[Detection2D] = None) -> None:
        self._record_fault(state, reason, now)
        self._publish_debug_image(img_bgr, state, det)
        self.publish_status_msg(self._status_line(state, now, reason=reason, det=det))
        if self.last_status != state:
            self.emit_event("ESTIMATE_OK" if state == "OK" else "ESTIMATE_STALE")
            self.last_status = state

    def on_status_tick(self) -> None:
        now = self.get_clock().now()
        self.publish_status_msg(self._status_line(self._current_state(now), now, det=self.last_selected_detection))

    def on_tick(self) -> None:
        now = self.get_clock().now()
        if self.last_image_msg is None:
            self._publish_state_only("waiting_for_image", now, None, "waiting_for_image")
            return
        if self.camera_model is None:
            self._publish_state_only("waiting_for_camera_info", now, None, "waiting_for_camera_info")
            return
        if not self.uav_pose_fresh(now):
            self._publish_state_only("waiting_for_uav_pose", now, None, "waiting_for_uav_pose")
            return
        if not self.image_fresh(now):
            self.last_det_conf = -1.0
            self.last_latency_ms = -1.0
            self.last_range_source = "none"
            self.last_reject_reason = "none"
            self._publish_state_only("STALE", now, None, "image_stale")
            return

        img_bgr = self._image_to_bgr(self.last_image_msg)
        if img_bgr is None:
            self.last_det_conf = -1.0
            self.last_latency_ms = -1.0
            self.last_range_source = "none"
            self.last_reject_reason = "image_decode"
            self._publish_state_only("DECODE_FAIL", now, None, "image_decode")
            return

        if not self.tracker.ready:
            self.last_det_conf = -1.0
            self.last_latency_ms = -1.0
            self.last_range_source = "none"
            self.last_reject_reason = self.tracker.error or "yolo_disabled"
            self._publish_state_only("YOLO_DISABLED", now, img_bgr, self.tracker.error or "yolo_disabled")
            return

        detections = self.tracker.infer(img_bgr)
        det = self.tracker.select(detections, stamp_ns=int(now.nanoseconds), ok_debounce_frames=self.ok_debounce_frames)
        self.last_selected_detection = det

        if det is None:
            self.good_det_streak = 0
            self.bad_det_streak += 1
            self.last_det_conf = -1.0
            image_stamp = self.last_image_stamp if self.last_image_stamp is not None else now
            self.last_latency_ms = max(0.0, (now - image_stamp).nanoseconds * 1e-6)
            self.last_range_source = "none"
            self.last_reject_reason = "no_detection"
            if self.bad_det_streak >= self.bad_debounce_frames:
                self.tracker.reset_active_track()
                self.projector.note_track_reset()
                self._publish_state_only("NO_DET", now, img_bgr, "no_detection")
            else:
                self._publish_state_only("REACQUIRE", now, img_bgr, "searching_for_track")
            return

        if det.track_switched:
            self.projector.note_track_reset()

        projected = self.projector.estimate(det, self.camera_model, self.uav_pose, self.last_depth_msg, int(now.nanoseconds))
        sane, reject_reason = self.projector.is_sane(projected.pose, projected.bearing_rad, int(now.nanoseconds))

        image_stamp = self.last_image_stamp if self.last_image_stamp is not None else now
        self.last_det_conf = float(det.conf)
        self.last_latency_ms = max(0.0, (now - image_stamp).nanoseconds * 1e-6)
        self.last_range_source = projected.range_source

        if not sane:
            self.good_det_streak = 0
            self.bad_det_streak += 1
            self.last_reject_reason = reject_reason
            det.track_state = "reacquire"
            if self.bad_det_streak >= self.bad_debounce_frames:
                self._publish_state_only("REJECT", now, img_bgr, reject_reason, det)
            else:
                self._publish_state_only("REACQUIRE", now, img_bgr, reject_reason, det)
            return

        self.bad_det_streak = 0
        self.good_det_streak += 1
        self.last_reject_reason = "none"
        state = "OK" if self.good_det_streak >= self.ok_debounce_frames else "REACQUIRE"
        if state == "REACQUIRE" and det.track_id is not None:
            det.track_state = "reacquire"
        self._publish_estimate(projected.pose, now, projected.bearing_rad)
        self._record_fault(state, "none", now)
        self._publish_debug_image(img_bgr, state, det)
        self.publish_status_msg(self._status_line(state, now, det=det))
        if self.last_status != state:
            self.emit_event("ESTIMATE_OK" if state == "OK" else "ESTIMATE_STALE")
            self.last_status = state


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeaderEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.emit_event("ESTIMATOR_NODE_SHUTDOWN")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
