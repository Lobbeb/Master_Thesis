#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from tf_transformations import quaternion_from_euler, euler_from_quaternion
from ros_gz_interfaces.srv import SetEntityPose


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def clamp_point_to_radius(center_x: float, center_y: float, x: float, y: float, r: float) -> Tuple[float, float]:
    """
    If (x,y) is farther than r from (center_x, center_y), project it onto the circle of radius r.
    """
    dx = x - center_x
    dy = y - center_y
    d = math.hypot(dx, dy)
    if d <= r or d < 1e-9:
        return x, y
    s = r / d
    return center_x + dx * s, center_y + dy * s


class FollowUav(Node):
    """
    Stage 2 (B1): Follow + Leash controller (robust).
    - Reads UGV odom (default: /a201_0000/platform/odom/filtered)
    - Computes target behind UGV at d_target
    - Enforces leash by clamping target within d_max around UGV (geometrically correct)
    - Commands UAV using /world/<world>/set_pose (Gazebo SetEntityPose)
    - Publishes commanded pose for bagging (/dji0/pose_cmd)
    - Publishes events to EVENT_TOPIC (default /coord/events)
    - Safety: pose freshness timeout -> HOLD (no new commands) + POSE_STALE event
    - Reliability: tracks service future to avoid request backlog
    - Optional smoothing (alpha=1.0 means off)
    """

    def __init__(self):
        super().__init__("follow_uav")

        # ---------- Parameters ----------
        self.declare_parameter("world", "orchard")
        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("ugv_odom_topic", "/a201_0000/platform/odom/filtered")

        self.declare_parameter("tick_hz", 5.0)
        self.declare_parameter("d_target", 5.0)
        self.declare_parameter("d_max", 15.0)
        self.declare_parameter("z_alt", 10.0)

        self.declare_parameter("follow_yaw", True)

        # Freshness and service pacing
        self.declare_parameter("pose_timeout_s", 0.75)   # stale if odom older than this
        self.declare_parameter("min_cmd_period_s", 0.10) # don't command faster than this even if tick is high

        # Smoothing (alpha=1.0 -> no smoothing; 0.0 -> freeze)
        self.declare_parameter("smooth_alpha", 1.0)

        # Events
        self.declare_parameter("event_topic", "/coord/events")
        self.declare_parameter("publish_events", True)

        # ---------- Read params ----------
        self.world = str(self.get_parameter("world").value)
        self.uav_name = str(self.get_parameter("uav_name").value)
        self.ugv_odom_topic = str(self.get_parameter("ugv_odom_topic").value)

        self.tick_hz = float(self.get_parameter("tick_hz").value)
        self.d_target = float(self.get_parameter("d_target").value)
        self.d_max = float(self.get_parameter("d_max").value)
        self.z_alt = float(self.get_parameter("z_alt").value)

        self.follow_yaw = bool(self.get_parameter("follow_yaw").value)

        self.pose_timeout_s = float(self.get_parameter("pose_timeout_s").value)
        self.min_cmd_period_s = float(self.get_parameter("min_cmd_period_s").value)
        self.smooth_alpha = float(self.get_parameter("smooth_alpha").value)

        self.event_topic = str(self.get_parameter("event_topic").value)
        self.publish_events = bool(self.get_parameter("publish_events").value)

        if self.tick_hz <= 0.0:
            raise ValueError("tick_hz must be > 0")
        if self.d_max <= 0.0 or self.d_target <= 0.0:
            raise ValueError("d_max and d_target must be > 0")
        if self.d_max <= self.d_target:
            self.get_logger().warn(
                f"d_max ({self.d_max}) <= d_target ({self.d_target}). "
                f"Leash will trigger/clamp often. Recommend d_max > d_target."
            )
        if not (0.0 <= self.smooth_alpha <= 1.0):
            raise ValueError("smooth_alpha must be in [0,1]")

        # ---------- State ----------
        self.have_ugv = False
        self.ugv_pose = Pose2D(0.0, 0.0, 0.0)
        self.last_ugv_stamp: Optional[Time] = None

        # Commanded UAV pose (deterministic internal state)
        self.uav_cmd = Pose2D(0.0, 0.0, 0.0)
        self.have_uav_cmd = False

        self.last_cmd_time: Optional[Time] = None
        self.pending_future = None

        # ---------- ROS I/O ----------
        self.odom_sub = self.create_subscription(
            Odometry,
            self.ugv_odom_topic,
            self.on_ugv_odom,
            10,
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            f"/{self.uav_name}/pose_cmd",
            10,
        )

        self.events_pub = self.create_publisher(String, self.event_topic, 10)

        srv_name = f"/world/{self.world}/set_pose"
        self.cli = self.create_client(SetEntityPose, srv_name)
        self.get_logger().info(f"[follow_uav] Waiting for service: {srv_name}")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("[follow_uav] Service not available, waiting again...")

        dt = 1.0 / self.tick_hz
        self.timer = self.create_timer(dt, self.on_tick)

        self.get_logger().info(
            f"[follow_uav] Started: world={self.world}, uav={self.uav_name}, "
            f"odom={self.ugv_odom_topic}, tick={self.tick_hz}Hz, "
            f"d_target={self.d_target}, d_max={self.d_max}, z={self.z_alt}, "
            f"pose_timeout_s={self.pose_timeout_s}, min_cmd_period_s={self.min_cmd_period_s}, "
            f"smooth_alpha={self.smooth_alpha}, event_topic={self.event_topic}"
        )
        self.emit_event("FOLLOW_NODE_START")

    def emit_event(self, s: str):
        if not self.publish_events:
            return
        msg = String()
        msg.data = s
        self.events_pub.publish(msg)

    def on_ugv_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.ugv_pose = Pose2D(float(p.x), float(p.y), float(yaw))
        self.have_ugv = True

        # Prefer message stamp if present; fall back to node time
        try:
            self.last_ugv_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_ugv_stamp = self.get_clock().now()

    def ugv_pose_is_fresh(self, now: Time) -> bool:
        if not self.have_ugv or self.last_ugv_stamp is None:
            return False
        age = (now - self.last_ugv_stamp).nanoseconds * 1e-9
        return age <= self.pose_timeout_s

    def can_send_command_now(self, now: Time) -> bool:
        if self.last_cmd_time is None:
            return True
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9
        return dt >= self.min_cmd_period_s

    def set_entity_pose_async(self, entity_name: str, x: float, y: float, z: float, yaw_rad: float):
        req = SetEntityPose.Request()
        req.entity.id = 0
        req.entity.name = entity_name
        req.entity.type = req.entity.MODEL

        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)

        quat = quaternion_from_euler(0.0, 0.0, float(yaw_rad))
        req.pose.orientation.x = float(quat[0])
        req.pose.orientation.y = float(quat[1])
        req.pose.orientation.z = float(quat[2])
        req.pose.orientation.w = float(quat[3])

        fut = self.cli.call_async(req)
        self.pending_future = fut

        def _done_cb(f):
            try:
                _ = f.result()  # may raise
            except Exception as e:
                self.get_logger().warn(f"[follow_uav] set_pose failed: {e}")

        fut.add_done_callback(_done_cb)

    def publish_pose_cmd(self, x: float, y: float, z: float, yaw_rad: float):
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "map"
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)

        quat = quaternion_from_euler(0.0, 0.0, float(yaw_rad))
        ps.pose.orientation.x = float(quat[0])
        ps.pose.orientation.y = float(quat[1])
        ps.pose.orientation.z = float(quat[2])
        ps.pose.orientation.w = float(quat[3])

        self.pose_pub.publish(ps)

    def on_tick(self):
        now = self.get_clock().now()

        # Need fresh UGV pose
        if not self.ugv_pose_is_fresh(now):
            # only emit once per stale period to avoid spam
            self.emit_event("POSE_STALE_HOLD")
            return

        # Avoid backlog: if previous request still pending, skip this tick
        if self.pending_future is not None and not self.pending_future.done():
            return

        # Rate-limit commands independently of tick rate
        if not self.can_send_command_now(now):
            return

        ugv = self.ugv_pose

        # Desired target behind UGV
        xt = ugv.x - self.d_target * math.cos(ugv.yaw)
        yt = ugv.y - self.d_target * math.sin(ugv.yaw)

        # Leash clamp: ensure target is within d_max of UGV
        xt2, yt2 = clamp_point_to_radius(ugv.x, ugv.y, xt, yt, self.d_max)
        if (xt2, yt2) != (xt, yt):
            self.emit_event("LEASH_CLAMPED")
        xt, yt = xt2, yt2

        # Optional smoothing on commanded XY (helps jitter; default alpha=1.0 => no smoothing)
        if self.have_uav_cmd and self.smooth_alpha < 1.0:
            a = self.smooth_alpha
            xt = a * xt + (1.0 - a) * self.uav_cmd.x
            yt = a * yt + (1.0 - a) * self.uav_cmd.y

        yaw_cmd = ugv.yaw if self.follow_yaw else (self.uav_cmd.yaw if self.have_uav_cmd else ugv.yaw)

        # Command UAV model only (camera moves with model)
        self.set_entity_pose_async(self.uav_name, xt, yt, self.z_alt, yaw_cmd)

        # Publish commanded pose for rosbag/metrics
        self.publish_pose_cmd(xt, yt, self.z_alt, yaw_cmd)

        # Update internal command state
        self.uav_cmd = Pose2D(xt, yt, yaw_cmd)
        self.have_uav_cmd = True
        self.last_cmd_time = now

        self.emit_event("FOLLOW_TICK")


def main(args=None):
    rclpy.init(args=args)
    node = FollowUav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.emit_event("FOLLOW_NODE_SHUTDOWN")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
