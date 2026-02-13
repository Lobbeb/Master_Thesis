#!/usr/bin/env python3
import random
import time
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class EventImpairmentBridge(Node):
    def __init__(self):
        super().__init__("ros_omnet_bridge")

        self.declare_parameter("enabled", True)
        self.declare_parameter("input_topic", "/coord/events_raw")
        self.declare_parameter("output_topic", "/coord/events")
        self.declare_parameter("delay_ms", 0.0)
        self.declare_parameter("delay_stddev_ms", 0.0)
        self.declare_parameter("packet_loss_percent", 0.0)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.delay_ms = float(self.get_parameter("delay_ms").value)
        self.delay_stddev_ms = float(self.get_parameter("delay_stddev_ms").value)
        self.loss_pct = float(self.get_parameter("packet_loss_percent").value)

        self.pub = self.create_publisher(String, self.output_topic, 10)
        self.sub = self.create_subscription(String, self.input_topic, self._cb, 10)

        self.queue = deque()
        self.running = True
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

        self.get_logger().info(
            f"Bridge up: enabled={self.enabled} "
            f"{self.input_topic} -> {self.output_topic} "
            f"delay={self.delay_ms}±{self.delay_stddev_ms}ms loss={self.loss_pct}%"
        )

    def _drop(self) -> bool:
        if not self.enabled or self.loss_pct <= 0.0:
            return False
        return random.random() * 100.0 < self.loss_pct

    def _delay_s(self) -> float:
        if not self.enabled or self.delay_ms <= 0.0:
            return 0.0
        d = random.gauss(self.delay_ms, self.delay_stddev_ms)
        return max(0.0, d / 1000.0)

    def _cb(self, msg: String):
        if self._drop():
            return
        t_pub = time.time() + self._delay_s()
        self.queue.append((t_pub, msg))

    def _worker_loop(self):
        while self.running:
            now = time.time()
            while self.queue and self.queue[0][0] <= now:
                _, msg = self.queue.popleft()
                self.pub.publish(msg)
            time.sleep(0.001)

    def destroy_node(self):
        self.running = False
        try:
            self.worker.join(timeout=1.0)
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = EventImpairmentBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
