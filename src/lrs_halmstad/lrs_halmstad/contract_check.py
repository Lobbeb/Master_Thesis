from __future__ import annotations

import os
import sys
import time
import rclpy
from rclpy.node import Node


REQUIRED_SERVICES = [
    # world is formatted in main()
    "/world/{world}/set_pose",
]

REQUIRED_TOPICS = [
    "/clock",
    "/a201_0000/platform/odom",
    "/a201_0000/platform/odom/filtered",
    "/a201_0000/platform/cmd_vel",
    "/a201_0000/tf",
    "/a201_0000/tf_static",
    # event topic is added dynamically in check()
]



#"/{uav}/pose",
class ContractChecker(Node):
    def __init__(self):
        super().__init__("contract_checker")

    def _topic_exists(self, name: str) -> bool:
        topics = self.get_topic_names_and_types()
        return any(t[0] == name for t in topics)

    def _service_exists(self, name: str) -> bool:
        services = self.get_service_names_and_types()
        return any(s[0] == name for s in services)

    def check(self, world: str, uav: str, timeout_s: float = 10.0) -> int:
        deadline = time.time() + timeout_s
        missing_topics = set()
        missing_services = set()

        event_topic = os.environ.get("EVENT_TOPIC", "/coord/events").strip() or "/coord/events"

        required_topics = [t.format(uav=uav) for t in REQUIRED_TOPICS]
        #required_topics.append(event_topic)

        # If harness publishes to raw, we also expect the impaired output topic to exist
        # Do NOT require the event topic to exist yet.
        # It will be created by the harness when we publish the first marker.
        # (In impaired mode, we still want the bridge output to exist if we're using raw.)

        if event_topic != "/coord/events":
            required_topics.append("/coord/events")

        required_services = [s.format(world=world) for s in REQUIRED_SERVICES]

        while time.time() < deadline:
            missing_topics = {t for t in required_topics if not self._topic_exists(t)}
            missing_services = {s for s in required_services if not self._service_exists(s)}
            if not missing_topics and not missing_services:
                self.get_logger().info("✅ Contract OK: required topics/services are available")
                return 0
            time.sleep(0.25)

        if missing_topics:
            self.get_logger().error("Missing topics:\n  " + "\n  ".join(sorted(missing_topics)))
        if missing_services:
            self.get_logger().error("Missing services:\n  " + "\n  ".join(sorted(missing_services)))

        return 2


def main(argv=None) -> None:
    argv = argv or sys.argv
    if len(argv) < 3:
        print("Usage: ros2 run lrs_halmstad contract_check <world> <uav_name> [timeout_s]")
        raise SystemExit(2)

    world = argv[1]
    uav = argv[2]
    timeout_s = float(argv[3]) if len(argv) >= 4 else 10.0

    rclpy.init()
    node = ContractChecker()
    rc = node.check(world, uav, timeout_s)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(rc)
