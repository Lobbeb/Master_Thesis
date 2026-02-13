#!/usr/bin/env python3
"""
OMNeT++ Bridge Node - Simplified Network Impairment Simulator

This node subscribes to ROS topics and republishes them with configurable
network impairments (delay, packet loss, bandwidth limitation) to simulate
realistic wireless communication between UAV and UGV.

For the thesis, this provides a lightweight alternative to full OMNeT++ integration,
allowing rapid experimentation with network parameters without external simulation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import random
import time
from collections import deque
import threading


class NetworkImpairmentNode(Node):
    """
    Simulates network impairments for UAV-UGV communication.
    
    Configuration parameters (from ROS parameters):
    - delay_ms: Mean one-way delay in milliseconds (default: 0)
    - delay_stddev_ms: Standard deviation of delay jitter (default: 0)
    - packet_loss_percent: Packet loss percentage 0-100 (default: 0)
    - bandwidth_kbps: Bandwidth limit in kbps (default: inf)
    - enabled: Enable/disable network impairment (default: true)
    """
    
    def __init__(self):
        super().__init__('network_impairment_node')
        
        # Declare parameters
        self.declare_parameter('delay_ms', 0.0)
        self.declare_parameter('delay_stddev_ms', 0.0)
        self.declare_parameter('packet_loss_percent', 0.0)
        self.declare_parameter('bandwidth_kbps', float('inf'))
        self.declare_parameter('enabled', True)
        
        # Get parameters
        self.delay_ms = self.get_parameter('delay_ms').value
        self.delay_stddev_ms = self.get_parameter('delay_stddev_ms').value
        self.packet_loss_percent = self.get_parameter('packet_loss_percent').value
        self.bandwidth_kbps = self.get_parameter('bandwidth_kbps').value
        self.enabled = self.get_parameter('enabled').value
        
        # Subscribe to original topics
        self.uav_pose_sub = self.create_subscription(
            PoseStamped,
            '/uav/pose_gt',
            self.uav_pose_callback,
            10
        )
        
        self.uav_cmd_sub = self.create_subscription(
            String,
            '/uav/command',
            self.uav_cmd_callback,
            10
        )
        
        self.ugv_pose_sub = self.create_subscription(
            PoseStamped,
            '/ugv/pose',
            self.ugv_pose_callback,
            10
        )
        
        # Publishers for impaired topics
        self.uav_pose_pub = self.create_publisher(PoseStamped, '/uav/pose_gt/impaired', 10)
        self.uav_cmd_pub = self.create_publisher(String, '/uav/command/impaired', 10)
        self.ugv_pose_pub = self.create_publisher(PoseStamped, '/ugv/pose/impaired', 10)
        
        # Delayed message queues (for simulating network delay)
        self.uav_pose_queue = deque()
        self.uav_cmd_queue = deque()
        self.ugv_pose_queue = deque()
        
        # Start delay processing thread
        self.running = True
        self.delay_thread = threading.Thread(target=self._process_delayed_messages, daemon=True)
        self.delay_thread.start()
        
        self.get_logger().info(
            f'Network Impairment Node started: '
            f'enabled={self.enabled}, '
            f'delay={self.delay_ms}±{self.delay_stddev_ms}ms, '
            f'loss={self.packet_loss_percent}%, '
            f'bandwidth={self.bandwidth_kbps}kbps'
        )
    
    def _apply_packet_loss(self):
        """Check if packet should be dropped based on loss probability."""
        if not self.enabled or self.packet_loss_percent == 0.0:
            return False
        return random.random() * 100.0 < self.packet_loss_percent
    
    def _calculate_delay(self):
        """Calculate delay with jitter (Gaussian distribution)."""
        if not self.enabled or self.delay_ms == 0.0:
            return 0.0
        
        delay = random.gauss(self.delay_ms, self.delay_stddev_ms)
        return max(0.0, delay / 1000.0)  # Convert to seconds, ensure non-negative
    
    def uav_pose_callback(self, msg):
        """Handle incoming UAV pose messages."""
        if self._apply_packet_loss():
            self.get_logger().debug('Dropped UAV pose packet (simulated loss)')
            return
        
        delay = self._calculate_delay()
        publish_time = time.time() + delay
        self.uav_pose_queue.append((publish_time, msg))
    
    def uav_cmd_callback(self, msg):
        """Handle incoming UAV command messages."""
        if self._apply_packet_loss():
            self.get_logger().debug('Dropped UAV command packet (simulated loss)')
            return
        
        delay = self._calculate_delay()
        publish_time = time.time() + delay
        self.uav_cmd_queue.append((publish_time, msg))
    
    def ugv_pose_callback(self, msg):
        """Handle incoming UGV pose messages."""
        if self._apply_packet_loss():
            self.get_logger().debug('Dropped UGV pose packet (simulated loss)')
            return
        
        delay = self._calculate_delay()
        publish_time = time.time() + delay
        self.ugv_pose_queue.append((publish_time, msg))
    
    def _process_delayed_messages(self):
        """Background thread to publish delayed messages at appropriate times."""
        while self.running:
            current_time = time.time()
            
            # Process UAV pose queue
            while self.uav_pose_queue and self.uav_pose_queue[0][0] <= current_time:
                _, msg = self.uav_pose_queue.popleft()
                self.uav_pose_pub.publish(msg)
            
            # Process UAV command queue
            while self.uav_cmd_queue and self.uav_cmd_queue[0][0] <= current_time:
                _, msg = self.uav_cmd_queue.popleft()
                self.uav_cmd_pub.publish(msg)
            
            # Process UGV pose queue
            while self.ugv_pose_queue and self.ugv_pose_queue[0][0] <= current_time:
                _, msg = self.ugv_pose_queue.popleft()
                self.ugv_pose_pub.publish(msg)
            
            time.sleep(0.001)  # 1ms sleep to avoid busy-waiting
    
    def destroy_node(self):
        """Clean shutdown."""
        self.running = False
        if self.delay_thread.is_alive():
            self.delay_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NetworkImpairmentNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
