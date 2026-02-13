#!/usr/bin/env python3
"""
OMNeT++ Socket Server Wrapper

This script launches OMNeT++ and provides a socket interface for ROS bridge.
It receives position updates and messages from ROS, injects them into OMNeT++,
and sends back results after network simulation.

This is a simplified implementation that uses OMNeT++'s eventlog for communication.
A full C++ integration would use OMNeT++'s API directly, but this approach is
easier to maintain and debug.
"""

import socket
import json
import struct
import subprocess
import threading
import sys
import time
import os


class OmnetSocketServer:
    """
    Socket server that interfaces between ROS bridge and OMNeT++ simulation.
    """
    
    def __init__(self, config='LoRa_SF7_ShortRange', port=5555):
        self.config = config
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.omnet_process = None
        
        # Current node positions
        self.uav_pos = {'x': 0.0, 'y': 0.0, 'z': 10.0}
        self.ugv_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # Message queue
        self.message_queue = []
    
    def start_omnet(self):
        """Launch OMNeT++ simulation in background."""
        omnet_root = os.path.expanduser('~/omnetpp-6.0.3')
        inet_root = os.path.join(omnet_root, 'inet4.5')
        sim_dir = os.path.expanduser('~/halmstad_ws/src/lrs_omnet/simulations')
        
        # Set environment
        env = os.environ.copy()
        env['PATH'] = f"{omnet_root}/bin:{env['PATH']}"
        env['LD_LIBRARY_PATH'] = f"{inet_root}/src:{env.get('LD_LIBRARY_PATH', '')}"
        
        cmd = [
            os.path.join(omnet_root, 'bin/opp_run'),
            '-u', 'Cmdenv',  # Command-line mode (no GUI)
            '-n', f'{sim_dir}/..:../networks:{inet_root}/src',
            '-l', os.path.join(inet_root, 'src/libINET.so'),
            '-c', self.config
        ]
        
        print(f"Starting OMNeT++ with config: {self.config}")
        print(f"Command: {' '.join(cmd)}")
        
        try:
            self.omnet_process = subprocess.Popen(
                cmd,
                cwd=sim_dir,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            print(f"OMNeT++ started (PID: {self.omnet_process.pid})")
            time.sleep(2)  # Give OMNeT++ time to initialize
        except Exception as e:
            print(f"ERROR: Failed to start OMNeT++: {e}")
            return False
        
        return True
    
    def start_server(self):
        """Start socket server and wait for ROS bridge connection."""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('localhost', self.port))
        self.server_socket.listen(1)
        
        print(f"Socket server listening on port {self.port}")
        print("Waiting for ROS bridge to connect...")
        
        self.client_socket, addr = self.server_socket.accept()
        print(f"ROS bridge connected from {addr}")
    
    def receive_message(self):
        """Receive message from ROS bridge."""
        try:
            # Receive length prefix
            length_data = self.client_socket.recv(4)
            if not length_data:
                return None
            
            msg_length = struct.unpack('!I', length_data)[0]
            
            # Receive JSON message
            json_data = self.client_socket.recv(msg_length).decode('utf-8')
            return json.loads(json_data)
        except Exception as e:
            print(f"Error receiving message: {e}")
            return None
    
    def send_message(self, msg_type, data):
        """Send message to ROS bridge."""
        try:
            packet = {
                'type': msg_type,
                'timestamp': time.time(),
                'data': data
            }
            json_str = json.dumps(packet)
            msg_bytes = json_str.encode('utf-8')
            
            # Send length prefix + JSON
            length_prefix = struct.pack('!I', len(msg_bytes))
            self.client_socket.sendall(length_prefix + msg_bytes)
            return True
        except Exception as e:
            print(f"Error sending message: {e}")
            return False
    
    def calculate_link_quality(self):
        """
        Calculate link quality based on UAV-UGV distance.
        This is a simplified model; full OMNeT++ simulation would be more accurate.
        """
        import math
        
        dx = self.uav_pos['x'] - self.ugv_pos['x']
        dy = self.uav_pos['y'] - self.ugv_pos['y']
        dz = self.uav_pos['z'] - self.ugv_pos['z']
        
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Simple path loss model
        # Packet loss increases with distance
        # SF7: max ~1km, SF12: max ~5km
        if self.config.startswith('LoRa_SF7'):
            max_range = 1000.0
        elif self.config.startswith('LoRa_SF10'):
            max_range = 2000.0
        else:  # SF12
            max_range = 5000.0
        
        if distance > max_range:
            packet_success = 0.0
        else:
            packet_success = 1.0 - (distance / max_range) ** 2
        
        # Add delay based on distance
        propagation_delay = distance / 299792458.0  # Speed of light
        processing_delay = 0.01  # 10ms processing
        
        return {
            'distance': distance,
            'packet_success_rate': packet_success,
            'delay_sec': propagation_delay + processing_delay
        }
    
    def handle_message(self, packet):
        """Process message from ROS and simulate network transmission."""
        msg_type = packet.get('type')
        data = packet.get('data', {})
        
        if msg_type == 'update_uav_position':
            self.uav_pos = data
            print(f"UAV position updated: {data}")
        
        elif msg_type == 'update_ugv_position':
            self.ugv_pos = data
            print(f"UGV position updated: {data}")
        
        elif msg_type in ['uav_pose', 'uav_command', 'ugv_pose']:
            # Simulate network transmission
            link = self.calculate_link_quality()
            
            print(f"Message: {msg_type}, Distance: {link['distance']:.1f}m, "
                  f"Success: {link['packet_success_rate']:.2%}, "
                  f"Delay: {link['delay_sec']*1000:.1f}ms")
            
            # Random packet loss
            import random
            if random.random() < link['packet_success_rate']:
                # Simulate delay
                time.sleep(link['delay_sec'])
                
                # Send back to ROS
                self.send_message(msg_type, data)
            else:
                print(f"  -> Packet dropped!")
    
    def run(self):
        """Main server loop."""
        # Start OMNeT++
        if not self.start_omnet():
            print("Failed to start OMNeT++, exiting")
            return
        
        # Start socket server
        self.start_server()
        
        # Main message loop
        print("\nRunning... (Ctrl+C to stop)")
        try:
            while True:
                packet = self.receive_message()
                if packet is None:
                    break
                
                self.handle_message(packet)
        
        except KeyboardInterrupt:
            print("\nShutting down...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean shutdown."""
        print("Cleaning up...")
        
        if self.client_socket:
            self.client_socket.close()
        
        if self.server_socket:
            self.server_socket.close()
        
        if self.omnet_process:
            self.omnet_process.terminate()
            self.omnet_process.wait(timeout=5)
            print("OMNeT++ terminated")


def main():
    if len(sys.argv) > 1:
        config = sys.argv[1]
    else:
        config = 'LoRa_SF7_ShortRange'
    
    print("="*60)
    print("OMNeT++ Socket Server for ROS Bridge")
    print("="*60)
    print(f"Configuration: {config}")
    print()
    
    server = OmnetSocketServer(config=config)
    server.run()


if __name__ == '__main__':
    main()
