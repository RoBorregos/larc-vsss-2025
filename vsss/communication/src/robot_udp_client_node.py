#!/usr/bin/env python3

# ROS2 node for sending RPMs to robots via UDP, subscribing to cmd_vel for each robot
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import struct


class RobotUDPClient:
    def __init__(self, robot_ip, robot_port):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.client_socket = None
        self.connect()

    def connect(self):
        print("Setting up UDP socket...")
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"UDP socket ready for {self.robot_ip}:{self.robot_port}")
        except socket.error as e:
            print(f"Socket error: {e}")
            self.client_socket = None

    def send_floats(self, float1, float2):
        if not self.client_socket:
            print("UDP socket not available.")
            return False
        try:
            # Pack both floats into a single UDP packet
            packed_data = struct.pack('<ff', float1, float2)
            self.client_socket.sendto(packed_data, (self.robot_ip, self.robot_port))
        except socket.error as e:
            print(f"Socket error during send: {e}")
            return False
        return True

    def close(self):
        if self.client_socket:
            self.client_socket.close()
            self.client_socket = None

def twist_to_rpm(twist, wheel_radius=0.03, wheel_base=0.076):
    # Example conversion: linear.x and angular.z to left/right wheel RPM
    v = twist.linear.x
    w = twist.angular.z
    v_left = v - w * wheel_base / 2.0
    v_right = v + w * wheel_base / 2.0
    rpm_left = (v_left / (2 * 3.1416 * wheel_radius)) * 60.0
    rpm_right = (v_right / (2 * 3.1416 * wheel_radius)) * 60.0
    return rpm_left, rpm_right

class SingleRobotUDPNode(Node):
    def __init__(self):
        super().__init__('single_robot_udp_node')
        # Parameters should be loaded from external YAML config via ROS2 launch or CLI
        
        self.get_logger().info(f"Waiting to Start")
        self.declare_parameter('robot_name', 'robot1')
        self.declare_parameter('robot_ip', '192.168.0.188')
        self.declare_parameter('robot_port', 8080)    

        name = self.get_parameter('robot_name').value
        ip = self.get_parameter('robot_ip').value
        port = self.get_parameter('robot_port').value

        
        self.get_logger().info(f"Node Started with values name {name}, connection->({ip}:{port})")
        self.client = RobotUDPClient(ip, port)

        self.get_logger().info(f"Waiting to Start")
        topic = f'/{name}/cmd_vel'
        self.create_subscription(Twist, topic, self.cmd_vel_callback, 10)
        self.get_logger().info(f"Subscribed to {topic} for {name} ({ip}:{port})")

    def destroy_node(self):
        # Ensure the UDP client is closed when the node is destroyed
        if self.client:
            self.client.close()
        super().destroy_node()

    def cmd_vel_callback(self, msg):
        rpm_left, rpm_right = twist_to_rpm(msg)
        sent = self.client.send_floats(rpm_left, rpm_right)

            self.get_logger().info(f"Sent: L={rpm_left:.2f} R={rpm_right:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = SingleRobotUDPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()