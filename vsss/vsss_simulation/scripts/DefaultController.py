#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DefaultContoller(Node):
    def __init__(self):
        super().__init__('DefaultController')
        
        #Declare Parameter
        self.declare_parameter('linear_vel',0.2)
        self.declare_parameter('angular_vel', 0.5)

        # Current velocity values
        self.linear_vel = self.get_parameter('linear_vel').value
        self.angular_vel = self.get_parameter('angular_vel').value # rad/s


        # Create publisher for cmd_vel topic
        self.publisher = self.create_publisher(Twist,'cmd_vel', 10)
        
        # Timer to publish commands at regular intervals
        timer_period = 0.1  # seconds (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        
        self.get_logger().info(f"Publishing linear: {self.linear_vel} m/s, angular: {self.angular_vel} rad/s")

    def timer_callback(self):
        """Callback function for timer"""
        msg = Twist()
        
        # Set linear and angular velocities
        msg.linear.x = self.linear_vel
        msg.angular.z = self.angular_vel
        
        # Publish the message
        self.publisher.publish(msg)
        
        # Optional: Log the published values
        # self.get_logger().info(f'Publishing: linear={msg.linear.x}, angular={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = DefaultContoller()
    
    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        # Send stop command before exiting
        stop_msg = Twist()
        velocity_publisher.publisher.publish(stop_msg)
        velocity_publisher.get_logger().info("Stopping the robot and shutting down...")
    
    # Cleanup
    velocity_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()