#!/usr/bin/env python3

import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from .vision_constants import CAMERA_TOPIC

"""
    Node that takes camera input and publishes it 
    to a ROS2 topic (CAMERA_TOPIC).
"""
class CameraFrames(Node):
    def __init__(self):
        """Initialize the camera frame publisher."""
        super().__init__('camera_frames')
        self.bridge = CvBridge()
        self.video_id = self.declare_parameter("Video_ID", 2)
        self.publisher_ = self.create_publisher(
            Image, CAMERA_TOPIC, 10
        )
        self.get_logger().info("CameraFrames node has started.")
        self.cap = cv2.VideoCapture(self.video_id.value)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.run()

    def run(self):
        """
        Get frames from camera(camera_id) and publish them.
        """
        while rclpy.ok():
            success, frame = self.cap.read()
            if not success:
                self.get_logger().info("No frame captured.")
                continue
            
            image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(image)
            frame_resized = cv2.resize(frame, None, fx=0.5, fy=0.5)
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = CameraFrames()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()