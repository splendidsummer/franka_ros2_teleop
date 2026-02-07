#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2

class DualCameraPublisher(Node):
    def __init__(self):
        super().__init__('dual_camera_pubulisher')
        self.bridge = CvBridge()

        # Replace with your RealSense serial numbers
        serial_1 = '123456789ABC'
        serial_2 = '987654321DEF'
        self.rate_hz = 30.0

        # Create two RealSense pipelines
        self.pipeline1 = rs.pipeline()
        self.pipeline2 = rs.pipeline()
        cfg1 = rs.config()
        cfg2 = rs.config()
        cfg1.enable_device(serial_1)
        cfg2.enable_device(serial_2)
        cfg1.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, int(self.rate_hz))
        cfg2.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, int(self.rate_hz))
        self.pipeline1.start(cfg1)
        self.pipeline2.start(cfg2)

        # Publishers for each camera
        self.pub0 = self.create_publisher(Image, 'camera0/image_raw', 10)
        self.pub1 = self.create_publisher(Image, 'camera1/image_raw', 10)

        # Timer to grab & publish frames
        self.timer = self.create_timer(1.0 / self.rate_hz, self.timer_callback)

        # OpenCV windows
        cv2.namedWindow('Camera 0', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Camera 1', cv2.WINDOW_NORMAL)

    def timer_callback(self):
        # Wait for and fetch frames from each pipeline
        frames0 = self.pipeline1.wait_for_frames()
        frames1 = self.pipeline2.wait_for_frames()
        color_frame0 = frames0.get_color_frame()
        color_frame1 = frames1.get_color_frame()
        if not color_frame0 or not color_frame1:
            return

        # Convert to numpy arrays
        img0 = np.asanyarray(color_frame0.get_data())
        img1 = np.asanyarray(color_frame1.get_data())

        # Prepare ROS Image messages
        now = self.get_clock().now().to_msg()
        msg0 = self.bridge.cv2_to_imgmsg(img0, encoding='bgr8')
        msg0.header.stamp = now
        msg0.header.frame_id = 'cam0_frame'
        msg1 = self.bridge.cv2_to_imgmsg(img1, encoding='bgr8')
        msg1.header.stamp = now
        msg1.header.frame_id = 'cam1_frame'

        # Publish
        self.pub0.publish(msg0)
        self.pub1.publish(msg1)

        # Display with OpenCV
        cv2.imshow('Camera 0', img0)
        cv2.imshow('Camera 1', img1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Shutting down cameras.")
            rclpy.shutdown()

    def destroy_node(self):
        # Stop pipelines and destroy windows on shutdown
        self.pipeline1.stop()
        self.pipeline2.stop()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DualCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()