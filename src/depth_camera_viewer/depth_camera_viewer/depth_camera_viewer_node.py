#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class DepthCameraViewer(Node):
    def __init__(self):
        super().__init__('depth_camera_viewer')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/depth_camera',
            self.image_callback,
            10
        )
        self.get_logger().info("Depth camera viewer node started")

    def image_callback(self, msg: Image):
        self.get_logger().info(f"Received image from /depth_camera - shape: {msg.width}x{msg.height}, encoding: {msg.encoding}")
        try:
            import numpy as np
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Replace NaN and inf values with 0
            cv_image = np.nan_to_num(cv_image, nan=0.0, posinf=0.0, neginf=0.0)
            
            # Log the depth data statistics
            min_val = cv_image.min()
            max_val = cv_image.max()
            mean_val = cv_image.mean()
            self.get_logger().info(f"Depth data - Min: {min_val}, Max: {max_val}, Mean: {mean_val}")
            
            # Normalize depth for visualization
            if max_val > min_val:
                depth_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
            else:
                depth_normalized = np.zeros_like(cv_image)
                self.get_logger().warn("Depth values are constant, displaying black")
            
            depth_colored = cv2.applyColorMap(depth_normalized.astype('uint8'), cv2.COLORMAP_JET)
            
            cv2.imshow('Depth Camera', depth_colored)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to convert depth image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthCameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
