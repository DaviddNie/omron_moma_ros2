# import cv2
# import numpy as np

# # Load an image
# image = cv2.imread("testPhotos/centre1.png")

# # Get the dimensions of the original image
# original_height, original_width = image.shape[:2]
# center_x, center_y = original_width / 2, original_height / 2
# print(f"Width and Height are: {original_width}, {original_height}")    

# # Show the result
# cv2.imshow("Output", image)

# # Wait for a key press to close the window
# cv2.waitKey(0)
# cv2.destroyAllWindows()

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from message_filters import ApproximateTimeSynchronizer, Subscriber

class OneFrameSubscriber(Node):
    def __init__(self):
        super().__init__('one_frame_subscriber')

        self.bridge = CvBridge()

        # Subscribers to both color and aligned depth
        self.color_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')

        # Synchronize the messages
        self.ts = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.callback)

        self.received = False

    def callback(self, color_msg, depth_msg):
        if self.received:
            return  # Only want one frame

        self.get_logger().info("Received synchronized color and depth frames")

        # Convert ROS Image messages to OpenCV
        color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # Optional: normalize depth for visualization
        depth_visual = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_visual = cv2.convertScaleAbs(depth_visual)

        # Show both images
        cv2.imshow('Color Image', color_image)
        cv2.imshow('Depth Image', depth_visual)

        # Use a short loop instead of waitKey(0)
        while True:
            if cv2.waitKey(100) == 27:  # ESC key to close
                break
        
        cv2.destroyAllWindows()

        self.received = True
        rclpy.shutdown()

def main():
    rclpy.init()
    node = OneFrameSubscriber()
    try:
        while rclpy.ok() and not node.received:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()