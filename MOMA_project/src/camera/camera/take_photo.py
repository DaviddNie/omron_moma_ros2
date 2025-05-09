import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from message_filters import ApproximateTimeSynchronizer, Subscriber
from ultralytics import YOLO


class OneFrameSubscriber(Node):
    def __init__(self):
        super().__init__('one_frame_subscriber')

        self.declare_parameter('yolo_model_path', '/absolute/path/to/yolov11m.pt')
        model_path = self.get_parameter('yolo_model_path').get_parameter_value().string_value

        self.bridge = CvBridge()

        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        self.model = YOLO(model_path)
        
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

        # Flip both vertically
        color_image = cv2.flip(color_image, 0)
        depth_image = cv2.flip(depth_image, 0)

        # === YOLO Detection ===
        results = self.model(color_image)[0]  # Get first result from the batch
        annotated_image = results.plot()  # Draw bounding boxes and labels

        # Optional: normalize depth for visualization
        depth_visual = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_visual = cv2.convertScaleAbs(depth_visual)

        # Show both images
        cv2.imshow('YOLO Detected Objects', annotated_image)
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
