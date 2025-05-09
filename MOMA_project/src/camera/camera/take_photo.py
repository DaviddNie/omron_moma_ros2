import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

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

        self.apple_class_index = 47  # Replace with the actual index for "apple"

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

        color_image = cv2.resize(color_image, (640, 480))
        depth_image = cv2.resize(depth_image, (640, 480))

        # === YOLO Detection ===
        results = self.model(color_image)[0]  # Get first result from the batch
        annotated_image = color_image.copy()  # Start with the original image to draw on

        # Get the bounding boxes (in xyxy format), class labels, and confidence scores
        boxes = results.boxes.xyxy.cpu().numpy()
        class_ids = results.boxes.cls.cpu().numpy()
        confidences = results.boxes.conf.cpu().numpy()  # Get the confidence scores

        # Filter out all detections that are not "apple"
        apple_boxes = []
        for i, class_id in enumerate(class_ids):
            if class_id == self.apple_class_index:
                box = boxes[i]
                confidence = confidences[i]  # Get the confidence for this detection

                # Only consider the detection if confidence is above a threshold (e.g., 0.5)
                if confidence >= 0.5:
                    # Calculate the center of the bounding box (x_center, y_center)
                    x_center = int((box[0] + box[2]) / 2)
                    y_center = int((box[1] + box[3]) / 2)

                    # Get the depth at the center of the bounding box
                    depth_value = depth_image[y_center, x_center]

                    # Log or print the center, depth, and confidence
                    self.get_logger().info(f"Detected apple at (x, y): ({x_center}, {y_center}), Depth: {depth_value} mm, Confidence: {confidence}")

                    # Add the bounding box, depth, and confidence data to the list of apple detections
                    apple_boxes.append((box, depth_value, confidence))

                    # Draw the apple bounding box
                    cv2.rectangle(annotated_image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)

                    # Draw center dot
                    cv2.circle(annotated_image, (x_center, y_center), 5, (0, 255, 0), -1)

                    # Display the depth next to the center dot
                    cv2.putText(annotated_image, f"Depth: {depth_value}mm", (x_center + 10, y_center),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                    # Display the confidence score at the bottom-right of the bounding box
                    conf_x = int(box[2])
                    conf_y = int(box[3]) + 15  # 15 pixels below the bottom-right corner
                    cv2.putText(annotated_image, f"{confidence:.2f} apple", (conf_x, conf_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # If no apples are detected, log this
        if not apple_boxes:
            self.get_logger().info("No apples detected in the image.")

        # Optional: normalize depth for visualization
        depth_visual = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_visual = cv2.convertScaleAbs(depth_visual)

        # Show the images with apple detections
        cv2.imshow('YOLO Detected Apples with Depth', annotated_image)
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
