import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from message_filters import ApproximateTimeSynchronizer, Subscriber
from ultralytics import YOLO
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import TransformStamped

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')

        # Declare parameters
        self.declare_parameter('yolo_model_path', '/absolute/path/to/yolov11m.pt')
        model_path = self.get_parameter('yolo_model_path').get_parameter_value().string_value

        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Load YOLO model
        self.get_logger().info(f"Loading YOLO model from: {model_path}")
        self.model = YOLO(model_path)
        
        # Camera intrinsics
        self.intrinsics = None
        
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.color_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        self.cam_info_sub = self.create_subscription(
            CameraInfo, 
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Synchronizer for color and depth images
        self.ts = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)

        # Flag to track if we've processed a frame
        self.received = False

        # Class index for the object we want to detect (47 for apple in COCO)
        self.target_class_index = 47  # Replace with your target class index

    def camera_info_callback(self, camera_info):
        """Callback for camera info to get intrinsics"""
        if self.intrinsics is not None:
            return
            
        self.intrinsics = rs.intrinsics()
        self.intrinsics.width = camera_info.width
        self.intrinsics.height = camera_info.height
        self.intrinsics.ppx = camera_info.k[2]
        self.intrinsics.ppy = camera_info.k[5]
        self.intrinsics.fx = camera_info.k[0]
        self.intrinsics.fy = camera_info.k[4]
        
        if camera_info.distortion_model == 'plumb_bob':
            self.intrinsics.model = rs.distortion.brown_conrady
        elif camera_info.distortion_model == 'equidistant':
            self.intrinsics.model = rs.distortion.kannala_brandt4
            
        self.intrinsics.coeffs = [i for i in camera_info.d]
        self.get_logger().info("Received camera intrinsics")

    def pixel_to_3d(self, pixel_x, pixel_y, depth_value):
        """Convert pixel coordinates + depth to 3D point in camera frame"""
        if self.intrinsics is None:
            self.get_logger().warn("Camera intrinsics not available yet")
            return None
            
        # Depth value is in mm, convert to meters for the deprojection
        depth_in_meters = depth_value * 0.001
        
        # Note: The order of pixel coordinates is (x, y) for rs2_deproject_pixel_to_point
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, 
            [pixel_x, pixel_y], 
            depth_in_meters
        )
        
        return point_3d

    def transform_to_base_frame(self, point_camera_frame, timestamp):
        for _ in range(10):  # Retry for 10 seconds
            try:
                transform = self.tf_buffer.lookup_transform(
                    'base',
                    'camera_link',
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                # Transform the point and return
                point_stamped = PointStamped()
                point_stamped.header.frame_id = "camera_link"
                point_stamped.header.stamp = timestamp
                point_stamped.point.x = point_camera_frame[0]
                point_stamped.point.y = point_camera_frame[1]
                point_stamped.point.z = point_camera_frame[2]
                return do_transform_point(point_stamped, transform)
                
            except tf2_ros.TransformException as e:
                self.get_logger().warn(f"Waiting for transform: {e}")
                rclpy.spin_once(self, timeout_sec=1.0)
        
        self.get_logger().error("Failed to get transform after 10 attempts!")
        return None

    def image_callback(self, color_msg, depth_msg):
        if self.received:
            return  # Only process one frame

        self.get_logger().info("Received synchronized color and depth frames")

        # Convert ROS Image messages to OpenCV
        try:
            color_image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Error converting images: {str(e)}")
            return

        # Flip both vertically (if needed)
        color_image = cv2.flip(color_image, 0)
        depth_image = cv2.flip(depth_image, 0)

        # Resize (if needed)
        color_image = cv2.resize(color_image, (640, 480))
        depth_image = cv2.resize(depth_image, (640, 480))

        # Run YOLO detection
        results = self.model(color_image)[0]
        annotated_image = color_image.copy()

        # Get detection results
        boxes = results.boxes.xyxy.cpu().numpy()
        class_ids = results.boxes.cls.cpu().numpy()
        confidences = results.boxes.conf.cpu().numpy()

        # Process detections
        for i, class_id in enumerate(class_ids):
            if class_id == self.target_class_index and confidences[i] >= 0.5:
                box = boxes[i]
                confidence = confidences[i]

                # Calculate center of bounding box
                x_center = int((box[0] + box[2]) / 2)
                y_center = int((box[1] + box[3]) / 2)

                # Get depth at center point
                depth_value = depth_image[y_center, x_center]

                # Convert to 3D coordinates in camera frame
                point_3d = self.pixel_to_3d(x_center, y_center, depth_value)
                
                if point_3d is not None:
                    # Transform to base frame
                    point_base = self.transform_to_base_frame(point_3d, color_msg.header.stamp)
                    
                    if point_base is not None:
                        self.get_logger().info(
                            f"Detected object at: "
                            f"Pixel: ({x_center}, {y_center}), "
                            f"Depth: {depth_value} mm, "
                            f"Camera Frame: ({point_3d[0]:.3f}, {point_3d[1]:.3f}, {point_3d[2]:.3f}) m, "
                            # f"Base Frame: ({point_base[0]:.3f}, {point_base[1]:.3f}, {point_base[2]:.3f}) m, "
                            f"Base Frame: ({point_base.point.x:.3f}, {point_base.point.y:.3f}, {point_base.point.z:.3f}) m, "
                            f"Confidence: {confidence:.2f}"
                        )

                        # Draw visualization
                        cv2.rectangle(annotated_image, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
                        cv2.circle(annotated_image, (x_center, y_center), 5, (0, 255, 0), -1)
                        
                        # Add text annotations
                        cv2.putText(annotated_image, f"Depth: {depth_value}mm", (x_center + 10, y_center),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        cv2.putText(annotated_image, f"{confidence:.2f}", (int(box[2]), int(box[3]) + 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                        
                        # Display base frame coordinates
                        cv2.putText(annotated_image, f"X: {point_base.point.x:.2f}m", (int(box[0]), int(box[1]) - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
                        cv2.putText(annotated_image, f"Y: {point_base.point.y:.2f}m", (int(box[0]), int(box[1]) - 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                        cv2.putText(annotated_image, f"Z: {point_base.point.z:.2f}m", (int(box[0]), int(box[1]) - 50),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        # Show results
        cv2.imshow('Object Detection with 3D Coordinates', annotated_image)
        
        # Normalize depth for visualization
        depth_visual = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        depth_visual = cv2.convertScaleAbs(depth_visual)
        cv2.imshow('Depth Image', depth_visual)
        
        # Wait for key press
        while True:
            if cv2.waitKey(100) == 27:  # ESC key to exit
                break

        cv2.destroyAllWindows()
        self.received = True
        rclpy.shutdown()

def main():
    rclpy.init()
    node = ObjectDetectionNode()
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