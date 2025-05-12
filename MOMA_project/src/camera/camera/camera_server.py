import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from camera_interface.srv import CameraSrv
from geometry_msgs.msg import Point

class CameraServer(Node):
    def __init__(self):
        super().__init__('camera_server')
        
        # Initialize YOLO model
        self.declare_parameter('yolo_model_path', 'default path')
        model_path = self.get_parameter('yolo_model_path').get_parameter_value().string_value
        self.model = YOLO(model_path)
        self.model.fuse()  # Optimize model
        
        # Camera setup
        self.bridge = CvBridge()
        self.intrinsics = None
        self.current_frame = None
        self.current_depth = None
        
        # Service
        self.srv = self.create_service(
            CameraSrv, 
            'camera_service', 
            self.handle_camera_request
        )
        
        # Subscribers for synchronized images
        self.color_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw')
        
        # Camera info subscriber
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Synchronizer for color+depth images
        self.ts = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.image_callback)
        
        self.get_logger().info("Camera Server ready")

    def camera_info_callback(self, msg):
        """Store camera intrinsics when available"""
        if self.intrinsics is None:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = msg.width
            self.intrinsics.height = msg.height
            self.intrinsics.ppx = msg.k[2]
            self.intrinsics.ppy = msg.k[5]
            self.intrinsics.fx = msg.k[0]
            self.intrinsics.fy = msg.k[4]
            self.intrinsics.model = rs.distortion.brown_conrady  # Default
            self.intrinsics.coeffs = list(msg.d)
            self.get_logger().info("Camera intrinsics received")

    def image_callback(self, color_msg, depth_msg):
        """Store the latest synchronized frames"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            self.current_depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {str(e)}")

    def pixel_to_3d(self, pixel_x, pixel_y, depth_value):
        """Convert pixel+depth to 3D point (camera frame)"""
        if None in [self.intrinsics, depth_value]:
            return None
            
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics,
            [pixel_x, pixel_y],
            depth_value * 0.001  # mm to meters
        )
        return point_3d

    def handle_camera_request(self, request, response):
        """Service handler for detection requests"""
        if request.command == "take_photo":
            if self.current_frame is None or self.current_depth is None:
                response.success = False
                response.message = "No frame available"
                return response
                
            # Run detection on current frame
            results = self.model(self.current_frame, verbose=False)[0]
            boxes = results.boxes.xyxy.cpu().numpy()
            class_ids = results.boxes.cls.cpu().numpy()
            confidences = results.boxes.conf.cpu().numpy()
            
            # Filter for requested class
            detections = []
            for i, class_id in enumerate(class_ids):
                if class_id == request.identifier and confidences[i] > 0.5:
                    box = boxes[i]
                    x_center = int((box[0] + box[2]) / 2)
                    y_center = int((box[1] + box[3]) / 2)
                    depth = self.current_depth[int(y_center), int(x_center)]
                    
                    point_3d = self.pixel_to_3d(x_center, y_center, depth)
                    if point_3d:
                        point_msg = Point()
                        point_msg.x = point_3d[0]
                        point_msg.y = point_3d[1]
                        point_msg.z = point_3d[2]
                        detections.append(point_msg)
            
            response.coordinates = detections
            response.success = True
            response.message = f"Found {len(detections)} objects"
        else:
            response.success = False
            response.message = "Unknown command"
            
        return response

def main():
    rclpy.init()
    server = CameraServer()
    
    # Multi-threaded executor for handling service calls while processing images
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        server.get_logger().info("Shutting down server")
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()