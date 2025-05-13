#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from cv_bridge import CvBridge
import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
from sensor_msgs.msg import Image, CameraInfo
from message_filters import ApproximateTimeSynchronizer, Subscriber
from camera_interface.srv import CameraSrv
from geometry_msgs.msg import Point, Pose, TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
from visualization_msgs.msg import Marker, MarkerArray

class CameraServer(Node):
    def __init__(self):
        super().__init__('camera_server')
        
        # Setup callback groups
        self.service_group = ReentrantCallbackGroup()
        self.image_group = ReentrantCallbackGroup()
        
        # Initialize YOLO model
        self.declare_parameter('yolo_model_path', 'default path')
        model_path = self.get_parameter('yolo_model_path').get_parameter_value().string_value
        self.model = YOLO(model_path)
        self.model.fuse()
        
        # Camera setup
        self.bridge = CvBridge()
        self.intrinsics = None
        self.current_frame = None
        self.current_depth = None
        self.last_detections = []
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Marker publisher for RViz visualization
        self.marker_pub = self.create_publisher(MarkerArray, 'detected_objects', 10)
        
        # Service
        self.srv = self.create_service(
            CameraSrv, 
            'camera_service', 
            self.handle_camera_request,
            callback_group=self.service_group
        )
        
        # Subscribers
        self.setup_subscribers()
        
        # Visualization timer
        self.vis_timer = self.create_timer(0.1, self.publish_visualization_data)
        
        self.get_logger().info("Camera Server ready")

    def setup_subscribers(self):
        """Configure image subscribers and synchronizer"""
        self.color_sub = Subscriber(
            self, 
            Image, 
            '/camera/camera/color/image_raw',
            callback_group=self.image_group
        )
        self.depth_sub = Subscriber(
            self, 
            Image, 
            '/camera/camera/aligned_depth_to_color/image_raw',
            callback_group=self.image_group
        )
        
        self.cam_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10,
            callback_group=self.image_group
        )
        
        self.ts = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.1,
        )
        self.ts.registerCallback(self.image_callback)

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
            self.intrinsics.model = rs.distortion.brown_conrady
            self.intrinsics.coeffs = list(msg.d)
            self.get_logger().info("Camera intrinsics received")

    def image_callback(self, color_msg, depth_msg):
        """Store the latest synchronized frames"""
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            self.current_depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {str(e)}")

    def transform_camera_to_world(self, point):
        """Proper coordinate transformation from camera to world frame"""
        return [
            point[2],   # Camera Z -> World X (forward)
            -point[1] + 0.038 + 0.18,   # Camera Y -> World Z (up)
            -point[0] - 0.2,  # Camera X -> World Y (left)
        ]
    
    def pixel_to_3d(self, pixel_x, pixel_y, depth_value):
        """Convert pixel+depth to 3D point in camera frame"""
        if None in [self.intrinsics, depth_value]:
            return None
            
        # Convert to camera frame coordinates (X right, Y down, Z forward)
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics,
            [pixel_y, pixel_x],
            depth_value * 0.001  # mm to meters
        )
        return self.transform_camera_to_world(point_3d)

    def handle_camera_request(self, request, response):
        """Service handler"""
        if request.command == "detect":
            return self.handle_detect(request, response)
        else:
            response.success = False
            response.message = f"Unknown command: {request.command}"
            return response

    def handle_detect(self, request, response):
        """Handle detection request"""
        if self.current_frame is None or self.current_depth is None:
            response.success = False
            response.message = "No frame available"
            return response
            
        try:
            results = self.model(self.current_frame, verbose=False)[0]
            boxes = results.boxes.xyxy.cpu().numpy()
            class_ids = results.boxes.cls.cpu().numpy()
            confidences = results.boxes.conf.cpu().numpy()
            
            detections = []
            for i, (box, cls_id, conf) in enumerate(zip(boxes, class_ids, confidences)):
                if cls_id == request.identifier and conf > 0.5:
                    x_center = int((box[0] + box[2]) / 2)
                    y_center = int((box[1] + box[3]) / 2)
                    depth = self.current_depth[int(y_center), int(x_center)]
                    
                    point_3d = self.pixel_to_3d(x_center, y_center, depth)
                    if point_3d:
                        point_msg = Point()
                        point_msg.x = point_3d[0]  # Camera X (right)
                        point_msg.y = point_3d[1]  # Camera Y (down)
                        point_msg.z = point_3d[2]  # Camera Z (forward)
                        detections.append(point_msg)
            
            self.last_detections = detections
            response.coordinates = detections
            response.success = True
            response.message = f"Found {len(detections)} objects"
            return response
            
        except Exception as e:
            self.get_logger().error(f"Detection error: {str(e)}")
            response.success = False
            response.message = f"Detection failed: {str(e)}"
            return response

    def publish_visualization_data(self):
        """Publish TF frames and markers for RViz visualization"""
        if not self.last_detections:
            return
            
        # Create marker array
        marker_array = MarkerArray()
        
        for i, detection in enumerate(self.last_detections):
            # Publish TF frame for each detection
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "camera_link"
            t.child_frame_id = f"detected_object_{i}"
            
            # Set transform (camera frame coordinates)
            t.transform.translation.x = detection.x
            t.transform.translation.y = detection.y
            t.transform.translation.z = detection.z
            
            # Default orientation (facing forward)
            q = tf_transformations.quaternion_from_euler(0, 0, 0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            
            self.tf_broadcaster.sendTransform(t)
            
            # Create RViz marker
            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "detections"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Set marker position
            marker.pose.position.x = detection.x
            marker.pose.position.y = detection.y
            marker.pose.position.z = detection.z
            
            # Set marker scale (size)
            marker.scale.x = 0.05  # 5cm diameter
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            
            # Set marker color (green)
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            # Set marker lifetime
            marker.lifetime.sec = 1
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

    def destroy_node(self):
        """Cleanup before shutdown"""
        super().destroy_node()

def main():
    rclpy.init()
    server = CameraServer()
    
    executor = MultiThreadedExecutor(num_threads=4)
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