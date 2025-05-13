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
import tf2_ros
import tf_transformations
from visualization_msgs.msg import Marker, MarkerArray
import threading
import asyncio

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
        self._visualization_lock = threading.Lock()
        
        # OpenCV Visualization Setup
        self.visualization_enabled = True
        cv2.namedWindow("Object Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Object Detection", 1280, 720)
        
        # RViz Visualization Setup
        self.tf_broadcaster = TransformBroadcaster(self)
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
        
        # Visualization timers
        self.rviz_vis_timer = self.create_timer(0.1, self.update_rviz_visualization)  # 10Hz
        
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
            with self._visualization_lock:
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
        if self.current_frame is None or self.current_depth is None:
            return None
            
        # Convert to camera frame coordinates (X right, Y down, Z forward)
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics,
            [pixel_y, pixel_x],
            depth_value * 0.001  # mm to meters
        )
        return self.transform_camera_to_world(point_3d)


    def handle_camera_request(self, request, response):
        """Service handler that uses async pattern"""
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

        result = loop.run_until_complete(self.handle_request_async(request))
        
        self.get_logger().info(f"Processing completed")

        if isinstance(result, dict):
            response.coordinates = result.get('coordinates', [])
            response.success = result.get('success', False)
            response.message = result.get('message', '')
        else:
            response.success = False
            response.message = "Invalid response format"
            
        return response
    
    async def handle_request_async(self, request):
        """Async handler for camera requests"""
        self.get_logger().info(f"Processing request: {request.command}")
        
        if request.command == "detect":
            return await self.handle_detect(request)
        else:
            return {
                'success': False,
                'message': f"Unknown command: {request.command}"
            }

 
    async def handle_detect(self, request):
        """Async handler for detect command"""
        if self.current_frame is None or self.current_depth is None:
            return {
                'success': False,
                'message': "No frame available"
            }
            
        try:
            # Run detection (this could be moved to a thread if too slow)
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
                        point_msg.x = point_3d[0]
                        point_msg.y = point_3d[1]
                        point_msg.z = point_3d[2]
                        
                        try:
                            # Transform point from camera frame to base frame
                            tf_buffer = tf2_ros.Buffer()
                            tf_listener = tf2_ros.TransformListener(tf_buffer, self)
                            
                            # Wait for the transform to be available
                            transform = tf_buffer.lookup_transform(
                                'base',
                                'camera_link',
                                rclpy.time.Time(),
                                timeout=rclpy.duration.Duration(seconds=2.0))
                            
                            # Create a PoseStamped with the point in camera frame
                            camera_pose = Pose()
                            camera_pose.position = point_msg
                            
                            # Transform the point to base frame
                            base_pose = tf2_ros.do_transform_pose(camera_pose, transform)
                            
                            # Create new point with transformed coordinates
                            transformed_point = Point()
                            transformed_point.x = base_pose.position.x
                            transformed_point.y = base_pose.position.y
                            transformed_point.z = base_pose.position.z
                            
                            detections.append(transformed_point)
                            
                            # Print the transformed coordinates
                            self.get_logger().info(
                                f"Transformed coordinates (base frame): "
                                f"X: {transformed_point.x:.3f}, "
                                f"Y: {transformed_point.y:.3f}, "
                                f"Z: {transformed_point.z:.3f}")
                                
                        except (tf2_ros.LookupException, 
                                tf2_ros.ConnectivityException, 
                                tf2_ros.ExtrapolationException) as e:
                            self.get_logger().error(f"TF transform failed: {str(e)}")
                            detections.append(point_msg)  # Fall back to camera frame coordinates
            
            # Update visualization state
            self.last_detections = [{
                'box': box,
                'center': (x_center, y_center),
                'point_3d': point_3d,
                'confidence': conf
            } for box, cls_id, conf, (x_center, y_center, point_3d) in 
              zip(boxes, class_ids, confidences, 
                  [(int((b[0]+b[2])/2), int((b[1]+b[3])/2), 
                    self.pixel_to_3d(int((b[0]+b[2])/2), int((b[1]+b[3])/2), 
                                    self.current_depth[int((b[1]+b[3])/2), int((b[0]+b[2])/2)]))
                   for b in boxes])
              if cls_id == request.identifier and conf > 0.5]
            
            self.update_cv_visualization()
            
            return {
                'coordinates': detections,
                'success': True,
                'message': f"Found {len(detections)} objects"
            }
            
        except Exception as e:
            self.get_logger().error(f"Detection error: {str(e)}")
            return {
                'success': False,
                'message': f"Detection failed: {str(e)}"
            }

    def update_cv_visualization(self):
        """Update the visualization window with current frame and detections"""
        if not self.visualization_enabled or self.current_frame is None:
            self.get_logger().info("No frame available for visualization", 
                                 throttle_duration_sec=1.0)
            return
        
        display_frame = self.current_frame.copy()
        
        # Draw detections
        for det in self.last_detections:
            box = det['box']
            center = det['center']
            point_3d = det['point_3d']
            conf = det['confidence']

            # Draw bounding box
            cv2.rectangle(display_frame, 
                         (int(box[0]), int(box[1])),
                         (int(box[2]), int(box[3])),
                         (0, 255, 0), 2)
            
            # Draw center point
            cv2.circle(display_frame, 
                      (int(center[0]), int(center[1])), 
                      5, (0, 255, 0), -1)
            
            # Draw coordinates text
            coord_text = f"X:{point_3d[0]:.2f}, Y:{point_3d[1]:.2f}, Z:{point_3d[2]:.2f}"
            cv2.putText(display_frame, coord_text,
                       (int(box[0]), int(box[1]) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            # Draw confidence
            conf_text = f"Conf: {conf:.2f}"
            cv2.putText(display_frame, conf_text,
                       (int(box[2]), int(box[3]) + 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Show the frame
        cv2.imshow("Object Detection", display_frame)
        cv2.waitKey(1)

    def update_rviz_visualization(self):
        """Update RViz visualization with TF frames and markers"""
        if not hasattr(self, 'last_detections') or not self.last_detections:
            return
            
        try:
            # Create marker array
            marker_array = MarkerArray()
            
            for i, detection in enumerate(self.last_detections):
                point_3d = detection['point_3d']

                # Publish TF frame for each detection
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "camera_link"
                t.child_frame_id = f"detected_object_{i}"
                
                # Set transform (camera frame coordinates)
                t.transform.translation.x = point_3d[0]
                t.transform.translation.y = point_3d[1]
                t.transform.translation.z = point_3d[2]
                
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
                marker.pose.position.x = point_3d[0]
                marker.pose.position.y = point_3d[1]
                marker.pose.position.z = point_3d[2]
                
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
            
        except Exception as e:
            self.get_logger().error(f"RViz Visualization error: {str(e)}")

    def destroy_node(self):
        """Cleanup before shutdown"""
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    server = CameraServer()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)

    # Start the executor in a background thread
    def spin_executor():
        executor.spin()
    executor_thread = threading.Thread(target=spin_executor, daemon=True)
    executor_thread.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(server, timeout_sec=0.1)
    except KeyboardInterrupt:
        server.get_logger().info("Shutting down server")
    finally:
        server.destroy_node()
        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    main()