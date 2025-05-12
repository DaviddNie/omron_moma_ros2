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
from geometry_msgs.msg import Point
import asyncio
import threading

class CameraServer(Node):
    def __init__(self):
        super().__init__('camera_server')
        
        # Setup callback groups
        self.service_group = ReentrantCallbackGroup()
        self.image_group = ReentrantCallbackGroup()
        self.visualization_group = ReentrantCallbackGroup()
        
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
        
        # Visualization window
        self.visualization_enabled = True
        cv2.namedWindow("Object Detection", cv2.WINDOW_NORMAL)
        
        # Service with reentrant callback group
        self.srv = self.create_service(
            CameraSrv, 
            'camera_service', 
            self.handle_camera_request,
            callback_group=self.service_group
        )
        
        # Subscribers with their own callback group
        self.setup_subscribers()
        
        # Visualization timer with its own callback group
        # self.vis_timer = self.create_timer(
        #     0.05,  # 20Hz
        #     self.update_visualization,
        #     callback_group=self.visualization_group
        # )
        
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
        
        # Create synchronizer with the same callback group as subscribers
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

    def pixel_to_3d(self, pixel_x, pixel_y, depth_value):
        """Convert pixel+depth to 3D point (camera frame)"""
        if self.current_frame is None or self.current_depth is None:
             return None
            
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics,
            [pixel_x, pixel_y],
            depth_value * 0.001  # mm to meters
        )
        return point_3d

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
                        detections.append(point_msg)
            
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
            
            self.update_visualization()
            
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

    def update_visualization(self):
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