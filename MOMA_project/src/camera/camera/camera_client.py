import rclpy
from rclpy.node import Node
from camera_interface.srv import CameraSrv
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge

class CameraClient(Node):
    def __init__(self):
        super().__init__('camera_client')
        self.client = self.create_client(CameraSrv, 'camera_service')
        self.bridge = CvBridge()
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
            
        self.get_logger().info("Camera Client ready")

    def take_photo(self, class_id):
        """Request photo detection for specific class"""
        request = CameraSrv.Request()
        request.command = "take_photo"
        request.identifier = class_id
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"Detection successful: {response.message}\n"
                    f"Found {len(response.coordinates)} objects:"
                )
                for i, point in enumerate(response.coordinates):
                    self.get_logger().info(
                        f"Object {i+1}: "
                        f"X={point.x:.3f}m, Y={point.y:.3f}m, Z={point.z:.3f}m"
                    )
                return response.coordinates
            else:
                self.get_logger().error(f"Detection failed: {response.message}")
        else:
            self.get_logger().error("Service call failed")
        return []

def main(args=None):
    rclpy.init(args=args)
    client = CameraClient()
    
    # Example: Detect mice (class 64)
    coordinates = client.take_photo(64)
    
    # Add your processing logic here with the coordinates
    client.get_logger().info("Processing complete")
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()