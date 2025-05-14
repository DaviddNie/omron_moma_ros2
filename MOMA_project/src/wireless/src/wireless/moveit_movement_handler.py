import rclpy
import tf2_ros
from geometry_msgs.msg import Point, Pose, TransformStamped
import tf_transformations
from rclpy.time import Time
import pyrealsense2 as rs
from sensor_msgs.msg import CameraInfo
from rclpy.callback_groups import ReentrantCallbackGroup
import tf2_geometry_msgs
from geometry_msgs.msg import Quaternion
from tm_msgs.srv import MovementRequest

class MoveitMovementHandler:
    def __init__(self, node):
        self.node = node
        
    def handle_move2point(self, positions):
        return