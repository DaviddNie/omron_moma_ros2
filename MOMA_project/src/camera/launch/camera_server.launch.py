import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    camera_pkg_dir = get_package_share_directory('camera')
    model_path = os.path.join(camera_pkg_dir, 'models', 'yolo11m.pt')

    return LaunchDescription([
        Node(
            package='camera',
            executable='camera_server',
            name='one_frame_subscriber',
            parameters=[
                {'yolo_model_path': model_path}
            ]
        )
    ])