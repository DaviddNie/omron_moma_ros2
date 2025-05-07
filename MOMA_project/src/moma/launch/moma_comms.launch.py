from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    amr_server = get_package_share_directory('om_aiv_util') + '/server.launch.py'

    return LaunchDescription([
        # For the AMR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(amr_server)
        ),
        # For the TM
        Node(
            package='tm_driver',
            executable='tm_driver',
            output='screen',
            emulate_tty=True,
            parameters=[{'192.168.1.2'}]
        )
    ])