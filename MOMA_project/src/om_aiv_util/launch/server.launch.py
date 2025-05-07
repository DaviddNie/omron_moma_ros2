from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

#parameters_file_path = os.path.join(get_package_share_directory('om_aiv_util'), 'config', 'main_param.yaml')

# ros2 launch om_aiv_util server.launch.py

def generate_launch_description():

	# CHECK THESE DESCRIPTIONS AGAIN - THE IP ADDRESS CAN BE CHANGED FOR WIRED CONFIGURATION

	# IP and Port no. of the AMR can be changed in SetNetGo - Network - Wireless Ethernet
	# Default is 192.168.2.2, 7171
	# Password is set under Mobile Planner - Configuration - Robot Interface - ARCL Server Setup - Password
	# This node is to connect to the AMR
	# ros2 run om_aiv_util arcl_api_server --ros-args -p ip_address:=192.168.2.2 -p port:=7171 -p def_arcl_passwd:=omron
    arcl_api = Node(
        package='om_aiv_util',
        executable='arcl_api_server',
        #name='arcl_api_server',
        output='log',
        parameters=[{
            'ip_address': "192.168.1.1", # 192.168.2.2 for wireless
            'port': 7171,
            'def_arcl_passwd': "omron"
        }]
    )
    
    # This is the machines IP and port
    # Can be changed on Mobile Planner - Configuration - Robot Interface - Outgoing ARCL Connection Setup
    # Static IP for Leo's wireless adapter on the lab computer is set as 192.168.2.50
    # Testing if 192.168.2.51 is more stable for laptop IP. Since its different to pc
    # This node is to retrieve info from the AMR via ARCL commands
    # ros2 run om_aiv_util ld_states_publisher --ros-args -p local_ip:=192.168.2.50 -p local_port:=7179
    ld_states = Node(
        package='om_aiv_util',
        executable='ld_states_publisher',
        #name='ld_states_publi',
        output='screen',
        parameters=[{
            'local_ip': "192.168.1.50",
            'local_port': 7179
        }]
    )
    
    # The LD's Network Settings for ARCL setup
    # Can be changed in SetNetGo - Network - User LAN Ethernet
    # This node is to control the AMR via ARCL commands
    # ros2 run om_aiv_navigation action_server --ros-args -p ip_address:=192.168.2.2 -p port:=7171 -p def_arcl_passwd:=omron
    action_serve = Node(
        package='om_aiv_navigation',
        executable='action_server',
        #name = 'action_server',
        output='screen',
        parameters=[{
            'ip_address': "192.168.1.1", # 192.168.2.2 for wireless
            'port': 7171,
            'def_arcl_passwd': "omron"
        }]
    )

    return LaunchDescription([
        arcl_api, 
        ld_states,
        action_serve
    ])

