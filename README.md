# UNSW Thesis Project : ROS2 <-> Omron MoMa
This repo contains the files and information necessary to use ROS2 Humble to interface with UNSW Mechatronics Lab MoMa system purshased from Omron. 

This repo is heavily based on the [Omron APAC repositories](https://github.com/OmronAPAC), that use ROS2 Foxy to interface with a similar Omron MoMa system, though condensed and rewritten to easily work with UNSW's MoMa system.

# Overview of Components Necessary
## Omron MoMa
The Omron MoMa consists of the following components:
- LD250
- TM12-S
- The plc and input output modules
- the gripper
- miscellaneous wiring components
- Add more details to this
To use the MoMa as supplied by Omron, the LD250 will need this program, the tm12s will need this program, and the plc will need this program. Refer to these documents on how to install and use the software
## ROS2 System
To use ROS2 to interface with the MoMa, the following components are necessary:
- Remote PC (Ubuntu 22.04)
- Onboard PC (Ubuntu 22.04)
- Configuration PC (Windows 10/11)
- Ethernet cables
- Wireless router

## Features
The folder consists of all packages to use MoveIt with the TM, and to use the LD

The following packages are for the TM
  - image_sub
  - tm_description
  - tm_driver
  - tm_image
  - tm_inspect
  - tm_mod_urdf
  - tm_moveit
  - tm_msgs

The following packages are for the LD
  - amr_visualisation
  - om_aiv_msg
  - om_aiv_navigation
  - om_aiv_util

The following packages are for the MoMa (i.e. it launches respective files to use both as one cohesive system)
  - moma

Note that when building from scratch your computer may freeze. Instead use export MAKEFLAGS="-j 1" on every terminal you wish to build in. This will ensure only 1 CPU core is being used - you can change 1 to alter the number of CPU cores you wish to use.

## How to connect to the LD
  - Ensure the IP addresses in om_aiv_util/launch/server.launch.py are correct
  - Build the package
  - source install/setup.bash
  - export ROS_DOMAIN_ID=5
  - ros2 launch om_aiv_util server.launch.py
  - ros2 launch amr_visualisation display.launch.py

## How to connect to the TM
  - Build the package
  - source install/setup.bash
  - export ROS_DOMAIN_ID=5
  - ros2 launch tm12s_moveit_config tm12s_run_move_group.launch.py robot_ip:=192.168.1.2
  - Note that this will run the tm_driver as well, so currently will only work wired. Will look to splitting the functionality soon

### Get Current Joint Configuration in radian **when moveit is running**

`ros2 topic echo /joint_states --once | grep position`

# Custom Packages


## amr_arcl_interface
The package allows the user to control amr in terms of the following:
1. Forward/Backward (mm)
2. Absolute Heading (deg)
3. say something
4. queueGoal
	* Note that the type is set to "Pickup"
	* Note that the priority is set to 10
5. queuequery
   *  This is to identify the following states:
		* After Goal <- the listen node will be opened
    		* e.g. `[arcl_api_server-1] QueueUpdate: PICKUP6 TM_Goal1' 10 InProgress After Goal "TM_Goal1" "Omron-LD250" 04/14/2025 11:59:23 None None 0`
		* Completed <- can continue
    		* e.g. `[arcl_api_server-1] QueueQuery: PICKUP4 PICKUP100' 10 Completed None Goal "TM_Goal1" "Omron-LD250" 04/14/2025 11:05:53 04/14/2025 11:06:29 "" 0`
6. go <- proceed the first job in the queue immediately
7. stay <- halt for one minute (either [halting the current running job] or [halt for one minute before proceeding with the first job in the queue])


**Example inputs**
1. `command = "move,param1"` 
   `param1 = distance in mm` e.g. `move,100`

2. `command = "setHeading,param1"`
	`param1 = heading in deg` e.g. `setHeading,180`

3. `command = "say"`
4. `param1 = string` e.g. `say,oh my god`

5. `command = "queueGoal,param1,param2"`
   `param1 = goalName` <- must match the Goal Name in Mobile Planner
   `param2 = jobId` <- an identifier for the job
   e.g. `queueGoal TM_Goal1 david_job1`

6. `command = "queuequery,param1"`
   `param1 = goalName`

7. `command = "go"` <- proceed with the first job in the queue immediately

8. `command = "stay"` <- wait one minute (can accumulate) before proceeding

To use the package, run the following in order, parallel to one another:

1. Open a new terminal, run the amr2Arcl connection setup node
`ros2 launch om_aiv_util server.launch.py `
1. Open a new terminal, run the interface node for controlling the amr
`ros2 run amr_arcl_interface server `
1. Open a new terminal, run the client node
`ros2 run amr_arcl_interface client`

## wireless package
The package allows the user to move the TM to a custom joint configuration

**Parameters**
`(command,param1)`

**List of commands**
1. `command = "light, param1"` 
   `param1 = "1"` || `param1 = "0"`

2. Move to a joint configuration in angles
  `command = "movejoint param1"` 
	`param1 = <string>` e.g. `param1 = "home"`

2. Move to (x,y,z) wrt. base
  `command = "move2point param1"`
	`param1 = <string>` e.g. `param1 = "home"`

3. `command = "grip`

4. `command = "release"`

5. `command = "exit"`

Note that you have to add your key-value pair in the dictionary for the command to work, it's located at `Omron_TM_ROS2/src/wireless/src/wireless/server.py`

e.g. `"home": [-90.0, 0.0, 90.0, 0.0, 90.0, 0.0],`

## Camera Code

`ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true pointcloud.enable:=true`
## YOLO code (Note that `cpu` is used here, remove the attribute if one has a gpu)
`ros2 launch yolo_bringup yolov11.launch.py imgsz_height:=480 imgsz_width:=640 input_image_topic:='/camera/camera/color/image_raw' input_depth_topic:='/camera/camera/depth/image_rect_raw' input_depth_info_topic:='/camera/camera/depth/camera_info' device:='cpu' `

## Demo program

The demo program demostrates the abaility of control the AMR and the TM independently in ROS2 without using MobilePlanner

In specific, it demonstrates the following ability:
* Moving AMR forward/backward
* Broadcast message
* setHeading on AMR
* Switch between AMR and TM
* Gripper grip/release

https://youtube.com/shorts/6QjGDZB__Jw?feature=share
[![Watch the video](https://img.youtube.com/vi/6QjGDZB__Jw/0.jpg)](https://www.youtube.com/watch?v=6QjGDZB__Jw)

**Caveats**
* There is **ZERO** error handling
  * All tasks are blocking, and there is no `catch`, the next task will wait indefinitely for the current task.

**How to run the demo program**
1. Open a new terminal, run the amr2Arcl connection setup node
`ros2 launch om_aiv_util server.launch.py `
1. Open a new terminal, run the interface node for controlling the amr
`ros2 run amr_arcl_interface server`
1. Open a new terminal, run the interface node for controlling the TM
`ros2 run tm_driver tm_driver 192.168.1.2`
1. Open a new terminal, run the node for TM-ROS2 communication
`ros2 run wireless server`
1. Open a new terminal, run the node for AMR-ROS2 communication
`ros2 run amr_arcl_interface server`
1. Finally, run the demo_program at a new terminal
`ros2 run demo_package demo_routine`