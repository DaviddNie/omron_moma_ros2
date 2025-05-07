# MOMA PROJECT

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
  - ros2 launch tm_12s_moveit_config tm12s_run_move_group.launch.py 192.168.1.2
  - Note that this will run the tm_driver as well, so currently will only work wired. Will look to splitting the functionality soon
