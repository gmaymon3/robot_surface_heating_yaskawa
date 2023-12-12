## Package Break Down 
There are 4 packages needed. 
- thermal_camera
- robot_move_scripts
- motoman
- heater_moveit_config

The thermal camera contains drivers and scripts for the thermal camera 
The robot move scripts contains the launch files and python move scripts for running the robot 
The motoman conatins the gp12 config and drivers needed for the yaskawa 
the heater moveit config contains the files generated for the rviz config for the yaskawa 

## For Coldest Node Movement

# To open Rviz
cd ~/Documents/GitHub/robot_surface_heating <br>
source devel/setup.bash <br>
roslaunch robot_move_scripts py_launch_heater.launch <br>

# To Run The Thermal Camera
Open new tab <br>
sudo -s <br>
source devel/setup.bash <br>
rosrun thermal_camera cold-node-transform-grid-uvc-radiometry-lab.py <br>

# To Set Up Robot Arm Movement
Open new tab <br>
source devel/setup.bash <br>
rosrun robot_move_scripts heat_to_Xdeg_cold_node_move_group_python_interface.py <br>


# NOTE: YOU MAY NEED TO UNPLUG AND PLUG THE CAMERA IF THERE IS A UVC ERROR


