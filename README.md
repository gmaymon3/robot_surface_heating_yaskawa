# To open Rviz
cd ~/Documents/GitHub/robot_surface_heating <br>
source devel/setup.bash <br>
roslaunch movement py_launch_heater.launch <br>


# To Set Up Robot Arm Movement
Open new tab <br>
source devel/setup.bash <br>
rosrun moveit_tutorials move_group_python_interface_tutorial.py <br>


# To Run The Thermal Camera
Open new tab <br>
sudo -s <br>
source devel/setup.bash <br>
rosrun test_exec transform-grid-uvc-radiometry-lab.py <br>


# NOTE: 
YOU MAY NEED TO UNPLUG AND PLUG THE CAMERA IF THERE IS A UVC ERROR


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


