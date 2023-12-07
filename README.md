# To open Rviz
cd ~/Documents/GitHub/robot_surface_heating <br>
source devel/setup.bash
roslaunch movement py_launch_heater.launch 


# To Set Up Robot Arm Movement
Open new tab
source devel/setup.bash
rosrun moveit_tutorials move_group_python_interface_tutorial.py


# To Run The Thermal Camera
Open new tab
sudo -s
source devel/setup.bash
rosrun test_exec transform-grid-uvc-radiometry-lab.py


# NOTE: 
YOU MAY NEED TO UNPLUG AND PLUG THE CAMERA IF THERE IS A UVC ERROR


