# MoveIt Demo
Repo for the code we will use in our ROS demonstration.

Steps to run the demo:
* To follow along with the normal MoveIt demo:
* Have ROS Noetic installed
* Install MoveIt - Binary install
* Their demo uses a panda manipulator to run through all of the commands
* For our demo we changed it to a UR3 manipulator
* Download the config file in your source directory with: git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git
* Add our .py script to: /home/’UserName’/ws_moveit/devel/lib/moveit_tutorials/doc/move_group_python_interface/scripts
* Make the script executable and add it to the Cmakelists.txt: tutorial here 
* Catkin Build and source the workspace:
  * source ~/ws_moveit/devel/setup.bash
* Ready to go
