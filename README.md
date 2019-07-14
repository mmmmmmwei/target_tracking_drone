# Target detection and tracking for drone

Target (circle) Tracking with simulated AR drone in ROS by using Gazebo simulator.

***Reference: "Visual control of the Parrot drone with OpenCV, ROS and Gazebo Simulator " by Artur Banach
http://repositorio.upct.es/bitstream/handle/10317/5442/pfc6362.pdf?sequence=1&isAllowed=y

OpenCV + ardrone_autonomy + Gazebo 7
Ubuntu 1604, ROS Kinetic

tum_simulator-implementation of Gazebo Simulator and ardrone_autonomy
http://wiki.ros.org/tum_simulator

ardrone_autonomy-ROS driver for Parrot AR-Drone
https://ardrone-autonomy.readthedocs.io/en/latest/

cv_camera-access USB/laptop built-in webcam
http://wiki.ros.org/cv_camera

----------------------------------------------------------------------------------------------------------

1) Install tum_simulator
https://github.com/angelsantamaria/tum_simulator

Open a terminal, enter following commands.
$ mkdir ~/ardrone_simulator/src
$ cd ~/ardrone_simulator/src
$ catkin_init_workplace
$ git clone https://github.com/angelsantamaria/tum_simulator.git
$ cd ..
$ catkin build


2) Install ardrone_autonomy
http://sites.bu.edu/uav/extra/installing-ros-kinetic-and-connection-to-parrot-drone/

$ cd ~/ardrone_simulator/src
$ git clone https://github.com/AutonomyLab/ardrone_autonomy
$ cd ..
$ rosdep install --from-paths src --ignore-src
$ catkin build


3) Install cv_camera
http://wiki.ros.org/cv_camera

$ sudo apt-get install ros-kinetic-cv-camera


4) Create ROS node
http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage

$ cd ~/ardrone_simulator/src
$ catkin_create_pkg drone cv_bridge roscpp sensor_msgs ardrone_autonomy geometry_msgs geographic_msgs image_transport std_msgs
$ Head to https://drive.google.com/open?id=1dewBVSzW2QFMgxMJu2oSVPVgSGWnL2W9, modify drone/src/main.cpp and drone/CMakeLists.txt
**(main.cpp code is modified from "visual control of the Parrot drone with OpenCV, Ros and Gazebo Simulator" by Artur Banach, refer to http://repositorio.upct.es/bitstream/handle/10317/5442/pfc6362.pdf?sequence=1&isAllowed=y)
**The function of OpenCV used in the coding can refer to this visual control of the Parrot drone with OpenCV, Ros and Gazebo Simulator, those are all explained.
$ cd ~/ardrone_simulator
$ catkin build


4) If no error, run the node
Open 4 terminal

Terminal 1:
$ roscore

Terminal 2:
$ rosparam set cv_camera/device_id 0 
**(choose your own devide_id: 0=build-in webcam, 1=USB webcam 1, ..)
$ rosrun cv_camera cv_camera_node

Terminal 3:
$ cd ardrone_simulator
$ source devel/setup.bash
$ roslaunch cvg_sim_gazebo ardrone_testworld.launch

Terminal 4:
$ cd ardrone_simulator
$ source devel/setup.bash
$ rosrun drone main
**(drone is package name, main is node name)
