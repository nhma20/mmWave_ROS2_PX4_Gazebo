# mmWave_ROS2_PX4_Gazebo
Simulate mmWave radar based drone control i Gazebo with ROS2 and PX4


### Install ROS2
https://docs.ros.org/en/foxy/Installation.html

### Install Gazebo
http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros

### Install PX4 (ROS2, RTPS, SITL, Gazebo)
https://docs.px4.io/master/en/ros/ros2_comm.html
(follow https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html#gazebo-jmavsim-and-nuttx-pixhawk-targets during sanity check)

### Install Gazebo HCA worlds/models
```sh
~/Drones4Energy_SDU_Only_code-master/Tools/simulationAssets$ ./installAssets.sh ~/PX4-Autopilot/
```
Add new worlds/models to ~/PX4-Autopilot/platforms/posix/cmake/sitl_target.cmake

### Install QGroundControl
https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html#ubuntu

### Test if all works
(https://docs.px4.io/master/en/ros/ros2_comm.html#sanity-check-the-installation)
1. Open a new terminal in the root of the PX4 Autopilot project, and then start a PX4 Gazebo simulation using:
   syntax: make <target> <simulator>_<vehiclemodel>__<world>
   ```sh
   make px4_sitl_rtps gazebo_iris_opt_flow__d4e_HCAairport
   ```
   Should make and open PX4 in same console, as well as a Gazebo window with chosen model and world
  
2. On a new terminal, source the ROS 2 workspace and then start the micrortps_agent daemon with UDP as the transport protocol:
   ```sh 
   $ source ~/px4_ros_com_ros2/install/setup.bash
   $ micrortps_agent -t UDP
   ```
  
3. On the original terminal (PX4 console) start the micrortps_client daemon with UDP:
   ```sh
   pxh> micrortps_client start -t UDP
   ```
  
4. Open a new terminal and start a "listener" using the provided launch file:
   ```sh 
   $ source ~/px4_ros_com_ros2/install/setup.bash
   $ ros2 launch px4_ros_com sensor_combined_listener.launch.py
   ```
  
