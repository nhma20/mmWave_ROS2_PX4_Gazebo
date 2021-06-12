# mmWave_ROS2_PX4_Gazebo
Simulate mmWave radar based drone control i Gazebo with ROS2 and PX4

### Prerequisites
Tested with:
- Ubuntu 20.04.2 LTS
- ROS2 Foxy
- Gazebo 11.5.1
- PX4 Autopilot v1.11.3


### Install ROS2
https://docs.ros.org/en/foxy/Installation.html

### Install Gazebo
http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros

### Install PX4 (ROS2, RTPS, SITL, Gazebo)
https://docs.px4.io/master/en/ros/ros2_comm.html

(follow https://docs.px4.io/master/en/dev_setup/dev_env_linux_ubuntu.html#gazebo-jmavsim-and-nuttx-pixhawk-targets during sanity check)

### Install Gazebo HCA worlds/models
```sh
$ ~/Drones4Energy_SDU_Only_code-master/Tools/simulationAssets$ ./installAssets.sh ~/PX4-Autopilot/
```

or from ```~/hcaa_pylon_setup/``` copy ```hcaa_pylon_setup``` folder to ```~/PX4-Autopilot/Tools/sitl_gazebo/models/```, and copy ```hca_full_setup.world``` to ```~/PX4-Autopilot/Tools/sitl_gazebo/worlds/```

Add new worlds/models to ~/PX4-Autopilot/platforms/posix/cmake/sitl_target.cmake

### Install QGroundControl
https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html#ubuntu

### Test if all works
(https://docs.px4.io/master/en/ros/ros2_comm.html#sanity-check-the-installation)
1. Open a new terminal in the root of the PX4 Autopilot project, and then start a PX4 Gazebo simulation using:
   ```sh
   $ make px4_sitl_rtps gazebo_iris__hca_full_setup
   ```
   or, for single cable:
    ```sh
   $ make px4_sitl_rtps gazebo_iris__d4s_HCAairport
   ```
   
   (syntax: ```make <target> <simulator>_<vehiclemodel>__<world> ```, prepend ```HEADLESS=1``` to launch without GUI)

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
   
5. Optionally, open QGroundControl which will connect with PX4. From here it is possible to set waypoints and execute missions.


### ROS2 offboard control
https://docs.px4.io/master/en/ros/ros2_offboard_control.html
https://github.com/PX4/px4_ros_com/blob/master/src/examples/offboard/offboard_control.cpp

6. Edit offboard_control.cpp to include wanted behavior, add any new files to ~/px4_ros_com_ros2/src/px4_ros_com/CMakeLists.txt.
7. Build colcon workspace with script:
   ```sh
   $ cd ~/px4_ros_com_ros2/src/px4_ros_com/scripts
   $ ./build_ros2_workspace.bash
   ```
8. After building the colcon workspace, and after starting PX4 SITL and both the microRTPS bridge client and agent, in a new terminal start offboard control:
   ```sh 
   $ source ~/px4_ros_com_ros2/install/setup.bash
   $ ros2 run px4_ros_com offboard_control
   ```
9. In another terminal, start the velocity vector advertiser:
   ```sh 
   $ source ~/px4_ros_com_ros2/install/setup.bash
   $ ros2 run px4_ros_com vel_ctrl_vec_pub
   ```
10. Simulated drone in Gazebo should arm, takeoff, and fly in a square pattern. May need to restart ```vel_ctrl_vec_pub``` and ```offboard_control``` ros2 runs.

11. ~/PX4-Autopilot/Tools/sitl_gazebo/models/iris/iris.sdf (or other models) can be edited to include sensors, like 2D lidar.


### MISC
1. Trajectory setpoint message:
   https://github.com/PX4/px4_msgs/blob/ros2/msg/TrajectorySetpoint.msg
2. Disabled param:
   pxh> param set NAV_RCL_ACT 0

   NAV_RCL_ACT: curr: 2 -> new: 0
3. Local positioning?
   https://github.com/PX4/px4_msgs/blob/ros2/msg/VehicleLocalPositionSetpoint.msg
   
4. Add any new ROS2 files to ~/px4_ros_com_ros2/src/px4_ros_com/CMakeLists.txt

5. Check armed? https://github.com/PX4/px4_msgs/blob/ros2/msg/ActuatorArmed.msg

6. libignition-common3 error (after software update?) - Copy existing file and rename to match missing file
7. Difference between ```make px4_sitl_rtps gazebo``` (works) and ```make px4_sitl_rtps gazebo_iris__empty``` (does not work) ?
8. If gazebo does not open, try running ```gazebo --verbose``` to troubleshoot. ```killall gzserver``` should kill any gazebo instances. Restart PC if all else fails.
9. inlude both iris.sdf and iris.sdf.jinja?
10. Implemented laser scanner with Gazebo and ROS2 https://github.com/chapulina/dolly
11. Make custom sensor plugin http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5
12. In ~/px4_ros_com_ros2/src/px4_ros_com/CMakeLists.txt add ```sensor_msgs``` under ```ament_target_dependencies```
13. After running ```./build_ros2_workspace``` restart all affected executables (micrortps_agent, offboard_control, vel_vec_ctrl_pub). Gazebo PX4 SITL can be left running.
14. Display simulated camera feed either with rviz2 or
   ```sh 
   $ source ~/px4_ros_com_ros2/install/setup.bash
   $ ros2 run image_tools showimage image:=/cable_camera/image_raw
   ```

### TODO
0. :green_circle: Install tools 
1. :green_circle: Figure out how to control drone via offboard_control.cpp 
2. :green_circle: Make ROS2 advertiser that generates control input for offboard_control.cpp for more advanced control
3. :green_circle: Figure out how to use simulated depth sensors
4. :green_circle: Implement depth data into ROS2 advertiser for even more advanced control
5. :green_circle: Control drone towards overhead cable

![Alt text](https://github.com/nhma20/mmWave_ROS2_PX4_Gazebo/blob/main/Pictures/Screenshot_from_2021-06-08_15-17-35.png?raw=true)

7. :yellow_circle: More tightly integrate with PX4 to optimize control based on e.g. drone state
   - get pose of drone to mitigate sideways motion when rotated around x or y.
   - use GPS positioning to counteract drift
9. :green_circle: Use drone mounted simulated camera to get images of overhead cable 
10. :yellow_circle: Implement cable detection AI to filter depth data and align drone yaw wrt. cable
11. :yellow_circle: Edit sensor plugin to act like mmWave sensor



