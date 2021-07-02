# mmWave_ROS2_PX4_Gazebo
Simulate mmWave radar based drone control i Gazebo with ROS2 and PX4

### Prerequisites
Tested with:
- Ubuntu 20.04.2 LTS
- ROS2 Foxy
- Gazebo 11.5.1
- PX4 Autopilot v1.11.3


### Install ROS2
https://docs.ros.org/en/foxy/Installation.html or https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
- Setup Sources:
```sh
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```
- Install ROS 2 packages:
```sh
sudo apt update
sudo apt install ros-foxy-desktop
sudo apt install python3-colcon-common-extensions
```


### Install Gazebo
http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros and http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install
- Default Gazebo installation:
```sh
cd ~
curl -sSL http://get.gazebosim.org | sh
```
- Install gazebo_ros_pkgs
```sh
sudo apt install ros-foxy-gazebo-ros-pkgs
```

### Install PX4 (ROS2, RTPS, SITL, Gazebo)
https://docs.px4.io/master/en/ros/ros2_comm.html
- Foonathan memory:
```sh
cd ~
git clone https://github.com/eProsima/foonathan_memory_vendor.git
cd foonathan_memory_vendor
mkdir build && cd build
cmake ..
sudo cmake --build . --target install
```
- Fast-RTPS (DDS)
```sh
cd ~
git clone --recursive https://github.com/eProsima/Fast-DDS.git -b v2.0.0 ~/FastDDS-2.0.0
cd ~/FastDDS-2.0.0
mkdir build && cd build
cmake -DTHIRDPARTY=ON -DSECURITY=ON ..
make -j$(nproc --all)
sudo make install
```
- Fast-RTPS-Gen
```sh
cd ~
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/Fast-RTPS-Gen \
    && cd ~/Fast-RTPS-Gen \
    && ./gradlew assemble \
    && sudo ./gradlew install
```
- Check install with ```which fastrtpsgen```
- Download PX4 Source code and run ```ubuntu.sh``` with no arguments:
```sh
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```
- Relogin or reboot computer before attempting to build NuttX targets
- Build ROS 2 Workspace
```sh
cd ~
mkdir -p ~/px4_ros_com_ros2/src
```
- Clone ROS 2 bridge packages ```px4_ros_com``` ```px4_msgs``` (default master branch)
```sh
cd ~
git clone https://github.com/PX4/px4_ros_com.git ~/px4_ros_com_ros2/src/px4_ros_com
git clone https://github.com/PX4/px4_msgs.git ~/px4_ros_com_ros2/src/px4_msgs
```
- Use script to build workspace including two aforementioned packages:
```sh
cd ~/px4_ros_com_ros2/src/px4_ros_com/scripts
./build_ros2_workspace.bash
```


### Install QGroundControl
https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html#ubuntu
```sh
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
```
- Download: https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage
```sh
cd ~/Downloads/
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage  (or double click)
   ```
   
### Install Gazebo HCA worlds/models (optional)
https://gitlab.drones4energy.dk/obs/Drones4Energy_SDU_Only_code/-/tree/iROS2021/Tools/simulationAssets
```sh
cd ~/Downloads/Drones4Energy_SDU_Only_code-iROS2021/Tools/simulationAssets
./installAssets.sh ~/PX4-Autopilot/
```

or from ```~/hcaa_pylon_setup/``` copy ```hcaa_pylon_setup``` folder to ```~/PX4-Autopilot/Tools/sitl_gazebo/models/```, and copy ```hca_full_setup.world``` to ```~/PX4-Autopilot/Tools/sitl_gazebo/worlds/```
   
   
### Install repository files
(overwrites existing files)
```sh
cd ~/mmWave_ROS2_PX4_Gazebo/
chmod +x ./install.sh
```
Execute install script (from the script directory). If same PX4 and px4_ros_com_ros2 roots:
```
./install.sh ~/PX4-Autopilot/ ~/px4_ros_com_ros2/
```

### Test if all works
(https://docs.px4.io/master/en/ros/ros2_comm.html#sanity-check-the-installation)
1. Open a new terminal in the root of the PX4 Autopilot project, and then start a PX4 Gazebo simulation using:
   ```sh
   cd ~/PX4-Autopilot/
   ```
   ```sh
   make px4_sitl_rtps gazebo_iris__hca_full_setup
   ```
   or, for single cable:
    ```sh
   make px4_sitl_rtps gazebo_iris__d4e_HCAairport
   ```
   or, for empty world (if no additional worlds/models installed):
   ```sh
   make px4_sitl_rtps gazebo
   ```
   
   (syntax: ```make <target> <simulator>_<vehiclemodel>__<world> ```
   prepend ```HEADLESS=1``` to launch without GUI
   prepend ```PX4_NO_FOLLOW_MODE=1``` to launch without following drone)

   Should make and open PX4 in same console, as well as a Gazebo window with chosen model and world
  
2. On a new terminal, source the ROS 2 workspace and then start the micrortps_agent daemon with UDP as the transport protocol:
   ```sh 
   source ~/px4_ros_com_ros2/install/setup.bash
   micrortps_agent -t UDP
   ```
  
3. On the original terminal (PX4 console) start the micrortps_client daemon with UDP:
   ```sh
   pxh> micrortps_client start -t UDP
   ```
   (may already be running if following message is generated: ```INFO  [micrortps_client] Already running
Command 'micrortps_client' failed, returned -1.```)
  
4. Open a new terminal and start a "listener" using the provided launch file:
   ```sh 
   source ~/px4_ros_com_ros2/install/setup.bash
   ros2 launch px4_ros_com sensor_combined_listener.launch.py
   ```
   
5. Optionally, open QGroundControl which will connect with PX4. From here it is possible to set waypoints and execute missions.


### ROS2 offboard control
https://docs.px4.io/master/en/ros/ros2_offboard_control.html
https://github.com/PX4/px4_ros_com/blob/master/src/examples/offboard/offboard_control.cpp

6. Edit offboard_control.cpp to include wanted behavior, add any new files to ~/px4_ros_com_ros2/src/px4_ros_com/CMakeLists.txt.
7. Build colcon workspace with script:
   ```sh
   cd ~/px4_ros_com_ros2/src/px4_ros_com/scripts
   ./build_ros2_workspace.bash
   ```
8. After building the colcon workspace, and after starting PX4 SITL and both the microRTPS bridge client and agent, in a new terminal start offboard control:
   ```sh 
   source ~/px4_ros_com_ros2/install/setup.bash
   ros2 run px4_ros_com offboard_control
   ```
9. In another terminal, start the velocity vector advertiser:
   ```sh 
   source ~/px4_ros_com_ros2/install/setup.bash
   ros2 run px4_ros_com vel_ctrl_vec_pub
   ```
10. Simulated drone in Gazebo should arm and takeoff. May need to restart ```vel_ctrl_vec_pub``` and ```offboard_control``` ros2 runs.

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

5. Check if drone armed? https://github.com/PX4/px4_msgs/blob/ros2/msg/ActuatorArmed.msg

6. libignition-common3 error (after software update?) - Copy existing file and rename to match missing file
7. If gazebo does not open, try running ```gazebo --verbose``` to troubleshoot. ```killall gzserver``` should kill any gazebo instances. Restart PC if all else fails.
8. inlude both iris.sdf and iris.sdf.jinja?
9. Implemented laser scanner with Gazebo and ROS2 https://github.com/chapulina/dolly
10. Make custom sensor plugin http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5
11. In ~/px4_ros_com_ros2/src/px4_ros_com/CMakeLists.txt add ```sensor_msgs``` under ```ament_target_dependencies```
12. After running ```./build_ros2_workspace``` restart all affected executables (micrortps_agent, offboard_control, vel_vec_ctrl_pub). Gazebo PX4 SITL can be left running.
13. Display simulated camera feed either with rviz2 or
   ```sh 
   source ~/px4_ros_com_ros2/install/setup.bash
   ros2 run image_tools showimage image:=/cable_camera/image_raw
   ```
14. Add new worlds/models to ~/PX4-Autopilot/platforms/posix/cmake/sitl_target.cmake (Oscar's worlds/models from https://gitlab.drones4energy.dk/obs/Drones4Energy_SDU_Only_code/-/tree/iROS2021/Tools/simulationAssets)
15. See local packages, and msgs, with: ```ros2 interface packages``` and e.g. ```ros2 interface package px4_msgs```
16. Camera intrinsic parameters for setting a custom perspective projection matrix (cannot be used with WideAngleCamera since this class uses image stitching from 6 different cameras for achieving a wide field of view). The focal lengths can be computed using focal_length_in_pixels = (image_width_in_pixels * 0.5) / tan(field_of_view_in_degrees * 0.5 * PI/180) (http://sdformat.org/spec?ver=1.7&elem=sensor#lens_intrinsics)



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
10. :green_circle: Visualize depth data in camera feed (nearest point, controlling towards)

![Alt text](https://github.com/nhma20/mmWave_ROS2_PX4_Gazebo/blob/main/Pictures/Screenshot_from_2021-06-30_17-39-15.png?raw=true)

12. :green_circle: Investigate occasional drone control loss
13. :yellow_circle: Make module that turns 2d lidar data into pointcloud to prepare for mmwave integration
14. :yellow_circle: Implement cable detection AI to filter depth data and align drone yaw wrt. cable
15. :yellow_circle: Edit sensor plugin to act like mmWave sensor



