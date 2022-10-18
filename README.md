# mmWave_ROS2_PX4_Gazebo
Simulate mmWave radar based drone control in Gazebo with ROS2 and PX4

### Prerequisites
Tested with:
- Ubuntu 20.04.3 LTS
- ROS2 Foxy
- Gazebo 11.9.0
- px4_ros_com master branch 18th October 2022
- px4_msgs master branch 18th October 2022
- PX4 Autopilot master branch 18th October 2022


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
Install gazebo ROS2 package:
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
   
### Install repository files
Oververwrites existing files. Will also download and install powerline test setup worlds and models from:
https://drive.google.com/file/d/1mqL6CPEd5GOK2gtuCNvOiPhDQ3f8qfzR

```sh
cd ~/mmWave_ROS2_PX4_Gazebo/
chmod +x ./install.sh
```
Execute install script (from the script directory). If PX4 and px4_ros_com_ros2 installed in home directory:
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
   make px4_sitl_rtps gazebo_iris__hca_full_pylon_setup
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


### Launch all
https://docs.px4.io/master/en/ros/ros2_offboard_control.html
https://github.com/PX4/px4_ros_com/blob/master/src/examples/offboard/offboard_control.cpp

0. If offboard_control.cpp or other files have been edited, re-run ```install.sh``` script (add new files to script and CMakeLists.txt):
   ```sh
   cd ~/mmWave_ROS2_PX4_Gazebo/
   ( chmod +x ./install.sh )
   ./install.sh
   ```
    If same PX4 and px4_ros_com_ros2 roots:
    ```
    ./install.sh ~/PX4-Autopilot/ ~/px4_ros_com_ros2/
    ```
1. Launch PX4 SITL:
   ```sh
    cd ~/PX4-Autopilot/ 
    make px4_sitl_rtps gazebo_iris__hca_full_pylon_setup
   ```
   Without Gazebo GUI:
   ```sh
    HEADLESS=1 make px4_sitl_rtps gazebo_iris__hca_full_pylon_setup
   ```
   Without drone following:
   ```sh
    PX4_NO_FOLLOW_MODE=1 make px4_sitl_rtps gazebo_iris__hca_full_pylon_setup
   ```
   After PX4 SITL fully launched, might need to manually start microRTPS client in same terminal:
   ```sh
    micrortps_client start -t UDP
   ```
   Will fail and return -1 if already running.
2. Open QGroundControl   
3. In a new terminal start microRTPS agent and offboard control:
   ```sh 
   source ~/px4_ros_com_ros2/install/setup.bash
   micrortps_agent start -t UDP & ros2 run px4_ros_com offboard_control 
   ```
4. In another terminal, start the velocity vector advertiser, lidar to mmwave converter, and 3d to 2d projection nodes:
   ```sh 
   source ~/px4_ros_com_ros2/install/setup.bash
   ros2 launch ~/mmWave_ROS2_PX4_Gazebo/launch/simulate_pointcloud_control_launch.py 
   ```
5. Simulated drone in Gazebo should arm and takeoff. May need to restart ```vel_ctrl_vec_pub``` and ```offboard_control``` ros2 runs.

6. Visualize simulated data in rviz2:
   ```sh 
   rviz2 ~/mmWave_ROS2_PX4_Gazebo/3d_and_2d_pointcloud_rgb.rviz 
   ```


### MISC
1. Trajectory setpoint message:
   https://github.com/PX4/px4_msgs/blob/ros2/msg/TrajectorySetpoint.msg
2. Changed parameters (to fix "Failsafe enabled: No manual control stick input" warning and not taking off):
   pxh> param set NAV_RCL_ACT 0
   pxh> param set COM_RCL_EXCEPT 4

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
13. iris.sdf (or other models) can be edited to include sensors, like 2D lidar.
14. Display simulated camera feed either with rviz2 or
   ```sh 
   source ~/px4_ros_com_ros2/install/setup.bash
   ros2 run image_tools showimage image:=/cable_camera/image_raw
   ```
14. Add new worlds/models to ~/PX4-Autopilot/platforms/posix/cmake/sitl_target.cmake (Oscar's worlds/models from https://gitlab.drones4energy.dk/obs/Drones4Energy_SDU_Only_code/-/tree/iROS2021/Tools/simulationAssets)
15. See local packages, and msgs, with: ```ros2 interface packages``` and e.g. ```ros2 interface package px4_msgs```
16. Camera intrinsic parameters for setting a custom perspective projection matrix (cannot be used with WideAngleCamera since this class uses image stitching from 6 different cameras for achieving a wide field of view). The focal lengths can be computed using focal_length_in_pixels = (image_width_in_pixels * 0.5) / tan(field_of_view_in_degrees * 0.5 * PI/180) (http://sdformat.org/spec?ver=1.7&elem=sensor#lens_intrinsics)
17. Drone spawn coordinates set in ~/PX4-Autopilot/Tools/sitl_run.sh ?
18. ```*** No rule to make target '/opt/ros/foxy/lib/libfastrtps.so.2.0.2', needed by 'libpx4_msgs__rosidl_typesupport_fastrtps_cpp.so'.  Stop.```
Fixed by renaming closest libfastrtps.so.x.y.z to libfastrtps.so.2.0.2.
19. Dependency errors with PX4, like ```ninja: error: '/usr/lib/x86_64-linux-gnu/libsdformat9.so.9.6.1', needed by 'libmav_msgs.so', missing and no known rule to make it``` may be solved by a PX4 reinstall (remember worlds, models, cmake files etc. must be also be reinstalled into new PX4).
20. If drone enters failsafe when starting offboard_control, ```param set COM_RCL_EXCEPT 4``` in the PX4 console may solve this. Else, try manually publish few setpoints to fmu/manual_control_setpoint/in and then start offboard mode.
21. Showing videos in readme: Just drag and drop your image/video from your local pc to github readme in editable mode.
22. If gradle not working, might have to downgrade Java (JDK) to 11: https://askubuntu.com/questions/1133216/downgrading-java-11-to-java-8
23. May have to set unused non-velocity parameters to NAN in TrajectorySetpoint message: https://discuss.px4.io/t/offboard-control-using-ros2-how-to-achieve-velocity-control/21875




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
10. :green_circle: Visualize depth data in camera feed

![Alt text](https://github.com/nhma20/mmWave_ROS2_PX4_Gazebo/blob/main/Pictures/Screenshot_from_2021-07-05_20-52-58.png?raw=true)

12. :green_circle: Investigate occasional drone control loss
13. :green_circle: Make module that turns 2d lidar data into noisy pointcloud to prepare for mmwave integration
14. :yellow_circle: Tracking of points in pointcloud (kalman?)
15. :yellow_circle: Implement cable detection AI to filter depth data and align drone yaw wrt. cable

![Alt text](https://github.com/nhma20/mmWave_ROS2_PX4_Gazebo/blob/main/Pictures/Screenshot_from_2021-07-06_09-54-53.png?raw=true)

https://user-images.githubusercontent.com/76950970/142616665-0cb08003-9355-4eed-a658-7d900c9f66fb.mp4




