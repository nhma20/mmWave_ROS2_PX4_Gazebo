#!/bin/bash

PX4FIRMDIR=$1
PX4ROSDIR=$2
CWD=$PWD
if [[ "$2" == "" ]]; then
  echo "USAGE: ./install_files.sh /path/to/PX4-Autopilot_root /path/to/px4_ros_com_ros2_root."
  exit
fi

echo "installing CMake file.."
cp -i $CWD/CMakeLists.txt $PX4ROSDIR/src/px4_ros_com/ -v

echo "installing Iris model.."
cp -i $CWD/iris.sdf $PX4FIRMDIR/Tools/sitl_gazebo/models/iris/ -v

echo "installing HCA Airport world.."
cp -i $CWD/hca_full_setup.world $PX4FIRMDIR/Tools/sitl_gazebo/worlds/ -v

echo "installing Code files.."
cp -i $CWD/lidar_test_sub.cpp $PX4ROSDIR/src/px4_ros_com/src/examples/offboard/ -v
cp -i $CWD/offboard_control.cpp $PX4ROSDIR/src/px4_ros_com/src/examples/offboard/ -v
cp -i $CWD/vel_ctrl_vec_pub.cpp $PX4ROSDIR/src/px4_ros_com/src/examples/offboard/ -v
cp -i $CWD/img_proj_depth.py $PX4ROSDIR/src/px4_ros_com/src/examples/offboard/ -v

#echo "building px4_ros_com_ros2.."
#cd $PX4ROSDIR/src/px4_ros_com/scripts/
#./build_ros2_workspace.bash
