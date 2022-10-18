#!/bin/bash

PX4FIRMDIR=$1
PX4ROSDIR=$2
CWD=$PWD

if [[ "$(dirname "$(readlink -f "$0")")" != "$PWD" ]]; then
  echo "ERROR: call from script directory."
  exit
fi


if [[ "$2" == "" ]]; then
  echo "USAGE: ./install.sh /path/to/PX4-Autopilot_root /path/to/px4_ros_com_ros2_root."
  exit
fi

echo "installing CMake files.."
cp -f $CWD/CMakeLists.txt $PX4ROSDIR/src/px4_ros_com/
cp -f $CWD/sitl_target.cmake $PX4FIRMDIR/platforms/posix/cmake/


# check where simulation directory is, support old and new location
if [ ! -d "$PX4FIRMDIR/Tools/sitl_gazebo" ] 
then
	echo "installing Iris model.."
	cp -f $CWD/iris.sdf $PX4FIRMDIR/Tools/simulation/gazebo/sitl_gazebo/models/iris/
	echo "installing HCA Airport world.."
	cp -f $CWD/hca_full_setup.world $PX4FIRMDIR/Tools/simulation/gazebo/sitl_gazebo/worlds/
	cp -f $CWD/d4e_HCAairport.world $PX4FIRMDIR/Tools/simulation/gazebo/sitl_gazebo/worlds/
	cp -f $CWD/sitl_targets_gazebo.cmake $PX4FIRMDIR/src/modules/simulation/simulator_mavlink/
else
	echo "installing Iris model.."
	cp -f $CWD/iris.sdf $PX4FIRMDIR/Tools/sitl_gazebo/models/iris/
	echo "installing HCA Airport world.."
	cp -f $CWD/hca_full_setup.world $PX4FIRMDIR/Tools/sitl_gazebo/worlds/
	cp -f $CWD/d4e_HCAairport.world $PX4FIRMDIR/Tools/sitl_gazebo/worlds/
fi


echo "installing Code files.."
cp -f $CWD/offboard_control.cpp $PX4ROSDIR/src/px4_ros_com/src/examples/offboard/
cp -f $CWD/vel_ctrl_vec_pub.cpp $PX4ROSDIR/src/px4_ros_com/src/examples/offboard/
cp -f $CWD/img_proj_depth.py $PX4ROSDIR/src/px4_ros_com/src/examples/offboard/
cp -f $CWD/img_3d_to_2d_proj.cpp $PX4ROSDIR/src/px4_ros_com/src/examples/offboard/
cp -f $CWD/lidar_to_mmwave.cpp $PX4ROSDIR/src/px4_ros_com/src/examples/offboard/

echo "building px4_ros_com_ros2.."
cd $PX4ROSDIR/src/px4_ros_com/scripts/
./build_ros2_workspace.bash
