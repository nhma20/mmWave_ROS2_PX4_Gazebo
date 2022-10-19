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
cp -f $CWD/cmake/CMakeLists.txt $PX4ROSDIR/src/px4_ros_com/
cp -f $CWD/cmake/sitl_target.cmake $PX4FIRMDIR/platforms/posix/cmake/


if [ ! -d "$CWD/hca_models_and_worlds" ] 
then
	echo "downloading models and worlds from: https://drive.google.com/file/d/1fe4J8eM5u7MvtayruwMDewYuuZDvQ5bX"
	# this is fragile, will probably break at some point
	# direct link is: https://drive.google.com/file/d/1fe4J8eM5u7MvtayruwMDewYuuZDvQ5bX
	fileid="1fe4J8eM5u7MvtayruwMDewYuuZDvQ5bX"
	filename="hca_models_and_worlds.zip"
	html=`curl -c ./cookie -s -L "https://drive.google.com/uc?export=download&id=${fileid}"`
	curl -Lb ./cookie "https://drive.google.com/uc?export=download&`echo ${html}|grep -Po '(confirm=[a-zA-Z0-9\-_]+)'`&id=${fileid}" -o ${filename}
	unzip hca_models_and_worlds.zip 
	rm hca_models_and_worlds.zip
else
	echo "hca_models_and_worlds directory already exists, using models and worlds from there" 
fi


# check where simulation directory is, support old and new location
if [ ! -d "$PX4FIRMDIR/Tools/sitl_gazebo" ] 
then
	echo "installing models.."
	cp -f -r $CWD/hca_models_and_worlds/models/* $PX4FIRMDIR/Tools/simulation/gazebo/sitl_gazebo/models/ -v
	echo "installing worlds.."
	cp -f -r $CWD/hca_models_and_worlds/worlds/* $PX4FIRMDIR/Tools/simulation/gazebo/sitl_gazebo/worlds/ -v
	cp -f -r $CWD/hca_models_and_worlds/new_iris/* $PX4FIRMDIR/Tools/simulation/gazebo/sitl_gazebo/models/iris/ -v
	cp -f $CWD/cmake/sitl_targets_gazebo.cmake $PX4FIRMDIR/src/modules/simulation/simulator_mavlink/
else
	echo "installing models.."
	cp -f -r $CWD/hca_models_and_worlds/models/* $PX4FIRMDIR/Tools/sitl_gazebo/models/
	cp -f -r $CWD/hca_models_and_worlds/old_iris/* $PX4FIRMDIR/Tools/sitl_gazebo/models/iris/
	echo "installing worlds.."
	cp -f -r $CWD/hca_models_and_worlds/worlds/* $PX4FIRMDIR/Tools/sitl_gazebo/worlds/
fi


echo "installing source files.."
cp -f $CWD/src/offboard_control.cpp $PX4ROSDIR/src/px4_ros_com/src/examples/offboard/
cp -f $CWD/src/vel_ctrl_vec_pub.cpp $PX4ROSDIR/src/px4_ros_com/src/examples/offboard/
cp -f $CWD/src/img_proj_depth.py $PX4ROSDIR/src/px4_ros_com/src/examples/offboard/
cp -f $CWD/src/img_3d_to_2d_proj.cpp $PX4ROSDIR/src/px4_ros_com/src/examples/offboard/
cp -f $CWD/src/lidar_to_mmwave.cpp $PX4ROSDIR/src/px4_ros_com/src/examples/offboard/

echo "building px4_ros_com_ros2.."
cd $PX4ROSDIR/src/px4_ros_com/scripts/
./build_ros2_workspace.bash

echo "done"
