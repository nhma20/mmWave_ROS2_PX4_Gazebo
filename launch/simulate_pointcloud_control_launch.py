from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros_com',
            #namespace='turtlesim1',
            executable='vel_ctrl_vec_pub',
            name='vel_ctrl_vec_pub'
        ),
        Node(
            package='px4_ros_com',
            #namespace='turtlesim2',
            executable='img_3d_to_2d_proj',
            name='img_3d_to_2d_proj'
        ),
        Node(
            package='px4_ros_com',
            #namespace='turtlesim2',
            executable='lidar_to_mmwave',
            name='lidar_to_mmwave'
        )
    ])
