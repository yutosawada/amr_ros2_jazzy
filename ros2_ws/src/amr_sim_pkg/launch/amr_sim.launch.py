import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Setup project paths
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_amr_sim = get_package_share_directory('amr_sim_pkg')
    sdf_file = os.path.join(pkg_amr_sim, 'worlds', 'building_robot.sdf')

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {sdf_file}'}.items(),
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
        remappings=[
            ('/lidar', '/scan')
        ],
        output='screen'
    )

    # Static Transform Publisher for LiDAR
    # Links the Gazebo frame (vehicle_blue/lidar/gpu_lidar) to the URDF frame (lidar_link)
    lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'vehicle_blue/lidar/gpu_lidar'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        bridge,
        lidar_tf,
    ])
