import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Setup project paths
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_amr_sim = get_package_share_directory('amr_sim_pkg')
    robot_file = os.path.join(pkg_amr_sim, 'worlds', 'amr.sdf')
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {robot_file}'}.items(),
    )



    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            # Odom is bridged as a topic for Nav2, but TF handles the transform
            '/model/vehicle_blue/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            # TF bridge (carries odom->base_link)
            # '/model/vehicle_blue/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/world/car_world/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '--ros-args',
            '-p',
            'use_sim_time:=false',
        ],
        remappings=[
            ('/lidar', '/scan'),
            ('/model/vehicle_blue/odometry', '/odom'),
            # ('/model/vehicle_blue/tf', '/tf'),
            ('/world/car_world/clock', '/clock')
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    lidar_frame_match = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'vehicle_blue/lidar/gpu_lidar'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    imu_frame_match = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'imu_link', 'vehicle_blue/base_link/imu_sensor'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        gz_sim,
        bridge,
        lidar_frame_match,
        imu_frame_match,
    ])
