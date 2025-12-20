import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_amr_sim = get_package_share_directory('amr_sim_pkg')
    pkg_amr_description = get_package_share_directory('amr_description')
    pkg_amr_bringup = get_package_share_directory('amr_bringup')
    pkg_my_rviz = get_package_share_directory('my_rviz_pkg')
    ekf_config_path = os.path.join(pkg_amr_bringup, 'config', 'ekf.yaml')

    amr_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_sim, 'launch', 'amr_sim.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_amr_description, 'launch', 'state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_my_rviz, 'launch', 'rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'
    )

    debug_lidar_node = Node(
        package='amr_debug',
        executable='debug_lidar',
        name='debug_lidar',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config_path,
            {'use_sim_time': True}
        ]
    )

    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        amr_sim_launch,
        state_publisher_launch,
        rviz_launch,
#        debug_lidar_node,
        teleop_node,
        ekf_node,
        slam_toolbox_launch
    ])
