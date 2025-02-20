import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions

def generate_launch_description():
    # Get the share directory for the peripherals package.
    peripherals_share_dir = get_package_share_directory('peripherals')
    # Include the MentorPi camera driver launch file.
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_share_dir, 'launch', 'depth_camera.launch.py')
        )
    )

    return LaunchDescription([
        depth_camera_launch,
        launch_ros.actions.Node(
            package='talent_show',
            executable='bark_node',
            name='bark_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='talent_show',
            executable='spin_node',
            name='spin_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='talent_show',
            executable='moonwalk_node',
            name='moonwalk_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='talent_show',
            executable='fetch',
            name='fetch',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='talent_show',
            executable='bt_control',
            name='bt_control',
            output='screen'
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
