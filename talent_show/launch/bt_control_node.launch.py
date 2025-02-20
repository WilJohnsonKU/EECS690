import os
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
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
            executable='bt_control',
            name='bt_control',
            output='screen'
        )
    ])
