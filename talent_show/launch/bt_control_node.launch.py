import os
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='talent_show',
            executable='bark_node.py',
            name='bark_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='talent_show',
            executable='bt_control_node.py',
            name='bt_control',
            output='screen'
        )
    ])
