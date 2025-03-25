import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node 
from launch import LaunchDescription 
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description(): # Define a function named generate_launch_description
    lidar_wall_avoider_node = Node( # Create a Node object for the lidar_wall_avoider node
        package='sumo', # Specify the package name
        executable='wall_avoider', # Specify the executable name
        name='wall_avoider', # Specify the node name
        output='screen', # Specify the output setting to display on the screen
        parameters=[{ # Specify the parameters for the node
            'scan_topic': '/scan', # Specify the scan topic
            'wall_info_topic': 'wall_info' # Specify the wall_info topic
        }]
    )

    centroid_node = Node( # Create a Node object for the centroid node
        package='sumo', # Specify the package name
        executable='centroid', # Specify the executable name
        name='centroid', # Specify the node name
        output='screen', # Specify the output setting to display on the screen
        parameters=[]
    )

    safety_node = Node( # Create a Node object for the safety node
        package='sumo', # Specify the package name
        executable='safety', # Specify the executable name
        name='safety', # Specify the node name
        output='screen', # Specify the output setting to display on the screen
        parameters=[]
    )

    square_controller_node = Node( # Create a Node object for the square_controller node
        package='sumo', # Specify the package name
        executable='square_controller', # Specify the executable name
        name='square_controller', # Specify the node name
        output='screen', # Specify the output setting to display on the screen
        parameters=[ # Specify the parameters for the node
            {'wall_distance_threshold': 0.5} # Specify the wall_distance_threshold parameter
        ]
    )

    attack_node = Node( # Create a Node object for the lidar_wall_avoider node
        package='sumo', # Specify the package name
        executable='attack_enemy', # Specify the executable name
        name='attack_enemy', # Specify the node name
        output='screen', # Specify the output setting to display on the screen
        parameters=[]
    )

    finder_node = Node( # Create a Node object for the lidar_wall_avoider node
        package='sumo', # Specify the package name
        executable='finder_node', # Specify the executable name
        name='finder_node', # Specify the node name
        output='screen', # Specify the output setting to display on the screen
        parameters=[]
    )

    imu_node = Node( # Create a Node object for the imu node
        package='sumo', # Specify the package name
        executable='imu', # Specify the executable name
        name='imu', # Specify the node name
        output='screen', # Specify the output setting to display on the screen
        parameters=[]
    )

    control_node = Node( # Create a Node object for the lidar_wall_avoider node
        package='sumo', # Specify the package name
        executable='control', # Specify the executable name
        name='control', # Specify the node name
        output='screen', # Specify the output setting to display on the screen
        parameters=[]
    )

    imu_frame = LaunchConfiguration('imu_frame', default='imu_link')
    imu_frame_arg = DeclareLaunchArgument('imu_frame', default_value=imu_frame)

    # Get the share directory for the peripherals package.
    peripherals_share_dir = get_package_share_directory('peripherals')
    # Include the MentorPi camera driver launch file.
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_share_dir, 'launch', 'depth_camera.launch.py')
        )
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_share_dir, 'launch', 'lidar.launch.py')
        )
    )

    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_share_dir, 'launch', 'depth_camera.launch.py')
        )
    )

    # DEBUG

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(peripherals_share_dir) + '/rviz/lidar_view.rviz'],
        output='screen',
    )

    return LaunchDescription([ # Create a LaunchDescription object
        lidar_launch,
        centroid_node,
        depth_camera_launch,
        lidar_wall_avoider_node, # Add the lidar_wall_avoider node to the launch description
        # square_controller_node, # Add the square_controller node to the launch description
        attack_node,
        finder_node,
        safety_node,
        imu_frame_arg,
        control_node,

        # rviz_node # DEBUG
    ])