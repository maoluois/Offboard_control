import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments (equivalent to <param> tags in ROS 1)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time'
    )

    load_state_filename_arg = DeclareLaunchArgument(
        'load_state_filename', default_value='/home/pi5/Desktop/map/my_map1.pbstream',
        description='Path to the saved .pbstream map file'
    )

    # Nodes
    # robot_state_publisher_node = Node(
    #    package='robot_state_publisher',
    #    executable='robot_state_publisher',
    #    name='robot_state_publisher',
    #    parameters=[{'robot_description': LaunchConfiguration('robot_description'), 'use_sim_time': LaunchConfiguration('use_sim_time')}],
    #    output='screen'
    # )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        arguments=[
            '-configuration_directory',
            FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files',
            '-configuration_basename', 'my_backpack_2d_localization.lua',
            '-load_state_filename', LaunchConfiguration('load_state_filename')
        ],
        remappings=[('echoes', 'scan')],
        output='screen'
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        arguments=['-resolution', '0.05', '-pure_localization', '1'],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',
                   FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/demo_2d.rviz'],
        output='screen'
    )

    return LaunchDescription([
        # Launch Arguments
        use_sim_time_arg,
        load_state_filename_arg,
        # Nodes
        # robot_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node
    ])
