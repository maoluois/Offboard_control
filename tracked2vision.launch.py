import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource

def generate_launch_description():
    # 获取包的路径
    lslidar_driver_launch_path = PathJoinSubstitution([FindPackageShare("lslidar_driver"), "launch", "lsn10_launch.py"])
    cartographer_ros_launch_path = PathJoinSubstitution([FindPackageShare("cartographer_ros"), "launch", "my_backpack_2d_localization.launch.py"])
    mavros_launch_path = PathJoinSubstitution([FindPackageShare("mavros"), "launch", "px4.launch"])



    return LaunchDescription([
        # 启动 lslidar_driver lsn10_launch.launch.py 文件 (Python)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lslidar_driver_launch_path),
            launch_arguments={}.items()
        ),
        # 启动 cartographer_ros my_laser.launch.py 文件 (Python)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cartographer_ros_launch_path),
            launch_arguments={}.items()
        ),
       #启动 mavros px4.launch 文件 (XML)
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mavros_launch_path),
            launch_arguments={}.items()
        ),
       
       
        #tf2:map->odom->base_link
        Node(
            package='tracked2vision',
            executable='tf_broadcaster_node',
            name='tf_broadcaster_node',
            output='screen'
        ),
        
        # 启动 tracked2vision 节点
        Node(
            package='tracked2vision',
            executable='tracked2vision',
            name='tracked2vision',
            output='screen'
        )
    ])