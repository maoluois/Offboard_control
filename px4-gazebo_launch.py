import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess,IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取 World 文件的绝对路径（替代 shell 的 realpath）
    world_path = os.path.realpath("/home/jogs/Desktop/MyWorld/my_world2.world")
    
    # 设置 Gazebo 环境变量
    gazebo_env = {
        "PX4_SITL_WORLD": world_path,
        "GAZEBO_MODEL_PATH": f"{os.environ.get('GAZEBO_MODEL_PATH', '')}:/home/jogs/Desktop/MyWorld",
        "LD_LIBRARY_PATH": os.environ.get("LD_LIBRARY_PATH", "")
    }
    
    

	
    return LaunchDescription([
        DeclareLaunchArgument(
            'workspace',
            default_value=os.path.expanduser('~/.ssh/PX4-Autopilot')
        ),

        # 启动 PX4 SITL 仿真
        ExecuteProcess(
            cmd=[
                "make", "px4_sitl", "gazebo-classic_iris_foggy_lidar"],
            cwd=LaunchConfiguration('workspace'),
            output='screen',
            env={  # 显式传递环境变量
                **os.environ,  # 继承当前环境变量
                **gazebo_env   # 覆盖或新增 Gazebo 相关变量
            }
        ),

        # 桥接 Gazebo 数据到 ROS2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
            output='screen'
        )

 

    ])
