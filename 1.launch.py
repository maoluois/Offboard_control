from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess, EmitEvent
from launch.event_handlers import OnProcessIO
from launch.events import Shutdown
from launch_ros.actions import Node
import re

def generate_launch_description():
    # 启动Cartographer建图节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        parameters=[{'use_sim_time': False}],
        arguments=[
            '-configuration_directory', '/path/to/cartographer_config',
            '-configuration_basename', 'my_map_builder.lua'
        ],
        output='screen'  # 必须输出到屏幕以捕获日志
    )

    # 定义终止轨迹0的服务调用
    finish_trajectory_service = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/cartographer_node/finish_trajectory',
            'cartographer_ros_msgs/srv/FinishTrajectory',
            '{"trajectory_id": 0}'
        ],
        output='screen'
    )

    # 事件处理器：监听Cartographer节点的日志输出
    def on_cartographer_output(event):
        text = event.text.decode('utf-8').strip()
        # 正则匹配目标错误日志
        if re.search(r"Can't run final optimization.*trajectory with ID 0", text):
            print("\n[检测到地图完成信号] 正在终止轨迹0...")
            return [finish_trajectory_service, EmitEvent(event=Shutdown(reason='map_done'))]  # 可选：终止launch
        return []

    # 注册事件监听器
    carto_log_handler = RegisterEventHandler(
        event_handler=OnProcessIO(
            target_action=cartographer_node,
            on_stdout=on_cartographer_output,
            on_stderr=on_cartographer_output  # 同时监控stderr
        )
    )

    return LaunchDescription([
        cartographer_node,
        carto_log_handler
    ])