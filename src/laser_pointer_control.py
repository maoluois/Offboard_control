#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from gpiozero import LED
import time

class LaserControlNode(Node):
    def __init__(self):
        super().__init__('laser_control_node')

        # 激光笔连接的 GPIO 引脚（假设为引脚17）
        self.laser_pin = LED(17)

        # 创建订阅者，订阅 control_laser_pointer 话题
        self.subscription = self.create_subscription(
            Float64MultiArray,  # 订阅消息类型
            'control_laser_pointer',  # 话题名称
            self.laser_control_callback,  # 回调函数
            10  # 队列大小
        )
        self.subscription  # 防止未使用的警告

    def laser_control_callback(self, msg):
        """ 处理控制激光笔亮灭的回调函数 """
        # 获取传入的消息，包含两个值：第一个值为激光状态，第二个值为持续时间
        laser_state = msg.data[0]  # 1 或 0
        laser_duration = msg.data[1]  # 点亮的时间，单位为秒

        if laser_state == 1:  # 激光笔点亮
            self.laser_pin.on()  # 点亮激光笔
            self.get_logger().info(f"Laser turned ON for {laser_duration} seconds.")
            time.sleep(laser_duration)  # 等待指定时间
            self.laser_pin.off()  # 关闭激光笔
            self.get_logger().info("Laser turned OFF.")
        elif laser_state == 0:  # 激光笔关闭
            self.laser_pin.off()  # 关闭激光笔
            self.get_logger().info("Laser turned OFF.")
        else:
            self.get_logger().warn("Invalid laser state received. Laser state should be 1 or 0.")

def main(args=None):
    rclpy.init(args=args)
    laser_control_node = LaserControlNode()

    try:
        rclpy.spin(laser_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        laser_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
