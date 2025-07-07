#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from gpiozero import OutputDevice
import time

# 设置GPIO引脚
IN1 = OutputDevice(5)  # 树莓派的引脚与驱动连接
IN2 = OutputDevice(6)
IN3 = OutputDevice(13)
IN4 = OutputDevice(19)

# 步进电机控制函数
def setStep(h1, h2, h3, h4):
    IN1.value = h1
    IN2.value = h2
    IN3.value = h3
    IN4.value = h4

# 电机步距角和减速比
step_angle = 5.625 / 64  # 每个步进电机步进的角度
gear_ratio = 64  # 减速比

# 计算每个角度对应的步数
def angle_to_steps(angle):
    return int((angle / 360) * 1024)  # 4096为一个完整旋转所需的步数

# 控制电机旋转的类
class StepperMotorNode(Node):
    def __init__(self):
        super().__init__('stepper_motor_node')
        self.subscription = self.create_subscription(
            Float64,
            'control_angle',  # 控制角度的话题名称
            self.angle_callback,
            10
        )
        self.subscription  # 防止未使用的警告
        self.delay = 0.003  # 控制转速
        self.last_angle = None  # 保存上一个目标角度

    def angle_callback(self, msg):
        # 获取目标角度
        target_angle = msg.data

        # 如果目标角度和上次的角度相同，则不进行旋转
        if target_angle == self.last_angle:
            #self.get_logger().info(f"目标角度未变化({target_angle}), 不执行旋转。")
            return

        # 更新上一个目标角度
        self.last_angle = target_angle

        # 计算需要的步数
        steps = angle_to_steps(target_angle)
        self.get_logger().info(f'Received angle: {target_angle}, steps: {steps}')

        # 根据目标角度的正负决定旋转方向
        if target_angle >= 0:  # 正向旋转（顺时针）
            for i in range(0, steps):
                setStep(1, 0, 0, 0)
                time.sleep(self.delay)
                setStep(0, 1, 0, 0)
                time.sleep(self.delay)
                setStep(0, 0, 1, 0)
                time.sleep(self.delay)
                setStep(0, 0, 0, 1)
                time.sleep(self.delay)
        else:  # 反向旋转（逆时针）
            for i in range(0, int(-steps/2)):  # 确保步数为正 
                setStep(0, 0, 0, 1)
                time.sleep(self.delay)
                setStep(0, 0, 1, 0)
                time.sleep(self.delay)
                setStep(0, 1, 0, 0)
                time.sleep(self.delay)
                setStep(1, 0, 0, 0)
                time.sleep(self.delay)

        setStep(0, 0, 0, 0)  # 停止电机

def main(args=None):
    rclpy.init(args=args)
    motor_node = StepperMotorNode()

    try:
        rclpy.spin(motor_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
