#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from gpiozero import PWMOutputDevice
from time import sleep

# 定义两个舵机的 GPIO 引脚（BCM 编号）
SERVO1_PIN = 18
SERVO2_PIN = 19

class DualServoController(Node):
    def __init__(self):
        super().__init__('dual_servo_controller')

        # 初始化两个 PWM 输出
        try:
            self.servo1 = PWMOutputDevice(SERVO1_PIN, frequency=50, initial_value=0)
            self.servo2 = PWMOutputDevice(SERVO2_PIN, frequency=50, initial_value=0)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize PWM device: {e}")
            raise

        # 订阅两个舵机的控制话题
        self.subscription1 = self.create_subscription(
            Int32,
            'servo1_cmd',
            self.servo1_callback,
            10)

        self.subscription2 = self.create_subscription(
            Int32,
            'servo2_cmd',
            self.servo2_callback,
            10)

    def angle_to_duty(self, angle):
        """将角度转换为 duty_cycle（SG90 的 0.5~2.5ms 占 20ms 周期）"""
        angle = max(0, min(180, angle))
        pulse_width_ms = 0.5 + (angle / 180.0) * 2.0
        return pulse_width_ms / 20.0

    def servo1_callback(self, msg):
        duty = self.angle_to_duty(msg.data)
        self.servo1.value = duty
        self.get_logger().info(f"[Servo1] Angle: {msg.data}°, Duty: {duty:.3f}")
        sleep(0.3)

    def servo2_callback(self, msg):
        duty = self.angle_to_duty(msg.data)
        self.servo2.value = duty
        self.get_logger().info(f"[Servo2] Angle: {msg.data}°, Duty: {duty:.3f}")
        sleep(0.3)

    def destroy_node(self):
        self.servo1.value = 0
        self.servo2.value = 0
        self.servo1.close()
        self.servo2.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DualServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
