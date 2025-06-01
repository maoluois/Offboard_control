#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from gpiozero import PWMOutputDevice
from time import sleep

SERVO_PIN = 18  # BCM编号，SG90连接的引脚

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')

        # 使用 gpiozero 的 PWMOutputDevice 精准设置频率
        try:
            self.servo = PWMOutputDevice(SERVO_PIN, frequency=50, initial_value=0)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize PWM device: {e}")
            raise

        self.subscription = self.create_subscription(
            Int32,
            'servo_cmd',
            self.servo_callback,
            10)

    def servo_callback(self, msg):
        angle = max(0, min(180, msg.data))

        # 将角度转换为 SG90 所需的高电平时间（ms）
        pulse_width_ms = 0.5 + (angle / 180.0) * 2.0  # 0.5ms ~ 2.5ms
        duty_cycle = pulse_width_ms / 20.0  # 转换为 duty_cycle (周期20ms)

        self.servo.value = duty_cycle  # 设置占空比（范围0~1）
        self.get_logger().info(f"Servo angle: {angle}°, PWM duty: {duty_cycle:.3f}")
        sleep(0.3)  # 保证舵机动作完成前信号稳定

    def destroy_node(self):
        self.servo.value = 0
        self.servo.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
