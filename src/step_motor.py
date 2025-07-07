import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import RPi.GPIO as GPIO
import time

# 设置GPIO引脚
IN1 = 5   # 树莓派的引脚与驱动连接
IN2 = 6
IN3 = 13
IN4 = 19

GPIO.setmode(GPIO.BCM)       # 使用BCM编码
GPIO.setwarnings(False)

GPIO.setup(IN1, GPIO.OUT)      # 设置为输出模式
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# 步进电机控制函数
def setStep(h1, h2, h3, h4):
    GPIO.output(IN1, h1)
    GPIO.output(IN2, h2)
    GPIO.output(IN3, h3)
    GPIO.output(IN4, h4)

# 电机步距角和减速比
step_angle = 5.625 / 64  # 每个步进电机步进的角度
gear_ratio = 64  # 减速比

# 计算每个角度对应的步数
def angle_to_steps(angle):
    return int((angle / (step_angle * gear_ratio)) * 4096)  # 4096为一个完整旋转所需的步数

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

    def angle_callback(self, msg):
        # 根据接收到的角度，计算步数并控制电机旋转
        target_angle = msg.data
        steps = angle_to_steps(target_angle)
        self.get_logger().info(f'Received angle: {target_angle}, steps: {steps}')

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
            for i in range(0, abs(steps)):  # 使用绝对值来确保步数为正
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
        GPIO.cleanup()

if __name__ == '__main__':
    main()
