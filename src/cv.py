#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import String
from pyzbar import pyzbar

class QRCodeDetector(Node):
    def __init__(self):
        super().__init__('qr_code_detector')
        
        # 初始化摄像头（降低分辨率）
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  
        
        # 创建二维码数据发布者
        self.qr_pub = self.create_publisher(String, 'qr_data', 10)
        
        # 定时检测二维码（10Hz）
        self.timer = self.create_timer(0.1, self.detect_qr_code)
        
    def detect_qr_code(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("无法读取摄像头画面")
            return
        
        # 转灰度图以提高识别效率
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 解码二维码
        decoded_objects = pyzbar.decode(gray)
        for obj in decoded_objects:
            qr_data = obj.data.decode('utf-8')
            self.get_logger().info(f"检测到二维码: {qr_data}")
            
            # 画框+标注
            (x, y, w, h) = obj.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, qr_data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, (0, 255, 0), 2)

            # 发布消息
            msg = String()
            msg.data = qr_data
            self.qr_pub.publish(msg)
        
        # 显示图像窗口
        cv2.imshow("QR Code Detector", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
           self.get_logger().info("检测器被用户终止")
           rclpy.shutdown()

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = QRCodeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
