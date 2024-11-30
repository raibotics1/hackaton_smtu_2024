#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge

class Example(Node):
    def __init__(self):
        super().__init__('example_py_node')
        
        # Подписка на топик лидара
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        # Подписка на топик камеры
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )
        
        # Подписка на топик команды движения
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Публикация команды движения
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Для преобразования изображений ROS2 в OpenCV
        self.bridge = CvBridge()
        
        # Переменная для хранения команды движения
        self.cmd_vel_msg = Twist()
        
        # Запуск движения вперед на 1 метр
        self.move_forward()

    def lidar_callback(self, msg):
        # Обработка данных лидара
        self.get_logger().info('Lidar data received')

    def camera_callback(self, msg):
        # Обработка изображений с камеры
        self.get_logger().info('Camera image received')

        # Преобразование изображения ROS2 в OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Отображение изображения
        cv2.imshow('Camera', cv_image)
        cv2.waitKey(1)

    def cmd_vel_callback(self, msg):
        # Обработка команды движения
        self.get_logger().info('Command velocity received')
        self.cmd_vel_msg = msg

    def move_forward(self):
        # Команда движения вперед на 1 метр
        self.cmd_vel_msg.linear.x = 0.5  # Скорость движения вперед
        self.cmd_vel_msg.angular.z = 0.0  # Без вращения
        
        # Публикация команды движения
        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        
        # Пауза для движения на 1 метр
        self.get_logger().info('Moving forward for 1 meter')
        rclpy.spin_once(self, timeout_sec=2.0)  # Движение в течение 2 секунд
        
        # Остановка движения
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel_msg)
        self.get_logger().info('Stopping')

def main(args=None):
    rclpy.init(args=args)
    node = Example()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
