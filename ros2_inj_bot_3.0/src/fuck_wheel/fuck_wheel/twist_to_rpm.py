import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
import math


class TwistToRPM(Node):
    def __init__(self):
        super().__init__('twist_to_rpm_with_feedback')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_diameter', 0.067),
                ('wheel_separation', 0.1655),
                ('max_rpm', 405), # 1.42 м/с
            ]
        )
        
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_rpm = self.get_parameter('max_rpm').value

        
        self.twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            2
        )
        
        # # Берём текущие обороты из топика, в который писали от драйвера
        # self.rpm_sub = self.create_subscription(
        #     Int32MultiArray,
        #     'wheel/current_rpm',
        #     self.rpm_callback,
        #     2
        # )
        
        # Публикуем целевые обороты (БЕЗ РЕГУЛИРВОКИ, просто желаемые, вся реглирвока на драйвере!)
        self.rpm_pub = self.create_publisher(
            Int32MultiArray, 
            'wheel/target_rpm', 
            2
        )
        
        # Частота работы
        self.control_timer = self.create_timer(0.050, self.control_loop)
        
        self.get_logger().info("Узел управления запущен")

    # def rpm_callback(self, msg):
    #     if len(msg.data) == 4:
    #         self.current_rpms = msg.data
    #         #self.last_rpm_update = self.get_clock().now()
    #     else:
    #         self.get_logger().warn("Некорректное сообщение RPM")

    def calculate_target_rpms(self, twist_msg):
        def constrain(rpm):
            return min(self.max_rpm, max(-self.max_rpm, rpm))

        # Конвертируем twist в целевые rpm
        linear_vel = twist_msg.linear.x
        angular_vel = twist_msg.angular.z
        
        left_vel = linear_vel - (angular_vel * self.wheel_separation / 2)
        right_vel = linear_vel + (angular_vel * self.wheel_separation / 2)
        
        wheel_circumference = math.pi * self.wheel_diameter
        left_rpm = round(constrain((left_vel / wheel_circumference) * 60 if wheel_circumference != 0 else 0.0))
        right_rpm = round(constrain((right_vel / wheel_circumference) * 60 if wheel_circumference != 0 else 0.0))

        return [right_rpm, left_rpm, left_rpm, right_rpm]

    def twist_callback(self, msg):
        # Store latest twist command
        self.target_rpms = self.calculate_target_rpms(msg)

    def control_loop(self):
        if not hasattr(self, 'target_rpms'):
            return
        
        rpm_msg = Int32MultiArray()
        rpm_msg.data = self.target_rpms
        self.rpm_pub.publish(rpm_msg)
        
        self.get_logger().info(
            f"Целевые: {self.target_rpms}",
            throttle_duration_sec=1.0
        )

def main():
    rclpy.init()
    twist_to_rpm = TwistToRPM()
    try:
        rclpy.spin(twist_to_rpm)
    except KeyboardInterrupt:
        twist_to_rpm.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()