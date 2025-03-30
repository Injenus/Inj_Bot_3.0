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
                ('wheel_diameter', 0.097),
                ('wheel_separation', 0.181),  # Расстояние между центрами левых и правых колёс
                ('max_rpm', 405),  # 1.42 м/с
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
        
        self.rpm_pub = self.create_publisher(
            Int32MultiArray, 
            'wheel/target_rpm', 
            2
        )
        
        self.control_timer = self.create_timer(0.050, self.control_loop)
        self.get_logger().info("Узел управления для омни-колёс запущен")

    def calculate_target_rpms(self, twist_msg):
        def constrain(rpm):
            return min(self.max_rpm, max(-self.max_rpm, rpm))

        Vx = twist_msg.linear.x
        Vy = twist_msg.linear.y  # Скорость вбок
        ω = twist_msg.angular.z

        # Расчёт скоростей для каждого колеса
        L1 = Vx + Vy - ω * (self.wheel_separation / 2)
        R1 = Vx - Vy + ω * (self.wheel_separation / 2)
        L2 = Vx - Vy - ω * (self.wheel_separation / 2)
        R2 = Vx + Vy + ω * (self.wheel_separation / 2)
        

        wheel_circumference = math.pi * self.wheel_diameter

        # Конвертация в RPM
        rpm_L1 = (L1 / wheel_circumference) * 60 if wheel_circumference != 0 else 0.0
        rpm_R1 = (R1 / wheel_circumference) * 60 if wheel_circumference != 0 else 0.0
        rpm_L2 = (L2 / wheel_circumference) * 60 if wheel_circumference != 0 else 0.0
        rpm_R2 = (R2 / wheel_circumference) * 60 if wheel_circumference != 0 else 0.0

        # Применяем ограничения
        rpm_L1 = round(constrain(rpm_L1))
        rpm_R1 = round(constrain(rpm_R1))
        rpm_L2 = round(constrain(rpm_L2))
        rpm_R2 = round(constrain(rpm_R2))

        # Порядок: [R1, L1, L2, R2] (уточните порядок для вашего робота!)
        return [rpm_R1, rpm_L1, rpm_L2, rpm_R2]

    def twist_callback(self, msg):
        self.target_rpms = self.calculate_target_rpms(msg)

    def control_loop(self):
        if not hasattr(self, 'target_rpms'):
            return
        
        rpm_msg = Int32MultiArray()
        rpm_msg.data = self.target_rpms
        self.rpm_pub.publish(rpm_msg)
        
        self.get_logger().info(
            f"Целевые RPM: {self.target_rpms}",
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