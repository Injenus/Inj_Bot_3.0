import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math

class WheelPIDController:
    def __init__(self, Kp, Ki, Kd, max_rpm, min_rpm):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_rpm = max_rpm
        self.min_rpm = min_rpm
        self.integral = 0.0
        self.prev_error = 0.0
        self.last_time = None

    def compute(self, target_rpm, current_rpm, current_time):
        if self.last_time is None:
            self.last_time = current_time
            return current_rpm  # Для первой итерации нет коррекции
        
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        error = target_rpm - current_rpm

        P = self.Kp * error
        self.integral += error * dt
        I = self.Ki * self.integral
        D = self.Kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        
        self.prev_error = error

        output = current_rpm + P + I + D  # Добавляем коррекцию к текущим RPM

        return max(min(output, self.max_rpm), self.min_rpm)

class TwistToRPM(Node):
    def __init__(self):
        super().__init__('twist_to_rpm_with_feedback')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('wheel_diameter', 0.067),
                ('wheel_separation', 0.2655),
                ('max_rpm', 558.14), # 2 m/s - коэффициент 8330.44
                ('pid.Kp', 0.5),
                ('pid.Ki', 0.0),
                ('pid.Kd', 0.0),
            ]
        )
        
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.max_rpm = self.wheel_diameter * 8330.44  # self.get_parameter('max_rpm').value
        Kp = self.get_parameter('pid.Kp').value
        Ki = self.get_parameter('pid.Ki').value
        Kd = self.get_parameter('pid.Kd').value
        
        # ПИДы для всех колёс
        self.pid_controllers = [
            WheelPIDController(Kp, Ki, Kd, self.max_rpm, -self.max_rpm),
            WheelPIDController(Kp, Ki, Kd, self.max_rpm, -self.max_rpm),
            WheelPIDController(Kp, Ki, Kd, self.max_rpm, -self.max_rpm),
            WheelPIDController(Kp, Ki, Kd, self.max_rpm, -self.max_rpm)
        ]
        
        # Храним текущие обороты
        self.current_rpms = [0.0, 0.0, 0.0, 0.0]
        self.last_rpm_update = self.get_clock().now()
        
        # Subscribers
        self.twist_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_callback,
            1
        )
        
        # Берём текущие обороты от data_exchange
        self.rpm_sub = self.create_subscription(
            Float32MultiArray,
            'wheel/current_rpm',
            self.rpm_callback,
            1
        )
        
        # Публикуем целевые обороты для data_exchange
        self.rpm_pub = self.create_publisher(
            Float32MultiArray, 
            'wheel/target_rpm', 
            1
        )
        
        # Частота работы
        self.control_timer = self.create_timer(0.01, self.control_loop)
        
        self.get_logger().info("Узел управления с обратной связью запущен")

    def rpm_callback(self, msg):
        if len(msg.data) == 4:
            self.current_rpms = msg.data
            self.last_rpm_update = self.get_clock().now()
        else:
            self.get_logger().warn("Некорректное сообщение RPM")

    def calculate_target_rpms(self, twist_msg):
        # Конвертируем twist в целевые rpm
        linear_vel = twist_msg.linear.x
        angular_vel = twist_msg.angular.z
        
        left_vel = linear_vel - (angular_vel * self.wheel_separation / 2)
        right_vel = linear_vel + (angular_vel * self.wheel_separation / 2)
        
        wheel_circumference = math.pi * self.wheel_diameter
        left_rpm = (left_vel / wheel_circumference) * 60 if wheel_circumference != 0 else 0.0
        right_rpm = (right_vel / wheel_circumference) * 60 if wheel_circumference != 0 else 0.0
        
        return [left_rpm, left_rpm, right_rpm, right_rpm]

    def twist_callback(self, msg):
        # Store latest twist command
        self.target_rpms = self.calculate_target_rpms(msg)

    def control_loop(self):
        if not hasattr(self, 'target_rpms'):
            return
            
        current_time = self.get_clock().now()
        corrected_rpms = []
        
        # Вычисление скорректированных пидом rpm
        for i in range(4):
            pid = self.pid_controllers[i]
            corrected = pid.compute(
                self.target_rpms[i], 
                self.current_rpms[i],
                current_time
            )
            corrected_rpms.append(corrected)
        
        rpm_msg = Float32MultiArray()
        rpm_msg.data = corrected_rpms
        self.rpm_pub.publish(rpm_msg)
        
        self.get_logger().info(
            f"Целевые: {self.target_rpms} | Текущие: {self.current_rpms} | Корректированные: {corrected_rpms}",
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