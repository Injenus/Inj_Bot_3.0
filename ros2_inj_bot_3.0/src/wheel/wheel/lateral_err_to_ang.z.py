import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class PIDController:
    def __init__(self, Kp, Ki, Kd, max_output, min_output):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.max_output = max_output
        self.min_output = min_output
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        P = self.Kp * error
        self.integral += error * dt
        I = self.Ki * self.integral
        
        if dt > 0:
            D = self.Kd * (error - self.prev_error) / dt
        else:
            D = 0.0
        self.prev_error = error
        
        output = P + I + D
        
        return max(min(output, self.max_output), self.min_output)

class LateralController(Node):
    def __init__(self):
        super().__init__('lateral_pid_controller')
        
        # Объявляем параметры
        self.declare_parameters(
            namespace='',
            parameters=[
                ('target_linear_speed', 0.21),
                ('wheel_diameter', 0.067),
                ('wheel_separation', 0.2655),
                ('pid.Kp', 1.0),
                ('pid.Ki', 0.0),
                ('pid.Kd', 0.0),
                ('max_angular_speed', 10.0)
            ]
        )
        
        self.target_linear_speed = self.get_parameter('target_linear_speed').value
        self.wheel_diameter = self.get_parameter('wheel_diameter').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        Kp = self.get_parameter('pid.Kp').value
        Ki = self.get_parameter('pid.Ki').value
        Kd = self.get_parameter('pid.Kd').value
        max_angular = self.get_parameter('max_angular_speed').value
        
        # Инициализация пида
        self.pid = PIDController(
            Kp=Kp,
            Ki=Ki,
            Kd=Kd,
            max_output=max_angular,
            min_output=-max_angular
        )
        
        # Для отлеживания dt
        self.last_time = self.get_clock().now()
        
        # Подписка на float32 топик
        self.subscription = self.create_subscription(
            Float32,
            'wheel/lateral_error',
            self.error_callback,
            1
        )
        
        # Публикация в стандартный twist
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        
    def error_callback(self, msg):

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        error = msg.data
        
        angular_z = self.pid.compute(error, dt)
        
        twist_msg = Twist()
        twist_msg.linear.x = self.target_linear_speed
        twist_msg.angular.z = angular_z
        
        self.publisher.publish(twist_msg)
        self.get_logger().debug(
            f'Publishing: linear.x={twist_msg.linear.x:.2f}, angular.z={twist_msg.angular.z:.2f}'
        )

def main():
    rclpy.init()
    lateral_controller = LateralController()
    try:
        rclpy.spin(lateral_controller)
    except:
        lateral_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()