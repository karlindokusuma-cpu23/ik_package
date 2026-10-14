import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class InverseKinematicsCalculator(Node):

    def __init__(self):
        super().__init__('inverse_kinematics_calculator')

        # Parameter fisik robot
        self.wheel_radius = 0.05
        self.wheel_base = 0.30
        
        # Publisher untuk hasil inverse kinematics
        self.inverse_publisher = self.create_publisher(
            Float64MultiArray,
            'Topic_satu',
            5
        )

        # Timer untuk menjalankan inverse kinematics secara periodik
        self.inverse_timer = self.create_timer(
            0.5,
            self.execute_inverse_kinematics
        )

        # Input parameter untuk inverse kinematics
        print('=' * 50)
        print('INVERSE KINEMATICS - INPUT PARAMETER')
        print('=' * 50)
        self.velocity_y = float(input('Kecepatan robot sumbu y (m/s): '))
        self.angular_velocity = float(input('Kecepatan angular robot (rad/s): '))
        print('=' * 50)

    def compute_inverse_kinematics(self):
        # Perhitungan inverse kinematics untuk roda kanan
        omega_right = (
            (2 * self.velocity_y + self.wheel_base * self.angular_velocity)
            / (2 * self.wheel_radius)
        )

        # Perhitungan inverse kinematics untuk roda kiri
        omega_left = (
            (2 * self.velocity_y - self.wheel_base * self.angular_velocity)
            / (2 * self.wheel_radius)
        )

        return omega_left, omega_right

    def execute_inverse_kinematics(self):
        # Jalankan inverse kinematics
        left_wheel, right_wheel = self.compute_inverse_kinematics()

        # Siapkan pesan hasil inverse kinematics
        inverse_result = Float64MultiArray()
        inverse_result.data = [
            left_wheel,
            right_wheel
        ]

        # Publish hasil inverse kinematics
        self.inverse_publisher.publish(inverse_result)

        # Tampilkan log inverse kinematics
        self.get_logger().info(
            'Vy = ' + str(self.velocity_y) +
            ' m/s | omega = ' + str(self.angular_velocity) +
            ' rad/s | omega_L = ' + str(left_wheel) +
            ' rad/s | omega_R = ' + str(right_wheel) +
            ' rad/s'
        )


def main(args=None):
    rclpy.init(args=args)

    inverse_node = InverseKinematicsCalculator()

    try:
        rclpy.spin(inverse_node)

    except KeyboardInterrupt:
        print('\nInverse kinematics node dihentikan')

    finally:
        inverse_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
