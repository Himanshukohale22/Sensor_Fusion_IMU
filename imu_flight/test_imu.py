import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import random

class ImuPublisher(Node):

    def __init__(self):
        super().__init__('imu_publisher_node')
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(0.1, self.publish_imu_data)  # 10 Hz
        self.get_logger().info("IMU Publisher Node has been started")

    def publish_imu_data(self):
        msg = Imu()

        # Simulating IMU data
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_frame"

        # Simulating 6DOF values
        msg.linear_acceleration.x = random.uniform(-1.0, 1.0)
        msg.linear_acceleration.y = random.uniform(-1.0, 1.0)
        msg.linear_acceleration.z = random.uniform(-1.0, 1.0)

        msg.angular_velocity.x = random.uniform(-math.pi, math.pi)
        msg.angular_velocity.y = random.uniform(-math.pi, math.pi)
        msg.angular_velocity.z = random.uniform(-math.pi, math.pi)

        # No orientation data (Quaternion) in this example, but can be added if needed

        # Log the published values
        self.get_logger().info(
            f'Publishing: linear_accel: [{msg.linear_acceleration.x:.2f}, {msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f}], '
            f'angular_velocity: [{msg.angular_velocity.x:.2f}, {msg.angular_velocity.y:.2f}, {msg.angular_velocity.z:.2f}]'
        )

        # Publish the message
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Shutdown the node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
