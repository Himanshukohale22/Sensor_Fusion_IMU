import rclpy
from rclpy.node import Node 
from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Imu

import serial 
import time 
# from .serial_data_extraction import ax, ay, az, roll, pitch, yaw

# X = values.ax
# Y = values.ay 
# Z = values.az 

# rollx = values.roll
# pitchy  = values.pitch
# yawz = values.yaw

## extracting data from arduino 
## as there is no certain library for ros2 
## in arduino env 

class imu_pub(Node):

    def __init__(self):
        super().__init__('imu_node')
        self.publisher_ = self.create_publisher(Imu, 'imu_raw', 10)
        timer_period = 0.5 
        self.timer_ = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("getting started with imu readings")
        
        self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)  # Change to your port


    def timer_callback(self):
        if self.serial_port.in_waiting > 0:
            # Read line from Arduino
            line = self.serial_port.readline().decode('utf-8').rstrip()
            data = line.split(',')
            if len(data) >= 9:  # Assuming you have 9 values (orientation, angular velocity, linear acceleration)
                imu_msg = Imu()
                imu_msg.orientation.x = float(data[0])
                imu_msg.orientation.y = float(data[1])
                imu_msg.orientation.z = float(data[2])
                # imu_msg.orientation.w = float(data[3])
                imu_msg.angular_velocity.x = float(data[3])
                imu_msg.angular_velocity.y = float(data[4])
                imu_msg.angular_velocity.z = float(data[5])
                # imu_msg.linear_acceleration.x = float(data[7])
                # imu_msg.linear_acceleration.y = float(data[8])
                # imu_msg.linear_acceleration.z = float(data[9])

                # Publish the message
                self.publisher_.publish(imu_msg)
                self.get_logger().info('Publishing: "%s"' % imu_msg)


        # msg = Imu()

        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = "imu_frame"

        # msg.linear_acceleration.x = ax
        # msg.linear_acceleration.y = ay
        # msg.linear_acceleration.z = az

        # msg.angular_velocity.x = roll
        # msg.angular_velocity.y = pitch
        # msg.angular_velocity.z = yaw

        # self.get_logger().info(
        #     f'Publishing: linear_accel: [{msg.linear_acceleration.x:.2f}, {msg.linear_acceleration.y:.2f}, {msg.linear_acceleration.z:.2f}], '
        #     f'angular_velocity: [{msg.angular_velocity.x:.2f}, {msg.angular_velocity.y:.2f}, {msg.angular_velocity.z:.2f}]'
        # )

        # self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = imu_pub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


