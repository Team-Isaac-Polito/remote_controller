import rclpy
import serial
import struct
import time
from reseq_interfaces.msg import Remote
from sys import exit
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

serial_port = "/dev/ttyUSB0"
baud_rate = 9600
remote_topic = "/remote"
qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, 
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)


class SerialPublisher(Node):
    def __init__(self):
        super().__init__("remote_controller")
        self.publisher = self.create_publisher(Remote, remote_topic, qos)

        try:
            self.serial = serial.Serial(serial_port, baud_rate)
            self.get_logger().info(f"Serial port {serial_port} opened successfully")
            time.sleep(1)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {str(e)}")
            self.destroy_node()
            rclpy.shutdown()
            exit(1)

    def start_arduino(self):
            self.get_logger().info("starting arduino...")
            time.sleep(3)
            self.serial.write(bytes([0]))
            time.sleep(1)
            self.serial.reset_input_buffer()
            self.serial.write(bytes([1]))

            code = self.serial.read(size=2)
            if code == bytes([4,4]):
                self.get_logger().info(f"Arduino ready!")
            elif code == bytes([6,4]):
                self.get_logger().error("Arduino error code!")
            else:
                self.get_logger().error("Error: unexpected code from Arduino")

    def publish_serial_data(self):
        while rclpy.ok():
            if self.serial.is_open:
                code = self.serial.read(size=2)
                if code == bytes([5,4]):
                    data = self.serial.read(size=14)
                    data = struct.unpack('<Hhhhhhh', data)
                    buttons = data[0]
                    joy_left = data[1:4]
                    joy_right = data[4:]

                    self.get_logger().info(f"left:{joy_left}, right:{joy_right}, buttons:{buttons}")

                    msg = Remote()
                    msg.left.x = self.scale(joy_left[0])
                    msg.left.y = self.scale(joy_left[1])
                    msg.left.z = self.scale(joy_left[2])
                    msg.right.x = self.scale(joy_right[0])
                    msg.right.y = self.scale(joy_right[1])
                    msg.right.z = self.scale(joy_right[2])
                    
                    # buttons are in the first 10 bits of the first byte
                    # they are inverted due to little endian convention
                    msg.buttons = [bool((buttons >> bit) & 0x1) for bit in range(9, -1, -1)]

                    self.publisher.publish(msg)
                else:
                    print("Arduino error code! Restarting...")
                    self.start_arduino()
                    time.sleep(2)

    # scale values from [-512,511] to [-1,1]
    def scale(self, m):
        if (m==0):
            return 0.0
        return round((m+512)/(511.5)-1, 4)

    def run(self):
        self.start_arduino()
        time.sleep(2)
        self.publish_serial_data()


def main(args=None):
    rclpy.init(args=args)

    serial_publisher = SerialPublisher()
    serial_publisher.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

