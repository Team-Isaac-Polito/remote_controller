import rclpy
from riclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Camera(Node):
    def __init__(self):
        super().__init__("camera")
        self.subscription_rgb = self.create_subscription(Image, "rgb_frame", self.rgb_frame_callback, 10)
        self.br_rgb = CvBridge()

    def rgb_frame_callback(self, data):
        current_frame = self.br_rgb.imgmsg_to_cv2(data)
        cv2.imshow("RGB", current_frame)
        cv2.waitKey(1)


def main(args = None):
    rclpy.init(args = None)
    camera = Camera()
    rclpy.spin(camera)
    camera.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()