import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImgTest(Node):
    def __init__(self):
        super().__init__("img_test")
        self.create_subscription(
            Image,
            "/world/ebtym/model/bird_eye_camera/link/camera_link/sensor/camera_sensor/image",
            self.cb,
            10,
        )

    def cb(self, msg):
        print(
            f"Received frame: {msg.header.stamp}, shape=({msg.height},{msg.width}), encoding={msg.encoding}"
        )


rclpy.init()
node = ImgTest()
rclpy.spin(node)
