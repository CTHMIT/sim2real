# rtsp_image_receiver.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class RTSPReceiver(Node):
    def __init__(self):
        super().__init__("rtsp_receiver")
        self.publisher = self.create_publisher(Image, "/camera/image_raw", 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(
            "rtspsrc location=rtsp://172.19.160.1:8554/live.sdp ! rtph264depay ! avdec_h264 ! videoconvert ! appsink",
            cv2.CAP_GSTREAMER,
        )
        self.timer = self.create_timer(0.03, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher.publish(msg)


def main():
    rclpy.init()
    node = RTSPReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
