# camera_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('robot_camera_node')
        self.declare_parameter('device', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera_link')
        device = int(self.get_parameter('device').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(Image, '/robot/camera', 2)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.timer = self.create_timer(1.0 / max(1.0, self.fps), self.timer_cb)
        self.get_logger().info('Camera node started')

    def timer_cb(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Empty frame from camera')
            return
        # OpenCV returns BGR by default
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        msg = self.bridge.cv2_to_imgmsg(gray, encoding='mono8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.cap.release()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down camera node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
