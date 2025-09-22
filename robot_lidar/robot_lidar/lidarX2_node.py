import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from LidarX2 import LidarX2
import numpy as np

class LidarX2Node(Node):
    def __init__(self):
        super().__init__('lidar_x2_node')

        # ----- Parâmetros -----
        self.declare_parameter('lidar_port', '/dev/ttyUSB0')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('range_min', 0.12)
        self.declare_parameter('range_max', 8.0)
        self.declare_parameter('publish_rate', 10.0)  # Hz

        lidar_port = self.get_parameter('lidar_port').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.range_min = self.get_parameter('range_min').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max').get_parameter_value().double_value
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        # ----- Lidar -----
        self.lidar = LidarX2(lidar_port)
        if not self.lidar.open():
            self.get_logger().error(f"Não foi possível abrir o Lidar em {lidar_port}")
            rclpy.shutdown()
            return

        self.publisher = self.create_publisher(LaserScan, '/robot/scan', 2)
        self.timer = self.create_timer(1.0 / rate, self.publish_scan)

    def publish_scan(self):
        measures = self.lidar.getMeasures()
        if not measures:
            return

        # Converte para ângulos (rad) e distâncias (m)
        angles = np.radians([m.angle for m in measures])
        ranges = [m.distance / 1000.0 for m in measures]  # mm → m

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = float(np.min(angles))
        scan.angle_max = float(np.max(angles))
        scan.angle_increment = (scan.angle_max - scan.angle_min) / max(1, len(ranges) - 1)
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.timer.timer_period_ns * 1e-9
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges

        self.publisher.publish(scan)

    def destroy_node(self):
        self.lidar.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarX2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
