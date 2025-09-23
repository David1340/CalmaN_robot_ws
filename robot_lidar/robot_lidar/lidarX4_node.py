# lidarX4_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import PyLidar3
import time

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidarX4_node')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('scan_rate', 10.0)
        self.declare_parameter('chunk_size', 1500)  #pontos

        port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.scan_rate = float(self.get_parameter('scan_rate').value)
        chunk_size = self.get_parameter('chunk_size').value
        self.pub = self.create_publisher(LaserScan, '/robot/scan', 2)

        # connect lidar
        self.get_logger().info(f'Connecting to LIDAR on {port}...')
        self.lidar = PyLidar3.YdLidarX4(port,chunk_size)
        connected = False
        for _ in range(5):
            if self.lidar.Connect():
                connected = True
                break
            self.get_logger().warning('Lidar connect failed, retrying in 1s...')
            time.sleep(1)

        if not connected:
            self.get_logger().error('Could not connect to LIDAR. Exiting node.')
            raise RuntimeError('Lidar connection failed')

        self.scan_gen = self.lidar.StartScanning()
        # timer to publish at approx scan_rate
        self.timer = self.create_timer(1.0 / self.scan_rate, self.timer_cb)
        self.get_logger().info('Lidar node started')

    def timer_cb(self):
        try:
            measures = next(self.scan_gen)  # dict angle->distance
            if not measures:
                return
            # Sort by angle
            angles = sorted(measures.keys())
            distances = [float(measures[a]/1000.0) for a in angles]
            angles_rad = [a * np.pi / 180.0 for a in angles]

            angle_min = min(angles_rad)
            angle_max = max(angles_rad)
            if len(distances) > 1:
                angle_inc = (angle_max - angle_min) / (len(distances) - 1)
            else:
                angle_inc = 0.0

            scan = LaserScan()
            scan.header.stamp = self.get_clock().now().to_msg()
            scan.header.frame_id = self.frame_id
            scan.angle_min = float(angle_min)
            scan.angle_max = float(angle_max)
            scan.angle_increment = float(angle_inc)
            scan.time_increment = 0.0
            scan.scan_time = 1.0 / self.scan_rate
            scan.range_min = 0.02
            scan.range_max = 10.0
            scan.ranges = distances
            scan.intensities = []

            self.pub.publish(scan)
        except StopIteration:
            self.get_logger().error('Lidar generator finished')
        except Exception as e:
            self.get_logger().warning(f'Lidar read error: {e}')

    def destroy_node(self):
        try:
            self.lidar.StopScanning()
            self.lidar.Disconnect()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down lidar node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
