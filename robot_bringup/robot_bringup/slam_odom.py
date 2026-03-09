import rclpy
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped

import math


class PoseSLAM(Node):

    def __init__(self):

        super().__init__('pose_slam_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(PoseStamped, '/pose_slam', 10)

        self.timer = self.create_timer(0.05, self.update_pose)


    def update_pose(self):

        try:

            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            x = t.transform.translation.x
            y = t.transform.translation.y

            q = t.transform.rotation

            yaw = math.atan2(
                2*(q.w*q.z + q.x*q.y),
                1 - 2*(q.y*q.y + q.z*q.z)
            )

            msg = PoseStamped()

            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"

            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = yaw   # yaw no eixo z

            msg.pose.orientation.w = 1.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0

            self.pub.publish(msg)

        except Exception:
            pass


def main():

    rclpy.init()

    node = PoseSLAM()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()