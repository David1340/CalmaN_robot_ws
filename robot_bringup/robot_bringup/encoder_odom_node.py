import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray, Header
from tf2_ros import TransformBroadcaster
#import tf_transformations
import numpy as np


class EncoderOdom(Node):
    def __init__(self):
        super().__init__('encoder_odom')

        # Parâmetros
        self.declare_parameter('wheels_radius', 0.03)  # em metros
        self.declare_parameter('wheels_distance', 0.173) # distância entre as rodas em metros
        self.declare_parameter('initial_state', [0.0, 0.0, 0.0]) # definição da posição inicial

        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber para os encoders
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/robot/encoder',
            self.encoder_callback,
            10
        )

        initial_state = self.get_parameter('initial_state').value
        if len(initial_state) != 3:
            self.get_logger().error("Parâmetro 'initial_state' deve conter [x, y, yaw]")
            initial_state = [0.0, 0.0, 0.0]
        # Estado do robô
        self.x, self.y, self.th = initial_state
        # Velocidades das rodas
        self.phiE = 0.0  # rad/s roda esquerda
        self.phiD = 0.0  # rad/s roda direita
        # Posições das rodas
        self.posE = 0.0 # rad roda esquerda
        self.posD = 0.0 # rad roda direita



        self.last_time = self.get_clock().now()
        self.create_timer(0.033, self.update_odometry)


        self.L = self.get_parameter('wheels_distance').value
        self.R = self.get_parameter('wheels_radius').value

    def update_odometry(self):
        """
        Integra pose e publica odometria 
        """
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        #if dt <= 0:
        #    return       
        
        # Cinemática diferencial
        vx = (self.phiD + self.phiE) * self.R / 2.0
        vth = (self.phiD - self.phiE)* self.R / self.L

        # Integração da pose
        self.x += vx * np.cos(self.th) * dt
        self.y += vx * np.sin(self.th) * dt
        self.th += vth * dt

        # Orientação como quaternion
        #odom_quat = tf_transformations.quaternion_from_euler(0, 0, self.th)

        # JointState (para o robot_state_publisher)
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.velocity = [self.phiE, self.phiD]
        joint_state.position = [self.posE, self.posD]  # posE(rad),posD(rad)
        self.joint_pub.publish(joint_state)

        # Publica TF odom → base_link
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = np.sin(self.th / 2)
        t.transform.rotation.w = np.cos(self.th / 2)
        self.tf_broadcaster.sendTransform(t)

        # Publica mensagem de odometria
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = np.sin(self.th / 2)
        odom.pose.pose.orientation.w = np.cos(self.th / 2)
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        self.odom_pub.publish(odom)




    def encoder_callback(self, msg: Float32MultiArray):
        """
        msg.data = [posE, posD, phiE, phiD]
        - posE / posD: posição absoluta (ticks)
        - phiE / phiD: velocidade (rad/s)
        """
        # Posições das rodas
        self.posE = msg.data[0] # rad roda esquerda
        self.posD = msg.data[1] # rad roda direita
        # Leitura das velocidades dos encoders
        self.phiE = msg.data[2]  # rad/s roda esquerda
        self.phiD = msg.data[3]  # rad/s roda direita

        #self.update_odometry()

 

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdom()
    try:
        rclpy.spin(node) # Mantém o nó ativo para callbacks
    except KeyboardInterrupt:
        pass
    finally: # o que fazer quando o nó for encerrado:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
