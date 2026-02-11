import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, Header
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from scipy.spatial import KDTree
import numpy as np

class Odometria:
    def __init__(self, r = 0.0325, L = 0.175, N = 4*224.4):
        self.r = r #raio da roda
        self.L = L #distância entre as rodas
        self.N = N #pulsos por revolução (com quadratura)
        self.gain = 2 * np.pi / N #Conversão de contagens para ângulo (radianos)
        self.c_range = (2**16 - 1) * self.gain #range de contagens do encoder (radianos)
        self.Pose = np.zeros(3) #Pose atual do robô [x,y,theta]
        self.PosePrev = np.zeros(3) #Pose anterior do robô [x,y,theta]
        self.phiE_prev = None #ângulo anterior da roda esquerda
        self.phiD_prev = None #ângulo anterior da roda direita
        
    def updatePose(self, PhiE, PhiD):
        dPhiE = PhiE - self.phiE_prev
        dPhiE = self.wrapAround(dPhiE)
        dPhiD = PhiD - self.phiD_prev
        dPhiD = self.wrapAround(dPhiD)
        self.phiE_prev = PhiE
        self.phiD_prev = PhiD


        dS     = (self.r/2) * (dPhiD + dPhiE)
        dTheta = (self.r/self.L) * (dPhiD - dPhiE)
        self.Pose[0] = self.PosePrev[0] + dS*np.cos(self.PosePrev[2] + dTheta/2) #x
        self.Pose[1] = self.PosePrev[1] + dS*np.sin(self.PosePrev[2] + dTheta/2) #y
        self.Pose[2] = self.PosePrev[2] + dTheta #theta
        self.PosePrev = self.Pose.copy()
    
    def setPose(self, x, y, theta):
        self.Pose = np.array([x,y,theta])
        self.PosePrev = self.Pose.copy()

    def setPhiPrev(self, phiE, phiD):
        self.phiE_prev = phiE
        self.phiD_prev = phiD

    def getPose(self):
        return self.Pose

    def wrapAround(self, Phi): 
        #garante que o ângulo fique sempre entre [-c_range, c_range]
        return (Phi + self.c_range/2) % self.c_range - self.c_range/2
    
class ICP2D:
    def __init__(self):
        self.max_iterations = 50
        self.tolerance = 1e-3
        self.filter = 0.2
        self.rot = np.eye(2) #matriz de rotação acumulada
        self.trans = np.zeros(2) #vetor de translação acumulada
        self.prev_pointCloud = None
        self.Pose = np.zeros(3) #Pose atual do robô [x,y,theta]

    def updatePose(self,pointCloud, method='point-to-point'):
        source = pointCloud.T
        source_copy = source.copy()
        tree = KDTree(self.prev_pointCloud)
        sensor = np.array([0,0])  # posição do sensor
        #correspondencias = correspondencias[distances < self.filter]
        
        self.rot = np.eye(2,2)
        self.trans = np.zeros(2)
        if(method == 'point-to-point'):
            for _ in range(self.max_iterations):
                distances, correspondencias = tree.query(source)
                target = self.prev_pointCloud[correspondencias,:]
                #Calcular a transformação usando SVD
                centroid_source = np.mean(source, axis=0)
                centroid_target= np.mean(target, axis=0)
                H = (source - centroid_source).T @ (target - centroid_target)
                U, S, Vt = np.linalg.svd(H)
                R = Vt.T @ U.T

                if(np.linalg.det(R) < 0):
                    Vt[1,:] *= -1
                    R = Vt.T @ U.T
                
                trans = centroid_target - centroid_source @ R.T

                #atualizar a transformação acumulada
                self.rot = R @ self.rot
                self.trans = self.trans @ R.T  + trans #R*t1 + t2

                #atualizar a nuvem de pontos
                source = source @ R.T + trans

                if(np.mean(distances) < self.tolerance):
                    break

        elif(method == 'point-to-plane'):
            k = 5 #vizinhos mais próximos para estimar o vetor normal
            target = self.prev_pointCloud.copy()
            normals = np.zeros_like(target)

            distances, idx_all = tree.query(target, k=k)   # shape (N,k)

            neighbors = target[idx_all]            # (N,k,2)
            centroid_source = neighbors.mean(axis=1, keepdims=True)
            neighbors_c = neighbors - centroid_source

            cov = np.einsum('nik,nil->nkl', neighbors_c, neighbors_c) / k
            eigvals, eigvecs = np.linalg.eigh(cov)
            normals = eigvecs[:, :, 0]   # menor autovalor
            normals /= np.linalg.norm(normals, axis=1, keepdims=True)

            A = np.zeros((len(source), 3))
            b = np.zeros(len(source))

            for _ in range(self.max_iterations):
                distances, correspondencias = tree.query(source)
                mask = distances <  0.1 # 10 cm 
                target = self.prev_pointCloud[correspondencias[mask],:]
                
                #Calcular a transformação usando pseudo inversa
                
                px = source[mask,0]
                py = source[mask,1]

                qx = target[:,0]
                qy = target[:,1]

                nx = normals[correspondencias[mask],0]
                ny = normals[correspondencias[mask],1]

                A = np.column_stack((nx,
                                    ny,
                                    -nx*py + ny*px))

                b = nx*(px - qx) + ny*(py - qy)

                x = -np.linalg.lstsq(A, b, rcond=None)[0]
                tx = x[0]
                ty = x[1]
                trans = np.array([tx, ty])
                theta = x[2]
                R = np.array([[np.cos(theta), -np.sin(theta)],
                              [np.sin(theta),  np.cos(theta)]])
                
                #atualizar a transformação acumulada
                self.rot = R @ self.rot
                self.trans = self.trans @ R.T  + trans #R*t1 + t2

                #atualizar a nuvem de pontos
                source = source @ R.T + trans

                if(np.linalg.norm(x) < self.tolerance):
                    break
        else:
            raise ValueError("Método desconhecido. Use 'point-to-point' ou 'point-to-plane'.")
        self.prev_pointCloud = source_copy
        tx = self.trans[0]
        ty = self.trans[1]
        dth = np.arctan2(self.rot[1,0], self.rot[0,0]) #theta

        self.Pose[0] = self.Pose[0] +  tx*np.cos(self.Pose[2]) - ty*np.sin(self.Pose[2])
        self.Pose[1] = self.Pose[1] +  tx*np.sin(self.Pose[2]) + ty*np.cos(self.Pose[2])
        self.Pose[2] = self.Pose[2] + dth

    def setPose(self, x, y, theta):
        self.Pose = np.array([x,y,theta])

    def getPose(self):
        return self.Pose
    
    def setPrev_pointCloud(self, pointCloud):
        self.prev_pointCloud = pointCloud.T

class Odom(Node):
    def __init__(self):
        super().__init__('encoder_scan_odom')
        self.get_logger().info("Nó criado")
        #messages
        self.poseMsg = Odometry()
        self.poseMsg.header.frame_id = "odom"
        self.poseMsg.child_frame_id = "base_link"

        #parameters
        self.declare_parameter('r', 0.0325) #raio da roda
        self.declare_parameter('L', 0.175)  #distância entre as rodas
        self.declare_parameter('N', 4*224.4) #pulsos por revolução (com quadratura)
        self.declare_parameter('initial_state', [0.0, 0.0, 0.0]) # definição da posição inicial
        self.declare_parameter('icp_method', 'point-to-plane') # método ICP

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        #self.serial_time = 10e-3
        #self.timer = self.create_timer(self.serial_time, self.send_data)  # 500 Hz (2 ms)

        # Subscribers 
        self.encoder_subscriber = self.create_subscription(
            Float32MultiArray,
            '/robot/encoder',
            self.encoder_callback,
            10
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/robot/scan',
            self.scan_callback,
            QoSProfile(depth=10, reliability=QoSReliabilityPolicy.SYSTEM_DEFAULT)
        )

        initial_state = self.get_parameter('initial_state').value
        if len(initial_state) != 3:
            self.get_logger().error("Parâmetro 'initial_state' deve conter [x, y, yaw]")
            initial_state = [0.0, 0.0, 0.0]

        # Inicialização das variáveis
        self.x, self.y, self.th = initial_state
        self.phiE = None
        self.phiD = None
        self.previousPointcloud = None
        self.L = self.get_parameter('L').value
        self.r = self.get_parameter('r').value
        self.N = self.get_parameter('N').value
        self.stoped = False
        self.odom = Odometria()
        self.odom.setPose(self.x, self.y, self.th)

        self.icp = ICP2D()
        self.icp.setPose(self.x, self.y, self.th)
        self.icp_method = self.get_parameter('icp_method').value

    def encoder_callback(self, msg: Float32MultiArray):
        if(self.odom.phiE_prev == None or self.odom.phiD_prev == None):
            self.odom.phiE_prev = msg.data[0]
            self.odom.phiD_prev = msg.data[1]
        else:
            now = self.get_clock().now()
            if(msg.data[0] == self.odom.phiE_prev and msg.data[1] == self.odom.phiD_prev):
                self.stoped = True

            self.odom.updatePose(msg.data[0], msg.data[1])
            Pose = self.odom.getPose()
            self.poseMsg.header.stamp = now.to_msg()
            self.poseMsg.pose.pose.position.x = Pose[0]
            self.poseMsg.pose.pose.position.y = Pose[1]
            self.poseMsg.pose.pose.orientation.z = np.sin(self.th / 2)
            self.poseMsg.pose.pose.orientation.w = np.cos(self.th / 2)
    
    def scan_callback(self, msg: LaserScan):
        distances = msg.ranges
        angles = np.linspace(msg.range_min, msg.range_max, len(distances))
        x = distances*np.cos(angles)
        y = distances*np.sin(angles)
        pointCloud = np.array([x, y])
        if(self.icp.prev_pointCloud is None):
            
            self.icp.setPrev_pointCloud(pointCloud)

        else:
            self.icp.updatePose(pointCloud, method=self.icp_method)
            Pose = self.icp.getPose()
            self.get_logger().info("Pose encoder: " + str(Pose))
            self.get_logger().info("Pose lidar: " + str(Pose))
            Pose_odom = self.odom.getPose()
            Pose[0] = Pose_odom[0]
            Pose[1] = Pose_odom[1]
            if(self.stoped):
                self.stoped = False
                Pose_odom[2] = Pose[2]
                self.get_logger().info("Leitura do lidar ignorada na fusão porque os encoders estão parados.")

            self.icp.setPose(Pose[0], Pose[1], Pose[2])
            self.odom.setPose(Pose[0], Pose[1], Pose[2])
            self.poseMsg.pose.pose.orientation.z = np.sin(self.th / 2)
            self.poseMsg.pose.pose.orientation.w = np.cos(self.th / 2)
            self.odom_pub.publish(self.poseMsg)
            self.get_logger().info("Pose Fusão (encoder + lidar): " + str(Pose_odom))

    #def send_data(self):
        #self.odom_pub.publish(self.poseMsg)

def main(args=None):
    rclpy.init(args=args)
    node = Odom()
    try:
        rclpy.spin(node) # Mantém o nó ativo para callbacks
    except KeyboardInterrupt:  
        pass
    finally: # o que fazer quando o nó for encerrado:
        
        node.destroy_node()
        rclpy.shutdown()