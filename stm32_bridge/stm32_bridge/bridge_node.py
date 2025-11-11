#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import serial
import numpy as np

def bitset(valor, pos, val=1):
    if val:
        return valor | (1 << (pos - 1))
    else:
        return valor & ~(1 << (pos - 1))

def inverter_uL(x):
    return np.sign(x)*(np.abs(x) - (-0.414))/1.459

def inverter_uR(x):
    return np.sign(x)*(np.abs(x) - (-0.404))/1.423

def satura(x):
    if np.abs(x)>1:
        return np.sign(x)
    else:
        return x

class STM32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge_node')
        self.get_logger().info('STM32 Bridge Node has been started.')

        #parâmetros de controle
        self.v_r = 0.0  # set point da velocidade da roda direita
        self.v_l = 0.0  # set point da velocidade da roda esquerda

        #parâmetros configuráveis
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheels_radius', 0.03)  # em metros
        self.declare_parameter('wheels_distance', 0.173) # distância entre as rodas em metros

        #carregar parâmetros
        self.port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.L = self.get_parameter('wheels_distance').value
        self.R = self.get_parameter('wheels_radius').value

        self.get_logger().info(f'Parametros: port={self.port} baud={self.baud_rate}')

        self._open_serial()

        #Subscriber para receber comandos de velocidade
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/robot/cmd_vel',
            self.cmd_vel_callback,
            10 
            )
        
        #Publisher para enviar dados do STM32
        self.encoder_publisher = self.create_publisher(
            Float32MultiArray,
            '/robot/encoder',
            10 
            )
        
        self.imu_pub = self.create_publisher(
        Imu,
        '/robot/imu',  
        10 
        )

        self.serial_time = 2e-3 # 500 Hz (2 ms)
        #self.publish_time = 4e-3 # 250 Hz (4ms)
        #self.cnt = 0

        self.timer = self.create_timer(self.serial_time, self.read_serial)  # 500 Hz (2 ms)

    def _open_serial(self): 
        try:
            self.serial = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Serial port {self.port} opened at {self.baud_rate} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.port}: {e}')
            self.serial = None

    def _close_serial(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.get_logger().info(f'Serial port {self.port} closed.')

    def cmd_vel_callback(self, msg):
        if self.serial and self.serial.is_open:
            v = msg.linear.x
            w = msg.angular.z
            self.v_r, self.v_l = self._inverse_kinematics(v, w) #set points de velocidade

    def read_serial(self):
        #self.cnt = self.cnt + 1
        if self.serial and self.serial.is_open:
            msg = self._montar_mensagem(self.v_r, self.v_l)
            self.serial.write(msg)
            try:
                data = self.serial.read(20)  # Ler 20 bytes
                values = np.frombuffer(data, dtype=np.int16).astype(np.float32)


                # publicando encoder
                encoder_msg = Float32MultiArray()
                tick2rad = (2*np.pi/(4*224.4))
                phiE = values[2]*tick2rad/(10e-3)#rad/s
                phiD = values[3]*tick2rad/(10e-3)# #rad/s
                encoder_msg.data = [(values[0] + 32000)*tick2rad,(values[1] + 32000)*tick2rad,
                                    phiE,phiD] # encoder1 e encoder2
                self.encoder_publisher.publish(encoder_msg)

                # publicando IMU
                imu_msg = Imu()
                imu_msg.linear_acceleration.x = values[4]*(9.81/16384.0)
                imu_msg.linear_acceleration.y = values[5]*(9.81/16384.0)
                imu_msg.linear_acceleration.z = values[6]*(9.81/16384.0)
                imu_msg.linear_acceleration_covariance = [
                    7.120e-04, -1.420e-04, -8.800e-05, 
                    -1.420e-04, 5.170e-04, 9.700e-05, 
                    -8.800e-05, 9.700e-05, 2.698e-03]
                
                imu_msg.angular_velocity.x = values[7]*np.pi/(180.0*131.0)
                imu_msg.angular_velocity.y = values[8]*np.pi/(180.0*131.0)
                imu_msg.angular_velocity.z = values[9]*np.pi/(180.0*131.0)
                imu_msg.angular_velocity_covariance = [
                    6.15e-06, -1.06e-06, 7.00e-08,
                    -1.06e-06,  9.08e-06, -3.20e-07,
                    7.00e-08, -3.20e-07,  5.58e-06]
                self.imu_pub.publish(imu_msg)

                
                #if self.cnt > (self.publish_time/self.serial_time):
                    # publicando encoder
                #    self.encoder_publisher.publish(encoder_msg)
                    # publicando IMU
                #    self.imu_pub.publish(imu_msg)
                #    self.cnt = 0

            except Exception as e:
                self.get_logger().error(f'Error reading from serial: {e}')

    def _inverse_kinematics(self, v, w):
        v_r = (2 * v + w * self.L) / (2 * self.R)
        v_l = (2 * v - w * self.L) / (2 * self.R)
        return v_r, v_l
    
    def _montar_mensagem(self, v_r, v_l):
        Vmax = 25.2 #rad/s velocidade máxima de cada roda
        uL = v_l/Vmax #m/s ->  -1 a 1
        uR = v_r/Vmax #m/s ->  -1 a 1

        uL = satura(inverter_uL(uL))
        uR = satura(inverter_uR(uR))

        msg = [254, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        logica = 0
        if uR >= 0:
            msg[8] = int(round(250 * abs(uR)))
            logica = bitset(logica, 2)
        else:
            msg[8] = int(round(250 * abs(uR)))
            logica = bitset(logica, 3)

        if uL >= 0:
            msg[9] = int(round(250 * abs(uL)))
            logica = bitset(logica, 4)
        else:
            msg[9] = int(round(250 * abs(uL)))
            logica = bitset(logica, 5)

        msg[7] = logica

        return bytes(msg)

def main(args=None):
        rclpy.init(args=args)
        node = STM32Bridge()
        try:
            rclpy.spin(node) # Mantém o nó ativo para callbacks
        except KeyboardInterrupt:
            pass
        finally: # o que fazer quando o nó for encerrado:
            node._close_serial()
            node.destroy_node()
            rclpy.shutdown()