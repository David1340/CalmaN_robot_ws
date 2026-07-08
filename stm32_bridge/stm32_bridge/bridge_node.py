#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
import serial
import numpy as np


def inverter_uL(x):
    return np.sign(x)*(np.abs(x) - (-0.320))/1.325

def inverter_uR(x):
    return np.sign(x)*(np.abs(x) - (-0.156))/1.167

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
        self.declare_parameter('wheels_radius', 0.033)  # em metros
        self.declare_parameter('wheels_distance', 0.159) # distância entre as rodas em metros

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
        

        self.serial_time = 2e-3 # 500 Hz (2 ms)

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
                # Procura o byte de sincronização
                sync = self.serial.read(1)
                if sync != b'\xFE':
                    self.get_logger().warning('Sync byte não encontrado, descartando')
                    return

                # Lê exatamente 9 bytes (4x int16 + 1 byte de checksum)
                data = self.serial.read(9)
                if len(data) != 9:
                    self.get_logger().warning(f'Leitura incompleta: {len(data)} bytes')
                    return

                payload, checksum = data[:8], data[8]

                calc = 0
                for byte in payload:
                    calc ^= byte
                if calc != checksum:
                    # False sync (0xFE was actually payload data) - don't consume,
                    # just go back to scanning one byte at a time
                    return

                values = np.frombuffer(payload, dtype=np.int16).astype(np.float32)

                encoder_msg = Float32MultiArray()
                tick2rad = (2 * np.pi / 3840)
                phiE = values[2]*tick2rad/(10e-3)#rad/s
                phiD = values[3]*tick2rad/(10e-3)# #rad/s
                encoder_msg.data = [values[0] * tick2rad, values[1] * tick2rad, phiE, phiD]
                self.encoder_publisher.publish(encoder_msg)

            except Exception as e:
                self.get_logger().error(f'Error reading from serial: {e}')

    def _inverse_kinematics(self, v, w):
        Wmax = 6.64
        Vmax = 0.528 
        
        if(abs(w) > Wmax):
            w = np.sign(w)*Wmax
        Vmax_w = Vmax*(1-abs(w)/Wmax)
        if(abs(v) > Vmax_w):
            v = np.sign(v)*Vmax_w
        v_r = (2 * v + w * self.L) / (2 * self.R)
        v_l = (2 * v - w * self.L) / (2 * self.R)
        return v_r, v_l
    
    def _montar_mensagem(self, v_r, v_l):
        Vmax = 16 #rad/s velocidade máxima de cada roda
        uL = v_l/Vmax #m/s ->  -1 a 1
        uR = v_r/Vmax #m/s ->  -1 a 1

        uL = satura(inverter_uL(uL))
        uR = satura(inverter_uR(uR))

        msg = [254, 0, 0, 0, 0]
        msg[1] = int(round(250 * abs(uL)))
        msg[3] = int(round(250 * abs(uR)))
        if uL >= 0:
            msg[2] = 0
        else:
            msg[2] = 1

        if uR >= 0:
            msg[4] = 0
        else:
            msg[4] = 1

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