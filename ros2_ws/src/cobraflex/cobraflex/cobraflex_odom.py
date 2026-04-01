#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String
import json
import serial
import sys
import math
import signal
import time

class CobraFlexROSDriver(Node):
    def __init__(self):
        super().__init__("cobraflex_ros_driver")

        # --- PARAMETERS ---
        self.declare_parameter("port", "/dev/ttyACM1") # Ajusta a tu puerto real
        self.declare_parameter("baud", 115200)
        self.declare_parameter("wheel_base", 0.154) # Distancia entre ruedas

        self.port = self.get_parameter("port").value
        self.baud = self.get_parameter("baud").value
        self.wheel_base = self.get_parameter("wheel_base").value
        
        # Frames
        self.base_frame_id = "base_link" # O base_link
        self.imu_frame_id = "imu_link"

        # --- PUBLISHERS ---
        # Publicamos TwistWithCovariance para que el EKF sepa qué tan fiable es el dato
        self.vel_pub = self.create_publisher(TwistWithCovarianceStamped, "/cobraflex/vel_raw", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.batt_pub = self.create_publisher(Float32, "/cobraflex/battery", 10)

        # --- SUBSCRIBER ---
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)

        # --- SERIAL CONNECTION ---
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Conectado a {self.port}")
        except Exception as e:
            self.get_logger().error(f"Error Serial: {e}")
            sys.exit(1)

        # Timers
        self.create_timer(0.02, self.read_serial)      # 50 Hz lectura
        self.create_timer(0.1, self.resend_last_cmd)   # 10 Hz keep-alive

        self.last_vx = 0.0
        self.last_wz = 0.0
        
        # Activar feedback continuo en el robot
        self.send_json({"T":131, "cmd":1})

        # Signal handler para cerrar limpio
        signal.signal(signal.SIGINT, self.shutdown_handler)

    def cmd_callback(self, msg):
        self.last_vx = msg.linear.x
        self.last_wz = msg.angular.z

    def resend_last_cmd(self):
        # Enviar comando de velocidad
        self.send_json({"T":13, "X":self.last_vx, "Z":self.last_wz})

    def send_json(self, data):
        try:
            line = json.dumps(data) + "\n"
            self.ser.write(line.encode("utf-8"))
        except Exception as e:
            self.get_logger().warn(f"Error escritura: {e}")

    def read_serial(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode("utf-8").strip()
                if not line: return
                data = json.loads(line)
                self.process_data(data)
            except ValueError:
                pass
            except Exception as e:
                self.get_logger().warn(f"Error lectura: {e}")

    def process_data(self, data):
        t_code = data.get("T")
        now = self.get_clock().now().to_msg()

        # --- T=1001: DATOS DE MOTORES Y BATERÍA ---
        if t_code == 1001:
            # 1. Batería
            if "v" in data:
                self.batt_pub.publish(Float32(data=float(data["v"])))

            # 2. Velocidad de ruedas (Asumiendo que 'odl'/'odr' son m/s)
            # Si son encoders raw, habría que derivar, pero en el JSON suelen venir procesados
            v_left = float(data.get("odl", 0.0))
            v_right = float(data.get("odr", 0.0))

            # Calcular velocidad lineal y angular del robot (Cinemática Diferencial)
            vx = (v_right + v_left) / 2.0
            wz = (v_right - v_left) / self.wheel_base

            # Crear mensaje Twist con Covarianza
            twist_msg = TwistWithCovarianceStamped()
            twist_msg.header.stamp = now
            twist_msg.header.frame_id = self.base_frame_id
            twist_msg.twist.twist.linear.x = vx
            twist_msg.twist.twist.angular.z = wz
            
            # Covarianza: [x, y, z, roll, pitch, yaw]
            # Confiamos mucho en lineal (0.01) y poco en angular (0.1) porque resbala
            twist_msg.twist.covariance = [
                0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.1
            ]
            self.vel_pub.publish(twist_msg)

        # --- T=1002: IMU (GIROSCOPIO Y ACELERÓMETRO) ---
        elif t_code == 1002:
            imu_msg = Imu()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = self.imu_frame_id
            
            # NOTA: Ajustar claves según el JSON real del Cobra Flex. 
            # Usualmente es ax, ay, az (accel) y gx, gy, gz (gyro)
            # Gyro suele venir en grados/s o rad/s. ROS necesita RADIANES/SEGUNDO.
            
            # Ejemplo asumiendo radianes (si viene en grados, multiplicar por math.pi/180)
            imu_msg.angular_velocity.x = float(data.get("gx", 0.0))
            imu_msg.angular_velocity.y = float(data.get("gy", 0.0))
            imu_msg.angular_velocity.z = float(data.get("gz", 0.0))

            imu_msg.linear_acceleration.x = float(data.get("ax", 0.0))
            imu_msg.linear_acceleration.y = float(data.get("ay", 0.0))
            imu_msg.linear_acceleration.z = float(data.get("az", 0.0))
            
            # Covarianza de IMU (Valores típicos)
            imu_msg.orientation_covariance[0] = -1 # No tenemos orientación absoluta (magnetómetro)
            imu_msg.angular_velocity_covariance[0] = 0.001
            imu_msg.linear_acceleration_covariance[0] = 0.01

            self.imu_pub.publish(imu_msg)

    def shutdown_handler(self, signum, frame):
        self.send_json({"T":13, "X":0.0, "Z":0.0})
        self.ser.close()
        self.destroy_node()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = CobraFlexROSDriver()
    rclpy.spin(node)