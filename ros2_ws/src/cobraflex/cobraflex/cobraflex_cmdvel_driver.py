#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import json


class CobraFlexDriver(Node):
    """
    Nodo basado en tu plantilla original, pero adaptado
    para convertir /cmd_vel -> JSON para el Cobraflex.
    """

    def __init__(self):
        super().__init__("cobraflex_cmdvel_driver")

        # -----------------------------
        # Parámetros configurables
        # -----------------------------
        self.declare_parameter("port", "/dev/ttyACM1")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("max_speed_value", 100)   # rango [0–100]
        self.declare_parameter("max_linear", 1.0)        # m/s
        self.declare_parameter("max_angular", 1.0)       # rad/s

        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value

        # Escalas de normalización
        self.max_speed_val = float(self.get_parameter("max_speed_value").value)
        self.max_lin = float(self.get_parameter("max_linear").value)
        self.max_ang = float(self.get_parameter("max_angular").value)

        # -----------------------------
        # Abrir puerto serie
        # -----------------------------
        self.ser = serial.Serial(port, baud, timeout=0.05)

        # -----------------------------
        # Subscripción a /cmd_vel
        # -----------------------------
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        self.get_logger().info("Cobraflex driver inicializado y escuchando /cmd_vel.")


    # ---------------------------------------------------------------------
    # Normalización de velocidades
    # ---------------------------------------------------------------------
    def scale_to_motor(self, lin_x, ang_z):
        """
        Convierte velocidades ROS2 -> escala Cobraflex.
        lin_x: m/s
        ang_z: rad/s

        Devuelve L y R entre [-max_speed_val, max_speed_val].
        """

        # Normalizar lineal y angular
        lin_norm = max(-1.0, min(1.0, lin_x / self.max_lin))
        ang_norm = max(-1.0, min(1.0, ang_z / self.max_ang))

        # Mezcla diferencial (estilo tank-drive)
        left = lin_norm - ang_norm
        right = lin_norm + ang_norm

        # Normalizar mezcla a [-1, 1]
        max_abs = max(abs(left), abs(right), 1)
        left /= max_abs
        right /= max_abs

        # Escalar al rango del Cobraflex
        L_val = int(left * self.max_speed_val)
        R_val = int(right * self.max_speed_val)

        return L_val, R_val


    # ---------------------------------------------------------------------
    # Callback de /cmd_vel
    # ---------------------------------------------------------------------
    def cmd_vel_callback(self, msg):
        lin = msg.linear.x
        ang = msg.angular.z

        # Convertir velocidades
        L, R = self.scale_to_motor(lin, ang)

        # Armar JSON final según tu plantilla
        data = {
            "T": 1,
            "L": L,
            "R": R
        }

        json_data = (json.dumps(data) + "\n").encode("utf-8")

        try:
            self.ser.write(json_data)
            self.get_logger().info(f"Enviado: {json_data}")
        except Exception as e:
            self.get_logger().error(f"ERROR enviando al Cobraflex: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CobraFlexDriver()

    # Igual que tu plantilla: spin_once para garantizar timers / callbacks
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
