#!/usr/bin/env python3
#
# lidar_zed_distance_comparison.py
#
# Subscribes to both the RPLIDAR and ZED depth topics, and compares the reported distances.
# It prints the difference and checks if the distances are within an acceptable range.
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
import numpy as np

try:
    from cv_bridge import CvBridge
    bridge = CvBridge()
except ImportError:
    bridge = None
    print("[INFO] cv_bridge not found, using manual decoding for ZED depth.")


class DistanceComparison(Node):
    def __init__(self):
        super().__init__('lidar_zed_distance_comparison')

        self.lidar_range = None
        self.depth_range = None

        # Suscribirse al topic del LiDAR
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.cb_lidar, 10)

        # Suscribirse al topic de profundidad de ZED
        self.sub_depth = self.create_subscription(Image, '/zed/zed_node/depth/depth_registered', self.cb_depth, 10)

        # Crear un timer para comparar cada segundo
        self.timer = self.create_timer(1.0, self.compare)

    def cb_lidar(self, msg: LaserScan):
        # Extraer las distancias del LiDAR
        ranges = np.array(msg.ranges)
        valid = np.logical_and(ranges > msg.range_min, ranges < msg.range_max)

        # Tomar el rango posterior (180°-360°)
        center_idx = len(ranges) // 2
        # Establecer un rango de 180° desde el centro (por ejemplo, tomar desde el 180° hasta el 360°)
        width = int((180.0 * np.pi / 180.0) / msg.angle_increment)  # 180°
        window = ranges[center_idx - width:center_idx + width]
        window = window[np.isfinite(window)]  # Filtrar valores inválidos

        if len(window) > 0:
            self.lidar_range = np.mean(window)  # Promedio de la ventana

    def cb_depth(self, msg: Image):
        if bridge:
            depth = bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        else:
            data = np.frombuffer(msg.data, dtype=np.float32)
            depth = data.reshape(msg.height, msg.width)

        # Tomar la línea central
        row = depth[depth.shape[0] // 2, :]
        valid = (row > 0.1) & (row < 20.0)  # Filtrar valores no válidos
        row = row[valid]

        if len(row) > 0:
            self.depth_range = np.nanmean(row[depth.shape[1] // 2 - 10 : depth.shape[1] // 2 + 10])  # Rango promedio

    def compare(self):
        # Asegurarse de que ambos sensores tengan datos válidos
        if self.lidar_range is None or self.depth_range is None:
            return

        # Calcular la diferencia
        diff = abs(self.lidar_range - self.depth_range)
        self.get_logger().info(
            f"LIDAR: {self.lidar_range:.2f} m | ZED Depth: {self.depth_range:.2f} m | Δ = {diff:.2f} m"
        )

        # Verificar si la diferencia es aceptable (por ejemplo, dentro de un rango de 0.1 m)
        if diff > 0.1:
            self.get_logger().warn(f"Warning: Distance difference exceeds threshold ({diff:.2f} m)!")
        else:
            self.get_logger().info(f"Distances are consistent (Δ = {diff:.2f} m)")


def main():
    rclpy.init()
    node = DistanceComparison()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

