import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import math

class CobraFlexOdom(Node):
    def __init__(self):
        super().__init__('cobraflex_odom')
        
        # Parámetros físicos (Ajustar según tu robot real)
        self.wheel_base = 0.154  # Distancia entre ruedas en metros
        
        # Estado del robot
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # Suscriptor a las velocidades del driver
        self.subscription = self.create_subscription(Twist, '/cobraflex/wheel_speeds', self.calc_odom, 10)
        
        # Publicadores
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def calc_odom(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # mdl y mdr de tu driver [cite: 10]
        v_left = msg.linear.x
        v_right = msg.linear.y

        # Cinemática diferencial
        v = (v_right + v_left) / 2.0
        w = (v_right - v_left) / self.wheel_base

        # Actualizar posición
        delta_x = (v * math.cos(self.th)) * dt
        delta_y = (v * math.sin(self.th)) * dt
        delta_th = w * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # Publicar Transformación TF (Odom -> Base_footprint)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)
        self.tf_broadcaster.sendTransform(t)

        # Publicar mensaje Odometry
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)
        
        self.last_time = current_time

def main():
    rclpy.init()
    node = CobraFlexOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()