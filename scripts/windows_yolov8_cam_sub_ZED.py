#!/usr/bin/env python3
#
# windows_yolov8_cam_sub_.py
#
# Subscribes to a ZED image topic, decodes it safely, runs YOLOv8 detection,
# and shows the frame with bounding boxes and labels.
#
# Requirements:
#   pip install ultralytics opencv-python numpy
#   (optional but recommended) pip install cv_bridge
#
# Works in Windows with ROS2 Humble (run inside ROS2 Command Prompt)
# Example topic: /zed/zed_node/left_raw/image_raw_color
#

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
import numpy as np
import cv2

# Try to import cv_bridge if available (helps decode ROS2 images)
try:
    from cv_bridge import CvBridge
    USE_BRIDGE = True
    bridge = CvBridge()
except ImportError:
    USE_BRIDGE = False
    print("[INFO] cv_bridge not found, using manual decoding instead.")


class Viewer(Node):
    def __init__(self):
        super().__init__('viewer_detect')

        # Load YOLOv8 nano (lightest model)
        self.model = YOLO('yolov8n.pt')
        self.palette = np.random.randint(0, 255, size=(80, 3), dtype=np.uint8)

        # ZED topic (adjust if needed)
        topic = '/zed/zed_node/left_raw/image_raw_color'
        self.sub = self.create_subscription(Image, topic, self.cb, 10)
        self.get_logger().info(f"Subscribed to {topic}")

    def decode_image(self, msg: Image):
        """Decode ROS2 Image to OpenCV BGR format."""
        if USE_BRIDGE:
            try:
                return bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().warn(f"CvBridge failed: {e}")
                return None

        # Manual fallback if cv_bridge is not installed
        enc = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if enc in ('bgr8', 'rgb8'):
            img = data.reshape(msg.height, msg.width, 3)
            if enc == 'rgb8':
                img = img[..., ::-1]
        elif enc == 'rgba8':
            img = data.reshape(msg.height, msg.width, 4)[..., :3]
        elif enc in ('yuv422', 'yuyv', 'yuv422_yuy2'):
            yuv = data.reshape(msg.height, msg.width * 2)
            img = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUY2)
        else:
            self.get_logger().warn(f"Unsupported encoding: {enc}")
            return None
        return img

    def cb(self, msg: Image):
        img = self.decode_image(msg)
        if img is None:
            return

        # Run YOLOv8 (expects RGB input)
        results = self.model(img[..., ::-1], verbose=False)[0]

        # Draw boxes
        for box, cls, conf in zip(results.boxes.xyxy,
                                  results.boxes.cls,
                                  results.boxes.conf):
            x1, y1, x2, y2 = map(int, box)
            c = int(cls)
            color = tuple(int(v) for v in self.palette[c])
            label = f'{self.model.names[c]} {conf:.2f}'

            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(img, (x1, y1 - th - 4), (x1 + tw, y1), color, -1)
            cv2.putText(img, label, (x1, y1 - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1, cv2.LINE_AA)

        # Display
        cv2.imshow('ZED + YOLOv8', img)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = Viewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
