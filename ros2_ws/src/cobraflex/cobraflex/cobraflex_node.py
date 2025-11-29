"""Entry point for the cobraflex control node."""

import rclpy
from rclpy.node import Node


class CobraflexNode(Node):
    """Placeholder node for Cobra chassis control logic."""

    def __init__(self) -> None:
        super().__init__('cobraflex')
        self.get_logger().info('cobraflex node initialized. Add publishers/subscribers here.')


def main() -> None:
    """Initialize and spin the cobraflex node."""
    rclpy.init()
    node = CobraflexNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
