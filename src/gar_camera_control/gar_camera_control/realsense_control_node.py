import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


class RealsenseControlNode(Node):
    def __init__(self):
        super().__init__("realsense_control_node")


def main(args=None):
    rclpy.init(args=args)
    node = RealsenseControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
