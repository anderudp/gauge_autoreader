import rclpy
from rclpy.node import Node
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition


class CalibratorNode(Node):
    def __init__(self):
        super().__init__("calibrator_node")


    def cb_set_position(self):
        msg = SetPosition()


def main(args=None):
    rclpy.init(args=args)
    node = CalibratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
