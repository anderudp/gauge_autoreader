from lifxlan import LifxLAN
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


class LifxDualControlNode(Node):

    def __init__(self):
        super().__init__(
            "lifx_dual_control_node"
        )

        qos = QoSProfile(depth=10)
        self.lifx = LifxLAN()
        lights = self.lifx.get_lights()
        if lights is None:
            self.get_logger().error("No lights found! Is your light onboarded on your network?")
        else:
            self.get_logger().info(f"Found {len(lights)} lights.")



def main(args=None):
    rclpy.init(args=args)
    node = LifxDualControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
