import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from gar_interfaces.msg import SetPosition, SetVelocity
from gar_interfaces.srv import GetPosition, GetVelocity
from pynput import keyboard


class CalibratorNode(Node):

    def __init__(self):
        super().__init__("calibrator_node")

        self.key_table = {
            "left": False,
            "right": False,
            "shift": False,
            "enter": False
        }

        self.torque = 0

        qos = QoSProfile(depth=10)

        self.kb_listener = keyboard.Listener(
            self.cb_key_press,
            self.cb_key_release
        )


    def cb_key_press(self, key):
        self.key_set(key, True)

    def cb_key_release(self, key):
        self.key_set(key, False)

    def key_set(self, key, value):
        match key:
            case keyboard.Key.left:
                self.key_table["left"] = value
            case keyboard.Key.right:
                self.key_table["right"] = value
            case keyboard.Key.shift:
                self.key_table["shift"] = value
            case keyboard.Key.enter:
                self.key_table["enter"] = value

    def calibrate(self):
        self.get_logger().info("""Setting first limit. Use the left and right arrow keys 
                               to set the needle to the furthest valid CCW position. 
                               Hold shift for faster rotation. Press Enter when done.""")
        
        # Publish torque per key_table, while Enter not pressed


def main(args=None):
    rclpy.init(args=args)
    node = CalibratorNode()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
