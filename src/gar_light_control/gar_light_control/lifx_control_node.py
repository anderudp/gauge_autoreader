from lifxlan import LifxLAN, Light, BLUE
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from gar_interfaces.msg import SetColor, SetInfrared, SetPower


class LifxControlNode(Node):

    def __init__(self):
        super().__init__(
            "lifx_control_node"
        )

        qos = QoSProfile(depth=10)
        lifx = LifxLAN()
        devices = lifx.get_lights()
        if devices is None:
            self.get_logger().error("No devices found! Are your lights onboarded to your network?")
            return
        
        self.get_logger().info(f"Found {len(devices)} devices.")

        self.lights: list[Light] = []  # TODO: Add lights in a predetermined order by MAC address
        for device in devices:
            try:
                assert isinstance(device, Light)
                self.get_logger().info(f"Device recognized as light: {device}")
                self.lights.append(device)
            except AssertionError:
                self.get_logger().warning(f"Device NOT recognized as light: {device}")
                continue
        
        if len(self.lights) == 0:
            self.get_logger().error("None of the devices were recognized as lights.")
            return
        
        self.set_color_sub = self.create_subscription(
            SetColor,
            "set_color",
            self.cb_set_color,
            qos
        )

        self.set_infrared_sub = self.create_subscription(
            SetInfrared,
            "set_infrared",
            self.cb_set_infrared,
            qos
        )

        self.set_power_sub = self.create_subscription(
            SetPower,
            "set_power",
            self.cb_set_power,
            qos
        )
        
    
    def cb_set_color(self, msg: SetColor):
        light = self.lights[msg.id]
        if not light.supports_color():
            self.get_logger().warning(f"Light {msg.id} does not support color!")
        if not light.supports_temperature():
            self.get_logger().warning(f"Light {msg.id} does not support white color temperature!")

        max_kelvin = light.get_max_kelvin()
        min_kelvin = light.get_min_kelvin()
        if msg.kelvin > max_kelvin:  
            self.get_logger().warning(f"Invalid color temperature! Got {msg.kelvin} Kelvin, maximum is {max_kelvin}.")
            msg.kelvin = max_kelvin
        elif msg.kelvin < min_kelvin:
            self.get_logger().warning(f"Invalid color temperature! Got {msg.kelvin} Kelvin, minimum is {min_kelvin}.")
            msg.kelvin = min_kelvin
        
        color = [msg.hue, msg.saturation, msg.brightness, msg.kelvin]
        self.lights[msg.id].set_color(color)


    def cb_set_infrared(self, msg: SetInfrared):
        if not self.lights[msg.id].supports_infrared():
            self.get_logger().warning(f"Device {msg.id} does not support infrared!")
            return
        self.lights[msg.id].set_infrared(msg.ir_brightness)


    def cb_set_power(self, msg: SetPower):
        self.lights[msg.id].set_power(msg.power)


def main(args=None):
    rclpy.init(args=args)
    node = LifxControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
