import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Header
from gar_interfaces.msg import ServoCommand
from gar_interfaces.srv import SetServoControlMode, GetServoState


class ServoTesterNode(Node):

    def __init__(self):
        super().__init__(
            "servo_tester_node", 
            automatically_declare_parameters_from_overrides=True
        )

        self.control_limits = {
            "position": {
                "min": 0,
                "max": 4095,
            },
            "velocity": {
                "min": -2047,
                "max": 2047,
            }
        }

        qos = QoSProfile(depth=10)

        self.set_servo_control_mode_cli = self.create_client(
            SetServoControlMode,
            "set_servo_control_mode",
            qos_profile=qos
        )

        self.get_servo_state_cli = self.create_client(
            GetServoState,
            "get_servo_state",
            qos_profile=qos
        )

        self.servo_command_pub = self.create_publisher(
            ServoCommand,
            "servo_command",
            qos_profile=qos
        )

        self.timer = self.create_timer(1.0, self.begin_tests)

    
    def wait_for_set_servo_control_mode_srv(self):
        while not self.set_servo_control_mode_cli.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn("Waiting for set_servo_control_mode service...")


    def wait_for_get_servo_state_cli_srv(self):
        while not self.get_servo_state_cli.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn("Waiting for get_servo_state service...")


    def get_param_as_int(self, param_name: str):
            val = self.get_parameter(param_name).value
            try:
                assert isinstance(val, int)
                return val
            except AssertionError:
                self.get_logger().error(f"Type mismatch in config file! {param_name} is {type(val)} when an integer was expected.")
                return 0
    

    def begin_tests(self):
        self.timer.cancel()
        self.wait_for_set_servo_control_mode_srv()
        self.wait_for_get_servo_state_cli_srv()
        self.get_logger().info("Beginning tests.")
        self.test_1()
    

    def test_1(self):
        self.get_logger().info("Setting position to CCW limit...")
        req = SetServoControlMode.Request()
        req.id = 1
        req.mode = "position"
        res = self.set_servo_control_mode_cli.call(req)
        if res is None:
            self.get_logger().error("No response received!")
            self.finish_tests()
        elif not res.success:
            self.get_logger().error("Failed to set control mode!")
            self.finish_tests()
        else:
            cmd = ServoCommand(
                header = Header(
                    stamp=self.get_clock().now().to_msg(), 
                    frame_id="map"
                ),
                id=1,
                value=self.control_limits["position"]["min"]
            )
            self.servo_command_pub.publish(cmd)
            self.timer = self.create_timer(5.0, self.test_2)
    

    def test_2(self):
        self.timer.cancel()
        self.get_logger().info("Setting CW velocity to 1 percent...")
        req = SetServoControlMode.Request()
        req.id = 1
        req.mode = "velocity"
        res = self.set_servo_control_mode_cli.call(req)
        if res is None:
            self.get_logger().error("No response received!")
            self.finish_tests()
        elif not res.success:
            self.get_logger().error("Failed to set control mode!")
            self.finish_tests()
        else:
            cmd = ServoCommand(
                header = Header(
                    stamp=self.get_clock().now().to_msg(), 
                    frame_id="map"
                ),
                id=1,
                value=self.control_limits["velocity"]["max"] // 100
            )
            self.servo_command_pub.publish(cmd)
            self.timer = self.create_timer(5.0, self.test_3)


    def test_3(self):
        self.timer.cancel()
        self.get_logger().info("Setting CW velocity to 10 percent...")
        
        cmd = ServoCommand(
            header = Header(
                stamp=self.get_clock().now().to_msg(), 
                frame_id="map"
            ),
            id=1,
            value=self.control_limits["velocity"]["max"] // 10
        )
        self.servo_command_pub.publish(cmd)
        self.timer = self.create_timer(5.0, self.test_4)

    
    def test_4(self):
        self.timer.cancel()
        self.get_logger().info("Setting CW velocity to 50 percent...")
        
        cmd = ServoCommand(
            header = Header(
                stamp=self.get_clock().now().to_msg(), 
                frame_id="map"
            ),
            id=1,
            value=self.control_limits["velocity"]["max"] // 2
        )
        self.servo_command_pub.publish(cmd)
        self.timer = self.create_timer(5.0, self.test_5)

    
    def test_5(self):
        self.timer.cancel()
        self.get_logger().info("Setting position to CW limit...")
        req = SetServoControlMode.Request()
        req.id = 1
        req.mode = "position"
        res = self.set_servo_control_mode_cli.call(req)
        if res is None:
            self.get_logger().error("No response received!")
            self.finish_tests()
        elif not res.success:
            self.get_logger().error("Failed to set control mode!")
            self.finish_tests()
        else:
            cmd = ServoCommand(
                header = Header(
                    stamp=self.get_clock().now().to_msg(), 
                    frame_id="map"
                ),
                id=1,
                value=self.control_limits["position"]["max"]
            )
            self.servo_command_pub.publish(cmd)
            self.timer = self.create_timer(5.0, self.test_6)


    def test_6(self):
        self.timer.cancel()
        self.get_logger().info("Setting CCW velocity to 1 percent...")
        req = SetServoControlMode.Request()
        req.id = 1
        req.mode = "velocity"
        res = self.set_servo_control_mode_cli.call(req)
        if res is None:
            self.get_logger().error("No response received!")
            self.finish_tests()
        elif not res.success:
            self.get_logger().error("Failed to set control mode!")
            self.finish_tests()
        else:
            cmd = ServoCommand(
                header = Header(
                    stamp=self.get_clock().now().to_msg(), 
                    frame_id="map"
                ),
                id=1,
                value=self.control_limits["velocity"]["min"] // 100
            )
            self.servo_command_pub.publish(cmd)
            self.timer = self.create_timer(5.0, self.test_7)

    
    def test_7(self):
        self.timer.cancel()
        self.get_logger().info("Setting CCW velocity to 10 percent...")
        
        cmd = ServoCommand(
            header = Header(
                stamp=self.get_clock().now().to_msg(), 
                frame_id="map"
            ),
            id=1,
            value=self.control_limits["velocity"]["min"] // 10
        )
        self.servo_command_pub.publish(cmd)
        self.timer = self.create_timer(5.0, self.test_8)

    
    def test_8(self):
        self.timer.cancel()
        self.get_logger().info("Setting CCW velocity to 50 percent...")
        
        cmd = ServoCommand(
            header = Header(
                stamp=self.get_clock().now().to_msg(), 
                frame_id="map"
            ),
            id=1,
            value=self.control_limits["velocity"]["min"] // 2
        )
        self.servo_command_pub.publish(cmd)
        self.timer = self.create_timer(5.0, self.test_9)

    
    def test_9(self):
        self.timer.cancel()
        self.get_logger().info("Setting position to center...")
        req = SetServoControlMode.Request()
        req.id = 1
        req.mode = "position"
        res = self.set_servo_control_mode_cli.call(req)
        if res is None:
            self.get_logger().error("No response received!")
            self.finish_tests()
        elif not res.success:
            self.get_logger().error("Failed to set control mode!")
            self.finish_tests()
        else:
            cmd = ServoCommand(
                header = Header(
                    stamp=self.get_clock().now().to_msg(), 
                    frame_id="map"
                ),
                id=1,
                value=(self.control_limits["position"]["max"] + self.control_limits["position"]["min"]) // 2
            )
            self.servo_command_pub.publish(cmd)
            self.timer = self.create_timer(5.0, self.finish_tests)


    def finish_tests(self):
        self.timer.cancel()
        self.get_logger().info("Tests completed, shutting down node...")
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoTesterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()