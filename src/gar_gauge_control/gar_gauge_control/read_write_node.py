#!/usr/bin/env python3

# Copyright 2021 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Wonho Yun, Will Son

# CHANGELOG
# Added velocity control mode functionality

from dynamixel_sdk import COMM_SUCCESS
from dynamixel_sdk import PacketHandler
from dynamixel_sdk import PortHandler
from gar_interfaces.msg import ServoCommand
from gar_interfaces.srv import GetServoState
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


class ReadWriteNode(Node):

    def __init__(self):
        super().__init__(
            'read_write_node', 
            automatically_declare_parameters_from_overrides=True
        )

        # Grab servo-specific control parameters for each desired control mode from config file
        self.control_mode_vars = {
            "position": {
                "control_mode": self.get_param_as_int("POSITION_CONTROL"), 
                "present": self.get_param_as_int("ADDR_PRESENT_POSITION"), 
                "goal": self.get_param_as_int("ADDR_GOAL_POSITION"),
                "min": self.get_param_as_int("MIN_POSITION_LIMIT"),
                "max": self.get_param_as_int("MAX_POSITION_LIMIT")
            },
            "velocity": {
                "control_mode": self.get_param_as_int("POSITION_CONTROL"), 
                "present": self.get_param_as_int("ADDR_PRESENT_VELOCITY"), 
                "goal": self.get_param_as_int("ADDR_GOAL_VELOCITY"),
                "min": self.get_param_as_int("MIN_VELOCITY_LIMIT"),
                "max": self.get_param_as_int("MAX_VELOCITY_LIMIT")
            }
        }

        self.addr_operating_mode = self.get_param_as_int("ADDR_OPERATING_MODE")
        self.dxl_id = self.get_param_as_int("DXL_ID")
        self.addr_torque_enable = self.get_param_as_int("ADDR_TORQUE_ENABLE")
        self.val_torque_enable = self.get_param_as_int("TORQUE_ENABLE")
        self.val_torque_disable = self.get_param_as_int("TORQUE_DISABLE")

        # Set up communication with servo
        self.port_handler = PortHandler(self.get_param_as_str("DEVICE_NAME"))
        self.packet_handler = PacketHandler(self.get_param_as_float("PROTOCOL_VERSION"))

        if not self.port_handler.openPort():
            self.get_logger().error('Failed to open the port!')
            return
        self.get_logger().info('Port opened successfully.')

        if not self.port_handler.setBaudRate(self.get_param_as_int("BAUDRATE")):
            self.get_logger().error('Failed to set the baudrate!')
            return
        self.get_logger().info('Baudrate set successfully.')

        # Initialize position control 
        self.control_mode = "position"
        self.set_control_mode(self.control_mode)

        # Declare internode comms channels
        qos = QoSProfile(depth=10)
        self.servo_command_sub = self.create_subscription(
            ServoCommand,
            'servo_command',
            self.cb_servo_command,
            qos
        )

        self.srv = self.create_service(
            GetServoState, 
            'get_servo_state', 
            self.cb_get_servo_state
        )


    def get_param_as_int(self, param_name: str):
        val = self.get_parameter("param_name").value
        try:
            assert isinstance(val, int)
            return val
        except AssertionError:
            self.get_logger().error(f"Type mismatch in config file! {param_name} is {type(val)} when an integer was expected.")
            return 0
    

    def get_param_as_float(self, param_name: str):
        val = self.get_parameter("param_name").value
        try:
            assert isinstance(val, float)
            return val
        except AssertionError:
            self.get_logger().error(f"Type mismatch in config file! {param_name} is {type(val)} when a float was expected.")
            return 0.0


    def get_param_as_str(self, param_name: str):
        val = self.get_parameter("param_name").value
        try:
            assert isinstance(val, str)
            return val
        except AssertionError:
            self.get_logger().error(f"Type mismatch in config file! {param_name} is {type(val)} when a string was expected.")


    def set_control_mode(self, mode):
        self.get_logger().info(f"Setting control mode to {mode}")
        try:
            # Disable torque
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, 
                self.dxl_id, 
                self.addr_torque_enable, 
                self.val_torque_disable
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to disable torque: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
                raise ConnectionError(dxl_error)
            else:
                self.get_logger().info('Torque disabled successfully.')

            # Set control mode
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, 
                self.dxl_id, 
                self.addr_operating_mode, 
                self.control_mode_vars[mode]["control_mode"]
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to set {mode} control mode: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
                raise ConnectionError(dxl_error)
            else:
                self.get_logger().info(f'Control mode ({mode}) set successfully.')

            # Re-enable torque
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, 
                self.dxl_id, 
                self.addr_torque_enable, 
                self.val_torque_enable
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to enable torque: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
                raise ConnectionError(dxl_error)
            else:
                self.get_logger().info('Torque enabled successfully.')
            
            # Save present control mode
            self.control_mode = mode

        except ConnectionError as err:
            self.get_logger().error(f'{err}')
            return


    def cb_servo_command(self, msg: ServoCommand):
        # Extract message
        target_servo_id = msg.id
        servo_mode = msg.mode
        target_value = msg.value
        
        # Set control mode
        while self.control_mode != servo_mode:
            self.set_control_mode(servo_mode)
        
        # Check control value validity
        expected_min = self.control_mode_vars[servo_mode]["min"]
        expected_max = self.control_mode_vars[servo_mode]["max"]
        if target_value <= expected_min or target_value >= expected_max:
            self.get_logger().error(f"Invalid contol value! Got: {target_value}, expected range: {expected_min}-{expected_max}")
            return

        goal_address = self.control_mode_vars[servo_mode]["goal"]

        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, 
            target_servo_id, 
            goal_address, 
            target_value
        )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'Error: {self.packet_handler.getRxPacketError(dxl_error)}')
        else:
            self.get_logger().info(f'[SET] Servo ID: {target_servo_id} Mode: {servo_mode} Target: {target_value}')


    def cb_get_servo_state(self, request, response):
        # Extract message
        query_servo_id = request.id
        query_mode = request.mode

        dxl_present_value, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, 
            query_servo_id, 
            self.control_mode_vars[query_mode]["present"]
        )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'Error: {self.packet_handler.getRxPacketError(dxl_error)}')
        else:
            self.get_logger().info(f'[GET] Servo ID: {query_servo_id} Mode: {query_mode} Present value: {dxl_present_value}')

        response.value = dxl_present_value
        return response


    def __del__(self):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, 
            1, 
            self.addr_torque_enable, 
            self.val_torque_disable
        )
        self.port_handler.closePort()
        self.get_logger().info('Shutting down read_write_node')


def main(args=None):
    rclpy.init(args=args)
    node = ReadWriteNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()