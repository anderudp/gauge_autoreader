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


from dynamixel_sdk import COMM_SUCCESS
from dynamixel_sdk import PacketHandler
from dynamixel_sdk import PortHandler
from gar_interfaces.msg import SetPosition, SetVelocity
from gar_interfaces.srv import GetPosition, GetVelocity
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from dataclasses import dataclass


@dataclass
class ControlModeConfig:
    control_mode_id: int
    present_addr: int
    goal_addr: int
    min_val: int
    max_val: int


class ReadWriteNode(Node):

    def __init__(self):
        super().__init__(
            'read_write_node', 
            automatically_declare_parameters_from_overrides=True
        )

        # Control mode starts out uninitialized
        self.present_control_mode = ""

        # Grab servo-specific control parameters for each desired control mode from config file
        self.control_modes = {
            "position": ControlModeConfig(
                control_mode_id=self.get_param_as_int("POSITION_CONTROL"),
                present_addr=self.get_param_as_int("ADDR_PRESENT_POSITION"),
                goal_addr=self.get_param_as_int("ADDR_GOAL_POSITION"),
                min_val=self.get_param_as_int("MIN_POSITION_LIMIT"),
                max_val=self.get_param_as_int("MAX_POSITION_LIMIT"),
            ),
            "velocity": ControlModeConfig(
                control_mode_id=self.get_param_as_int("VELOCITY_CONTROL"),
                present_addr=self.get_param_as_int("ADDR_PRESENT_VELOCITY"),
                goal_addr=self.get_param_as_int("ADDR_GOAL_VELOCITY"),
                min_val=self.get_param_as_int("MIN_VELOCITY_LIMIT"),
                max_val=self.get_param_as_int("MAX_VELOCITY_LIMIT"),
            ),
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

        # Declare internode comms channels
        qos = QoSProfile(depth=10)

        self.set_position_sub = self.create_subscription(
            SetPosition,
            'set_position',
            self.cb_set_position,
            qos
        )

        self.set_velocity_sub = self.create_subscription(
            SetVelocity,
            'set_velocity',
            self.cb_set_velocity,
            qos
        )

        self.get_position_srv = self.create_service(
            GetPosition, 
            'get_position', 
            self.cb_get_position,
            qos_profile=qos
        )

        self.get_velocity_srv = self.create_service(
            GetVelocity, 
            'get_velocity', 
            self.cb_get_velocity,
            qos_profile=qos
        )


    def get_param_as_int(self, param_name: str):
        val = self.get_parameter(param_name).value
        try:
            assert isinstance(val, int)
            return val
        except AssertionError:
            self.get_logger().error(f"Type mismatch in config file! {param_name} is {type(val)} when an integer was expected.")
            return 0
    

    def get_param_as_float(self, param_name: str):
        val = self.get_parameter(param_name).value
        try:
            assert isinstance(val, float)
            return val
        except AssertionError:
            self.get_logger().error(f"Type mismatch in config file! {param_name} is {type(val)} when a float was expected.")
            return 0.0


    def get_param_as_str(self, param_name: str):
        val = self.get_parameter(param_name).value
        try:
            assert isinstance(val, str)
            return val
        except AssertionError:
            self.get_logger().error(f"Type mismatch in config file! {param_name} is {type(val)} when a string was expected.")


    def change_control_mode(self, target_mode: str):
        # Disable torque
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, 
            self.dxl_id, 
            self.addr_torque_enable, 
            self.val_torque_disable
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to disable torque: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info('Torque disabled successfully.')

        # Change mode
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, 
            self.dxl_id, 
            self.addr_operating_mode, 
            self.control_modes[target_mode].control_mode_id
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to set control mode: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info(f'Control mode set to {target_mode} successfully.')

        # Re-enable torque
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, 
            self.dxl_id, 
            self.addr_torque_enable, 
            self.val_torque_enable
        )
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Failed to enable torque: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        else:
            self.get_logger().info('Torque enabled successfully.')


    def cb_set_position(self, msg):
        self.set_servo_value(msg.id, msg.position, "position")

    
    def cb_set_velocity(self, msg):
        self.set_servo_value(msg.id, msg.velocity, "velocity")

    
    def set_servo_value(self, target_servo, target_value, target_mode):
        if target_mode != self.present_control_mode:
            self.change_control_mode(target_mode)
        
        expected_min_val = self.control_modes[target_mode].min_val
        expected_max_val = self.control_modes[target_mode].max_val
        if not expected_min_val <= target_value <= expected_max_val:
            self.get_logger().error(f"Invalid {target_mode}! Expected {expected_min_val} to {expected_max_val}, received {target_value}.")
            return

        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, 
            target_servo, 
            self.control_modes[target_mode].goal_addr, 
            target_value
        )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'Error: {self.packet_handler.getRxPacketError(dxl_error)}')
        else:
            self.get_logger().info(f"Set servo {target_servo} to target {target_mode} {target_value} successfully.")


    def cb_get_position(self, request, response):
        pos = self.get_servo_value(request.id, "position")
        response.position = pos
        return response
    

    def cb_get_velocity(self, request, response):
        vel = self.get_servo_value(request.id, "velocity")
        response.velocity = vel
        return response


    def get_servo_value(self, query_servo, query_mode):
        dxl_present_value, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, 
            query_servo, 
            self.control_modes[query_mode].present_addr
        )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'Error: {self.packet_handler.getRxPacketError(dxl_error)}')
        else:
            self.get_logger().info(f"Received {query_mode} of servo {query_servo} successfully. Value: {dxl_present_value}")

        return dxl_present_value


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