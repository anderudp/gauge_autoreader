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

from const_xc330_t181 import *
from dynamixel_sdk import COMM_SUCCESS
from dynamixel_sdk import PacketHandler
from dynamixel_sdk import PortHandler
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition
from gar_interfaces.msg import ServoCommand
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


class ReadWriteNode(Node):

    def __init__(self):
        super().__init__('read_write_node')

        self.control_mode = "position"

        self.control_vals = {
            "position": {
                "control_mode": POSITION_CONTROL, 
                "present": ADDR_PRESENT_POSITION, 
                "goal": ADDR_GOAL_POSITION
            },
            "velocity": {
                "control_mode": POSITION_CONTROL, 
                "present": ADDR_PRESENT_VELOCITY, 
                "goal": ADDR_GOAL_VELOCITY
            }
        }

        self.port_handler = PortHandler(DEVICE_NAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error('Failed to open the port!')
            return
        self.get_logger().info('Port opened successfully.')

        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error('Failed to set the baudrate!')
            return
        self.get_logger().info('Baudrate set successfully.')

        self.set_control_mode(POSITION_CONTROL)
        qos = QoSProfile(depth=10)

        self.servo_command_sub = self.create_subscription(
            ServoCommand,
            'servo_command',
            self.cb_servo_command,
            qos
        )

        self.srv = self.create_service(GetPosition, 'get_position', self.get_position_callback)


    def set_control_mode(self, mode):
        try:
            # Disable torque
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to disable torque: \
                                        {self.packet_handler.getTxRxResult(dxl_comm_result)}')
                raise ConnectionError(dxl_error)
            else:
                self.get_logger().info('Torque disabled successfully.')

            # Set control mode
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, DXL_ID, ADDR_OPERATING_MODE, self.control_vals[mode]["control_mode"]
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to set {mode} control mode: \
                                        {self.packet_handler.getTxRxResult(dxl_comm_result)}')
                raise ConnectionError(dxl_error)
            else:
                self.get_logger().info(f'Control mode ({mode}) set successfully.')

            # Re-enable torque
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
            )
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to enable torque: \
                                        {self.packet_handler.getTxRxResult(dxl_comm_result)}')
                raise ConnectionError(dxl_error)
            else:
                self.get_logger().info('Torque enabled successfully.')
            
            # Save current control mode
            self.control_mode = mode

        except ConnectionError as err:
            self.get_logger().error(f'{err}')
            return


    def cb_servo_command(self, msg:ServoCommand):
        # Set control mode
        while self.control_mode != msg.mode:
            self.set_control_mode(msg.mode)

        servo_target = msg.value
        goal_address = self.control_vals[self.control_mode]["goal"]

        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, DXL_ID, goal_address, servo_target
        )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Error: \
                                    {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'Error: {self.packet_handler.getRxPacketError(dxl_error)}')
        else:
            self.get_logger().info(f'Set {self.control_mode} to target: {msg.value}')

    def get_position_callback(self, request, response):
        dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, request.id, ADDR_PRESENT_POSITION
        )

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'Error: {self.packet_handler.getTxRxResult(dxl_comm_result)}')
        elif dxl_error != 0:
            self.get_logger().error(f'Error: {self.packet_handler.getRxPacketError(dxl_error)}')
        else:
            self.get_logger().info(f'Get [ID: {request.id}] \
                                   [Present Position: {dxl_present_position}]')

        response.position = dxl_present_position
        return response

    def __del__(self):
        self.packet_handler.write1ByteTxRx(self.port_handler,
                                           1,
                                           ADDR_TORQUE_ENABLE,
                                           TORQUE_DISABLE)
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