# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node


from std_msgs.msg import Int64
# from std_msgs.msg import Int32MultiArray
import os
from dynamixel_sdk import * # Uses Dynamixel SDK library

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

MY_DXL = 'XL430'        # [WARNING] Operating Voltage : 7.4V


# Control table address
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 1000000
ADDR_OPERATING_MODE = 11
ADDR_GOAL_VELOCITY = 104
ADDR_PRESENT_VELOCITY = 128

PROTOCOL_VERSION            = 2.0
DXL_ID1                      = 1
DXL_ID2                      = 2
DEVICENAME                  = '/dev/ttyACM0'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

class MotorController_sub(Node):

    def __init__(self):
        super().__init__('motor_control')
        self.subscription = self.create_subscription(
            Int64,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        #初始化馬達連接
        # self.portHandler = PortHandler(DEVICENAME)
        # self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        # Set port baudrate
        if portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        # dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")


    def listener_callback(self, msg):
        # speed_ID1,speed_ID2 = msg.data
        speed_ID1 = msg.data
        # self.get_logger().info(f'Received speed command: ID1: {speed_ID1},speed_ID2: {speed_ID2}')
        self.get_logger().info(f'Received speed command: ID1: {speed_ID1}')
        # self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID1, ADDR_GOAL_VELOCITY, int(speed))
        packetHandler.write4ByteTxRx(portHandler, DXL_ID1, ADDR_GOAL_VELOCITY, int(speed_ID1))
        # packetHandler.write4ByteTxRx(portHandler, DXL_ID2, ADDR_GOAL_VELOCITY, int(speed_ID2))
        # self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID2, ADDR_GOAL_VELOCITY, int(speed))

    def destroy(self):
        self.disable_torque(DXL_ID1)
        # self.disable_torque(DXL_ID2)
        self.portHandler.closePort()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_control = MotorController_sub()
    try:
        rclpy.spin(motor_control)
    except KeyboardInterrupt:
        pass
    motor_control.destroy()
    rclpy.shutdown()


    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    # dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))


if __name__ == '__main__':
    main()
