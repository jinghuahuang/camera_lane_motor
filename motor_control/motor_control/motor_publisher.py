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

# from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray

class MotorController_pub(Node):

    def __init__(self):
        super().__init__('motor_control_pub')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'motor_speed', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.speed_ID1 = 0  # 預設速度
        # self.speed_ID2 = 0  # 預設速度

    def timer_callback(self):
        speed_ID1 = 20
        speed_ID2 = 60
        msg = Int32MultiArray()
        msg.data = [speed_ID1,speed_ID2]
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Published speeds -> Left: {self.speed_ID1}, Right: {self.speed_ID2}')
        self.get_logger().info(f'Published speeds Left: {speed_ID1},Right: {speed_ID2}')

def main(args=None):
    rclpy.init(args=args)

    speed_publisher = MotorController_pub()

    rclpy.spin(speed_publisher)

    # try:
    #     while True:
    #         left_speed = int(input("Enter left wheel speed: "))
    #         # right_speed = int(input("Enter right wheel speed: "))
    #         speed_publisher.timer_callback(left_speed)
    # except KeyboardInterrupt:
    #     pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    speed_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
