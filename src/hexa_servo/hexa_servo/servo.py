#!/usr/bin/env python3
import math
import serial
import pprint

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .lewansoul_lx16a_controller import ServoController


EPSILON = 0.001
POSE_ITEMS = 18

prev_position = POSE_ITEMS * [0.0]


def compare_poses(old_pose, new_pos):
    for item in range(POSE_ITEMS):
        if abs(new_pos[item] - old_pose[item]) > EPSILON:
            return False

    return True 


def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class Servo(Node):

    def __init__(self):
        super().__init__("servo")

        self.joint_sub = self.create_subscription(
            JointState, 
            "/joint_states", 
            self.servo_callback, 
            10
        )
        self.init_servos()

    def init_servos(self):
        self.ctrl = ServoController(
            serial.Serial(
                "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
                9600,
                timeout=2),
            timeout=5
        )

    def servo_callback(self, msg: JointState):
        global prev_position
        poses = {}

        for id, pos in enumerate(msg.position):
            if math.isnan(pos):
                pos = 0.0

            steps = round(
                map_value(math.degrees(pos), -120, 120, 0, 1000)
            )
            poses[id+1] = steps

        if not compare_poses(msg.position, prev_position):
            self.ctrl.move(poses, time=50)

        prev_position = msg.position


def main(args=None):
    rclpy.init(args=args)
    node = Servo()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__()":
    main()