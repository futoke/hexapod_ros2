#!/usr/bin/env python3
import math
import serial
import pprint

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .lewansoul_lx16a_controller import ServoController


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
                "/dev/ttyS0",
                9600,
                timeout=2),
            timeout=5
        )

    def servo_callback(self, msg: JointState):
        poses = {}

        for id, pos in enumerate(msg.position):
            steps = round(
                map_value(math.degrees(pos), -120, 120, 0, 1000)
            )
            poses[id+1] = steps

        self.ctrl.move(poses, time=50)
        pprint.pprint(poses)


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