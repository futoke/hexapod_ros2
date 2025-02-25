#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from itertools import product, cycle


values = [-0.8, 0.0, 0.8]
combinations = list(product(values, repeat=2))
cyclic_combinations = cycle(combinations)


class FakeSLAM(Node):
    def __init__(self):
        super().__init__("fake_slam")
        self.publisher_ = self.create_publisher(Joy, "joy", 10)

        timer_period = 5.0
        self.timer = self.create_timer(timer_period, self.publish_message)

    def publish_message(self):
        msg = Joy()
        x, y = next(cyclic_combinations)
        msg.axes = [0.0, 0.0, x, y, 1.0, 1.0, 0.0, 0.0]
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.axes}")


def main(args=None):
    rclpy.init(args=args)
    node = FakeSLAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        msg = Joy()
        msg.axes = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0]
        node.publisher_.publish(msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()