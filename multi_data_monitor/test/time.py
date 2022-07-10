#!/usr/bin/env python3

import rclpy
import rclpy.node
import std_msgs.msg


class MyNode(rclpy.node.Node):

    def __init__(self):
        super().__init__("talker")
        self.pub_header = self.create_publisher(std_msgs.msg.Header, "/monitor/header", 1)
        self.pub_double = self.create_publisher(std_msgs.msg.Float64, "/monitor/double", 1)
        self.count = 0
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        self.publish_header()
        self.publish_double()
        self.count += 1

    def publish_header(self):
        msg = std_msgs.msg.Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = f"test {self.count}"
        self.pub_header.publish(msg)

    def publish_double(self):
        msg = std_msgs.msg.Float64()
        msg.data = self.count * 0.1
        self.pub_double.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
