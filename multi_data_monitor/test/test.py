#!/usr/bin/env python3

import rclpy
import rclpy.node
import std_msgs.msg
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy


class MyNode(rclpy.node.Node):

    def __init__(self):
        super().__init__("talker")
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_header = self.create_publisher(std_msgs.msg.Header, "/monitor/header", 1)
        self.pub_double = self.create_publisher(std_msgs.msg.Float64, "/monitor/double", 1)
        self.pub_sint32 = self.create_publisher(std_msgs.msg.Int32, "/monitor/int", 1)
        self.pub_vector = self.create_publisher(std_msgs.msg.Int32MultiArray, "/monitor/array", qos)
        self.count = 0
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        self.publish_header()
        self.publish_double()
        self.publish_sint32()
        self.publish_vector()
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

    def publish_sint32(self):
        msg = std_msgs.msg.Int32()
        msg.data = self.count % 5
        self.pub_sint32.publish(msg)

    def publish_vector(self):
        msg = std_msgs.msg.Int32MultiArray()
        msg.data = [(i * 100) + (self.count % 10) for i in range(1, 6)]
        self.pub_vector.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
