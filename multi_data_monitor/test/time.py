#!/usr/bin/env python3
import rclpy
import rclpy.node
import std_msgs.msg

class MyNode(rclpy.node.Node):

    def __init__(self):
        super().__init__("talker")
        self.pub = self.create_publisher(std_msgs.msg.Header, "/monitor/header", 1)
        self.tmr = self.create_timer(1.0, self.callback)

    def callback(self):
        msg = std_msgs.msg.Header()
        msg.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
