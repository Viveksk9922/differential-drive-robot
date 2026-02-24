#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist

class CmdVelConverter(Node):
    def __init__(self):
        super().__init__('cmd_vel_converter')
        self.sub = self.create_subscription(
            TwistStamped, '/cmd_vel_nav', self.cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("cmd_vel_nav == cmd_vel")

    def cb(self, msg):
        twist = Twist()
        twist.linear = msg.twist.linear
        twist.angular = msg.twist.angular
        self.pub.publish(twist)

rclpy.init()
node = CmdVelConverter()
rclpy.spin(node)
