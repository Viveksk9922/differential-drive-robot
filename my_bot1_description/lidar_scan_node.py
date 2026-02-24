#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarScanNode(Node):
    def __init__(self):
        super().__init__("lidar_scan_node")

        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('target_frame', 'lidar_1')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value

        self.publisher = self.create_publisher(LaserScan,"/scan",20)
        self.subscriber = self.create_subscription(LaserScan,"/scan_raw",self.callback_topic,10)

        self.get_logger().info(f'Remapping {input_topic} → {output_topic} with frame_id = {self.target_frame}')

    def callback_topic(self, msg):
        msg.header.frame_id = self.target_frame
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()