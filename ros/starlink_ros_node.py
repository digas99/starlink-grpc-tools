#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread

class StarlinkNode(Node):
    def __init__(self, topic='starlink', pub_rate=1, callback=None):
        rclpy.init()
        self.topic = topic
        self.pub_rate = pub_rate
        self.callback = callback

        super().__init__('starlink_node')

        self.get_logger().info('Starting Starlink Node')
        self.get_logger().info(f'Publishing data to topic: /{self.topic} at rate: {self.pub_rate}s')

    
    def start(self, callback=None):
        if callback:
            self.callback = callback

        thread = Thread(target=self.init_pub)
        thread.start()

        rclpy.spin(self)

    def init_pub(self):
        self.pub = self.create_publisher(String, self.topic, 10)
        self.timer = self.create_timer(self.pub_rate, self.publish_data)

    def publish_data(self):
        if not self.callback:
            self.get_logger().error('No data callback function found')

        msg = String()
        msg.data = self.callback()
        self.pub.publish(msg)

    def stop(self):
        self.get_logger().info('Stopping Starlink Node')
        self.destroy_node()
        rclpy.shutdown()