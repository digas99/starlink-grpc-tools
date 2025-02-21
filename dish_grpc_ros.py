#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from threading import Thread
import json
from std_msgs.msg import String

import dish_common
import starlink_grpc


class StarlinkNode(Node):
    def __init__(self, topic='starlink'):
        rclpy.init()
        self.topic = topic

        super().__init__('starlink_node')

    
    def start(self):
        thread = Thread(target=self.init_pub)
        thread.start()

        rclpy.spin(self)

    def init_pub(self):
        self.pub = self.create_publisher(String, self.topic, 10)
        self.timer = self.create_timer(1, self.publish_data)

    def publish_data(self):
        status = starlink_grpc.all_status_data(context=dish_common.GlobalState().context)
        data = {
            "seconds_to_first_nonempty_slot": getattr(status, "seconds_to_first_nonempty_slot", None),
            "pop_ping_drop_rate": getattr(status, "pop_ping_drop_rate", None),
            "downlink_throughput_bps": getattr(status, "downlink_throughput_bps", None),
            "uplink_throughput_bps": getattr(status, "uplink_throughput_bps", None),
            "pop_ping_latency_ms": getattr(status, "pop_ping_latency_ms", None)
        }
        msg = String()
        msg.data = json.dumps(data)
        print(data)
        self.pub.publish(msg)

    def stop(self):
        self.get_logger().info('Stopping Starlink Node')
        self.destroy_node()
        rclpy.shutdown()


def main():
    node = StarlinkNode()
    try:
        node.start()
    except KeyboardInterrupt:
        node.stop()

if __name__ == "__main__":
    main()
