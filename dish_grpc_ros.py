#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread

import json
import pprint
from argparse import ArgumentParser

import dish_common
import starlink_grpc

from collections.abc import Mapping, Sequence


def sanitize_value(value):
    if "nan" in str(value):
        return None 
    
    return value

def handle_serialization(object):
    if isinstance(object, Mapping):
        return {k: handle_serialization(v) for k, v in object.items()}
    elif isinstance(object, Sequence) and not isinstance(object, str):
        return [handle_serialization(v) for v in object]
    
    # handle other types not yet covered
    elif not isinstance(object, str):
        return "<" + object.__class__.__name__ + ">"

    return object


class StarlinkNode(Node):
    def __init__(self, topic='starlink', pub_rate=1):
        rclpy.init()
        self.topic = topic
        self.pub_rate = pub_rate

        super().__init__('starlink_node')

        self.get_logger().info('Starting Starlink Node')
        self.get_logger().info(f'Publishing data to topic: /{self.topic} at rate: {self.pub_rate}s')

    
    def start(self):
        thread = Thread(target=self.init_pub)
        thread.start()

        rclpy.spin(self)

    def init_pub(self):
        self.pub = self.create_publisher(String, self.topic, 10)
        self.timer = self.create_timer(self.pub_rate, self.publish_data)

    def process_data(self, data):
        def recursive_process(data):
            if hasattr(data, "DESCRIPTOR") and hasattr(data.DESCRIPTOR, "fields_by_name"):
                keys = data.DESCRIPTOR.fields_by_name.keys()
                processed_data = {}
                for key in keys:
                    value = getattr(data, key)
                    processed_data[key] = recursive_process(value) if hasattr(value, "DESCRIPTOR") else sanitize_value(value)
                return processed_data
            return data # Base case: return raw value if it's not a Protobuf object
    
        processed_data = recursive_process(data)

        return processed_data

    def publish_data(self):
        try:
            self.get_logger().info('Getting status data from dish...')
            status = starlink_grpc.all_status_data(context=dish_common.GlobalState().context)
        except starlink_grpc.GrpcError:
            self.get_logger().error('Failed to get status data. Are you connected to the dish?')
            return

        processed_data = {key: self.process_data(getattr(status, key)) for key in status.DESCRIPTOR.fields_by_name.keys()}

        msg = String()
        msg.data = json.dumps(processed_data, default=handle_serialization)
        
        pprint.pprint(json.loads(msg.data))
        
        self.pub.publish(msg)

    def stop(self):
        self.get_logger().info('Stopping Starlink Node')
        self.destroy_node()
        rclpy.shutdown()


def main():
    parser = ArgumentParser()
    parser.add_argument('--pub_rate', type=float, default=1,
                        help='Rate at which to publish data to ROS from the dish')
    args = parser.parse_args()

    node = StarlinkNode(pub_rate=args.pub_rate)
    try:
        node.start()
    except KeyboardInterrupt:
        node.stop()

if __name__ == "__main__":
    main()
