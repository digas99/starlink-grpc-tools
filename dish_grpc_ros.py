#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from threading import Thread
import json
from std_msgs.msg import String

import dish_common
import starlink_grpc

from collections.abc import Mapping, Sequence

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


    # TODO: HANDLE ELSE CASE TO DISPLAY NAME OF CLASS
    def handle_serialization(self, object):
        if isinstance(object, Mapping):
            return {k: self.handle_serialization(v) for k, v in object.items()}
        elif isinstance(object, Sequence) and not isinstance(object, str):
            return [self.handle_serialization(v) for v in object]
        return object

    def process_data(self, data):
        def recursive_process(data):
            if hasattr(data, "DESCRIPTOR") and hasattr(data.DESCRIPTOR, "fields_by_name"):
                keys = data.DESCRIPTOR.fields_by_name.keys()
                processed_data = {}
                for key in keys:
                    value = getattr(data, key)
                    processed_data[key] = recursive_process(value) if hasattr(value, "DESCRIPTOR") else value
                return processed_data
            return data  # Base case: return raw value if it's not a Protobuf object
    
        processed_data = recursive_process(data)

        return processed_data

    def publish_data(self):
        status = starlink_grpc.all_status_data(context=dish_common.GlobalState().context)
        msg = String()

        processed_data = {}
        keys = status.DESCRIPTOR.fields_by_name.keys()
        for key in keys:
            processed_data[key] = self.process_data(getattr(status, key))

        # pprint.pprint(processed_data)
        msg.data = json.dumps(processed_data, default=self.handle_serialization)
        print(msg.data)
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
