#!/usr/bin/env python3
from argparse import ArgumentParser

from ros.starlink_ros_node import StarlinkNode
import dish_grpc_all_data as starlink_data


def main():
    parser = ArgumentParser()
    parser.add_argument('--pub_rate', type=float, default=1,
                        help='Rate at which to publish data to ROS from the dish')
    args = parser.parse_args()

    node = StarlinkNode(pub_rate=args.pub_rate)
    try:
        node.start(callback=starlink_data.fetch_data)
    except KeyboardInterrupt:
        node.stop()

if __name__ == "__main__":
    main()
