#!/usr/bin/env python3

import signal
import sys
import json
import time

from argparse import ArgumentParser
from pymongo import MongoClient, errors

import dish_grpc_all_data as starlink_data

MONGODB_HOST = 'localhost'
MONGODB_PORT = 27017
MONGODB_DB = 'starlink'
MONGODB_COLLECTION = 'data'

def signal_handler(sig, frame):
    print('Exiting Starlink API with MongoDB')
    sys.exit(0)

def main():
    parser = ArgumentParser()
    parser.add_argument('-r', '--rate', type=float, default=0.5,
                        help='Rate at which to publish data to MongoDB from the dish.')
    args = parser.parse_args()

    signal.signal(signal.SIGINT, signal_handler)

    try:
        client = MongoClient(MONGODB_HOST, MONGODB_PORT, serverSelectionTimeoutMS=1000)
        client.server_info()  # Forces a connection attempt
        print("Connected to MongoDB at %s:%d" % (MONGODB_HOST, MONGODB_PORT))
    except errors.ServerSelectionTimeoutError as err:
        print("Failed to connect to MongoDB\n%s" % err)
        sys.exit(1)
    except errors.ConnectionFailure as err:
        print("Failed to connect to MongoDB\n%s" % err)
        sys.exit(1)

    db = client[MONGODB_DB]
    collection = db[MONGODB_COLLECTION]


    while True:
        data = starlink_data.fetch_data()
        
        if 'error' in data:
            continue
        
        data = json.loads(data)
        collection.insert_one(data)

        time.sleep(args.rate)

if __name__ == "__main__":
    main()
