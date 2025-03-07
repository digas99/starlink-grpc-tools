#!/usr/bin/env python3
import json
import pprint

import dish_common
import starlink_grpc

from collections.abc import Mapping, Sequence
from ros.ros_style_logger import RosLogger

logger = RosLogger('starlink_node')

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

def process_data(data):
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

def fetch_data():
    try:
        logger.info('Getting status data from dish...')
        status = starlink_grpc.all_status_data(context=dish_common.GlobalState().context)
    except starlink_grpc.GrpcError:
        logger.error('Failed to get status data. Are you connected to the dish?')
        msg = json.dumps({'error': 'Failed to get status data. Are you connected to the dish?'})
        return msg

    processed_data = {key: process_data(getattr(status, key)) for key in status.DESCRIPTOR.fields_by_name.keys()}

    msg = json.dumps(processed_data, default=handle_serialization)
    
    pprint.pprint(json.loads(msg))
    
    return msg