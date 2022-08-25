
# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import math
import yaml
import argparse

import socket
import urllib3
import time
import threading

import rclpy
from door_adapter_dormakaba.DormakabaClientAPI import DormakabaAPI
from rclpy.node import Node
from rclpy.time import Time
from rmf_door_msgs.msg import DoorRequest, DoorState, DoorMode

###############################################################################

class DoorAdapter(Node):
    def __init__(self,config_yaml):
        super().__init__('dormakaba_door_adapter')
        self.get_logger().info('Greetings, starting dormakaba door adapter')

        # Get value from config file
        self.door_name = config_yaml['door']['name']
        url = config_yaml['door']['api_endpoint']
        api_key = config_yaml['door']['header_key']
        api_value = config_yaml['door']['header_value']
        door_id = config_yaml['door']['door_id']

        self.api = DormakabaAPI(url,api_key,api_value,door_id)
        assert self.api.connected, "Unable to establish connection with door"
        self.get_logger().info('...Successfully connected to Door API')
        self.get_logger().info('Ready to accept Door Request')

        # default door state - closed mode
        self.door_mode = DoorMode.MODE_CLOSED

        self.door_states_pub = self.create_publisher(
            DoorState, 'door_states', 1)

        self.door_request_sub = self.create_subscription(
            DoorRequest, 'door_requests', self.door_request_cb, 10)

        self.periodic_timer = self.create_timer(
            1.0, self.time_cb)


    def time_cb(self):
        self.door_mode = self.api.get_mode()
        state_msg = DoorState()
        state_msg.door_time = self.get_clock().now().to_msg()
        # publish states of the door
        state_msg.door_name = self.door_name
        state_msg.current_mode.value = self.door_mode
        self.door_states_pub.publish(state_msg)

    def door_request_cb(self, msg: DoorRequest):
        # check DoorRequest msg whether the door name of the request is same as the current door. If not, ignore the request
        if msg.door_name == self.door_name:
            self.get_logger().info(f"Door mode [{msg.requested_mode.value}] requested by {msg.requester_id}")
            if msg.requested_mode.value == DoorMode.MODE_OPEN:
                # open door implementation
                success = self.api.open_door()
                if success:
                    self.get_logger().info(f"Request to open door [{self.door_name}] is successful")
                else:
                    self.get_logger().info(f"Request to open door [{self.door_name}] is unsuccessful")
            elif msg.requested_mode.value == DoorMode.MODE_CLOSED:
                # close door implementation
                success = self.api.close_door()
                if success:
                    self.get_logger().info(f"Request to close door [{self.door_name}] is successful")
                else:
                    self.get_logger().info(f"Request to close door [{self.door_name}] is unsuccessful")
            else:
                self.get_logger().error('Invalid door mode requested. Ignoring...')

###############################################################################

def main(argv=sys.argv):
    rclpy.init(args=argv)

    args_without_ros = rclpy.utilities.remove_ros_args(argv)
    parser = argparse.ArgumentParser(
        prog="door_adapter_dormakaba",
        description="Configure and spin up door adapter for Dormakaba doors")
    parser.add_argument("-c", "--config_file", type=str, required=True,
                        help="Path to the config.yaml file for this door adapter")
    args = parser.parse_args(args_without_ros[1:])
    config_path = args.config_file

    # Load config and nav graph yamls
    with open(config_path, "r") as f:
        config_yaml = yaml.safe_load(f)

    door_adapter = DoorAdapter(config_yaml)
    rclpy.spin(door_adapter)

    door_adapter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)