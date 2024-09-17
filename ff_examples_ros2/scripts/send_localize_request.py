#!/usr/bin/env python3

# Copyright 2019 Open Source Robotics Foundation, Inc.
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
import argparse

import rclpy
from rclpy.node import Node

from rmf_fleet_msgs.msg import LocalizeRequest


def main(argv=sys.argv):
    """
    Example localize request:
    - fleet_name: magni
    - robot_name: magni123
    - task_id: 6tyghb4edujrefyd
    - x: 0.0
    - y: 0.0
    - yaw: 0.0
    - level_name: B1
    """

    default_fleet_name = "amr_vdm"
    default_robot_name = "amr_002"
    default_task_id = "576y13ewgyffeijuais"
    default_desired_x = 0.0
    default_desired_y = 0.0
    default_desired_yaw = 0.0
    default_level_name = "L1"
    default_topic_name = "robot_localize_requests"

    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--fleet-name", default=default_fleet_name)
    parser.add_argument("-r", "--robot-name", default=default_robot_name)
    parser.add_argument("-x", default=default_desired_x)
    parser.add_argument("-y", default=default_desired_y)
    parser.add_argument("--yaw", default=default_desired_yaw)
    parser.add_argument("-l", "--level-name", default=default_level_name)
    parser.add_argument("-i", "--task-id", default=default_task_id)
    parser.add_argument("-t", "--topic-name", default=default_topic_name)
    args = parser.parse_args(argv[1:])

    print("fleet_name: {}".format(args.fleet_name))
    print("robot_name: {}".format(args.robot_name))
    print("x: {}".format(args.x))
    print("y: {}".format(args.y))
    print("yaw: {}".format(args.yaw))
    print("level_name: {}".format(args.level_name))
    print("task_id: {}".format(args.task_id))
    print("topic_name: {}".format(args.topic_name))

    rclpy.init()
    node = rclpy.create_node("send_localize_request_node")
    pub = node.create_publisher(LocalizeRequest, args.topic_name, 10)

    msg = LocalizeRequest()
    msg.fleet_name = args.fleet_name
    msg.robot_name = args.robot_name
    msg.task_id = args.task_id
    # ignore time for now
    msg.destination.x = float(args.x)
    msg.destination.y = float(args.y)
    msg.destination.yaw = float(args.yaw)
    msg.destination.level_name = args.level_name

    rclpy.spin_once(node, timeout_sec=1.0)
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    print("all done!")
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
