#!/usr/bin/env python3
import ctypes
import os
import json
import socket
import threading
import rclpy

from math import pi, sin, cos
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Empty, String, UInt32

class ROSNode(Node):
    def __init__(self, name, shared):
        super().__init__(name)
        self.shared = shared

        # Publisher for pose
        self.pub = self.create_publisher(Pose, '~/pose', 10)

        # UI‐style commands
        self.create_subscription(Point,    '~/goal',   self.cb_goal,   10)
        self.create_subscription(Empty,    '~/explore',self.cb_explore,10)

        # New: prize system subscriptions
        self.create_subscription(String,   '~/dicts',  self.cb_dicts,  10)
        self.create_subscription(UInt32,   '~/fetch',  self.cb_fetch,  10)

        self.timer = self.create_timer(1.0, self.cb_timer)
        self.get_logger().info(f"ROS Node '{name}' running")

    def shutdown(self):
        self.destroy_timer(self.timer)
        self.destroy_node()

    def cb_timer(self):
        if self.shared.acquire():
            x = self.shared.robotx
            y = self.shared.roboty
            h = self.shared.robotheading
            self.shared.release()

        theta = pi/4 * float(h + 2)
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.orientation.z = sin(theta/2)
        msg.orientation.w = cos(theta/2)
        self.pub.publish(msg)

    def cb_goal(self, msg):
        xg, yg = int(msg.x), int(msg.y)
        self.get_logger().info(f"Received goal command ({xg},{yg})")
        if self.shared.acquire():
            self.shared.command = 'goal'
            self.shared.params  = [str(xg), str(yg)]
            self.shared.release()

    def cb_explore(self, msg):
        self.get_logger().info("Received explore command")
        if self.shared.acquire():
            self.shared.command = 'explore'
            self.shared.params  = []
            self.shared.release()

    def cb_fetch(self, msg):
        prize_id = int(msg.data)
        self.get_logger().info(f"Received fetch command ({prize_id})")
        if self.shared.acquire():
            # Mode and prize id for your robot thread
            self.shared.command        = 'fetch'
            self.shared.params         = [str(prize_id)]
            self.shared.release()

    def cb_dicts(self, msg: String):
        # Grab the raw message data.
        raw_data = json.loads(msg.data)

        # Extract the intersection → { prize_id: distance } dictionary
        raw_dist_dict = raw_data["dist_dict"]
        inter_prize_distance_dict = {
            int(key): {
                int(prize_key): raw_dist_dict[key][prize_key]
                for prize_key in raw_dist_dict[key]
            }
            for key in raw_dist_dict
        }

        # Extract the prize → info dictionary
        raw_info_dict = raw_data["info_dict"]
        prize_info_dict = {
            int(key): raw_info_dict[key]
            for key in raw_info_dict
        }

        # Report.
        # self.get_logger().info("Received dicts message")
        # self.get_logger().info(f"DISTANCES: {inter_prize_distance_dict}")
        # self.get_logger().info(f"INFO:      {prize_info_dict}")

        # Place in shared data!
        if self.shared.acquire():
            self.shared.inter_prize_distance_dict = inter_prize_distance_dict
            self.shared.prize_info_dict          = prize_info_dict
            self.shared.release()


def runros(shared):
    os.environ['ROS_LOCALHOST_ONLY'] = '0'
    os.environ['ROS_DOMAIN_ID']      = '1'
    rclpy.init()
    node = ROSNode(socket.gethostname(), shared)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.shutdown()
    rclpy.shutdown()
