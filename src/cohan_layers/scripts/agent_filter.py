#!/usr/bin/env python3

"""
Software License Agreement (MIT License)

Copyright (c) 2020–2026 LAAS-CNRS

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

Author: Phani Teja Singamaneni
"""

"""
@file agent_filter.py
@brief Node that filters tracked agents from laser data for HATEB planner

@details This node subscribes to tracked agents and laser scan data, removes 
the agent detections from the scan, and publishes the filtered laser scan
used by the HATEB local planner.
@author Phani Teja Singamaneni
"""

import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
import tf2_ros
from sensor_msgs.msg import LaserScan
from cohan_msgs.msg import TrackedAgents, TrackedSegmentType
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

def euler_from_quaternion(q):
    return R.from_quat(q).as_euler('xyz', degrees=False)

## Some Global Variables
SCAN_TOPIC = "/scan"
MAP_FRAME = "map"
LASER_FRAME = "base_laser_link"
TRACKED_AGENTS_TOPIC = "/tracked_agents"

class AgentFilter(Node):
    """Filters tracked agents from laser scan data (ROS2 node)

    Subscribes to laser scan and tracked agents topics, removes agent
    detections from the scan data, and publishes filtered scan for navigation.
    """

    def __init__(self, ns):
        super().__init__('agent_filter')

        if ":=" in ns:
            self.ns_ = ""
        else:
            self.ns_ = ns

        self.filtered_scan = LaserScan()
        self.segment_type = TrackedSegmentType.TORSO
        self.agents = []
        self.laser_transform = TransformStamped()
        self.got_scan = False
        self.laser_frame = LASER_FRAME

        # tf2 buffer and listener (ROS2)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Adjust topics for namespace
        laser_scan = SCAN_TOPIC
        tracked_agents = TRACKED_AGENTS_TOPIC
        if self.ns_ != "":
            laser_scan = "/" + self.ns_ + SCAN_TOPIC
            tracked_agents = "/" + self.ns_ + tracked_agents
            self.laser_frame = self.ns_ + "/" + LASER_FRAME
        print(laser_scan)

        # Subscriptions and publisher
        self.create_subscription(LaserScan, laser_scan, self.laserCB, 10)
        self.create_subscription(TrackedAgents, tracked_agents, self.agentsCB, 10)
        self.laser_pub = self.create_publisher(LaserScan, 'base_scan_filtered', 10)

        # Timer to publish filtered scan
        self.create_timer(0.02, self.publishScan)

    def laserCB(self, scan: LaserScan):
        filtered_scan = scan
        filtered_scan.ranges = list(scan.ranges)
        filtered_scan.header.stamp = self.get_clock().now().to_msg()

        try:
            # lookup transform from laser frame to map
            self.laser_transform = self.tf_buffer.lookup_transform(MAP_FRAME, self.laser_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # couldn't get transform; keep previous if any
            pass

        if getattr(self.laser_transform.header, 'frame_id', '') != '':
            laser_pose = self.laser_transform.transform.translation

            rot = self.laser_transform.transform.rotation
            r, p, y = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            base_laser_dir = [np.cos(y), np.sin(y)]

            # Filtering agents from the scan
            for pose_type in self.agents:
                rh_vec = [pose_type[0].position.x - laser_pose.x, pose_type[0].position.y - laser_pose.y]
                sign = math.copysign(1, base_laser_dir[0] * -rh_vec[1] + base_laser_dir[1] * rh_vec[0])
                t_angle = scan.angle_max - scan.angle_min
                # project angle
                mid_angle = t_angle / 2 - sign * np.arccos((base_laser_dir[0] * rh_vec[0] + base_laser_dir[1] * rh_vec[1]) / (np.linalg.norm(rh_vec)))

                if math.isnan(mid_angle):
                    continue

                mid_idx = int((mid_angle) / scan.angle_increment)
                if mid_idx >= len(scan.ranges):
                    continue

                if pose_type[1] == 0:
                    r_agent = 0.6
                else:
                    r_agent = 0.4
                d = np.linalg.norm(rh_vec)
                mr = scan.ranges[mid_idx]

                if mr <= (d - r_agent):
                    continue

                if r_agent <= d:
                    beta = np.arcsin(r_agent / d)
                else:
                    beta = np.pi / 2

                min_idx = int(np.floor((mid_angle - beta) / scan.angle_increment))
                max_idx = int(np.ceil((mid_angle + beta) / scan.angle_increment))

                for i in range(min_idx, max_idx):
                    if 0 <= i < len(scan.ranges):
                        filtered_scan.ranges[i] = scan.range_max

        self.filtered_scan = filtered_scan
        self.got_scan = True

    def agentsCB(self, msg: TrackedAgents):
        for agent in msg.agents:
            for segment in agent.segments:
                if segment.type == self.segment_type:
                    if len(self.agents) < agent.track_id:
                        self.agents.append([segment.pose.pose, agent.type])
                    else:
                        self.agents[agent.track_id - 1] = [segment.pose.pose, agent.type]

    def publishScan(self):
        if self.got_scan:
            self.filtered_scan.header.stamp = self.get_clock().now().to_msg()
            self.laser_pub.publish(self.filtered_scan)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        ns = ""
    else:
        ns = sys.argv[1]

    node = AgentFilter(ns=ns)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
