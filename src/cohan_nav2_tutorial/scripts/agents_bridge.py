#!/usr/bin/env python3
"""
Software License Agreement (MIT License)

Copyright (c) 2020-2025 LAAS-CNRS

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

# Brief: This node subscribes to the robots published on /humani, i=1,2, .. and robot, and publishes /tracked_agents required for CoHAN
# Author: Phani Teja Singamaneni

import sys
import rclpy
from rclpy.node import Node
from cohan_msgs.msg import TrackedAgents, TrackedAgent, TrackedSegment, TrackedSegmentType, AgentType
from nav_msgs.msg import Odometry
from message_filters import Subscriber, TimeSynchronizer
from geometry_msgs.msg import Pose, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
import math


def fast_rotate_local_90(q_curr, axis='x'):
    """
    Rotates an incoming orientation quaternion by exactly 90 degrees 
    around its local X, Y, or Z axis.
    
    :param q_curr: geometry_msgs/msg/Quaternion (current heading)
    :param axis: String, either 'x', 'y', or 'z'
    :return: geometry_msgs/msg/Quaternion (rotated orientation)
    """
    # Local stack references for maximum execution speed
    cx, cy, cz, cw = q_curr.x, q_curr.y, q_curr.z, q_curr.w
    SQRT_2_OVER_2 = 0.70710678118
    
    q_out = Quaternion()
    axis_lower = axis.lower()
    
    if axis_lower == 'x':
        q_out.x = SQRT_2_OVER_2 * (cw + cx)
        q_out.y = SQRT_2_OVER_2 * (cy + cz)
        q_out.z = SQRT_2_OVER_2 * (cz - cy)
        q_out.w = SQRT_2_OVER_2 * (cw - cx)
        
    elif axis_lower == 'y':
        q_out.x = SQRT_2_OVER_2 * (cx - cz)
        q_out.y = SQRT_2_OVER_2 * (cw + cy)
        q_out.z = SQRT_2_OVER_2 * (cx + cz)
        q_out.w = SQRT_2_OVER_2 * (cw - cy)
        
    elif axis_lower == 'z':
        q_out.x = SQRT_2_OVER_2 * (cx + cy)
        q_out.y = SQRT_2_OVER_2 * (cy - cx)
        q_out.z = SQRT_2_OVER_2 * (cw + cz)
        q_out.w = SQRT_2_OVER_2 * (cw - cz)
        
    else:
        return q_curr
        
    return q_out

def get_yaw_from_pose(pose):
    """
    Helper function to get the yaw from Pose message
    """
    qx, qy, qz, qw = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

class SimAgents(Node):
    """
    Bridge between CoHAN Sim agents and CoHAN tracked_agents message.

    Subscribes to simulated human and robot odometry topics, converts them to TrackedAgents messages,
    and publishes them for use by CoHAN Navigation. Handles both human and robot agents, synchronizes their states,
    and assigns appropriate segment and agent types.
    """

    def __init__(self, num_hum):
        """
        Initialize the SimAgents bridge.

        Args:
            num_hum (int): Number of human agents in the simulation.
        """
        super().__init__('sim_agents')
        self.num_hum = num_hum
        # self.ns = self.get_namespace().strip('/')
        self.ns = self.get_namespace()
        self.tracked_agents_pub = []
        self.Segment_Type = TrackedSegmentType.TORSO
        self.agents = TrackedAgents()
        self.robot = TrackedAgent()
        self.sig_1 = False
        self.sig_2 = False
        
        self.setup_subscribers_and_publisher()

    def setup_subscribers_and_publisher(self):
        """
        Set up subscribers and publisher for tracked agents.
        Synchronizes human agent odometry and robot odometry, and starts publishing tracked_agents messages.
        """
        agent_sub = []

        for agent_id in range(1, self.num_hum + 1):
            name = 'human' + str(agent_id)
            if self.ns != name:
                agent_sub.append(Subscriber(self, Odometry, "/" + name + "/base_pose_ground_truth"))
        
        # Subscribe to the robot
        self.robot_sub = self.create_subscription(Odometry, "/base_pose_ground_truth", self.RobotCB, 10)
        if self.ns == "/":
            self.sig_2 = True

        self.tracked_agents_pub = self.create_publisher(TrackedAgents, "tracked_agents", 10)
        
        self.human_markers_pub = self.create_publisher(MarkerArray, "tracked_human_markers", 10)
        
        self.robot_pub = self.create_publisher(MarkerArray, 'robot_marker', 10)
        
        
        # Set up message filter synchronization
        if agent_sub:
            self.pose_msg = TimeSynchronizer(agent_sub, 10)
            self.pose_msg.registerCallback(self.AgentsCB)
        
        # Create timer to publish at 50 Hz (0.02 seconds)
        self.publish_timer = self.create_timer(0.02, self.publishAgents)

    def AgentsCB(self,*msg):
        """
        Callback for synchronized human agent odometry messages.
        Converts odometry to TrackedAgent messages and updates the tracked_agents list.

        Args:
            *msg: Synchronized odometry messages for human agents.
        """
        # if len(msg) != self.num_hum:
        #     return
    
        tracked_agents = TrackedAgents()
        idx = 0
        for agent_id in range(1,self.num_hum+1):
            if self.ns == "human"+str(agent_id):
                continue
            agent_segment = TrackedSegment()
            agent_segment.type = self.Segment_Type
            # print(agent_id-1)
            agent_segment.pose.pose = msg[idx].pose.pose
            agent_segment.twist.twist = msg[idx].twist.twist
            tracked_agent = TrackedAgent()
            tracked_agent.type = AgentType.HUMAN
            tracked_agent.name = "human"+str(agent_id)
            tracked_agent.segments.append(agent_segment)
            tracked_agents.agents.append(tracked_agent)
            idx += 1
        if(tracked_agents.agents):
            self.agents = tracked_agents
            self.sig_1 = True

    def RobotCB(self, msg):
        """
        Callback for robot odometry messages.
        Converts odometry to a TrackedAgent message for the robot and updates the tracked_agents list.

        Args:
            msg (Odometry): Odometry message for the robot.
        """
        if self.num_hum < 2:
            self.agents = TrackedAgents()
        agent_segment = TrackedSegment()
        agent_segment.type = self.Segment_Type
        agent_segment.pose.pose = msg.pose.pose
        agent_segment.twist.twist = msg.twist.twist
        tracked_agent = TrackedAgent()
        tracked_agent.type = AgentType.ROBOT
        tracked_agent.name = "robot"
        tracked_agent.segments.append(agent_segment)
        self.robot = tracked_agent
        self.sig_2 = True

    def publishAgents(self):
        """
        Publishes the current tracked_agents message if both human and robot data are available.
        Assigns track IDs and sets the header fields.
        """
        now = self.get_clock().now().to_msg()
        if(self.sig_1 and self.sig_2):
            self.agents.header.stamp = now
            self.agents.header.frame_id = "map"
            if(self.ns != "/"):
                self.agents.agents.append(self.robot)
            for agent_id in range(0, len(self.agents.agents)):
                self.agents.agents[agent_id].track_id = agent_id+1
            self.tracked_agents_pub.publish(self.agents)
            self.publish_human_markers(now)
            rx = self.robot.segments[0].pose.pose.position.x
            ry = self.robot.segments[0].pose.pose.position.y
            r_yaw = get_yaw_from_pose(self.robot.segments[0].pose.pose)
            self.publish_robot_marker([rx, ry, r_yaw], now)  
            if self.num_hum >= 2:
                self.sig_1 = False
            if self.ns != "/":
                self.sig_2 = False          
                
                
    def publish_human_markers(self, current_time):
        """
        This marker respresentation is inspired from the Spensor Markers used in PedSim ROS
        """

        marker_array = MarkerArray()
        BODY_HEIGHT = 1.3
        BODY_RADIUS = 0.25
        HEAD_DIAMETER = 0.3
        
        for idx, agent in enumerate(self.agents.agents):
            for segment in agent.segments:
                if segment.type == self.Segment_Type:
                    body_id = idx * 2
                    head_id = idx * 2 + 1
                    
                    body_marker = Marker()
                    body_marker.header.frame_id = "map"
                    body_marker.header.stamp = current_time
                    body_marker.ns = "human_bodies"
                    body_marker.id = body_id
                    body_marker.type = Marker.CYLINDER
                    body_marker.action = Marker.ADD
                    
                    # Position the cylinder base onto the ground plane
                    body_marker.pose = Pose()
                    body_marker.pose.position.x = segment.pose.pose.position.x
                    body_marker.pose.position.y = segment.pose.pose.position.y
                    body_marker.pose.position.z = segment.pose.pose.position.z + (BODY_HEIGHT / 2.0)            
                    
                    rot1 = fast_rotate_local_90(segment.pose.pose.orientation, 'y')
                    body_marker.pose.orientation=fast_rotate_local_90(rot1, 'x')
                    
                    body_marker.scale.x = BODY_HEIGHT 
                    body_marker.scale.y = BODY_RADIUS  # Depth
                    body_marker.scale.z = BODY_RADIUS * 2.0  # Width
                    
                    body_marker.color.r = 0.12
                    body_marker.color.g = 0.53
                    body_marker.color.b = 0.70
                    body_marker.color.a = 0.65  # Alpha gives that translucent look
                    
                    body_marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()
                    marker_array.markers.append(body_marker)
                    
                    head_marker = Marker()
                    head_marker.header.frame_id = "map"
                    head_marker.header.stamp = current_time
                    head_marker.ns = "spencer_human_heads"
                    head_marker.id = head_id
                    head_marker.type = Marker.SPHERE
                    head_marker.action = Marker.ADD
                    
                    head_marker.pose = Pose()
                    head_marker.pose.position.x = segment.pose.pose.position.x
                    head_marker.pose.position.y = segment.pose.pose.position.y
                    head_marker.pose.position.z = segment.pose.pose.position.z + BODY_HEIGHT + (HEAD_DIAMETER / 2.0)
                    head_marker.pose.orientation = segment.pose.pose.orientation
                    
                    head_marker.scale.x = HEAD_DIAMETER
                    head_marker.scale.y = HEAD_DIAMETER
                    head_marker.scale.z = HEAD_DIAMETER
                    
                    head_marker.color.r = 0.08
                    head_marker.color.g = 0.40
                    head_marker.color.b = 0.55
                    head_marker.color.a = 1.0  # Solid opacity
                    
                    head_marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()
                    marker_array.markers.append(head_marker)
                    

                    arrow_marker = Marker()
                    arrow_marker.header.frame_id = "map"
                    arrow_marker.header.stamp = current_time
                    arrow_marker.ns = "human_arrows"
                    arrow_marker.id = idx 
                    arrow_marker.type = Marker.ARROW
                    arrow_marker.action = Marker.ADD
                    
                    # Position at waist level, using the raw agent orientation
                    arrow_marker.pose = Pose()
                    arrow_marker.pose.position.x = segment.pose.pose.position.x
                    arrow_marker.pose.position.y = segment.pose.pose.position.y
                    arrow_marker.pose.position.z = segment.pose.pose.position.z + 0.8* BODY_HEIGHT  
                    arrow_marker.pose.orientation = segment.pose.pose.orientation      
                    
                    arrow_marker.scale.x = 0.5  
                    arrow_marker.scale.y = 0.08  # Arrow width
                    arrow_marker.scale.z = 0.05  # Arrow thickness
                    
                    arrow_marker.color.r = 1.0
                    arrow_marker.color.g = 0.75
                    arrow_marker.color.b = 0.0
                    arrow_marker.color.a = 1.0   # Fully opaque
                    
                    arrow_marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()
                    marker_array.markers.append(arrow_marker)
                    
                    ring_marker = Marker()
                    ring_marker.header.frame_id = "map"
                    ring_marker.header.stamp = current_time
                    ring_marker.ns = "human_rings"
                    ring_marker.id = idx  # Safe to reuse idx due to unique namespace
                    ring_marker.type = Marker.LINE_STRIP
                    ring_marker.action = Marker.ADD
                    
                    ring_marker.pose = Pose()
                    ring_marker.pose.position.x = segment.pose.pose.position.x
                    ring_marker.pose.position.y = segment.pose.pose.position.y
                    ring_marker.pose.position.z = segment.pose.pose.position.z + 0.01
                    ring_marker.pose.orientation = segment.pose.pose.orientation
                    
                    ring_marker.scale.x = 0.03  # 3 cm thick outline line
                    
                    ring_marker.color.r = 0.12
                    ring_marker.color.g = 0.53
                    ring_marker.color.b = 0.70
                    ring_marker.color.a = 0.80  # Mostly opaque for a sharp outline
                    
                    RING_RADIUS = 0.6
                    NUM_POINTS = 32
                    import math
                    from geometry_msgs.msg import Point
                    
                    for i in range(NUM_POINTS + 1): 
                        angle = 2.0 * math.pi * i / NUM_POINTS
                        p = Point()
                        p.x = RING_RADIUS * math.cos(angle)
                        p.y = RING_RADIUS * math.sin(angle)
                        p.z = 0.0  # Flat on its local plane
                        ring_marker.points.append(p)
                    
                    ring_marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()
                    marker_array.markers.append(ring_marker)

        self.human_markers_pub.publish(marker_array)
        
    def publish_robot_marker(self, pose, now):
        marker_array = MarkerArray()
        
        # Extract pose variables
        x, y, theta = pose[0], pose[1], pose[2]

        # Standard conversion from Z-axis Euler angle to Quaternion
        cos_t = math.cos(theta * 0.5)
        sin_t = math.sin(theta * 0.5)
        
        body_orientation = Quaternion()
        body_orientation.x = 0.0
        body_orientation.y = 0.0
        body_orientation.z = sin_t
        body_orientation.w = cos_t

        # Wheel Orientation: Rotated 90 deg around X (0.7071, 0, 0, 0.7071) and rotated by yaw
        wheel_orientation = Quaternion()
        wheel_orientation.x = 0.7071 * cos_t
        wheel_orientation.y = 0.7071 * sin_t
        wheel_orientation.z = 0.7071 * sin_t
        wheel_orientation.w = 0.7071 * cos_t

        # Precompute trigonometry for offset rotations
        c = math.cos(theta)
        s = math.sin(theta)

        # ROBOT CHASSIS (Cylinder)
        body_marker = Marker()
        body_marker.header.frame_id = "map"
        body_marker.header.stamp = now
        body_marker.ns = "robot"
        body_marker.id = 0
        body_marker.type = Marker.CYLINDER
        body_marker.action = Marker.ADD
        body_marker.pose.position.x = x
        body_marker.pose.position.y = y
        body_marker.pose.position.z = 0.15  
        body_marker.pose.orientation = body_orientation
        body_marker.scale.x = 0.4   
        body_marker.scale.y = 0.4   
        body_marker.scale.z = 0.1   
        body_marker.color.r = 0.8
        body_marker.color.g = 1.0
        body_marker.color.b = 0.2
        body_marker.color.a = 1.0
        marker_array.markers.append(body_marker)

        # LEFT MAIN WHEEL 
        left_wheel = Marker()
        left_wheel.header.frame_id = "map"
        left_wheel.header.stamp = now
        left_wheel.ns = "robot"
        left_wheel.id = 1
        left_wheel.type = Marker.CYLINDER
        # Rotate local offset (x=0, y=0.22) by yaw angle
        left_wheel.pose.position.x = x + (0.0 * c - 0.22 * s)
        left_wheel.pose.position.y = y + (0.0 * s + 0.22 * c)
        left_wheel.pose.position.z = 0.1   
        left_wheel.pose.orientation = wheel_orientation
        left_wheel.scale.x = 0.2
        left_wheel.scale.y = 0.2
        left_wheel.scale.z = 0.04
        left_wheel.color.r = 0.1
        left_wheel.color.g = 0.1
        left_wheel.color.b = 0.1
        left_wheel.color.a = 1.0
        marker_array.markers.append(left_wheel)

        # RIGHT MAIN WHEEL
        right_wheel = Marker()
        right_wheel.header.frame_id = "map"
        right_wheel.header.stamp = now
        right_wheel.ns = "robot"
        right_wheel.id = 2
        right_wheel.type = Marker.CYLINDER
        # Rotate local offset (x=0, y=-0.22) by yaw angle
        right_wheel.pose.position.x = x + (0.0 * c - (-0.22) * s)
        right_wheel.pose.position.y = y + (0.0 * s + (-0.22) * c)
        right_wheel.pose.position.z = 0.1
        right_wheel.pose.orientation = wheel_orientation
        right_wheel.scale.x = 0.2
        right_wheel.scale.y = 0.2
        right_wheel.scale.z = 0.04
        right_wheel.color.r = 0.1
        right_wheel.color.g = 0.1
        right_wheel.color.b = 0.1
        right_wheel.color.a = 1.0
        marker_array.markers.append(right_wheel)

        # FRONT CASTER BALL (Sphere)
        caster_ball = Marker()
        caster_ball.header.frame_id = "map"
        caster_ball.header.stamp = now
        caster_ball.ns = "robot"
        caster_ball.id = 3
        caster_ball.type = Marker.SPHERE
        caster_ball.action = Marker.ADD
        # Rotate local offset (x=0.14, y=0) by yaw angle
        caster_ball.pose.position.x = x + (0.14 * c - 0.0 * s)
        caster_ball.pose.position.y = y + (0.14 * s + 0.0 * c)
        caster_ball.pose.position.z = 0.05  
        caster_ball.pose.orientation = body_orientation
        caster_ball.scale.x = 0.1
        caster_ball.scale.y = 0.1
        caster_ball.scale.z = 0.1
        caster_ball.color.r = 0.1
        caster_ball.color.g = 0.1
        caster_ball.color.b = 0.1  
        caster_ball.color.a = 1.0
        marker_array.markers.append(caster_ball)

        # FRONT DIRECTION MARKER (Small Box/Arrow) 
        direction_marker = Marker()
        direction_marker.header.frame_id = "map"
        direction_marker.header.stamp = now
        direction_marker.ns = "robot_arrow"
        direction_marker.id = 4
        direction_marker.type = Marker.ARROW
        direction_marker.action = Marker.ADD
        # Rotate local offset (x=0.20, y=0) by yaw angle
        direction_marker.pose.position.x = x #+ (0.10 * c - 0.0 * s)
        direction_marker.pose.position.y = y #+ (0.10 * s + 0.0 * c)
        direction_marker.pose.position.z = 0.2  
        direction_marker.pose.orientation = body_orientation
        direction_marker.scale.x = 0.2
        direction_marker.scale.y = 0.03
        direction_marker.scale.z = 0.03
        direction_marker.color.r = 1.0
        direction_marker.color.g = 0.0
        direction_marker.color.b = 0.0   
        direction_marker.color.a = 1.0
        marker_array.markers.append(direction_marker)

        # Publish the fully assembled robot model
        self.robot_pub.publish(marker_array)


def main():
    rclpy.init()
    
    # Filter out ROS2 arguments to get only our custom arguments
    filtered_args = []
    skip_next = False
    for i, arg in enumerate(sys.argv[1:], 1):
        if skip_next:
            skip_next = False
            continue
        if arg.startswith('--ros-args'):
            # Skip all remaining ROS args
            break
        if arg.startswith('-r') or arg.startswith('--remap'):
            skip_next = True  # Skip the next argument too (remap value)
            continue
        filtered_args.append(arg)
    
    if len(filtered_args) < 1:
        print("Usage: ros2 run cohan_sim_navigation agents_bridge.py <num_humans>")
        return
    
    nh = filtered_args[0]

    agents = SimAgents(num_hum=int(nh))
    agents.get_logger().info("Starting agents_bridge with {} humans".format(nh))
    
    try:
        rclpy.spin(agents)
    except KeyboardInterrupt:
        pass
    finally:
        agents.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
