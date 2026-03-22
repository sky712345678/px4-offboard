#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"
__note__ = "MAVROS version - converted from uXRCE-DDS PX4 implementation"

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker


class PX4VisualizerMAVROS(Node):
    def __init__(self):
        super().__init__("visualizer_mavros")

        # QoS profiles
        qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=0
        )

        # Subscribe to MAVROS local position and attitude
        self.local_position_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.local_position_callback,
            qos_profile_sub,
        )

        # Subscribe to setpoint position
        self.setpoint_sub = self.create_subscription(
            PoseStamped,
            "/mavros/setpoint_position/local",
            self.setpoint_callback,
            qos_profile_sub,
        )

        # Publishers for visualization
        self.vehicle_pose_pub = self.create_publisher(
            PoseStamped, "px4_visualizer/vehicle_pose", 10
        )
        self.vehicle_vel_pub = self.create_publisher(
            Marker, "px4_visualizer/vehicle_velocity", 10
        )
        self.vehicle_path_pub = self.create_publisher(
            Path, "px4_visualizer/vehicle_path", 10
        )
        self.setpoint_path_pub = self.create_publisher(
            Path, "px4_visualizer/setpoint_path", 10
        )

        self.vehicle_pose = None
        self.setpoint_position = np.array([0.0, 0.0, 0.0])
        self.vehicle_path_msg = Path()
        self.setpoint_path_msg = Path()

        # trail size
        self.trail_size = 1000

        # time stamp for the last local position update received on ROS2 topic
        self.last_local_pos_update = 0.0
        # time after which existing path is cleared upon receiving new
        # local position ROS2 message
        self.declare_parameter("path_clearing_timeout", -1.0)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)

        self.get_logger().info("Visualizer MAVROS node initialized")

    def local_position_callback(self, msg):
        """Callback for local position and attitude from MAVROS"""
        path_clearing_timeout = (
            self.get_parameter("path_clearing_timeout")
            .get_parameter_value()
            .double_value
        )
        if path_clearing_timeout >= 0 and (
            (self.get_clock().now().nanoseconds / 1e9 - self.last_local_pos_update)
            > path_clearing_timeout
        ):
            self.vehicle_path_msg.poses.clear()
        self.last_local_pos_update = Clock().now().nanoseconds / 1e9

        self.vehicle_pose = msg

    def setpoint_callback(self, msg):
        """Callback for setpoint position"""
        self.setpoint_position[0] = msg.pose.position.x
        self.setpoint_position[1] = msg.pose.position.y
        self.setpoint_position[2] = msg.pose.position.z

    def create_arrow_marker(self, id, tail, vector):
        """Create an arrow marker for visualization"""
        msg = Marker()
        msg.action = Marker.ADD
        msg.header.frame_id = "map"
        msg.ns = "arrow"
        msg.id = id
        msg.type = Marker.ARROW
        msg.scale.x = 0.1
        msg.scale.y = 0.2
        msg.scale.z = 0.0
        msg.color.r = 0.5
        msg.color.g = 0.5
        msg.color.b = 0.0
        msg.color.a = 1.0
        dt = 0.3
        tail_point = Point()
        tail_point.x = tail[0]
        tail_point.y = tail[1]
        tail_point.z = tail[2]
        head_point = Point()
        head_point.x = tail[0] + dt * vector[0]
        head_point.y = tail[1] + dt * vector[1]
        head_point.z = tail[2] + dt * vector[2]
        msg.points = [tail_point, head_point]
        return msg

    def append_vehicle_path(self, msg):
        """Append pose to vehicle path"""
        self.vehicle_path_msg.poses.append(msg)
        if len(self.vehicle_path_msg.poses) > self.trail_size:
            del self.vehicle_path_msg.poses[0]

    def append_setpoint_path(self, msg):
        """Append pose to setpoint path"""
        self.setpoint_path_msg.poses.append(msg)
        if len(self.setpoint_path_msg.poses) > self.trail_size:
            del self.setpoint_path_msg.poses[0]

    def cmdloop_callback(self):
        """Main visualization loop"""
        if self.vehicle_pose is None:
            return

        # Publish vehicle pose
        self.vehicle_pose_pub.publish(self.vehicle_pose)

        # Publish time history of the vehicle path
        self.vehicle_path_msg.header = self.vehicle_pose.header
        self.append_vehicle_path(self.vehicle_pose)
        self.vehicle_path_pub.publish(self.vehicle_path_msg)

        # Create and publish setpoint path
        setpoint_pose_msg = PoseStamped()
        setpoint_pose_msg.header.stamp = self.get_clock().now().to_msg()
        setpoint_pose_msg.header.frame_id = "map"
        setpoint_pose_msg.pose.position.x = self.setpoint_position[0]
        setpoint_pose_msg.pose.position.y = self.setpoint_position[1]
        setpoint_pose_msg.pose.position.z = self.setpoint_position[2]
        setpoint_pose_msg.pose.orientation.w = 1.0

        self.setpoint_path_msg.header = setpoint_pose_msg.header
        self.append_setpoint_path(setpoint_pose_msg)
        self.setpoint_path_pub.publish(self.setpoint_path_msg)

        # Publish velocity marker (zero for now since MAVROS doesn't stream velocity by default)
        # To get velocity, subscribe to /mavros/local_position/velocity_local (TwistStamped)
        velocity_vector = np.array([0.0, 0.0, 0.0])
        velocity_msg = self.create_arrow_marker(
            1,
            np.array([self.vehicle_pose.pose.position.x,
                      self.vehicle_pose.pose.position.y,
                      self.vehicle_pose.pose.position.z]),
            velocity_vector
        )
        self.vehicle_vel_pub.publish(velocity_msg)


def main(args=None):
    rclpy.init(args=args)

    px4_visualizer = PX4VisualizerMAVROS()

    try:
        rclpy.spin(px4_visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        px4_visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
