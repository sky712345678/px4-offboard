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
__note__ = "MAVROS version with automatic OFFBOARD mode - converted from uXRCE-DDS PX4 implementation"

import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool


class OffboardControlMAVROSAuto(Node):

    def __init__(self):
        super().__init__('offboard_control_mavros_auto')

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

        # Subscribe to MAVROS state
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            qos_profile_sub)

        # Publish setpoints to MAVROS
        self.setpoint_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            qos_profile_pub)

        # Service clients for mode setting and arming
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        timer_period = 0.02  # seconds (50 Hz - matches original)
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period

        self.declare_parameter('radius', 10.0)
        self.declare_parameter('omega', 5.0)
        self.declare_parameter('altitude', 5.0)

        # MAVROS state
        self.is_armed = False
        self.is_offboard_mode = False
        self.vehicle_connected = False

        # Mode transition state
        self.offboard_mode_requested = False
        self.arm_requested = False
        self.mode_transition_timer = 0

        # Trajectory parameters
        self.theta = 0.0
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value
        self.altitude = self.get_parameter('altitude').value

        self.initial_setpoint_counter = 0

        self.get_logger().info("Offboard Control MAVROS Auto node initialized")
        self.get_logger().info("Will automatically request OFFBOARD mode when vehicle is ready...")

    def state_callback(self, msg):
        """Callback to monitor vehicle state"""
        self.vehicle_connected = msg.connected
        self.is_armed = msg.armed
        prev_offboard_mode = self.is_offboard_mode
        self.is_offboard_mode = msg.mode == "OFFBOARD"
        
        if msg.connected and not self.vehicle_connected:
            self.get_logger().info("Vehicle connected")
        
        if self.is_offboard_mode and not prev_offboard_mode:
            self.get_logger().info("Vehicle entered OFFBOARD mode")

    def set_offboard_mode(self):
        """Request OFFBOARD mode"""
        if not self.set_mode_client.service_is_ready():
            return False
        
        request = SetMode.Request()
        request.custom_mode = "OFFBOARD"
        
        future = self.set_mode_client.call_async(request)
        future.add_done_callback(self.set_mode_callback)
        return True

    def set_mode_callback(self, future):
        """Callback for set_mode service"""
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("OFFBOARD mode request sent")
            else:
                self.get_logger().warn("Failed to request OFFBOARD mode")
        except Exception as e:
            self.get_logger().error(f"Set mode service call failed: {e}")

    def arm_vehicle(self):
        """Request vehicle to arm"""
        if not self.arm_client.service_is_ready():
            return False
        
        request = CommandBool.Request()
        request.value = True
        
        future = self.arm_client.call_async(request)
        future.add_done_callback(self.arm_callback)
        return True

    def arm_callback(self, future):
        """Callback for arm service"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Arm command sent successfully")
            else:
                self.get_logger().warn("Arm command failed")
        except Exception as e:
            self.get_logger().error(f"Arm service call failed: {e}")

    # def cmdloop_callback(self):
    #     """Main control loop - publishes setpoint position"""

    #     # Publish offboard control mode (continuously required by PX4)
    #     setpoint_msg = PoseStamped()
    #     setpoint_msg.header.stamp = self.get_clock().now().to_msg()
    #     setpoint_msg.header.frame_id = "map"

    #     # Circular trajectory in local NED frame
    #     # Note: MAVROS uses NED frame by default
    #     # Positive Z is down in NED
    #     setpoint_msg.pose.position.x = self.radius * np.cos(self.theta)
    #     setpoint_msg.pose.position.y = self.radius * np.sin(self.theta)
    #     setpoint_msg.pose.position.z = self.altitude

    #     # No rotation (level attitude)
    #     setpoint_msg.pose.orientation.w = 1.0
    #     setpoint_msg.pose.orientation.x = 0.0
    #     setpoint_msg.pose.orientation.y = 0.0
    #     setpoint_msg.pose.orientation.z = 0.0

    #     self.setpoint_pub.publish(setpoint_msg)

    #     self.initial_setpoint_counter += 1
    #     if self.initial_setpoint_counter < 100:
    #         return

    #     # Request OFFBOARD mode and arm if not already done
    #     if self.vehicle_connected and not self.is_offboard_mode:
    #         self.get_logger().info("Requesting OFFBOARD mode...")
    #         self.set_offboard_mode()
    #         self.offboard_mode_requested = True
        
    #     if self.vehicle_connected and not self.is_armed:
    #         self.get_logger().info("Requesting to arm vehicle...")
    #         self.arm_vehicle()
    #         self.arm_requested = True

    #     # Update trajectory only if armed and in OFFBOARD mode
    #     if self.is_armed and self.is_offboard_mode:
    #         self.theta = self.theta + self.omega * self.dt
    #         # if self.theta >= 2 * np.pi:
    #         #     self.theta = self.theta - 2 * np.pi

    def cmdloop_callback(self):
        setpoint_msg = PoseStamped()
        setpoint_msg.header.stamp = self.get_clock().now().to_msg()
        setpoint_msg.header.frame_id = "map"

        # --- 修正 1: 分階段控制 ---
        # 如果還沒解鎖或剛切換模式，先讓它穩定在圓周的起點上方，避免瞬間衝刺
        if self.initial_setpoint_counter < 1500:
            # 先停在半徑起點 (x=radius, y=0) 且高度為 altitude
            target_x = 0.0
            target_y = 0.0
            target_z = self.altitude
            current_yaw = 0.0 # 初始朝東

            self.get_logger().info("Running takeoff...")
        elif self.initial_setpoint_counter < 3000:
            # 在這個階段，保持在起點附近
            target_x = self.radius
            target_y = 0.0
            target_z = self.altitude
            current_yaw = self.theta + np.pi/2

            self.get_logger().info("Moving to radius...")
        else:
            # 進入 Offboard 且解鎖後，才開始更新 theta
            self.theta += self.omega * self.dt
            target_x = self.radius * np.cos(self.theta)
            target_y = self.radius * np.sin(self.theta)
            target_z = self.altitude
            # --- 修正 2: 讓機頭轉向切線方向 (Tangent) ---
            # 在 ENU 中，切線角為 theta + 90度 (PI/2)
            current_yaw = self.theta + np.pi/2

            self.get_logger().info(f"Circling...x: {target_x}, y: {target_y}, z: {target_z}, yaw: {current_yaw}")

        setpoint_msg.pose.position.x = target_x
        setpoint_msg.pose.position.y = target_y
        setpoint_msg.pose.position.z = target_z

        # --- 修正 3: 正確的 Orientation (Yaw 轉四元數) ---
        # 這裡使用簡單的 Euler to Quaternion 公式 (僅 Yaw)
        setpoint_msg.pose.orientation.w = math.cos(current_yaw / 2.0)
        setpoint_msg.pose.orientation.z = math.sin(current_yaw / 2.0)
        setpoint_msg.pose.orientation.x = 0.0
        setpoint_msg.pose.orientation.y = 0.0

        self.setpoint_pub.publish(setpoint_msg)

        # ... 模式切換與解鎖邏輯 (保持不變) ...
        # Request OFFBOARD mode and arm if not already done
        if self.vehicle_connected and not self.is_offboard_mode:
            self.get_logger().info("Requesting OFFBOARD mode...")
            self.set_offboard_mode()
            self.offboard_mode_requested = True
        
        if self.vehicle_connected and not self.is_armed:
            self.get_logger().info("Requesting to arm vehicle...")
            self.arm_vehicle()
            self.arm_requested = True
        
        self.initial_setpoint_counter += 1


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControlMAVROSAuto()

    try:
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        pass
    finally:
        offboard_control.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
