#!/usr/bin/env python3
import argparse
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Float64
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State, Altitude, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def euler_deg_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class PlaneControlNode(Node):
    def __init__(self, config: dict=None):
        super().__init__('plane_control_node')

        # Flight state
        self.mode = ''
        self.armed = False

        # Telemetry
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0

        self.altitude_m = 0.0
        self.vertical_speed_mps = 0.0
        self.groundspeed_mps = 0.0

        # Valid flags
        self.imu_valid = False
        self.alt_valid = False
        self.odom_valid = False
        self.heading_valid = False

        # Hold state
        self.roll_hold_enabled = False
        self.pitch_hold_enabled = False
        self.throttle_hold_enabled = False

        self.target_roll_deg = 0.0
        self.target_pitch_deg = 0.0
        self.target_throttle = 0.0
        self.target_altitude_m = 0.0

        # Commanded setpoint currently being sent
        self.cmd_roll_deg = 0.0
        self.cmd_pitch_deg = 0.0
        self.cmd_yaw_deg = 0.0
        self.cmd_throttle = 0.0

        # Offboard warmup
        self.offboard_request_pending = False
        self.offboard_mode_sent = False
        self.offboard_warmup_counter = 0
        self.offboard_warmup_required = 30  # 30 cycles @ 20 Hz = 1.5 sec

        # Simple altitude assist when roll hold is active but pitch hold is not
        self.enable_altitude_assist = True
        self.alt_p_gain = 0.12
        self.alt_vz_gain = 0.08
        self.max_alt_pitch_correction_deg = 8.0
        self.max_cmd_roll_deg = 45.0
        self.max_cmd_pitch_deg = 20.0

        # QoS
        sensor_qos = qos_profile_sensor_data
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.create_subscription(Imu, '/mavros/imu/data', self.imu_cb, sensor_qos)
        self.create_subscription(Altitude, '/mavros/altitude', self.alt_cb, best_effort_qos)
        self.create_subscription(Odometry, '/mavros/local_position/odom', self.odom_cb, best_effort_qos)
        self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.hdg_cb, best_effort_qos)
        self.create_subscription(State, '/mavros/state', self.state_cb, best_effort_qos)
        self.create_subscription(String, '/gcs/cmd', self.cmd_cb, 10)

        # Publishers
        self.telemetry_pub = self.create_publisher(String, '/gcs/telemetry', 10)
        self.att_sp_pub = self.create_publisher(AttitudeTarget, '/mavros/setpoint_raw/attitude', 10)

        # Services
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Timers
        self.create_timer(0.2, self.publish_telemetry)   # 5 Hz UI
        self.create_timer(0.05, self.control_loop)       # 20 Hz setpoint

        self.get_logger().info('plane_control_node_v4 started')
        self.get_logger().info('listen cmd topic: /gcs/cmd')
        self.get_logger().info('publish telemetry topic: /gcs/telemetry')
        self.get_logger().info('publish attitude setpoint: /mavros/setpoint_raw/attitude')

        # Custom args
        self.use_alt_takeoff = config.get('alt_takeoff', False) if config is not None else False

    # ---------- Telemetry callbacks ----------
    def imu_cb(self, msg: Imu):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qw * qy - qz * qx)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        self.roll_deg = math.degrees(roll)
        self.pitch_deg = math.degrees(pitch)

        if not self.heading_valid:
            self.yaw_deg = math.degrees(yaw)

        self.imu_valid = True

    def alt_cb(self, msg: Altitude):
        self.altitude_m = msg.local
        self.alt_valid = True

    def odom_cb(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z

        self.groundspeed_mps = math.sqrt(vx * vx + vy * vy)
        self.vertical_speed_mps = vz
        self.odom_valid = True

    def hdg_cb(self, msg: Float64):
        self.yaw_deg = msg.data
        self.heading_valid = True

    def state_cb(self, msg: State):
        self.mode = msg.mode
        self.armed = msg.armed

    # ---------- Command handling ----------
    def cmd_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'bad json: {e}')
            return

        cmd = data.get('cmd', '')
        self.get_logger().info(f'cmd received: {msg.data}')

        if cmd == 'arm':
            self.do_arm(True)

        elif cmd == 'disarm':
            self.do_arm(False)

        elif cmd == 'takeoff':
            altitude = float(data.get('altitude', 30.0))
            self.do_takeoff(altitude)

        elif cmd == 'offboard':
            self.start_offboard_warmup()

        elif cmd == 'roll_hold':
            self.roll_hold_enabled = True
            self.target_roll_deg = clamp(float(data.get('roll_deg', 0.0)), -45.0, 45.0)
            self.target_altitude_m = self.altitude_m
            self.get_logger().info(
                f'ROLL HOLD ON target={self.target_roll_deg:.1f} deg alt={self.target_altitude_m:.1f} m'
            )

        elif cmd == 'roll_hold_off':
            self.roll_hold_enabled = False
            self.target_roll_deg = 0.0
            self.get_logger().info('ROLL HOLD OFF')

        elif cmd == 'pitch_hold':
            if not self.armed:
                self.get_logger().warn('pitch_hold rejected: aircraft not armed')
                return
            self.pitch_hold_enabled = True
            self.target_pitch_deg = clamp(float(data.get('pitch_deg', 0.0)), -20.0, 20.0)
            self.get_logger().info(f'PITCH HOLD ON target={self.target_pitch_deg:.1f} deg')

        elif cmd == 'pitch_hold_off':
            self.pitch_hold_enabled = False
            self.target_pitch_deg = 0.0
            self.get_logger().info('PITCH HOLD OFF')

        elif cmd == 'throttle_set':
            value = clamp(float(data.get('throttle', 0.0)), 0.0, 1.0)
            self.target_throttle = value
            self.cmd_throttle = value
            self.throttle_hold_enabled = True
            self.get_logger().info(f'THROTTLE HOLD ON target={self.target_throttle:.2f}')

        elif cmd == 'throttle_up':
            step = float(data.get('step', 0.025))
            base = self.target_throttle if self.throttle_hold_enabled else self.cmd_throttle
            value = clamp(base + step, 0.0, 1.0)
            self.target_throttle = value
            self.cmd_throttle = value
            self.throttle_hold_enabled = True
            self.get_logger().info(f'THROTTLE UP target={self.target_throttle:.2f}')

        elif cmd == 'throttle_down':
            step = float(data.get('step', 0.025))
            base = self.target_throttle if self.throttle_hold_enabled else self.cmd_throttle
            value = clamp(base - step, 0.0, 1.0)
            self.target_throttle = value
            self.cmd_throttle = value
            self.throttle_hold_enabled = True
            self.get_logger().info(f'THROTTLE DOWN target={self.target_throttle:.2f}')

        elif cmd == 'throttle_hold_off':
            self.throttle_hold_enabled = False
            self.get_logger().info('THROTTLE HOLD OFF')

    # ---------- Services ----------
    def do_arm(self, value: bool):
        if not self.arm_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/mavros/cmd/arming service unavailable')
            return

        req = CommandBool.Request()
        req.value = value
        future = self.arm_client.call_async(req)
        future.add_done_callback(self.arm_done_cb)

    def arm_done_cb(self, future):
        try:
            res = future.result()
            self.get_logger().info(f'arm response success={res.success} result={res.result}')
        except Exception as e:
            self.get_logger().error(f'arm service call failed: {e}')

    def do_takeoff(self, altitude: float):
        if self.use_alt_takeoff:
            self.get_logger().info('Using alternative takeoff method: setting mode to AUTO.TAKEOFF')
            self.do_set_mode('AUTO.TAKEOFF')
            return

        if not self.takeoff_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/mavros/cmd/takeoff service unavailable')
            return

        req = CommandTOL.Request()
        req.altitude = altitude
        req.latitude = 0.0
        req.longitude = 0.0
        req.min_pitch = 0.0
        req.yaw = 0.0

        future = self.takeoff_client.call_async(req)
        future.add_done_callback(self.takeoff_done_cb)

    def takeoff_done_cb(self, future):
        try:
            res = future.result()
            self.get_logger().info(f'takeoff response success={res.success} result={res.result}')
        except Exception as e:
            self.get_logger().error(f'takeoff service call failed: {e}')

    def do_set_mode(self, mode_name: str):
        if not self.mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('/mavros/set_mode service unavailable')
            return

        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode_name

        future = self.mode_client.call_async(req)
        future.add_done_callback(self.mode_done_cb)

    def mode_done_cb(self, future):
        try:
            res = future.result()
            self.get_logger().info(f'set_mode response mode_sent={res.mode_sent}')
        except Exception as e:
            self.get_logger().error(f'set_mode call failed: {e}')

    # ---------- Offboard ----------
    def start_offboard_warmup(self):
        self.offboard_request_pending = True
        self.offboard_mode_sent = False
        self.offboard_warmup_counter = 0
        self.get_logger().info('OFFBOARD warmup started')

    # ---------- Control / setpoint ----------
    def control_loop(self):
        should_stream = (
            self.offboard_request_pending
            or self.mode == 'OFFBOARD'
            or self.roll_hold_enabled
            or self.pitch_hold_enabled
            or self.throttle_hold_enabled
        )

        if not should_stream:
            return

        # Roll command
        if self.roll_hold_enabled:
            cmd_roll = self.target_roll_deg
        else:
            cmd_roll = self.roll_deg

        # Pitch command
        if self.pitch_hold_enabled:
            cmd_pitch = self.target_pitch_deg
        else:
            cmd_pitch = self.pitch_deg

        # Simple altitude assist during roll hold
        if self.roll_hold_enabled and (not self.pitch_hold_enabled) and self.enable_altitude_assist:
            alt_error = self.target_altitude_m - self.altitude_m
            pitch_correction = self.alt_p_gain * alt_error - self.alt_vz_gain * self.vertical_speed_mps
            pitch_correction = clamp(
                pitch_correction,
                -self.max_alt_pitch_correction_deg,
                self.max_alt_pitch_correction_deg
            )
            cmd_pitch = clamp(cmd_pitch + pitch_correction, -self.max_cmd_pitch_deg, self.max_cmd_pitch_deg)

        # Yaw command: keep current heading
        cmd_yaw = self.yaw_deg

        # Throttle command
        if self.throttle_hold_enabled:
            cmd_throttle = self.target_throttle
        else:
            cmd_throttle = self.cmd_throttle

        cmd_roll = clamp(cmd_roll, -self.max_cmd_roll_deg, self.max_cmd_roll_deg)
        cmd_pitch = clamp(cmd_pitch, -self.max_cmd_pitch_deg, self.max_cmd_pitch_deg)
        cmd_throttle = clamp(cmd_throttle, 0.0, 1.0)

        self.cmd_roll_deg = cmd_roll
        self.cmd_pitch_deg = cmd_pitch
        self.cmd_yaw_deg = cmd_yaw
        self.cmd_throttle = cmd_throttle

        self.publish_attitude_target(cmd_roll, cmd_pitch, cmd_yaw, cmd_throttle)
        
        if self.offboard_request_pending:
            self.offboard_warmup_counter += 1
            if self.offboard_warmup_counter >= self.offboard_warmup_required and not self.offboard_mode_sent:
                self.do_set_mode('OFFBOARD')
                self.offboard_mode_sent = True

            if self.mode == 'OFFBOARD':
                self.offboard_request_pending = False
                self.get_logger().info('OFFBOARD active')

    def publish_attitude_target(self, roll_deg: float, pitch_deg: float, yaw_deg: float, thrust: float):
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()

        qx, qy, qz, qw = euler_deg_to_quaternion(roll_deg, pitch_deg, yaw_deg)
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.body_rate.x = 0.0
        msg.body_rate.y = 0.0
        msg.body_rate.z = 0.0

        msg.thrust = thrust

        msg.type_mask = (
            AttitudeTarget.IGNORE_ROLL_RATE
            | AttitudeTarget.IGNORE_PITCH_RATE
            | AttitudeTarget.IGNORE_YAW_RATE
        )

        self.att_sp_pub.publish(msg)

    # ---------- Telemetry ----------
    def publish_telemetry(self):
        msg = {
            'mode': self.mode,
            'armed': self.armed,
            'roll': self.roll_deg,
            'pitch': self.pitch_deg,
            'yaw': self.yaw_deg,
            'altitude': self.altitude_m,
            'vertical_speed': self.vertical_speed_mps,
            'groundspeed': self.groundspeed_mps,
            'airspeed_text': 'N/A',

            'throttle_cmd': self.cmd_throttle,
            'throttle_hold': self.throttle_hold_enabled,
            'target_throttle': self.target_throttle,

            'roll_hold': self.roll_hold_enabled,
            'target_roll': self.target_roll_deg,
            'target_alt': self.target_altitude_m,

            'pitch_hold': self.pitch_hold_enabled,
            'target_pitch': self.target_pitch_deg,

            'imu_valid': self.imu_valid,
            'alt_valid': self.alt_valid,
            'odom_valid': self.odom_valid,
            'heading_valid': self.heading_valid,

            'offboard_pending': self.offboard_request_pending,
            'cmd_roll': self.cmd_roll_deg,
            'cmd_pitch': self.cmd_pitch_deg,
            'cmd_yaw': self.cmd_yaw_deg,
        }

        out = String()
        out.data = json.dumps(msg)
        self.telemetry_pub.publish(out)


def main(args: dict=None):
    rclpy.init()
    node = PlaneControlNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plane Control Node v4')
    parser.add_argument(
        '--alt_takeoff',
        action='store_true',
        help='Use alternative takeoff method by setting mode to AUTO.TAKEOFF instead of using the takeoff service'
    )
    args = parser.parse_args()
    main(vars(args))
