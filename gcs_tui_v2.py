#!/usr/bin/env python3
import json
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GCSTUI(Node):
    def __init__(self):
        super().__init__('gcs_tui')
        self.telemetry = {}
        self.cmd_pub = self.create_publisher(String, '/gcs/cmd', 10)
        # self.create_subscription(String, '/gcs/telemetry', self.telemetry_cb, 10)
        threading.Thread(target=self.keyboard_loop, daemon=True).start()
        # self.keyboard_loop()

    def telemetry_cb(self, msg: String):
        try:
            self.telemetry = json.loads(msg.data)
        except Exception:
            return
        self.print_screen()

    def send_cmd(self, data: dict):
        msg = String()
        msg.data = json.dumps(data)
        self.cmd_pub.publish(msg)

    def yes_no(self, v):
        return 'OK' if v else '--'

    def print_screen(self):
        print('\033c', end='')
        t = self.telemetry
        print('=== PX4 Plane GCS TUI v2 ===')
        print('a=ARM  z=DISARM  t=TAKEOFF')
        print('r=ROLL+10  l=ROLL-10  o=ROLL OFF')
        print('u=PITCH+10  d=PITCH-10  p=PITCH OFF')
        print('1=THR 30%  2=THR 50%  3=THR 70%')
        print('+=THR UP  -=THR DOWN  0=THR OFF')
        print('m=OFFBOARD  q=QUIT')
        print('指令會持續保持，直到對應 OFF。ROLL / PITCH / THROTTLE 可同時存在。\n')
        # if not t:
        #     print('waiting telemetry...')
        #     return
        print(f"Mode             : {t.get('mode', '')}")
        print(f"Armed            : {t.get('armed', False)}")
        print()
        print(f"Roll             : {t.get('roll', 0.0):.2f} deg")
        print(f"Pitch            : {t.get('pitch', 0.0):.2f} deg")
        print(f"Yaw              : {t.get('yaw', 0.0):.2f} deg")
        print(f"Altitude         : {t.get('altitude', 0.0):.2f} m")
        print(f"Vertical Speed   : {t.get('vertical_speed', 0.0):.2f} m/s")
        print(f"Groundspeed      : {t.get('groundspeed', 0.0):.2f} m/s")
        print(f"Airspeed         : {t.get('airspeed_text', 'N/A')}")
        print()
        print(f"Throttle Cmd     : {t.get('throttle_cmd', 0.0):.2f}")
        print(f"Throttle Hold    : {t.get('throttle_hold', False)}  target={t.get('target_throttle', 0.0):.2f}")
        print(f"Roll Hold        : {t.get('roll_hold', False)}  target={t.get('target_roll', 0.0):.2f}")
        print(f"Target Altitude  : {t.get('target_alt', 0.0):.2f} m")
        print(f"Pitch Hold       : {t.get('pitch_hold', False)}  target={t.get('target_pitch', 0.0):.2f} deg")
        print()
        print(
            'Data Valid       : '
            f"IMU {self.yes_no(t.get('imu_valid', False))} / "
            f"ALT {self.yes_no(t.get('alt_valid', False))} / "
            f"ODOM {self.yes_no(t.get('odom_valid', False))} / "
            f"HDG {self.yes_no(t.get('heading_valid', False))}"
        )

    def keyboard_loop(self):
        while True:
            key = input().strip().lower()
            if key == 'a':
                self.send_cmd({'cmd': 'arm'})
            elif key == 'z':
                self.send_cmd({'cmd': 'disarm'})
            elif key == 't':
                alt_str = input('takeoff altitude(m): ').strip()
                try:
                    alt = float(alt_str)
                except Exception:
                    alt = 30.0
                self.send_cmd({'cmd': 'takeoff', 'altitude': alt})
            elif key == 'r':
                self.send_cmd({'cmd': 'roll_hold', 'roll_deg': 10.0})
            elif key == 'l':
                self.send_cmd({'cmd': 'roll_hold', 'roll_deg': -10.0})
            elif key == 'o':
                self.send_cmd({'cmd': 'roll_hold_off'})
            elif key == 'u':
                self.send_cmd({'cmd': 'pitch_hold', 'pitch_deg': 10.0})
            elif key == 'd':
                self.send_cmd({'cmd': 'pitch_hold', 'pitch_deg': -10.0})
            elif key == 'p':
                self.send_cmd({'cmd': 'pitch_hold_off'})
            elif key == '1':
                self.send_cmd({'cmd': 'throttle_set', 'throttle': 0.30})
            elif key == '2':
                self.send_cmd({'cmd': 'throttle_set', 'throttle': 0.50})
            elif key == '3':
                self.send_cmd({'cmd': 'throttle_set', 'throttle': 0.80})
            elif key == '+':
                self.send_cmd({'cmd': 'throttle_up', 'step': 0.025})
            elif key == '-':
                self.send_cmd({'cmd': 'throttle_down', 'step': 0.025})
            elif key == '0':
                self.send_cmd({'cmd': 'throttle_hold_off'})
            elif key == 'm':
                self.send_cmd({'cmd': 'offboard'})
            elif key == 'q':
                break


def main():
    rclpy.init()
    node = GCSTUI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
