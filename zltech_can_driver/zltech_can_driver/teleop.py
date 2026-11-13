"""teleop - TELEOP BAN PHIM ROS 2: phim -> /cmd_vel (10 lan/s). Khong dung CAN, chay cung can_bridge.

    ros2 run zltech_can_driver teleop
    ros2 run zltech_can_driver teleop --ros-args -p v_max:=0.5 -p w_max:=1.0

    W / S : tang / giam van toc tien (10 lan bam = v_max)   A / D : quay trai / phai (10 lan bam = w_max)
    SPACE : dung em (v = w = 0, banh giam toc theo ramp)      X     : PHANH GAP (/emergency_brake)
    Q / Ctrl+C : thoat (gui v = w = 0 truoc)
Moi lan bam nhay ngay 1 bac (khong ramp nhu turtlebot3_teleop) -> bridge gui xuong banh ngay.
Teleop chet -> bridge het cmd_timeout (0.5 s) tu cho v = w = 0.
Mac dinh v_max / w_max = gioi han cua bridge (4 vong/s, banh 139 mm, rong 0.382 m).
"""
import math
import os
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Empty

V_MAX_MD = 4.0 * math.pi * 0.139    # 1.75 m/s
W_MAX_MD = V_MAX_MD / 0.382         # 4.57 rad/s


class Teleop(Node):
    def __init__(self):
        super().__init__("can_teleop")
        self.v_max = self.declare_parameter("v_max", V_MAX_MD).value
        self.w_max = self.declare_parameter("w_max", W_MAX_MD).value
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_brake = self.create_publisher(Empty, "emergency_brake", 10)
        self.v = self.w = 0.0
        self.fd = sys.stdin.fileno()
        self.co_bridge = None
        self.create_timer(1.0, self._kiem_bridge)
        self.create_timer(0.1, self._gui)
        self.create_timer(0.01, self._doc_phim)
        print("W/S tien-lui, A/D quay trai-phai, SPACE dung em, X PHANH GAP, Q thoat.\n")

    def _kiem_bridge(self):
        co = self.pub.get_subscription_count() > 0     # bridge co dang nghe cmd_vel khong (loi xem o terminal bridge)
        if co != self.co_bridge:
            self.co_bridge = co
            print("\n  [bridge] " + ("da ket noi" if co else
                  "KHONG thay can_bridge (chua chay / da tat) -> banh se khong quay"), flush=True)

    def _gui(self):
        m = Twist()
        m.linear.x, m.angular.z = self.v, self.w
        self.pub.publish(m)
        print(f"\r  v {self.v:+.2f} m/s  w {self.w:+.2f} rad/s   ", end="", flush=True)

    def _doc_phim(self):
        doi = False
        while select.select([sys.stdin], [], [], 0)[0]:
            c = os.read(self.fd, 1).decode(errors="ignore").lower()
            if c == "q":
                raise KeyboardInterrupt
            if c == "x":
                self.v = self.w = 0.0
                self.pub_brake.publish(Empty())
                print("\n  PHANH GAP", flush=True)
            elif c == "w":
                self.v += self.v_max / 10
            elif c == "s":
                self.v -= self.v_max / 10
            elif c == "a":
                self.w += self.w_max / 10
            elif c == "d":
                self.w -= self.w_max / 10
            elif c == " ":
                self.v = self.w = 0.0
            else:
                continue
            self.v = max(-self.v_max, min(self.v_max, round(self.v, 3)))
            self.w = max(-self.w_max, min(self.w_max, round(self.w, 3)))
            doi = True
        if doi:
            self._gui()                  # gui ngay khi bam phim, khong cho chu ky


def main():
    if not os.isatty(sys.stdin.fileno()):
        sys.exit("can_teleop.py can chay trong terminal")
    cu = termios.tcgetattr(sys.stdin.fileno())
    tty.setcbreak(sys.stdin.fileno())
    rclpy.init()
    node = Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, cu)
        if rclpy.ok():
            node.v = node.w = 0.0
            node._gui()
        print("\nThoat teleop.")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
