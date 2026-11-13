"""can_bridge - CAU DRIVER ROS 2: /cmd_vel -> 2 banh vi sai qua CAN (firmware Can-sync, SYNC_GO dong bo 2 banh).

    ros2 launch zltech_can_driver can_bridge.launch.py
    ros2 run zltech_can_driver can_bridge --ros-args --params-file config/can_bridge_params.yaml

Topic / service (ten topic doi duoc bang tham so cmd_vel_topic / odom_topic):
    sub  cmd_vel           geometry_msgs/Twist   linear.x [m/s], angular.z [rad/s]. Khong co lenh cmd_timeout s -> v = w = 0
    sub  vel_cmd           geometry_msgs/Twist   GIONG cmd_vel, tat mac dinh (vel_cmd_topic = ""). Dat ten khac
                                                 de nhan them 1 topic van toc; cung luc thi lenh moi nhat thang.
    sub  emergency_brake   std_msgs/Empty        PHANH GAP (PASSTHROUGH ve 0, cho dung roi tra VEL_RAMP)
    pub  wheel_odom        nav_msgs/Odometry     50 Hz, tich phan delta vi tri encoder (0x09); publish_tf -> TF
    pub  raw_encoder       sensor_msgs/JointState 50 Hz, encoder THO tu board (0x09), name [left_wheel, right_wheel]:
                                                 position [vong], velocity [vong/s], CHUA dao dau invert. "" = tat
    srv  ~/enable          std_srvs/Trigger      kiem tra heartbeat + vao CLOSED_LOOP
    srv  ~/disable         std_srvs/Trigger      2 banh ve IDLE
Log: moi lan cmd_vel doi gia tri 1 dong "cmd_vel moi"; moi status_period s 1 dong trang thai (cmd_vel nhan, lenh ap,
     vong/s gui xuong, vong/s encoder do, thieu ACK). File log: ~/.ros/log/

Giong odrive_controller (ms_robot_mini): cmd_vel moi -> gui SYNC NGAY trong callback, ngoai ra lap lai 20 lan/s.
Sau khi enable, lenh khac 0 bi chan cho toi khi thay 1 lenh dung (v = w = 0) -> teleop dang giu ga thi xe khong lao.
Mat lien lac / mat ACK (do 31/10: Waveshare dung hinh CA 2 CHIEU ~0.5-1 s, vai phut 1 lan, khong loi USB):
    KHONG tat, van gui lenh. Adapter song lai < 0.5 s -> chay tiep lien mach. Lau hon -> watchdog firmware tu cat
    (WATCHDOG, banh dung) -> lien lac ve thi TU xoa loi + vao lai CLOSED_LOOP + tang toc lai tu 0 theo ramp.
    Mat lien lac lien tuc > 2 s -> tat han (IDLE).
    "Mat lien lac" = khong frame nao trong 0.3 s / thieu ACK 2 chu ky / heartbeat vang > HB_TIMEOUT (3 s).
    Heartbeat vang ngan (13/11 do duoc ~1 s, encoder van ve) chi ghi canh bao "Heartbeat tre", khong cat.
Loi phan cung (motor / encoder / controller) -> IDLE, KHONG tu enable: sua xong goi
    ros2 service call /can_bridge/enable std_srvs/srv/Trigger
Sau khi tat han, tu enable lai (moi 2 s) khi ca 2 banh song, khong loi / chi WATCHDOG, VA lenh dang la v = w = 0.
Thoat (Ctrl+C, kill, dong terminal = SIGINT/SIGTERM/SIGHUP): toc do 0 -> VEL_RAMP -> IDLE roi moi dong CAN.
(kill -9 khong bat duoc: watchdog firmware tu cat sau 0.5 s, lan chay sau tu xoa WATCHDOG.)
"""
import math
import signal
import time

import can
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from zltech_can_driver import protocol as p

CHU_KY = 0.05                      # s, gui lai lenh dong bo (an watchdog CAN 0.5 s cua firmware)
ACK_TIMEOUT = 0.02                 # s cho ACK ca 2 node (trung vi do duoc ~4 ms)
MAT_ACK = 2                        # chu ky thieu ACK lien tiep -> coi nhu mat lien lac
MAT_LL_TOI_DA = 2.0                # s mat lien lac lien tuc -> tat han
# Song / chet xet theo frame encoder 0x09 (100/s, rx 0.3 s) + ACK. Heartbeat (10/s) chi de doc state / loi:
# 13/11 heartbeat 1 board vang ~1.04 s trong khi encoder + ACK van ve binh thuong -> hb_timeout cu 1.0 s bao
# "mat lien lac" sai -> 2 s sau cat IDLE. Chi coi la mat khi heartbeat vang > HB_TIMEOUT.
HB_TIMEOUT = 3.0                   # s
HB_TRE_CANH_BAO = 0.5              # s, khoang trong heartbeat lon hon -> ghi canh bao (khong cat)
VEL_DUNG = 0.05                    # vong/s, coi nhu da dung (phanh gap)
PHANH_TOI_DA = 1.5                 # s
# Chong banh tu chay (runaway): do 3/11 banh PHAI lenh 0 ma encoder do 0 -> -0.85 -> -6 vong/s trong ~2 s
# (sai offset / Hall nhieu -> commutation sai), firmware SPINOUT moi cat. Bridge cat som hon:
VUOT_LENH = 0.5                    # vong/s, |do| vuot |lenh lon nhat trong CUA_SO_LENH s| bao nhieu thi coi la vuot
CUA_SO_LENH = 1.0                  # s, firmware ramp 10 vong/s^2 -> giam tu 4 vong/s mat 0.4 s
VUOT_TOI_DA = 0.2                  # s vuot lien tuc -> IDLE


class CanBridge(Node):
    def __init__(self):
        super().__init__("can_bridge")
        par = lambda ten, md: self.declare_parameter(ten, md).value
        adapter = par("adapter", "waveshare")              # waveshare | socketcan | gs_usb | slcan
        port = par("port", "")                              # "" = tu do
        baud_serial = par("baud_serial", 2000000)
        self.left, self.right = par("left_node", 0), par("right_node", 2)
        invert = par("invert", "PHAI").upper()              # "" | TRAI | PHAI (banh lap nguoc)
        self.rong = par("wheel_separation", 0.382)
        self.dk = par("wheel_diameter", 0.139)
        self.vong_max = min(par("vmax", 4.0), 5.5)         # vong/s moi banh, board vel_limit 5.5
        self.cmd_timeout = par("cmd_timeout", 0.5)
        cmd_vel_topic = par("cmd_vel_topic", "cmd_vel")
        vel_cmd_topic = par("vel_cmd_topic", "")           # topic van toc phu, "" = tat (chi dung cmd_vel)
        odom_topic = par("odom_topic", "wheel_odom")
        raw_encoder_topic = par("raw_encoder_topic", "raw_encoder")   # encoder tho, "" = tat
        self.frame_odom, self.frame_base = par("odom_frame", "odom"), par("base_frame", "base_link")
        self.publish_tf = par("publish_tf", False)
        status_period = par("status_period", 1.0)          # s, 0 = tat log trang thai
        if self.left == self.right:
            raise SystemExit("left_node va right_node phai khac nhau")

        self.nodes = (self.left, self.right)
        self.ten = {self.left: "TRAI", self.right: "PHAI"}
        self.dau = {self.left: -1.0 if invert == "TRAI" else 1.0, self.right: -1.0 if invert == "PHAI" else 1.0}
        self.link = p.CanLink(self.nodes, adapter, port, baud_serial)
        self.tt = self.link.tt
        self.get_logger().info(f"CAN mo: {self.link.bus.channel_info}  TRAI = node {self.left}, "
                               f"PHAI = node {self.right}, dao {invert or 'khong'}")

        self.active = False
        self.v = self.w = 0.0
        self.t_cmd = 0.0
        self.t_phanh = None
        self.sai_ack = {n: 0 for n in self.nodes}
        self.t_gui_cuoi = 0.0              # lan cuoi gui SYNC (bat vong lap bi tre -> WATCHDOG)
        self.t_thu_enable = time.monotonic()
        self.cho_lenh_dung = True          # sau enable: chan lenh khac 0 cho toi khi thay lenh dung
        self.t_mat_ll = None               # thoi diem bat dau mat lien lac (None = binh thuong)
        self.x = self.y = self.th = 0.0
        self._pos_truoc = None             # (pos TRAI, pos PHAI) lan odom truoc
        self.so_cmd = 0                    # so cmd_vel nhan duoc trong ky log
        self.so_gui = self.so_thieu = 0    # so chu ky SYNC da gui / co thieu ACK trong ky log
        self.vong_gui = {n: 0.0 for n in self.nodes}
        self.ls_lenh = []                  # [(t, {node: vong/s gui})] trong CUA_SO_LENH s
        self.t_vuot = {n: None for n in self.nodes}
        self.khoa = ""                     # khac "" = da cat vi runaway: KHONG tu enable, chi ~/enable bang tay

        self.create_subscription(Twist, cmd_vel_topic, self._cb_cmd, 10)
        if vel_cmd_topic and vel_cmd_topic.lstrip("/") != cmd_vel_topic.lstrip("/"):
            self.create_subscription(Twist, vel_cmd_topic, self._cb_cmd, 10)
            self.get_logger().info(f"Nhan van toc tu {cmd_vel_topic} va {vel_cmd_topic} (linear.x, angular.z)")
        self.create_subscription(Empty, "emergency_brake", self._cb_brake, 10)
        self.pub_odom = self.create_publisher(Odometry, odom_topic, 10)
        self.pub_raw = self.create_publisher(JointState, raw_encoder_topic, 10) if raw_encoder_topic else None
        self.tf = TransformBroadcaster(self) if self.publish_tf else None
        self.create_service(Trigger, "~/enable", self._srv_enable)
        self.create_service(Trigger, "~/disable", self._srv_disable)
        self.create_timer(CHU_KY, self._loop)
        self.create_timer(0.02, self._odom)
        if status_period > 0:
            self.create_timer(status_period, self._log_trang_thai)

        ok, msg = self.enable()
        # KHONG viet (logger.info if ok else logger.error)(msg): rclpy cam 1 dong log doi muc -> ValueError, crash
        if ok:
            self.get_logger().info(msg)
        else:
            self.get_logger().error(msg)

    # ---------------------------------------------------------------- bat / tat
    def _cho_va_an(self, giay):
        """Cho `giay` s nhung van gui van toc 0 moi CHU_KY toi CA 2 node: banh con CLOSED_LOOP (watchdog dang
        bat) khong bi bo doi > 0.5 s -> khong bi WATCHDOG lay theo (13/11: sleep 0.6 s trong enable() lam
        banh TRAI dang chay cung bi cat -> 'KHONG enable')."""
        t_het = time.monotonic() + giay
        while True:
            self.link.van_toc_0()
            con = t_het - time.monotonic()
            if con <= 0:
                return
            time.sleep(min(CHU_KY, con))
    def _kiem_tra(self):
        """'' neu ca 2 banh song, sach loi, state IDLE/CLOSED_LOOP; nguoc lai mo ta loi."""
        loi = []
        for n in self.nodes:
            s = self.tt[n]
            if not s.t_hb or time.monotonic() - s.t_hb > HB_TIMEOUT:
                loi.append(f"{self.ten[n]} (node {n}) KHONG co heartbeat")
            elif s.loi() or s.state not in (p.IDLE, p.CLOSED_LOOP):
                loi.append(f"{self.ten[n]} (node {n}) state {s.state} {s.loi()}")
        return "; ".join(loi)

    def enable(self, cho_dung=True):
        """cho_dung=False: vao lai sau WATCHDOG do adapter dung hinh -> ap tiep lenh teleop (firmware ramp tu 0)."""
        self.active = False
        self.link.van_toc_0()              # 1 banh bi cat -> banh kia ve 0 ngay (khong quay tai cho), an watchdog
        self.link.cho_heartbeat(1.5)
        # Chi co WATCHDOG (0x800) = lan truoc dung ngang / adapter dung hinh -> tu xoa. Loi khac van chan.
        chi_wd = [n for n in self.nodes
                  if self.tt[n].axis_error == p.ERR_WATCHDOG and not any(self.tt[n].details.values())]
        if chi_wd:
            self.get_logger().warn(f"Xoa loi WATCHDOG con lai: {', '.join(self.ten[n] for n in chi_wd)}")
            self.link.xoa_loi(chi_wd)
            self._cho_va_an(0.6)           # KHONG time.sleep: banh kia dang CLOSED_LOOP se bi WATCHDOG
        loi = self._kiem_tra()
        if loi:
            return False, f"KHONG enable: {loi}"

        self.link.van_toc_0()              # input_vel cu con trong RAM -> VEL_RAMP se chay toi do ngay
        self.link.dat_input_mode(p.VEL_RAMP)
        self.link.dat_state(p.CLOSED_LOOP)
        t0 = t_an = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            if all(self.tt[n].state == p.CLOSED_LOOP and self.link.con_song(n, hb_timeout=HB_TIMEOUT) for n in self.nodes):
                if cho_dung:
                    self.v = self.w = 0.0
                self.t_phanh = None
                self.sai_ack = {n: 0 for n in self.nodes}
                self.t_mat_ll = None
                self.t_gui_cuoi = time.monotonic()
                self.cho_lenh_dung = cho_dung
                self.active = True
                return True, f"CLOSED_LOOP ca 2 banh sau {time.monotonic() - t0:.2f} s"
            if any(self.tt[n].loi() for n in self.nodes):
                break
            if time.monotonic() - t_an >= CHU_KY:
                self.link.van_toc_0()      # an watchdog CAN trong luc cho
                t_an = time.monotonic()
            time.sleep(0.02)
        self.link.dat_state(p.IDLE)
        return False, "KHONG vao duoc CLOSED_LOOP: " + "; ".join(
            f"{self.ten[n]} state {self.tt[n].state} {self.tt[n].loi()}" for n in self.nodes)

    def disable(self, ly_do):
        was = self.active
        self.active = False
        try:
            self.link.dua_ve_idle()
        except can.CanError as e:
            self.get_logger().error(f"gui IDLE loi: {e}")
        if was:
            self.get_logger().warn(f"2 banh -> IDLE: {ly_do}")

    def _srv_enable(self, req, res):
        if self.khoa:
            self.get_logger().warn(f"~/enable bang tay -> bo khoa ({self.khoa})")
            self.khoa = ""
        res.success, res.message = self.enable()
        return res

    def _srv_disable(self, req, res):
        self.disable("service disable")
        res.success, res.message = True, "IDLE"
        return res

    def _lenh_dung(self):
        return time.monotonic() - self.t_cmd > self.cmd_timeout or (self.v == 0.0 and self.w == 0.0)

    def _tu_enable(self):
        """Khong active: thu enable lai moi 2 s neu loi chi la WATCHDOG / khong loi va lenh dang dung.
        Da cat vi runaway (self.khoa) -> KHONG tu enable (3/11: tu enable lai -> banh runaway lan nua)."""
        if self.khoa or time.monotonic() - self.t_thu_enable < 2.0 or not self._lenh_dung():
            return
        self.t_thu_enable = time.monotonic()
        if not all(self.link.con_song(n, hb_timeout=HB_TIMEOUT) for n in self.nodes):
            return
        for n in self.nodes:
            s = self.tt[n]
            if any(s.details.values()) or s.axis_error & ~p.ERR_WATCHDOG:
                return                       # loi phan cung -> phai goi ~/enable bang tay
        ok, msg = self.enable()
        if ok:
            self.get_logger().info(f"Tu enable lai: {msg}")
        else:
            self.get_logger().warn(f"Tu enable lai: {msg}")

    # ---------------------------------------------------------------- callback
    def _cb_cmd(self, msg):
        doi = (msg.linear.x, msg.angular.z) != (self.v, self.w)
        if doi:
            self.get_logger().info(f"cmd_vel moi: v {msg.linear.x:+.3f} m/s  w {msg.angular.z:+.3f} rad/s"
                                   f"{'' if self.active else '  (CHUA enable -> bo qua)'}")
        self.v, self.w = msg.linear.x, msg.angular.z
        self.t_cmd = time.monotonic()
        self.so_cmd += 1
        if doi and self.active and self.t_phanh is None:
            self._ap_lenh()                  # gui ngay, khong cho chu ky (nhu odrive_controller)

    def _cb_brake(self, _):
        if not self.active or self.t_phanh is not None:
            return
        self.v = self.w = 0.0
        self.link.van_toc_0()                # input_vel = 0 TRUOC khi doi PASSTHROUGH (tranh vot lenh cu)
        self.link.dat_input_mode(p.PASSTHROUGH)
        self.t_phanh = time.monotonic()
        self.get_logger().warn("PHANH GAP")

    def _log_trang_thai(self):
        L, R = self.left, self.right
        if not self.active:
            ly_do = "; ".join(f"{self.ten[n]} state {self.tt[n].state} {self.tt[n].loi()}" for n in self.nodes)
            cho = (f"  -> KHOA ({self.khoa}): ke banh len, calib, roi ros2 service call /can_bridge/enable "
                   f"std_srvs/srv/Trigger" if self.khoa else
                   "" if self._lenh_dung() else "  -> bam DUNG (v = w = 0) tren teleop de tu enable lai")
            self.get_logger().warn(f"KHONG active ({ly_do})  cmd_vel nhan {self.so_cmd}{cho}",
                                   throttle_duration_sec=5.0)
        else:
            cu = time.monotonic() - self.t_cmd
            nguon = "PHANH" if self.t_phanh is not None else (
                "cmd_vel" if cu <= self.cmd_timeout else
                ("CHUA co cmd_vel" if not self.t_cmd else f"cmd_vel cu {cu:.1f}s -> 0"))
            self.get_logger().info(
                f"cmd_vel nhan {self.so_cmd}/ky | ap v {self.v:+.2f} w {self.w:+.2f} ({nguon}) | "
                f"gui TRAI {self.vong_gui[L]:+.2f} PHAI {self.vong_gui[R]:+.2f} vong/s | "
                f"do TRAI {self.tt[L].vel:+.2f} PHAI {self.tt[R].vel:+.2f} vong/s | "
                f"thieu ACK {self.so_thieu}/{self.so_gui}")
        self.so_cmd = self.so_gui = self.so_thieu = 0
        # Chan doan frame nhan: binh thuong moi giay ~10 heartbeat, ~100 encoder moi node
        thong_ke = {n: (self.tt[n].so_hb, self.tt[n].so_enc, self.tt[n].hb_gap_max) for n in self.nodes}
        for n in self.nodes:
            self.tt[n].so_hb = self.tt[n].so_enc = 0
            self.tt[n].hb_gap_max = 0.0
        tre = [n for n in self.nodes if thong_ke[n][2] > HB_TRE_CANH_BAO]
        if tre:
            self.get_logger().warn(
                "Heartbeat tre (khong cat, encoder/ACK van song): " + "; ".join(
                    f"{self.ten[n]} vang {thong_ke[n][2]:.2f} s" for n in tre) + " | ky nay nhan " + "; ".join(
                    f"{self.ten[n]} hb {thong_ke[n][0]} enc {thong_ke[n][1]}" for n in self.nodes))

    # ---------------------------------------------------------------- dieu khien
    def _toc_do_banh(self, v, w):
        """Dong hoc vi sai (body twist -> banh):
            v_left  = v_b,x - w_b,z * W / 2        v_b,x = linear.x [m/s] (tien +), w_b,z = angular.z [rad/s] (trai +)
            v_right = v_b,x + w_b,z * W / 2        W = wheel_separation [m]
        roi m/s -> vong/s (chia pi * D), nhan dau lap nguoc (invert), giu ti le trai/phai neu cham vmax."""
        m_tren_vong = math.pi * self.dk
        vl = (v - w * self.rong / 2) / m_tren_vong
        vr = (v + w * self.rong / 2) / m_tren_vong
        k = max(1.0, abs(vl) / self.vong_max, abs(vr) / self.vong_max)   # giu ti le trai/phai khi cham gioi han
        return {self.left: self.dau[self.left] * vl / k, self.right: self.dau[self.right] * vr / k}

    def _kiem_luong_nhan(self):
        if self.link.so_loi_nhan:
            self.get_logger().warn(f"Luong nhan: {self.link.so_loi_nhan} frame serial hong (bo qua), "
                                   f"cuoi: {self.link.loi_nhan_cuoi}")
            self.link.so_loi_nhan = 0
        if self.link.kiem_luong_nhan():
            self.get_logger().error("Luong nhan CAN da chet -> da khoi dong lai")

    def _loop(self):
        self._kiem_luong_nhan()
        if not self.active:
            self._tu_enable()
            return
        now = time.monotonic()
        mat = [n for n in self.nodes if not self.link.con_song(n, hb_timeout=HB_TIMEOUT) or self.sai_ack[n] >= MAT_ACK]
        if mat:
            if self.t_mat_ll is None:
                self.t_mat_ll = now
                self.get_logger().warn("Mat lien lac " + ", ".join(
                    f"{self.ten[n]} (frame cuoi {(now - self.tt[n].t_rx) * 1000:.0f} ms, heartbeat cuoi "
                    f"{(now - self.tt[n].t_hb) * 1000:.0f} ms, thieu ACK {self.sai_ack[n]})"
                    for n in self.nodes) + " -> van gui lenh, cho adapter song lai")
            elif now - self.t_mat_ll > MAT_LL_TOI_DA:
                ly_do = f"mat lien lac lien tuc {now - self.t_mat_ll:.1f} s"
                self.disable(ly_do)
                self.get_logger().error(f"{ly_do} -> kiem tra adapter / day CAN / nguon board")
                self.t_mat_ll = None
                return
        else:
            if self.t_mat_ll is not None:
                self.get_logger().info(f"Lien lac hoi phuc sau {now - self.t_mat_ll:.2f} s")
                self.t_mat_ll = None
            # du lieu moi -> moi xet state / loi
            for n in self.nodes:
                s = self.tt[n]
                if any(s.details.values()) or s.axis_error & ~p.ERR_WATCHDOG:
                    ly_do = f"{self.ten[n]} loi phan cung: {s.loi()}"
                    self.disable(ly_do)
                    self.get_logger().error(f"{ly_do} -> sua xong goi ~/enable (hoac ros2 run zltech_can_driver calib)")
                    return
            bi_cat = [n for n in self.nodes if self.tt[n].state != p.CLOSED_LOOP or self.tt[n].axis_error]
            if bi_cat:
                self.get_logger().warn(
                    "Bi cat khoi CLOSED_LOOP (" + "; ".join(
                        f"{self.ten[n]} state {self.tt[n].state} {self.tt[n].loi()}" for n in bi_cat) +
                    ") -> tu vao lai, tang toc lai tu 0")
                ok, msg = self.enable(cho_dung=False)
                if ok:
                    self.get_logger().info(f"Vao lai: {msg}")
                else:
                    self.get_logger().error(f"Vao lai: {msg}")
                return

        if self._kiem_runaway(now):
            return

        if self.t_phanh is not None:
            da_dung = all(abs(self.tt[n].vel) < VEL_DUNG for n in self.nodes)
            if da_dung or time.monotonic() - self.t_phanh >= PHANH_TOI_DA:
                self.link.dat_input_mode(p.VEL_RAMP)
                self.get_logger().info(f"Phanh xong sau {time.monotonic() - self.t_phanh:.2f} s -> VEL_RAMP")
                self.t_phanh = None

        self._ap_lenh()

    def _kiem_runaway(self, now):
        """True neu da cat vi banh chay nhanh hon lenh (khong ap dung luc dang phanh gap)."""
        self.ls_lenh = [(t, v) for t, v in self.ls_lenh if now - t <= CUA_SO_LENH]
        if self.t_phanh is not None or not self.ls_lenh:
            return False
        for n in self.nodes:
            gioi_han = max(abs(v[n]) for _, v in self.ls_lenh) + VUOT_LENH
            do = self.tt[n].vel
            if abs(do) <= gioi_han:
                self.t_vuot[n] = None
                continue
            if self.t_vuot[n] is None:
                self.t_vuot[n] = now
            elif now - self.t_vuot[n] >= VUOT_TOI_DA:
                ly_do = (f"{self.ten[n]} CHAY VUOT LENH: do {do:+.2f} vong/s, lenh toi da {gioi_han - VUOT_LENH:.2f} "
                         f"vong/s suot {now - self.t_vuot[n]:.2f} s (nghi sai offset encoder / Hall nhieu)")
                self.disable(ly_do)
                self.khoa = f"{self.ten[n]} runaway"
                self.get_logger().error(f"{ly_do} -> KHOA, khong tu chay lai. KE BANH LEN, calib lai "
                                        f"(ros2 run zltech_can_driver calib) roi goi ~/enable")
                self.t_vuot = {k: None for k in self.nodes}
                self.ls_lenh = []
                return True
        return False

    def _ap_lenh(self):
        if self.cho_lenh_dung and self._lenh_dung():
            self.cho_lenh_dung = False
        if self.t_phanh is not None or time.monotonic() - self.t_cmd > self.cmd_timeout:
            v = w = 0.0                      # dang phanh / teleop chet -> dung
        elif self.cho_lenh_dung:
            v = w = 0.0                      # vua enable ma teleop dang giu ga -> cho lenh dung
            self.get_logger().warn("Vua enable, lenh dang khac 0 -> bam DUNG tren teleop truoc",
                                   throttle_duration_sec=2.0)
        else:
            v, w = self.v, self.w
        self.vong_gui = self._toc_do_banh(v, w)
        tre = time.monotonic() - self.t_gui_cuoi
        if tre > 0.2:
            self.get_logger().warn(f"Vong gui lenh bi tre {tre:.2f} s (watchdog firmware cat o 0.5 s)")
        self.ls_lenh.append((time.monotonic(), dict(self.vong_gui)))
        thieu = self.link.gui_dong_bo(self.vong_gui, ACK_TIMEOUT)
        self.t_gui_cuoi = time.monotonic()
        self.so_gui += 1
        self.so_thieu += bool(thieu)
        for n in self.nodes:
            self.sai_ack[n] = self.sai_ack[n] + 1 if n in thieu else 0

    def _odom(self):
        """Nhu computeOdom() cua odrive_controller: tich phan DELTA VI TRI encoder (0x09 pos, vong) -> x, y, theta;
        van toc tu vel encoder. 50 Hz, cung covariance."""
        L, R = self.left, self.right
        m_tren_vong = math.pi * self.dk
        pos = (self.tt[L].pos, self.tt[R].pos)
        if not (self.tt[L].t_rx and self.tt[R].t_rx):
            return                           # chua co encoder
        stamp = self.get_clock().now().to_msg()
        if self.pub_raw:
            j = JointState()
            j.header.stamp = stamp
            j.name = ["left_wheel", "right_wheel"]
            j.position = [self.tt[L].pos, self.tt[R].pos]     # vong, nguyen tu board (chua nhan invert)
            j.velocity = [self.tt[L].vel, self.tt[R].vel]     # vong/s
            self.pub_raw.publish(j)
        if self._pos_truoc is None:
            self._pos_truoc = pos
        dL = (pos[0] - self._pos_truoc[0]) * self.dau[L] * m_tren_vong
        dR = (pos[1] - self._pos_truoc[1]) * self.dau[R] * m_tren_vong
        self._pos_truoc = pos
        if max(abs(dL), abs(dR)) > 0.5:      # board khoi dong lai (pos ve 0) -> bo buoc nhay
            self.get_logger().warn(f"Odom: bo buoc nhay encoder TRAI {dL:+.2f} m, PHAI {dR:+.2f} m")
            dL = dR = 0.0
        dc, dth = (dL + dR) / 2, (dR - dL) / self.rong
        self.x += dc * math.cos(self.th + dth / 2)
        self.y += dc * math.sin(self.th + dth / 2)
        self.th += dth
        vl = self.dau[L] * self.tt[L].vel * m_tren_vong
        vr = self.dau[R] * self.tt[R].vel * m_tren_vong
        v, w = (vl + vr) / 2, (vr - vl) / self.rong

        qz, qw = math.sin(self.th / 2), math.cos(self.th / 2)
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id, o.child_frame_id = self.frame_odom, self.frame_base
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x, o.twist.twist.angular.z = v, w
        o.pose.covariance[0] = o.pose.covariance[7] = 1e-3     # giong odrive_controller
        o.pose.covariance[35] = 1e-2
        o.twist.covariance[0] = 1e-3
        o.twist.covariance[35] = 1e-2
        self.pub_odom.publish(o)
        if self.tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id, t.child_frame_id = self.frame_odom, self.frame_base
            t.transform.translation.x, t.transform.translation.y = self.x, self.y
            t.transform.rotation.z, t.transform.rotation.w = qz, qw
            self.tf.sendTransform(t)

    def destroy_node(self):
        self.disable("tat node")
        self.link.dong()
        self.get_logger().info("2 banh -> IDLE, dong CAN.")
        super().destroy_node()


def _thoat(sig, _frame):
    raise SystemExit(f"tin hieu {signal.Signals(sig).name}")


def main():
    rclpy.init()
    # kill (SIGTERM) / dong terminal (SIGHUP): thoat qua finally -> banh ve 0 + IDLE (mac dinh Python chet ngay)
    signal.signal(signal.SIGTERM, _thoat)
    signal.signal(signal.SIGHUP, _thoat)
    node = CanBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        p.bo_qua_tin_hieu()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
