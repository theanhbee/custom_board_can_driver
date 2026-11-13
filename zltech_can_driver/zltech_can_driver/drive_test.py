"""drive_test - LAI THU 2 BANH TRUC TIEP QUA CAN, KHONG CAN ROS (ban Linux cua 29_can_control.py).

    ros2 run zltech_can_driver drive_test                      # Waveshare tu do CH340, trai = node 0, phai = node 2
    ros2 run zltech_can_driver drive_test --dao=PHAI           # banh lap nguoc (TRAI / PHAI / '' = khong dao)
    ros2 run zltech_can_driver drive_test --vmax=2.0           # toc do toi da moi banh (vong/s, board vel_limit 5.5)
    ros2 run zltech_can_driver drive_test --rong=0.382 --duong-kinh=0.139 --tre-ms=5 --xoa-loi
    ros2 run zltech_can_driver drive_test --adapter=socketcan --cong=can0

Buoc 1 - KIEM TRA: nghe heartbeat 2 s. Phai thay CA 2 node, state IDLE / CLOSED_LOOP, khong loi. Chi WATCHDOG (lan
         truoc dung ngang) -> tu xoa. Loi khac -> thoat.
Buoc 2 - VAO CLOSED_LOOP ca 2 node (toi da 3 s).
Buoc 3 - LAI (phim trong terminal, khong can Enter):
         W / S : tang / giam van toc tien (10 lan bam = vmax)  A / D : quay trai / phai (10 lan bam = toi da)
         SPACE : dung em (ramp)                                X     : PHANH GAP (PASSTHROUGH ve 0, roi tra VEL_RAMP)
         Q / Ctrl+C : thoat, 2 banh ve IDLE
Moi 0.1 s (va ngay khi bam phim): 0x1E staged ca 2 node -> cho ACK -> 0x7FF SYNC_GO.
An toan: banh mat lien lac (0.3 s khong frame / 1 s khong heartbeat), roi CLOSED_LOOP, bao loi, hoac thieu ACK
        3 chu ky lien tiep -> ca 2 ve IDLE. Day la tool THU: dung ngay khi co loi (khong tu phuc hoi nhu can_bridge).
Phai TAT can_bridge truoc (cong bi khoa).
"""
import argparse
import math
import os
import select
import sys
import termios
import time
import tty

import can

from zltech_can_driver import protocol as p

CHU_KY = 0.1                       # s, gui lai lenh dong bo
ACK_TIMEOUT = 0.015                # s cho ACK
SAI_ACK_TOI_DA = 3                 # chu ky thieu ACK lien tiep (0.3 s) -> dung xe
VEL_DUNG = 0.05                    # vong/s, coi nhu da dung
PHANH_TOI_DA = 1.5                 # s


class BanPhim:
    """Terminal o che do cbreak -> doc tung phim khong can Enter, Ctrl+C van hoat dong."""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.cu = None

    def __enter__(self):
        if os.isatty(self.fd):
            self.cu = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, *a):
        if self.cu is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.cu)

    def kbhit(self):
        return bool(select.select([sys.stdin], [], [], 0)[0])

    def getch(self):
        return os.read(self.fd, 1).decode(errors="ignore")


class DriveTest:
    def __init__(self, args):
        self.args = args
        self.ten = p.ten_banh(args)
        self.left, self.right = args.trai, args.phai
        self.nodes = (self.left, self.right)
        dao = args.dao.upper()
        self.dau = {self.left: -1.0 if dao == "TRAI" else 1.0, self.right: -1.0 if dao == "PHAI" else 1.0}
        self.vong_max = min(args.vmax, 5.5)
        self.v_max = self.vong_max * math.pi * args.duong_kinh
        self.w_max = self.v_max / args.rong
        self.link = p.CanLink(self.nodes, args.adapter, args.cong, args.baud_serial)
        self.tt = self.link.tt

    def kiem_tra_heartbeat(self):
        """Buoc 1. True neu ca 2 banh song va sach loi."""
        print(f"Nghe heartbeat 2 s: TRAI = node {self.left}, PHAI = node {self.right}...")
        time.sleep(2.0)
        ok = True
        for n in self.nodes:
            s = self.tt[n]
            if not s.t_hb:
                print(f"  !! {self.ten[n]} (node {n}): KHONG co heartbeat -> sai node_id / mat nguon / day CAN")
                ok = False
                continue
            loi = s.loi()
            trang_thai = "OK" if not loi and s.state in (p.IDLE, p.CLOSED_LOOP) else "LOI"
            print(f"  {self.ten[n]} (node {n}): state {p.STATE_NAMES.get(s.state, s.state)}, "
                  f"axis_error 0x{s.axis_error:X}  {trang_thai}  {loi}")
            ok = ok and trang_thai == "OK"
        return ok

    def xoa_loi(self, nodes):
        print(f"Gui Clear_Errors cho {', '.join(self.ten[n] for n in nodes)}...")
        self.link.xoa_loi(nodes)
        time.sleep(0.3)

    def vao_closed_loop(self):
        """Buoc 2."""
        print("Vao CLOSED_LOOP ca 2 banh...")
        self.link.van_toc_0()          # input_vel cu con trong RAM -> VEL_RAMP se chay toi do ngay
        self.link.dat_input_mode(p.VEL_RAMP)
        self.link.dat_state(p.CLOSED_LOOP)
        t0 = t_an = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            if all(self.tt[n].state == p.CLOSED_LOOP and self.link.con_song(n) for n in self.nodes):
                print(f"  OK sau {time.monotonic() - t0:.2f} s")
                return True
            if any(self.tt[n].loi() for n in self.nodes):
                break
            if time.monotonic() - t_an >= CHU_KY:
                self.link.van_toc_0()  # an watchdog CAN trong luc cho
                t_an = time.monotonic()
            time.sleep(0.02)
        for n in self.nodes:
            print(f"  {self.ten[n]}: state {self.tt[n].state}  {self.tt[n].loi()}")
        return False

    def toc_do_banh(self, v, w):
        m_tren_vong = math.pi * self.args.duong_kinh
        vl = (v - w * self.args.rong / 2) / m_tren_vong
        vr = (v + w * self.args.rong / 2) / m_tren_vong
        k = max(1.0, abs(vl) / self.vong_max, abs(vr) / self.vong_max)   # giu ti le trai/phai khi cham gioi han
        return {self.left: self.dau[self.left] * vl / k, self.right: self.dau[self.right] * vr / k}

    def gui_dong_bo(self, vong):
        t0 = time.monotonic()
        thieu = self.link.gui_dong_bo(vong, ACK_TIMEOUT, self.args.tre_ms / 1000.0)
        return time.monotonic() - t0, thieu

    def ly_do_dung(self):
        now = time.monotonic()
        for n in self.nodes:
            s = self.tt[n]
            if not self.link.con_song(n):
                return (f"{self.ten[n]} mat lien lac (frame cuoi {now - s.t_rx:.2f} s,"
                        f" heartbeat cuoi {now - s.t_hb:.2f} s truoc)")
            if s.state != p.CLOSED_LOOP:
                return f"{self.ten[n]} roi CLOSED_LOOP (state {s.state})"
            if s.loi():
                return f"{self.ten[n]} loi: {s.loi()}"
        return None

    def lai(self, kb):
        """Buoc 3."""
        print("\nW/S tien-lui, A/D quay trai-phai, SPACE dung em, X PHANH GAP, Q thoat.\n")
        buoc_v, buoc_w = self.v_max / 10, self.w_max / 10
        v = w = 0.0
        sai_ack = {n: 0 for n in self.nodes}
        so_chu_ky = so_thieu = 0
        tre_max = 0.0
        t_gui = 0.0
        t_phanh = None
        while True:
            doi = False
            while kb.kbhit():
                c = kb.getch().lower()
                if c == "q":
                    return "nguoi dung thoat"
                if t_phanh is not None:
                    continue           # dang phanh: bo qua phim lai
                if c == "x":
                    v = w = 0.0
                    self.link.van_toc_0()                  # ve 0 TRUOC khi doi PASSTHROUGH (tranh vot lenh cu)
                    self.link.dat_input_mode(p.PASSTHROUGH)
                    t_phanh = time.monotonic()
                    print("\n  PHANH GAP...", flush=True)
                    continue
                if c == "w":
                    v += buoc_v
                elif c == "s":
                    v -= buoc_v
                elif c == "a":
                    w += buoc_w
                elif c == "d":
                    w -= buoc_w
                elif c == " ":
                    v = w = 0.0
                else:
                    continue
                v = max(-self.v_max, min(self.v_max, round(v, 3)))
                w = max(-self.w_max, min(self.w_max, round(w, 3)))
                doi = True

            ly_do = self.ly_do_dung()
            if ly_do:
                return ly_do

            if t_phanh is not None:
                da_dung = all(abs(self.tt[n].vel) < VEL_DUNG for n in self.nodes)
                if da_dung or time.monotonic() - t_phanh >= PHANH_TOI_DA:
                    self.link.dat_input_mode(p.VEL_RAMP)
                    print(f"  {'da dung' if da_dung else f'het {PHANH_TOI_DA} s'} sau "
                          f"{time.monotonic() - t_phanh:.2f} s -> tra lai VEL_RAMP", flush=True)
                    t_phanh = None

            if doi or time.monotonic() - t_gui >= CHU_KY:
                vong = self.toc_do_banh(v, w)
                tre, thieu = self.gui_dong_bo(vong)
                t_gui = time.monotonic()
                so_chu_ky += 1
                so_thieu += bool(thieu)
                tre_max = max(tre_max, tre)
                for n in self.nodes:
                    sai_ack[n] = sai_ack[n] + 1 if n in thieu else 0
                    if sai_ack[n] >= SAI_ACK_TOI_DA:
                        return f"{self.ten[n]} mat ACK staged {SAI_ACK_TOI_DA} chu ky lien tiep"
                print(f"\r  v {v:+.2f} m/s  w {w:+.2f} rad/s  |  TRAI {vong[self.left]:+.2f}  "
                      f"PHAI {vong[self.right]:+.2f} vong/s  |  GO sau {tre * 1000:4.1f} ms "
                      f"(max {tre_max * 1000:5.1f})  thieu ACK {so_thieu}/{so_chu_ky}   ", end="", flush=True)
            time.sleep(0.005)

    def chay(self):
        try:
            if self.args.xoa_loi:
                self.xoa_loi(self.nodes)
            ok = self.kiem_tra_heartbeat()
            chi_wd = [n for n in self.nodes
                      if self.tt[n].axis_error == p.ERR_WATCHDOG and not any(self.tt[n].details.values())]
            if not ok and chi_wd and all(not self.tt[n].loi() or n in chi_wd for n in self.nodes):
                print(f"  -> chi la loi WATCHDOG con lai tu lan chay truoc "
                      f"({', '.join(self.ten[n] for n in chi_wd)}): tu xoa")
                self.xoa_loi(chi_wd)
                ok = self.kiem_tra_heartbeat()
            if not ok:
                sys.exit("!! Heartbeat chua dat -> KHONG chay. Sua loi (clear_errors / calib) roi chay lai.")
            if not self.vao_closed_loop():
                sys.exit("!! Khong vao duoc CLOSED_LOOP ca 2 banh.")
            with BanPhim() as kb:
                ly_do = self.lai(kb)
            print(f"\n\nDung: {ly_do}")
            for n in self.nodes:
                if self.tt[n].loi():
                    print(f"  {self.ten[n]}: {self.tt[n].loi()}")
        except KeyboardInterrupt:
            print("\n\nDung: Ctrl+C")
        finally:
            p.bo_qua_tin_hieu()
            try:
                self.link.dua_ve_idle()
            except can.CanError as e:
                print(f"!! gui IDLE loi: {e}")
            print("2 banh -> IDLE.")
            self.link.dong()


def main():
    ap = p.them_tham_so_can(argparse.ArgumentParser(description="Lai thu 2 banh qua CAN (khong ROS)"))
    ap.add_argument("--dao", default="", help="banh lap nguoc: TRAI / PHAI / '' (khong dao)")
    ap.add_argument("--vmax", type=float, default=4.0, help="vong/s toi da moi banh (<= 5.5)")
    ap.add_argument("--rong", type=float, default=0.382, help="khoang cach 2 banh [m]")
    ap.add_argument("--duong-kinh", type=float, default=0.139, help="duong kinh banh [m]")
    ap.add_argument("--tre-ms", type=float, default=0.0, help="cho them giua luc du ACK va luc phat GO [ms]")
    ap.add_argument("--xoa-loi", action="store_true", help="gui Clear_Errors truoc khi kiem tra")
    DriveTest(ap.parse_args()).chay()


if __name__ == "__main__":
    main()
