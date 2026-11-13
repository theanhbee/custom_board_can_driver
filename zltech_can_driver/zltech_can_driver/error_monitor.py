"""error_monitor - NGHE BUS CAN, GIAI MA MOI BIT LOI cua firmware Can-sync thanh ten + y nghia + goi y xu ly.

    ros2 run zltech_can_driver error_monitor                         # Waveshare tu do, node 0 = TRAI, 2 = PHAI
    ros2 run zltech_can_driver error_monitor --clear                 # gui CLEAR_ERRORS luc bat dau
    ros2 run zltech_can_driver error_monitor --log loi.csv           # ghi CSV: time, node, frame, raw_hex, decoded
    ros2 run zltech_can_driver error_monitor --nodes 0,1,2,3 --names 0=TRAI,2=PHAI
    ros2 run zltech_can_driver error_monitor --interface socketcan --channel can0
    ros2 run zltech_can_driver error_monitor --thu-dong              # chi nghe, KHONG gui RTR / clear
    ros2 run zltech_can_driver error_monitor --replay candump.log    # doc log candump, khong can phan cung

Phim (khi chay that): c = CLEAR_ERRORS cac node dang theo doi, r = hoi lai frame loi (RTR), q / Ctrl+C = thoat.
In log CHI KHI THAY DOI (state, bit loi them / het, CLEARED, mat / co lai heartbeat), khong spam frame lap 100 ms.
Dong cuoi terminal = bang tong hop lam moi lien tuc.
Tu hoi (RTR) frame chi tiet khi heartbeat bao *_FAILED ma 200 ms chua thay frame do (sensorless 0x05 luon phai hoi);
motor SYSTEM_LEVEL / BRAKE_RESISTOR_DISARMED -> hoi Vbus (0x17) va in bang loi he thong (khong doc duoc qua CAN).
Waveshare la cong doc quyen: phai TAT can_bridge / drive_test truoc. Gui RTR / clear cung "cho an" watchdog firmware.
Giai ma theo can_heartbeat_error_spec.md (thu vien odrive_can_errors.py).
"""
import argparse
import csv
import os
import re
import select
import struct
import sys
import termios
import time
import tty

from zltech_can_driver import odrive_can_errors as E

HB_MAT = 0.3                       # s khong co heartbeat -> canh bao mat heartbeat
CHO_CHI_TIET = 0.2                 # s sau *_FAILED ma chua co frame chi tiet -> hoi RTR
HOI_LAI_SAU = 1.0                  # s, khong hoi lai cung 1 frame day hon
BANG_CHU_KY = 0.5                  # s lam moi dong tong hop

_CANDUMP = re.compile(r"^\s*\((\d+(?:\.\d+)?)\)\s+\S+\s+([0-9A-Fa-f]{3}|[0-9A-Fa-f]{8})#(R\d?|[0-9A-Fa-f]*)\s*$")


def parse_candump_line(line):
    """'(1690000000.123) can0 041#0002000100000000' -> (t, arb, data, is_extended, is_remote); dong khac -> None."""
    m = _CANDUMP.match(line)
    if not m:
        return None
    t, ident, payload = float(m.group(1)), m.group(2), m.group(3)
    rtr = payload.upper().startswith("R")
    return t, int(ident, 16), b"" if rtr else bytes.fromhex(payload), len(ident) == 8, rtr


def gio(t):
    return time.strftime("%H:%M:%S", time.localtime(t)) + f".{int(t * 1000) % 1000:03d}"


class NodeView:
    def __init__(self):
        self.t_hb = None               # heartbeat cuoi
        self.hb_mat = False
        self.state = None
        self.axis_error = 0
        self.err = {k: 0 for k in ("motor", "encoder", "controller", "sensorless")}
        self.t_chi_tiet = {}           # kind -> lan cuoi nhan frame chi tiet
        self.t_failed = {}             # kind -> luc heartbeat bat dau bao *_FAILED
        self.t_hoi = {}                # cmd -> lan cuoi gui RTR
        self.vbus = None
        self.canh_bao_du_phong = False


class Monitor:
    def __init__(self, nodes, names, gui=None, csv_file=None, live_status=True):
        """nodes: set node_id theo doi (None = tat ca). gui(arb, rtr) hoac None (replay / thu dong)."""
        self.nodes = nodes
        self.names = names
        self.gui = gui
        self.csv = csv.writer(csv_file) if csv_file else None
        if self.csv:
            self.csv.writerow(["time", "node", "frame", "raw_hex", "decoded"])
        self.live_status = live_status and sys.stdout.isatty()
        self.v = {}
        self.t0 = None
        self.t_bang = 0.0
        self.so_su_kien = 0

    # ---------------------------------------------------------------- in
    def ten(self, n):
        if n < 0:
            return ""
        return f"{self.names.get(n, 'NODE')}(node {n})"

    def in_dong(self, t, n, msg):
        self.so_su_kien += 1
        if self.live_status:
            sys.stdout.write("\r\033[K")
        print(f"{gio(t)}  {self.ten(n):<14} {msg}", flush=True)

    def in_bit(self, t, n, nhom, e, them):
        if them:
            goi_y = f"\n{'':29}-> {e['hint']}" if e["hint"] else ""
            self.in_dong(t, n, f"{nhom}: + {e['name']} (0x{e['mask']:X}) {e['desc']}{goi_y}")
        else:
            self.in_dong(t, n, f"{nhom}: - {e['name']} het")

    def in_diff(self, t, n, nhom, cu, moi, table):
        for e in E.decode_bits(moi & ~cu, table):
            self.in_bit(t, n, nhom, e, True)
        for e in E.decode_bits(cu & ~moi, table):
            self.in_bit(t, n, nhom, e, False)

    def bang(self, t, ep=False):
        if not self.live_status or (not ep and t - self.t_bang < BANG_CHU_KY):
            return
        self.t_bang = t
        phan = []
        for n in sorted(self.v):
            s = self.v[n]
            if s.t_hb is None:
                tt = "chua co HB"
            elif s.hb_mat:
                tt = f"MAT HB {t - s.t_hb:.1f}s"
            else:
                loi = [k for k, val in s.err.items() if val] + (["axis"] if s.axis_error else [])
                tt = f"{E.state_name(s.state)} {'LOI ' + ','.join(loi) if loi else 'ok'}"
            phan.append(f"{self.names.get(n, n)}: {tt}")
        sys.stdout.write("\r\033[K  [" + " | ".join(phan) + "]  c=clear r=hoi q=thoat")
        sys.stdout.flush()

    # ---------------------------------------------------------------- xu ly frame
    def xu_ly(self, t, arb, data, is_ext=False, is_rtr=False):
        if self.t0 is None:
            self.t0 = t
        n, cmd = arb >> 5, arb & 0x1F
        if is_ext or is_rtr or (self.nodes is not None and n not in self.nodes):
            return
        if cmd == E.CMD_BUS_VOLTAGE_CURRENT and len(data) >= 8:
            vbus, ibus = struct.unpack("<ff", bytes(data[:8]))
            s = self.v.setdefault(n, NodeView())
            if s.vbus is None or abs(s.vbus - vbus) > 0.5:
                self.in_dong(t, n, f"Vbus {vbus:.2f} V, Ibus {ibus:.2f} A (frame 0x17)")
            s.vbus = vbus
            return
        d = E.decode_frame(arb, data)
        if d is None:
            return
        s = self.v.setdefault(n, NodeView())
        if self.csv:
            self.csv.writerow([f"{t:.6f}", n, f"0x{arb:03X} {d['kind']}", bytes(data).hex(), self.tom_tat(d)])
        if d["kind"] == "heartbeat":
            self._heartbeat(t, n, s, d)
        else:
            self._chi_tiet(t, n, s, d)

    @staticmethod
    def tom_tat(d):
        if d["kind"] == "heartbeat":
            return f"{d['state_name']} axis_error=0x{d['axis_error']:X} " + "|".join(e["name"] for e in d["errors"])
        return f"{d['kind']}_error=0x{d['value']:X} " + ("CLEARED" if d["cleared"] else
                                                         "|".join(e["name"] for e in d["errors"]))

    def _heartbeat(self, t, n, s, d):
        if s.t_hb is None:
            self.in_dong(t, n, f"heartbeat dau tien: {d['state_name']}"
                               f"{', axis_error 0x%X' % d['axis_error'] if d['axis_error'] else ', khong loi'}")
        elif s.hb_mat:
            self.in_dong(t, n, f"CO LAI heartbeat sau {t - s.t_hb:.2f} s")
        s.t_hb, s.hb_mat = t, False
        if d["reserved_nonzero"] and not s.canh_bao_du_phong:
            s.canh_bao_du_phong = True
            self.in_dong(t, n, f"!! heartbeat byte 4-7 = 0x{d['reserved']:08X} khac 0 -> firmware la (khong phai "
                               f"Can-sync err-event?), giai ma co the sai")
        if s.state is not None and d["state"] != s.state:
            self.in_dong(t, n, f"STATE {E.state_name(s.state)} -> {d['state_name']}")
        s.state = d["state"]
        cu, moi = s.axis_error, d["axis_error"]
        if cu != moi:
            self.in_diff(t, n, "axis", cu, moi, E.AXIS_ERRORS)
        s.axis_error = moi
        for bit, (kind, _) in E.FAILED_TO_DETAIL.items():
            if moi & bit:
                s.t_failed.setdefault(kind, t)
            else:
                s.t_failed.pop(kind, None)

    def _chi_tiet(self, t, n, s, d):
        kind, cu, moi = d["kind"], s.err[d["kind"]], d["value"]
        s.t_chi_tiet[kind] = t
        if moi == cu:
            return
        if d["cleared"]:
            self.in_dong(t, n, f"{kind}: CLEARED (truoc: {E.bit_names(cu, E.table_for(kind))})")
        else:
            self.in_diff(t, n, kind, cu, moi, E.table_for(kind))
            if kind == "motor" and (moi & ~cu) & E.MOTOR_SYSTEM_HINT_MASK:
                self.in_dong(t, n, "motor: nguyen nhan goc thuong la LOI HE THONG ODrive (khong co tren CAN): " +
                             ", ".join(f"{v[0]} ({v[1]})" for v in E.SYSTEM_ERRORS.values()))
                self.hoi(t, n, s, E.CMD_BUS_VOLTAGE_CURRENT, "Vbus")
        s.err[kind] = moi

    # ---------------------------------------------------------------- dinh ky
    def hoi(self, t, n, s, cmd, ly_do):
        if t - s.t_hoi.get(cmd, -1e9) < HOI_LAI_SAU:
            return
        s.t_hoi[cmd] = t
        if self.gui is None:
            self.in_dong(t, n, f"(se hoi RTR 0x{E.arb_id(n, cmd):03X} {ly_do}; replay / thu dong -> khong gui)")
            return
        try:
            self.gui(E.arb_id(n, cmd), True)
        except Exception as e:
            self.in_dong(t, n, f"!! gui RTR 0x{E.arb_id(n, cmd):03X} loi: {e}")

    def tick(self, t):
        for n, s in self.v.items():
            if s.t_hb is not None and not s.hb_mat and t - s.t_hb > HB_MAT:
                s.hb_mat = True
                self.in_dong(t, n, f"!! MAT HEARTBEAT > {HB_MAT * 1000:.0f} ms (node offline / mat nguon / mat CAN / "
                                   f"adapter dung hinh)")
            for kind, t_fail in s.t_failed.items():
                cmd = next(c for b, (k, c) in E.FAILED_TO_DETAIL.items() if k == kind)
                chua_co = s.t_chi_tiet.get(kind, -1e9) < t_fail
                if t - t_fail > CHO_CHI_TIET and (chua_co or kind == "sensorless"):
                    self.hoi(t, n, s, cmd, f"{kind}: heartbeat bao *_FAILED ma chua co frame chi tiet")
        if self.nodes is not None and self.t0 is not None and t - self.t0 > 1.0:
            for n in self.nodes:
                if n not in self.v:
                    self.v[n] = NodeView()
                    self.in_dong(t, n, "!! chua nhan heartbeat nao sau 1 s (sai node / mat nguon / day CAN)")
        self.bang(t)

    def hoi_tat_ca(self, t):
        for n, s in self.v.items():
            for cmd in (E.CMD_MOTOR_ERROR, E.CMD_ENCODER_ERROR, E.CMD_CONTROLLER_ERROR, E.CMD_SENSORLESS_ERROR):
                s.t_hoi.pop(cmd, None)
                self.hoi(t, n, s, cmd, "theo yeu cau (phim r)")

    def tong_ket(self):
        if self.live_status:
            sys.stdout.write("\r\033[K")
        print(f"\n===== TONG KET ({self.so_su_kien} su kien)")
        for n in sorted(self.v):
            s = self.v[n]
            print(f"  {self.ten(n)}: state {E.state_name(s.state) if s.state is not None else '-'}, "
                  f"axis 0x{s.axis_error:X} {E.bit_names(s.axis_error, E.AXIS_ERRORS)}")
            for kind, val in s.err.items():
                if val:
                    print(f"      {kind}: 0x{val:X} {E.bit_names(val, E.table_for(kind))}")


# ============================================================================================================
def parse_names(text):
    out = {}
    for part in filter(None, text.split(",")):
        k, _, v = part.partition("=")
        out[int(k)] = v
    return out


def chay_replay(args, mon):
    t = None
    with open(args.replay) as f:
        for line in f:
            r = parse_candump_line(line)
            if r is None:
                continue
            t, arb, data, ext, rtr = r
            mon.tick(t)
            mon.xu_ly(t, arb, data, ext, rtr)
    if t is not None:
        mon.tick(t)


def chay_that(args, mon_tao):
    import can
    from zltech_can_driver import protocol as p

    bus = p.mo_bus(args.interface, args.channel, args.baud_serial, args.bitrate)

    def gui(arb, rtr=False):
        bus.send(can.Message(arbitration_id=arb, is_remote_frame=rtr, dlc=0, data=b"", is_extended_id=False))

    mon = mon_tao(None if args.thu_dong else gui)
    print(f"Nghe {getattr(bus, 'channel_info', args.interface)}, node {args.nodes} ... (c=clear, r=hoi, q=thoat)")
    if args.clear and not args.thu_dong:
        for n in mon.nodes or []:
            gui(E.arb_id(n, E.CMD_CLEAR_ERRORS))
        print(f"Da gui CLEAR_ERRORS cho node {sorted(mon.nodes or [])}")
    fd = sys.stdin.fileno()
    cu = termios.tcgetattr(fd) if os.isatty(fd) else None
    if cu:
        tty.setcbreak(fd)
    try:
        while True:
            try:
                m = bus.recv(timeout=0.05)
            except Exception:
                m = None               # frame serial hong -> bo qua, doc tiep
            now = time.time()
            if m is not None:
                mon.xu_ly(now, m.arbitration_id, bytes(m.data), m.is_extended_id, m.is_remote_frame)
            mon.tick(now)
            if cu and select.select([sys.stdin], [], [], 0)[0]:
                c = os.read(fd, 1).decode(errors="ignore").lower()
                if c == "q":
                    break
                if c == "c":
                    if args.thu_dong:
                        mon.in_dong(now, -1, "(--thu-dong: khong gui CLEAR_ERRORS)")
                    else:
                        for n in sorted(mon.v):
                            gui(E.arb_id(n, E.CMD_CLEAR_ERRORS))
                        mon.in_dong(now, -1, f"da gui CLEAR_ERRORS cho node {sorted(mon.v)}")
                elif c == "r":
                    mon.hoi_tat_ca(now)
    except KeyboardInterrupt:
        pass
    finally:
        if cu:
            termios.tcsetattr(fd, termios.TCSADRAIN, cu)
        bus.shutdown()
    return mon


def main():
    ap = argparse.ArgumentParser(description="Giai ma loi CAN firmware Can-sync (ODrive v3.6)")
    ap.add_argument("--interface", default="waveshare", help="waveshare | slcan | gs_usb | socketcan")
    ap.add_argument("--channel", default="", help="cong / interface ('' = tu do Waveshare)")
    ap.add_argument("--bitrate", type=int, default=250000)
    ap.add_argument("--baud-serial", type=int, default=2000000, help="baud serial Waveshare")
    ap.add_argument("--nodes", default="0,2", help="node theo doi, vd 0,2 hoac all")
    ap.add_argument("--names", default="0=TRAI,2=PHAI")
    ap.add_argument("--log", default="", help="file CSV")
    ap.add_argument("--clear", action="store_true", help="gui CLEAR_ERRORS luc bat dau")
    ap.add_argument("--thu-dong", action="store_true", help="chi nghe, khong gui RTR / clear")
    ap.add_argument("--replay", default="", help="file log candump")
    args = ap.parse_args()
    nodes = None if args.nodes == "all" else {int(x) for x in args.nodes.split(",") if x}
    names = parse_names(args.names)

    csv_file = open(args.log, "w", newline="") if args.log else None
    try:
        if args.replay:
            mon = Monitor(nodes, names, None, csv_file, live_status=False)
            chay_replay(args, mon)
        else:
            mon = chay_that(args, lambda gui: Monitor(nodes, names, gui, csv_file))
        mon.tong_ket()
    finally:
        if csv_file:
            csv_file.close()
            print(f"CSV: {args.log}")


if __name__ == "__main__":
    main()
