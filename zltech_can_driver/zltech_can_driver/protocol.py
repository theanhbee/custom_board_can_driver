"""protocol.py - giao thuc CAN firmware Can-sync (ODrive v3.6) + adapter Waveshare USB-CAN-A + lop CanLink dung chung.

Dung chung cho can_bridge / calib / clear_errors / drive_test. Khong doc sys.argv, khong tao ROS node.

Arbitration id 11 bit = (node_id << 5) | cmd. Bus 250 kbit/s.
Board tu gui: heartbeat 0x01 (100 ms: byte 0-2 axis_error, byte 3 state), encoder 0x09 (10 ms: float pos, float vel
vong/s), frame loi chi tiet 0x03 motor / 0x04 encoder / 0x1D controller khi co loi.
Dong bo 2 banh: 0x1E Set_Input_Pos_Staged (float pos = 0, float vel) nap vao dem -> board ACK (cmd 0x1E, dlc 1,
data[0] = 2) -> 0x7FF SYNC_GO data[0] = 0 -> moi board ap cung luc.
Firmware can-watchdog: vao CLOSED_LOOP qua CAN thi 0.5 s khong co lenh CAN -> board tu cat (axis_error 0x800).
"""
import fcntl
import os
import signal
import struct
import threading
import time

import can
from can.interfaces.seeedstudio import SeeedBus

from zltech_can_driver import odrive_can_errors as E

BITRATE = 250000

# ---- cmd id (5 bit thap) ----
CMD_HEARTBEAT = 0x01
CMD_SET_STATE = 0x07
CMD_ENCODER_EST = 0x09
CMD_SET_MODES = 0x0B
CMD_SET_INPUT_VEL = 0x0D
CMD_CLEAR_ERRORS = 0x18
CMD_STAGED = 0x1E
ID_SYNC_GO = 0x7FF                 # (node 0x3F << 5) | 0x1F, broadcast
ACK_STAGED = 2                     # data[0] ACK cua ban Can-sync

# ---- state / mode ----
IDLE, FULL_CALIB, MOTOR_CALIB, ENCODER_OFFSET, CLOSED_LOOP = 1, 3, 4, 7, 8
STATE_NAMES = {1: "IDLE", 3: "FULL_CALIB", 4: "MOTOR_CALIB", 7: "ENCODER_OFFSET", 8: "CLOSED_LOOP",
               12: "HALL_POLARITY", 13: "HALL_PHASE"}
VELOCITY_CONTROL, PASSTHROUGH, VEL_RAMP = 2, 1, 2
ERR_WATCHDOG = 0x800               # axis_error WATCHDOG_TIMER_EXPIRED

# ============================================================================================================
# Ma loi
# ============================================================================================================
# Bang ma loi day du (ten, y nghia, goi y) nam o odrive_can_errors.py; o day chi lay ten.
AXIS_ERRORS = {m: v[0] for m, v in E.AXIS_ERRORS.items()}
MOTOR_ERRORS = {m: v[0] for m, v in E.MOTOR_ERRORS.items()}
ENCODER_ERRORS = {m: v[0] for m, v in E.ENCODER_ERRORS.items()}
CONTROLLER_ERRORS = {m: v[0] for m, v in E.CONTROLLER_ERRORS.items()}
# cmd -> (ten bo phan, bang ma loi, so byte)
DETAIL_FRAMES = {
    0x03: ("motor", MOTOR_ERRORS, 8),
    0x04: ("encoder", ENCODER_ERRORS, 4),
    0x1D: ("controller", CONTROLLER_ERRORS, 4),
}


def decode_heartbeat(data):
    """8 byte heartbeat -> (axis_error, state)."""
    d = bytes(data)
    return int.from_bytes(d[0:3], "little"), d[3]


def decode_detail(cmd, data):
    """Frame loi chi tiet -> (ten bo phan, gia tri loi). cmd khong phai frame loi -> None."""
    if cmd not in DETAIL_FRAMES:
        return None
    name, _, nbytes = DETAIL_FRAMES[cmd]
    return name, int.from_bytes(bytes(data)[:nbytes], "little")


def ten_loi(value, table):
    """0x240 -> 'MOTOR_FAILED|CONTROLLER_FAILED'. Bit khong co trong bang thi in dang hex."""
    names = [n for bit, n in table.items() if value & bit]
    unknown = value & ~sum(table)
    if unknown:
        names.append(f"0x{unknown:X}")
    return "|".join(names)


def mo_ta_loi(axis_error, details):
    """details: {'motor': gia tri, 'encoder': ..., 'controller': ...}. Tra ve chuoi ngan, '' neu khong loi."""
    if not axis_error and not any(details.values()):
        return ""
    parts = [ten_loi(axis_error, AXIS_ERRORS)] if axis_error else []
    for name, table, _ in DETAIL_FRAMES.values():
        if details.get(name):
            parts.append(f"{name}: {ten_loi(details[name], table)}")
    return "; ".join(parts)


# ============================================================================================================
# Waveshare USB-CAN-A
# ============================================================================================================
# Driver goc python-can "seeedstudio" dat byte "Send once" = 0x01 -> adapter KHONG tu gui lai frame thua arbitration
# -> mat 15-50 % lenh. Ban nay bat tu gui lai va giu cach 2 ms giua 2 frame gui.
GIAN_CACH_GUI = 0.002  # [s]


def _dang_bi_giu(dev):
    """True neu cong dang bi WaveshareBus khac giu (flock). KHONG doc cong do (doc trom -> chuong trinh kia mat ACK)."""
    try:
        fd = os.open(dev, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return False
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        fcntl.flock(fd, fcntl.LOCK_UN)
        return False
    except BlockingIOError:
        return True
    finally:
        os.close(fd)


def _ai_giu(dev):
    """'can_bridge (pid 123)' cho cong dev, doc tu /proc; '' neu khong tim duoc."""
    that = os.path.realpath(dev)
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            for fd in os.listdir(f"/proc/{pid}/fd"):
                if os.path.realpath(f"/proc/{pid}/fd/{fd}") == that:
                    ten = open(f"/proc/{pid}/cmdline").read().split("\0")
                    ten = next((os.path.basename(x) for x in ten if "zltech_can_driver" in x or x.endswith(".py")),
                               os.path.basename(ten[0]))
                    return f"{ten} (pid {pid})"
        except OSError:
            continue
    return ""


def _dem_frame_waveshare(dev, baud, thoi_gian=0.3):
    """CHI DOC (khong ghi gi) dev trong thoi_gian s, dem goi Waveshare hop le: AA, info (0xC0|dlc), id 2 byte, data, 55."""
    import serial
    try:
        with serial.Serial(dev, baud, timeout=0.05) as s:
            s.reset_input_buffer()
            time.sleep(thoi_gian)
            d = s.read(s.in_waiting or 1)
    except (serial.SerialException, OSError):
        return 0
    dem = 0
    for i in range(len(d) - 4):
        if d[i] == 0xAA and d[i + 1] & 0xF0 == 0xC0:
            j = i + 4 + (d[i + 1] & 0x0F)
            dem += j < len(d) and d[j] == 0x55
    return dem


def find_adapter_port(baud=2000000):
    """Tim cong Waveshare USB-CAN-A (chip CH340, VID:PID 1A86:7523).
    May co NHIEU CH340 (vd IMU HWT901 cung CH340, 3/11) -> chi doc thu dong tung cong, chon cong co frame Waveshare.
    KHONG ghi gi vao cong khi do (ghi nham vao IMU / mach khac la nguy hiem)."""
    from serial.tools import list_ports
    ch340 = sorted(p.device for p in list_ports.comports() if p.vid == 0x1A86 and p.pid == 0x7523)
    if len(ch340) == 1:
        return ch340[0]
    if not ch340:
        ports = ", ".join(f"{p.device} ({p.description})" for p in list_ports.comports() if p.vid)
        raise SystemExit(f"Khong thay adapter CH340. Cac cong USB hien co: {ports or '(khong co)'}")
    bi_giu = [dev for dev in ch340 if _dang_bi_giu(dev)]
    if bi_giu:
        raise SystemExit("!! " + ", ".join(f"{d} dang bi {_ai_giu(d) or 'chuong trinh khac'} giu" for d in bi_giu) +
                         " -> Waveshare dang duoc dung. Tat no truoc (Ctrl+C o terminal bridge / navigation).")
    dem = {dev: _dem_frame_waveshare(dev, baud) for dev in ch340}
    co_frame = [dev for dev in ch340 if dem[dev] >= 3]
    if len(co_frame) == 1:
        return co_frame[0]
    goi_y = (" Ca 2 cong deu im lang -> cac board CAN dang TAT NGUON / tuot day CAN?" if not any(dem.values()) else "")
    raise SystemExit(f"Co {len(ch340)} cong CH340 ({', '.join(f'{d}: {dem[d]} frame CAN' for d in ch340)}) -> "
                     f"khong xac dinh duoc Waveshare.{goi_y} Hoac chi dinh cong: port (yaml / launch) / --cong, "
                     f"nen dung /dev/serial/by-path/... (co dinh theo lo USB)")


class WaveshareBus(SeeedBus):
    def __init__(self, channel, bitrate=BITRATE, baudrate=2000000, loc_id=0, loc_mask=0, tu_gui_lai=True,
                 gian_cach=GIAN_CACH_GUI, **kw):
        self._loc_id, self._loc_mask, self._tu_gui_lai = loc_id, loc_mask, tu_gui_lai
        self._gian_cach, self._t_gui, self._khoa_gui = gian_cach, 0.0, threading.Lock()
        # Khoa doc quyen cong: Linux cho 2 chuong trinh mo chung 1 tty, moi ben chi doc duoc 1 phan frame -> mat ACK.
        self._fd_khoa = os.open(channel, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(self._fd_khoa, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(self._fd_khoa)
            raise SystemExit(f"!! {channel} dang bi chuong trinh khac giu (can_bridge / drive_test / calib?)."
                             f" Tat no truoc. Xem: fuser -v {channel}")
        kw.setdefault("frame_type", "STD")
        kw.setdefault("operation_mode", "normal")
        super().__init__(channel=channel, baudrate=baudrate, bitrate=bitrate, **kw)

    def send(self, msg, timeout=None):
        with self._khoa_gui:
            cho = self._t_gui + self._gian_cach - time.perf_counter()
            if cho > 0:
                time.sleep(cho)
            super().send(msg, timeout)
            self._t_gui = time.perf_counter()

    def shutdown(self):
        super().shutdown()
        os.close(self._fd_khoa)

    def init_frame(self, timeout=None):
        # Giong SeeedBus.init_frame, khac: id/mask loc little-endian va byte tu gui lai = 0x00.
        self.filter_id = bytearray(self._loc_id.to_bytes(4, "little"))
        self.mask_id = bytearray(self._loc_mask.to_bytes(4, "little"))
        m = bytearray([0xAA, 0x55, 0x12, SeeedBus.BITRATE[self.bit_rate], SeeedBus.FRAMETYPE[self.frame_type]])
        m += self.filter_id + self.mask_id
        m += bytes([SeeedBus.OPERATIONMODE[self.op_mode], 0x00 if self._tu_gui_lai else 0x01, 0, 0, 0, 0])
        m.append(sum(m[2:]) & 0xFF)
        try:
            self.ser.write(m)
        except Exception as error:
            raise can.CanInitializationError("could not send init frame") from error


def bo_qua_tin_hieu():
    """Goi dau khoi don dep (finally): bo qua SIGINT/SIGTERM/SIGHUP de chuoi 've 0 -> IDLE -> dong CAN' chay tron.
    Khong co: Ctrl+C lan 2 / KeyboardInterrupt den tre (rclpy chuyen tiep SIGINT cho Python, ros2 launch) ngat giua
    chung -> banh khong ve IDLE (do 3/11 voi ros2 launch)."""
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, signal.SIG_IGN)


def mo_bus(adapter="waveshare", port="", baud_serial=2000000, bitrate=BITRATE):
    """adapter: waveshare | socketcan | gs_usb | slcan. port "" = mac dinh (Waveshare tu do CH340).
    socketcan: bitrate dat bang ip link (can.bash), tham so bitrate bi bo qua."""
    if adapter == "waveshare":
        return WaveshareBus(channel=port or find_adapter_port(baud_serial), bitrate=bitrate, baudrate=baud_serial)
    if adapter == "socketcan":
        return can.Bus(interface="socketcan", channel=port or "can0")
    if adapter == "gs_usb":
        return can.Bus(interface="gs_usb", channel=0, index=0, bitrate=bitrate)
    if adapter == "slcan":
        return can.Bus(interface="slcan", channel=port or "/dev/ttyACM0", bitrate=bitrate)
    raise SystemExit(f"adapter '{adapter}' khong ho tro (waveshare | socketcan | gs_usb | slcan)")


# ============================================================================================================
# CanLink: bus + luong nhan + trang thai tung node + lenh
# ============================================================================================================
class NodeState:
    """Du lieu moi nhat cua 1 node, cap nhat tu luong nhan."""

    def __init__(self):
        self.t_hb = 0.0                # heartbeat cuoi (monotonic)
        self.t_rx = 0.0                # frame bat ky cuoi (encoder 0x09 100 lan/s)
        self.state = None
        self.axis_error = 0
        self.pos = 0.0                 # vong, tu 0x09
        self.vel = 0.0                 # vong/s, tu 0x09
        self.details = {name: 0 for name, _, _ in DETAIL_FRAMES.values()}
        self.ack = threading.Event()
        # Chan doan (13/11: heartbeat 1 node vang ~1 s trong khi encoder van ve). Doc xong tu dat lai = 0.
        self.so_hb = 0                 # so heartbeat nhan
        self.so_enc = 0                # so frame encoder 0x09 nhan (board gui 100/s)
        self.hb_gap_max = 0.0          # khoang trong heartbeat lon nhat [s] (board gui moi 0.1 s)

    def loi(self):
        return mo_ta_loi(self.axis_error, self.details)


class CanLink:
    def __init__(self, nodes, adapter="waveshare", port="", baud_serial=2000000):
        """nodes: cac node_id can theo doi (vd (0, 2))."""
        self.nodes = tuple(nodes)
        self.tt = {n: NodeState() for n in self.nodes}
        self.bus = mo_bus(adapter, port, baud_serial)
        self.so_loi_nhan, self.loi_nhan_cuoi = 0, ""
        self._dung = threading.Event()
        self._nhan = None
        self.kiem_luong_nhan()

    # ---------------------------------------------------------------- nhan
    def _luong_nhan(self):
        while not self._dung.is_set():
            try:
                m = self.bus.recv(timeout=0.1)
            except Exception as e:
                # SeeedBus doc thieu byte (nhieu / lech khung) -> struct.error. Khong bat thi luong nhan chet im lang
                # -> khong ai doc serial -> adapter nghen.
                self.so_loi_nhan += 1
                self.loi_nhan_cuoi = f"{type(e).__name__}: {e}"
                continue
            if m is None or m.is_extended_id or m.is_remote_frame:
                continue
            node, cmd = m.arbitration_id >> 5, m.arbitration_id & 0x1F
            s = self.tt.get(node)
            if s is None:
                continue
            now = time.monotonic()
            s.t_rx = now
            if cmd == CMD_HEARTBEAT and m.dlc >= 4:
                s.axis_error, s.state = decode_heartbeat(m.data)
                if s.t_hb:
                    s.hb_gap_max = max(s.hb_gap_max, now - s.t_hb)
                s.t_hb = now
                s.so_hb += 1
            elif cmd == CMD_ENCODER_EST and m.dlc >= 8:
                s.pos, s.vel = struct.unpack("<ff", bytes(m.data[:8]))
                s.so_enc += 1
            elif cmd == CMD_STAGED and m.dlc == 1 and m.data[0] == ACK_STAGED:
                s.ack.set()
            else:
                d = decode_detail(cmd, m.data)
                if d:
                    s.details[d[0]] = d[1]

    def kiem_luong_nhan(self):
        """Khoi dong (lai) luong nhan neu chua chay / da chet. Tra ve True neu phai khoi dong lai."""
        if self._nhan is not None and self._nhan.is_alive():
            return False
        chet = self._nhan is not None
        self._nhan = threading.Thread(target=self._luong_nhan, daemon=True)
        self._nhan.start()
        return chet

    def cho_heartbeat(self, timeout=1.5, nodes=None):
        """Cho toi khi moi node co heartbeat. Tra ve True neu du."""
        nodes = nodes or self.nodes
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if all(self.tt[n].t_hb for n in nodes):
                return True
            time.sleep(0.05)
        return all(self.tt[n].t_hb for n in nodes)

    def con_song(self, n, rx_timeout=0.3, hb_timeout=1.0):
        now = time.monotonic()
        return now - self.tt[n].t_rx < rx_timeout and now - self.tt[n].t_hb < hb_timeout

    # ---------------------------------------------------------------- gui
    def gui(self, arb_id, data=b""):
        self.bus.send(can.Message(arbitration_id=arb_id, data=data, is_extended_id=False))

    def gui_node(self, n, cmd, data=b""):
        self.gui((n << 5) | cmd, data)

    def dat_state(self, state, nodes=None):
        for n in nodes or self.nodes:
            self.gui_node(n, CMD_SET_STATE, struct.pack("<i", state))

    def dat_input_mode(self, input_mode, nodes=None):
        for n in nodes or self.nodes:
            self.gui_node(n, CMD_SET_MODES, struct.pack("<ii", VELOCITY_CONTROL, input_mode))

    def van_toc_0(self, nodes=None):
        """0x0D Set_Input_Vel = 0 truc tiep (khong qua dem staged)."""
        for n in nodes or self.nodes:
            self.gui_node(n, CMD_SET_INPUT_VEL, struct.pack("<ff", 0.0, 0.0))

    def xoa_loi(self, nodes=None):
        """Gui Clear_Errors, bo heartbeat / frame loi cu (con loi) de cho ban moi. Khong sleep."""
        for n in nodes or self.nodes:
            self.gui_node(n, CMD_CLEAR_ERRORS)
            self.tt[n].details = {k: 0 for k in self.tt[n].details}
            self.tt[n].t_hb = 0.0

    def gui_dong_bo(self, vong, ack_timeout=0.02, tre_go=0.0):
        """vong: {node: vong/s}. Nap vao dem CA cac board, cho ACK chung (thieu thi gui lai 1 lan), cho them tre_go s,
        roi LUON phat SYNC_GO. Tra ve danh sach node thieu ACK."""
        cho = list(vong)
        for _ in range(2):
            for n in cho:
                self.tt[n].ack.clear()
                self.gui_node(n, CMD_STAGED, struct.pack("<ff", 0.0, vong[n]))
            han = time.monotonic() + ack_timeout
            for n in cho:
                self.tt[n].ack.wait(max(0.0, han - time.monotonic()))
            cho = [n for n in cho if not self.tt[n].ack.is_set()]
            if not cho:
                break
        if tre_go:
            time.sleep(tre_go)
        self.gui(ID_SYNC_GO, b"\x00")
        return cho

    def dua_ve_idle(self):
        """Toc do 0 (dong bo) -> VEL_RAMP -> IDLE x2. Dung khi thoat / tat."""
        self.gui_dong_bo({n: 0.0 for n in self.nodes})
        self.dat_input_mode(VEL_RAMP)
        self.dat_state(IDLE)
        time.sleep(0.05)
        self.dat_state(IDLE)

    def dong(self):
        self._dung.set()
        if self._nhan is not None:
            self._nhan.join(timeout=0.5)
        self.bus.shutdown()


# ============================================================================================================
# Tham so dong lenh dung chung cho cac tool (calib / clear_errors / drive_test)
# ============================================================================================================
def them_tham_so_can(ap):
    """Them --trai --phai --adapter --cong --baud-serial vao argparse.ArgumentParser."""
    ap.add_argument("--trai", type=int, default=0, help="node_id banh trai (mac dinh 0)")
    ap.add_argument("--phai", type=int, default=2, help="node_id banh phai (mac dinh 2)")
    ap.add_argument("--adapter", default="waveshare", help="waveshare | socketcan | gs_usb | slcan")
    ap.add_argument("--cong", default="", help="cong / interface; '' = tu do (Waveshare CH340)")
    ap.add_argument("--baud-serial", type=int, default=2000000, help="baud serial Waveshare")
    return ap


def ten_banh(args):
    """{node_id: 'TRAI'/'PHAI'} tu args; thoat neu 2 node trung."""
    if args.trai == args.phai:
        raise SystemExit("!! --trai va --phai phai khac nhau")
    return {args.trai: "TRAI", args.phai: "PHAI"}


def chon_banh(args, ten):
    """--chi=TRAI/PHAI -> danh sach node; khong co --chi -> ca 2."""
    chi = (getattr(args, "chi", "") or "").upper()
    nodes = [n for n in ten if not chi or ten[n] == chi]
    if not nodes:
        raise SystemExit("--chi phai la TRAI hoac PHAI")
    return nodes
