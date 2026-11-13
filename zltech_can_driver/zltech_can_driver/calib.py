"""calib - CALIB LAI 2 BANH QUA CAN (Set_Axis_State), lan luot tung banh, in tung buoc + loi.

    ros2 run zltech_can_driver calib                  # FULL_CALIBRATION (motor + encoder offset) ca 2 banh
    ros2 run zltech_can_driver calib --chi=PHAI       # chi 1 banh (TRAI / PHAI)
    ros2 run zltech_can_driver calib --kieu=motor     # full (mac dinh, state 3) | motor (state 4) | encoder (state 7)
    ros2 run zltech_can_driver calib --yes            # khong hoi xac nhan
    ros2 run zltech_can_driver calib --trai=0 --phai=2 --cong=/dev/ttyUSB0

CANH BAO: calib lam motor keu bip roi QUAY banh 1 doan -> KE BANH LEN khoi mat dat. Banh cham dat / bi tai
         -> hay loi CPR_POLEPAIRS_MISMATCH. Truoc khi calib tu xoa loi cu. Ctrl+C -> banh ve IDLE.
Phai TAT can_bridge / drive_test truoc (cong bi khoa).
Ket qua calib chi nam trong RAM board; mat nguon co the phai calib lai (tuy cau hinh firmware).
Thoat 0 = cac banh OK, 1 = co banh loi.
"""
import argparse
import sys
import time

from zltech_can_driver import protocol as p

KIEU = {"full": p.FULL_CALIB, "motor": p.MOTOR_CALIB, "encoder": p.ENCODER_OFFSET}
TOI_DA = 40.0                          # s cho 1 banh


def calib_1_banh(link, n, ten, state_calib):
    s = link.tt[n]
    print(f"\n== {ten} (node {n}): xoa loi cu...")
    link.xoa_loi([n])
    if not link.cho_heartbeat(1.5, [n]):
        print("   !! KHONG co heartbeat -> sai node / mat nguon / day CAN")
        return False
    time.sleep(0.2)                    # cho frame loi chi tiet (neu loi hien lai)
    if s.state != p.IDLE:
        link.dat_state(p.IDLE, [n])
        time.sleep(0.3)
    print(f"   truoc calib: state {p.STATE_NAMES.get(s.state, s.state)}, loi '{s.loi() or 'khong'}'")
    link.dat_state(state_calib, [n])
    t0 = time.monotonic()
    cu = None
    while time.monotonic() - t0 < TOI_DA:
        hien = (s.state, s.axis_error, tuple(s.details.values()))
        if hien != cu:
            print(f"   t={time.monotonic() - t0:5.2f}s  {p.STATE_NAMES.get(s.state, s.state)}  {s.loi()}", flush=True)
            cu = hien
        if time.monotonic() - t0 > 1.0 and s.state == p.IDLE:
            break
        time.sleep(0.05)
    else:
        print(f"   !! qua {TOI_DA:.0f} s chua xong -> ve IDLE")
        link.dat_state(p.IDLE, [n])
        return False
    ok = not s.loi()
    print(f"== {ten}: {'OK' if ok else 'LOI: ' + s.loi()}  ({time.monotonic() - t0:.1f} s)")
    if not ok and s.details.get("motor"):
        print("   -> loi motor (vd UNBALANCED_PHASES / PHASE_RESISTANCE): kiem tra 3 day pha, do dien tro A-B, B-C, C-A")
    elif not ok and s.details.get("encoder"):
        print("   -> loi encoder (vd CPR_POLEPAIRS_MISMATCH): ke banh len cho quay tu do, kiem tra giac encoder/hall")
    return ok


def main():
    ap = p.them_tham_so_can(argparse.ArgumentParser(description="Calib lai banh qua CAN"))
    ap.add_argument("--kieu", default="full", choices=list(KIEU), help="full | motor | encoder")
    ap.add_argument("--chi", default="", help="chi 1 banh: TRAI / PHAI")
    ap.add_argument("--yes", action="store_true", help="khong hoi xac nhan")
    args = ap.parse_args()
    ten = p.ten_banh(args)
    nodes = p.chon_banh(args, ten)

    print(f"CALIB {args.kieu.upper()} (state {KIEU[args.kieu]}): {', '.join(f'{ten[n]} = node {n}' for n in nodes)}")
    print("!! Motor se QUAY banh 1 doan -> banh phai duoc KE LEN khoi mat dat.")
    if not args.yes:
        try:
            input("Enter de bat dau, Ctrl+C de huy: ")
        except (KeyboardInterrupt, EOFError):
            sys.exit("\nHuy.")

    link = p.CanLink(nodes, args.adapter, args.cong, args.baud_serial)
    ket_qua = {}
    try:
        time.sleep(0.5)
        for n in nodes:
            ket_qua[n] = calib_1_banh(link, n, ten[n], KIEU[args.kieu])
    except KeyboardInterrupt:
        print("\n\nHuy (Ctrl+C)")
    finally:
        p.bo_qua_tin_hieu()
        link.dat_state(p.IDLE)
        time.sleep(0.05)
        link.dong()

    print("\nKET QUA: " + ", ".join(f"{ten[n]} {'OK' if ket_qua.get(n) else 'LOI'}" for n in nodes))
    sys.exit(0 if ket_qua and all(ket_qua.get(n) for n in nodes) else 1)


if __name__ == "__main__":
    main()
