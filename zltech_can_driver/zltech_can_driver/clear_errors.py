"""clear_errors - XOA LOI 2 BANH QUA CAN (Clear_Errors 0x18), in loi TRUOC va SAU khi xoa.

    ros2 run zltech_can_driver clear_errors              # ca 2 banh: TRAI (node 0), PHAI (node 2)
    ros2 run zltech_can_driver clear_errors --chi=TRAI   # chi 1 banh
    ros2 run zltech_can_driver clear_errors --xem        # chi doc loi, KHONG xoa
    ros2 run zltech_can_driver clear_errors --trai=0 --phai=2 --cong=/dev/ttyUSB0

Khong lam banh quay. Clear_Errors khong doi state (banh dang IDLE van IDLE).
Loi xoa xong lai hien ngay (motor / encoder hong that) -> in canh bao: sua phan cung hoac calib lai.
Phai TAT can_bridge / drive_test truoc (cong bi khoa).
Thoat 0 = cac banh sach loi, 1 = con loi / khong co heartbeat.
"""
import argparse
import sys
import time

from zltech_can_driver import protocol as p


def in_trang_thai(link, nodes, ten, tieu_de):
    print(tieu_de)
    ok = True
    for n in nodes:
        s = link.tt[n]
        if not s.t_hb or time.monotonic() - s.t_hb > 1.0:
            print(f"  {ten[n]} (node {n}): !! KHONG co heartbeat -> sai node / mat nguon / day CAN")
            ok = False
            continue
        loi = s.loi()
        print(f"  {ten[n]} (node {n}): state {p.STATE_NAMES.get(s.state, s.state)}, "
              f"axis_error 0x{s.axis_error:X}  {loi or 'sach loi'}")
        ok = ok and not loi
    return ok


def main():
    ap = p.them_tham_so_can(argparse.ArgumentParser(description="Xoa loi banh qua CAN"))
    ap.add_argument("--chi", default="", help="chi 1 banh: TRAI / PHAI")
    ap.add_argument("--xem", action="store_true", help="chi doc loi, khong xoa")
    args = ap.parse_args()
    ten = p.ten_banh(args)
    nodes = p.chon_banh(args, ten)

    link = p.CanLink(nodes, args.adapter, args.cong, args.baud_serial)
    try:
        time.sleep(1.0)                    # gom heartbeat (10 lan/s) + frame loi chi tiet
        sach = in_trang_thai(link, nodes, ten, "TRUOC:")
        if args.xem:
            return 0 if sach else 1
        print(f"Gui Clear_Errors cho {', '.join(ten[n] for n in nodes)}...")
        link.xoa_loi(nodes)
        time.sleep(0.8)
        sach = in_trang_thai(link, nodes, ten, "SAU:")
        if not sach:
            print("!! Loi hien lai ngay sau khi xoa -> loi that (phan cung / chua calib): ros2 run zltech_can_driver calib")
        return 0 if sach else 1
    finally:
        p.bo_qua_tin_hieu()
        link.dong()


if __name__ == "__main__":
    sys.exit(main())
