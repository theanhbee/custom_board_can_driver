// clear_errors - XOA LOI 2 BANH QUA CAN (Clear_Errors 0x18), in loi TRUOC va SAU khi xoa.
// Ban C++ cua zltech_can_driver/clear_errors.py.
//
//     ros2 run zltech_can_driver_cpp clear_errors              # ca 2 banh: TRAI (node 0), PHAI (node 2)
//     ros2 run zltech_can_driver_cpp clear_errors --chi=TRAI   # chi 1 banh
//     ros2 run zltech_can_driver_cpp clear_errors --xem        # chi doc loi, KHONG xoa
//     ros2 run zltech_can_driver_cpp clear_errors --trai=0 --phai=2 --cong=/dev/ttyUSB0
//
// Khong lam banh quay. Clear_Errors khong doi state (banh dang IDLE van IDLE).
// Loi xoa xong lai hien ngay (motor / encoder hong that) -> in canh bao: sua phan cung hoac calib lai.
// Phai TAT can_bridge / drive_test truoc (cong bi khoa).
// Thoat 0 = cac banh sach loi, 1 = con loi / khong co heartbeat.
#include <cstdio>

#include "zltech_can_driver_cpp/cli.hpp"
#include "zltech_can_driver_cpp/protocol.hpp"

using namespace zltech;
using cli::ngu;

static bool in_trang_thai(CanLink& link, const std::vector<int>& nodes, std::map<int, std::string>& ten,
                          const char* tieu_de) {
  std::printf("%s\n", tieu_de);
  bool ok = true;
  for (int n : nodes) {
    NodeState& s = link.tt(n);
    if (!s.t_hb || mono() - s.t_hb > 1.0) {
      std::printf("  %s (node %d): !! KHONG co heartbeat -> sai node / mat nguon / day CAN\n", ten[n].c_str(), n);
      ok = false;
      continue;
    }
    std::string loi = s.loi();
    std::printf("  %s (node %d): state %s, axis_error 0x%X  %s\n", ten[n].c_str(), n, ten_state(s.state).c_str(),
                s.axis_error.load(), loi.empty() ? "sach loi" : loi.c_str());
    ok = ok && loi.empty();
  }
  return ok;
}

int main(int argc, char** argv) {
  cli::Args ap("Xoa loi banh qua CAN");
  cli::them_tham_so_can(ap);
  ap.them("chi", "", "chi 1 banh: TRAI / PHAI");
  ap.co("xem", "chi doc loi, khong xoa");
  ap.doc(argc, argv);
  try {
    auto ten = cli::ten_banh(ap);
    auto nodes = cli::chon_banh(ap, ten);
    cli::bat_tin_hieu();
    CanLink link(nodes, ap.s("adapter"), ap.s("cong"), ap.i("baud-serial"));
    int ma = 1;
    try {
      ngu(1.0);  // gom heartbeat (10 lan/s) + frame loi chi tiet
      bool sach = in_trang_thai(link, nodes, ten, "TRUOC:");
      if (ap.b("xem")) {
        ma = sach ? 0 : 1;
      } else {
        std::vector<std::string> ds;
        for (int n : nodes) ds.push_back(ten[n]);
        std::printf("Gui Clear_Errors cho %s...\n", noi(ds, ", ").c_str());
        link.xoa_loi(nodes);
        ngu(0.8);
        sach = in_trang_thai(link, nodes, ten, "SAU:");
        if (!sach)
          std::printf("!! Loi hien lai ngay sau khi xoa -> loi that (phan cung / chua calib): "
                      "ros2 run zltech_can_driver_cpp calib\n");
        ma = sach ? 0 : 1;
      }
    } catch (const cli::NgatCtrlC&) {
      std::printf("\nHuy (Ctrl+C)\n");
    }
    bo_qua_tin_hieu();
    link.dong();
    return ma;
  } catch (const FatalError& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
}
