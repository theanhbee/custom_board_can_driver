// calib - CALIB LAI 2 BANH QUA CAN (Set_Axis_State), lan luot tung banh, in tung buoc + loi.
// Ban C++ cua zltech_can_driver/calib.py.
//
//     ros2 run zltech_can_driver_cpp calib                  # FULL_CALIBRATION (motor + encoder offset) ca 2 banh
//     ros2 run zltech_can_driver_cpp calib --chi=PHAI       # chi 1 banh (TRAI / PHAI)
//     ros2 run zltech_can_driver_cpp calib --kieu=motor     # full (mac dinh, state 3) | motor (4) | encoder (7)
//     ros2 run zltech_can_driver_cpp calib --yes            # khong hoi xac nhan
//     ros2 run zltech_can_driver_cpp calib --trai=0 --phai=2 --cong=/dev/ttyUSB0
//
// CANH BAO: calib lam motor keu bip roi QUAY banh 1 doan -> KE BANH LEN khoi mat dat. Banh cham dat / bi tai
//          -> hay loi CPR_POLEPAIRS_MISMATCH. Truoc khi calib tu xoa loi cu. Ctrl+C -> banh ve IDLE.
// Phai TAT can_bridge / drive_test truoc (cong bi khoa).
// Ket qua calib chi nam trong RAM board; mat nguon co the phai calib lai (tuy cau hinh firmware).
// Thoat 0 = cac banh OK, 1 = co banh loi.
#include <cstdio>
#include <iostream>
#include <tuple>

#include "zltech_can_driver_cpp/cli.hpp"
#include "zltech_can_driver_cpp/protocol.hpp"

using namespace zltech;
using cli::ngu;

static const std::map<std::string, int> KIEU = {{"full", FULL_CALIB}, {"motor", MOTOR_CALIB},
                                                {"encoder", ENCODER_OFFSET}};
constexpr double TOI_DA = 40.0;  // s cho 1 banh

static bool calib_1_banh(CanLink& link, int n, const std::string& ten, int state_calib) {
  NodeState& s = link.tt(n);
  std::printf("\n== %s (node %d): xoa loi cu...\n", ten.c_str(), n);
  link.xoa_loi({n});
  if (!link.cho_heartbeat(1.5, {n})) {
    std::printf("   !! KHONG co heartbeat -> sai node / mat nguon / day CAN\n");
    return false;
  }
  ngu(0.2);  // cho frame loi chi tiet (neu loi hien lai)
  if (s.state != IDLE) {
    link.dat_state(IDLE, {n});
    ngu(0.3);
  }
  std::string loi = s.loi();
  std::printf("   truoc calib: state %s, loi '%s'\n", ten_state(s.state).c_str(), loi.empty() ? "khong" : loi.c_str());
  link.dat_state(state_calib, {n});
  double t0 = mono();
  std::optional<std::tuple<int, uint32_t, uint64_t, uint64_t, uint64_t>> cu;
  bool xong = false;
  while (mono() - t0 < TOI_DA) {
    auto hien = std::make_tuple(s.state.load(), s.axis_error.load(), s.motor.load(), s.encoder.load(),
                                s.controller.load());
    if (hien != cu) {
      std::printf("   t=%5.2fs  %s  %s\n", mono() - t0, ten_state(s.state).c_str(), s.loi().c_str());
      std::fflush(stdout);
      cu = hien;
    }
    if (mono() - t0 > 1.0 && s.state == IDLE) {
      xong = true;
      break;
    }
    ngu(0.05);
  }
  if (!xong) {
    std::printf("   !! qua %.0f s chua xong -> ve IDLE\n", TOI_DA);
    link.dat_state(IDLE, {n});
    return false;
  }
  loi = s.loi();
  bool ok = loi.empty();
  std::printf("== %s: %s  (%.1f s)\n", ten.c_str(), ok ? "OK" : ("LOI: " + loi).c_str(), mono() - t0);
  if (!ok && s.motor)
    std::printf("   -> loi motor (vd UNBALANCED_PHASES / PHASE_RESISTANCE): kiem tra 3 day pha, do dien tro A-B, "
                "B-C, C-A\n");
  else if (!ok && s.encoder)
    std::printf("   -> loi encoder (vd CPR_POLEPAIRS_MISMATCH): ke banh len cho quay tu do, kiem tra giac "
                "encoder/hall\n");
  return ok;
}

int main(int argc, char** argv) {
  cli::Args ap("Calib lai banh qua CAN");
  cli::them_tham_so_can(ap);
  ap.them("kieu", "full", "full | motor | encoder", {"full", "motor", "encoder"});
  ap.them("chi", "", "chi 1 banh: TRAI / PHAI");
  ap.co("yes", "khong hoi xac nhan");
  ap.doc(argc, argv);
  try {
    auto ten = cli::ten_banh(ap);
    auto nodes = cli::chon_banh(ap, ten);
    std::string kieu = ap.s("kieu");
    std::vector<std::string> ds;
    for (int n : nodes) ds.push_back(strf("%s = node %d", ten[n].c_str(), n));
    std::string kieu_hoa = kieu;
    for (auto& c : kieu_hoa) c = static_cast<char>(toupper(c));
    std::printf("CALIB %s (state %d): %s\n", kieu_hoa.c_str(), KIEU.at(kieu), noi(ds, ", ").c_str());
    std::printf("!! Motor se QUAY banh 1 doan -> banh phai duoc KE LEN khoi mat dat.\n");
    if (!ap.b("yes")) {
      std::printf("Enter de bat dau, Ctrl+C de huy: ");
      std::fflush(stdout);
      std::string dong;
      if (!std::getline(std::cin, dong)) {
        std::fprintf(stderr, "\nHuy.\n");
        return 1;
      }
    }

    cli::bat_tin_hieu();
    CanLink link(nodes, ap.s("adapter"), ap.s("cong"), ap.i("baud-serial"));
    std::map<int, bool> ket_qua;
    try {
      ngu(0.5);
      for (int n : nodes) ket_qua[n] = calib_1_banh(link, n, ten[n], KIEU.at(kieu));
    } catch (const cli::NgatCtrlC&) {
      std::printf("\n\nHuy (Ctrl+C)\n");
    }
    bo_qua_tin_hieu();
    link.dat_state(IDLE);
    ngu_s(0.05);
    link.dong();

    std::vector<std::string> kq;
    bool tat_ca = !ket_qua.empty();
    for (int n : nodes) {
      bool ok = ket_qua.count(n) && ket_qua[n];
      kq.push_back(ten[n] + (ok ? " OK" : " LOI"));
      tat_ca = tat_ca && ok;
    }
    std::printf("\nKET QUA: %s\n", noi(kq, ", ").c_str());
    return tat_ca ? 0 : 1;
  } catch (const FatalError& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
}
