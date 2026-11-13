// drive_test - LAI THU 2 BANH TRUC TIEP QUA CAN, KHONG CAN ROS. Ban C++ cua zltech_can_driver/drive_test.py.
//
//     ros2 run zltech_can_driver_cpp drive_test                      # Waveshare tu do CH340, trai = 0, phai = 2
//     ros2 run zltech_can_driver_cpp drive_test --dao=PHAI           # banh lap nguoc (TRAI / PHAI / '' = khong dao)
//     ros2 run zltech_can_driver_cpp drive_test --vmax=2.0           # toc do toi da moi banh (vong/s, <= 5.5)
//     ros2 run zltech_can_driver_cpp drive_test --rong=0.382 --duong-kinh=0.139 --tre-ms=5 --xoa-loi
//     ros2 run zltech_can_driver_cpp drive_test --adapter=socketcan --cong=can0
//
// Buoc 1 - KIEM TRA: nghe heartbeat 2 s. Phai thay CA 2 node, state IDLE / CLOSED_LOOP, khong loi. Chi WATCHDOG (lan
//          truoc dung ngang) -> tu xoa. Loi khac -> thoat.
// Buoc 2 - VAO CLOSED_LOOP ca 2 node (toi da 3 s).
// Buoc 3 - LAI (phim trong terminal, khong can Enter):
//          W / S : tang / giam van toc tien (10 lan bam = vmax)  A / D : quay trai / phai (10 lan bam = toi da)
//          SPACE : dung em (ramp)                                X     : PHANH GAP (PASSTHROUGH ve 0, roi VEL_RAMP)
//          Q / Ctrl+C : thoat, 2 banh ve IDLE
// Moi 0.1 s (va ngay khi bam phim): 0x1E staged ca 2 node -> cho ACK -> 0x7FF SYNC_GO.
// An toan: banh mat lien lac (0.3 s khong frame / 1 s khong heartbeat), roi CLOSED_LOOP, bao loi, hoac thieu ACK
//         3 chu ky lien tiep -> ca 2 ve IDLE. Day la tool THU: dung ngay khi co loi (khong tu phuc hoi).
// Phai TAT can_bridge truoc (cong bi khoa).
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <optional>

#include "zltech_can_driver_cpp/cli.hpp"
#include "zltech_can_driver_cpp/protocol.hpp"

using namespace zltech;
using cli::ngu;

constexpr double CHU_KY = 0.1;        // s, gui lai lenh dong bo
constexpr double ACK_TIMEOUT = 0.015; // s cho ACK
constexpr int SAI_ACK_TOI_DA = 3;     // chu ky thieu ACK lien tiep (0.3 s) -> dung xe
constexpr double VEL_DUNG = 0.05;     // vong/s, coi nhu da dung
constexpr double PHANH_TOI_DA = 1.5;  // s

// Loi dung chuong trinh (sys.exit("...") ben Python): in roi thoat 1 SAU khi dua banh ve IDLE
struct Thoat {
  std::string msg;
};

class DriveTest {
 public:
  explicit DriveTest(const cli::Args& a)
      : a_(a), ten_(cli::ten_banh(a)), left_(a.i("trai")), right_(a.i("phai")), nodes_{left_, right_} {
    std::string dao = a.s("dao");
    std::transform(dao.begin(), dao.end(), dao.begin(), ::toupper);
    dau_ = {{left_, dao == "TRAI" ? -1.0 : 1.0}, {right_, dao == "PHAI" ? -1.0 : 1.0}};
    rong_ = a.d("rong");
    dk_ = a.d("duong-kinh");
    vong_max_ = std::min(a.d("vmax"), 5.5);
    v_max_ = vong_max_ * M_PI * dk_;
    w_max_ = v_max_ / rong_;
    link_ = std::make_unique<CanLink>(nodes_, a.s("adapter"), a.s("cong"), a.i("baud-serial"));
  }

  int chay() {
    int ma = 0;
    try {
      if (a_.b("xoa-loi")) xoa_loi(nodes_);
      bool ok = kiem_tra_heartbeat();
      std::vector<int> chi_wd;
      std::vector<std::string> ten_wd;
      for (int n : nodes_)
        if (link_->tt(n).axis_error == ERR_WATCHDOG && !link_->tt(n).details().any()) {
          chi_wd.push_back(n);
          ten_wd.push_back(ten_[n]);
        }
      bool chi_loi_wd = !chi_wd.empty();
      for (int n : nodes_)
        chi_loi_wd = chi_loi_wd &&
                     (link_->tt(n).loi().empty() || std::find(chi_wd.begin(), chi_wd.end(), n) != chi_wd.end());
      if (!ok && chi_loi_wd) {
        std::printf("  -> chi la loi WATCHDOG con lai tu lan chay truoc (%s): tu xoa\n", noi(ten_wd, ", ").c_str());
        xoa_loi(chi_wd);
        ok = kiem_tra_heartbeat();
      }
      if (!ok) throw Thoat{"!! Heartbeat chua dat -> KHONG chay. Sua loi (clear_errors / calib) roi chay lai."};
      if (!vao_closed_loop()) throw Thoat{"!! Khong vao duoc CLOSED_LOOP ca 2 banh."};
      std::string ly_do;
      {
        cli::BanPhim kb;
        ly_do = lai(kb);
      }
      std::printf("\n\nDung: %s\n", ly_do.c_str());
      for (int n : nodes_)
        if (!link_->tt(n).loi().empty()) std::printf("  %s: %s\n", ten_[n].c_str(), link_->tt(n).loi().c_str());
    } catch (const cli::NgatCtrlC&) {
      std::printf("\n\nDung: Ctrl+C\n");
    } catch (const Thoat& t) {
      std::fprintf(stderr, "%s\n", t.msg.c_str());
      ma = 1;
    }
    bo_qua_tin_hieu();
    try {
      link_->dua_ve_idle();
    } catch (const CanError& e) {
      std::printf("!! gui IDLE loi: %s\n", e.what());
    }
    std::printf("2 banh -> IDLE.\n");
    link_->dong();
    return ma;
  }

 private:
  // Buoc 1. true neu ca 2 banh song va sach loi.
  bool kiem_tra_heartbeat() {
    std::printf("Nghe heartbeat 2 s: TRAI = node %d, PHAI = node %d...\n", left_, right_);
    std::fflush(stdout);
    ngu(2.0);
    bool ok = true;
    for (int n : nodes_) {
      NodeState& s = link_->tt(n);
      if (!s.t_hb) {
        std::printf("  !! %s (node %d): KHONG co heartbeat -> sai node_id / mat nguon / day CAN\n", ten_[n].c_str(), n);
        ok = false;
        continue;
      }
      std::string loi = s.loi();
      bool tot = loi.empty() && (s.state == IDLE || s.state == CLOSED_LOOP);
      std::printf("  %s (node %d): state %s, axis_error 0x%X  %s  %s\n", ten_[n].c_str(), n,
                  ten_state(s.state).c_str(), s.axis_error.load(), tot ? "OK" : "LOI", loi.c_str());
      ok = ok && tot;
    }
    return ok;
  }

  void xoa_loi(const std::vector<int>& nodes) {
    std::vector<std::string> ds;
    for (int n : nodes) ds.push_back(ten_[n]);
    std::printf("Gui Clear_Errors cho %s...\n", noi(ds, ", ").c_str());
    link_->xoa_loi(nodes);
    ngu(0.3);
  }

  // Buoc 2.
  bool vao_closed_loop() {
    std::printf("Vao CLOSED_LOOP ca 2 banh...\n");
    link_->van_toc_0();  // input_vel cu con trong RAM -> VEL_RAMP se chay toi do ngay
    link_->dat_input_mode(VEL_RAMP);
    link_->dat_state(CLOSED_LOOP);
    double t0 = mono(), t_an = t0;
    while (mono() - t0 < 3.0) {
      bool du = true, co_loi = false;
      for (int n : nodes_) {
        du = du && link_->tt(n).state == CLOSED_LOOP && link_->con_song(n);
        co_loi = co_loi || !link_->tt(n).loi().empty();
      }
      if (du) {
        std::printf("  OK sau %.2f s\n", mono() - t0);
        return true;
      }
      if (co_loi) break;
      if (mono() - t_an >= CHU_KY) {
        link_->van_toc_0();  // an watchdog CAN trong luc cho
        t_an = mono();
      }
      ngu(0.02);
    }
    for (int n : nodes_)
      std::printf("  %s: state %s  %s\n", ten_[n].c_str(), so_state(link_->tt(n).state).c_str(),
                  link_->tt(n).loi().c_str());
    return false;
  }

  VongMap toc_do_banh(double v, double w) {
    double m_tren_vong = M_PI * dk_;
    double vl = (v - w * rong_ / 2) / m_tren_vong;
    double vr = (v + w * rong_ / 2) / m_tren_vong;
    double k = std::max({1.0, std::abs(vl) / vong_max_, std::abs(vr) / vong_max_});  // giu ti le khi cham gioi han
    return {{left_, dau_[left_] * vl / k}, {right_, dau_[right_] * vr / k}};
  }

  std::optional<std::string> ly_do_dung() {
    double now = mono();
    for (int n : nodes_) {
      NodeState& s = link_->tt(n);
      if (!link_->con_song(n))
        return strf("%s mat lien lac (frame cuoi %.2f s, heartbeat cuoi %.2f s truoc)", ten_[n].c_str(),
                    now - s.t_rx, now - s.t_hb);
      if (s.state != CLOSED_LOOP) return strf("%s roi CLOSED_LOOP (state %s)", ten_[n].c_str(), so_state(s.state).c_str());
      if (!s.loi().empty()) return ten_[n] + " loi: " + s.loi();
    }
    return std::nullopt;
  }

  static double lam_tron(double x) { return std::round(x * 1000.0) / 1000.0; }

  // Buoc 3.
  std::string lai(const cli::BanPhim& kb) {
    std::printf("\nW/S tien-lui, A/D quay trai-phai, SPACE dung em, X PHANH GAP, Q thoat.\n\n");
    double buoc_v = v_max_ / 10, buoc_w = w_max_ / 10, v = 0.0, w = 0.0;
    std::map<int, int> sai_ack = {{left_, 0}, {right_, 0}};
    int so_chu_ky = 0, so_thieu = 0;
    double tre_max = 0.0, t_gui = 0.0;
    std::optional<double> t_phanh;
    while (true) {
      if (cli::g_dung) throw cli::NgatCtrlC{};
      bool doi = false;
      while (kb.kbhit()) {
        char c = static_cast<char>(std::tolower(kb.getch()));
        if (c == 'q') return "nguoi dung thoat";
        if (t_phanh) continue;  // dang phanh: bo qua phim lai
        if (c == 'x') {
          v = w = 0.0;
          link_->van_toc_0();  // ve 0 TRUOC khi doi PASSTHROUGH (tranh vot lenh cu)
          link_->dat_input_mode(PASSTHROUGH);
          t_phanh = mono();
          std::printf("\n  PHANH GAP...\n");
          std::fflush(stdout);
          continue;
        }
        if (c == 'w')
          v += buoc_v;
        else if (c == 's')
          v -= buoc_v;
        else if (c == 'a')
          w += buoc_w;
        else if (c == 'd')
          w -= buoc_w;
        else if (c == ' ')
          v = w = 0.0;
        else
          continue;
        v = std::clamp(lam_tron(v), -v_max_, v_max_);
        w = std::clamp(lam_tron(w), -w_max_, w_max_);
        doi = true;
      }

      if (auto ly_do = ly_do_dung()) return *ly_do;

      if (t_phanh) {
        bool da_dung = true;
        for (int n : nodes_) da_dung = da_dung && std::abs(link_->tt(n).vel.load()) < VEL_DUNG;
        if (da_dung || mono() - *t_phanh >= PHANH_TOI_DA) {
          link_->dat_input_mode(VEL_RAMP);
          std::string why = da_dung ? "da dung" : strf("het %.1f s", PHANH_TOI_DA);
          std::printf("  %s sau %.2f s -> tra lai VEL_RAMP\n", why.c_str(), mono() - *t_phanh);
          std::fflush(stdout);
          t_phanh.reset();
        }
      }

      if (doi || mono() - t_gui >= CHU_KY) {
        VongMap vong = toc_do_banh(v, w);
        double t0 = mono();
        auto thieu = link_->gui_dong_bo(vong, ACK_TIMEOUT, a_.d("tre-ms") / 1000.0);
        double tre = mono() - t0;
        t_gui = mono();
        ++so_chu_ky;
        so_thieu += !thieu.empty();
        tre_max = std::max(tre_max, tre);
        for (int n : nodes_) {
          sai_ack[n] = std::find(thieu.begin(), thieu.end(), n) != thieu.end() ? sai_ack[n] + 1 : 0;
          if (sai_ack[n] >= SAI_ACK_TOI_DA) return strf("%s mat ACK staged %d chu ky lien tiep", ten_[n].c_str(), SAI_ACK_TOI_DA);
        }
        std::printf("\r  v %+.2f m/s  w %+.2f rad/s  |  TRAI %+.2f  PHAI %+.2f vong/s  |  GO sau %4.1f ms "
                    "(max %5.1f)  thieu ACK %d/%d   ",
                    v, w, vong[left_], vong[right_], tre * 1000, tre_max * 1000, so_thieu, so_chu_ky);
        std::fflush(stdout);
      }
      ngu_s(0.005);
    }
  }

  const cli::Args& a_;
  std::map<int, std::string> ten_;
  int left_, right_;
  std::vector<int> nodes_;
  std::map<int, double> dau_;
  double rong_, dk_, vong_max_, v_max_, w_max_;
  std::unique_ptr<CanLink> link_;
};

int main(int argc, char** argv) {
  cli::Args ap("Lai thu 2 banh qua CAN (khong ROS)");
  cli::them_tham_so_can(ap);
  ap.them("dao", "", "banh lap nguoc: TRAI / PHAI / '' (khong dao)");
  ap.them("vmax", "4.0", "vong/s toi da moi banh (<= 5.5)");
  ap.them("rong", "0.382", "khoang cach 2 banh [m]");
  ap.them("duong-kinh", "0.139", "duong kinh banh [m]");
  ap.them("tre-ms", "0.0", "cho them giua luc du ACK va luc phat GO [ms]");
  ap.co("xoa-loi", "gui Clear_Errors truoc khi kiem tra");
  ap.doc(argc, argv);
  cli::bat_tin_hieu();
  try {
    DriveTest dt(ap);
    return dt.chay();
  } catch (const FatalError& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
}
