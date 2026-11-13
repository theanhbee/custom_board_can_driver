// error_monitor - NGHE BUS CAN, GIAI MA MOI BIT LOI cua firmware Can-sync thanh ten + y nghia + goi y xu ly.
// Ban C++ cua zltech_can_driver/error_monitor.py.
//
//     ros2 run zltech_can_driver_cpp error_monitor                         # Waveshare tu do, node 0 = TRAI, 2 = PHAI
//     ros2 run zltech_can_driver_cpp error_monitor --clear                 # gui CLEAR_ERRORS luc bat dau
//     ros2 run zltech_can_driver_cpp error_monitor --log loi.csv           # ghi CSV: time, node, frame, raw_hex, decoded
//     ros2 run zltech_can_driver_cpp error_monitor --nodes 0,1,2,3 --names 0=TRAI,2=PHAI
//     ros2 run zltech_can_driver_cpp error_monitor --interface socketcan --channel can0
//     ros2 run zltech_can_driver_cpp error_monitor --thu-dong              # chi nghe, KHONG gui RTR / clear
//     ros2 run zltech_can_driver_cpp error_monitor --replay candump.log    # doc log candump, khong can phan cung
//
// Phim (khi chay that): c = CLEAR_ERRORS cac node dang theo doi, r = hoi lai frame loi (RTR), q / Ctrl+C = thoat.
// In log CHI KHI THAY DOI (state, bit loi them / het, CLEARED, mat / co lai heartbeat), khong spam frame lap 100 ms.
// Dong cuoi terminal = bang tong hop lam moi lien tuc.
// Tu hoi (RTR) frame chi tiet khi heartbeat bao *_FAILED ma 200 ms chua thay frame do (sensorless 0x05 luon phai hoi);
// motor SYSTEM_LEVEL / BRAKE_RESISTOR_DISARMED -> hoi Vbus (0x17) va in bang loi he thong (khong doc duoc qua CAN).
// Waveshare la cong doc quyen: phai TAT can_bridge / drive_test truoc. Gui RTR / clear cung "cho an" watchdog firmware.
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <functional>
#include <optional>
#include <set>

#include "zltech_can_driver_cpp/cli.hpp"
#include "zltech_can_driver_cpp/protocol.hpp"

using namespace zltech;
namespace E = zltech::err;

constexpr double HB_MAT = 0.3;        // s khong co heartbeat -> canh bao mat heartbeat
constexpr double CHO_CHI_TIET = 0.2;  // s sau *_FAILED ma chua co frame chi tiet -> hoi RTR
constexpr double HOI_LAI_SAU = 1.0;   // s, khong hoi lai cung 1 frame day hon
constexpr double BANG_CHU_KY = 0.5;   // s lam moi dong tong hop
static const char* KINDS[] = {"motor", "encoder", "controller", "sensorless"};

static double gio_that() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

static std::string gio(double t) {
  time_t s = static_cast<time_t>(t);
  tm lt{};
  localtime_r(&s, &lt);
  char b[32];
  std::strftime(b, sizeof(b), "%H:%M:%S", &lt);
  return strf("%s.%03d", b, static_cast<int>(static_cast<long long>(t * 1000) % 1000));
}

static std::string hex(const std::vector<uint8_t>& d) {
  std::string s;
  for (auto b : d) s += strf("%02x", b);
  return s;
}

struct NodeView {
  std::optional<double> t_hb;  // heartbeat cuoi
  bool hb_mat = false;
  std::optional<int> state;
  uint64_t axis_error = 0;
  std::map<std::string, uint64_t> err = {{"motor", 0}, {"encoder", 0}, {"controller", 0}, {"sensorless", 0}};
  std::map<std::string, double> t_chi_tiet;  // kind -> lan cuoi nhan frame chi tiet
  std::map<std::string, double> t_failed;    // kind -> luc heartbeat bat dau bao *_FAILED
  std::map<int, double> t_hoi;               // cmd -> lan cuoi gui RTR
  std::optional<double> vbus;
  bool canh_bao_du_phong = false;
};

using GuiFn = std::function<void(uint32_t arb, bool rtr)>;

class Monitor {
 public:
  // nodes: node_id theo doi (nullopt = tat ca). gui(arb, rtr) hoac rong (replay / thu dong).
  Monitor(std::optional<std::set<int>> nodes, std::map<int, std::string> names, GuiFn gui, std::ofstream* csv,
          bool live_status = true)
      : nodes(std::move(nodes)), names_(std::move(names)), gui_(std::move(gui)), csv_(csv),
        live_(live_status && isatty(STDOUT_FILENO)) {
    if (csv_) *csv_ << "time,node,frame,raw_hex,decoded\r\n";
  }

  std::optional<std::set<int>> nodes;
  std::map<int, NodeView> v;

  // ---------------------------------------------------------------- in
  std::string ten(int n) const {
    if (n < 0) return "";
    auto it = names_.find(n);
    return strf("%s(node %d)", it != names_.end() ? it->second.c_str() : "NODE", n);
  }

  void in_dong(double t, int n, const std::string& msg) {
    ++so_su_kien_;
    if (live_) std::fputs("\r\033[K", stdout);
    std::printf("%s  %-14s %s\n", gio(t).c_str(), ten(n).c_str(), msg.c_str());
    std::fflush(stdout);
  }

  void in_bit(double t, int n, const std::string& nhom, const E::BitInfo& e, bool them) {
    if (them) {
      std::string goi_y = e.hint.empty() ? "" : "\n" + std::string(29, ' ') + "-> " + e.hint;
      in_dong(t, n, strf("%s: + %s (0x%llX) %s", nhom.c_str(), e.name.c_str(), static_cast<unsigned long long>(e.mask),
                         e.desc.c_str()) + goi_y);
    } else {
      in_dong(t, n, nhom + ": - " + e.name + " het");
    }
  }

  void in_diff(double t, int n, const std::string& nhom, uint64_t cu, uint64_t moi, const E::Table& table) {
    for (const auto& e : E::decode_bits(moi & ~cu, table)) in_bit(t, n, nhom, e, true);
    for (const auto& e : E::decode_bits(cu & ~moi, table)) in_bit(t, n, nhom, e, false);
  }

  void bang(double t, bool ep = false) {
    if (!live_ || (!ep && t - t_bang_ < BANG_CHU_KY)) return;
    t_bang_ = t;
    std::vector<std::string> phan;
    for (const auto& [n, s] : v) {
      std::string tt;
      if (!s.t_hb) {
        tt = "chua co HB";
      } else if (s.hb_mat) {
        tt = strf("MAT HB %.1fs", t - *s.t_hb);
      } else {
        std::vector<std::string> loi;
        for (const char* k : KINDS)
          if (s.err.at(k)) loi.push_back(k);
        if (s.axis_error) loi.push_back("axis");
        tt = E::state_name(s.state.value_or(0)) + " " + (loi.empty() ? "ok" : "LOI " + noi(loi, ","));
      }
      auto it = names_.find(n);
      phan.push_back((it != names_.end() ? it->second : std::to_string(n)) + ": " + tt);
    }
    std::printf("\r\033[K  [%s]  c=clear r=hoi q=thoat", noi(phan, " | ").c_str());
    std::fflush(stdout);
  }

  // ---------------------------------------------------------------- xu ly frame
  void xu_ly(double t, uint32_t arb, const std::vector<uint8_t>& data, bool is_ext = false, bool is_rtr = false) {
    if (!t0_) t0_ = t;
    int n = static_cast<int>(arb >> 5), cmd = static_cast<int>(arb & 0x1F);
    if (is_ext || is_rtr || (nodes && !nodes->count(n))) return;
    if (cmd == E::CMD_BUS_VOLTAGE_CURRENT && data.size() >= 8) {
      float vbus, ibus;
      std::memcpy(&vbus, data.data(), 4);
      std::memcpy(&ibus, data.data() + 4, 4);
      NodeView& s = v[n];
      if (!s.vbus || std::abs(*s.vbus - vbus) > 0.5)
        in_dong(t, n, strf("Vbus %.2f V, Ibus %.2f A (frame 0x17)", vbus, ibus));
      s.vbus = vbus;
      return;
    }
    auto d = E::decode_frame(arb, data);
    if (!d) return;
    NodeView& s = v[n];
    if (csv_) {
      std::string dec = tom_tat(*d);
      if (dec.find_first_of(",\"") != std::string::npos) {
        std::string q;
        for (char c : dec) q += c == '"' ? std::string("\"\"") : std::string(1, c);
        dec = "\"" + q + "\"";
      }
      *csv_ << strf("%.6f,%d,0x%03X %s,", t, n, arb, d->kind.c_str()) << hex(data) << "," << dec << "\r\n";
    }
    if (d->kind == "heartbeat")
      heartbeat(t, n, s, *d);
    else
      chi_tiet(t, n, s, *d);
  }

  static std::string tom_tat(const E::Decoded& d) {
    std::vector<std::string> ns;
    for (const auto& e : d.errors) ns.push_back(e.name);
    if (d.kind == "heartbeat")
      return strf("%s axis_error=0x%llX ", d.state_name.c_str(), static_cast<unsigned long long>(d.axis_error)) +
             noi(ns, "|");
    return strf("%s_error=0x%llX ", d.kind.c_str(), static_cast<unsigned long long>(d.value)) +
           (d.cleared ? "CLEARED" : noi(ns, "|"));
  }

  void heartbeat(double t, int n, NodeView& s, const E::Decoded& d) {
    if (!s.t_hb)
      in_dong(t, n, "heartbeat dau tien: " + d.state_name +
                        (d.axis_error ? strf(", axis_error 0x%llX", static_cast<unsigned long long>(d.axis_error))
                                      : ", khong loi"));
    else if (s.hb_mat)
      in_dong(t, n, strf("CO LAI heartbeat sau %.2f s", t - *s.t_hb));
    s.t_hb = t;
    s.hb_mat = false;
    if (d.reserved_nonzero && !s.canh_bao_du_phong) {
      s.canh_bao_du_phong = true;
      in_dong(t, n, strf("!! heartbeat byte 4-7 = 0x%08X khac 0 -> firmware la (khong phai Can-sync err-event?), "
                         "giai ma co the sai",
                         d.reserved));
    }
    if (s.state && d.state != *s.state) in_dong(t, n, "STATE " + E::state_name(*s.state) + " -> " + d.state_name);
    s.state = d.state;
    uint64_t cu = s.axis_error, moi = d.axis_error;
    if (cu != moi) in_diff(t, n, "axis", cu, moi, E::AXIS_ERRORS);
    s.axis_error = moi;
    for (const auto& f : E::FAILED_TO_DETAIL) {
      if (moi & f.bit)
        s.t_failed.emplace(f.kind, t);  // setdefault
      else
        s.t_failed.erase(f.kind);
    }
  }

  void chi_tiet(double t, int n, NodeView& s, const E::Decoded& d) {
    const std::string& kind = d.kind;
    uint64_t cu = s.err.at(kind), moi = d.value;
    s.t_chi_tiet[kind] = t;
    if (moi == cu) return;
    if (d.cleared) {
      in_dong(t, n, kind + ": CLEARED (truoc: " + E::bit_names(cu, E::table_for(kind)) + ")");
    } else {
      in_diff(t, n, kind, cu, moi, E::table_for(kind));
      if (kind == "motor" && ((moi & ~cu) & E::MOTOR_SYSTEM_HINT_MASK)) {
        std::vector<std::string> ds;
        for (const auto& [m, info] : E::SYSTEM_ERRORS) ds.push_back(std::string(info.name) + " (" + info.desc + ")");
        in_dong(t, n, "motor: nguyen nhan goc thuong la LOI HE THONG ODrive (khong co tren CAN): " + noi(ds, ", "));
        hoi(t, n, s, E::CMD_BUS_VOLTAGE_CURRENT, "Vbus");
      }
    }
    s.err[kind] = moi;
  }

  // ---------------------------------------------------------------- dinh ky
  void hoi(double t, int n, NodeView& s, int cmd, const std::string& ly_do) {
    auto it = s.t_hoi.find(cmd);
    if (it != s.t_hoi.end() && t - it->second < HOI_LAI_SAU) return;
    s.t_hoi[cmd] = t;
    uint32_t arb = E::arb_id(n, cmd);
    if (!gui_) {
      in_dong(t, n, strf("(se hoi RTR 0x%03X %s; replay / thu dong -> khong gui)", arb, ly_do.c_str()));
      return;
    }
    try {
      gui_(arb, true);
    } catch (const std::exception& e) {
      in_dong(t, n, strf("!! gui RTR 0x%03X loi: %s", arb, e.what()));
    }
  }

  void tick(double t) {
    for (auto& [n, s] : v) {
      if (s.t_hb && !s.hb_mat && t - *s.t_hb > HB_MAT) {
        s.hb_mat = true;
        in_dong(t, n, strf("!! MAT HEARTBEAT > %.0f ms (node offline / mat nguon / mat CAN / adapter dung hinh)",
                           HB_MAT * 1000));
      }
      for (const auto& [kind, t_fail] : std::map<std::string, double>(s.t_failed)) {
        int cmd = 0;
        for (const auto& f : E::FAILED_TO_DETAIL)
          if (kind == f.kind) cmd = f.cmd;
        auto ct = s.t_chi_tiet.find(kind);
        bool chua_co = ct == s.t_chi_tiet.end() || ct->second < t_fail;
        if (t - t_fail > CHO_CHI_TIET && (chua_co || kind == "sensorless"))
          hoi(t, n, s, cmd, kind + ": heartbeat bao *_FAILED ma chua co frame chi tiet");
      }
    }
    if (nodes && t0_ && t - *t0_ > 1.0)
      for (int n : *nodes)
        if (!v.count(n)) {
          v[n] = NodeView();
          in_dong(t, n, "!! chua nhan heartbeat nao sau 1 s (sai node / mat nguon / day CAN)");
        }
    bang(t);
  }

  void hoi_tat_ca(double t) {
    for (auto& [n, s] : v)
      for (int cmd : {E::CMD_MOTOR_ERROR, E::CMD_ENCODER_ERROR, E::CMD_CONTROLLER_ERROR, E::CMD_SENSORLESS_ERROR}) {
        s.t_hoi.erase(cmd);
        hoi(t, n, s, cmd, "theo yeu cau (phim r)");
      }
  }

  void tong_ket() {
    if (live_) std::fputs("\r\033[K", stdout);
    std::printf("\n===== TONG KET (%d su kien)\n", so_su_kien_);
    for (const auto& [n, s] : v) {
      std::printf("  %s: state %s, axis 0x%llX %s\n", ten(n).c_str(), s.state ? E::state_name(*s.state).c_str() : "-",
                  static_cast<unsigned long long>(s.axis_error), E::bit_names(s.axis_error, E::AXIS_ERRORS).c_str());
      for (const char* k : KINDS)
        if (uint64_t val = s.err.at(k))
          std::printf("      %s: 0x%llX %s\n", k, static_cast<unsigned long long>(val),
                      E::bit_names(val, E::table_for(k)).c_str());
    }
  }

 private:
  std::map<int, std::string> names_;
  GuiFn gui_;
  std::ofstream* csv_;
  bool live_;
  std::optional<double> t0_;
  double t_bang_ = 0.0;
  int so_su_kien_ = 0;
};

// ============================================================================================================
static std::map<int, std::string> parse_names(const std::string& text) {
  std::map<int, std::string> out;
  size_t s = 0;
  while (s <= text.size()) {
    size_t e = text.find(',', s);
    std::string part = text.substr(s, e == std::string::npos ? std::string::npos : e - s);
    if (!part.empty()) {
      size_t eq = part.find('=');
      out[std::stoi(part.substr(0, eq))] = eq == std::string::npos ? "" : part.substr(eq + 1);
    }
    if (e == std::string::npos) break;
    s = e + 1;
  }
  return out;
}

static void chay_replay(const std::string& file, Monitor& mon) {
  std::ifstream f(file);
  if (!f) throw FatalError("khong mo duoc " + file);
  std::optional<double> t;
  std::string line;
  while (std::getline(f, line)) {
    auto r = E::parse_candump_line(line);
    if (!r) continue;
    t = r->t;
    mon.tick(*t);
    mon.xu_ly(*t, r->arb, r->data, r->is_extended, r->is_remote);
  }
  if (t) mon.tick(*t);
}

static void chay_that(const cli::Args& a, Monitor& mon, CanBus& bus) {
  bool thu_dong = a.b("thu-dong");
  std::printf("Nghe %s, node %s ... (c=clear, r=hoi, q=thoat)\n", bus.channel_info().c_str(), a.s("nodes").c_str());
  auto gui_clear = [&](int n) {
    CanFrame f;
    f.id = E::arb_id(n, E::CMD_CLEAR_ERRORS);
    bus.send(f);
  };
  auto ds_node = [](const auto& c) {
    std::vector<std::string> v;
    for (const auto& x : c) v.push_back(std::to_string(x));
    return "[" + noi(v, ", ") + "]";
  };
  if (a.b("clear") && !thu_dong) {
    std::set<int> ns = mon.nodes.value_or(std::set<int>{});
    for (int n : ns) gui_clear(n);
    std::printf("Da gui CLEAR_ERRORS cho node %s\n", ds_node(ns).c_str());
  }
  cli::BanPhim kb;
  while (!cli::g_dung) {
    CanFrame m;
    bool co = false;
    try {
      co = bus.recv(m, 0.05);
    } catch (const CanError&) {
      co = false;  // frame serial hong -> bo qua, doc tiep
    }
    double now = gio_that();
    if (co) mon.xu_ly(now, m.id, std::vector<uint8_t>(m.data, m.data + m.dlc), m.ext, m.rtr);
    mon.tick(now);
    if (kb.kbhit()) {
      char c = static_cast<char>(std::tolower(kb.getch()));
      if (c == 'q') break;
      if (c == 'c') {
        if (thu_dong) {
          mon.in_dong(now, -1, "(--thu-dong: khong gui CLEAR_ERRORS)");
        } else {
          std::vector<int> ns;
          for (const auto& kv : mon.v) ns.push_back(kv.first);
          for (int n : ns) gui_clear(n);
          mon.in_dong(now, -1, "da gui CLEAR_ERRORS cho node " + ds_node(ns));
        }
      } else if (c == 'r') {
        mon.hoi_tat_ca(now);
      }
    }
  }
}

int main(int argc, char** argv) {
  cli::Args ap("Giai ma loi CAN firmware Can-sync (ODrive v3.6)");
  ap.them("interface", "waveshare", "waveshare | socketcan");
  ap.them("channel", "", "cong / interface ('' = tu do Waveshare)");
  ap.them("bitrate", "250000", "bitrate CAN");
  ap.them("baud-serial", "2000000", "baud serial Waveshare");
  ap.them("nodes", "0,2", "node theo doi, vd 0,2 hoac all");
  ap.them("names", "0=TRAI,2=PHAI", "ten node");
  ap.them("log", "", "file CSV");
  ap.co("clear", "gui CLEAR_ERRORS luc bat dau");
  ap.co("thu-dong", "chi nghe, khong gui RTR / clear");
  ap.them("replay", "", "file log candump");
  ap.doc(argc, argv);

  try {
    std::optional<std::set<int>> nodes;
    if (ap.s("nodes") != "all") {
      nodes = std::set<int>{};
      for (const auto& [n, _] : parse_names(ap.s("nodes"))) nodes->insert(n);
    }
    auto names = parse_names(ap.s("names"));
    std::unique_ptr<std::ofstream> csv;
    if (!ap.s("log").empty()) {
      csv = std::make_unique<std::ofstream>(ap.s("log"));
      if (!*csv) throw FatalError("khong ghi duoc " + ap.s("log"));
    }
    if (!ap.s("replay").empty()) {
      Monitor mon(nodes, names, nullptr, csv.get(), false);
      chay_replay(ap.s("replay"), mon);
      mon.tong_ket();
    } else {
      cli::bat_tin_hieu();
      auto bus = mo_bus(ap.s("interface"), ap.s("channel"), ap.i("baud-serial"), ap.i("bitrate"));
      GuiFn gui;
      if (!ap.b("thu-dong"))
        gui = [&bus](uint32_t arb, bool rtr) {
          CanFrame f;
          f.id = arb;
          f.rtr = rtr;
          bus->send(f);
        };
      Monitor mon(nodes, names, gui, csv.get());
      chay_that(ap, mon, *bus);
      bus->shutdown();
      mon.tong_ket();
    }
    if (csv) {
      csv->close();
      std::printf("CSV: %s\n", ap.s("log").c_str());
    }
    return 0;
  } catch (const FatalError& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
}
