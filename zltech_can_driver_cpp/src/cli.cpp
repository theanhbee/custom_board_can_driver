// cli.cpp - xem cli.hpp.
#include "zltech_can_driver_cpp/cli.hpp"

#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <csignal>
#include <cstdio>
#include <cstdlib>

#include "zltech_can_driver_cpp/protocol.hpp"

namespace zltech::cli {

std::atomic<bool> g_dung{false};

static void bat(int) { g_dung = true; }

void bat_tin_hieu() {
  for (int sig : {SIGINT, SIGTERM, SIGHUP}) std::signal(sig, bat);
}

void ngu(double s) {
  double han = mono() + s;
  while (true) {
    if (g_dung) throw NgatCtrlC{};
    double con = han - mono();
    if (con <= 0) return;
    ngu_s(std::min(con, 0.01));
  }
}

// ---------------------------------------------------------------- Args
void Args::them(const std::string& ten, const std::string& mac_dinh, const std::string& help,
                const std::vector<std::string>& chon) {
  thu_tu_.push_back(ten);
  opt_[ten] = Opt{mac_dinh, help, mac_dinh, chon};
}

void Args::co(const std::string& ten, const std::string& help) {
  thu_tu_.push_back(ten);
  Opt o;
  o.help = help;
  o.la_co = true;
  opt_[ten] = o;
}

void Args::loi(const std::string& msg) const {
  std::fprintf(stderr, "%s: loi: %s (xem --help)\n", prog_.c_str(), msg.c_str());
  std::exit(2);
}

void Args::doc(int argc, char** argv) {
  prog_ = argc ? argv[0] : "tool";
  for (int k = 1; k < argc; ++k) {
    std::string a = argv[k];
    if (a == "--ros-args") break;  // ros2 run them vao -> bo qua phan con lai
    if (a == "-h" || a == "--help") {
      std::printf("%s\n\nTham so:\n", mo_ta_.c_str());
      for (const auto& ten : thu_tu_) {
        const Opt& o = opt_.at(ten);
        std::printf("  --%-14s %s%s\n", ten.c_str(), o.help.c_str(),
                    o.la_co ? "" : (" (mac dinh '" + o.mac_dinh + "')").c_str());
      }
      std::exit(0);
    }
    if (a.rfind("--", 0) != 0) loi("tham so la: " + a);
    std::string ten = a.substr(2), gia_tri;
    bool co_bang = false;
    if (auto eq = ten.find('='); eq != std::string::npos) {
      gia_tri = ten.substr(eq + 1);
      ten = ten.substr(0, eq);
      co_bang = true;
    }
    std::replace(ten.begin(), ten.end(), '_', '-');
    auto it = opt_.find(ten);
    if (it == opt_.end()) loi("khong co tham so --" + ten);
    Opt& o = it->second;
    if (o.la_co) {
      if (co_bang) loi("--" + ten + " khong nhan gia tri");
      o.da_dat = true;
      continue;
    }
    if (!co_bang) {
      if (k + 1 >= argc) loi("--" + ten + " thieu gia tri");
      gia_tri = argv[++k];
    }
    if (!o.chon.empty() && std::find(o.chon.begin(), o.chon.end(), gia_tri) == o.chon.end())
      loi("--" + ten + " phai la mot trong: " + noi(o.chon, ", "));
    o.gia_tri = gia_tri;
    o.da_dat = true;
  }
}

std::string Args::s(const std::string& ten) const { return opt_.at(ten).gia_tri; }

int Args::i(const std::string& ten) const {
  try {
    return std::stoi(s(ten));
  } catch (const std::exception&) {
    loi("--" + ten + " phai la so nguyen");
  }
}

double Args::d(const std::string& ten) const {
  try {
    return std::stod(s(ten));
  } catch (const std::exception&) {
    loi("--" + ten + " phai la so");
  }
}

bool Args::b(const std::string& ten) const { return opt_.at(ten).da_dat; }

Args& them_tham_so_can(Args& ap) {
  ap.them("trai", "0", "node_id banh trai");
  ap.them("phai", "2", "node_id banh phai");
  ap.them("adapter", "waveshare", "waveshare | socketcan");
  ap.them("cong", "", "cong / interface; '' = tu do (Waveshare CH340)");
  ap.them("baud-serial", "2000000", "baud serial Waveshare");
  return ap;
}

std::map<int, std::string> ten_banh(const Args& a) {
  if (a.i("trai") == a.i("phai")) throw FatalError("!! --trai va --phai phai khac nhau");
  return {{a.i("trai"), "TRAI"}, {a.i("phai"), "PHAI"}};
}

std::vector<int> chon_banh(const Args& a, const std::map<int, std::string>& ten) {
  std::string chi = a.s("chi");
  std::transform(chi.begin(), chi.end(), chi.begin(), ::toupper);
  std::vector<int> nodes;
  for (int n : {a.i("trai"), a.i("phai")})
    if (chi.empty() || ten.at(n) == chi) nodes.push_back(n);
  if (nodes.empty()) throw FatalError("--chi phai la TRAI hoac PHAI");
  return nodes;
}

// ---------------------------------------------------------------- BanPhim
BanPhim::BanPhim() : fd_(STDIN_FILENO) {
  if (isatty(fd_) && tcgetattr(fd_, &cu_) == 0) {
    co_cu_ = true;
    termios t = cu_;
    t.c_lflag &= ~(ICANON | ECHO);  // nhu tty.setcbreak: giu ISIG -> Ctrl+C van la SIGINT
    t.c_cc[VMIN] = 1;
    t.c_cc[VTIME] = 0;
    tcsetattr(fd_, TCSAFLUSH, &t);
  }
}

BanPhim::~BanPhim() {
  if (co_cu_) tcsetattr(fd_, TCSADRAIN, &cu_);
}

bool BanPhim::kbhit() const {
  if (!co_cu_) return false;  // khong phai terminal (stdin = /dev/null...) -> khong co phim
  pollfd pf{fd_, POLLIN, 0};
  return ::poll(&pf, 1, 0) > 0 && (pf.revents & POLLIN);
}

char BanPhim::getch() const {
  char c = 0;
  return ::read(fd_, &c, 1) == 1 ? c : 0;
}

}  // namespace zltech::cli
