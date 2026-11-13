// protocol.cpp - ban C++ cua protocol.py (xem protocol.hpp).
#include "zltech_can_driver_cpp/protocol.hpp"

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdarg>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <thread>

namespace fs = std::filesystem;

namespace zltech {

// ============================================================================================================
// Tien ich
// ============================================================================================================
double mono() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void ngu_s(double s) {
  if (s > 0) std::this_thread::sleep_for(std::chrono::duration<double>(s));
}

std::string strf(const char* fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  va_list ap2;
  va_copy(ap2, ap);
  int n = std::vsnprintf(nullptr, 0, fmt, ap);
  va_end(ap);
  std::string s(n > 0 ? n : 0, '\0');
  if (n > 0) std::vsnprintf(s.data(), n + 1, fmt, ap2);
  va_end(ap2);
  return s;
}

std::string noi(const std::vector<std::string>& v, const std::string& sep) {
  std::string s;
  for (size_t i = 0; i < v.size(); ++i) s += (i ? sep : "") + v[i];
  return s;
}

std::string so_state(int state) { return state == CHUA_CO_STATE ? "None" : std::to_string(state); }

std::string ten_state(int state) {
  static const std::map<int, const char*> ten = {{1, "IDLE"},           {3, "FULL_CALIB"},    {4, "MOTOR_CALIB"},
                                                 {7, "ENCODER_OFFSET"}, {8, "CLOSED_LOOP"},   {12, "HALL_POLARITY"},
                                                 {13, "HALL_PHASE"}};
  auto it = ten.find(state);
  return it != ten.end() ? it->second : so_state(state);
}

std::vector<uint8_t> pack_ff(float a, float b) {
  std::vector<uint8_t> v(8);
  std::memcpy(v.data(), &a, 4);
  std::memcpy(v.data() + 4, &b, 4);
  return v;
}

std::vector<uint8_t> pack_i(int32_t a) {
  std::vector<uint8_t> v(4);
  std::memcpy(v.data(), &a, 4);
  return v;
}

std::vector<uint8_t> pack_ii(int32_t a, int32_t b) {
  std::vector<uint8_t> v(8);
  std::memcpy(v.data(), &a, 4);
  std::memcpy(v.data() + 4, &b, 4);
  return v;
}

// ============================================================================================================
// Ma loi
// ============================================================================================================
const std::vector<DetailFrame> DETAIL_FRAMES = {
    {0x03, "motor", &err::MOTOR_ERRORS, 8},
    {0x04, "encoder", &err::ENCODER_ERRORS, 4},
    {0x1D, "controller", &err::CONTROLLER_ERRORS, 4},
};

std::string ten_loi(uint64_t value, const err::Table& table) {
  std::vector<std::string> names;
  uint64_t biet = 0;
  for (const auto& [bit, info] : table) {
    biet |= bit;
    if (value & bit) names.push_back(info.name);
  }
  if (uint64_t la = value & ~biet) names.push_back(strf("0x%llX", static_cast<unsigned long long>(la)));
  return noi(names, "|");
}

std::string mo_ta_loi(uint32_t axis_error, const Details& d) {
  if (!axis_error && !d.any()) return "";
  std::vector<std::string> parts;
  if (axis_error) parts.push_back(ten_loi(axis_error, err::AXIS_ERRORS));
  const uint64_t vals[] = {d.motor, d.encoder, d.controller};
  for (size_t i = 0; i < DETAIL_FRAMES.size(); ++i)
    if (vals[i]) parts.push_back(std::string(DETAIL_FRAMES[i].name) + ": " + ten_loi(vals[i], *DETAIL_FRAMES[i].table));
  return noi(parts, "; ");
}

// ============================================================================================================
// Serial
// ============================================================================================================
static speed_t toc_do_baud(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    default: throw FatalError(strf("baud serial %d khong ho tro", baud));
  }
}

static void cau_hinh_serial(int fd, int baud) {
  termios t{};
  if (tcgetattr(fd, &t) != 0) throw FatalError(strf("tcgetattr loi: %s", std::strerror(errno)));
  cfmakeraw(&t);
  t.c_cflag |= CLOCAL | CREAD;
  t.c_cflag &= ~CRTSCTS;
  t.c_iflag &= ~(IXON | IXOFF | IXANY);
  t.c_cc[VMIN] = 0;
  t.c_cc[VTIME] = 0;
  speed_t sp = toc_do_baud(baud);
  cfsetispeed(&t, sp);
  cfsetospeed(&t, sp);
  if (tcsetattr(fd, TCSANOW, &t) != 0) throw FatalError(strf("tcsetattr loi: %s", std::strerror(errno)));
}

// true neu cong dang bi WaveshareBus khac giu (flock). KHONG doc cong do (doc trom -> chuong trinh kia mat ACK).
static bool dang_bi_giu(const std::string& dev) {
  int fd = ::open(dev.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) return false;
  bool giu = false;
  if (flock(fd, LOCK_EX | LOCK_NB) == 0)
    flock(fd, LOCK_UN);
  else
    giu = errno == EWOULDBLOCK;
  ::close(fd);
  return giu;
}

// "can_bridge (pid 123)" cho cong dev, doc tu /proc; "" neu khong tim duoc.
static std::string ai_giu(const std::string& dev) {
  std::error_code ec;
  fs::path that = fs::canonical(dev, ec);
  if (ec) return "";
  for (const auto& p : fs::directory_iterator("/proc", ec)) {
    std::string pid = p.path().filename();
    if (pid.empty() || !std::all_of(pid.begin(), pid.end(), ::isdigit)) continue;
    std::error_code ec2;
    for (const auto& fd : fs::directory_iterator(p.path() / "fd", ec2)) {
      std::error_code ec3;
      if (fs::read_symlink(fd.path(), ec3) != that) continue;
      std::ifstream f(p.path() / "cmdline");
      std::string all((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
      std::vector<std::string> args;
      size_t s = 0;
      for (size_t i = 0; i <= all.size(); ++i)
        if (i == all.size() || all[i] == '\0') {
          if (i > s) args.push_back(all.substr(s, i - s));
          s = i + 1;
        }
      std::string ten = args.empty() ? "?" : fs::path(args[0]).filename().string();
      for (const auto& a : args)
        if (a.find("zltech_can_driver") != std::string::npos ||
            (a.size() > 3 && a.compare(a.size() - 3, 3, ".py") == 0)) {
          ten = fs::path(a).filename().string();
          break;
        }
      return ten + " (pid " + pid + ")";
    }
  }
  return "";
}

// CHI DOC (khong ghi gi) dev trong thoi_gian s, dem goi Waveshare hop le: AA, info (0xC0|dlc), id 2 byte, data, 55.
static int dem_frame_waveshare(const std::string& dev, int baud, double thoi_gian = 0.3) {
  int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) return 0;
  std::vector<uint8_t> d;
  try {
    cau_hinh_serial(fd, baud);
    tcflush(fd, TCIFLUSH);
    ngu_s(thoi_gian);
    uint8_t tmp[4096];
    ssize_t n;
    while ((n = ::read(fd, tmp, sizeof(tmp))) > 0) d.insert(d.end(), tmp, tmp + n);
  } catch (const FatalError&) {
  }
  ::close(fd);
  int dem = 0;
  for (size_t i = 0; i + 4 < d.size(); ++i)
    if (d[i] == 0xAA && (d[i + 1] & 0xF0) == 0xC0) {
      size_t j = i + 4 + (d[i + 1] & 0x0F);
      dem += j < d.size() && d[j] == 0x55;
    }
  return dem;
}

struct CongUsb {
  std::string dev, mo_ta;
  int vid = -1, pid = -1;
};

// Liet ke cong tty USB (ttyUSB*, ttyACM*...) kem VID:PID doc tu sysfs (giong serial.tools.list_ports).
static std::vector<CongUsb> liet_ke_cong() {
  std::vector<CongUsb> out;
  std::error_code ec;
  for (const auto& e : fs::directory_iterator("/sys/class/tty", ec)) {
    fs::path devlink = e.path() / "device";
    std::error_code ec2;
    if (!fs::exists(devlink, ec2)) continue;
    fs::path p = fs::canonical(devlink, ec2);
    if (ec2) continue;
    CongUsb c;
    c.dev = "/dev/" + e.path().filename().string();
    for (; p.has_parent_path() && p != "/sys" && p != "/"; p = p.parent_path()) {
      if (!fs::exists(p / "idVendor", ec2)) continue;
      auto doc = [&](const char* ten) {
        std::ifstream f(p / ten);
        std::string s;
        std::getline(f, s);
        return s;
      };
      try {
        c.vid = std::stoi(doc("idVendor"), nullptr, 16);
        c.pid = std::stoi(doc("idProduct"), nullptr, 16);
      } catch (const std::exception&) {
      }
      c.mo_ta = doc("product");
      break;
    }
    if (c.vid >= 0) out.push_back(c);
  }
  std::sort(out.begin(), out.end(), [](const CongUsb& a, const CongUsb& b) { return a.dev < b.dev; });
  return out;
}

std::string find_adapter_port(int baud) {
  // May co NHIEU CH340 (vd IMU HWT901 cung CH340, 3/11) -> chi doc thu dong tung cong, chon cong co frame Waveshare.
  // KHONG ghi gi vao cong khi do (ghi nham vao IMU / mach khac la nguy hiem).
  auto cong = liet_ke_cong();
  std::vector<std::string> ch340;
  for (const auto& c : cong)
    if (c.vid == 0x1A86 && c.pid == 0x7523) ch340.push_back(c.dev);
  if (ch340.size() == 1) return ch340[0];
  if (ch340.empty()) {
    std::vector<std::string> ds;
    for (const auto& c : cong) ds.push_back(c.dev + " (" + c.mo_ta + ")");
    throw FatalError("Khong thay adapter CH340. Cac cong USB hien co: " + (ds.empty() ? "(khong co)" : noi(ds, ", ")));
  }
  std::vector<std::string> bi_giu;
  for (const auto& dev : ch340)
    if (dang_bi_giu(dev)) {
      std::string ai = ai_giu(dev);
      bi_giu.push_back(dev + " dang bi " + (ai.empty() ? "chuong trinh khac" : ai) + " giu");
    }
  if (!bi_giu.empty())
    throw FatalError("!! " + noi(bi_giu, ", ") +
                     " -> Waveshare dang duoc dung. Tat no truoc (Ctrl+C o terminal bridge / navigation).");
  std::map<std::string, int> dem;
  std::vector<std::string> co_frame, mo_ta;
  bool co_gi = false;
  for (const auto& dev : ch340) {
    dem[dev] = dem_frame_waveshare(dev, baud);
    if (dem[dev] >= 3) co_frame.push_back(dev);
    co_gi = co_gi || dem[dev];
    mo_ta.push_back(strf("%s: %d frame CAN", dev.c_str(), dem[dev]));
  }
  if (co_frame.size() == 1) return co_frame[0];
  std::string goi_y = co_gi ? "" : " Ca 2 cong deu im lang -> cac board CAN dang TAT NGUON / tuot day CAN?";
  throw FatalError(strf("Co %zu cong CH340 (", ch340.size()) + noi(mo_ta, ", ") +
                   ") -> khong xac dinh duoc Waveshare." + goi_y +
                   " Hoac chi dinh cong: port (yaml / launch) / --cong, nen dung /dev/serial/by-path/... "
                   "(co dinh theo lo USB)");
}

// ============================================================================================================
// Waveshare USB-CAN-A
// ============================================================================================================
static uint8_t ma_bitrate(int bitrate) {
  static const std::map<int, uint8_t> m = {{1000000, 0x01}, {800000, 0x02}, {500000, 0x03}, {400000, 0x04},
                                           {250000, 0x05},  {200000, 0x06}, {125000, 0x07}, {100000, 0x08},
                                           {50000, 0x09},   {20000, 0x0A},  {10000, 0x0B},  {5000, 0x0C}};
  auto it = m.find(bitrate);
  if (it == m.end()) throw FatalError(strf("bitrate %d khong ho tro", bitrate));
  return it->second;
}

WaveshareBus::WaveshareBus(const std::string& channel, int bitrate, int baudrate, uint32_t loc_id, uint32_t loc_mask,
                           bool tu_gui_lai, double gian_cach)
    : channel_(channel), gian_cach_(gian_cach) {
  if (channel.empty()) throw FatalError("Chua chi dinh cong serial");
  fd_ = ::open(channel.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) throw FatalError(strf("Khong mo duoc %s: %s", channel.c_str(), std::strerror(errno)));
  // Khoa doc quyen cong TRUOC khi dat baud: Linux cho 2 chuong trinh mo chung 1 tty, moi ben chi doc duoc 1 phan
  // frame -> mat ACK. Dung chung kieu khoa voi ban Python -> 2 ban cung khong mo trung cong.
  if (flock(fd_, LOCK_EX | LOCK_NB) != 0) {
    ::close(fd_);
    fd_ = -1;
    throw FatalError("!! " + channel + " dang bi chuong trinh khac giu (can_bridge / drive_test / calib?)." +
                     " Tat no truoc. Xem: fuser -v " + channel);
  }
  try {
    cau_hinh_serial(fd_, baudrate);
    // Giong SeeedBus.init_frame, khac: id/mask loc little-endian va byte tu gui lai = 0x00.
    std::vector<uint8_t> m = {0xAA, 0x55, 0x12, ma_bitrate(bitrate), 0x01 /* STD */};
    for (int i = 0; i < 4; ++i) m.push_back((loc_id >> (8 * i)) & 0xFF);
    for (int i = 0; i < 4; ++i) m.push_back((loc_mask >> (8 * i)) & 0xFF);
    m.insert(m.end(), {0x00 /* normal */, static_cast<uint8_t>(tu_gui_lai ? 0x00 : 0x01), 0, 0, 0, 0});
    unsigned sum = 0;
    for (size_t i = 2; i < m.size(); ++i) sum += m[i];
    m.push_back(sum & 0xFF);
    ghi_het(m.data(), m.size());
  } catch (const CanError& e) {
    shutdown();
    throw FatalError(std::string("could not send init frame: ") + e.what());
  } catch (...) {
    shutdown();
    throw;
  }
}

WaveshareBus::~WaveshareBus() { shutdown(); }

void WaveshareBus::shutdown() {
  if (fd_ >= 0) {
    ::close(fd_);  // dong fd -> nha flock
    fd_ = -1;
  }
}

void WaveshareBus::ghi_het(const uint8_t* p, size_t n) {
  while (n) {
    if (fd_ < 0) throw CanError("writing to closed port");
    ssize_t w = ::write(fd_, p, n);
    if (w > 0) {
      p += w;
      n -= static_cast<size_t>(w);
    } else if (w < 0 && (errno == EAGAIN || errno == EINTR)) {
      pollfd pf{fd_, POLLOUT, 0};
      ::poll(&pf, 1, 100);
    } else {
      throw CanError(strf("ghi serial loi: %s", std::strerror(errno)));
    }
  }
}

void WaveshareBus::send(const CanFrame& f) {
  std::lock_guard<std::mutex> lk(khoa_gui_);
  ngu_s(t_gui_ + gian_cach_ - mono());
  std::vector<uint8_t> m = {0xAA, static_cast<uint8_t>(0xC0 | (f.ext ? 0x20 : 0) | (f.rtr ? 0x10 : 0) | f.dlc)};
  for (int i = 0; i < (f.ext ? 4 : 2); ++i) m.push_back((f.id >> (8 * i)) & 0xFF);
  if (!f.rtr) m.insert(m.end(), f.data, f.data + f.dlc);
  m.push_back(0x55);
  ghi_het(m.data(), m.size());
  t_gui_ = mono();
}

// Tach 1 goi tu bo dem: AA | info (0xC0 | ext<<5 | rtr<<4 | dlc) | id 2/4 byte LE | data | 55.
// Goi trang thai AA 55 ... (20 byte) bo qua. Goi hong -> bo 1 byte, nem CanError (luong nhan dem loi).
bool WaveshareBus::tach_frame(CanFrame& f) {
  while (true) {
    auto it = std::find(buf_.begin(), buf_.end(), 0xAA);
    buf_.erase(buf_.begin(), it);
    if (buf_.size() < 2) return false;
    uint8_t info = buf_[1];
    if (info == 0x55) {
      if (buf_.size() < 20) return false;
      buf_.erase(buf_.begin(), buf_.begin() + 20);
      continue;
    }
    size_t dlc = info & 0x0F;
    bool ext = info & 0x20, rtr = info & 0x10;
    if ((info & 0xC0) != 0xC0 || dlc > 8) {
      buf_.erase(buf_.begin());
      throw CanError(strf("goi serial hong (info 0x%02X)", info));
    }
    size_t idlen = ext ? 4 : 2, dlen = dlc, tong = 2 + idlen + dlen + 1;  // nhu SeeedBus: luon doc dlc byte
    if (buf_.size() < tong) return false;
    if (buf_[tong - 1] != 0x55) {
      buf_.erase(buf_.begin());
      throw CanError("goi serial hong (sai byte ket thuc)");
    }
    f = CanFrame{};
    for (size_t i = 0; i < idlen; ++i) f.id |= static_cast<uint32_t>(buf_[2 + i]) << (8 * i);
    f.ext = ext;
    f.rtr = rtr;
    f.dlc = static_cast<uint8_t>(dlc);
    std::copy(buf_.begin() + 2 + idlen, buf_.begin() + 2 + idlen + dlen, f.data);
    buf_.erase(buf_.begin(), buf_.begin() + tong);
    return true;
  }
}

bool WaveshareBus::recv(CanFrame& f, double timeout) {
  double han = mono() + timeout;
  while (true) {
    if (tach_frame(f)) return true;
    double con = han - mono();
    if (con <= 0) return false;
    if (fd_ < 0) throw CanError("reading from closed port");
    pollfd pf{fd_, POLLIN, 0};
    int r = ::poll(&pf, 1, static_cast<int>(con * 1000) + 1);
    if (r < 0) {
      if (errno == EINTR) continue;
      throw CanError(strf("poll serial loi: %s", std::strerror(errno)));
    }
    if (r == 0) continue;
    if (pf.revents & (POLLERR | POLLHUP | POLLNVAL)) throw CanError("cong serial loi / bi rut");
    uint8_t tmp[1024];
    ssize_t n = ::read(fd_, tmp, sizeof(tmp));
    if (n > 0)
      buf_.insert(buf_.end(), tmp, tmp + n);
    else if (n == 0 || (errno != EAGAIN && errno != EINTR))
      throw CanError("doc serial loi / cong bi rut");
  }
}

// ============================================================================================================
// SocketCAN
// ============================================================================================================
SocketCanBus::SocketCanBus(const std::string& ifname) : ifname_(ifname) {
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) throw FatalError(strf("socket CAN loi: %s", std::strerror(errno)));
  ifreq ifr{};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    ::close(fd_);
    throw FatalError(strf("khong thay interface %s (ip link set %s up type can bitrate 250000?)", ifname.c_str(),
                          ifname.c_str()));
  }
  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(fd_);
    throw FatalError(strf("bind %s loi: %s", ifname.c_str(), std::strerror(errno)));
  }
}

SocketCanBus::~SocketCanBus() { shutdown(); }

void SocketCanBus::shutdown() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void SocketCanBus::send(const CanFrame& f) {
  can_frame cf{};
  cf.can_id = f.id | (f.ext ? CAN_EFF_FLAG : 0) | (f.rtr ? CAN_RTR_FLAG : 0);
  cf.can_dlc = f.dlc;
  std::memcpy(cf.data, f.data, f.dlc);
  if (::write(fd_, &cf, sizeof(cf)) != static_cast<ssize_t>(sizeof(cf)))
    throw CanError(strf("gui CAN loi: %s", std::strerror(errno)));
}

bool SocketCanBus::recv(CanFrame& f, double timeout) {
  pollfd pf{fd_, POLLIN, 0};
  int r = ::poll(&pf, 1, static_cast<int>(timeout * 1000));
  if (r < 0 && errno != EINTR) throw CanError(strf("poll CAN loi: %s", std::strerror(errno)));
  if (r <= 0) return false;
  can_frame cf{};
  if (::read(fd_, &cf, sizeof(cf)) != static_cast<ssize_t>(sizeof(cf))) throw CanError("doc CAN loi");
  if (cf.can_id & CAN_ERR_FLAG) return false;
  f = CanFrame{};
  f.ext = cf.can_id & CAN_EFF_FLAG;
  f.rtr = cf.can_id & CAN_RTR_FLAG;
  f.id = cf.can_id & (f.ext ? CAN_EFF_MASK : CAN_SFF_MASK);
  f.dlc = std::min<uint8_t>(cf.can_dlc, 8);
  std::memcpy(f.data, cf.data, f.dlc);
  return true;
}

std::unique_ptr<CanBus> mo_bus(const std::string& adapter, const std::string& port, int baud_serial, int bitrate) {
  if (adapter == "waveshare")
    return std::make_unique<WaveshareBus>(port.empty() ? find_adapter_port(baud_serial) : port, bitrate, baud_serial);
  if (adapter == "socketcan") return std::make_unique<SocketCanBus>(port.empty() ? "can0" : port);
  if (adapter == "gs_usb" || adapter == "slcan")
    throw FatalError("adapter '" + adapter + "': ban C++ dung qua SocketCAN -> " +
                     (adapter == "gs_usb" ? "cam adapter (driver kernel gs_usb tao can0)"
                                          : "chay 'sudo slcand -o -s5 /dev/ttyACM0 slcan0'") +
                     ", 'sudo ip link set <if> up' roi chon adapter socketcan");
  throw FatalError("adapter '" + adapter + "' khong ho tro (waveshare | socketcan)");
}

void bo_qua_tin_hieu() {
  // Khong co: Ctrl+C lan 2 / tin hieu den tre ngat giua chung -> banh khong ve IDLE.
  for (int sig : {SIGINT, SIGTERM, SIGHUP}) std::signal(sig, SIG_IGN);
}

// ============================================================================================================
// CanLink
// ============================================================================================================
CanLink::CanLink(std::vector<int> nodes, const std::string& adapter, const std::string& port, int baud_serial)
    : nodes_(std::move(nodes)) {
  for (int n : nodes_) tt_[n] = std::make_unique<NodeState>();
  bus_ = mo_bus(adapter, port, baud_serial);
  kiem_luong_nhan();
}

CanLink::~CanLink() { dong(); }

void CanLink::luong_nhan() {
  try {
    CanFrame m;
    while (!dung_) {
      bool co;
      try {
        co = bus_->recv(m, 0.1);
      } catch (const CanError& e) {
        // Goi serial hong (nhieu / lech khung). Khong bat thi luong nhan chet -> khong ai doc serial -> adapter nghen.
        {
          std::lock_guard<std::mutex> lk(loi_mtx_);
          ++so_loi_nhan_;
          loi_nhan_cuoi_ = std::string("CanError: ") + e.what();
        }
        ngu_s(0.005);  // cong bi rut -> khong quay vong 100 % CPU
        continue;
      }
      if (!co || m.ext || m.rtr) continue;
      int node = static_cast<int>(m.id >> 5), cmd = static_cast<int>(m.id & 0x1F);
      auto it = tt_.find(node);
      if (it == tt_.end()) continue;
      NodeState& s = *it->second;
      const double now = mono();
      s.t_rx = now;
      if (cmd == CMD_HEARTBEAT && m.dlc >= 4) {
        s.axis_error = m.data[0] | (m.data[1] << 8) | (m.data[2] << 16);
        s.state = m.data[3];
        const double t_hb_cu = s.t_hb.load();
        if (t_hb_cu > 0.0 && now - t_hb_cu > s.hb_gap_max.load()) s.hb_gap_max = now - t_hb_cu;
        s.t_hb = now;
        ++s.so_hb;
      } else if (cmd == CMD_ENCODER_EST && m.dlc >= 8) {
        float pos, vel;
        std::memcpy(&pos, m.data, 4);
        std::memcpy(&vel, m.data + 4, 4);
        s.pos = pos;
        s.vel = vel;
        ++s.so_enc;
      } else if (cmd == CMD_STAGED && m.dlc == 1 && m.data[0] == ACK_STAGED) {
        {
          std::lock_guard<std::mutex> lk(ack_mtx_);
          s.ack = true;
        }
        ack_cv_.notify_all();
      } else {
        for (const auto& d : DETAIL_FRAMES) {
          if (d.cmd != cmd) continue;
          uint64_t v = 0;
          for (int i = 0; i < d.nbytes && i < m.dlc; ++i) v |= static_cast<uint64_t>(m.data[i]) << (8 * i);
          (d.cmd == 0x03 ? s.motor : d.cmd == 0x04 ? s.encoder : s.controller) = v;
        }
      }
    }
  } catch (const std::exception& e) {
    std::lock_guard<std::mutex> lk(loi_mtx_);
    loi_nhan_cuoi_ = std::string("luong nhan chet: ") + e.what();
  }
  nhan_song_ = false;
}

bool CanLink::kiem_luong_nhan() {
  if (nhan_song_ || dung_) return false;
  bool chet = nhan_.joinable();
  if (chet) nhan_.join();
  nhan_song_ = true;
  nhan_ = std::thread(&CanLink::luong_nhan, this);
  return chet;
}

int CanLink::lay_loi_nhan(std::string& cuoi) {
  std::lock_guard<std::mutex> lk(loi_mtx_);
  int n = so_loi_nhan_;
  so_loi_nhan_ = 0;
  cuoi = loi_nhan_cuoi_;
  return n;
}

bool CanLink::cho_heartbeat(double timeout, std::vector<int> nodes) {
  const auto& ns = chon(nodes);
  auto du = [&] {
    for (int n : ns)
      if (!tt(n).t_hb) return false;
    return true;
  };
  double t0 = mono();
  while (mono() - t0 < timeout) {
    if (du()) return true;
    ngu_s(0.05);
  }
  return du();
}

bool CanLink::con_song(int n, double rx_timeout, double hb_timeout) {
  double now = mono();
  return now - tt(n).t_rx < rx_timeout && now - tt(n).t_hb < hb_timeout;
}

void CanLink::gui(uint32_t arb_id, const std::vector<uint8_t>& data, bool rtr) {
  CanFrame f;
  f.id = arb_id;
  f.rtr = rtr;
  f.dlc = static_cast<uint8_t>(std::min<size_t>(data.size(), 8));
  std::copy(data.begin(), data.begin() + f.dlc, f.data);
  bus_->send(f);
}

void CanLink::gui_node(int n, int cmd, const std::vector<uint8_t>& data) { gui(err::arb_id(n, cmd), data); }

void CanLink::dat_state(int state, std::vector<int> nodes) {
  for (int n : chon(nodes)) gui_node(n, CMD_SET_STATE, pack_i(state));
}

void CanLink::dat_input_mode(int input_mode, std::vector<int> nodes) {
  for (int n : chon(nodes)) gui_node(n, CMD_SET_MODES, pack_ii(VELOCITY_CONTROL, input_mode));
}

void CanLink::van_toc_0(std::vector<int> nodes) {
  for (int n : chon(nodes)) gui_node(n, CMD_SET_INPUT_VEL, pack_ff(0.0f, 0.0f));
}

void CanLink::xoa_loi(std::vector<int> nodes) {
  for (int n : chon(nodes)) {
    gui_node(n, CMD_CLEAR_ERRORS);
    tt(n).motor = 0;
    tt(n).encoder = 0;
    tt(n).controller = 0;
    tt(n).t_hb = 0.0;
  }
}

std::vector<int> CanLink::gui_dong_bo(const VongMap& vong, double ack_timeout, double tre_go) {
  std::vector<int> cho;
  for (const auto& kv : vong) cho.push_back(kv.first);
  for (int lan = 0; lan < 2; ++lan) {
    for (int n : cho) {
      tt(n).ack = false;
      gui_node(n, CMD_STAGED, pack_ff(0.0f, static_cast<float>(vong.at(n))));
    }
    auto han = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                      std::chrono::duration<double>(ack_timeout));
    {
      std::unique_lock<std::mutex> lk(ack_mtx_);
      ack_cv_.wait_until(lk, han, [&] {
        for (int n : cho)
          if (!tt(n).ack) return false;
        return true;
      });
    }
    std::vector<int> thieu;
    for (int n : cho)
      if (!tt(n).ack) thieu.push_back(n);
    cho = thieu;
    if (cho.empty()) break;
  }
  ngu_s(tre_go);
  gui(ID_SYNC_GO, {0x00});
  return cho;
}

void CanLink::dua_ve_idle() {
  VongMap khong;
  for (int n : nodes_) khong[n] = 0.0;
  gui_dong_bo(khong);
  dat_input_mode(VEL_RAMP);
  dat_state(IDLE);
  ngu_s(0.05);
  dat_state(IDLE);
}

void CanLink::dong() {
  if (da_dong_) return;
  da_dong_ = true;
  dung_ = true;
  if (nhan_.joinable()) nhan_.join();
  if (bus_) bus_->shutdown();
}

}  // namespace zltech
