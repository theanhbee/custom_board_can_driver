// protocol.hpp - giao thuc CAN firmware Can-sync (ODrive v3.6) + adapter Waveshare USB-CAN-A + lop CanLink dung chung.
// Ban C++ cua zltech_can_driver/protocol.py.
//
// Dung chung cho can_bridge / calib / clear_errors / drive_test / error_monitor. Khong doc argv, khong tao ROS node.
//
// Arbitration id 11 bit = (node_id << 5) | cmd. Bus 250 kbit/s.
// Board tu gui: heartbeat 0x01 (100 ms: byte 0-2 axis_error, byte 3 state), encoder 0x09 (10 ms: float pos, float vel
// vong/s), frame loi chi tiet 0x03 motor / 0x04 encoder / 0x1D controller khi co loi.
// Dong bo 2 banh: 0x1E Set_Input_Pos_Staged (float pos = 0, float vel) nap vao dem -> board ACK (cmd 0x1E, dlc 1,
// data[0] = 2) -> 0x7FF SYNC_GO data[0] = 0 -> moi board ap cung luc.
// Firmware can-watchdog: vao CLOSED_LOOP qua CAN thi 0.5 s khong co lenh CAN -> board tu cat (axis_error 0x800).
#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "zltech_can_driver_cpp/odrive_can_errors.hpp"

namespace zltech {

constexpr int BITRATE = 250000;

// ---- cmd id (5 bit thap)
constexpr int CMD_HEARTBEAT = 0x01;
constexpr int CMD_SET_STATE = 0x07;
constexpr int CMD_ENCODER_EST = 0x09;
constexpr int CMD_SET_MODES = 0x0B;
constexpr int CMD_SET_INPUT_VEL = 0x0D;
constexpr int CMD_CLEAR_ERRORS = 0x18;
constexpr int CMD_STAGED = 0x1E;
constexpr uint32_t ID_SYNC_GO = 0x7FF;  // (node 0x3F << 5) | 0x1F, broadcast
constexpr int ACK_STAGED = 2;           // data[0] ACK cua ban Can-sync

// ---- state / mode
constexpr int IDLE = 1, FULL_CALIB = 3, MOTOR_CALIB = 4, ENCODER_OFFSET = 7, CLOSED_LOOP = 8;
constexpr int VELOCITY_CONTROL = 2, PASSTHROUGH = 1, VEL_RAMP = 2;
constexpr uint32_t ERR_WATCHDOG = 0x800;  // axis_error WATCHDOG_TIMER_EXPIRED
constexpr int CHUA_CO_STATE = -1;         // chua nhan heartbeat (None ben Python)

// "IDLE", "CLOSED_LOOP"...; state khong co ten -> so; chua co -> "None"
std::string ten_state(int state);
// so state; chua co -> "None"
std::string so_state(int state);

// ---- tien ich
double mono();                                  // giay, dong ho monotonic
void ngu_s(double s);                           // sleep s giay
std::string strf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));
std::string noi(const std::vector<std::string>& v, const std::string& sep);

// Loi nghiem trong -> in thong bao roi thoat (SystemExit ben Python)
struct FatalError : std::runtime_error {
  using std::runtime_error::runtime_error;
};
// Loi gui / nhan CAN (can.CanError ben Python)
struct CanError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

// ============================================================================================================
// Ma loi
// ============================================================================================================
// Bang ma loi day du (ten, y nghia, goi y) nam o odrive_can_errors; o day chi lay ten.
struct Details {
  uint64_t motor = 0, encoder = 0, controller = 0;
  bool any() const { return motor || encoder || controller; }
};
// cmd frame loi chi tiet -> (ten bo phan, bang ma loi, so byte): 0x03 motor 8, 0x04 encoder 4, 0x1D controller 4
struct DetailFrame {
  int cmd;
  const char* name;
  const err::Table* table;
  int nbytes;
};
extern const std::vector<DetailFrame> DETAIL_FRAMES;

// 0x240 -> "MOTOR_FAILED|CONTROLLER_FAILED". Bit khong co trong bang thi in dang hex.
std::string ten_loi(uint64_t value, const err::Table& table);
// Chuoi ngan mo ta loi, "" neu khong loi.
std::string mo_ta_loi(uint32_t axis_error, const Details& d);

// ============================================================================================================
// Bus CAN
// ============================================================================================================
struct CanFrame {
  uint32_t id = 0;
  bool ext = false, rtr = false;
  uint8_t dlc = 0;
  uint8_t data[8] = {};
};

class CanBus {
 public:
  virtual ~CanBus() = default;
  virtual void send(const CanFrame& f) = 0;
  // true neu nhan duoc frame trong timeout s. Frame hong -> CanError.
  virtual bool recv(CanFrame& f, double timeout) = 0;
  virtual std::string channel_info() const = 0;
  virtual void shutdown() {}
};

// Waveshare USB-CAN-A (giao thuc serial cua python-can "seeedstudio").
// Driver goc python-can dat byte "Send once" = 0x01 -> adapter KHONG tu gui lai frame thua arbitration
// -> mat 15-50 % lenh. Ban nay bat tu gui lai va giu cach 2 ms giua 2 frame gui.
constexpr double GIAN_CACH_GUI = 0.002;  // [s]

class WaveshareBus : public CanBus {
 public:
  WaveshareBus(const std::string& channel, int bitrate = BITRATE, int baudrate = 2000000, uint32_t loc_id = 0,
               uint32_t loc_mask = 0, bool tu_gui_lai = true, double gian_cach = GIAN_CACH_GUI);
  ~WaveshareBus() override;
  void send(const CanFrame& f) override;
  bool recv(CanFrame& f, double timeout) override;
  std::string channel_info() const override { return "Serial interface: " + channel_; }
  void shutdown() override;

 private:
  bool tach_frame(CanFrame& f);
  void ghi_het(const uint8_t* p, size_t n);
  std::string channel_;
  int fd_ = -1;
  double gian_cach_, t_gui_ = 0.0;
  std::mutex khoa_gui_;
  std::vector<uint8_t> buf_;
};

class SocketCanBus : public CanBus {
 public:
  explicit SocketCanBus(const std::string& ifname);
  ~SocketCanBus() override;
  void send(const CanFrame& f) override;
  bool recv(CanFrame& f, double timeout) override;
  std::string channel_info() const override { return "socketcan channel '" + ifname_ + "'"; }
  void shutdown() override;

 private:
  std::string ifname_;
  int fd_ = -1;
};

// Tim cong Waveshare USB-CAN-A (chip CH340, VID:PID 1A86:7523). CHI doc thu dong, khong ghi gi khi do.
std::string find_adapter_port(int baud = 2000000);

// adapter: waveshare | socketcan. port "" = mac dinh (Waveshare tu do CH340, socketcan can0).
// gs_usb / slcan: dung driver kernel (gs_usb -> can0; slcand -> slcan0) roi chon socketcan.
std::unique_ptr<CanBus> mo_bus(const std::string& adapter = "waveshare", const std::string& port = "",
                               int baud_serial = 2000000, int bitrate = BITRATE);

// Goi dau khoi don dep: bo qua SIGINT/SIGTERM/SIGHUP de chuoi 've 0 -> IDLE -> dong CAN' chay tron.
void bo_qua_tin_hieu();

// ============================================================================================================
// CanLink: bus + luong nhan + trang thai tung node + lenh
// ============================================================================================================
// Du lieu moi nhat cua 1 node, cap nhat tu luong nhan (atomic: doc tu luong chinh).
struct NodeState {
  std::atomic<double> t_hb{0.0};  // heartbeat cuoi (monotonic)
  std::atomic<double> t_rx{0.0};  // frame bat ky cuoi (encoder 0x09 100 lan/s)
  std::atomic<int> state{CHUA_CO_STATE};
  std::atomic<uint32_t> axis_error{0};
  std::atomic<double> pos{0.0};  // vong, tu 0x09
  std::atomic<double> vel{0.0};  // vong/s, tu 0x09
  std::atomic<uint64_t> motor{0}, encoder{0}, controller{0};
  std::atomic<bool> ack{false};
  // Chan doan (13/11: heartbeat 1 node vang ~1 s trong khi encoder van ve). Doc xong tu dat lai = 0.
  std::atomic<int> so_hb{0};              // so heartbeat nhan
  std::atomic<int> so_enc{0};             // so frame encoder 0x09 nhan (board gui 100/s)
  std::atomic<double> hb_gap_max{0.0};    // khoang trong heartbeat lon nhat [s] (board gui moi 0.1 s)

  Details details() const { return {motor.load(), encoder.load(), controller.load()}; }
  std::string loi() const { return mo_ta_loi(axis_error.load(), details()); }
};

using VongMap = std::map<int, double>;  // node -> vong/s

class CanLink {
 public:
  // nodes: cac node_id can theo doi (vd {0, 2}).
  CanLink(std::vector<int> nodes, const std::string& adapter = "waveshare", const std::string& port = "",
          int baud_serial = 2000000);
  ~CanLink();

  NodeState& tt(int n) { return *tt_.at(n); }
  const std::vector<int>& nodes() const { return nodes_; }
  std::string channel_info() const { return bus_->channel_info(); }

  // Khoi dong (lai) luong nhan neu chua chay / da chet. true neu phai khoi dong lai.
  bool kiem_luong_nhan();
  // so frame hong tu lan doc truoc (doc xong ve 0) + loi cuoi
  int lay_loi_nhan(std::string& cuoi);

  // Cho toi khi moi node co heartbeat. true neu du. nodes rong = tat ca.
  bool cho_heartbeat(double timeout = 1.5, std::vector<int> nodes = {});
  bool con_song(int n, double rx_timeout = 0.3, double hb_timeout = 1.0);

  void gui(uint32_t arb_id, const std::vector<uint8_t>& data = {}, bool rtr = false);
  void gui_node(int n, int cmd, const std::vector<uint8_t>& data = {});
  void dat_state(int state, std::vector<int> nodes = {});
  void dat_input_mode(int input_mode, std::vector<int> nodes = {});
  void van_toc_0(std::vector<int> nodes = {});  // 0x0D Set_Input_Vel = 0 truc tiep (khong qua dem staged)
  void xoa_loi(std::vector<int> nodes = {});    // Clear_Errors, bo heartbeat / frame loi cu. Khong sleep.
  // Nap vao dem CA cac board, cho ACK chung (thieu thi gui lai 1 lan), cho them tre_go s, roi LUON phat SYNC_GO.
  // Tra ve danh sach node thieu ACK.
  std::vector<int> gui_dong_bo(const VongMap& vong, double ack_timeout = 0.02, double tre_go = 0.0);
  void dua_ve_idle();  // toc do 0 (dong bo) -> VEL_RAMP -> IDLE x2. Dung khi thoat / tat.
  void dong();

 private:
  void luong_nhan();
  const std::vector<int>& chon(const std::vector<int>& nodes) const { return nodes.empty() ? nodes_ : nodes; }

  std::vector<int> nodes_;
  std::map<int, std::unique_ptr<NodeState>> tt_;
  std::unique_ptr<CanBus> bus_;
  std::thread nhan_;
  std::atomic<bool> dung_{false}, nhan_song_{false};
  std::mutex ack_mtx_, loi_mtx_;
  std::condition_variable ack_cv_;
  int so_loi_nhan_ = 0;
  std::string loi_nhan_cuoi_;
  bool da_dong_ = false;
};

// little-endian
std::vector<uint8_t> pack_ff(float a, float b);
std::vector<uint8_t> pack_i(int32_t a);
std::vector<uint8_t> pack_ii(int32_t a, int32_t b);

}  // namespace zltech
