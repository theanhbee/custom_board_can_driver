// can_bridge - CAU DRIVER ROS 2: /cmd_vel -> 2 banh vi sai qua CAN (firmware Can-sync, SYNC_GO dong bo 2 banh).
// Ban C++ cua zltech_can_driver/can_bridge.py: cung tham so, topic, service, log va logic an toan.
//
//     ros2 launch zltech_can_driver_cpp can_bridge.launch.py
//     ros2 run zltech_can_driver_cpp can_bridge --ros-args --params-file config/can_bridge_params.yaml
//
// Topic / service (ten topic doi duoc bang tham so cmd_vel_topic / odom_topic):
//     sub  cmd_vel           geometry_msgs/Twist   linear.x [m/s], angular.z [rad/s]. Khong co lenh cmd_timeout s -> 0
//     sub  vel_cmd           geometry_msgs/Twist   GIONG cmd_vel, tat mac dinh (vel_cmd_topic = "").
//     sub  emergency_brake   std_msgs/Empty        PHANH GAP (PASSTHROUGH ve 0, cho dung roi tra VEL_RAMP)
//     pub  wheel_odom        nav_msgs/Odometry     50 Hz, tich phan delta vi tri encoder (0x09); publish_tf -> TF
//     pub  raw_encoder       sensor_msgs/JointState 50 Hz, encoder THO (vong, vong/s, CHUA dao dau invert). "" = tat
//     srv  ~/enable          std_srvs/Trigger      kiem tra heartbeat + vao CLOSED_LOOP
//     srv  ~/disable         std_srvs/Trigger      2 banh ve IDLE
// Mat lien lac < 2 s: van gui lenh, watchdog firmware cat thi tu xoa loi + vao lai CLOSED_LOOP. > 2 s -> IDLE.
// Loi phan cung -> IDLE, KHONG tu enable: sua xong goi ros2 service call /can_bridge/enable std_srvs/srv/Trigger
// Banh chay vuot lenh (runaway) -> IDLE + KHOA, chi ~/enable bang tay moi bo khoa.
// Thoat (Ctrl+C, kill, dong terminal = SIGINT/SIGTERM/SIGHUP): toc do 0 -> VEL_RAMP -> IDLE roi moi dong CAN.
#include <atomic>
#include <cmath>
#include <csignal>
#include <deque>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "zltech_can_driver_cpp/protocol.hpp"

using namespace std::chrono_literals;
using namespace zltech;
using Trigger = std_srvs::srv::Trigger;

constexpr double CHU_KY = 0.05;         // s, gui lai lenh dong bo (an watchdog CAN 0.5 s cua firmware)
constexpr double ACK_TIMEOUT = 0.02;    // s cho ACK ca 2 node (trung vi do duoc ~4 ms)
constexpr int MAT_ACK = 2;              // chu ky thieu ACK lien tiep -> coi nhu mat lien lac
constexpr double MAT_LL_TOI_DA = 2.0;   // s mat lien lac lien tuc -> tat han
// Song / chet xet theo frame encoder 0x09 (100/s, rx 0.3 s) + ACK. Heartbeat (10/s) chi de doc state / loi:
// 13/11 heartbeat 1 board vang ~1.04 s trong khi encoder + ACK van ve binh thuong -> hb_timeout cu 1.0 s bao
// "mat lien lac" sai -> 2 s sau cat IDLE. Chi coi la mat khi heartbeat vang > HB_TIMEOUT.
constexpr double HB_TIMEOUT = 3.0;       // s
constexpr double HB_TRE_CANH_BAO = 0.5;  // s, khoang trong heartbeat lon hon -> ghi canh bao (khong cat)
constexpr double VEL_DUNG = 0.05;       // vong/s, coi nhu da dung (phanh gap)
constexpr double PHANH_TOI_DA = 1.5;    // s
// Chong banh tu chay (runaway): do 3/11 banh PHAI lenh 0 ma encoder do 0 -> -0.85 -> -6 vong/s trong ~2 s
// (sai offset / Hall nhieu -> commutation sai), firmware SPINOUT moi cat. Bridge cat som hon:
constexpr double VUOT_LENH = 0.5;       // vong/s, |do| vuot |lenh lon nhat trong CUA_SO_LENH s| bao nhieu thi la vuot
constexpr double CUA_SO_LENH = 1.0;     // s, firmware ramp 10 vong/s^2 -> giam tu 4 vong/s mat 0.4 s
constexpr double VUOT_TOI_DA = 0.2;     // s vuot lien tuc -> IDLE

static std::atomic<bool> g_thoat{false};
static void bat_tin_hieu(int) { g_thoat = true; }

class CanBridge : public rclcpp::Node {
 public:
  CanBridge() : Node("can_bridge") {
    std::string adapter = par_s("adapter", "waveshare");  // waveshare | socketcan
    std::string port = par_s("port", "");                  // "" = tu do
    int baud_serial = par_i("baud_serial", 2000000);
    left_ = par_i("left_node", 0);
    right_ = par_i("right_node", 2);
    std::string invert = par_s("invert", "PHAI");          // "" | TRAI | PHAI (banh lap nguoc)
    for (auto& c : invert) c = static_cast<char>(toupper(c));
    rong_ = par_d("wheel_separation", 0.382);
    dk_ = par_d("wheel_diameter", 0.139);
    vong_max_ = std::min(par_d("vmax", 4.0), 5.5);         // vong/s moi banh, board vel_limit 5.5
    cmd_timeout_ = par_d("cmd_timeout", 0.5);
    std::string cmd_vel_topic = par_s("cmd_vel_topic", "cmd_vel");
    std::string vel_cmd_topic = par_s("vel_cmd_topic", "");          // topic van toc phu, "" = tat
    std::string odom_topic = par_s("odom_topic", "wheel_odom");
    std::string raw_encoder_topic = par_s("raw_encoder_topic", "raw_encoder");  // encoder tho, "" = tat
    frame_odom_ = par_s("odom_frame", "odom");
    frame_base_ = par_s("base_frame", "base_link");
    bool publish_tf = par_b("publish_tf", false);
    double status_period = par_d("status_period", 1.0);   // s, 0 = tat log trang thai
    if (left_ == right_) throw FatalError("left_node va right_node phai khac nhau");

    nodes_ = {left_, right_};
    ten_ = {{left_, "TRAI"}, {right_, "PHAI"}};
    dau_ = {{left_, invert == "TRAI" ? -1.0 : 1.0}, {right_, invert == "PHAI" ? -1.0 : 1.0}};
    link_ = std::make_unique<CanLink>(nodes_, adapter, port, baud_serial);
    info(strf("CAN mo: %s  TRAI = node %d, PHAI = node %d, dao %s", link_->channel_info().c_str(), left_, right_,
              invert.empty() ? "khong" : invert.c_str()));

    t_thu_enable_ = mono();
    for (int n : nodes_) {
      sai_ack_[n] = 0;
      vong_gui_[n] = 0.0;
      t_vuot_[n].reset();
    }

    auto cb = [this](geometry_msgs::msg::Twist::ConstSharedPtr m) { cb_cmd(*m); };
    subs_.push_back(create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic, 10, cb));
    if (!vel_cmd_topic.empty() && bo_gach(vel_cmd_topic) != bo_gach(cmd_vel_topic)) {
      subs_.push_back(create_subscription<geometry_msgs::msg::Twist>(vel_cmd_topic, 10, cb));
      info("Nhan van toc tu " + cmd_vel_topic + " va " + vel_cmd_topic + " (linear.x, angular.z)");
    }
    sub_brake_ = create_subscription<std_msgs::msg::Empty>(
        "emergency_brake", 10, [this](std_msgs::msg::Empty::ConstSharedPtr) { cb_brake(); });
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    if (!raw_encoder_topic.empty()) pub_raw_ = create_publisher<sensor_msgs::msg::JointState>(raw_encoder_topic, 10);
    if (publish_tf) tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    srv_en_ = create_service<Trigger>("~/enable", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
      srv_enable(*r);
    });
    srv_dis_ = create_service<Trigger>("~/disable", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr r) {
      disable("service disable");
      r->success = true;
      r->message = "IDLE";
    });
    timers_.push_back(create_wall_timer(std::chrono::duration<double>(CHU_KY), [this] { loop(); }));
    timers_.push_back(create_wall_timer(20ms, [this] { odom(); }));
    if (status_period > 0)
      timers_.push_back(create_wall_timer(std::chrono::duration<double>(status_period), [this] { log_trang_thai(); }));

    auto [ok, msg] = enable();
    ok ? info(msg) : error(msg);
  }

  // destroy_node ben Python: ve IDLE roi dong CAN
  void tat() {
    disable("tat node");
    link_->dong();
    info("2 banh -> IDLE, dong CAN.");
  }

 private:
  // ---------------------------------------------------------------- tham so (nhan ca int lan double tu yaml)
  rclcpp::ParameterValue khai_bao(const std::string& ten, const rclcpp::ParameterValue& md) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.dynamic_typing = true;
    return declare_parameter(ten, md, d);
  }
  double par_d(const std::string& ten, double md) {
    auto v = khai_bao(ten, rclcpp::ParameterValue(md));
    return v.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER ? static_cast<double>(v.get<int64_t>())
                                                                    : v.get<double>();
  }
  int par_i(const std::string& ten, int md) {
    auto v = khai_bao(ten, rclcpp::ParameterValue(md));
    return v.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ? static_cast<int>(v.get<double>())
                                                                   : static_cast<int>(v.get<int64_t>());
  }
  std::string par_s(const std::string& ten, const std::string& md) {
    auto v = khai_bao(ten, rclcpp::ParameterValue(md));
    return v.get_type() == rclcpp::ParameterType::PARAMETER_STRING ? v.get<std::string>() : rclcpp::to_string(v);
  }
  bool par_b(const std::string& ten, bool md) {
    auto v = khai_bao(ten, rclcpp::ParameterValue(md));
    if (v.get_type() == rclcpp::ParameterType::PARAMETER_STRING) return v.get<std::string>() == "true";
    return v.get<bool>();
  }
  static std::string bo_gach(std::string s) {  // lstrip("/")
    s.erase(0, s.find_first_not_of('/'));
    return s;
  }

  void info(const std::string& s) { RCLCPP_INFO(get_logger(), "%s", s.c_str()); }
  void warn(const std::string& s) { RCLCPP_WARN(get_logger(), "%s", s.c_str()); }
  void error(const std::string& s) { RCLCPP_ERROR(get_logger(), "%s", s.c_str()); }

  std::string mo_ta(int n) {
    NodeState& s = link_->tt(n);
    return ten_[n] + " state " + so_state(s.state) + " " + s.loi();
  }
  std::string mo_ta_ca_2() { return noi({mo_ta(left_), mo_ta(right_)}, "; "); }
  bool chi_watchdog_hoac_sach(int n) {
    NodeState& s = link_->tt(n);
    return !s.details().any() && !(s.axis_error & ~ERR_WATCHDOG);
  }

  // ---------------------------------------------------------------- bat / tat
  // "" neu ca 2 banh song, sach loi, state IDLE/CLOSED_LOOP; nguoc lai mo ta loi.
  std::string kiem_tra() {
    std::vector<std::string> loi;
    for (int n : nodes_) {
      NodeState& s = link_->tt(n);
      if (!s.t_hb || mono() - s.t_hb > HB_TIMEOUT)
        loi.push_back(strf("%s (node %d) KHONG co heartbeat", ten_[n].c_str(), n));
      else if (!s.loi().empty() || (s.state != IDLE && s.state != CLOSED_LOOP))
        loi.push_back(strf("%s (node %d) state %s %s", ten_[n].c_str(), n, so_state(s.state).c_str(), s.loi().c_str()));
    }
    return noi(loi, "; ");
  }

  // cho_dung=false: vao lai sau WATCHDOG do adapter dung hinh -> ap tiep lenh teleop (firmware ramp tu 0).
  std::pair<bool, std::string> enable(bool cho_dung = true) {
    active_ = false;
    link_->van_toc_0();  // 1 banh bi cat -> banh kia ve 0 ngay (khong quay tai cho), an watchdog
    link_->cho_heartbeat(1.5);
    // Chi co WATCHDOG (0x800) = lan truoc dung ngang / adapter dung hinh -> tu xoa. Loi khac van chan.
    std::vector<int> chi_wd;
    std::vector<std::string> ten_wd;
    for (int n : nodes_)
      if (link_->tt(n).axis_error == ERR_WATCHDOG && !link_->tt(n).details().any()) {
        chi_wd.push_back(n);
        ten_wd.push_back(ten_[n]);
      }
    if (!chi_wd.empty()) {
      warn("Xoa loi WATCHDOG con lai: " + noi(ten_wd, ", "));
      link_->xoa_loi(chi_wd);
      // KHONG ngu_s(0.6) tron: banh kia dang CLOSED_LOOP (watchdog bat) bi bo doi > 0.5 s -> WATCHDOG lay theo
      // -> 'KHONG enable' (13/11). Cho 0.6 s nhung van gui van toc 0 moi CHU_KY toi ca 2 node.
      for (double t_het = mono() + 0.6;;) {
        link_->van_toc_0();
        double con = t_het - mono();
        if (con <= 0) break;
        ngu_s(std::min(CHU_KY, con));
      }
    }
    std::string loi = kiem_tra();
    if (!loi.empty()) return {false, "KHONG enable: " + loi};

    link_->van_toc_0();  // input_vel cu con trong RAM -> VEL_RAMP se chay toi do ngay
    link_->dat_input_mode(VEL_RAMP);
    link_->dat_state(CLOSED_LOOP);
    double t0 = mono(), t_an = t0;
    while (mono() - t0 < 3.0) {
      bool du = true, co_loi = false;
      for (int n : nodes_) {
        du = du && link_->tt(n).state == CLOSED_LOOP && link_->con_song(n, 0.3, HB_TIMEOUT);
        co_loi = co_loi || !link_->tt(n).loi().empty();
      }
      if (du) {
        if (cho_dung) v_ = w_ = 0.0;
        t_phanh_.reset();
        for (int n : nodes_) sai_ack_[n] = 0;
        t_mat_ll_.reset();
        t_gui_cuoi_ = mono();
        cho_lenh_dung_ = cho_dung;
        active_ = true;
        return {true, strf("CLOSED_LOOP ca 2 banh sau %.2f s", mono() - t0)};
      }
      if (co_loi) break;
      if (mono() - t_an >= CHU_KY) {
        link_->van_toc_0();  // an watchdog CAN trong luc cho
        t_an = mono();
      }
      ngu_s(0.02);
    }
    link_->dat_state(IDLE);
    return {false, "KHONG vao duoc CLOSED_LOOP: " + mo_ta_ca_2()};
  }

  void disable(const std::string& ly_do) {
    bool was = active_;
    active_ = false;
    try {
      link_->dua_ve_idle();
    } catch (const CanError& e) {
      error(std::string("gui IDLE loi: ") + e.what());
    }
    if (was) warn("2 banh -> IDLE: " + ly_do);
  }

  void srv_enable(Trigger::Response& res) {
    if (!khoa_.empty()) {
      warn("~/enable bang tay -> bo khoa (" + khoa_ + ")");
      khoa_.clear();
    }
    std::tie(res.success, res.message) = enable();
  }

  bool lenh_dung() { return mono() - t_cmd_ > cmd_timeout_ || (v_ == 0.0 && w_ == 0.0); }

  // Khong active: thu enable lai moi 2 s neu loi chi la WATCHDOG / khong loi va lenh dang dung.
  // Da cat vi runaway (khoa_) -> KHONG tu enable (3/11: tu enable lai -> banh runaway lan nua).
  void tu_enable() {
    if (!khoa_.empty() || mono() - t_thu_enable_ < 2.0 || !lenh_dung()) return;
    t_thu_enable_ = mono();
    for (int n : nodes_)
      if (!link_->con_song(n, 0.3, HB_TIMEOUT)) return;
    for (int n : nodes_)
      if (!chi_watchdog_hoac_sach(n)) return;  // loi phan cung -> phai goi ~/enable bang tay
    auto [ok, msg] = enable();
    ok ? info("Tu enable lai: " + msg) : warn("Tu enable lai: " + msg);
  }

  // ---------------------------------------------------------------- callback
  void cb_cmd(const geometry_msgs::msg::Twist& msg) {
    bool doi = msg.linear.x != v_ || msg.angular.z != w_;
    if (doi)
      info(strf("cmd_vel moi: v %+.3f m/s  w %+.3f rad/s%s", msg.linear.x, msg.angular.z,
                active_ ? "" : "  (CHUA enable -> bo qua)"));
    v_ = msg.linear.x;
    w_ = msg.angular.z;
    t_cmd_ = mono();
    ++so_cmd_;
    if (doi && active_ && !t_phanh_) ap_lenh();  // gui ngay, khong cho chu ky (nhu odrive_controller)
  }

  void cb_brake() {
    if (!active_ || t_phanh_) return;
    v_ = w_ = 0.0;
    link_->van_toc_0();  // input_vel = 0 TRUOC khi doi PASSTHROUGH (tranh vot lenh cu)
    link_->dat_input_mode(PASSTHROUGH);
    t_phanh_ = mono();
    warn("PHANH GAP");
  }

  void log_trang_thai() {
    if (!active_) {
      std::string cho = !khoa_.empty() ? "  -> KHOA (" + khoa_ +
                                             "): ke banh len, calib, roi ros2 service call /can_bridge/enable "
                                             "std_srvs/srv/Trigger"
                        : lenh_dung() ? ""
                                      : "  -> bam DUNG (v = w = 0) tren teleop de tu enable lai";
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "KHONG active (%s)  cmd_vel nhan %d%s",
                           mo_ta_ca_2().c_str(), so_cmd_, cho.c_str());
    } else {
      double cu = mono() - t_cmd_;
      std::string nguon = t_phanh_ ? "PHANH"
                          : cu <= cmd_timeout_ ? "cmd_vel"
                          : !t_cmd_ ? "CHUA co cmd_vel"
                                    : strf("cmd_vel cu %.1fs -> 0", cu);
      NodeState &L = link_->tt(left_), &R = link_->tt(right_);
      info(strf("cmd_vel nhan %d/ky | ap v %+.2f w %+.2f (%s) | gui TRAI %+.2f PHAI %+.2f vong/s | "
                "do TRAI %+.2f PHAI %+.2f vong/s | thieu ACK %d/%d",
                so_cmd_, v_, w_, nguon.c_str(), vong_gui_[left_], vong_gui_[right_], L.vel.load(), R.vel.load(),
                so_thieu_, so_gui_));
    }
    so_cmd_ = so_gui_ = so_thieu_ = 0;
    // Chan doan frame nhan: binh thuong moi giay ~10 heartbeat, ~100 encoder moi node
    std::vector<std::string> tre, dem;
    for (int n : nodes_) {
      NodeState& s = link_->tt(n);
      int hb = s.so_hb.exchange(0), enc = s.so_enc.exchange(0);
      double gap = s.hb_gap_max.exchange(0.0);
      if (gap > HB_TRE_CANH_BAO) tre.push_back(strf("%s vang %.2f s", ten_[n].c_str(), gap));
      dem.push_back(strf("%s hb %d enc %d", ten_[n].c_str(), hb, enc));
    }
    if (!tre.empty())
      warn("Heartbeat tre (khong cat, encoder/ACK van song): " + noi(tre, "; ") + " | ky nay nhan " + noi(dem, "; "));
  }

  // ---------------------------------------------------------------- dieu khien
  // Dong hoc vi sai (body twist -> banh):
  //     v_left  = v_b,x - w_b,z * W / 2        v_b,x = linear.x [m/s] (tien +), w_b,z = angular.z [rad/s] (trai +)
  //     v_right = v_b,x + w_b,z * W / 2        W = wheel_separation [m]
  // roi m/s -> vong/s (chia pi * D), nhan dau lap nguoc (invert), giu ti le trai/phai neu cham vmax.
  VongMap toc_do_banh(double v, double w) {
    double m_tren_vong = M_PI * dk_;
    double vl = (v - w * rong_ / 2) / m_tren_vong;
    double vr = (v + w * rong_ / 2) / m_tren_vong;
    double k = std::max({1.0, std::abs(vl) / vong_max_, std::abs(vr) / vong_max_});
    return {{left_, dau_[left_] * vl / k}, {right_, dau_[right_] * vr / k}};
  }

  void kiem_luong_nhan() {
    std::string cuoi;
    if (int n = link_->lay_loi_nhan(cuoi))
      warn(strf("Luong nhan: %d frame serial hong (bo qua), cuoi: %s", n, cuoi.c_str()));
    if (link_->kiem_luong_nhan()) error("Luong nhan CAN da chet -> da khoi dong lai");
  }

  void loop() {
    kiem_luong_nhan();
    if (!active_) {
      tu_enable();
      return;
    }
    double now = mono();
    bool mat = false;
    for (int n : nodes_) mat = mat || !link_->con_song(n, 0.3, HB_TIMEOUT) || sai_ack_[n] >= MAT_ACK;
    if (mat) {
      if (!t_mat_ll_) {
        t_mat_ll_ = now;
        std::vector<std::string> ct;
        for (int n : nodes_)
          ct.push_back(strf("%s (frame cuoi %.0f ms, heartbeat cuoi %.0f ms, thieu ACK %d)", ten_[n].c_str(),
                            (now - link_->tt(n).t_rx) * 1000, (now - link_->tt(n).t_hb) * 1000, sai_ack_[n]));
        warn("Mat lien lac " + noi(ct, ", ") + " -> van gui lenh, cho adapter song lai");
      } else if (now - *t_mat_ll_ > MAT_LL_TOI_DA) {
        std::string ly_do = strf("mat lien lac lien tuc %.1f s", now - *t_mat_ll_);
        disable(ly_do);
        error(ly_do + " -> kiem tra adapter / day CAN / nguon board");
        t_mat_ll_.reset();
        return;
      }
    } else {
      if (t_mat_ll_) {
        info(strf("Lien lac hoi phuc sau %.2f s", now - *t_mat_ll_));
        t_mat_ll_.reset();
      }
      // du lieu moi -> moi xet state / loi
      for (int n : nodes_)
        if (!chi_watchdog_hoac_sach(n)) {
          std::string ly_do = ten_[n] + " loi phan cung: " + link_->tt(n).loi();
          disable(ly_do);
          error(ly_do + " -> sua xong goi ~/enable (hoac ros2 run zltech_can_driver_cpp calib)");
          return;
        }
      std::vector<std::string> bi_cat;
      for (int n : nodes_)
        if (link_->tt(n).state != CLOSED_LOOP || link_->tt(n).axis_error) bi_cat.push_back(mo_ta(n));
      if (!bi_cat.empty()) {
        warn("Bi cat khoi CLOSED_LOOP (" + noi(bi_cat, "; ") + ") -> tu vao lai, tang toc lai tu 0");
        auto [ok, msg] = enable(false);
        ok ? info("Vao lai: " + msg) : error("Vao lai: " + msg);
        return;
      }
    }

    if (kiem_runaway(now)) return;

    if (t_phanh_) {
      bool da_dung = true;
      for (int n : nodes_) da_dung = da_dung && std::abs(link_->tt(n).vel.load()) < VEL_DUNG;
      if (da_dung || mono() - *t_phanh_ >= PHANH_TOI_DA) {
        link_->dat_input_mode(VEL_RAMP);
        info(strf("Phanh xong sau %.2f s -> VEL_RAMP", mono() - *t_phanh_));
        t_phanh_.reset();
      }
    }

    ap_lenh();
  }

  // true neu da cat vi banh chay nhanh hon lenh (khong ap dung luc dang phanh gap).
  bool kiem_runaway(double now) {
    while (!ls_lenh_.empty() && now - ls_lenh_.front().first > CUA_SO_LENH) ls_lenh_.pop_front();
    if (t_phanh_ || ls_lenh_.empty()) return false;
    for (int n : nodes_) {
      double lon_nhat = 0.0;
      for (const auto& [t, v] : ls_lenh_) lon_nhat = std::max(lon_nhat, std::abs(v.at(n)));
      double gioi_han = lon_nhat + VUOT_LENH, do_ = link_->tt(n).vel;
      if (std::abs(do_) <= gioi_han) {
        t_vuot_[n].reset();
        continue;
      }
      if (!t_vuot_[n]) {
        t_vuot_[n] = now;
      } else if (now - *t_vuot_[n] >= VUOT_TOI_DA) {
        std::string ly_do = strf("%s CHAY VUOT LENH: do %+.2f vong/s, lenh toi da %.2f vong/s suot %.2f s "
                                 "(nghi sai offset encoder / Hall nhieu)",
                                 ten_[n].c_str(), do_, gioi_han - VUOT_LENH, now - *t_vuot_[n]);
        disable(ly_do);
        khoa_ = ten_[n] + " runaway";
        error(ly_do + " -> KHOA, khong tu chay lai. KE BANH LEN, calib lai "
                      "(ros2 run zltech_can_driver_cpp calib) roi goi ~/enable");
        for (int k : nodes_) t_vuot_[k].reset();
        ls_lenh_.clear();
        return true;
      }
    }
    return false;
  }

  void ap_lenh() {
    if (cho_lenh_dung_ && lenh_dung()) cho_lenh_dung_ = false;
    double v, w;
    if (t_phanh_ || mono() - t_cmd_ > cmd_timeout_) {
      v = w = 0.0;  // dang phanh / teleop chet -> dung
    } else if (cho_lenh_dung_) {
      v = w = 0.0;  // vua enable ma teleop dang giu ga -> cho lenh dung
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Vua enable, lenh dang khac 0 -> bam DUNG tren teleop truoc");
    } else {
      v = v_;
      w = w_;
    }
    vong_gui_ = toc_do_banh(v, w);
    double tre = mono() - t_gui_cuoi_;
    if (tre > 0.2) warn(strf("Vong gui lenh bi tre %.2f s (watchdog firmware cat o 0.5 s)", tre));
    ls_lenh_.emplace_back(mono(), vong_gui_);
    std::vector<int> thieu;
    try {
      thieu = link_->gui_dong_bo(vong_gui_, ACK_TIMEOUT);
    } catch (const CanError& e) {
      // Python: loi gui lam vong lap chet; o day coi nhu thieu ACK ca 2 -> logic mat lien lac xu ly
      warn(std::string("gui lenh loi: ") + e.what());
      thieu = nodes_;
    }
    t_gui_cuoi_ = mono();
    ++so_gui_;
    so_thieu_ += !thieu.empty();
    for (int n : nodes_)
      sai_ack_[n] = std::find(thieu.begin(), thieu.end(), n) != thieu.end() ? sai_ack_[n] + 1 : 0;
  }

  // Nhu computeOdom() cua odrive_controller: tich phan DELTA VI TRI encoder (0x09 pos, vong) -> x, y, theta;
  // van toc tu vel encoder. 50 Hz, cung covariance.
  void odom() {
    NodeState &L = link_->tt(left_), &R = link_->tt(right_);
    double m_tren_vong = M_PI * dk_;
    std::pair<double, double> pos{L.pos, R.pos};
    if (!(L.t_rx && R.t_rx)) return;  // chua co encoder
    auto stamp = now();
    if (pub_raw_) {
      sensor_msgs::msg::JointState j;
      j.header.stamp = stamp;
      j.name = {"left_wheel", "right_wheel"};
      j.position = {L.pos, R.pos};  // vong, nguyen tu board (chua nhan invert)
      j.velocity = {L.vel, R.vel};  // vong/s
      pub_raw_->publish(j);
    }
    if (!pos_truoc_) pos_truoc_ = pos;
    double dL = (pos.first - pos_truoc_->first) * dau_[left_] * m_tren_vong;
    double dR = (pos.second - pos_truoc_->second) * dau_[right_] * m_tren_vong;
    pos_truoc_ = pos;
    if (std::max(std::abs(dL), std::abs(dR)) > 0.5) {  // board khoi dong lai (pos ve 0) -> bo buoc nhay
      warn(strf("Odom: bo buoc nhay encoder TRAI %+.2f m, PHAI %+.2f m", dL, dR));
      dL = dR = 0.0;
    }
    double dc = (dL + dR) / 2, dth = (dR - dL) / rong_;
    x_ += dc * std::cos(th_ + dth / 2);
    y_ += dc * std::sin(th_ + dth / 2);
    th_ += dth;
    double vl = dau_[left_] * L.vel * m_tren_vong, vr = dau_[right_] * R.vel * m_tren_vong;
    double v = (vl + vr) / 2, w = (vr - vl) / rong_;

    double qz = std::sin(th_ / 2), qw = std::cos(th_ / 2);
    nav_msgs::msg::Odometry o;
    o.header.stamp = stamp;
    o.header.frame_id = frame_odom_;
    o.child_frame_id = frame_base_;
    o.pose.pose.position.x = x_;
    o.pose.pose.position.y = y_;
    o.pose.pose.orientation.z = qz;
    o.pose.pose.orientation.w = qw;
    o.twist.twist.linear.x = v;
    o.twist.twist.angular.z = w;
    o.pose.covariance[0] = o.pose.covariance[7] = 1e-3;  // giong odrive_controller
    o.pose.covariance[35] = 1e-2;
    o.twist.covariance[0] = 1e-3;
    o.twist.covariance[35] = 1e-2;
    pub_odom_->publish(o);
    if (tf_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = stamp;
      t.header.frame_id = frame_odom_;
      t.child_frame_id = frame_base_;
      t.transform.translation.x = x_;
      t.transform.translation.y = y_;
      t.transform.rotation.z = qz;
      t.transform.rotation.w = qw;
      tf_->sendTransform(t);
    }
  }

  // ---- cau hinh
  int left_ = 0, right_ = 2;
  double rong_ = 0.382, dk_ = 0.139, vong_max_ = 4.0, cmd_timeout_ = 0.5;
  std::string frame_odom_, frame_base_;
  std::vector<int> nodes_;
  std::map<int, std::string> ten_;
  std::map<int, double> dau_;
  std::unique_ptr<CanLink> link_;
  // ---- trang thai
  bool active_ = false;
  double v_ = 0.0, w_ = 0.0, t_cmd_ = 0.0;
  std::optional<double> t_phanh_;
  std::map<int, int> sai_ack_;
  double t_gui_cuoi_ = 0.0;      // lan cuoi gui SYNC (bat vong lap bi tre -> WATCHDOG)
  double t_thu_enable_ = 0.0;
  bool cho_lenh_dung_ = true;    // sau enable: chan lenh khac 0 cho toi khi thay lenh dung
  std::optional<double> t_mat_ll_;  // thoi diem bat dau mat lien lac (nullopt = binh thuong)
  double x_ = 0.0, y_ = 0.0, th_ = 0.0;
  std::optional<std::pair<double, double>> pos_truoc_;  // (pos TRAI, pos PHAI) lan odom truoc
  int so_cmd_ = 0, so_gui_ = 0, so_thieu_ = 0;           // dem trong ky log
  VongMap vong_gui_;
  std::deque<std::pair<double, VongMap>> ls_lenh_;      // (t, {node: vong/s gui}) trong CUA_SO_LENH s
  std::map<int, std::optional<double>> t_vuot_;
  std::string khoa_;             // khac "" = da cat vi runaway: KHONG tu enable, chi ~/enable bang tay
  // ---- ROS
  std::vector<rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr> subs_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_brake_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_raw_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::Service<Trigger>::SharedPtr srv_en_, srv_dis_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
};

int main(int argc, char** argv) {
  // Tu bat SIGINT / SIGTERM / SIGHUP (khong de rclcpp tat context) -> thoat vong lap roi moi dua banh ve IDLE.
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  for (int sig : {SIGINT, SIGTERM, SIGHUP}) std::signal(sig, bat_tin_hieu);
  std::shared_ptr<CanBridge> node;
  try {
    node = std::make_shared<CanBridge>();
  } catch (const FatalError& e) {
    std::fprintf(stderr, "%s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);
  int ma = 0;
  try {
    while (!g_thoat && rclcpp::ok()) ex.spin_once(50ms);
  } catch (const std::exception& e) {  // loi trong callback -> van dua banh ve IDLE truoc khi thoat
    RCLCPP_FATAL(node->get_logger(), "Loi: %s", e.what());
    ma = 1;
  }
  bo_qua_tin_hieu();
  ex.remove_node(node);
  node->tat();
  node.reset();
  rclcpp::shutdown();
  return ma;
}
