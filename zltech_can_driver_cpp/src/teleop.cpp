// teleop - TELEOP BAN PHIM ROS 2: phim -> /cmd_vel (10 lan/s). Khong dung CAN, chay cung can_bridge.
// Ban C++ cua zltech_can_driver/teleop.py.
//
//     ros2 run zltech_can_driver_cpp teleop
//     ros2 run zltech_can_driver_cpp teleop --ros-args -p v_max:=0.5 -p w_max:=1.0
//
//     W / S : tang / giam van toc tien (10 lan bam = v_max)   A / D : quay trai / phai (10 lan bam = w_max)
//     SPACE : dung em (v = w = 0, banh giam toc theo ramp)      X     : PHANH GAP (/emergency_brake)
//     Q / Ctrl+C : thoat (gui v = w = 0 truoc)
// Moi lan bam nhay ngay 1 bac -> bridge gui xuong banh ngay. Teleop chet -> bridge het cmd_timeout (0.5 s) tu dung.
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <geometry_msgs/msg/twist.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

#include "zltech_can_driver_cpp/cli.hpp"

using namespace std::chrono_literals;

static const double V_MAX_MD = 4.0 * M_PI * 0.139;  // 1.75 m/s
static const double W_MAX_MD = V_MAX_MD / 0.382;    // 4.57 rad/s

static std::atomic<bool> g_thoat{false};
static void bat_tin_hieu(int) { g_thoat = true; }

class Teleop : public rclcpp::Node {
 public:
  explicit Teleop(const zltech::cli::BanPhim& kb) : Node("can_teleop"), kb_(kb) {
    v_max_ = declare_parameter("v_max", V_MAX_MD);
    w_max_ = declare_parameter("w_max", W_MAX_MD);
    pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub_brake_ = create_publisher<std_msgs::msg::Empty>("emergency_brake", 10);
    timers_.push_back(create_wall_timer(1s, [this] { kiem_bridge(); }));
    timers_.push_back(create_wall_timer(100ms, [this] { gui(); }));
    timers_.push_back(create_wall_timer(10ms, [this] { doc_phim(); }));
    std::printf("W/S tien-lui, A/D quay trai-phai, SPACE dung em, X PHANH GAP, Q thoat.\n\n");
  }

  void dung() {
    v_ = w_ = 0.0;
    gui();
  }

 private:
  void kiem_bridge() {
    bool co = pub_->get_subscription_count() > 0;  // bridge co dang nghe cmd_vel khong (loi xem o terminal bridge)
    if (co_bridge_ != co) {
      co_bridge_ = co;
      std::printf("\n  [bridge] %s\n",
                  co ? "da ket noi" : "KHONG thay can_bridge (chua chay / da tat) -> banh se khong quay");
      std::fflush(stdout);
    }
  }

  void gui() {
    geometry_msgs::msg::Twist m;
    m.linear.x = v_;
    m.angular.z = w_;
    pub_->publish(m);
    std::printf("\r  v %+.2f m/s  w %+.2f rad/s   ", v_, w_);
    std::fflush(stdout);
  }

  static double lam_tron(double x) { return std::round(x * 1000.0) / 1000.0; }

  void doc_phim() {
    bool doi = false;
    while (kb_.kbhit()) {
      char c = static_cast<char>(std::tolower(kb_.getch()));
      if (c == 'q') {
        g_thoat = true;
        return;
      }
      if (c == 'x') {
        v_ = w_ = 0.0;
        pub_brake_->publish(std_msgs::msg::Empty());
        std::printf("\n  PHANH GAP\n");
        std::fflush(stdout);
      } else if (c == 'w') {
        v_ += v_max_ / 10;
      } else if (c == 's') {
        v_ -= v_max_ / 10;
      } else if (c == 'a') {
        w_ += w_max_ / 10;
      } else if (c == 'd') {
        w_ -= w_max_ / 10;
      } else if (c == ' ') {
        v_ = w_ = 0.0;
      } else {
        continue;
      }
      v_ = std::clamp(lam_tron(v_), -v_max_, v_max_);
      w_ = std::clamp(lam_tron(w_), -w_max_, w_max_);
      doi = true;
    }
    if (doi) gui();  // gui ngay khi bam phim, khong cho chu ky
  }

  const zltech::cli::BanPhim& kb_;
  double v_max_, w_max_, v_ = 0.0, w_ = 0.0;
  std::optional<bool> co_bridge_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_brake_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
};

int main(int argc, char** argv) {
  if (!isatty(STDIN_FILENO)) {
    std::fprintf(stderr, "teleop can chay trong terminal\n");
    return 1;
  }
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  for (int sig : {SIGINT, SIGTERM, SIGHUP}) std::signal(sig, bat_tin_hieu);
  {
    zltech::cli::BanPhim kb;  // cbreak; tra lai terminal khi ra khoi khoi nay
    auto node = std::make_shared<Teleop>(kb);
    rclcpp::executors::SingleThreadedExecutor ex;
    ex.add_node(node);
    while (!g_thoat && rclcpp::ok()) ex.spin_once(10ms);
    node->dung();  // gui v = w = 0 truoc khi thoat
  }
  std::printf("\nThoat teleop.\n");
  rclcpp::shutdown();
  return 0;
}
