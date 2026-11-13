// cli.hpp - tien ich cho cac tool dong lenh (calib / clear_errors / drive_test / error_monitor):
// doc tham so --ten=gia_tri, ban phim cbreak, bat Ctrl+C.
#pragma once

#include <termios.h>

#include <atomic>
#include <map>
#include <string>
#include <vector>

namespace zltech::cli {

// ---- Ctrl+C / kill / dong terminal -> g_dung = true (thay cho KeyboardInterrupt ben Python)
extern std::atomic<bool> g_dung;
void bat_tin_hieu();
struct NgatCtrlC {};
// sleep s giay, nem NgatCtrlC neu co Ctrl+C
void ngu(double s);

// ---- tham so dong lenh kieu argparse: --ten=gia_tri | --ten gia_tri | --co
class Args {
 public:
  explicit Args(std::string mo_ta) : mo_ta_(std::move(mo_ta)) {}
  void them(const std::string& ten, const std::string& mac_dinh, const std::string& help,
            const std::vector<std::string>& chon = {});
  void co(const std::string& ten, const std::string& help);  // flag
  void doc(int argc, char** argv);  // --help -> in roi thoat; sai -> in loi roi thoat 2
  std::string s(const std::string& ten) const;
  int i(const std::string& ten) const;
  double d(const std::string& ten) const;
  bool b(const std::string& ten) const;

 private:
  struct Opt {
    std::string mac_dinh, help, gia_tri;
    std::vector<std::string> chon;
    bool la_co = false, da_dat = false;
  };
  [[noreturn]] void loi(const std::string& msg) const;
  std::string mo_ta_, prog_;
  std::vector<std::string> thu_tu_;
  std::map<std::string, Opt> opt_;
};

// Them --trai --phai --adapter --cong --baud-serial
Args& them_tham_so_can(Args& ap);
// {node_id: "TRAI"/"PHAI"}; FatalError neu 2 node trung
std::map<int, std::string> ten_banh(const Args& a);
// --chi=TRAI/PHAI -> danh sach node; khong co --chi -> ca 2 (thu tu trai, phai)
std::vector<int> chon_banh(const Args& a, const std::map<int, std::string>& ten);

// ---- Terminal o che do cbreak -> doc tung phim khong can Enter, Ctrl+C van hoat dong.
class BanPhim {
 public:
  BanPhim();
  ~BanPhim();
  bool kbhit() const;
  char getch() const;

 private:
  int fd_;
  bool co_cu_ = false;
  termios cu_{};
};

}  // namespace zltech::cli
