// odrive_can_errors.hpp - GIAI MA heartbeat + frame loi CAN cua firmware Firmware-PlatformIO-Can-sync (ODrive v3.6 fork,
// patch err-event / collision / can-watchdog / ack-retry). Thu vien THUAN: khong phu thuoc phan cung, test bang gtest.
// Ban C++ cua zltech_can_driver/odrive_can_errors.py.
//
// Theo can_heartbeat_error_spec.md. Heartbeat KHAC chuan ODrive 0.5.x:
//     byte 0-2 axis_error (uint24 LE), byte 3 axis_state, byte 4-7 = 0 (khac 0 -> firmware la).
// Frame loi (tu gui theo su kien: doi gia tri -> gui ngay; con loi -> lap lai 100 ms; het loi -> 1 frame toan 0):
//     0x03 motor_error uint64 LE (bit toi 35, PHAI doc du 8 byte), 0x04 encoder / 0x1D controller uint32 LE.
//     0x05 sensorless uint32 LE: chi tra loi khi host hoi (RTR).
//
//     decode_bits(value, table) -> [{mask, bit, name, desc, hint}]
//     decode_frame(arbitration_id, data, is_extended, is_remote) -> Decoded | nullopt
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace zltech::err {

// ---- Bang 3: axis_state
extern const std::map<int, const char*> AXIS_STATES;
std::string state_name(int state);

// ---- Bang 5: ma loi {mask: (ten, y nghia, goi y xu ly)}
struct ErrInfo {
  const char* name;
  const char* desc;
  const char* hint;
};
using Table = std::map<uint64_t, ErrInfo>;

extern const Table AXIS_ERRORS;
extern const Table MOTOR_ERRORS;
extern const Table ENCODER_ERRORS;
extern const Table CONTROLLER_ERRORS;
extern const Table SENSORLESS_ERRORS;
// 5.6: loi he thong ODrive (odrv.error) - KHONG co frame CAN, chi de in goi y khi thay SYSTEM_LEVEL /
// BRAKE_RESISTOR_DISARMED
extern const Table SYSTEM_ERRORS;
constexpr uint64_t MOTOR_SYSTEM_HINT_MASK = 0x001000000ULL | 0x000800000ULL;  // SYSTEM_LEVEL | BRAKE_RESISTOR_DISARMED

// ---- CAN id
constexpr int CMD_HEARTBEAT = 0x01;
constexpr int CMD_ESTOP = 0x02;
constexpr int CMD_MOTOR_ERROR = 0x03;
constexpr int CMD_ENCODER_ERROR = 0x04;
constexpr int CMD_SENSORLESS_ERROR = 0x05;
constexpr int CMD_BUS_VOLTAGE_CURRENT = 0x17;
constexpr int CMD_CLEAR_ERRORS = 0x18;
constexpr int CMD_CONTROLLER_ERROR = 0x1D;

// cmd -> (kind, bang, so byte payload)
struct ErrorFrame {
  const char* kind;
  const Table* table;
  int nbytes;
};
extern const std::map<int, ErrorFrame> ERROR_FRAMES;

// bit axis_error *_FAILED -> (kind, cmd frame chi tiet)
struct FailedDetail {
  uint64_t bit;
  const char* kind;
  int cmd;
};
extern const std::vector<FailedDetail> FAILED_TO_DETAIL;

inline uint32_t arb_id(int node, int cmd) { return (static_cast<uint32_t>(node) << 5) | static_cast<uint32_t>(cmd); }

struct BitInfo {
  uint64_t mask;
  int bit;
  std::string name, desc, hint;
};

// Tach bitmask -> [{mask, bit, name, desc, hint}] theo thu tu bit tang dan. Bit la -> UNKNOWN_BIT_n.
std::vector<BitInfo> decode_bits(uint64_t value, const Table& table);
// 0x140 -> "MOTOR_FAILED|ENCODER_FAILED"; 0 -> "".
std::string bit_names(uint64_t value, const Table& table);

// heartbeat: kind = "heartbeat", axis_error, state, state_name, reserved, reserved_nonzero
// loi:       kind = "motor" | "encoder" | "controller" | "sensorless", value, cleared
struct Decoded {
  int node = 0;
  std::string kind;
  std::vector<uint8_t> raw;
  std::vector<BitInfo> errors;
  uint64_t axis_error = 0;
  int state = 0;
  std::string state_name;
  uint32_t reserved = 0;
  bool reserved_nonzero = false;
  uint64_t value = 0;
  bool cleared = false;
};

// Giai ma 1 frame. nullopt neu khong lien quan (cmd khac, extended, RTR, DLC thieu).
std::optional<Decoded> decode_frame(uint32_t arbitration_id, const std::vector<uint8_t>& data,
                                    bool is_extended = false, bool is_remote = false);

const Table& table_for(const std::string& kind);

// '(1690000000.123) can0 041#0002000100000000' -> {t, arb, data, is_extended, is_remote}; dong khac -> nullopt.
struct CandumpLine {
  double t;
  uint32_t arb;
  std::vector<uint8_t> data;
  bool is_extended;
  bool is_remote;
};
std::optional<CandumpLine> parse_candump_line(const std::string& line);

}  // namespace zltech::err
