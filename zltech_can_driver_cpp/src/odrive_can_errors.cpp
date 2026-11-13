// odrive_can_errors.cpp - bang ma loi + giai ma frame (ban C++ cua odrive_can_errors.py).
#include "zltech_can_driver_cpp/odrive_can_errors.hpp"

#include <cstdio>
#include <regex>
#include <stdexcept>

namespace zltech::err {

const std::map<int, const char*> AXIS_STATES = {
    {0, "UNDEFINED"},
    {1, "IDLE"},
    {2, "STARTUP_SEQUENCE"},
    {3, "FULL_CALIBRATION_SEQUENCE"},
    {4, "MOTOR_CALIBRATION"},
    {6, "ENCODER_INDEX_SEARCH"},
    {7, "ENCODER_OFFSET_CALIBRATION"},
    {8, "CLOSED_LOOP_CONTROL"},
    {9, "LOCKIN_SPIN"},
    {10, "ENCODER_DIR_FIND"},
    {11, "HOMING"},
    {12, "ENCODER_HALL_POLARITY_CALIBRATION"},
    {13, "ENCODER_HALL_PHASE_CALIBRATION"},
};

std::string state_name(int state) {
  auto it = AXIS_STATES.find(state);
  return it != AXIS_STATES.end() ? it->second : "UNKNOWN_STATE(" + std::to_string(state) + ")";
}

const Table AXIS_ERRORS = {
    {0x00000001, {"INVALID_STATE", "Yeu cau state khong hop le (vd vao CLOSED_LOOP khi motor/encoder chua calib).",
                  "Calib truoc roi moi chay."}},
    {0x00000040, {"MOTOR_FAILED", "Motor loi.", "Xem frame 0x03 motor_error."}},
    {0x00000080, {"SENSORLESS_ESTIMATOR_FAILED", "Bo uoc luong sensorless loi.", "Hoi frame 0x05 (RTR)."}},
    {0x00000100, {"ENCODER_FAILED", "Encoder loi.", "Xem frame 0x04 encoder_error."}},
    {0x00000200, {"CONTROLLER_FAILED", "Controller loi (vd va cham, overspeed).", "Xem frame 0x1D controller_error."}},
    {0x00000800, {"WATCHDOG_TIMER_EXPIRED",
                  "Qua watchdog_timeout khong nhan frame CAN nao (patch can-watchdog: bat khi vao CLOSED_LOOP qua CAN).",
                  "Master ngung gui lenh / mat CAN / adapter dung hinh. Kiem tra master con chay, day CAN; clear errors."}},
    {0x00001000, {"MIN_ENDSTOP_PRESSED", "Cong tac hanh trinh min bi nhan.", "Kiem tra endstop."}},
    {0x00002000, {"MAX_ENDSTOP_PRESSED", "Cong tac hanh trinh max bi nhan.", "Kiem tra endstop."}},
    {0x00004000, {"ESTOP_REQUESTED", "Co frame ESTOP (cmd 0x02) gui toi node.", "Tim ai gui ESTOP; clear errors."}},
    {0x00020000, {"HOMING_WITHOUT_ENDSTOP", "Homing nhung chua bat min endstop.", "Bat min endstop hoac bo homing."}},
    {0x00040000, {"OVER_TEMP", "(Khong dung o firmware nay).", "Xem motor_error (THERMISTOR_OVER_TEMP)."}},
    {0x00080000, {"UNKNOWN_POSITION", "Khong co uoc luong vi tri hop le.", "Kiem tra encoder / calib."}},
};

const Table MOTOR_ERRORS = {
    {0x000000001ULL, {"PHASE_RESISTANCE_OUT_OF_RANGE", "Dien tro pha do khi calib nam ngoai khoang hop ly.",
                      "Kiem tra day pha; chinh resistance_calib_max_voltage (> calibration_current x R, < 0.5 x Vbus) / "
                      "calibration_current."}},
    {0x000000002ULL, {"PHASE_INDUCTANCE_OUT_OF_RANGE", "Dien cam pha ngoai khoang hop ly.",
                      "Kiem tra day pha; chinh resistance_calib_max_voltage / calibration_current."}},
    {0x000000008ULL, {"DRV_FAULT", "Chip gate driver (DRV8301) bao loi.",
                      "Thuong do sut ap nguon gate driver khi dong lon, hoac ngan mach / FET hong."}},
    {0x000000010ULL, {"CONTROL_DEADLINE_MISSED", "Vong dieu khien tre deadline.", ""}},
    {0x000000080ULL, {"MODULATION_MAGNITUDE", "Dien ap bus khong du day dong yeu cau.",
                      "Toc do qua cao so voi Vbus, hoac resistance_calib_max_voltage > 1/2 Vbus khi calib."}},
    {0x000000400ULL, {"CURRENT_SENSE_SATURATION", "Mach do dong bao hoa (current_lim > requested_current_range).",
                      "Board ZLTECH shunt 5 mOhm: kiem tra dung firmware / range."}},
    {0x000001000ULL, {"CURRENT_LIMIT_VIOLATION", "Dong vuot current_lim + current_lim_margin (overshoot PI).",
                      "Tang current_lim_margin hoac giam bandwidth / gain."}},
    {0x000010000ULL, {"MODULATION_IS_NAN", "Modulation ra NaN (loi tinh toan).", ""}},
    {0x000020000ULL, {"MOTOR_THERMISTOR_OVER_TEMP", "Nhiet dien tro motor vuot temp_limit_upper.", "Cho nguoi, giam tai."}},
    {0x000040000ULL, {"FET_THERMISTOR_OVER_TEMP", "Nhiet FET/inverter vuot temp_limit_upper.", "Cho nguoi, tan nhiet."}},
    {0x000080000ULL, {"TIMER_UPDATE_MISSED", "Lo 1 su kien update timer (khong mong doi o firmware chuan).", ""}},
    {0x000100000ULL, {"CURRENT_MEASUREMENT_UNAVAILABLE", "ADC khong lay mau dong kip.", ""}},
    {0x000200000ULL, {"CONTROLLER_FAILED", "Motor bi disarm vi bo dieu khien (thuong FOC) loi.", ""}},
    {0x000400000ULL, {"I_BUS_OUT_OF_RANGE", "Dong DC cua motor ngoai I_bus_hard_min..I_bus_hard_max.",
                      "Thuong do ham / regen qua manh."}},
    {0x000800000ULL, {"BRAKE_RESISTOR_DISARMED", "Chay PWM khi dien tro xa dang bi disarm.",
                      "Reboot hoac clear errors; nguyen nhan goc thuong la loi he thong (bang 5.6)."}},
    {0x001000000ULL, {"SYSTEM_LEVEL", "Motor bi disarm vi LOI HE THONG ODrive (khong doc duoc qua CAN).",
                      "Kiem tra nguon / Vbus (frame 0x17); xem bang loi he thong ben duoi."}},
    {0x002000000ULL, {"BAD_TIMING", "Vong chinh lech nhip voi vong dieu khien motor (vong chinh bi ket).", ""}},
    {0x004000000ULL, {"UNKNOWN_PHASE_ESTIMATE", "Khong co goc pha hop le.", "Encoder chua calib / chua can Hall."}},
    {0x008000000ULL, {"UNKNOWN_PHASE_VEL", "Khong co van toc pha hop le.", ""}},
    {0x010000000ULL, {"UNKNOWN_TORQUE", "Khong co torque setpoint hop le.", ""}},
    {0x020000000ULL, {"UNKNOWN_CURRENT_COMMAND", "Khong co setpoint dong hop le.", "Cau hinh controller sai."}},
    {0x040000000ULL, {"UNKNOWN_CURRENT_MEASUREMENT", "Khong co gia tri do dong hop le.", ""}},
    {0x080000000ULL, {"UNKNOWN_VBUS_VOLTAGE", "Khong co gia tri Vbus hop le.", ""}},
    {0x100000000ULL, {"UNKNOWN_VOLTAGE_COMMAND", "Khong co dien ap feedforward hop le.", ""}},
    {0x200000000ULL, {"UNKNOWN_GAINS", "Chua co gain dong.",
                      "Chay motor calib hoac set phase_resistance / phase_inductance."}},
    {0x400000000ULL, {"CONTROLLER_INITIALIZING", "Gia tri noi bo khi controller chua san sang tao PWM.", ""}},
    {0x800000000ULL, {"UNBALANCED_PHASES", "Cac pha motor khong can bang.",
                      "Day pha long / dut, cuon day hong: siet 3 day pha, do dien tro A-B, B-C, C-A."}},
};

const Table ENCODER_ERRORS = {
    {0x0001, {"UNSTABLE_GAIN", "Bandwidth encoder PLL qua cao, khong on dinh.", "Giam encoder bandwidth."}},
    {0x0002, {"CPR_POLEPAIRS_MISMATCH", "CPR hoac so cap cuc sai so voi do duoc khi calib.",
              "Kiem tra cpr, pole_pairs; tang calib_scan_distance; ke banh len (cogging / ket co khi)."}},
    {0x0004, {"NO_RESPONSE", "Encoder khong phan hoi.", "Kiem tra day / cam dung chan."}},
    {0x0008, {"UNSUPPORTED_ENCODER_MODE", "Mode encoder khong ho tro.", ""}},
    {0x0010, {"ILLEGAL_HALL_STATE", "Hall doc ra trang thai cam (000/111). Nhieu hoac day hong.",
              "Board ZLTECH: Hall C o gpio11 phai PULL_UP (gpio11_mode = 1). Them tu 22 nF xuong GND neu nhieu."}},
    {0x0020, {"INDEX_NOT_FOUND_YET", "Chua tim thay xung index Z.", ""}},
    {0x0040, {"ABS_SPI_TIMEOUT", "Encoder SPI tuyet doi timeout.", ""}},
    {0x0080, {"ABS_SPI_COM_FAIL", "Loi giao tiep SPI.", ""}},
    {0x0100, {"ABS_SPI_NOT_READY", "Encoder SPI chua san sang.", ""}},
    {0x0200, {"HALL_NOT_CALIBRATED_YET", "Hall chua calib (polarity / phase).", "Chay calib Hall."}},
};

const Table CONTROLLER_ERRORS = {
    {0x0001, {"OVERSPEED", "Toc do vuot vel_limit x vel_limit_tolerance.",
              "Tang vel_limit / tolerance; tai dot ngot cung co the gay."}},
    {0x0002, {"INVALID_INPUT_MODE", "input_mode khong hop le.", ""}},
    {0x0004, {"UNSTABLE_GAIN", "Bandwidth controller qua cao, khong on dinh.", ""}},
    {0x0008, {"INVALID_MIRROR_AXIS", "axis_to_mirror khong hop le.", ""}},
    {0x0010, {"INVALID_LOAD_ENCODER", "load_encoder_axis khong hop le.", ""}},
    {0x0020, {"INVALID_ESTIMATE", "Encoder khong cho ra vi tri / van toc tuyen tinh hop le.", ""}},
    {0x0040, {"INVALID_CIRCULAR_RANGE", "Encoder khong cho ra vi tri circular hop le.", ""}},
    {0x0080, {"SPINOUT_DETECTED", "Cong suat co va dien khong khop.",
              "Encoder truot hoac offset calib sai; index nhieu. Calib lai, kiem tra giac encoder."}},
    {0x0100, {"COLLISION_DETECTED",
              "[Patch custom] Co lenh chay nhung banh khong tien trong khi Iq cao suot collision_time -> vat can / "
              "ket banh. Motor bi nha.",
              "Go vat can roi clear errors."}},
};

const Table SENSORLESS_ERRORS = {
    {0x0001, {"UNSTABLE_GAIN", "PLL sensorless khong on dinh.", ""}},
    {0x0002, {"UNKNOWN_CURRENT_MEASUREMENT", "Khong co gia tri do dong hop le.", ""}},
};

const Table SYSTEM_ERRORS = {
    {0x01, {"CONTROL_ITERATION_MISSED", "Lo vong dieu khien", ""}},
    {0x02, {"DC_BUS_UNDER_VOLTAGE", "Vbus thap hon nguong (pin yeu, sut ap khi tang toc)", ""}},
    {0x04, {"DC_BUS_OVER_VOLTAGE", "Vbus cao hon nguong (regen khi ham, khong co dien tro xa)", ""}},
    {0x08, {"DC_BUS_OVER_REGEN_CURRENT", "Dong hoan ve nguon qua lon", ""}},
    {0x10, {"DC_BUS_OVER_CURRENT", "Dong lay tu nguon qua lon", ""}},
    {0x20, {"BRAKE_DEADTIME_VIOLATION", "Vi pham deadtime dien tro xa", ""}},
    {0x40, {"BRAKE_DUTY_CYCLE_NAN", "Duty dien tro xa NaN", ""}},
    {0x80, {"INVALID_BRAKE_RESISTANCE", "Gia tri dien tro xa khong hop le", ""}},
};

const std::map<int, ErrorFrame> ERROR_FRAMES = {
    {CMD_MOTOR_ERROR, {"motor", &MOTOR_ERRORS, 8}},
    {CMD_ENCODER_ERROR, {"encoder", &ENCODER_ERRORS, 4}},
    {CMD_CONTROLLER_ERROR, {"controller", &CONTROLLER_ERRORS, 4}},
    {CMD_SENSORLESS_ERROR, {"sensorless", &SENSORLESS_ERRORS, 4}},
};

const std::vector<FailedDetail> FAILED_TO_DETAIL = {
    {0x40, "motor", CMD_MOTOR_ERROR},
    {0x80, "sensorless", CMD_SENSORLESS_ERROR},
    {0x100, "encoder", CMD_ENCODER_ERROR},
    {0x200, "controller", CMD_CONTROLLER_ERROR},
};

std::vector<BitInfo> decode_bits(uint64_t value, const Table& table) {
  std::vector<BitInfo> out;
  for (int bit = 0; bit < 64 && (value >> bit); ++bit) {
    uint64_t mask = 1ULL << bit;
    if (!(value & mask)) continue;
    auto it = table.find(mask);
    if (it != table.end()) {
      out.push_back({mask, bit, it->second.name, it->second.desc, it->second.hint});
    } else {
      char desc[96];
      std::snprintf(desc, sizeof(desc), "Bit la (0x%llX) khong co trong bang.", static_cast<unsigned long long>(mask));
      out.push_back({mask, bit, "UNKNOWN_BIT_" + std::to_string(bit), desc, "Kiem tra phien ban firmware."});
    }
  }
  return out;
}

std::string bit_names(uint64_t value, const Table& table) {
  std::string s;
  for (const auto& e : decode_bits(value, table)) s += (s.empty() ? "" : "|") + e.name;
  return s;
}

static uint64_t le(const std::vector<uint8_t>& d, size_t from, size_t n) {
  uint64_t v = 0;
  for (size_t i = 0; i < n; ++i) v |= static_cast<uint64_t>(d[from + i]) << (8 * i);
  return v;
}

std::optional<Decoded> decode_frame(uint32_t arbitration_id, const std::vector<uint8_t>& data, bool is_extended,
                                    bool is_remote) {
  if (is_extended || is_remote || arbitration_id > 0x7FF) return std::nullopt;
  int node = static_cast<int>(arbitration_id >> 5), cmd = static_cast<int>(arbitration_id & 0x1F);
  if (node == 0x3F) return std::nullopt;  // broadcast (SYNC_GO ...), khong phai trang thai
  Decoded d;
  d.node = node;
  d.raw = data;
  if (cmd == CMD_HEARTBEAT) {
    if (data.size() < 4) return std::nullopt;
    d.kind = "heartbeat";
    d.axis_error = le(data, 0, 3);
    d.state = data[3];
    d.state_name = state_name(d.state);
    d.errors = decode_bits(d.axis_error, AXIS_ERRORS);
    d.reserved = data.size() >= 8 ? static_cast<uint32_t>(le(data, 4, 4)) : 0;
    d.reserved_nonzero = d.reserved != 0;
    return d;
  }
  auto it = ERROR_FRAMES.find(cmd);
  if (it != ERROR_FRAMES.end()) {
    const auto& f = it->second;
    if (data.size() < static_cast<size_t>(f.nbytes)) return std::nullopt;
    d.kind = f.kind;
    d.value = le(data, 0, f.nbytes);
    d.cleared = d.value == 0;
    d.errors = decode_bits(d.value, *f.table);
    return d;
  }
  return std::nullopt;
}

const Table& table_for(const std::string& kind) {
  if (kind == "axis") return AXIS_ERRORS;
  if (kind == "motor") return MOTOR_ERRORS;
  if (kind == "encoder") return ENCODER_ERRORS;
  if (kind == "controller") return CONTROLLER_ERRORS;
  if (kind == "sensorless") return SENSORLESS_ERRORS;
  throw std::out_of_range("table_for: " + kind);
}

std::optional<CandumpLine> parse_candump_line(const std::string& line) {
  static const std::regex re(
      R"(^\s*\((\d+(?:\.\d+)?)\)\s+\S+\s+([0-9A-Fa-f]{3}|[0-9A-Fa-f]{8})#(R\d?|[0-9A-Fa-f]*)\s*$)");
  std::smatch m;
  if (!std::regex_match(line, m, re)) return std::nullopt;
  std::string ident = m[2], payload = m[3];
  CandumpLine r;
  r.t = std::stod(m[1]);
  r.arb = static_cast<uint32_t>(std::stoul(ident, nullptr, 16));
  r.is_extended = ident.size() == 8;
  r.is_remote = !payload.empty() && (payload[0] == 'R' || payload[0] == 'r');
  if (!r.is_remote) {
    if (payload.size() % 2) return std::nullopt;
    for (size_t i = 0; i < payload.size(); i += 2)
      r.data.push_back(static_cast<uint8_t>(std::stoul(payload.substr(i, 2), nullptr, 16)));
  }
  return r;
}

}  // namespace zltech::err
