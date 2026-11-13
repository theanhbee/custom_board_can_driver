// Unit test odrive_can_errors theo dung cac vi du muc 6 cua can_heartbeat_error_spec.md
// (cung bo test voi zltech_can_driver/test/test_odrive_can_errors.py).
//
//     colcon test --packages-select zltech_can_driver_cpp && colcon test-result --verbose
#include <gtest/gtest.h>

#include "zltech_can_driver_cpp/odrive_can_errors.hpp"
#include "zltech_can_driver_cpp/protocol.hpp"

namespace E = zltech::err;

static std::vector<uint8_t> hx(std::string s) {
  std::vector<uint8_t> out;
  s.erase(std::remove(s.begin(), s.end(), ' '), s.end());
  for (size_t i = 0; i + 1 < s.size(); i += 2) out.push_back(static_cast<uint8_t>(std::stoul(s.substr(i, 2), nullptr, 16)));
  return out;
}

static E::Decoded frame(uint32_t arb, const std::string& hexdata) {
  auto d = E::decode_frame(arb, hx(hexdata));
  EXPECT_TRUE(d.has_value());
  return d.value_or(E::Decoded{});
}

static std::vector<std::string> names(const E::Decoded& d) {
  std::vector<std::string> v;
  for (const auto& e : d.errors) v.push_back(e.name);
  return v;
}

using V = std::vector<std::string>;

// ---------------------------------------------------------------- muc 6 (10 vi du)
TEST(Spec, HbPhaiClosedLoopKhongLoi) {
  auto d = frame(0x041, "00 00 00 08 00 00 00 00");
  EXPECT_EQ(d.node, 2);
  EXPECT_EQ(d.kind, "heartbeat");
  EXPECT_EQ(d.state_name, "CLOSED_LOOP_CONTROL");
  EXPECT_EQ(d.axis_error, 0u);
  EXPECT_TRUE(d.errors.empty());
  EXPECT_FALSE(d.reserved_nonzero);
}

TEST(Spec, HbPhaiIdleControllerFailed) {
  auto d = frame(0x041, "00 02 00 01 00 00 00 00");
  EXPECT_EQ(d.node, 2);
  EXPECT_EQ(d.state_name, "IDLE");
  EXPECT_EQ(d.axis_error, 0x200u);
  EXPECT_EQ(names(d), V{"CONTROLLER_FAILED"});
}

TEST(Spec, ControllerCollision) {
  auto d = frame(0x05D, "00 01 00 00 00 00 00 00");
  EXPECT_EQ(d.node, 2);
  EXPECT_EQ(d.kind, "controller");
  EXPECT_EQ(d.value, 0x100u);
  EXPECT_EQ(names(d), V{"COLLISION_DETECTED"});
}

TEST(Spec, HbTraiMotorVaEncoderFailed) {
  auto d = frame(0x001, "40 01 00 01 00 00 00 00");
  EXPECT_EQ(d.node, 0);
  EXPECT_EQ(d.state_name, "IDLE");
  EXPECT_EQ(d.axis_error, 0x140u);
  EXPECT_EQ(names(d), (V{"MOTOR_FAILED", "ENCODER_FAILED"}));
}

TEST(Spec, MotorCurrentLimitViolation) {
  auto d = frame(0x003, "00 10 00 00 00 00 00 00");
  EXPECT_EQ(d.node, 0);
  EXPECT_EQ(d.kind, "motor");
  EXPECT_EQ(d.value, 0x1000u);
  EXPECT_EQ(names(d), V{"CURRENT_LIMIT_VIOLATION"});
}

TEST(Spec, MotorSystemLevel) {
  auto d = frame(0x003, "00 00 00 01 00 00 00 00");
  EXPECT_EQ(d.value, 0x1000000u);
  EXPECT_EQ(names(d), V{"SYSTEM_LEVEL"});
}

TEST(Spec, MotorUnbalancedPhasesBit35OByte4) {
  auto d = frame(0x003, "00 00 00 00 08 00 00 00");
  EXPECT_EQ(d.value, 0x800000000ULL);
  EXPECT_EQ(names(d), V{"UNBALANCED_PHASES"});
}

TEST(Spec, EncoderIllegalHall) {
  auto d = frame(0x004, "10 00 00 00 00 00 00 00");
  EXPECT_EQ(d.node, 0);
  EXPECT_EQ(d.kind, "encoder");
  EXPECT_EQ(d.value, 0x10u);
  EXPECT_EQ(names(d), V{"ILLEGAL_HALL_STATE"});
}

TEST(Spec, EncoderCleared) {
  auto d = frame(0x004, "00 00 00 00 00 00 00 00");
  EXPECT_EQ(d.kind, "encoder");
  EXPECT_TRUE(d.cleared);
  EXPECT_TRUE(d.errors.empty());
}

TEST(Spec, HbPhaiWatchdog) {
  auto d = frame(0x041, "00 08 00 01 00 00 00 00");
  EXPECT_EQ(d.node, 2);
  EXPECT_EQ(d.state_name, "IDLE");
  EXPECT_EQ(d.axis_error, 0x800u);
  EXPECT_EQ(names(d), V{"WATCHDOG_TIMER_EXPIRED"});
}

// ---------------------------------------------------------------- quy tac khac trong spec
TEST(Spec, BitLaVaStateLa) {
  V n;
  for (const auto& e : E::decode_bits(0x8 | 0x2, E::AXIS_ERRORS)) n.push_back(e.name);
  EXPECT_EQ(n, (V{"UNKNOWN_BIT_1", "UNKNOWN_BIT_3"}));
  EXPECT_EQ(E::state_name(5), "UNKNOWN_STATE(5)");
}

TEST(Spec, HbByteDuPhongKhac0) { EXPECT_TRUE(frame(0x001, "00 00 00 08 01 00 00 00").reserved_nonzero); }

TEST(Spec, BoQuaFrameKhongLienQuan) {
  EXPECT_FALSE(E::decode_frame(0x009, hx("0000000000000000")));        // encoder estimates
  EXPECT_FALSE(E::decode_frame(0x01E, hx("02")));                      // staged ACK
  EXPECT_FALSE(E::decode_frame(0x7FF, hx("00")));                      // SYNC_GO broadcast
  EXPECT_FALSE(E::decode_frame(0x001, hx("0000000000000000"), true));  // extended
  EXPECT_FALSE(E::decode_frame(0x003, {}, false, true));               // RTR
  EXPECT_FALSE(E::decode_frame(0x003, hx("00 10 00 00")));             // motor can 8 byte -> DLC thieu
  EXPECT_FALSE(E::decode_frame(0x001, hx("00 00")));                   // heartbeat can >= 4 byte
}

TEST(Spec, SensorlessVaMotor64bitNhieuBit) {
  auto d = frame(0x005, "03 00 00 00 00 00 00 00");
  EXPECT_EQ(d.kind, "sensorless");
  EXPECT_EQ(names(d), (V{"UNSTABLE_GAIN", "UNKNOWN_CURRENT_MEASUREMENT"}));
  d = frame(0x043, "08 00 00 10 00 00 00 00");  // node 2: DRV_FAULT | UNKNOWN_TORQUE (bit 28)
  EXPECT_EQ(d.node, 2);
  EXPECT_EQ(names(d), (V{"DRV_FAULT", "UNKNOWN_TORQUE"}));
}

// ---------------------------------------------------------------- doc log candump (replay)
TEST(Candump, Parse) {
  auto r = E::parse_candump_line("(1690000000.123456) can0 041#0002000100000000");
  ASSERT_TRUE(r);
  EXPECT_DOUBLE_EQ(r->t, 1690000000.123456);
  EXPECT_EQ(r->arb, 0x041u);
  EXPECT_EQ(r->data, hx("0002000100000000"));
  EXPECT_FALSE(r->is_extended);
  EXPECT_FALSE(r->is_remote);
  r = E::parse_candump_line("(1.5) can0 005#R");
  ASSERT_TRUE(r);
  EXPECT_EQ(r->arb, 0x005u);
  EXPECT_TRUE(r->data.empty());
  EXPECT_TRUE(r->is_remote);
  EXPECT_TRUE(E::parse_candump_line("(2.0) can0 12345678#00")->is_extended);
  EXPECT_FALSE(E::parse_candump_line("rac khong phai candump"));
}

// ---------------------------------------------------------------- protocol: ten loi ngan (giong protocol.py)
TEST(Protocol, MoTaLoi) {
  EXPECT_EQ(zltech::ten_loi(0x240, E::AXIS_ERRORS), "MOTOR_FAILED|CONTROLLER_FAILED");
  EXPECT_EQ(zltech::ten_loi(0x802, E::AXIS_ERRORS), "WATCHDOG_TIMER_EXPIRED|0x2");
  EXPECT_EQ(zltech::mo_ta_loi(0, {}), "");
  zltech::Details d;
  d.encoder = 0x10;
  EXPECT_EQ(zltech::mo_ta_loi(0x100, d), "ENCODER_FAILED; encoder: ILLEGAL_HALL_STATE");
}
