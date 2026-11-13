"""Unit test odrive_can_errors theo dung cac vi du muc 6 cua can_heartbeat_error_spec.md.

    cd src/zltech_can_driver && python3 -m pytest test -q
"""
import pytest

from zltech_can_driver import odrive_can_errors as E
from zltech_can_driver.error_monitor import parse_candump_line


def frame(arb, hexdata):
    return E.decode_frame(arb, bytes.fromhex(hexdata.replace(" ", "")))


def names(d):
    return [e["name"] for e in d["errors"]]


# ---------------------------------------------------------------- muc 6 (10 vi du)
def test_hb_phai_closed_loop_khong_loi():
    d = frame(0x041, "00 00 00 08 00 00 00 00")
    assert (d["node"], d["kind"], d["state_name"], d["axis_error"]) == (2, "heartbeat", "CLOSED_LOOP_CONTROL", 0)
    assert d["errors"] == [] and not d["reserved_nonzero"]


def test_hb_phai_idle_controller_failed():
    d = frame(0x041, "00 02 00 01 00 00 00 00")
    assert (d["node"], d["state_name"], d["axis_error"]) == (2, "IDLE", 0x200)
    assert names(d) == ["CONTROLLER_FAILED"]


def test_controller_collision():
    d = frame(0x05D, "00 01 00 00 00 00 00 00")
    assert (d["node"], d["kind"], d["value"]) == (2, "controller", 0x100)
    assert names(d) == ["COLLISION_DETECTED"]


def test_hb_trai_motor_va_encoder_failed():
    d = frame(0x001, "40 01 00 01 00 00 00 00")
    assert (d["node"], d["state_name"], d["axis_error"]) == (0, "IDLE", 0x140)
    assert names(d) == ["MOTOR_FAILED", "ENCODER_FAILED"]


def test_motor_current_limit_violation():
    d = frame(0x003, "00 10 00 00 00 00 00 00")
    assert (d["node"], d["kind"], d["value"]) == (0, "motor", 0x1000)
    assert names(d) == ["CURRENT_LIMIT_VIOLATION"]


def test_motor_system_level():
    d = frame(0x003, "00 00 00 01 00 00 00 00")
    assert d["value"] == 0x1000000 and names(d) == ["SYSTEM_LEVEL"]


def test_motor_unbalanced_phases_bit35_o_byte4():
    d = frame(0x003, "00 00 00 00 08 00 00 00")
    assert d["value"] == 0x800000000 and names(d) == ["UNBALANCED_PHASES"]


def test_encoder_illegal_hall():
    d = frame(0x004, "10 00 00 00 00 00 00 00")
    assert (d["node"], d["kind"], d["value"]) == (0, "encoder", 0x10)
    assert names(d) == ["ILLEGAL_HALL_STATE"]


def test_encoder_cleared():
    d = frame(0x004, "00 00 00 00 00 00 00 00")
    assert d["kind"] == "encoder" and d["cleared"] and d["errors"] == []


def test_hb_phai_watchdog():
    d = frame(0x041, "00 08 00 01 00 00 00 00")
    assert (d["node"], d["state_name"], d["axis_error"]) == (2, "IDLE", 0x800)
    assert names(d) == ["WATCHDOG_TIMER_EXPIRED"]


# ---------------------------------------------------------------- quy tac khac trong spec
def test_bit_la_va_state_la():
    assert [e["name"] for e in E.decode_bits(0x8 | 0x2, E.AXIS_ERRORS)] == ["UNKNOWN_BIT_1", "UNKNOWN_BIT_3"]
    assert E.state_name(5) == "UNKNOWN_STATE(5)"


def test_hb_byte_du_phong_khac_0():
    assert frame(0x001, "00 00 00 08 01 00 00 00")["reserved_nonzero"]


@pytest.mark.parametrize("arb,hexdata,kw", [
    (0x009, "00" * 8, {}),                         # encoder estimates -> bo qua
    (0x01E, "02", {}),                             # staged ACK -> bo qua
    (0x7FF, "00", {}),                             # SYNC_GO broadcast -> bo qua
    (0x001, "00" * 8, {"is_extended": True}),      # extended -> bo qua
    (0x003, "", {"is_remote": True}),              # RTR -> bo qua
    (0x003, "00 10 00 00", {}),                    # motor can 8 byte -> DLC thieu
    (0x001, "00 00", {}),                          # heartbeat can >= 4 byte
])
def test_bo_qua_frame_khong_lien_quan(arb, hexdata, kw):
    assert E.decode_frame(arb, bytes.fromhex(hexdata.replace(" ", "")), **kw) is None


def test_sensorless_va_motor_64bit_nhieu_bit():
    d = frame(0x005, "03 00 00 00 00 00 00 00")
    assert d["kind"] == "sensorless" and names(d) == ["UNSTABLE_GAIN", "UNKNOWN_CURRENT_MEASUREMENT"]
    d = frame(0x043, "08 00 00 10 00 00 00 00")   # node 2: DRV_FAULT | UNKNOWN_TORQUE (bit 28)
    assert d["node"] == 2 and names(d) == ["DRV_FAULT", "UNKNOWN_TORQUE"]


# ---------------------------------------------------------------- doc log candump (replay)
def test_parse_candump():
    assert parse_candump_line("(1690000000.123456) can0 041#0002000100000000") == \
        (1690000000.123456, 0x041, bytes.fromhex("0002000100000000"), False, False)
    assert parse_candump_line("(1.5) can0 005#R") == (1.5, 0x005, b"", False, True)
    assert parse_candump_line("(2.0) can0 12345678#00")[3] is True          # extended
    assert parse_candump_line("rac khong phai candump") is None
