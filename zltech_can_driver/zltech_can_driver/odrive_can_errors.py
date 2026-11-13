"""odrive_can_errors.py - GIAI MA heartbeat + frame loi CAN cua firmware Firmware-PlatformIO-Can-sync (ODrive v3.6 fork,
patch err-event / collision / can-watchdog / ack-retry). Thu vien THUAN: khong phu thuoc phan cung, test bang pytest.

Theo can_heartbeat_error_spec.md. Heartbeat KHAC chuan ODrive 0.5.x:
    byte 0-2 axis_error (uint24 LE), byte 3 axis_state, byte 4-7 = 0 (khac 0 -> firmware la).
Frame loi (tu gui theo su kien: doi gia tri -> gui ngay; con loi -> lap lai 100 ms; het loi -> 1 frame toan 0):
    0x03 motor_error uint64 LE (bit toi 35, PHAI doc du 8 byte), 0x04 encoder / 0x1D controller uint32 LE.
    0x05 sensorless uint32 LE: chi tra loi khi host hoi (RTR).

    decode_bits(value, table) -> [{mask, bit, name, desc, hint}]
    decode_frame(arbitration_id, data, is_extended=False, is_remote=False) -> dict | None
"""

# ============================================================================================================
# Bang 3: axis_state
# ============================================================================================================
AXIS_STATES = {
    0: "UNDEFINED",
    1: "IDLE",
    2: "STARTUP_SEQUENCE",
    3: "FULL_CALIBRATION_SEQUENCE",
    4: "MOTOR_CALIBRATION",
    6: "ENCODER_INDEX_SEARCH",
    7: "ENCODER_OFFSET_CALIBRATION",
    8: "CLOSED_LOOP_CONTROL",
    9: "LOCKIN_SPIN",
    10: "ENCODER_DIR_FIND",
    11: "HOMING",
    12: "ENCODER_HALL_POLARITY_CALIBRATION",
    13: "ENCODER_HALL_PHASE_CALIBRATION",
}


def state_name(state):
    return AXIS_STATES.get(state, f"UNKNOWN_STATE({state})")


# ============================================================================================================
# Bang 5: ma loi {mask: (ten, y nghia, goi y xu ly)}
# ============================================================================================================
AXIS_ERRORS = {
    0x00000001: ("INVALID_STATE", "Yeu cau state khong hop le (vd vao CLOSED_LOOP khi motor/encoder chua calib).",
                 "Calib truoc roi moi chay."),
    0x00000040: ("MOTOR_FAILED", "Motor loi.", "Xem frame 0x03 motor_error."),
    0x00000080: ("SENSORLESS_ESTIMATOR_FAILED", "Bo uoc luong sensorless loi.", "Hoi frame 0x05 (RTR)."),
    0x00000100: ("ENCODER_FAILED", "Encoder loi.", "Xem frame 0x04 encoder_error."),
    0x00000200: ("CONTROLLER_FAILED", "Controller loi (vd va cham, overspeed).", "Xem frame 0x1D controller_error."),
    0x00000800: ("WATCHDOG_TIMER_EXPIRED",
                 "Qua watchdog_timeout khong nhan frame CAN nao (patch can-watchdog: bat khi vao CLOSED_LOOP qua CAN).",
                 "Master ngung gui lenh / mat CAN / adapter dung hinh. Kiem tra master con chay, day CAN; clear errors."),
    0x00001000: ("MIN_ENDSTOP_PRESSED", "Cong tac hanh trinh min bi nhan.", "Kiem tra endstop."),
    0x00002000: ("MAX_ENDSTOP_PRESSED", "Cong tac hanh trinh max bi nhan.", "Kiem tra endstop."),
    0x00004000: ("ESTOP_REQUESTED", "Co frame ESTOP (cmd 0x02) gui toi node.", "Tim ai gui ESTOP; clear errors."),
    0x00020000: ("HOMING_WITHOUT_ENDSTOP", "Homing nhung chua bat min endstop.", "Bat min endstop hoac bo homing."),
    0x00040000: ("OVER_TEMP", "(Khong dung o firmware nay).", "Xem motor_error (THERMISTOR_OVER_TEMP)."),
    0x00080000: ("UNKNOWN_POSITION", "Khong co uoc luong vi tri hop le.", "Kiem tra encoder / calib."),
}

MOTOR_ERRORS = {
    0x000000001: ("PHASE_RESISTANCE_OUT_OF_RANGE", "Dien tro pha do khi calib nam ngoai khoang hop ly.",
                  "Kiem tra day pha; chinh resistance_calib_max_voltage (> calibration_current x R, < 0.5 x Vbus) / "
                  "calibration_current."),
    0x000000002: ("PHASE_INDUCTANCE_OUT_OF_RANGE", "Dien cam pha ngoai khoang hop ly.",
                  "Kiem tra day pha; chinh resistance_calib_max_voltage / calibration_current."),
    0x000000008: ("DRV_FAULT", "Chip gate driver (DRV8301) bao loi.",
                  "Thuong do sut ap nguon gate driver khi dong lon, hoac ngan mach / FET hong."),
    0x000000010: ("CONTROL_DEADLINE_MISSED", "Vong dieu khien tre deadline.", ""),
    0x000000080: ("MODULATION_MAGNITUDE", "Dien ap bus khong du day dong yeu cau.",
                  "Toc do qua cao so voi Vbus, hoac resistance_calib_max_voltage > 1/2 Vbus khi calib."),
    0x000000400: ("CURRENT_SENSE_SATURATION", "Mach do dong bao hoa (current_lim > requested_current_range).",
                  "Board ZLTECH shunt 5 mOhm: kiem tra dung firmware / range."),
    0x000001000: ("CURRENT_LIMIT_VIOLATION", "Dong vuot current_lim + current_lim_margin (overshoot PI).",
                  "Tang current_lim_margin hoac giam bandwidth / gain."),
    0x000010000: ("MODULATION_IS_NAN", "Modulation ra NaN (loi tinh toan).", ""),
    0x000020000: ("MOTOR_THERMISTOR_OVER_TEMP", "Nhiet dien tro motor vuot temp_limit_upper.", "Cho nguoi, giam tai."),
    0x000040000: ("FET_THERMISTOR_OVER_TEMP", "Nhiet FET/inverter vuot temp_limit_upper.", "Cho nguoi, tan nhiet."),
    0x000080000: ("TIMER_UPDATE_MISSED", "Lo 1 su kien update timer (khong mong doi o firmware chuan).", ""),
    0x000100000: ("CURRENT_MEASUREMENT_UNAVAILABLE", "ADC khong lay mau dong kip.", ""),
    0x000200000: ("CONTROLLER_FAILED", "Motor bi disarm vi bo dieu khien (thuong FOC) loi.", ""),
    0x000400000: ("I_BUS_OUT_OF_RANGE", "Dong DC cua motor ngoai I_bus_hard_min..I_bus_hard_max.",
                  "Thuong do ham / regen qua manh."),
    0x000800000: ("BRAKE_RESISTOR_DISARMED", "Chay PWM khi dien tro xa dang bi disarm.",
                  "Reboot hoac clear errors; nguyen nhan goc thuong la loi he thong (bang 5.6)."),
    0x001000000: ("SYSTEM_LEVEL", "Motor bi disarm vi LOI HE THONG ODrive (khong doc duoc qua CAN).",
                  "Kiem tra nguon / Vbus (frame 0x17); xem bang loi he thong ben duoi."),
    0x002000000: ("BAD_TIMING", "Vong chinh lech nhip voi vong dieu khien motor (vong chinh bi ket).", ""),
    0x004000000: ("UNKNOWN_PHASE_ESTIMATE", "Khong co goc pha hop le.", "Encoder chua calib / chua can Hall."),
    0x008000000: ("UNKNOWN_PHASE_VEL", "Khong co van toc pha hop le.", ""),
    0x010000000: ("UNKNOWN_TORQUE", "Khong co torque setpoint hop le.", ""),
    0x020000000: ("UNKNOWN_CURRENT_COMMAND", "Khong co setpoint dong hop le.", "Cau hinh controller sai."),
    0x040000000: ("UNKNOWN_CURRENT_MEASUREMENT", "Khong co gia tri do dong hop le.", ""),
    0x080000000: ("UNKNOWN_VBUS_VOLTAGE", "Khong co gia tri Vbus hop le.", ""),
    0x100000000: ("UNKNOWN_VOLTAGE_COMMAND", "Khong co dien ap feedforward hop le.", ""),
    0x200000000: ("UNKNOWN_GAINS", "Chua co gain dong.", "Chay motor calib hoac set phase_resistance / phase_inductance."),
    0x400000000: ("CONTROLLER_INITIALIZING", "Gia tri noi bo khi controller chua san sang tao PWM.", ""),
    0x800000000: ("UNBALANCED_PHASES", "Cac pha motor khong can bang.",
                  "Day pha long / dut, cuon day hong: siet 3 day pha, do dien tro A-B, B-C, C-A."),
}

ENCODER_ERRORS = {
    0x0001: ("UNSTABLE_GAIN", "Bandwidth encoder PLL qua cao, khong on dinh.", "Giam encoder bandwidth."),
    0x0002: ("CPR_POLEPAIRS_MISMATCH", "CPR hoac so cap cuc sai so voi do duoc khi calib.",
             "Kiem tra cpr, pole_pairs; tang calib_scan_distance; ke banh len (cogging / ket co khi)."),
    0x0004: ("NO_RESPONSE", "Encoder khong phan hoi.", "Kiem tra day / cam dung chan."),
    0x0008: ("UNSUPPORTED_ENCODER_MODE", "Mode encoder khong ho tro.", ""),
    0x0010: ("ILLEGAL_HALL_STATE", "Hall doc ra trang thai cam (000/111). Nhieu hoac day hong.",
             "Board ZLTECH: Hall C o gpio11 phai PULL_UP (gpio11_mode = 1). Them tu 22 nF xuong GND neu nhieu."),
    0x0020: ("INDEX_NOT_FOUND_YET", "Chua tim thay xung index Z.", ""),
    0x0040: ("ABS_SPI_TIMEOUT", "Encoder SPI tuyet doi timeout.", ""),
    0x0080: ("ABS_SPI_COM_FAIL", "Loi giao tiep SPI.", ""),
    0x0100: ("ABS_SPI_NOT_READY", "Encoder SPI chua san sang.", ""),
    0x0200: ("HALL_NOT_CALIBRATED_YET", "Hall chua calib (polarity / phase).", "Chay calib Hall."),
}

CONTROLLER_ERRORS = {
    0x0001: ("OVERSPEED", "Toc do vuot vel_limit x vel_limit_tolerance.",
             "Tang vel_limit / tolerance; tai dot ngot cung co the gay."),
    0x0002: ("INVALID_INPUT_MODE", "input_mode khong hop le.", ""),
    0x0004: ("UNSTABLE_GAIN", "Bandwidth controller qua cao, khong on dinh.", ""),
    0x0008: ("INVALID_MIRROR_AXIS", "axis_to_mirror khong hop le.", ""),
    0x0010: ("INVALID_LOAD_ENCODER", "load_encoder_axis khong hop le.", ""),
    0x0020: ("INVALID_ESTIMATE", "Encoder khong cho ra vi tri / van toc tuyen tinh hop le.", ""),
    0x0040: ("INVALID_CIRCULAR_RANGE", "Encoder khong cho ra vi tri circular hop le.", ""),
    0x0080: ("SPINOUT_DETECTED", "Cong suat co va dien khong khop.",
             "Encoder truot hoac offset calib sai; index nhieu. Calib lai, kiem tra giac encoder."),
    0x0100: ("COLLISION_DETECTED",
             "[Patch custom] Co lenh chay nhung banh khong tien trong khi Iq cao suot collision_time -> vat can / "
             "ket banh. Motor bi nha.",
             "Go vat can roi clear errors."),
}

SENSORLESS_ERRORS = {
    0x0001: ("UNSTABLE_GAIN", "PLL sensorless khong on dinh.", ""),
    0x0002: ("UNKNOWN_CURRENT_MEASUREMENT", "Khong co gia tri do dong hop le.", ""),
}

# 5.6: loi he thong ODrive (odrv.error) - KHONG co frame CAN, chi de in goi y khi thay SYSTEM_LEVEL /
# BRAKE_RESISTOR_DISARMED
SYSTEM_ERRORS = {
    0x01: ("CONTROL_ITERATION_MISSED", "Lo vong dieu khien", ""),
    0x02: ("DC_BUS_UNDER_VOLTAGE", "Vbus thap hon nguong (pin yeu, sut ap khi tang toc)", ""),
    0x04: ("DC_BUS_OVER_VOLTAGE", "Vbus cao hon nguong (regen khi ham, khong co dien tro xa)", ""),
    0x08: ("DC_BUS_OVER_REGEN_CURRENT", "Dong hoan ve nguon qua lon", ""),
    0x10: ("DC_BUS_OVER_CURRENT", "Dong lay tu nguon qua lon", ""),
    0x20: ("BRAKE_DEADTIME_VIOLATION", "Vi pham deadtime dien tro xa", ""),
    0x40: ("BRAKE_DUTY_CYCLE_NAN", "Duty dien tro xa NaN", ""),
    0x80: ("INVALID_BRAKE_RESISTANCE", "Gia tri dien tro xa khong hop le", ""),
}
MOTOR_SYSTEM_HINT_MASK = 0x001000000 | 0x000800000     # SYSTEM_LEVEL | BRAKE_RESISTOR_DISARMED

# ============================================================================================================
# CAN id
# ============================================================================================================
CMD_HEARTBEAT = 0x01
CMD_ESTOP = 0x02
CMD_MOTOR_ERROR = 0x03
CMD_ENCODER_ERROR = 0x04
CMD_SENSORLESS_ERROR = 0x05
CMD_BUS_VOLTAGE_CURRENT = 0x17
CMD_CLEAR_ERRORS = 0x18
CMD_CONTROLLER_ERROR = 0x1D

# cmd -> (kind, bang, so byte payload)
ERROR_FRAMES = {
    CMD_MOTOR_ERROR: ("motor", MOTOR_ERRORS, 8),
    CMD_ENCODER_ERROR: ("encoder", ENCODER_ERRORS, 4),
    CMD_CONTROLLER_ERROR: ("controller", CONTROLLER_ERRORS, 4),
    CMD_SENSORLESS_ERROR: ("sensorless", SENSORLESS_ERRORS, 4),
}
# bit axis_error *_FAILED -> (kind, cmd frame chi tiet)
FAILED_TO_DETAIL = {
    0x40: ("motor", CMD_MOTOR_ERROR),
    0x80: ("sensorless", CMD_SENSORLESS_ERROR),
    0x100: ("encoder", CMD_ENCODER_ERROR),
    0x200: ("controller", CMD_CONTROLLER_ERROR),
}


def arb_id(node, cmd):
    return (node << 5) | cmd


def decode_bits(value, table):
    """Tach bitmask -> [{mask, bit, name, desc, hint}] theo thu tu bit tang dan. Bit la -> UNKNOWN_BIT_n."""
    out = []
    bit = 0
    while value >> bit:
        mask = 1 << bit
        if value & mask:
            name, desc, hint = table.get(mask, (f"UNKNOWN_BIT_{bit}", f"Bit la (0x{mask:X}) khong co trong bang.",
                                                "Kiem tra phien ban firmware."))
            out.append({"mask": mask, "bit": bit, "name": name, "desc": desc, "hint": hint})
        bit += 1
    return out


def bit_names(value, table):
    """0x140 -> 'MOTOR_FAILED|ENCODER_FAILED'; 0 -> ''."""
    return "|".join(e["name"] for e in decode_bits(value, table))


def decode_frame(arbitration_id, data, is_extended=False, is_remote=False):
    """Giai ma 1 frame. Tra ve None neu khong lien quan (cmd khac, extended, RTR, DLC thieu).
    heartbeat: {node, kind='heartbeat', raw, errors, axis_error, state, state_name, reserved, reserved_nonzero}
    loi:       {node, kind='motor'|'encoder'|'controller'|'sensorless', raw, errors, value, cleared}"""
    if is_extended or is_remote or arbitration_id > 0x7FF:
        return None
    data = bytes(data)
    node, cmd = arbitration_id >> 5, arbitration_id & 0x1F
    if node == 0x3F:                   # broadcast (SYNC_GO ...), khong phai trang thai
        return None
    if cmd == CMD_HEARTBEAT:
        if len(data) < 4:
            return None
        axis_error = int.from_bytes(data[0:3], "little")
        reserved = int.from_bytes(data[4:8], "little") if len(data) >= 8 else 0
        return {"node": node, "kind": "heartbeat", "raw": data, "axis_error": axis_error, "state": data[3],
                "state_name": state_name(data[3]), "errors": decode_bits(axis_error, AXIS_ERRORS),
                "reserved": reserved, "reserved_nonzero": reserved != 0}
    if cmd in ERROR_FRAMES:
        kind, table, nbytes = ERROR_FRAMES[cmd]
        if len(data) < nbytes:
            return None
        value = int.from_bytes(data[:nbytes], "little")
        return {"node": node, "kind": kind, "raw": data, "value": value, "cleared": value == 0,
                "errors": decode_bits(value, table)}
    return None


def table_for(kind):
    return {"axis": AXIS_ERRORS, "motor": MOTOR_ERRORS, "encoder": ENCODER_ERRORS,
            "controller": CONTROLLER_ERRORS, "sensorless": SENSORLESS_ERRORS}[kind]
