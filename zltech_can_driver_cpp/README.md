# zltech_can_driver_cpp — bản C++ của zltech_can_driver

Chuyển từ package Python `zltech_can_driver`: cùng giao thức CAN, cùng tên chương trình,
tham số yaml, topic, service, log và logic an toàn. Hai bản **dùng chung kiểu khóa cổng**,
nên không thể chạy cùng lúc trên một adapter.

| Python (`zltech_can_driver`) | C++ (`zltech_can_driver_cpp`) |
|---|---|
| `protocol.py`, `odrive_can_errors.py` | `src/protocol.cpp`, `src/odrive_can_errors.cpp` (+ `include/`) |
| `can_bridge.py` | `src/can_bridge.cpp` |
| `teleop.py`, `calib.py`, `clear_errors.py`, `drive_test.py`, `error_monitor.py` | `src/<cùng tên>.cpp` |
| `test/test_odrive_can_errors.py` (pytest) | `test/test_odrive_can_errors.cpp` (gtest) |
| python-can (`seeedstudio`, `socketcan`…) | tự viết: serial Waveshare (termios) + SocketCAN |

## Build
```bash
cd ~/Desktop/RobotClient/robot_mini_zltech
colcon build --packages-select zltech_can_driver_cpp
source install/setup.bash
colcon test --packages-select zltech_can_driver_cpp && colcon test-result --verbose
```

## Chạy (giống bản Python, chỉ đổi tên package)
```bash
ros2 launch zltech_can_driver_cpp can_bridge.launch.py          # bridge /cmd_vel -> 2 banh
ros2 run zltech_can_driver_cpp teleop                           # lai bang ban phim
ros2 run zltech_can_driver_cpp clear_errors [--xem]
ros2 run zltech_can_driver_cpp calib [--chi=PHAI] [--kieu=full|motor|encoder]   # KE BANH LEN
ros2 run zltech_can_driver_cpp drive_test [--dao=PHAI]          # lai thu khong ROS
ros2 run zltech_can_driver_cpp error_monitor [--replay candump.log]
ros2 service call /can_bridge/enable std_srvs/srv/Trigger
```

## Khác bản Python
- Adapter: `waveshare` và `socketcan`. `gs_usb` / `slcan` thì đi qua SocketCAN
  (`gs_usb` → kernel tạo `can0`; `slcan` → `sudo slcand -o -s5 /dev/ttyACM0 slcan0`).
- `can_bridge`: gửi lệnh lỗi (rút adapter) được tính như thiếu ACK → sau 2 s về IDLE, thay vì làm chết vòng lặp.
  Thoát bằng Ctrl+C không in traceback.
- Các tool dòng lệnh bắt cả SIGTERM/SIGHUP (kill, đóng terminal), nên banh vẫn được đưa về IDLE.

## Đã kiểm tra (2026-11-04, không có phần cứng)
Dùng bộ giả lập Waveshare + 2 board ODrive trên pty, chạy cả hai bản với cùng kịch bản:
- Byte khởi tạo adapter, chuỗi lệnh enable/thoát và tốc độ gửi xuống **giống hệt** bản Python.
- Adapter đứng hình 0.8 s → hồi phục; mất liên lạc 3 s → IDLE rồi tự enable lại; banh runaway → IDLE + KHÓA.
- `clear_errors`, `error_monitor` (thật + replay), khóa cổng; 16 gtest qua.

**Chưa chạy trên robot thật.** Lần đầu nên kê bánh lên và thử `drive_test` trước.
