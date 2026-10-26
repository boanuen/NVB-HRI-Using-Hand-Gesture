# Điều khiển TurtleBot 4 bằng cử chỉ tay (Hand Gesture Control)

Hệ thống nhận diện cử chỉ tay qua webcam bằng MediaPipe, điều khiển TurtleBot 4 trong Gazebo thông qua ROS 2 Humble.

**Bài tập 6** — Giao tiếp phi ngôn ngữ và điều khiển TurtleBot 4 bằng cử chỉ tay.

## Video Demo

> [Link video demo](https://drive.google.com/...)

## Kiến trúc hệ thống

```
┌─────────┐     ┌─────────────────────┐     /detected_gesture     ┌────────────────────┐     /cmd_vel     ┌────────────┐
│ Webcam  │────▶│ gesture_recognition │────────────────────────▶│ robot_behavior     │──────────────▶│ TurtleBot 4│
│         │     │ _node               │     (std_msgs/String)    │ _node              │   (Twist)      │ (Gazebo)   │
└─────────┘     └─────────────────────┘                          └────────────────────┘                └────────────┘
                  • MediaPipe Hands                                • State machine:
                  • OpenCV (MJPEG)                                   IDLE → SELECTED → EXECUTING
                  • Debounce 8 frame                               • Safety timeout 10s
                  • Cooldown 1s
```

### ROS 2 Nodes & Topics

| Node | Chức năng | Publish | Subscribe |
|------|-----------|---------|-----------|
| `gesture_recognition_node` | Nhận diện cử chỉ tay từ webcam | `/detected_gesture` (String) | — |
| `robot_behavior_node` | Điều khiển robot theo cử chỉ | `/cmd_vel` (Twist) | `/detected_gesture` (String) |

## Bảng ánh xạ cử chỉ — Hành vi robot

| Cử chỉ | Hình minh họa | Hành vi robot | Cần xác nhận? |
|---------|--------------|---------------|---------------|
| **Pointing Up** (giơ 1 ngón trỏ) | ☝️ | Chọn hành vi: tiến–lùi 4 lần | Cần Thumbs Up |
| **Victory** (giơ 2 ngón) | ✌️ | Chọn hành vi: di chuyển theo quỹ đạo tròn | Cần Thumbs Up |
| **Thumbs Up** (giơ ngón cái) | 👍 | Xác nhận và thực hiện hành vi đã chọn | — |
| **Open Palm** (xòe bàn tay) | 🖐️ | Dừng robot ngay lập tức (emergency stop) | Không |
| **Closed Fist** (nắm tay) | ✊ | Hủy hành vi hiện tại, trở về trạng thái chờ | Không |
| **Wave** (vẫy tay) | 👋 | Robot xoay trái–phải để chào lại | Không |

## Luồng hoạt động

```
1. Giơ Pointing Up hoặc Victory    →  Chọn hành vi (IDLE → SELECTED)
2. Giơ Thumbs Up                   →  Xác nhận (SELECTED → EXECUTING)
3. Robot thực hiện hành vi         →  Tự động kết thúc (→ IDLE)
4. Open Palm bất cứ lúc nào       →  Dừng khẩn cấp (→ IDLE)
5. Closed Fist bất cứ lúc nào     →  Hủy (→ IDLE)
6. Wave                            →  Chào ngay (không cần xác nhận)
```

## Cấu trúc thư mục

```
gesture_control/
├── package.xml                        # Metadata ROS 2 package
├── setup.py                           # Entry points & install config
├── setup.cfg
├── resource/
│   └── gesture_control                # ament index marker
├── gesture_control/
│   ├── __init__.py
│   ├── gesture_node.py                # Node nhận diện cử chỉ (MediaPipe + OpenCV)
│   └── behavior_node.py               # Node điều khiển robot (state machine)
├── launch/
│   └── gesture_control.launch.py      # Launch cả 2 node
└── README.md
```

## Cơ chế chống nhiễu

- **Debounce**: Cử chỉ phải ổn định liên tục 8 frame (~0.5 giây) mới được publish.
- **Cooldown**: Tối thiểu 1 giây giữa 2 lần publish gesture khác nhau.
- **Bỏ qua frame mơ hồ**: Khi tay ở tư thế không rõ ràng, hệ thống giữ nguyên trạng thái cũ thay vì publish "Unknown".
- **Safety timeout**: Robot tự dừng nếu không nhận được tín hiệu gesture trong 10 giây.

## Yêu cầu hệ thống

- Ubuntu 22.04 (WSL2)
- ROS 2 Humble
- TurtleBot 4 Simulation (`ros-humble-turtlebot4-ignition-bringup`)
- Webcam laptop (cần mount vào WSL2)
- Python 3.10

## Cài đặt

### 1. Cài usbipd-win (Windows) — để mount webcam vào WSL2

Mở **PowerShell (Admin)** trên Windows:

```powershell
winget install --exact dorssel.usbipd-win
```

### 2. Mount webcam vào WSL2

**PowerShell (Admin):**

```powershell
# Liệt kê thiết bị USB → tìm webcam
usbipd list

# Bind và attach (thay <busid> bằng ID webcam, ví dụ: 1-5)
usbipd bind --busid <busid>
usbipd attach --wsl --busid <busid>
```

**Trong WSL2:**

```bash
# Kiểm tra webcam đã mount
ls /dev/video*

# Cấp quyền truy cập
sudo chmod 666 /dev/video0 /dev/video1
```

### 3. Cài thư viện Python

```bash
pip3 install 'mediapipe==0.10.14' 'numpy==1.24.4' 'opencv-contrib-python>=4.8,<4.11'
```

### 4. Clone và build

```bash
cd ~/tb4_ws/src
git clone https://github.com/<your-username>/gesture_control.git

cd ~/tb4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select gesture_control --symlink-install
source install/setup.bash
```

## Chạy chương trình

### Terminal 1 — Gazebo + TurtleBot 4

```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py world:=warehouse
```

Đợi Gazebo khởi động hoàn tất (~30–60 giây).

### Terminal 2 — Undock robot + tắt safety override

```bash
source /opt/ros/humble/setup.bash

# Undock robot ra khỏi dock sạc
ros2 action send_goal /undock irobot_create_msgs/action/Undock {}

# Tắt giới hạn lùi (cần cho hành vi tiến-lùi)
ros2 param set /motion_control safety_override full
```

### Terminal 3 — Gesture Control

```bash
cd ~/tb4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch gesture_control gesture_control.launch.py
```

Cửa sổ **"Gesture Recognition"** sẽ hiện lên hiển thị webcam + kết quả nhận diện.

### (Tùy chọn) Terminal 4 — Theo dõi gesture

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /detected_gesture
```

## Giải thích kỹ thuật

### gesture_node.py — Nhận diện cử chỉ

- Đọc webcam qua **OpenCV** với backend **V4L2 + MJPEG** (tối ưu cho WSL2).
- Dùng **MediaPipe Hands** để phát hiện 21 điểm landmark trên bàn tay.
- Phân loại cử chỉ bằng thuật toán **distance ratio**: so sánh khoảng cách đầu ngón tay đến cổ tay với khoảng cách khớp MCP đến cổ tay — ngón duỗi nếu tip xa hơn.
- Phát hiện **Thumbs Up** riêng: kiểm tra ngón cái hướng lên trên (tip.y < mcp.y) + các ngón khác nắm.
- Phát hiện **Wave**: theo dõi vị trí x cổ tay, đếm số lần đổi hướng (oscillation ≥ 3 lần).
- Hiển thị: tên cử chỉ, độ tin cậy (%), thanh progress debounce.

### behavior_node.py — Điều khiển robot

- **State machine** 3 trạng thái: `IDLE` → `SELECTED` → `EXECUTING`.
- Robot chỉ thực hiện sau khi nhận **Thumbs Up** xác nhận (trừ Wave/Stop/Cancel).
- Gửi lệnh `/cmd_vel` ở tần số **10 Hz** cho chuyển động mượt.
- Tốc độ thấp: linear 0.2 m/s, angular 0.8 rad/s — an toàn trong mô phỏng.
- **Safety timeout**: tự dừng sau 10 giây không nhận gesture.
