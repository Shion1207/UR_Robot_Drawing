# Bài thực hành 06 — Điều khiển TurtleBot 4 bằng cử chỉ tay

Hệ thống giao tiếp phi ngôn ngữ: đọc **webcam laptop/USB** bằng OpenCV, nhận diện bàn tay bằng
**MediaPipe Gesture Recognizer**, gửi kết quả qua ROS 2 để điều khiển **TurtleBot 4 trong Gazebo**.
Robot chỉ thực hiện hành vi sau khi người dùng chọn và xác nhận bằng ngón cái.

Nhánh nộp bài: **`assignments_6`**. Package ROS 2 tiếp tục mang tên `assign4_bringup` để sử dụng
lại phần mô phỏng của các tuần trước. Bài 6 chạy chế độ simulation, không cần map hoặc Nav2.

## Chức năng

- Nhận 6 cử chỉ: Wave, Pointing Up, Victory, Thumb Up, Open Palm, Closed Fist.
- Hiển thị landmark, tên cử chỉ, confidence và trạng thái điều khiển trên ảnh webcam.
- Chọn hành vi, xác nhận và thực hiện một lần: tiến–lùi 4 chu kỳ, vòng tròn hoặc xoay chào.
- Lọc cử chỉ không ổn định; giữ nguyên ngón cái không kích hoạt lặp lại hành vi.
- Dừng/hủy bằng cử chỉ, dừng khi mất tay/webcam hoặc thiếu dữ liệu cảm biến.
- Dùng odometry để đo quãng đường/góc quay và LiDAR để dừng trước vật cản.

## Yêu cầu và cài đặt

- Ubuntu 22.04, ROS 2 Humble Desktop; TurtleBot 4 Simulator đã chạy được như bài trước.
- Webcam xuất hiện trong Ubuntu dưới dạng `/dev/video0` hoặc thiết bị tương ứng.
- Python 3, `python3-venv`, `curl` và `cv_bridge`.

Nếu cần bổ sung các gói cho môi trường ROS 2 Humble đã cài:

```bash
sudo apt update
sudo apt install python3-venv curl ros-humble-cv-bridge \
  ros-humble-turtlebot4-simulator ros-humble-turtlebot4-navigation
```

Tải nhánh bài 6 và build:

```bash
mkdir -p ~/workspaces
cd ~/workspaces
git clone -b assignments_6 https://github.com/Shion1207/UR_Robot_Drawing.git assign4
cd assign4
./scripts/install_gesture_deps.bash
./scripts/build.bash
```

Nếu đã có workspace `~/workspaces/assign4` đang chạy thì không clone đè lên thư mục đó.
Script cài đặt tạo `.venv-gesture` riêng, cài MediaPipe 0.10.21, OpenCV contrib 4.11, NumPy 1.x
và tải model chính thức Google vào `src/assign4_bringup/models/gesture_recognizer.task`.
Không cần cài YOLO cho bài này. Sau khi sửa source, chạy lại `./scripts/build.bash`.

## Chạy chương trình

Dừng các phiên Nav2, teleop và autonomy cũ bằng Ctrl+C trước khi chạy. Chỉ dùng một bộ điều
khiển vận tốc tại một thời điểm.

**Terminal 1 — khởi động Gazebo:**

```bash
cd ~/workspaces/assign4
./scripts/run.bash simulation world:=gesture_arena model:=lite rviz:=false
```

Chờ thấy `Configured and activated diffdrive_controller`.

**Terminal 2 — chuẩn bị robot và mở webcam:**

```bash
cd ~/workspaces/assign4
./scripts/prepare_gesture_demo.bash && ./scripts/run_gestures.bash
```

Script chuẩn bị nhả robot khỏi dock, dọn model dock trong sân `gesture_arena`, và đặt
`safety_override=backup_only` để cho phép lùi trên sân mô phỏng phẳng. Các kiểm tra LiDAR
trước/sau và các bảo vệ khác vẫn hoạt động. Chỉ dùng cấu hình này cho mô phỏng của bài thực
hành; không áp dụng script chuẩn bị lên robot thật.

Nếu camera 0 không phải webcam RGB:

```bash
ls -l /dev/video*
./scripts/run_gestures.bash camera_index:=2
```

Với WSL/container, cần chuyển tiếp webcam để Ubuntu thấy `/dev/video*` trước.

## Bảng cử chỉ và cách điều khiển

| Cử chỉ | Nhãn ROS | Tác dụng |
|---|---|---|
| Xòe tay, đưa ngang trái–phải–trái | `Wave` | Chọn robot xoay trái–phải chào |
| Giơ một ngón trỏ | `Pointing_Up` | Chọn tiến–lùi 4 chu kỳ (8 đoạn), 0,10 m/đoạn |
| Giơ hai ngón chữ V | `Victory` | Chọn đi một vòng tròn bán kính 0,40 m |
| Giơ ngón cái | `Thumb_Up` | Xác nhận và thực hiện hành vi đã chọn |
| Xòe bàn tay đứng yên | `Open_Palm` | Dừng ngay và xóa lựa chọn |
| Nắm tay | `Closed_Fist` | Hủy hành vi và trở về chờ |

Ví dụ: **chữ V → đợi `SELECTED: circle` → giơ ngón cái → `RUNNING: circle`**.
Chỉ giơ chữ V hoặc ngón trỏ thì robot chưa chạy. Phải xác nhận trong 8 giây, nếu không cần chọn lại.

Giữ ngón cái trong khung hình khi robot chạy để duy trì tín hiệu. Bỏ tay ra quá 0,9 giây làm
robot dừng. Sau khi dừng/hủy/hoàn thành, muốn chạy lại phải chọn rồi xác nhận lại.

Để vẫy tay, xòe tay và di chuyển cổ tay ngang qua lại 1–2 giây; đợi `SELECTED: greeting`, rồi
chuyển ngay sang ngón cái. Wave được nhận từ chuyển động qua thời gian, còn Open Palm đứng yên
là lệnh dừng. Chỉ đưa một bàn tay vào khung hình để tránh lệnh không rõ ràng.

## Node, topic và cấu trúc source

```text
Webcam -> gesture_recognition_node -> /detected_gesture
                                      |
                                      v
                              robot_behavior_node -> /cmd_vel -> TurtleBot 4
                                ^           |
                          /scan, /odom      +-> /gesture_behavior/status
```

| Topic | Kiểu dữ liệu | Nội dung |
|---|---|---|
| `/detected_gesture` | `std_msgs/msg/String` | Nhãn đã lọc hoặc `None` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Vận tốc tuyến tính/góc |
| `/scan` | `sensor_msgs/msg/LaserScan` | Khoảng cách vật cản |
| `/odom` | `nav_msgs/msg/Odometry` | Vị trí/hướng và chuyển động thực tế |
| `/gesture/annotated_image` | `sensor_msgs/msg/Image` | Webcam có landmark, nhãn và confidence |
| `/gesture_behavior/status` | `std_msgs/msg/String` | Trạng thái chọn/chạy/dừng/hoàn thành |

Các tệp chính của tuần 6:

```text
requirements-gesture.txt
scripts/install_gesture_deps.bash
scripts/prepare_gesture_demo.bash
scripts/run_gestures.bash
src/assign4_bringup/config/gesture.yaml
src/assign4_bringup/launch/gesture.launch.py
src/assign4_bringup/scripts/gesture_recognition_node.py
src/assign4_bringup/scripts/robot_behavior_node.py
src/assign4_bringup/scripts/gesture_logic.py
src/assign4_bringup/worlds/gesture_arena.sdf
tests/test_gesture_logic.py
tests/test_gesture_ros.py
```

Các tệp mô phỏng, build/setup và Undock được kế thừa từ các tuần trước. Tham số nhận diện,
vận tốc, chiều dài đoạn, bán kính vòng tròn và watchdog nằm trong `config/gesture.yaml`.

## Cơ chế an toàn và kiểm tra

- Ngưỡng confidence 0,65; cử chỉ chọn/xác nhận phải ổn định ít nhất 0,25 giây.
- Open Palm và Closed Fist không phải chờ bộ lọc thời gian.
- Mất cử chỉ hợp lệ 0,9 giây hoặc thiếu `/scan`/`/odom` 3 giây: dừng và xóa hành vi.
- Vật cản trước/sau gần hơn 0,50 m hoặc quanh robot khi quay gần hơn 0,30 m: dừng.
- Không có chuyển động đủ lớn trong 10 giây: dừng, tránh mắc kẹt ở giới hạn an toàn.
- Watchdog dùng đồng hồ thực; độ dài/góc hành vi dùng odometry để phù hợp khi Gazebo chạy chậm.

Chạy kiểm tra logic và giao tiếp ROS 2 (test ROS dùng domain riêng):

```bash
cd ~/workspaces/assign4
source setup_local.bash
python3 -m unittest discover -s tests -v
```

## Xử lý tình huống thường gặp

**`IDLE: selection expired`:** mới chọn hành vi nhưng chưa xác nhận trong 8 giây. Nắm tay,
chọn lại và đổi sang ngón cái.

**Kẹt ở bước chuẩn bị sau `Undock ... SUCCEEDED`:** có thể lệnh đặt tham số bị mất phản hồi.
Nhấn Ctrl+C riêng Terminal 2, giữ nguyên Gazebo rồi kiểm tra:

```bash
source setup_local.bash
timeout 15s ros2 param get /motion_control safety_override
```

Nếu đã là `backup_only`, chỉ chạy `./scripts/run_gestures.bash`; không chạy lại bước chuẩn bị
vì dock đã được dọn. Nếu vẫn là `none`, đặt lại bằng `timeout 15s ros2 param set /motion_control
safety_override backup_only` rồi kiểm tra giá trị trước khi chạy nhận diện.

**Không có cửa sổ OpenCV:** dùng topic ảnh để quan sát:

```bash
# Terminal 2
./scripts/run_gestures.bash show_window:=false
# Terminal 3
source setup_local.bash
rqt_image_view /gesture/annotated_image
```

**`STOP: odometry / LiDAR not ready or stale`:** chờ mô phỏng/cảm biến ổn định rồi chọn và xác
nhận lại. **`STOP: obstacle too close`:** kiểm tra khoảng trống và bước nhả/dọn dock.

## Nội dung video và file nộp

Video quay webcam cùng Gazebo, thể hiện chọn/xác nhận, tiến–lùi đủ 4 chu kỳ, vòng tròn, xoay
chào, xòe tay dừng, nắm tay hủy và bỏ tay khỏi webcam để dừng. Không điều khiển robot bằng phím
trong demo. Video Drive công khai và PDF báo cáo được nộp riêng.

Nhánh GitHub chứa source, launch/config, world, script cài đặt/chạy và tests. `.gitignore`
loại `build/`, `install/`, `log/`, `vendor/`, `.venv-gesture/`, model tải về, báo cáo và cache.
Máy mới tải lại model bằng script cài đặt, không cần upload môi trường Python lên GitHub.

Tài liệu: [MediaPipe Gesture Recognizer](https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer/python),
[OpenCV](https://docs.opencv.org/), [TurtleBot 4](https://turtlebot.github.io/turtlebot4-user-manual/),
[Create 3 Safety](https://iroboteducation.github.io/create3_docs/api/safety/).
