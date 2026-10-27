# Bài 6 — Điều khiển TurtleBot 4 bằng cử chỉ tay

Dùng webcam laptop/USB với **OpenCV + MediaPipe Gesture Recognizer**. Hai ROS 2 node:

- `gesture_recognition_node`: webcam → AI → `/detected_gesture` (`std_msgs/msg/String`).
- `robot_behavior_node`: chọn/xác nhận hành vi → `/cmd_vel` (`geometry_msgs/msg/Twist`).

## 1. Cài đặt một lần

Trong Ubuntu 22.04 + ROS 2 Humble đã chạy bài trước:

```bash
cd ~/workspaces/assign4
./scripts/install_gesture_deps.bash
./scripts/build.bash
```

Nếu thiếu venv: `sudo apt install python3-venv`. Script tạo `.venv-gesture` riêng,
cài MediaPipe 0.10.21, NumPy 1.x, OpenCV contrib 4.11 và tải model chính thức Google
`gesture_recognizer.task` vào `src/assign4_bringup/models/`. Không sửa thư viện YOLO toàn máy.
Không commit venv/model; máy mới tải lại bằng script.

## 2. Chạy hai terminal

Dừng các phiên Nav2, teleop, autonomy tuần trước bằng Ctrl+C. Chỉ một bộ điều khiển được
publish vận tốc; không chạy `run_autonomy.bash` trong bài này. Bài 6 không cần map/AMCL.

Terminal 1 — mô phỏng phòng trống rộng 10 × 10 m:

```bash
cd ~/workspaces/assign4
./scripts/run.bash simulation world:=gesture_arena model:=lite rviz:=false
```

Chờ controller khởi động. Terminal 2 — nhả dock trước, sau đó chạy cử chỉ:

```bash
cd ~/workspaces/assign4
./scripts/prepare_gesture_demo.bash
./scripts/run_gestures.bash
```

Script chuẩn bị nhả robot rồi xóa riêng model dock trong world `gesture_arena` để dock không
cản hành vi tiến/lùi. Đây là bước chuẩn bị, không phải điều khiển bằng phím trong demo. Sau đó
chờ LiDAR cập nhật vài giây và bắt đầu quay webcam + Gazebo. Cửa sổ OpenCV
hiển thị landmark, tên cử chỉ, confidence và trạng thái hành vi. Không dùng phím điều khiển.

Nếu laptop có nhiều camera:

```bash
ls -l /dev/video*
./scripts/run_gestures.bash camera_index:=2
```

Nếu dùng WSL/container, cần chuyển tiếp webcam để Ubuntu thấy `/dev/video*` trước. Camera RGB
trên robot mô phỏng không thay thế webcam laptop trong bài này.

## 3. Cách ra lệnh

| Cử chỉ | Label gửi lên ROS | Tác dụng |
|---|---|---|
| Vẫy bàn tay ngang trái–phải ≥ 2 lần đổi hướng | `Wave` | Chọn hành vi robot xoay trái–phải chào |
| Một ngón trỏ giơ lên | `Pointing_Up` | Chọn tiến–lùi **4 chu kỳ** (8 đoạn), 0,10 m mỗi đoạn |
| Giơ hai ngón chữ V | `Victory` | Chọn đi một vòng tròn bán kính 0,40 m |
| Giơ ngón cái | `Thumb_Up` | Xác nhận và thực hiện hành vi đã chọn |
| Xòe tay đứng yên | `Open_Palm` | Dừng ngay, xóa lệnh đang chọn |
| Nắm tay | `Closed_Fist` | Hủy hành vi và về trạng thái chờ |

Giữ cử chỉ chọn khoảng 0,4 giây, sau đó giơ ngón cái. **Giữ ngón cái trong khung hình khi robot
chạy** để duy trì tín hiệu; bỏ tay khỏi webcam thì robot dừng. Chọn hành vi không làm robot chạy
cho tới khi nhận xác nhận. Vẫy tay cũng cần xác nhận; vẫy xong chuyển sang ngón cái ngay khi
hiện `SELECTED: greeting`.

Wave được nhận bằng Open_Palm từ AI kết hợp dịch chuyển cổ tay ngang và hai lần đảo chiều
trong 1,8 s. Đây là cử chỉ động, không giả lập bằng một tư thế khác. Đưa một bàn tay vào camera;
hai bàn tay hoặc confidence thấp được xem là `None` để tránh lệnh không rõ.

## 4. Cơ chế an toàn

- Nhãn chọn/xác nhận phải ổn định ít nhất 0,25 s; Open_Palm/Closed_Fist không chờ debounce.
- Giữ lặp ngón cái không khởi động lại hành vi. Muốn chạy lại phải chọn lại rồi xác nhận.
- Mất tín hiệu hợp lệ 0,9 s (mất tay, camera hỏng, node nhận diện chết) → publish vận tốc 0.
- Thiếu/không cập nhật `/scan`, `/odom` trong 3 s → dừng và xóa hành vi.
- Vật cản trước/sau gần hơn 0,50 m hoặc quanh robot khi xoay gần hơn 0,30 m → dừng.
- Sau khi dừng/hủy/mất tín hiệu, không tự chạy tiếp; phải chọn và xác nhận lại.
- Chiều dài đoạn và vòng tròn đo bằng odometry, tránh đếm thời gian sai khi Gazebo chậm.
- Không có chuyển động thực tế trong 10 s dù đang có lệnh → dừng để tránh mắc kẹt ở safety limit.
- Watchdog chạy theo đồng hồ thực, vẫn dừng khi `/clock` mô phỏng bị pause.

Thông số nằm trong `src/assign4_bringup/config/gesture.yaml`. Tốc độ mặc định 0,15 m/s,
xoay chào 0,55 rad/s. LiDAR là cơ chế dừng bổ sung; thực hiện ở sân trống và quan sát Gazebo.
Đoạn lùi ngắn 0,10 m để demo trong vùng nhỏ. Create 3 vẫn có thể giữ `BACKUP_LIMIT` sau Undock.
Vì vậy script chuẩn bị đặt riêng `safety_override=backup_only` trong **sân mô phỏng phẳng
gesture_arena**, cho phép lùi; không dùng `full`. Các bảo vệ khác và kiểm tra LiDAR phía sau
vẫn bật. Không dùng script này trên robot thật; script từ chối khi không có world gesture_arena.

## 5. Xem ảnh và topic

Nếu GUI OpenCV không dùng được, vẫn có ảnh với tên/confidence trên ROS:

```bash
# Terminal 2
./scripts/run_gestures.bash show_window:=false
# Terminal 3
source setup_local.bash
rqt_image_view /gesture/annotated_image
```

```bash
source setup_local.bash
ros2 topic echo /detected_gesture
ros2 topic echo /gesture_behavior/status
ros2 topic echo /cmd_vel
```

`STOP: odometry / LiDAR not ready or stale`: chờ Gazebo/scan/odom ổn định rồi chọn lại.
`STOP: obstacle too close`: robot đang gần dock/tường; xác nhận đã Undock thành công và còn
khoảng trống. Không tắt kiểm tra vật cản để ép robot chạy.

## 6. Nội dung quay/nộp

Quay webcam và Gazebo cùng màn hình: một ngón → ngón cái → tiến/lùi đủ 4 chu kỳ;
V → ngón cái → vòng tròn; vẫy tay → ngón cái → robot chào. Khi robot đang chạy, xòe tay để
dừng, nắm tay để hủy, sau đó thử bỏ tay khỏi camera để chứng minh mất tín hiệu cũng dừng.
Cho thấy robot đứng yên ở bước chọn, chỉ chạy ở bước xác nhận. Không dùng bàn phím để điều
khiển robot trong video. Upload Drive công khai; GitHub chứa source hai node, launch/config,
world và README; không cần upload venv/model/build/install/log.

## 7. Kiểm tra logic

```bash
python3 -m unittest discover -s tests -v
```

Test bao phủ chọn/xác nhận, chống lặp, dừng/hủy, mất tay, thiếu cảm biến, vật cản trước/sau,
đủ bốn chu kỳ, đủ một vòng tròn và phân biệt vẫy tay với bàn tay đứng yên.

Tài liệu: [MediaPipe Gesture Recognizer Python](https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer/python),
[OpenCV](https://docs.opencv.org/), [TurtleBot 4](https://turtlebot.github.io/turtlebot4-user-manual/).
[Create 3 backup safety](https://iroboteducation.github.io/create3_docs/api/safety/).
