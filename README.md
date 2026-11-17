
## Pipeline

Webcam → OpenCV → DeepFace Emotion → lọc ổn định → `/detected_emotion`
→ `emotion_motion_node` → `/cmd_vel` → robot. LiDAR, odometry và TF phản hồi
cho bộ điều khiển; trạng thái trả về cửa sổ webcam.

| Nhãn | Hành vi |
|---|---|
| happy | Cong trái trong khi giữ nhãn hợp lệ |
| sad | Cong phải trong khi giữ nhãn hợp lệ |
| neutral | Dừng |
| angry | Lùi 0,15 m, một lần |
| surprise | Xoay 2π theo odometry, một lần |
| fear | Tiến 0,15 m, một lần |

DeepFace phân loại biểu hiện trên ảnh, không xác định chắc chắn cảm xúc bên trong.
Không mặt/nhiều mặt, confidence dưới 0,60, disgust hoặc ảnh suy luận quá cũ
được coi là không có lệnh. Nhãn cần ổn định 0,6 s. Không cần bước READY.
Sau một hành vi hữu hạn hoặc dừng an toàn, đổi biểu cảm hoặc trở về neutral
trước khi lặp lại. Happy/Sad vẫn bị giới hạn thời gian và các watchdog.

## Cài đặt

Máy phải có Ubuntu 22.04 và ROS 2 Humble. Cài TurtleBot 4 Simulation/Gazebo
Fortress theo hướng dẫn nhà cung cấp; các package mô phỏng cần có:
`turtlebot4_ignition_bringup`, `turtlebot4_description`,
`irobot_create_ignition_bringup`, `irobot_create_description`,
`irobot_create_ignition_plugins`, `turtlebot4_ignition_gui_plugins`,
`ros_ign_gazebo`, `ros_gz_bridge`, cùng controller và TF phụ thuộc.
Tham khảo: https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html

Thư mục vendor cục bộ không được đưa vào source. Trên máy mới phải cài các
package trên; chỉ copy source không thay thế việc cài simulator.
Đảm bảo có `python3-venv`, `python3-pip`, `python3-colcon-common-extensions`.

```bash
cd ~/workspaces/assign8_submission
./scripts/install_deps.bash
./scripts/build.bash
```

Installer tạo `.venv` và tải model Emotion vào `models/`; cần mạng khi cài lần đầu.
Không commit các thư mục này. Nếu đổi tên workspace thành `assign8`, thay lệnh cd tương ứng.

## Chạy chính thức

Terminal 1:

```bash
cd ~/workspaces/assign8_submission
./scripts/run_simulation.bash
```

Đợi robot và controller hoạt động. Terminal 2:

```bash
cd ~/workspaces/assign8_submission
./scripts/prepare_demo.bash
./scripts/run_emotions.bash
```

Chỉ chạy nhận diện sau khi chuẩn bị thành công. `prepare_demo.bash` là script
Undock/dọn dock, không phát nhãn biểu cảm và không điều khiển chuỗi quay video.
Nó đặt `backup_only` riêng cho Create 3 trong mô phỏng phẳng; không dùng trên robot thật.
Giữ đúng một mặt trước webcam, đủ sáng. Cửa sổ hiện AI/score, Published và trạng thái.
Không chạy đồng thời teleop/Nav2 hoặc nhiều phiên Gazebo.

## An toàn và chẩn đoán

- Neutral/mất mặt hoặc nhãn không hợp lệ: dừng; mất nhãn hợp lệ quá 1,5 s: dừng.
- LiDAR/odom quá 2,5 s không cập nhật hoặc thiếu TF scan → base_link: dừng.
- Khoảng cách trước/sau tối thiểu 0,50 m; chuyển động quay cần 0,35 m quanh robot.
- Không tiến triển 10 s hoặc hành vi quá 180 s: dừng.
- Các giới hạn dùng wall-time, không vô hiệu khi Gazebo pause. Đây không phải
  hệ thống an toàn được chứng nhận; không có Nav2 để tìm đường vòng.

`./scripts/stop.bash` yêu cầu dừng; Q/Esc thoát webcam, Ctrl+C tắt launch.
`expression timeout` là thiếu nhịp nhãn; `sensors not ready/stale` là thiếu cảm biến.
Đóng ứng dụng nặng, giảm chất lượng quay màn hình; không tắt watchdog để che lỗi.
Máy yếu có thể chạy mô phỏng không GUI bằng `headless:=true`, nhưng không còn cửa sổ Gazebo.
Mặc định domain ROS 88, partition Gazebo `assign8` để tách khỏi bài cũ.

## Kiểm thử và báo cáo

```bash
source setup_local.bash
.venv/bin/python -m unittest discover -s tests -v
```
