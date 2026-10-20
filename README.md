# TurtleBot 4 — SLAM, Nav2 tự động và nhận diện YOLO

Workspace ROS 2 Humble mô phỏng **TurtleBot 4** trong Gazebo, xây dựng bản đồ bằng
**SLAM Toolbox**, định vị bằng **AMCL**, tự động đi qua nhiều điểm bằng **Nav2** và nhận diện
vật thể từ camera RGB bằng **YOLO**.

Môi trường mặc định là phòng `small_room` kích thước 8 × 6 m, có ba vật cản bất đối xứng và
các lối đi đủ rộng để thực hiện mapping và navigation. Bản đồ đã lưu nằm tại
`src/assign4_bringup/maps/map.yaml` và `map.pgm`.

## Chức năng đã thực hiện

- Khởi chạy TurtleBot 4 và môi trường `small_room` trong Gazebo.
- Publish và kiểm tra các topic cơ bản: `/scan`, `/odom`, `/cmd_vel`.
- Quan sát RobotModel, TF, LaserScan, bản đồ và costmap trên RViz2.
- Điều khiển robot bằng WASD hoặc các phím mũi tên.
- Xây dựng và lưu bản đồ bằng SLAM Toolbox.
- Nạp lại bản đồ và định vị robot bằng Map Server + AMCL.
- Đặt Nav2 Goal để robot tự lập đường, bám đường và tránh vật cản.
- ROS 2 Node tự gửi tuần tự ba goal cho action `/navigate_to_pose`.
- Hiển thị ba điểm mục tiêu trên RViz qua topic `/waypoints`.
- ROS 2 Node nhận ảnh OAK-D, chạy YOLO và vẽ bounding box, class, confidence.
- Publish ảnh nhận diện trên `/yolo/annotated_image` và in kết quả ra terminal.
- Hỗ trợ nhả robot khỏi dock trước khi localization/navigation.

## Cấu trúc thư mục

```text
.
|-- scripts/
|   |-- build.bash
|   |-- check_topics.bash
|   |-- install_yolo_deps.bash
|   |-- run.bash
|   |-- run_autonomy.bash
|   |-- save_map.bash
|   |-- teleop.bash
|   `-- undock.bash
|-- setup_local.bash
`-- src/assign4_bringup/
    |-- config/
    |   |-- slam.yaml
    |   |-- localization.yaml
    |   |-- nav2.yaml
    |   `-- autonomy.yaml
    |-- launch/
    |   |-- assign4.launch.py
    |   `-- autonomy.launch.py
    |-- maps/
    |   |-- map.pgm
    |   `-- map.yaml
    |-- rviz/
    |-- scripts/
    |   |-- auto_explore.py
    |   |-- wasd_teleop.py
    |   |-- waypoint_navigator.py
    |   `-- yolo_detector.py
    `-- worlds/small_room.sdf
```

## Yêu cầu

- Ubuntu 22.04.
- ROS 2 Humble Desktop.
- TurtleBot 4 Simulator, TurtleBot 4 Navigation.
- SLAM Toolbox, Navigation2 và RViz2.
- Python 3, `cv_bridge`, OpenCV và Ultralytics YOLO.

Cài các package cần thiết:

```bash
sudo apt update
sudo apt install \
  ros-humble-turtlebot4-simulator \
  ros-humble-turtlebot4-navigation \
  ros-humble-turtlebot4-desktop \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-simple-commander \
  ros-humble-cv-bridge \
  ros-humble-teleop-twist-keyboard
```

Cài thư viện YOLO. Script giữ NumPy ở phiên bản 1.x để tương thích với `cv_bridge` của ROS 2
Humble:

```bash
./scripts/install_yolo_deps.bash
```

## Tải mã nguồn và build

```bash
git clone -b assignments_4 \
  https://github.com/Shion1207/UR_Robot_Drawing.git assign4
cd assign4
./scripts/build.bash
```

Sau khi sửa source, chạy lại `./scripts/build.bash` trước khi khởi động chương trình.

## Kiểm tra mô phỏng và topic

Terminal 1:

```bash
cd ~/workspaces/assign4
./scripts/run.bash simulation
```

Terminal 2:

```bash
cd ~/workspaces/assign4
./scripts/check_topics.bash
```

Kết quả hợp lệ cần có dữ liệu trên `/scan`, `/odom`; topic `/cmd_vel` phải tồn tại.

## Xây dựng bản đồ

Terminal 1:

```bash
cd ~/workspaces/assign4
./scripts/run.bash mapping
```

Terminal 2:

```bash
cd ~/workspaces/assign4
./scripts/teleop.bash
```

Phím điều khiển:

- `W` hoặc `↑`: tiến.
- `S` hoặc `↓`: lùi.
- `A` hoặc `←`: quay trái.
- `D` hoặc `→`: quay phải.
- `Space` hoặc `X`: dừng ngay.
- `+`/`-`: tăng hoặc giảm tốc độ.
- `Q`: thoát teleop.

Sau khi robot đã khảo sát đủ phòng và quay lại các vùng cũ để khép vòng, dừng robot rồi mở
terminal thứ ba:

```bash
cd ~/workspaces/assign4
./scripts/save_map.bash
```

Lệnh trên ghi đè hai tệp:

```text
src/assign4_bringup/maps/map.yaml
src/assign4_bringup/maps/map.pgm
```

## Localization và navigation

Dừng hoàn toàn phiên mapping trước khi chạy navigation:

```bash
cd ~/workspaces/assign4
./scripts/run.bash navigation model:=lite
```

TurtleBot 4 simulator khởi tạo robot trên dock. Sau khi Gazebo và các node đã sẵn sàng, mở
terminal khác và nhả dock:

```bash
cd ~/workspaces/assign4
./scripts/undock.bash
```

Trong RViz2:

1. Chọn **2D Pose Estimate** và đặt đúng vị trí/hướng hiện tại của robot.
2. Chờ RobotModel và LaserScan khớp với bản đồ.
3. Chọn **Nav2 Goal**, nhấn vào vùng trống và kéo mũi tên để đặt hướng đích.
4. Chờ robot hoàn thành goal trước khi đặt goal tiếp theo.
5. Không dùng teleop trong lúc Nav2 đang điều khiển robot.

Goal nên cách tường, dock và vật cản ít nhất khoảng 0,4–0,5 m. Cấu hình `model:=lite` giúp giảm
tải mô phỏng; có thể bỏ đối số này nếu máy đủ mạnh và cần dùng mô hình TurtleBot 4 Standard.

## Chạy tự động qua ba goal và nhận diện YOLO

Terminal 1, khởi động Gazebo, AMCL và Nav2:

```bash
cd ~/workspaces/assign4
./scripts/run.bash navigation model:=lite
```

Terminal 2, nhả robot khỏi dock:

```bash
cd ~/workspaces/assign4
./scripts/undock.bash
```

Trong RViz, dùng **2D Pose Estimate** đặt đúng vị trí và hướng ban đầu. Chỉ chạy bước tiếp theo
khi LaserScan khớp với tường trên bản đồ.

Terminal 3, chạy đồng thời Node điều hướng và Node YOLO:

```bash
cd ~/workspaces/assign4
./scripts/run_autonomy.bash
```

Node điều hướng chờ `/amcl_pose`, sau đó tự gửi lần lượt ba goal trong
`src/assign4_bringup/config/autonomy.yaml`. Không cần và không được bấm **Nav2 Goal** trong lúc
chuỗi tự động đang chạy. Các goal mặc định là:

| Điểm | x (m) | y (m) | yaw (rad) |
|---|---:|---:|---:|
| Goal 1 | 2.6 | -1.9 | 1.57 |
| Goal 2 | 2.7 | 2.1 | 3.14 |
| Goal 3 | -1.0 | 2.2 | -1.57 |

YOLO đọc camera từ `/oakd/rgb/preview/image_raw`, mở cửa sổ OpenCV và publish ảnh kết quả lên
`/yolo/annotated_image`. Nhấn `Q` khi cửa sổ YOLO đang được chọn để chỉ đóng phần hiển thị.

Môi trường `small_room` có thêm một mô hình ghế gần phía bắc phòng để thử nhận diện. Đây là vật
cản mới chưa có trong bản đồ tĩnh; local costmap của Nav2 sẽ nhận nó từ LiDAR và tránh khi chạy.

Nếu chỉ cần chạy một phần:

```bash
# Chỉ chạy ba navigation goal
./scripts/run_autonomy.bash yolo:=false

# Chỉ chạy YOLO
./scripts/run_autonomy.bash navigation:=false

# Không mở cửa sổ OpenCV, chỉ publish ảnh kết quả
./scripts/run_autonomy.bash show_window:=false
```

Kiểm tra topic ảnh:

```bash
ros2 topic hz /oakd/rgb/preview/image_raw
ros2 topic hz /yolo/annotated_image
```

Lần đầu dùng `yolov8n.pt`, Ultralytics có thể tải weight từ Internet. Có thể truyền đường dẫn
đến weight đã tải sẵn:

```bash
./scripts/run_autonomy.bash model_path:=/duong/dan/yolov8n.pt
```

Trên máy hiện tại, script tự sử dụng `~/yolo_project/yolov8n.pt` nếu file này tồn tại, nên không
cần tải lại model.

## Các chế độ chạy

```bash
./scripts/run.bash simulation
./scripts/run.bash mapping
./scripts/run.bash localization
./scripts/run.bash navigation model:=lite
```

Có thể chọn world khác bằng launch argument, ví dụ:

```bash
./scripts/run.bash mapping world:=maze
```

Các world hỗ trợ: `small_room`, `depot`, `maze`, `warehouse`.

## Thông tin bản đồ hiện tại

- Kích thước: 158 × 118 ô.
- Độ phân giải: 0,05 m/ô.
- Origin: `[-3.95, -2.96, 0]`.
- Ngưỡng occupied: `0.65`.
- Ngưỡng free: `0.25`.

## Lưu ý

- Không chạy đồng thời nhiều phiên mapping/navigation vì có thể gây trùng controller và topic.
- Nếu `undock.bash` báo action chưa sẵn sàng, chờ Gazebo khởi động thêm khoảng 10 giây rồi thử lại.
- Nếu Node tự động cứ báo chưa có `/amcl_pose`, cần đặt lại **2D Pose Estimate** trong RViz.
- Không chạy teleop đồng thời với chuỗi goal tự động vì hai nguồn cùng ghi lệnh vận tốc.
