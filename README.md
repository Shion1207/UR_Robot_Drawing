# Bài thực hành 7 — Phân vùng bản đồ và điều hướng bằng giọng nói

Workspace **`assign7`** và package **`assign7_bringup`** độc lập với bài trước. Dùng lại bản đồ
`map.pgm`/`map.yaml`, world `small_room` và cấu hình AMCL/Nav2. Không chạy gesture/YOLO của assign4.

## Nội dung đã triển khai

1. Xử lý occupancy grid bằng OpenCV; ô chưa biết/vật cản không được dùng làm điểm đến.
2. Distance transform loại vùng cách vật cản dưới 0,40 m; phân chia vùng đi được theo khoảng
   cách đường đi từ ba seed (multi-source geodesic Voronoi), không đi xuyên tường.
3. Gán nhãn A — Phòng học; B — Phòng thí nghiệm; C — Hành lang. Lưu tên, alias, goal hợp lệ,
   diện tích và hash bản đồ trong JSON, cùng ảnh màu và ảnh nhãn.
4. Whisper đa ngôn ngữ chạy cục bộ, nhận tiếng Việt từ microphone; chuyển câu sang `/voice/text`.
5. ROS node phân tích toàn bộ lệnh trước khi gửi action `NavigateToPose`. Lệnh A đến B sẽ đến
   A rồi mới đến B; nếu đang ở nơi khác cũng không bỏ qua A. Hỗ trợ đi qua nhiều khu.
6. Từ chối khu không tồn tại, câu không rõ, chưa localization hoặc lệnh mới khi đang bận.
   Nói “Dừng lại” để yêu cầu Nav2 hủy; không tự chạy goal kế tiếp sau khi hủy/thất bại.

Đây là phân vùng chức năng của bản đồ phòng mở có vật cản, không khẳng định các vùng là các
phòng vật lý khép kín. Nav2 đảm nhiệm tìm đường và tránh vật cản từ map/LiDAR.

## Cấu trúc chính

```text
assign7/
├── scripts/               # Cài đặt, build, phân vùng, chạy và Undock
├── setup_local.bash
├── requirements.txt
├── tests/
└── src/assign7_bringup/
    ├── config/            # AMCL, Nav2, seed/names các khu
    ├── launch/            # navigation.launch.py, voice.launch.py
    ├── maps/              # map.pgm, map.yaml gốc
    ├── semantic/          # regions.json, region_labels.png, segmented_map.png
    ├── scripts/           # Phân vùng, semantic markers, ASR, voice navigation
    ├── rviz/
    └── worlds/            # Cùng môi trường đã dùng để tạo map
```

Thư mục `vendor/` là bản sao dependency mô phỏng cần trên máy hiện tại, không phụ thuộc đường
dẫn assign4 và không upload Git. Máy mới cài TurtleBot 4 Simulator/Navigation bằng package hệ thống.

## Cài đặt và build

Ubuntu 22.04 + ROS 2 Humble; cần TurtleBot 4 Simulator/Navigation, Nav2, RViz, `python3-venv`,
ALSA `arecord`. Nếu thiếu công cụ:

```bash
sudo apt install python3-venv alsa-utils
```

```bash
cd ~/workspaces/assign7
./scripts/install_deps.bash
./scripts/segment_map.bash
./scripts/build.bash
```

Installer tạo `.venv` riêng và tải Whisper **base multilingual**, CPU int8, hai luồng xử lý.
Chỉ cần mạng khi cài/model download; lúc nhận giọng nói không gọi dịch vụ cloud. Mô hình base
ưu tiên nhẹ máy; nhận diện có thể sai do tiếng ồn/phát âm nên phải quan sát văn bản nghe được.

## Chạy hai terminal

Dừng mô phỏng và các bộ điều khiển của tuần trước trước khi chạy.

**Terminal 1 — Gazebo + map + AMCL + Nav2 + RViz:**

```bash
cd ~/workspaces/assign7
./scripts/run_navigation.bash world:=small_room
```

Đợi `diffdrive_controller` hoạt động. Trong Terminal 2 nhả dock:

```bash
cd ~/workspaces/assign7
./scripts/undock.bash
```

Trong RViz dùng **2D Pose Estimate** đặt đúng vị trí/hướng sau Undock. Đợi robot và LaserScan
khớp bản đồ; ba vùng màu và nhãn A/B/C hiện trên RViz. Hệ thống chỉ nhận lệnh chạy khi có
transform `map -> base_link`; việc chỉ click 2D Pose Estimate không được coi là đã định vị thành công.

**Terminal 2 — bắt đầu nhận giọng nói và điều hướng:**

```bash
./scripts/run_voice.bash
```

Đợi thông báo `MIC ĐANG NGHE`. Nói một câu, rồi ngừng khoảng một giây:

- “Đi đến khu vực A.”
- “Đi đến phòng thí nghiệm.”
- “Di chuyển từ khu vực A đến khu vực B.”
- “Đi từ phòng học qua hành lang C rồi đến khu vực B.”
- “Đi đến khu vực D.” → từ chối, không gửi goal.
- “Dừng lại.” → hủy điều hướng đang thực hiện.

Theo dõi `NGHE ĐƯỢC`, `Đã hiểu lộ trình`, `Gửi goal`, `Đã đến khu vực` và `HOÀN THÀNH`.
Trong khi đang chạy, lệnh đích mới bị từ chối để tránh ghi đè lộ trình; muốn đổi đích thì nói
dừng, chờ kết thúc/hủy rồi nói lệnh mới. Đây không phải điều khiển giữ-tay của tuần 6: im lặng
sau một câu hợp lệ không tự hủy goal. Ctrl+C node giọng nói không thay thế lệnh dừng khẩn cấp.

## Microphone và chẩn đoán ASR

Mặc định dùng thiết bị ALSA `pulse`, lấy nguồn thu mặc định của desktop. Kiểm tra trong
Settings → Sound → Input, tăng mic và cấp quyền microphone nếu cần. Liệt kê thiết bị:

```bash
arecord -L
pactl get-default-source
```

Nếu `pulse` không có, thử:

```bash
./scripts/run_voice.bash audio_device:=default
```

Không dùng nguồn `monitor` của loa thay cho mic. Đặt robot/phòng có tiếng ồn nhỏ; nói rõ tên
“phòng học”, “phòng thí nghiệm”, “hành lang” thường dễ phân biệt hơn chỉ nói một chữ cái.

Có thể dùng file WAV thu sẵn để kiểm tra cùng pipeline nhận dạng (không thay thế video nói thật):

```bash
./scripts/run_voice.bash audio_file:=/duong/dan/lenh.wav
```

Chỉ kiểm tra parser/Nav2 bằng văn bản khi debug:

```bash
# Terminal 2: không mở microphone
./scripts/run_voice.bash speech:=false
# Terminal 3
source setup_local.bash
ros2 topic pub --once /voice/text std_msgs/msg/String "{data: 'Đi từ khu vực A đến khu vực B'}"
```

Demo nộp bài phải dùng microphone/giọng nói, không chỉ dùng lệnh text ở trên.

## File vùng và phương pháp đổi tọa độ

Xem ảnh: `src/assign7_bringup/semantic/segmented_map.png`.
File `regions.json` lưu đích của từng vùng trong frame `map`; không dùng thẳng tọa độ pixel.
Với ô ảnh `(u,v)`, độ phân giải `r`, chiều cao `H`, tọa độ cục bộ là
`((u+0.5)*r, (H-v-0.5)*r)`, sau đó quay theo yaw và cộng origin của `map.yaml`.
Mã xử lý có hỗ trợ origin quay; ô chưa quan sát không được coi là ô trống.

Muốn đổi tên/seed: sửa `config/region_seeds.yaml`, chạy `segment_map.bash`, rồi build lại.
Nếu đổi bản đồ nhưng chưa tạo lại regions, chương trình từ chối dùng file cũ dựa trên hash.
Khi chọn môi trường Gazebo khác, phải dùng đúng bản đồ của môi trường đó.

## Topic/action

| Tên | Kiểu | Vai trò |
|---|---|---|
| `/voice/text` | `std_msgs/msg/String` | Câu tiếng Việt từ ASR |
| `/voice/speech_status` | `std_msgs/msg/String` | Đang nghe, nhận dạng, lỗi mic/model |
| `/voice/navigation_status` | `std_msgs/msg/String` | Cách hiểu lệnh và tiến độ các đích |
| `/semantic_regions` | `visualization_msgs/msg/MarkerArray` | Vùng màu, nhãn và goal trong RViz |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Gửi/hủy goal và nhận kết quả |

## Kiểm tra và nộp bài

```bash
source setup_local.bash
python3 -m unittest discover -s tests -v
```

Quay video liên tục: bản đồ có 3 vùng → nói ít nhất 3 lệnh → terminal nhận văn bản/hiểu lệnh →
robot tự đến đúng nơi bằng Nav2. Có một lệnh A → B (hoặc qua nhiều đích) và một lệnh vùng D để
thể hiện xử lý lỗi. GitHub chứa source, launch/config, bản đồ gốc, ảnh phân vùng và regions.json.
Không upload `vendor`, `.venv`, model Whisper, build/install/log, ghi âm cá nhân. Video Drive công
khai và báo cáo PDF 1–2 trang nộp riêng sau khi có kết quả thực nghiệm.

Tham khảo: [faster-whisper](https://github.com/SYSTRAN/faster-whisper),
[Nav2 NavigateToPose](https://api.nav2.org/actions/humble/navigatetopose.html),
[TurtleBot 4](https://turtlebot.github.io/turtlebot4-user-manual/).
