## 1. Giới thiệu đề tài

Dự án thực hiện việc điều khiển cánh tay robot công nghiệp **Universal Robots UR3e** trong môi trường mô phỏng vật lý **Ignition Gazebo (Fortress)** và khung điều khiển chuyển động **MoveIt 2**. Cánh tay robot tự động thoát điểm kỳ dị ban đầu (*Singularity Escape*), định vị khâu tác động cuối (*end-effector tool0*), tiếp cận mặt phẳng vẽ an toàn và di chuyển vẽ chính xác theo quỹ đạo hình học trong không gian Cartesian (quỹ đạo chữ D khép kín / hình tròn). Toàn bộ vệt di chuyển được hiển thị trực quan theo thời gian thực dưới dạng nét mực (Marker Line Strip) trong RViz2.

* **Video Demo hoạt động:** [Google Drive](https://drive.google.com/drive/folders/1IeT-tV6a_JuA-gXAOzLQCFsEt0d9lj75)

---

##  2. Cấu trúc thư mục Workspace

```
ur_gz/
├── .gitignore                   # Loại trừ các file build, log và file tạm
├── README.md                    # Hướng dẫn chi tiết cài đặt và sử dụng
└── src/
    ├── ur_simulation_gz/        # Package mô phỏng robot UR trên Ignition Gazebo
    │   └── ur_simulation_gz/
    │       ├── config/          # Cấu hình controller, joint tolerances
    │       ├── launch/          # Launch file khởi chạy Gazebo và MoveIt
    │       └── urdf/            # Mô hình robot URDF/xacro có bổ sung gain vị trí
    └── ur_write_d/              # Package do sinh viên phát triển
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        │   └── write_d.launch.py   # Launch file mở Gazebo, MoveIt và RViz
        └── src/
            └── ur_write_d_node.cpp # Node C++ tính quỹ đạo và publish Marker
```

---

##  3. Yêu cầu hệ thống và Cài đặt

### Yêu cầu tiên quyết:
* **Hệ điều hành:** Ubuntu 22.04 LTS (Jammy Jellyfish)
* **ROS 2:** ROS 2 Humble Hawksbill (Desktop Install)
* **Simulator:** Ignition Gazebo Fortress (`ros_gz`)
* **Motion Planning:** MoveIt 2

### Cài đặt các gói phụ thuộc (Dependencies):
Mở terminal và cài đặt các thư viện cần thiết:

```bash
# 1. Cập nhật hệ thống
sudo apt update && sudo apt upgrade -y

# 2. Cài đặt các gói ROS 2 Control và MoveIt 2
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-joint-trajectory-controller \
  ros-humble-moveit-resources-ur-moveit-config \
  ros-humble-ur-description \
  ros-humble-ur-moveit-config

# 3. Cài đặt Ignition Gazebo Fortress và ROS GZ Bridge
sudo apt install -y \
  ignition-fortress \
  ros-humble-ros-ign \
  ros-humble-ros-ign-bridge \
  ros-humble-ign-ros2-control
```

---

##  4. Hướng dẫn Build

Di chuyển vào thư mục workspace và tiến hành build bằng `colcon`:

```bash
cd ~/workspaces/ur_gz

# Build toàn bộ các package trong workspace
colcon build --symlink-install

# Nạp biến môi trường
source install/setup.bash
```

> **Lưu ý cấu hình Tolerances cho Gazebo Ignition:**  
> Trong môi trường mô phỏng vật lý có trọng lực, cánh tay máy có thể xuất hiện độ trễ bám vị trí (*tracking lag*). Để bộ điều khiển `joint_trajectory_controller` không bị hủy lệnh (*Abort*), dung sai quỹ đạo đã được cấu hình nới lỏng lên `10.0 rad` trong file `ur_simulation_gz/config/ur_controllers.yaml`. Nếu cần áp dụng tự động, chạy:
> ```bash
> sed -i 's/trajectory: 0.2/trajectory: 10.0/g' ~/workspaces/ur_gz/src/ur_simulation_gz/ur_simulation_gz/config/ur_controllers.yaml
> ```

---

##  5. Hướng dẫn Chạy chương trình

Chương trình được thiết kế chạy tách biệt 2 Terminal để người dùng có đầy đủ thời gian quan sát và cấu hình hiển thị Marker trong RViz2 trước khi robot thực thi.

### Bước 1: Dọn dẹp tiến trình cũ (nếu có)
```bash
killall -9 rviz2 ruby; pkill -f ros; pkill -f ign; pkill -f gz
```

### Bước 2: Khởi động Mô phỏng & RViz (Terminal 1)
Mở **Terminal 1** và chạy lệnh:
```bash
cd ~/workspaces/ur_gz
source install/setup.bash
ros2 launch ur_write_d write_d.launch.py
```
**Thao tác trong cửa sổ RViz2:**
1. Nhìn xuống bảng điều khiển góc dưới bên trái, bấm nút **Add**.
2. Trong danh sách hiện ra, chọn **Marker**, sau đó bấm **OK**.
3. Tại cây danh mục **Displays** bên trái, mở rộng mục **Marker** vừa thêm.
4. Tìm dòng **Durability Policy**, đổi giá trị từ `Volatile` thành **`Transient Local`** *(Bắt buộc để lưu giữ trọn vẹn vệt vẽ)*.

---

### Bước 3: Ra lệnh cho Robot vẽ quỹ đạo (Terminal 2)
Mở một tab hoặc cửa sổ **Terminal 2** mới và chạy:
```bash
cd ~/workspaces/ur_gz
source install/setup.bash
ros2 run ur_write_d ur_write_d_node --ros-args -p use_sim_time:=true
```

Ngay sau khi lệnh chạy:
1. Robot tự động quay khớp thoát khỏi điểm kỳ dị duỗi thẳng, đưa tay về tư thế **Ready Pose** an toàn.
2. Robot di chuyển thẳng tắp tới điểm bắt đầu của nét vẽ trên mặt phẳng cách đế 0.30 m.
3. Đầu gắp đặt bút và vẽ trọn vẹn quỹ đạo hình học chữ D (thân thẳng đứng kết hợp cung tròn khép kín).
4. Vệt mực xanh lá cây sáng nổi bật trên RViz và được lưu giữ vĩnh viễn trên màn hình để quan sát.

---

##  6. Nguyên lý Kỹ thuật cốt lõi

### 1. Singularity Escape (Thoát điểm kỳ dị)
Khi khởi động trong Gazebo, cánh tay UR3e mặc định ở cấu hình tất cả các góc khớp bằng 0 (cánh tay duỗi thẳng tắp theo phương ngang). Ở tư thế này, ma trận Jacobian bị suy biến (mất bậc tự do), các bộ giải Động học nghịch (IK) sẽ bị lỗi chia cho 0 nếu cố gắng lập kế hoạch Cartesian ngay. Chương trình giải quyết triệt để bằng cách đưa robot về tư thế gập góc trước:
$$\mathbf{q}_{\text{ready}} = [0, -\pi/2, \pi/2, -\pi/2, -\pi/2, 0]$$

### 2. Tính toán quỹ đạo Cartesian vi phân
Hàm `makeCleanD` tạo ra chuỗi các mốc tọa độ Pose $(X, Y, Z, \mathbf{q}_{\text{orientation}})$ liên tục trong không gian:
* **Thân thẳng:** Tăng dần tọa độ $Z$ từ $Z_{\text{bottom}}$ đến $Z_{\text{bottom}} + H$.
* **Bụng tròn:** Quét góc $\alpha \in [\pi/2, -\pi/2]$, tính $Y(\alpha) = Y_{\text{left}} + W \cdot \cos(\alpha)$ và $Z(\alpha) = Z_{\text{mid}} + R \cdot \sin(\alpha)$.
* Hàm `computeCartesianPath` với độ phân giải vi phân $eef\_step = 0.01\text{ m}$ và tắt kiểm tra va chạm ảo trên không (`avoid_collisions = false`) đảm bảo tính toán 100% đường cong mượt mà.

### 3. Vết mực Marker thời gian thực
Một timer định kỳ $25\text{ ms}$ sử dụng `tf2_ros::Buffer` liên tục tra cứu phép biến đổi tọa độ thực tế giữa `planning_frame` (world) và `tool_link` (`tool0`). Mỗi tọa độ thực tế được thêm vào danh sách điểm của `visualization_msgs::msg::Marker` dạng `LINE_STRIP`, xuất bản với QoS `Transient Local` để đảm bảo không bị mất gói tin hiển thị.

---

##  Tùy chỉnh tham số (Parameters)

Có thể thay đổi kích thước và vị trí vẽ trực tiếp qua tham số dòng lệnh ROS 2 mà không cần sửa code C++:

```bash
ros2 run ur_write_d ur_write_d_node --ros-args \
  -p use_sim_time:=true \
  -p plane_x:=0.32 \
  -p letter_height:=0.14 \
  -p letter_width:=0.12 \
  -p letter_bottom_z:=0.18
```

| Tham số | Mặc định | Ý nghĩa |
| :--- | :---: | :--- |
| `plane_x` | `0.30` | Khoảng cách từ gốc robot tới mặt phẳng vẽ (mét) |
| `letter_center_y` | `0.05` | Tọa độ tâm theo phương ngang Y (mét) |
| `letter_bottom_z` | `0.20` | Độ cao đáy chữ so với mặt đất Z (mét) |
| `letter_height` | `0.15` | Chiều cao chữ D (mét) |
| `letter_width` | `0.15` | Độ rộng bụng chữ D (mét) |
| `lift_distance` | `0.05` | Khoảng cách nhấc bút khi tiếp cận/kết thúc (mét) |

---

##  License
Phần mềm được phát hành dưới giấy phép mã nguồn mở Apache-2.0 License.

