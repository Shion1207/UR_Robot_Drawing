# Bài thực hành 03 — LLM Skill Planning với Gripper và Camera

Hệ thống ROS 2 Humble điều khiển tay máy **UR3/UR3e + Robotiq 2F-85** trong Ignition Gazebo
bằng câu lệnh ngôn ngữ tự nhiên. LLM kết nối qua **9Router** chỉ lập kế hoạch mức robot skill;
MoveIt 2 sinh quỹ đạo và kiểm tra va chạm. Camera RGB overhead cung cấp vị trí runtime của năm
cube và trạng thái chiếm dụng của ba vùng đích.

```text
User command → 9Router/LLM → JSON plan → Plan Validator → Skill Server → MoveIt 2 → Robot
                     ↑                                              ↓
                     └──── scene_state ← Camera perception ← Gazebo camera
```

## Nội dung chính

- Một UR3/UR3e, gripper Robotiq 2F-85, bàn thao tác và camera RGB cố định trên cao.
- Ba vùng thật: `zone_a`, `zone_b`, `zone_c`; một vị trí tạm logic `zone_tmp`.
- Năm cube: đỏ, vàng, xanh dương, xanh lá và tím.
- Nhận dạng HSV, chiếu pixel xuống mặt bàn và cập nhật Planning Scene từ camera.
- Bảy robot skill: `home`, `pick`, `place`, `move_above`, `move_to_zone`, `open_gripper`,
  `close_gripper`.
- Plan Validator ngăn LLM sinh tọa độ, joint hoặc trajectory trực tiếp.
- Tự phát hiện vùng đích bị chiếm và lập kế hoạch dọn vật qua `zone_tmp`.

Mã nguồn chính nằm tại [`src/ur_llm_planner`](src/ur_llm_planner). Hướng dẫn kỹ thuật đầy đủ
nằm trong [`src/ur_llm_planner/README.md`](src/ur_llm_planner/README.md).

## Cài đặt và build

```bash
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash

sudo apt install ros-humble-robotiq-description python3-opencv python3-numpy
rosdep install --ignore-src --from-paths src -y

colcon build --packages-select ur_llm_planner
source install/setup.bash
```

## Chạy chương trình

### Terminal 1 — Gazebo, MoveIt, gripper và camera perception

```bash
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_llm_planner sim.launch.py
```

### Terminal 2 — 9Router

```bash
9router
```

Trong dashboard 9Router, kết nối provider/model và tạo API key. Endpoint mặc định của package là
`http://localhost:20128/v1`.

### Terminal 3 — LLM Planner

```bash
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash
source install/setup.bash
export NINEROUTER_API_KEY="<api_key_tu_9router>"

ros2 run ur_llm_planner llm_planner_node.py \
  --ros-args \
  --params-file src/ur_llm_planner/config/llm.yaml
```

Nhập yêu cầu tại dấu nhắc `>>>`, ví dụ:

```text
Put the red cube in Zone B.
```

Scene mặc định đặt `blue_cube` trong Zone B. Vì vậy robot phải dời cube xanh sang `zone_tmp`,
sau đó mới gắp cube đỏ vào Zone B và trở về home.

### Terminal 4 — Xem ảnh camera (tùy chọn)

```bash
source /opt/ros/humble/setup.bash
source ~/workspaces/ur_gz/install/setup.bash
ros2 run rqt_image_view rqt_image_view /overhead_camera/image
```

Camera nằm tại `(x, y, z) = (0.15, 0.0, 1.20)` m và nhìn thẳng xuống mặt bàn.

## Kiểm thử

```bash
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select ur_llm_planner --event-handlers console_direct+
colcon test-result --test-result-base build/ur_llm_planner/test_results --verbose
```

## Lưu ý bảo mật

Không ghi API key vào `config/llm.yaml` hoặc commit lên Git. Khai báo key bằng biến môi trường
`NINEROUTER_API_KEY` tại terminal chạy LLM Planner.
