# ur_llm_planner — Điều khiển UR3/UR3e bằng LLM và Skill-based Planning

Package ROS 2 Humble cho phép điều khiển UR3/UR3e (mô phỏng Gazebo + MoveIt 2) bằng câu lệnh
ngôn ngữ tự nhiên. LLM (kết nối qua **9Router**) **chỉ** hiểu yêu cầu và chọn/sắp xếp các robot
skill. LLM **không** sinh joint trajectory hay giá trị khớp.

```
Natural Language Command ──► LLM Planner ──► JSON Plan ──► Plan Validator ──► Skill Executor ──► MoveIt 2 ──► UR3/UR3e
   (bàn phím / topic)        (9Router)                      (whitelist)       (/execute_skill)
Camera RGB ──► nhận dạng 5 màu ──► `/scene_state` ──► LLM prompt + Validator + vị trí gắp
```

## 1. Kiến trúc

| Thành phần | File | Vai trò |
|---|---|---|
| Môi trường Gazebo | `worlds/pick_place.sdf` | Bàn, camera RGB, 5 cube động, 3 zone; `blue_cube` khởi đầu trong `zone_b` để demo vật cản |
| Camera perception | `scripts/camera_perception_node.py`, `llm_planner/perception.py` | Phân đoạn HSV, chiếu pixel xuống mặt bàn và xác định object/zone |
| Robot + gripper | `urdf/ur_robotiq.urdf.xacro` | UR3/UR3e + Robotiq 2F-85, TCP `grasp_tcp`, DetachableJoint cho từng khối |
| MoveIt SRDF | `srdf/ur_robotiq.srdf.xacro` | SRDF của UR + tắt va chạm giữa các khâu gripper |
| Controller gripper | `config/gripper_controller.yaml` | `gripper_controller` (JointTrajectoryController, 6 khớp ngón) |
| Khai báo scene | `config/scene.yaml` | Pose spawn/reset, vị trí zone, màu object và thông số camera |
| Thông tin sinh viên | `config/student_config.yaml` | `student.name`, `student.id` → nhiệm vụ cá nhân |
| Nhiệm vụ cá nhân | `llm_planner/student.py` | `P = XX mod 6` → vật nào ở zone A/B/C |
| Skill Server (C++) | `src/skill_server.cpp` | Thực thi skill bằng MoveIt 2, trả về trạng thái |
| Interface | `srv/ExecuteSkill.srv` | `skill, object, zone` → `status, message` |
| LLM client | `llm_planner/llm_client.py` | Gọi 9Router (API tương thích OpenAI) |
| Prompt | `llm_planner/prompt.py` | Mô tả skill/vật/vùng + quy tắc, yêu cầu trả về JSON |
| Plan Validator | `llm_planner/plan_validator.py` | Kiểm tra kế hoạch, từ chối skill/object/zone không hợp lệ |
| LLM Planner node | `scripts/llm_planner_node.py` | Nhận câu lệnh → LLM → Validator → gọi từng skill |
| Launch | `launch/sim.launch.py` | Gazebo + camera bridge + perception + ros2_control + MoveIt 2 + skill_server |

### 1.1. Robot Skills

| Skill | Tham số | Mô tả |
|---|---|---|
| `home()` | – | Về tư thế home an toàn phía trên bàn (joint target, OMPL) |
| `pick(object)` | object | `move_above(object)` + `close_gripper()` |
| `place(object, zone)` | object, zone | `move_to_zone(zone)` + `open_gripper()` |
| `move_above(object)` | object | Đưa `grasp_tcp` (hướng xuống) tới phía trên tâm vật 12 cm |
| `move_to_zone(zone)` | zone | Đưa `grasp_tcp` tới phía trên vùng (tính cả vật đang cầm) |
| `close_gripper()` | – | Mở ngón, hạ thẳng xuống tâm vật, kẹp, khoá vật, nâng lên |
| `open_gripper()` | – | Hạ vật xuống mặt bên dưới, mở ngón, nhả vật, nâng lên |

Mỗi skill trả về một trong các trạng thái:

`SUCCESS`, `FAILED`, `INVALID_SKILL`, `INVALID_OBJECT`, `INVALID_ZONE`, `PLANNING_FAILED`,
`EXECUTION_FAILED`, `OBJECT_NOT_HELD`, `GRIPPER_BUSY`, `NO_OBJECT_TO_GRASP`, `PERCEPTION_UNAVAILABLE`.

### 1.2. An toàn chuyển động

- **Joint limit**: mọi quỹ đạo do MoveIt 2 lập kế hoạch theo `joint_limits.yaml` của
  `ur_moveit_config`. Khi giải IK, skill server còn giới hạn cấu hình (`|shoulder_pan| ≤ π`,
  elbow-up, cổ tay hướng xuống) để tránh các nghiệm "xoắn". Nghiệm IK còn phải không tự va chạm
  và không chạm bàn. Nếu OMPL vẫn không lập được kế hoạch tới nghiệm đó, skill server giải lại IK
  từ seed khác (tối đa 22 lần).
- **Self-collision**: kiểm tra bằng SRDF của `ur_moveit_config`.
- **Va chạm môi trường**: bàn và 5 khối được thêm vào planning scene dưới dạng collision object.
  Gripper là một phần của robot trong MoveIt. Khối đang cầm được *attach* vào `grasp_tcp`, nên
  MoveIt tính cả gripper và khối đó khi kiểm tra va chạm.
- Các đoạn hạ/nâng thẳng đứng dùng `computeCartesianPath(..., avoid_collisions = true)`. Nếu
  fraction < 98% thì trả về `PLANNING_FAILED` và **không** thực thi.

### 1.3. Gripper Robotiq 2F-85

Robotiq 2F-85 gắn tại `tool0`. Mô hình lấy từ gói `robotiq_description`.

- **TCP**: link `grasp_tcp` cách đế gripper 0.145 m, nằm ở tâm giữa hai ngón. Khi kẹp, tâm khối
  trùng `grasp_tcp`, đầu ngón cách mặt bàn khoảng 4 mm.
- **Ngón kẹp**: `ign_ros2_control` của Humble không hỗ trợ khớp mimic, nên `gripper_controller`
  điều khiển cả 6 khớp ngón. Skill server gửi góc `q` nhân với hệ số mimic của URDF
  (`gripper_mimic_multipliers`). `q = 0` là mở (khe hở 85 mm), `q = 0.43` là kẹp khối 4 cm
  (khe hở 42 mm, mỗi bên cách khối 1 mm, không ép vào vật).
- **Giữ vật**: các khối là vật thể động, có khối lượng và va chạm. Sau khi ngón đóng, skill server
  khoá khối vào `wrist_3_link` bằng plugin **DetachableJoint** của Gazebo (topic
  `/gripper/<vật>/attach`) để khối không trượt. Khi thả, nó nhả qua `/gripper/<vật>/detach`.
  Plugin này tự khoá mọi khối lúc khởi động, nên skill server nhả tất cả rồi đặt lại vị trí ban
  đầu qua service `/world/pick_place_world/set_pose`.
- Trong MoveIt: `attachObject` / `detachObject` khối vào `grasp_tcp`, với `touch_links` là các khâu
  của gripper.

### 1.4. Camera và trạng thái môi trường

Camera RGB cố định nhìn vuông góc xuống bàn; hệ thống không lấy pose runtime của cube từ YAML.
`scene.yaml` chỉ chứa pose để spawn/reset mô phỏng và tâm các zone đã biết.

- Gazebo phát `/overhead_camera/image` và `/overhead_camera/camera_info`.
- Perception phân đoạn HSV theo năm màu, lấy tâm blob và dùng mô hình pinhole để chiếu xuống bàn.
- `/camera/object_poses` cấp pose đo được cho Skill Server; `/camera/scene_state` dùng để quan sát.
- Skill Server cập nhật collision object, suy ra `in_zone` và phát `/scene_state` với `source=camera`.
- Planner và Skill Server đều trả `PERCEPTION_UNAVAILABLE` nếu camera chưa thấy đủ năm cube.

### 1.5. Plan Validator

Validator dùng cơ chế **whitelist**. Chỉ cần một bước sai là **toàn bộ** kế hoạch bị từ chối,
robot không chạy dù chỉ một phần:

- JSON phải có dạng `{"plan": [ ... ]}` và có tối đa 20 bước.
- `skill` phải thuộc danh sách 7 skill ở trên.
- Mỗi bước chỉ được có đúng các tham số của skill đó. Mọi trường khác (`joints`, `trajectory`,
  `position`...) đều bị từ chối, nên LLM không thể điều khiển khớp trực tiếp.
- `object`/`zone` phải nằm trong `object_names`/`zone_names` của `scene.yaml`.
- Validator mô phỏng trạng thái gripper để bắt các thứ tự vô lý: `place` khi chưa `pick`, `pick`
  khi đang cầm vật khác, `close_gripper` mà không có `move_above` ngay trước đó...
- Validator mô phỏng cả **trạng thái các vùng** (lấy từ `/scene_state`): mỗi vùng chỉ chứa một
  vật. Kế hoạch đặt vật vào vùng đang có vật khác sẽ bị từ chối. Lỗi được gửi lại cho LLM để nó
  chèn bước dọn vùng (chuyển vật đang chiếm sang vùng đích của nó hoặc sang `zone_tmp`).

Khi kế hoạch bị từ chối, planner gửi danh sách lỗi lại cho LLM để nó tự sửa (tối đa
`llm.max_repair_attempts` lần, mặc định 2). Kế hoạch sau khi sửa vẫn phải qua validator.
Skill server cũng kiểm tra lại skill/object/zone một lần nữa (phòng thủ hai lớp).

### 1.6. Cá nhân hoá theo MSSV

Khai báo trong `config/student_config.yaml` (nhớ thay bằng tên và MSSV **thật** của bạn):

```yaml
/**:
  ros__parameters:
    student:
      name: "Nguyen Van An"
      id: "23020123"
```

Hoặc ghi đè khi chạy: `--ros-args -p student.name:="Nguyen Van An" -p student.id:=23020123`.

`P = XX mod 6` (XX là hai chữ số cuối MSSV) quyết định vật nào phải nằm ở zone A/B/C:

| P | Zone A | Zone B | Zone C |
|---|---|---|---|
| 0 | red | yellow | blue |
| 1 | red | blue | yellow |
| 2 | yellow | red | blue |
| 3 | yellow | blue | red |
| 4 | blue | red | yellow |
| 5 | blue | yellow | red |

Ví dụ: MSSV 23020123 → 23 mod 6 = 5 → A = blue, B = yellow, C = red. Khi khởi động, planner in:

```
STUDENT: Nguyen Van An | ID: 23020123 | P = 23 mod 6 = 5
PERSONAL TASK: zone_a <- blue_cube | zone_b <- yellow_cube | zone_c <- red_cube
```

Bảng gán này được đưa vào system prompt. Nhờ vậy LLM tự lập kế hoạch cho các lệnh như
`Arrange all objects according to my student ID.`

### 1.7. Vùng đích bị chiếm

- Prompt cho LLM biết vật nào đang ở vùng nào và vùng nào đang trống. Chỉ có tên vùng, không
  có toạ độ.
- Quy tắc: nếu vùng đích đang có vật X, chuyển X tới vùng đích của X nếu vùng đó trống; nếu
  không (trường hợp vòng tròn, ví dụ đổi chỗ 2 vật) thì chuyển X tới vùng tạm `zone_tmp`.
- Validator kiểm tra lại quy tắc này. Nếu LLM sai, lỗi được gửi lại để LLM sửa
  (`llm.max_repair_attempts` = 2).

Scene mặc định đặt `blue_cube` trong `zone_b`. Với lệnh `Put the red cube in Zone B`, kế hoạch phải là:

```
pick(blue_cube) -> place(blue_cube, zone_tmp)    # dọn vật đang chiếm chỗ
pick(red_cube) -> place(red_cube, zone_b)
home()
```

### 1.8. Đầu ra trên terminal

```
USER COMMAND:
Put the red cube in zone B.

LLM PLAN:
    pick(blue_cube)
    place(blue_cube, zone_tmp)
    pick(red_cube)
    place(red_cube, zone_b)
    home()
VALIDATOR: plan hop le (5 buoc)

EXECUTION:
pick(blue_cube) ........... SUCCESS
place(blue_cube, zone_tmp)  SUCCESS
pick(red_cube) ............ SUCCESS
place(red_cube, zone_b) ... SUCCESS
home() .................... SUCCESS

TASK SUCCESS
```

Nếu một bước thất bại, các bước còn lại được đánh dấu `SKIPPED` và kết thúc bằng `TASK FAILED`.
Kế hoạch bị validator từ chối kết thúc bằng `TASK REJECTED`. Thêm `-p verbose:=true` để in câu
trả lời thô của LLM.

## 2. Cài đặt

Yêu cầu: Ubuntu 22.04 (hoặc WSL2 + Ubuntu 22.04), ROS 2 Humble, Gazebo Fortress, MoveIt 2,
`Universal_Robots_ROS2_Driver` và `ur_simulation_gz` (đã có trong workspace này).

```bash
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-robotiq-description python3-opencv python3-numpy
rosdep install --ignore-src --from-paths src -y
colcon build --packages-select ur_llm_planner
source install/setup.bash
```

### 2.1. Cài và cấu hình 9Router

9Router là router LLM chạy cục bộ, cung cấp endpoint tương thích OpenAI.

1. Cài Node.js, rồi cài và chạy 9Router theo hướng dẫn của dự án 9Router (ví dụ
   `npm install -g 9router` rồi `9router`).
2. Mở dashboard 9Router: kết nối một provider/model, tạo API key, rồi ghi lại **base URL** (mặc
   định trong package là `http://localhost:20128/v1`) và **tên model/combo**.
3. Đặt API key bằng biến môi trường, không ghi key vào file:

   ```bash
   export NINEROUTER_API_KEY=<api_key_tu_dashboard>
   ```

4. Điền `model` (và `base_url` nếu khác) trong `config/llm.yaml`, hoặc truyền bằng
   `-p llm.model:=...` khi chạy.

> Planner gửi request chuẩn OpenAI `POST {base_url}/chat/completions`, nên có thể dùng bất kỳ
> endpoint tương thích OpenAI nào bằng cách đổi `llm.base_url`.

## 3. Chạy

**Terminal 1** — mô phỏng + MoveIt 2 + skill server + camera perception:

```bash
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur_llm_planner sim.launch.py              # UR3e (mặc định)
# ros2 launch ur_llm_planner sim.launch.py ur_type:=ur3
```

Đợi đến khi log hiện dòng `Skill server san sang`.

**Terminal 2** — chạy 9Router (giữ terminal này mở):

```bash
9router
```

**Terminal 3** — LLM planner (nhập yêu cầu tại dấu nhắc `>>>`):

```bash
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash
source install/setup.bash
export NINEROUTER_API_KEY=<api_key>
ros2 run ur_llm_planner llm_planner_node.py --ros-args \
  --params-file src/ur_llm_planner/config/llm.yaml
```

Rồi gõ lệnh:

```
>>> Đưa khối màu đỏ vào vùng B.
>>> Hãy lấy khối màu vàng và đặt nó vào ô A.
>>> Move the blue cube to zone C.
```

Ví dụ kết quả:

```
[Command] Đưa khối màu đỏ vào vùng B.
[LLM] {"plan": [{"skill": "pick", "object": "blue_cube"}, {"skill": "place", "object": "blue_cube", "zone": "zone_tmp"}, {"skill": "pick", "object": "red_cube"}, {"skill": "place", "object": "red_cube", "zone": "zone_b"}, {"skill": "home"}]}
[Validator] Ke hoach hop le: pick(blue_cube) -> place(blue_cube, zone_tmp) -> pick(red_cube) -> place(red_cube, zone_b) -> home()
[Executor] (1/3) pick(red_cube) ...
           -> SUCCESS: pick(red_cube) thanh cong
...
[Result] SUCCESS
```

**Terminal 4 (tùy chọn)** — xem ảnh RGB mà perception đang sử dụng:

```bash
source /opt/ros/humble/setup.bash
source ~/workspaces/ur_gz/install/setup.bash
ros2 run rqt_image_view rqt_image_view /overhead_camera/image
```

### 3.1. Các cách gửi lệnh khác

```bash
# Gửi câu lệnh qua topic (planner đang chạy)
ros2 topic pub --once /nl_command std_msgs/msg/String "{data: 'Move the blue cube to zone C.'}"

# Chỉ sinh + kiểm tra kế hoạch, không di chuyển robot (dry run)
ros2 run ur_llm_planner llm_planner_node.py --ros-args -p llm.model:=<ten_model> -p execute:=false

# Kiểm thử Validator/Executor không qua LLM: gửi thẳng kế hoạch JSON
ros2 topic pub --once /plan_json std_msgs/msg/String \
  "{data: '{\"plan\": [{\"skill\": \"pick\", \"object\": \"orange_cube\"}]}'}"
#  -> [Validator] Ke hoach bi tu choi: object 'orange_cube' khong ton tai

# Gọi trực tiếp một skill
ros2 service call /execute_skill ur_llm_planner/srv/ExecuteSkill "{skill: pick, object: red_cube}"
ros2 service call /execute_skill ur_llm_planner/srv/ExecuteSkill "{skill: place, object: red_cube, zone: zone_b}"
ros2 service call /execute_skill ur_llm_planner/srv/ExecuteSkill "{skill: home}"
```

### 3.2. Topic / Service

| Tên | Kiểu | Mô tả |
|---|---|---|
| `/execute_skill` | `ur_llm_planner/srv/ExecuteSkill` | Thực thi một skill (skill_server) |
| `/overhead_camera/image` | `sensor_msgs/Image` | Ảnh RGB thật từ sensor Gazebo |
| `/overhead_camera/camera_info` | `sensor_msgs/CameraInfo` | Intrinsic dùng để chiếu pixel về world |
| `/camera/object_poses` | `geometry_msgs/PoseArray` | Pose năm cube theo đúng thứ tự `object_names` |
| `/camera/scene_state` | `std_msgs/String` (JSON, latched) | Kết quả trực tiếp của perception |
| `/scene_state` | `std_msgs/String` (JSON, latched) | State camera hợp nhất với vật gripper đang giữ |
| `/nl_command` | `std_msgs/String` | Câu lệnh ngôn ngữ tự nhiên |
| `/plan_json` | `std_msgs/String` | Kế hoạch JSON gửi thẳng (bỏ qua LLM, để kiểm thử) |
| `/llm_planner/plan` | `std_msgs/String` | Kế hoạch đã qua validator |
| `/llm_planner/result` | `std_msgs/String` (JSON) | Kết quả task, gồm cả `PERCEPTION_UNAVAILABLE` |
| `/skill_server/markers` | `visualization_msgs/MarkerArray` | Vùng A/B/C trên RViz (Add → By topic) |

Trạng thái `/scene_state` được đưa vào prompt, nên LLM biết vật nào đang ở đâu. Nhờ vậy nó xử lý
được các lệnh như "chuyển mọi khối ở vùng B sang vùng C".

## 4. Thay đổi môi trường

Pose object trong `config/scene.yaml` chỉ dùng để spawn/reset (frame `world`, mặt bàn `z = 0`);
pose runtime luôn đến từ camera. Nếu thay đổi pose ban đầu, sửa tương ứng model trong
`worlds/pick_place.sdf`. Để thêm vật hoặc vùng mới: thêm tên vào `object_names`/`zone_names`, khai
báo `position`/`description`/`color`, thêm model vào world và thêm HSV range nếu dùng màu mới.
Với vật mới, cần thêm tên vào mặc định của
`grasp_objects` trong `urdf/ur_robotiq.urdf.xacro` để có DetachableJoint cho vật đó. Prompt và
validator tự lấy danh sách từ parameter.

Các tham số chính của `skill_server`: `ee_link` (`grasp_tcp`), `approach_height` (0.12 m),
`velocity_scaling` (0.3), `acceleration_scaling` (0.3), `planning_time` (5 s), `planning_retries`
(3), `min_cartesian_fraction` (0.98), `home_joints` (`[0, -1.5708, 1.5708, -1.5708, -1.5708, 0]`).

Tham số gripper: `gripper_open_position` (0.0), `gripper_grasp_position` (0.43 rad, dành cho khối
4 cm), `gripper_motion_time` (1.0 s), `gripper_action`
(`/gripper_controller/follow_joint_trajectory`). Nếu đổi `cube_size` thì cần tính lại
`gripper_grasp_position` cho khe hở giữa hai ngón lớn hơn cạnh khối khoảng 2 mm.

## 5. Kiểm thử

```bash
cd ~/workspaces/ur_gz
colcon test --packages-select ur_llm_planner --event-handlers console_direct+
# hoặc chạy nhanh: python3 -m pytest src/ur_llm_planner/test -q
```

`test/test_plan_validator.py` có 27 test cho validator, gồm: skill/object/zone lạ, LLM gửi
`joints`/`trajectory`/toạ độ, thiếu tham số, `place` trước `pick`, `pick` hai lần, kế hoạch rỗng,
trích JSON từ markdown, đặt vật vào vùng bị chiếm, và đổi chỗ qua `zone_tmp`.
`test/test_student.py` kiểm tra bảng P → zone A/B/C, xử lý MSSV không hợp lệ, và nội dung prompt.
`test/test_perception.py` kiểm tra đủ năm màu, phép chiếu camera và phân loại zone.

## 6. Xử lý lỗi thường gặp

- `LLM_ERROR: Khong ket noi duoc ...`: 9Router chưa chạy, hoặc sai `llm.base_url`.
- `LLM_ERROR: HTTP 401`: sai hoặc thiếu API key (`NINEROUTER_API_KEY`).
- `LLM_ERROR: Chua cau hinh llm.model`: chưa truyền `-p llm.model:=...`.
- `Service /execute_skill chua san sang`: terminal 1 chưa chạy xong. Đợi dòng `Skill server san sang`.
- `PERCEPTION_UNAVAILABLE`: kiểm tra `/overhead_camera/image`, `/overhead_camera/camera_info` và
  `ros2 topic echo --once --full-length /scene_state`; cần `camera_ready: true`.
- `package 'robotiq_description' not found`: chưa cài `ros-humble-robotiq-description`.
- `Action /gripper_controller/follow_joint_trajectory chua san sang`: `gripper_controller` chưa
  chạy. Kiểm tra bằng `ros2 control list_controllers`.
- Ngón kẹp đóng nhưng khối không đi theo gripper: kiểm tra bridge DetachableJoint
  `ros2 topic info /gripper/red_cube/attach` (phải có 1 subscriber).
- Các khối trong Gazebo lệch với MoveIt sau khi chạy nhiều lần: khởi động lại `sim.launch.py`
  (skill server đặt lại vị trí ban đầu lúc khởi động).
