# 雙手臂 MoveIt 配置與控制

這個套件提供了雙手臂機器人的 MoveIt 配置，支援透過 cuMotion 在 RViz 和 Omniverse 中進行軌跡規劃和控制。

## 快速開始

### 建置套件

在使用此套件之前，您需要先建置它：

```bash
# 進入工作空間
cd /workspaces/isaac_ros-dev  # 或 /home/user/isaac_ros-dev

# 使用建置腳本（推薦）
./build_workspace.sh

# 或手動建置
source /opt/ros/humble/setup.bash
colcon build --packages-select dualarm_description dualarm_moveit_config --symlink-install

# 載入工作空間
source install/setup.bash
```

### 驗證安裝

建置完成後，驗證套件是否正確安裝：

```bash
# 檢查套件是否可找到
ros2 pkg list | grep dualarm_moveit_config

# 列出可用的 launch 檔案
ros2 pkg prefix dualarm_moveit_config
```

## 套件結構

```
dualarm_moveit_config/
├── config/              # MoveIt 配置檔案
│   ├── arm_dualarm.srdf                    # 語義機器人描述
│   ├── joint_limits.yaml                   # 關節限制
│   ├── kinematics.yaml                     # 運動學求解器配置
│   ├── moveit_controllers.yaml             # MoveIt 控制器配置
│   ├── ros2_controllers.yaml               # ROS2 控制器配置
│   ├── isaac_ros_cumotion_planning.yaml    # cuMotion 規劃器配置
│   └── cumotion_parameters.yaml            # cuMotion 參數
├── launch/             # Launch 檔案
│   ├── demo.launch.py                      # 基本示範
│   ├── move_group.launch.py                # MoveIt move_group 節點
│   ├── cumotion_planner.launch.py          # cuMotion 規劃器
│   └── ...
└── scripts/            # 控制腳本
    └── simple_ee_control.py                # 雙手臂末端執行器控制腳本
```

## 機器人配置

### 雙手臂結構

機器人包含兩個獨立的 6 自由度手臂：

- **左手臂 (left_arm)**
  - 關節: j1_Joint_L ~ j6_Joint_L
  - 末端執行器: j6_Link_L
  - 基座: base_link_L

- **右手臂 (right_arm)**
  - 關節: j1_Joint_R ~ j6_Joint_R
  - 末端執行器: j6_Link_R
  - 基座: base_link_R

### 控制器配置

系統配置了以下控制器：

1. `left_arm_controller` - 左手臂軌跡控制器
2. `right_arm_controller` - 右手臂軌跡控制器
3. `joint_state_broadcaster` - 關節狀態廣播器

## 使用方式

### 1. 啟動 MoveIt + RViz

```bash
ros2 launch dualarm_moveit_config demo.launch.py
```

這會啟動：
- MoveIt move_group 節點
- RViz 視覺化介面
- 模擬控制器

### 2. 啟動 cuMotion 規劃器

如果要使用 NVIDIA cuMotion 進行加速路徑規劃：

```bash
ros2 launch dualarm_moveit_config cumotion_planner.launch.py
```

### 3. 使用雙手臂控制腳本

提供了 Python 控制腳本，支援三種模式：

#### 分別控制模式
分別控制左右手臂：

```bash
ros2 run dualarm_moveit_config simple_ee_control.py separate
```

#### 同時控制模式
同時規劃並執行左右手臂的協調運動：

```bash
ros2 run dualarm_moveit_config simple_ee_control.py simultaneous
```

#### 關節控制模式
通過關節角度控制手臂：

```bash
ros2 run dualarm_moveit_config simple_ee_control.py joint
```

#### 運行所有示範
```bash
ros2 run dualarm_moveit_config simple_ee_control.py
```

## 在程式中使用控制 API

### 基本使用範例

```python
#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Pose
from dualarm_moveit_config.scripts.simple_ee_control import DualArmController

def main():
    rclpy.init()

    # 創建控制器
    controller = DualArmController()

    # 移動到初始位姿
    controller.move_to_initial_pose()

    # 創建目標位姿
    target_pose = Pose()
    target_pose.position.x = 0.3
    target_pose.position.y = 0.3
    target_pose.position.z = 1.2
    target_pose.orientation.w = 1.0

    # 移動左手臂
    controller.move_left_arm_to_pose(target_pose)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### API 參考

#### 移動到目標位姿

```python
# 移動左手臂到目標位姿
controller.move_left_arm_to_pose(pose: Pose) -> bool

# 移動右手臂到目標位姿
controller.move_right_arm_to_pose(pose: Pose) -> bool

# 同時移動兩個手臂
controller.move_both_arms_to_pose(
    left_pose: Pose,
    right_pose: Pose
) -> bool
```

#### 移動到關節角度

```python
# 移動左手臂到指定關節角度
controller.move_left_arm_to_joints(
    joint_values: list[float]  # 6個關節角度
) -> bool

# 移動右手臂到指定關節角度
controller.move_right_arm_to_joints(
    joint_values: list[float]  # 6個關節角度
) -> bool
```

#### 移動到預定義位姿

```python
# 移動到初始位姿（定義在 SRDF 中）
controller.move_to_initial_pose() -> bool
```

## 與 Omniverse 整合

### 設置環境變量

確保設置了正確的工作空間路徑：

```bash
export ISAAC_ROS_WS=/home/user/isaac_ros-dev
```

### 啟動 Isaac Sim 橋接

```bash
ros2 launch dualarm_moveit_config isaac_sim_bridge.launch.py
```

這會啟動：
- Isaac Sim 和 ROS2 之間的橋接
- 關節狀態同步
- TF 廣播

## 配置調整

### 修改關節限制

編輯 `config/joint_limits.yaml`：

```yaml
joint_limits:
  j1_Joint_L:
    has_velocity_limits: true
    max_velocity: 0.87266460000000001
    has_acceleration_limits: false
    max_acceleration: 0
```

### 修改 cuMotion 參數

編輯 `config/cumotion_parameters.yaml`：

```yaml
cumotion_planner_node:
  ros__parameters:
    # 工作空間範圍
    grid_center_m: [0.0, 0.0, 0.6]
    grid_size_m: [2.0, 2.0, 1.5]

    # 雙臂末端執行器
    tool_frames: ["j6_Link_L", "j6_Link_R"]

    # 碰撞參數
    collision_padding_m: 0.02
```

### 修改速度縮放

編輯 `config/joint_limits.yaml`：

```yaml
# 調整速度和加速度縮放因子（0.0 到 1.0）
default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1
```

## 故障排除

### 問題：規劃失敗

1. 檢查目標位姿是否在工作空間內
2. 檢查是否有碰撞
3. 增加規劃時間限制
4. 調整關節限制

### 問題：控制器無法連接

1. 確認控制器已啟動：
   ```bash
   ros2 control list_controllers
   ```

2. 檢查控制器狀態：
   ```bash
   ros2 control list_hardware_interfaces
   ```

### 問題：cuMotion 規劃器無法啟動

1. 檢查 URDF 和 XRDF 路徑是否正確
2. 確認 cuMotion 相關套件已安裝
3. 檢查日誌輸出

## 修改記錄

### 2025-11-21
- 修正 `isaac_ros_cumotion_planning.yaml` 中的 URDF 路徑
- 修正 `cumotion_planner.launch.py` 中的 URDF 檔名
- 新增 `xrdf/dualarm.xrdf` 配置檔案
- 新增 `scripts/simple_ee_control.py` 雙手臂控制腳本
- 更新 CMakeLists.txt 以包含新檔案
- 配置已支援雙手臂獨立和協調控制

## 相關資源

- [MoveIt 2 文檔](https://moveit.picknik.ai/main/index.html)
- [Isaac ROS cuMotion](https://nvidia-isaac-ros.github.io/concepts/manipulation/cumotion/index.html)
- [ROS 2 Control](https://control.ros.org/)

## 授權

請參閱套件根目錄的 LICENSE 檔案。
