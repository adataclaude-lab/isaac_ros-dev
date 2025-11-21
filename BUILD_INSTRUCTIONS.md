# 如何建置 dualarm_moveit_config 套件

## 問題說明

當您嘗試執行以下命令時：
```bash
ros2 launch dualarm_moveit_config isaac_sim_bridge.launch.py
ros2 run dualarm_moveit_config simple_ee_control.py separate
```

會遇到錯誤：`Package 'dualarm_moveit_config' not found`

這是因為套件尚未被建置。

## 解決方案

### 步驟 1: 進入 Isaac ROS 容器

```bash
cd ~/workspaces/isaac_ros-dev
src/isaac_ros_common/scripts/run_dev.sh
```

### 步驟 2: 在容器內建置套件

進入容器後，執行：

```bash
cd /workspaces/isaac_ros-dev
./build_in_container.sh
```

或者手動建置：

```bash
cd /workspaces/isaac_ros-dev
colcon build --packages-select dualarm_description dualarm_moveit_config --symlink-install
```

### 步驟 3: 載入工作空間

建置完成後：

```bash
source install/setup.bash
```

### 步驟 4: 驗證安裝

```bash
# 檢查套件是否可找到
ros2 pkg list | grep dualarm_moveit_config

# 應該會看到
# dualarm_moveit_config
```

### 步驟 5: 執行您的命令

現在您可以執行：

```bash
# 啟動 Isaac Sim 橋接
ros2 launch dualarm_moveit_config isaac_sim_bridge.launch.py

# 或執行控制腳本
ros2 run dualarm_moveit_config simple_ee_control.py separate
```

## 快速參考

### 完整流程（一次執行所有命令）

```bash
# 在容器外
cd ~/workspaces/isaac_ros-dev
src/isaac_ros_common/scripts/run_dev.sh

# 進入容器後
cd /workspaces/isaac_ros-dev
colcon build --packages-select dualarm_description dualarm_moveit_config --symlink-install
source install/setup.bash

# 驗證
ros2 pkg list | grep dualarm_moveit_config

# 執行
ros2 launch dualarm_moveit_config isaac_sim_bridge.launch.py
```

## 注意事項

1. **必須在容器內建置**：Isaac ROS 套件必須在 Isaac ROS 容器內建置，因為它們依賴特定的 ROS 2 環境和 NVIDIA 函式庫。

2. **每次開新容器都要 source**：每次進入新的容器會話時，都需要執行：
   ```bash
   source install/setup.bash
   ```

3. **符號連結安裝**：使用 `--symlink-install` 選項可以讓您修改 Python 腳本和 launch 檔案而不需要重新建置。

4. **如果修改了 C++ 代碼或 CMakeLists.txt**：需要重新建置：
   ```bash
   colcon build --packages-select dualarm_moveit_config
   ```

## 疑難排解

### 問題：找不到 colcon 命令

確認您在 Isaac ROS 容器內：
```bash
echo $ROS_DISTRO  # 應該顯示 humble 或其他 ROS 版本
```

如果沒有輸出，請確保您已經進入容器。

### 問題：建置失敗，找不到依賴

確認所有依賴套件都已安裝：
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 問題：修改了代碼但沒有生效

1. 對於 Python 和 launch 檔案（使用 --symlink-install）：不需要重新建置，只需要重新啟動節點
2. 對於 C++ 代碼：需要重新建置並 source

## 相關檔案

- `/workspaces/isaac_ros-dev/src/dualarm_moveit_config/` - 套件原始碼
- `/workspaces/isaac_ros-dev/install/dualarm_moveit_config/` - 安裝位置
- `/workspaces/isaac_ros-dev/build/dualarm_moveit_config/` - 建置輸出
