# 项目简介

## MoveIt2 接入

已新增 `arm_moveit` 包用于 MoveIt2 规划与执行。

### 编译

```bash
source /opt/ros/humble/setup.bash
cd /home/dev/work/robotic_arm
colcon build --packages-select arm_description arm_moveit --symlink-install
source install/setup.bash
```

### 启动（真实硬件链路）

```bash
source /opt/ros/humble/setup.bash
cd /home/dev/work/robotic_arm
source install/setup.bash
ros2 launch arm_moveit moveit_real.launch.py
```

### 说明

- 启动文件会先拉起 `arm_description_real.launch.py`（控制器与状态发布）
- 然后启动 MoveIt 的 `move_group`
- 最后启动 MoveIt 对应的 RViz 界面
