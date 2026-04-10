# stm32h743-4tof-safety-bridge

STM32H743 4 路 TOF 安全桥。

这份 README 只保留一条最实用的真实部署路线，给第一次接触这个项目的人直接照着做。

部署完成后，你会得到：

- 板子通过 USB 接到 Ubuntu
- ROS1 SDK 读取板子串口数据
- `rqt` 面板显示 4 路 TOF、急停状态、阈值
- 可以在 `rqt` 里下发参数和人工解除急停

## 1. 你需要准备什么

硬件：

- 已焊好并能上电的 STM32H743 板
- 4 路 TOF 传感器
- USB 数据线

电脑环境：

- Ubuntu 20.04
- ROS1 Noetic

如果你不是这个环境，先尽量按这个环境来，不要一上来自己改系统版本。

## 2. 先把固件烧进板子

仓库里已经有编好的固件，直接烧这个文件：

- `release/stm32h743-4tof-safety-bridge.hex`

如果你已经确认板子里烧的就是这个版本，可以跳过这一步。

烧录完成后，把板子通过 USB 接到 Ubuntu。

## 3. 在 Ubuntu 安装 ROS1 和依赖

打开终端，直接执行：

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-rqt \
  ros-noetic-rqt-gui \
  ros-noetic-rqt-gui-py \
  ros-noetic-dynamic-reconfigure \
  python3-serial
```

再把当前用户加入串口权限组：

```bash
sudo usermod -a -G dialout $USER
```

这一步做完后，退出系统重新登录一次。

## 4. 创建 ROS 工作空间

打开新终端，执行：

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 5. 把本项目的两个 ROS 包放进工作空间

假设你的仓库在：

```bash
/path/to/stm32h743-4tof-safety-bridge
```

把下面两条命令里的路径改成你自己的实际路径，然后执行：

```bash
ln -s /path/to/stm32h743-4tof-safety-bridge/sdk/h7_tof_safety_bridge_ros1 ~/catkin_ws/src/h7_tof_safety_bridge
ln -s /path/to/stm32h743-4tof-safety-bridge/sdk/h7_tof_safety_bridge_rqt ~/catkin_ws/src/h7_tof_safety_bridge_rqt
```

## 6. 编译

执行：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source ~/catkin_ws/devel/setup.bash
```

如果这里报错，不要继续往下。先解决编译错误。

## 7. 确认板子串口号

插上板子后执行：

```bash
ls /dev/ttyACM*
```

常见结果是：

```bash
/dev/ttyACM0
```

如果你看到的是别的，比如 `/dev/ttyACM1`，后面启动命令就把它替换掉。

## 8. 一条命令启动整套系统

执行：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch h7_tof_safety_bridge_rqt h7_tof_safety_bridge_with_rqt.launch port:=/dev/ttyACM0
```

如果你的串口不是 `/dev/ttyACM0`，自己替换成实际串口。

## 9. 启动成功后你应该看到什么

`rqt` 面板打开后，正常情况下会看到：

- 左上角显示 `SDK 状态：在线`
- 4 路 TOF 数值在刷新
- 能看到 `Baseline / Tol / Threshold`
- 能看到急停状态和剩余解除时间

如果看不到这些，说明还没部署成功。

## 10. 怎么操作

### 改阈值参数

在 `rqt` 面板里：

1. 修改 `Baseline`
2. 修改 `Tolerance`
3. 点击“下发参数”
4. 等顶部状态变成“板端已确认”

### 人工解除急停

在 `rqt` 面板里：

1. 设置“人工解除(ms)”
2. 点击“恢复急停”
3. 看剩余时间是否开始倒计时

如果时间到了，但故障还在，板子会再次急停。这是正常安全行为。

## 11. 最简单的验收方法

按下面顺序检查：

1. 板子接上 USB 后能识别出 `/dev/ttyACM*`
2. `roslaunch` 后 `rqt` 能打开
3. 面板左上角显示 `SDK 状态：在线`
4. 4 路 TOF 数据在刷新
5. 修改参数后能显示“板端已确认”
6. 人工解除后倒计时会变化

这 6 条都满足，就说明已经部署成功。

## 12. 最常见的 3 个问题

### 问题 1：`ls /dev/ttyACM*` 没有设备

先检查：

- USB 线是不是数据线，不是只充电线
- 板子有没有正常上电
- 固件有没有正确烧进去

### 问题 2：`roslaunch` 启动了，但 `rqt` 里显示离线

先检查：

- 启动命令里的 `port:=...` 对不对
- 当前用户有没有串口权限
- 有没有重新登录过系统

### 问题 3：点“下发参数”后一直不确认

先检查：

- 固件是不是这个仓库里的新版本
- 板子是否真的在线
- 串口数据是否稳定

## 13. 相关目录

- 固件：`release/stm32h743-4tof-safety-bridge.hex`
- ROS SDK：`sdk/h7_tof_safety_bridge_ros1`
- `rqt` 面板：`sdk/h7_tof_safety_bridge_rqt`
