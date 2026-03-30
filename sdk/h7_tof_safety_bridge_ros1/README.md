# h7_tof_safety_bridge ROS1 SDK

这个目录是独立于当前仓库 `src/` 的 ROS1 SDK 骨架，后面可以直接复制到新的 catkin 工作区里使用。

## 包含内容

- `scripts/h7_tof_bridge_node.py`
  通过串口向 H7 周期发送 `H7CTL`，并解析 H7 回传的 `H7TOF`
- `src/h7_tof_safety_bridge/protocol.py`
  文本协议的 Python 解析和封装
- `msg/TofSafetyStatus.msg`
  上层系统消费的标准状态消息
- `launch/h7_tof_safety_bridge.launch`
  基础启动入口

## 使用方式

1. 把整个目录复制到你的 catkin 工作区 `src/` 下
2. 安装依赖：`python3-serial`
3. 在工作区根目录执行 `catkin_make`
4. `source devel/setup.bash`
5. 启动：

```bash
roslaunch h7_tof_safety_bridge h7_tof_safety_bridge.launch port:=/dev/ttyACM0
```

## 默认接口

- 订阅：`/cmd_vel`
- 订阅：`/h7_tof/takeover_enable`
- 发布：`/h7_tof/status`
- 发布：`/h7_tof/estop`
- 发布：`/h7_tof/raw_line`
- 服务：`/h7_tof/release_estop`

完整架构、协议字段和验收说明见：

- [`docs/H7_TOF_SAFETY_SYSTEM.md`](../../docs/H7_TOF_SAFETY_SYSTEM.md)
