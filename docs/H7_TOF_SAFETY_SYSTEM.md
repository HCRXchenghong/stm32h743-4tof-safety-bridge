# H7 TOF 安全冗余系统总文档

## 1. 目标

这套系统把 `STM32H743` 从“4 路 TOF 采集板”升级成“安全冗余裁判板”。

它负责：

- 采集 4 路 TOF 距离
- 根据当前运动方向决定哪些 TOF 生效
- 判断是否越过安全窗口 `391mm ± 28mm`
- 主动输出急停
- 把实时状态通过 USB CDC 发给上位机
- 接收上位机发来的运动信息、释放急停、接管标志

## 2. TOF 安装方位

- `TOF1 = 右前`
- `TOF2 = 右后`
- `TOF3 = 左后`
- `TOF4 = 左前`

位图统一按 `bit0..bit3 = TOF1..TOF4` 编码。

## 3. 安全策略

### 3.1 安全窗口

- 基准值：`391 mm`
- 容差：`±28 mm`
- 安全范围：`363 mm ~ 419 mm`

当前生效的 TOF 只要超出这个窗口，就会被判定为越界。

### 3.2 哪些 TOF 生效

- 前进：只启用前方 `TOF1 + TOF4`
- 后退：只启用后方 `TOF2 + TOF3`
- 转向：4 路全启用
- 横移 / 混合运动：4 路全启用
- 上位机运动信息超时：回退到 4 路全启用
- 静止：4 路全启用

### 3.3 急停判定

以下任一条件满足，H7 会主动触发 `self_estop`：

- 当前生效 TOF 无有效帧
- 当前生效 TOF 距离越界

总急停 `estop` 的判定为：

```text
estop = self_estop OR external_estop
```

其中：

- `self_estop`：H7 自己根据安全策略触发
- `external_estop`：外部急停输入 `PE11` 被拉低

### 3.4 急停恢复

当前实现是“距离恢复正常或收到释放命令，满足任一条件即可恢复”，但为了避免抖动，板端增加了一个很短的稳定窗口：

- 越界消失后，连续稳定约 `80ms` 自动释放
- 上位机发送 `release_req=1` 时，如果当前已经没有实时越界，会立即释放
- 如果实时越界仍然存在，收到释放命令后会在下一轮判断里再次触发急停

## 4. 固件架构

固件入口在：

- [`firmware/h743_tof_usb_bridge_cubeide/Core/Src/main.c`](../firmware/h743_tof_usb_bridge_cubeide/Core/Src/main.c)

主要分 4 层：

### 4.1 采集层

- 4 路 UART 分别接 4 个 TOF
- 每路解析 `0xFF Data_H Data_L SUM`
- 每路维护：
  - 最近距离
  - 是否有效
  - 帧计数
  - 同步错误
  - 校验错误
  - UART 错误

### 4.2 控制输入层

- 通过 USB CDC 接收上位机的 `H7CTL` 文本帧
- 保存最近一次的：
  - `vx_mmps`
  - `vy_mmps`
  - `wz_mradps`
  - `takeover_enable`
  - `release_req`
- 如果控制输入超过 `200ms` 没更新，就认为上位机运动信息超时

### 4.3 安全判定层

- 根据 `cmd_vel` 对应速度换算后的 `vx / vy / wz` 判断运动模式
- 计算 `active_mask`
- 对当前生效 TOF 逐路判断：
  - 是否有效
  - 是否越界
  - 是否触发 `trip_mask`

### 4.4 输出层

- `PE10` 为低电平时向底盘输出急停
- 同时通过 USB CDC 发出 `H7TOF` 状态帧

## 5. 串口协议

### 5.1 H7 -> 上位机

推荐状态帧：

```text
$H7TOF,seq,tof1,tof2,tof3,tof4,estop,self_estop,ext_estop,active_mask,trip_mask,valid_mask,fault_mask,motion_mode,takeover*CS
```

字段含义：

- `seq`：递增序号
- `tof1..tof4`：4 路距离，单位 `mm`
- `estop`：总急停
- `self_estop`：STM32 主动急停
- `ext_estop`：外部急停输入
- `active_mask`：当前参与保护的 TOF 位图
- `trip_mask`：当前导致板端急停的 TOF 位图
- `valid_mask`：当前有有效距离的 TOF 位图
- `fault_mask`：当前串口帧异常 / 无效的 TOF 位图
- `motion_mode`：
  - `0` 静止
  - `1` 前进
  - `2` 后退
  - `3` 转向
  - `4` 横移 / 混合
  - `5` 失联保护
- `takeover`：上位机/底盘接管标志

H7 目前还会在后面追加调试字段，例如：

- `R=`：每路收到的字节数
- `F=`：每路有效帧数
- `S=`：同步错误数
- `C=`：校验错误数
- `E=`：UART 错误数
- `L=`：每路最近一个字节
- `H=`：收到的控制帧数 / 解析错误数 / 校验错误数

上位机解析时只需要消费前 15 个字段，后面的调试字段可选。

### 5.2 上位机 -> H7

控制帧：

```text
$H7CTL,seq,vx_mmps,vy_mmps,wz_mradps,release_req,takeover_enable*CS
```

字段含义：

- `seq`：上位机控制序号
- `vx_mmps`：前后速度，单位 `mm/s`
- `vy_mmps`：横向速度，单位 `mm/s`
- `wz_mradps`：角速度，单位 `mrad/s`
- `release_req`：`1` 表示请求释放板端急停
- `takeover_enable`：`1` 表示当前由上位机/底盘接管

### 5.3 校验

- 帧头前缀：`$`
- 结尾校验：`*CS`
- `CS` 为 payload 的逐字节 XOR，16 进制大写

## 6. HTML 联调页面

页面路径：

- [`tools/web_serial_dashboard/index.html`](../tools/web_serial_dashboard/index.html)

页面新增了：

- 急停红灯
- 4 路 TOF 的方位标注
- `active_mask` 显示
- `trip_mask` 显示
- `self_estop / external_estop` 分离显示
- 每个传感器卡片显示“生效中 / 已屏蔽 / 触发急停”

兼容性：

- 兼容旧版 9 字段 `H7TOF`
- 兼容新版 15 字段 `H7TOF`
- 兼容单行 JSON

## 7. 独立 ROS1 SDK

SDK 目录：

- [`sdk/h7_tof_safety_bridge_ros1`](../sdk/h7_tof_safety_bridge_ros1)

它不依赖当前仓库的 `src`，目的是后面你可以直接把 `src` 删掉，再把这个 SDK 拷到新的 catkin 工作区里。

### 7.1 包含内容

- `scripts/h7_tof_bridge_node.py`
- `src/h7_tof_safety_bridge/protocol.py`
- `msg/TofSafetyStatus.msg`
- `launch/h7_tof_safety_bridge.launch`

### 7.2 默认 ROS 接口

- 订阅 `/cmd_vel`
- 订阅 `/h7_tof/takeover_enable`
- 发布 `/h7_tof/status`
- 发布 `/h7_tof/estop`
- 发布 `/h7_tof/raw_line`
- 服务 `/h7_tof/release_estop`

### 7.3 节点职责

- 把 ROS `Twist` 转成 `mm/s` 与 `mrad/s`
- 以固定周期发送 `H7CTL`
- 一次性发送 `release_req`
- 解析 H7 回传的 `H7TOF`
- 发布标准状态消息给上层

## 8. 集成步骤

### 8.1 板端

1. 用 CubeIDE 打开 `firmware/h743_tof_usb_bridge_cubeide`
2. 编译并烧录 H7
3. 确认 4 路 TOF 接线和方位对应关系正确

### 8.2 ROS1 SDK

1. 把 `sdk/h7_tof_safety_bridge_ros1` 复制到新的 catkin 工作区 `src/`
2. 安装依赖：`python3-serial`
3. `catkin_make`
4. `source devel/setup.bash`
5. `roslaunch h7_tof_safety_bridge h7_tof_safety_bridge.launch port:=/dev/ttyACM0`

### 8.3 HTML

1. 在仓库根目录启动静态服务器
2. 打开 `http://localhost:8000/tools/web_serial_dashboard/index.html`
3. 连接 H7 串口

## 9. 联调注意事项

- HTML 和 ROS SDK 不能同时占用同一个 H7 USB CDC 串口
- 如果上位机控制帧超时，H7 会回退为 4 路 TOF 全部生效
- `takeover_enable` 目前是协议状态位，没有新增 GPIO 接管线
- 当前安全窗口是固定值，如果后面要做现场标定，应改成可配置参数

## 10. 验收清单

### 10.1 方向相关

- 前进时，仅 `TOF1 / TOF4` 能触发急停
- 后退时，仅 `TOF2 / TOF3` 能触发急停
- 转向时，4 路都能触发急停

### 10.2 急停来源

- 板端越界时 `self_estop=1`
- 外部急停拉低时 `ext_estop=1`
- 两者任一成立时 `estop=1`

### 10.3 恢复

- 越界恢复正常后，板端自动释放
- ROS SDK 发送 `release_estop` 后，若当前已无越界，应立即释放
- 若当前仍在越界，释放命令不会让系统持续处于放行状态

### 10.4 页面显示

- 急停时红灯亮
- 传感器屏蔽时页面显示“已屏蔽”
- 触发时页面显示“触发急停”
