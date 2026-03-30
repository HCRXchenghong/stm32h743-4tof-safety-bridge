# stm32h743-4tof-safety-bridge

<p>
  <strong>语言 / Language</strong><br>
  点击下方标题可切换语言内容<br>
  中文默认展开，English 默认收起
</p>

---

<details open>
<summary><strong>中文</strong></summary>

`stm32h743-4tof-safety-bridge` 是一个基于 `STM32H743ZITx` 的四路 TOF 安全冗余桥接项目。  
该项目的目标不是单纯采集距离数据，而是在板端形成一套独立的安全判断链路：采集四路 UART TOF、根据运动方向决定保护区域、在 STM32 侧主动输出急停，并通过 `USB CDC` 与上位机进行双向状态交换。

### 项目定位

该项目提供三部分能力：

- 板端固件：负责四路 TOF 采集、方向相关安全策略、急停状态机与 USB CDC 协议
- 浏览器联调页面：负责直接读取 STM32 串口并可视化显示 TOF 状态与急停信息
- 独立 ROS1 SDK：负责把上位机运动信息编码为控制帧下发给 H7，并把 H7 状态接入 ROS

项目当前不依赖仓库中的 ROS 工作区源码，`sdk/` 目录中的 ROS1 包可以直接复制到其他 catkin 工作区中单独使用。

### 主要特性

- 四路 `DYP-R01` 串口 TOF 采集
- `STM32H743` 板端主动急停，不依赖上位机决策闭环
- 固定安全窗口：`391 mm ± 28 mm`
- 方向相关保护策略：
  - 前进仅启用前侧 TOF
  - 后退仅启用后侧 TOF
  - 转向与混合运动启用全部 TOF
  - 上位机运动信息超时自动回退到全通道保护
- `USB CDC` 双向文本协议
- 浏览器 `Web Serial` 联调界面
- 独立 ROS1 SDK，包括消息、节点、协议封装和启动文件

### TOF 安装定义

四路 TOF 的物理方位在本项目中固定如下：

- `TOF1 = 右前`
- `TOF2 = 右后`
- `TOF3 = 左后`
- `TOF4 = 左前`

所有位图统一按照 `bit0..bit3 = TOF1..TOF4` 编码。

### 安全策略

#### 安全窗口

- 基准值：`391 mm`
- 容差：`±28 mm`
- 安全范围：`363 mm ~ 419 mm`

当前生效的 TOF 通道只要超出上述范围，即判定为越界。

#### 生效规则

- 前进：启用 `TOF1 + TOF4`
- 后退：启用 `TOF2 + TOF3`
- 转向：启用全部 TOF
- 横移 / 混合运动：启用全部 TOF
- 静止：启用全部 TOF
- 控制输入超时：启用全部 TOF，进入失联保护

#### 急停规则

STM32 在以下条件成立时触发 `self_estop`：

- 当前生效 TOF 无有效帧
- 当前生效 TOF 距离越界

总急停逻辑为：

```text
estop = self_estop OR external_estop
```

其中：

- `self_estop`：板端安全逻辑主动触发
- `external_estop`：外部急停输入触发

#### 释放规则

当前固件实现为“自动恢复 + 命令释放”并存：

- 越界消失并稳定一小段时间后，板端自动释放
- 上位机发送释放命令时，如果当前已经没有实时越界，也会立即释放
- 如果实时越界依旧存在，释放后会在下一轮判断中再次触发急停

### 系统架构

```text
4 路 TOF UART
  -> STM32H743 安全板
     -> 板端安全判断
     -> PE10 输出急停
     -> USB CDC 状态帧上报
     <- USB CDC 控制帧输入

ROS1 SDK
  -> 订阅 cmd_vel
  -> 周期发送 H7CTL
  <- 接收 H7TOF
  -> 发布状态话题与急停接口

Web Serial Dashboard
  <- 直接读取 H7TOF 串口帧
  -> 显示距离、掩码、急停灯与触发状态
```

### 仓库结构

```text
stm32h743-4tof-safety-bridge/
├─ README.md
├─ 激光DYP-RD-产品规格书(DYP-R01-V1.0)-250412-A0.pdf
├─ firmware/
│  └─ h743_tof_usb_bridge_cubeide/
├─ release/
│  └─ stm32h743-4tof-safety-bridge.hex
├─ sdk/
│  └─ h7_tof_safety_bridge_ros1/
└─ tools/
   └─ web_serial_dashboard/
      └─ index.html
```

关键文件：

- 固件入口：`firmware/h743_tof_usb_bridge_cubeide/Core/Src/main.c`
- USB CDC 接口：`firmware/h743_tof_usb_bridge_cubeide/USB_DEVICE/App/usbd_cdc_if.c`
- 联调页面：`tools/web_serial_dashboard/index.html`
- ROS1 节点：`sdk/h7_tof_safety_bridge_ros1/scripts/h7_tof_bridge_node.py`
- ROS1 协议封装：`sdk/h7_tof_safety_bridge_ros1/src/h7_tof_safety_bridge/protocol.py`

### 硬件接口

#### TOF 串口映射

| 通道 | 外设 | MCU TX | MCU RX | 方位 |
|---|---|---|---|---|
| TOF1 | USART1 | PA9 | PA10 | 右前 |
| TOF2 | USART6 | PC6 | PC7 | 右后 |
| TOF3 | USART3 | PD8 | PD9 | 左后 |
| TOF4 | USART2 | PD5 | PD6 | 左前 |

`DYP-R01` 默认输出参数：

- 波特率：`115200`
- 格式：`8N1`
- 帧格式：`0xFF Data_H Data_L SUM`

典型接线：

- 红 -> `5V`
- 黑 -> `GND`
- 绿 -> MCU `RX`
- 黄 -> MCU `TX`

#### 急停 IO

- `PE10 = ESTOP_OUT_N`
- `PE11 = ESTOP_IN_N`

电平定义：

- `PE10` 低电平：触发急停
- `PE10` 高电平：释放急停
- `PE11` 低电平：外部请求急停

GPIO 配置：

- `PE10`：开漏输出，上拉，默认高
- `PE11`：输入，上拉

### 固件工程

固件工程路径：

- `firmware/h743_tof_usb_bridge_cubeide`

推荐开发环境：

- `STM32CubeIDE`
- 目标芯片：`STM32H743ZITx`

固件已经实现：

- 四路 UART TOF 解帧
- `USB CDC` 双向通信
- 根据主机运动信息计算 `active_mask`
- 根据距离和帧有效性计算 `trip_mask`
- 板端 `self_estop / external_estop / estop` 状态机
- 串口与协议调试计数

#### 编译与烧录

1. 在 `STM32CubeIDE` 中打开 `firmware/h743_tof_usb_bridge_cubeide`
2. 直接编译工程
3. 使用编译产物或预编译固件烧录

预编译固件：

- `release/stm32h743-4tof-safety-bridge.hex`

推荐烧录方式：

- `STM32CubeProgrammer`
- `USB DFU`
- `ST-LINK`

### USB CDC 协议

#### H7 -> Host 状态帧

推荐状态帧格式：

```text
$H7TOF,seq,tof1,tof2,tof3,tof4,estop,self_estop,ext_estop,active_mask,trip_mask,valid_mask,fault_mask,motion_mode,takeover*CS
```

字段说明：

- `seq`：状态帧序号
- `tof1..tof4`：四路距离，单位 `mm`
- `estop`：总急停状态
- `self_estop`：板端主动急停状态
- `ext_estop`：外部急停输入状态
- `active_mask`：当前参与保护的 TOF 位图
- `trip_mask`：当前触发板端急停的 TOF 位图
- `valid_mask`：当前有有效距离的 TOF 位图
- `fault_mask`：当前存在帧无效或串口异常的 TOF 位图
- `motion_mode`：
  - `0` 静止
  - `1` 前进
  - `2` 后退
  - `3` 转向
  - `4` 横移 / 混合
  - `5` 失联保护
- `takeover`：主机接管标志回传

主帧之后允许追加调试字段：

- `R=`：接收字节数
- `F=`：有效帧数
- `S=`：同步错误数
- `C=`：校验错误数
- `E=`：UART 错误数
- `L=`：最近一个字节
- `H=`：主机控制帧接收 / 解析 / 校验统计

#### Host -> H7 控制帧

```text
$H7CTL,seq,vx_mmps,vy_mmps,wz_mradps,release_req,takeover_enable*CS
```

字段说明：

- `seq`：控制序号
- `vx_mmps`：前后速度，单位 `mm/s`
- `vy_mmps`：横向速度，单位 `mm/s`
- `wz_mradps`：角速度，单位 `mrad/s`
- `release_req`：`1` 表示请求释放板端急停
- `takeover_enable`：`1` 表示主机接管有效

#### 校验规则

- 前缀：`$`
- 后缀：`*CS`
- `CS` 为 payload 全部字节异或值

### Web Serial 联调页面

页面路径：

- `tools/web_serial_dashboard/index.html`

页面能力：

- 实时显示四路 TOF 距离
- 独立急停红灯
- 显示 `self_estop / external_estop`
- 显示 `active_mask / trip_mask / valid_mask / fault_mask`
- 显示每个 TOF 当前是“生效 / 屏蔽 / 触发急停”
- 兼容旧版和新版 `H7TOF` 帧

推荐启动方式：

```bash
python3 -m http.server 8000
```

访问地址：

```text
http://localhost:8000/tools/web_serial_dashboard/index.html
```

推荐浏览器：

- `Chrome`
- `Edge`

### ROS1 SDK

SDK 路径：

- `sdk/h7_tof_safety_bridge_ros1`

该目录是一个独立 ROS1 包，设计目标是复制到新的 catkin 工作区后单独构建使用。

SDK 内容：

- `package.xml`
- `CMakeLists.txt`
- `setup.py`
- `msg/TofSafetyStatus.msg`
- `scripts/h7_tof_bridge_node.py`
- `src/h7_tof_safety_bridge/protocol.py`
- `launch/h7_tof_safety_bridge.launch`

默认 ROS 接口：

- 订阅：`/cmd_vel`
- 订阅：`/h7_tof/takeover_enable`
- 发布：`/h7_tof/status`
- 发布：`/h7_tof/estop`
- 发布：`/h7_tof/raw_line`
- 服务：`/h7_tof/release_estop`

#### 集成方式

1. 将 `sdk/h7_tof_safety_bridge_ros1` 复制到 catkin 工作区 `src/`
2. 安装依赖 `python3-serial`
3. 执行 `catkin_make`
4. `source devel/setup.bash`
5. 启动节点：

```bash
roslaunch h7_tof_safety_bridge h7_tof_safety_bridge.launch port:=/dev/ttyACM0
```

SDK 节点职责：

- 将 `geometry_msgs/Twist` 转换为 `vx_mmps / vy_mmps / wz_mradps`
- 周期发送 `H7CTL`
- 发送一次性释放命令
- 解析 `H7TOF`
- 向上层系统发布标准状态消息

### 验证清单

- 前进时仅 `TOF1 / TOF4` 可触发急停
- 后退时仅 `TOF2 / TOF3` 可触发急停
- 转向时任意 TOF 可触发急停
- 生效 TOF 无有效帧时必须触发急停
- `PE11` 拉低时必须产生 `external_estop`
- 控制帧超时后必须回退到四路全保护
- 联调页面能够正确显示屏蔽通道和触发通道
- ROS1 SDK 能正确编码 `H7CTL` 并解析扩展 `H7TOF`

### 说明

- 浏览器联调页面与 ROS1 SDK 不能同时占用同一个 `USB CDC` 端口
- 当前安全窗口为固件内固定值，尚未引入现场标定或参数化配置
- `takeover_enable` 当前仅为协议状态位，本版本未定义额外接管 GPIO

---

</details>

<details>
<summary><strong>English</strong></summary>

`stm32h743-4tof-safety-bridge` is a four-channel TOF safety bridge built around `STM32H743ZITx`.  
The project is designed as an independent safety controller rather than a passive distance collector: it acquires four UART TOF sensors, selects active protection zones according to motion direction, asserts emergency stop on the STM32 side, and exchanges state with the host through `USB CDC`.

### Scope

The repository provides three deliverables:

- firmware for the STM32 safety board
- a browser-based serial dashboard for direct bring-up
- a standalone ROS1 SDK for integration in an external catkin workspace

### Key Features

- four-channel `DYP-R01` UART TOF acquisition
- board-side emergency stop independent of host-side decision loops
- fixed safety window: `391 mm ± 28 mm`
- direction-aware protection masks
- bidirectional `USB CDC` text protocol
- `Web Serial` dashboard
- standalone ROS1 SDK under `sdk/`

### TOF Orientation

- `TOF1 = right front`
- `TOF2 = right rear`
- `TOF3 = left rear`
- `TOF4 = left front`

Bit masks always follow `bit0..bit3 = TOF1..TOF4`.

### Safety Logic

- forward: `TOF1 + TOF4`
- reverse: `TOF2 + TOF3`
- turning: all four channels
- planar / mixed motion: all four channels
- host timeout: all four channels in fail-safe mode

Trip conditions on active TOFs:

- invalid or timed-out frame
- distance outside `363 mm ~ 419 mm`

Final stop logic:

```text
estop = self_estop OR external_estop
```

### Repository Layout

```text
stm32h743-4tof-safety-bridge/
├─ README.md
├─ 激光DYP-RD-产品规格书(DYP-R01-V1.0)-250412-A0.pdf
├─ firmware/
├─ release/
├─ sdk/
└─ tools/
```

### Firmware

Firmware entry:

- `firmware/h743_tof_usb_bridge_cubeide/Core/Src/main.c`

The firmware currently implements:

- four-channel UART TOF decoding
- bidirectional USB CDC transport
- active-mask computation from host motion
- trip-mask computation from distance and channel validity
- `self_estop / external_estop / estop` state machine

Prebuilt image:

- `release/stm32h743-4tof-safety-bridge.hex`

### Status Frame

```text
$H7TOF,seq,tof1,tof2,tof3,tof4,estop,self_estop,ext_estop,active_mask,trip_mask,valid_mask,fault_mask,motion_mode,takeover*CS
```

### Control Frame

```text
$H7CTL,seq,vx_mmps,vy_mmps,wz_mradps,release_req,takeover_enable*CS
```

### Dashboard

Dashboard entry:

- `tools/web_serial_dashboard/index.html`

The dashboard displays:

- four TOF distances
- estop warning lamp
- `self_estop / external_estop`
- `active_mask / trip_mask / valid_mask / fault_mask`
- active, masked, and tripped sensor states

### ROS1 SDK

SDK path:

- `sdk/h7_tof_safety_bridge_ros1`

Default ROS interfaces:

- subscribe: `/cmd_vel`
- subscribe: `/h7_tof/takeover_enable`
- publish: `/h7_tof/status`
- publish: `/h7_tof/estop`
- publish: `/h7_tof/raw_line`
- service: `/h7_tof/release_estop`

Launch example:

```bash
roslaunch h7_tof_safety_bridge h7_tof_safety_bridge.launch port:=/dev/ttyACM0
```

### Notes

- The browser dashboard and the ROS1 SDK must not open the same USB CDC port at the same time.
- The current safety window is fixed in firmware.
- `takeover_enable` is currently a protocol state bit only.

</details>
