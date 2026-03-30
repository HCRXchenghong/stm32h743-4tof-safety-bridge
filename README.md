# stm32h743-4tof-safety-bridge

基于 `STM32H743ZITx` 的 4 路 UART TOF 安全冗余板工程。

## 项目目标

- 采集 4 路 `DYP-R01` 激光 TOF 串口数据
- 板端根据运动方向决定哪些 TOF 生效
- 在板端主动判断越界并输出急停
- 通过 `USB CDC` 与上位机双向通信
- 提供本地 HTML 联调页面
- 提供独立于当前工作区的 ROS1 SDK 与完整文档

## 当前状态

当前版本已经实现：
- STM32 `USB CDC` 双向协议
- 4 路 UART TOF 接收与解析
- 板端安全策略：
  - `TOF1=右前`
  - `TOF2=右后`
  - `TOF3=左后`
  - `TOF4=左前`
  - 安全窗口 `391mm ± 28mm`
  - 前进只看前侧、后退只看后侧、转向 4 路全启用
- 板端 `self_estop / ext_estop / estop` 状态机
- HTML Web Serial 联调页面升级
- 独立 ROS1 SDK 骨架
- Markdown 总文档

## 目录结构

```text
H7-4路TOF/
├─ README.md
├─ docs/
│  └─ H7_TOF_SAFETY_SYSTEM.md
├─ 激光DYP-RD-产品规格书(DYP-R01-V1.0)-250412-A0.pdf
├─ firmware/
│  └─ h743_tof_usb_bridge_cubeide/
├─ sdk/
│  └─ h7_tof_safety_bridge_ros1/
├─ tools/
│  └─ web_serial_dashboard/
│     └─ index.html
└─ release/
   └─ stm32h743-4tof-safety-bridge.hex
```

## 硬件接口

### TOF 接线原则

`DYP-R01` 使用自动输出 UART，默认参数：
- `115200`
- `8N1`
- 二进制帧：`0xFF Data_H Data_L SUM`

传感器常见线色：
- 红：`VCC`
- 黑：`GND`
- 黄：`Sensor RX`
- 绿：`Sensor TX`

接线原则：
- 红 -> `5V`
- 黑 -> `GND`
- 绿 -> MCU `RX`
- 黄 -> MCU `TX`

### 当前固件 UART 映射

| 通道 | 外设 | MCU TX | MCU RX | 传感器关键接线 |
|---|---|---|---|---|
| TOF1 | USART1 | PA9 | PA10 | 绿 -> PA10，黄 -> PA9 |
| TOF2 | USART6 | PC6 | PC7 | 绿 -> PC7，黄 -> PC6 |
| TOF3 | USART3 | PD8 | PD9 | 绿 -> PD9，黄 -> PD8 |
| TOF4 | USART2 | PD5 | PD6 | 绿 -> PD6，黄 -> PD5 |

说明：
- 对自动输出 TOF 来说，最关键的是 `传感器 TX -> MCU RX`
- 如果只做最小验证，`MCU TX` 那根可以先不接

## 急停 IO

当前固件里：
- `PE10 = ESTOP_OUT_N`
- `PE11 = ESTOP_IN_N`

电平逻辑：
- `PE10` 输出低电平：触发底盘急停
- `PE10` 输出高电平：释放急停
- `PE11` 被外部拉低：请求急停

GPIO 配置：
- `PE10`：开漏输出，上拉，默认高
- `PE11`：输入，上拉

## 固件工程

CubeIDE 主工程路径：
- `firmware/h743_tof_usb_bridge_cubeide`

建议使用：
- `STM32CubeIDE`
- 芯片选择 `STM32H743ZITx`

工程内核心文件：
- `Core/Src/main.c`
- `Core/Src/stm32h7xx_hal_msp.c`
- `Core/Src/stm32h7xx_it.c`
- `USB_DEVICE/App/usbd_cdc_if.c`
- `h743_tof_usb_bridge_cubeide.ioc`

## 编译与烧录

### 直接烧录现成固件

预编译固件：
- `release/stm32h743-4tof-safety-bridge.hex`

推荐烧录方式：
- `STM32CubeProgrammer`
- `USB DFU` 或 `ST-LINK`

### CubeIDE 编译

1. 打开 `firmware/h743_tof_usb_bridge_cubeide`
2. 用 `STM32CubeIDE` 打开工程
3. 直接 `Build Project`
4. 编译输出位于工程 `Debug/` 目录

## HTML 联调页面

页面路径：
- `tools/web_serial_dashboard/index.html`

使用方式：
1. 在项目根目录启动本地静态服务器
2. 用 `Chrome` 或 `Edge` 打开页面
3. 通过浏览器 `Web Serial` 连接 STM32 的 `USB CDC` 串口

页面现在支持显示：
- 急停红灯
- `self_estop / ext_estop`
- `active_mask / trip_mask / valid_mask / fault_mask`
- 4 路 TOF 当前是“生效 / 屏蔽 / 触发急停”

最简单的本地启动方式：

```powershell
cd H7-4路TOF
py -m http.server 8000
```

然后打开：

```text
http://localhost:8000/tools/web_serial_dashboard/index.html
```

## STM32 输出协议

当前固件推荐输出：

```text
$H7TOF,seq,tof1,tof2,tof3,tof4,estop,self_estop,ext_estop,active_mask,trip_mask,valid_mask,fault_mask,motion_mode,takeover*CS
```

无效距离使用：

```text
65535
```

联调阶段固件还会附带调试字段，例如：
- `R=`：收到字节数
- `F=`：成功帧数
- `S=`：同步错误数
- `C=`：校验错误数
- `E=`：UART 错误数
- `L=`：最后收到的字节
- `H=`：控制帧接收 / 解析错误 / 校验错误

上位机发给 H7 的控制帧为：

```text
$H7CTL,seq,vx_mmps,vy_mmps,wz_mradps,release_req,takeover_enable*CS
```

## ROS1 SDK

独立 SDK 路径：
- `sdk/h7_tof_safety_bridge_ros1`

这个 SDK 不依赖当前仓库的 `src/`，可以直接复制到新的 catkin 工作区里使用。

默认 ROS 接口：
- 订阅 `/cmd_vel`
- 订阅 `/h7_tof/takeover_enable`
- 发布 `/h7_tof/status`
- 发布 `/h7_tof/estop`
- 发布 `/h7_tof/raw_line`
- 服务 `/h7_tof/release_estop`

## 完整文档

完整架构、协议字段、集成步骤、验收清单见：
- `docs/H7_TOF_SAFETY_SYSTEM.md`
