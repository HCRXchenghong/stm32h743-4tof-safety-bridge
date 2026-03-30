# stm32h743-4tof-safety-bridge

基于 `STM32H743ZITx` 的 4 路 UART TOF 采集、USB CDC 上报与急停联动固件。

## 项目目标

- 采集 4 路 `DYP-R01` TOF UART 数据
- 通过 `USB CDC` 向上位机输出距离、状态和安全位图
- 提供本地 `HTML + Web Serial` 调试页面
- 预留底盘急停输入输出联动
- 为后续 ROS1 / 上位机控制接入保留协议

## TOF 安装定义

- `TOF1 = 右前`
- `TOF2 = 右后`
- `TOF3 = 左后`
- `TOF4 = 左前`

所有位图统一按 `bit0..bit3 = TOF1..TOF4` 编码。

## 安全策略

### 安全窗口

- 基准值：`391 mm`
- 容差：`±28 mm`
- 安全范围：`363 mm ~ 419 mm`

当前生效的 TOF 通道只要超出上述范围，即判定为越界。

### 生效规则

- 前进：启用 `TOF1 + TOF4`
- 后退：启用 `TOF2 + TOF3`
- 转向：启用全部 TOF
- 横移 / 混合运动：启用全部 TOF
- 静止：启用全部 TOF
- 控制输入超时：启用全部 TOF，进入失联保护

### 急停规则

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

| 通道 | 外设 | MCU TX | MCU RX | 物理方位 |
|---|---|---|---|---|
| TOF1 | USART1 | PA9 | PA10 | 右前 |
| TOF2 | USART6 | PC6 | PC7 | 右后 |
| TOF3 | USART3 | PD8 | PD9 | 左后 |
| TOF4 | USART2 | PD5 | PD6 | 左前 |

说明：

- 对自动输出 TOF 来说，最关键的是 `Sensor TX -> MCU RX`
- 如果只做最小验证，`MCU TX` 可先不接

### 急停 IO

- `PE10 = ESTOP_OUT_N`
- `PE11 = ESTOP_IN_N`

电平逻辑：

- `PE10` 输出低电平：触发底盘急停
- `PE10` 输出高电平：释放急停
- `PE11` 被外部拉低：请求急停

GPIO 配置：

- `PE10`：开漏输出，上拉，默认高
- `PE11`：输入，上拉

## 控制输入协议

固件当前支持通过 `USB CDC` 接收简化控制输入，用于决定当前生效 TOF 掩码。

### 文本模式

单行命令，以 `\r\n` 或 `\n` 结尾：

```text
FWD
REV
TURN
STRAFE
MIXED
IDLE
```

也兼容：

```text
MODE:FWD
CMD,REV
```

### JSON 模式

支持按模式名发送：

```json
{"mode":"FWD"}
```

也支持按速度分量发送：

```json
{"vx":1.0,"vy":0.0,"wz":0.0}
```

分类规则：

- 仅 `vx > 0`：前进
- 仅 `vx < 0`：后退
- 仅 `wz != 0`：转向
- 仅 `vy != 0`：横移
- 多轴同时非零：混合运动
- 全部接近零：静止

若超过约 `200 ms` 未收到新控制输入，固件自动进入失联保护并启用全部 TOF。

## STM32 输出协议

当前输出主格式：

```text
$H7TOF,seq,tof1,tof2,tof3,tof4,estop,valid_mask,fault_mask,R=... ,F=... ,S=... ,C=... ,E=... ,L=...,A=active_mask,M=motion_class,SE=self_estop,EE=external_estop*CS
```

其中：

- `tof1..tof4`：4 路测距，单位 `mm`
- `estop`：总急停状态
- `valid_mask`：当前 4 路是否有有效帧
- `fault_mask`：当前生效 TOF 中是否故障 / 越界
- `A`：当前生效通道位图
- `M`：运动分类枚举
- `SE`：`self_estop`
- `EE`：`external_estop`

无效距离使用：

```text
65535
```

## 目录结构

```text
stm32h743-4tof-safety-bridge/
|-- README.md
|-- firmware/
|   `-- h743_tof_usb_bridge_cubeide/
|-- tools/
|   `-- web_serial_dashboard/
|       `-- index.html
`-- release/
    `-- stm32h743-4tof-safety-bridge.hex
```

## 固件工程

CubeIDE 主工程路径：

- `firmware/h743_tof_usb_bridge_cubeide`

核心源码：

- `Core/Src/main.c`
- `USB_DEVICE/App/usbd_cdc_if.c`
- `Core/Src/stm32h7xx_hal_msp.c`
- `Core/Src/stm32h7xx_it.c`
- `h743_tof_usb_bridge_cubeide.ioc`

## 编译与烧录

### 直接使用预编译固件

- `release/stm32h743-4tof-safety-bridge.hex`

推荐烧录方式：

- `STM32CubeProgrammer`
- `USB DFU`
- `ST-LINK`

### 本地一键生成 hex

项目根目录提供本地脚本：

```powershell
powershell -ExecutionPolicy Bypass -File .\local-build-hex.ps1
```

会自动生成：

- `firmware/h743_tof_usb_bridge_cubeide/Debug/1123.elf`
- `firmware/h743_tof_usb_bridge_cubeide/Debug/1123.hex`
- `release/stm32h743-4tof-safety-bridge.hex`

### CubeIDE 编译

1. 打开 `firmware/h743_tof_usb_bridge_cubeide`
2. 用 `STM32CubeIDE` 导入或打开工程
3. 选择 `Debug`
4. 执行 `Build Project`

如果本机 CubeIDE 工具链环境异常，可先用根目录脚本生成固件；当前仓库也已按本机环境补齐了更稳的本地构建路径。

## HTML 调试页面

页面路径：

- `tools/web_serial_dashboard/index.html`

启动方式：

```powershell
py -m http.server 8000
```

然后打开：

```text
http://localhost:8000/tools/web_serial_dashboard/index.html
```

说明：

- 推荐 `Chrome` 或 `Edge`
- 页面解析 `H7TOF` 文本帧
- 也兼容简化 JSON 输入
- HTML 与 ROS / 其他串口工具不能同时占用同一 CDC 口
