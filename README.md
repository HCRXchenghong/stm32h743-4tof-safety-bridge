# stm32h743-4tof-safety-bridge

基于 `STM32H743ZITx` 的 4 路 UART TOF 板端采集与急停联调工程。

这个仓库现在保留的是一套可直接继续开发和上传 GitHub 的最小结构：
- `firmware/h743_tof_usb_bridge_cubeide`：主固件工程，`STM32CubeIDE` 可继续打开
- `tools/web_serial_dashboard`：本地 HTML 串口调试页面
- `release/stm32h743-4tof-safety-bridge.hex`：当前可烧录的预编译固件
- `激光DYP-RD-产品规格书(DYP-R01-V1.0)-250412-A0.pdf`：传感器规格书

推荐 GitHub 仓库名：
- `stm32h743-4tof-safety-bridge`

## 项目目标

- 采集 4 路 `DYP-R01` 激光 TOF 串口数据
- 板端通过 `USB CDC` 把 4 路距离和状态发给电脑
- 提供一个本地 HTML 页面查看串口数据
- 预留底盘 `低电平触发` 急停 IO
- 后续可继续补 ROS1 串口桥和安全策略

## 当前状态

当前版本已经打通：
- STM32 `USB CDC` 枚举
- 4 路 UART 接收框架
- TOF 二进制帧解析
- HTML Web Serial 联调
- 低电平急停 IO 基础逻辑

当前版本还没有定死：
- 防跌落策略阈值
- 急停锁存与人工复位策略
- ROS1 节点正式实现

## 目录结构

```text
H7-4路TOF/
├─ README.md
├─ 激光DYP-RD-产品规格书(DYP-R01-V1.0)-250412-A0.pdf
├─ firmware/
│  └─ h743_tof_usb_bridge_cubeide/
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

当前页面默认解析：

```text
$H7TOF,seq,tof1,tof2,tof3,tof4,estop,valid_mask,fault_mask*CS
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

## 推荐上传到 GitHub 的内容

当前仓库已经按精简结构整理，适合直接上传。

推荐仓库名：
- `stm32h743-4tof-safety-bridge`

推荐提交说明：

```text
Initial commit: STM32H743 4-TOF USB CDC bridge with web serial dashboard
```
