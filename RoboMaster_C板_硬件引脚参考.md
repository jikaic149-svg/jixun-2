# RoboMaster 开发板 C 型硬件引脚参考

本文用于后续 AI 生成 STM32 代码时查阅硬件资源、引脚分配和当前工程实际占用情况。

校对来源：

- `RoboMaster  开发板 C 型用户手册.pdf`，版本 `v1.0 / 2020.01`。
- 当前工程 `spi_bmi088.ioc`。
- 当前工程源码：`Src/gpio.c`、`Src/spi.c`、`Src/tim.c`、`Src/can.c`、`bsp/bsp_uart.c`、`bsp/bsp_uart6.c`、`bsp/bsp_can.c`。

## 1. 核心硬件参数

| 项目 | 参数 |
|---|---|
| 开发板 | RoboMaster 开发板 C 型 |
| 主控 | STM32F407，当前工程 `.ioc` 为 `STM32F407IGHx` |
| 输入电压 | 8 V ~ 28 V |
| 待机电流 | 0.01 A @ DC 24 V |
| 尺寸 | 60 x 41 x 16.3 mm |
| 重量 | 38 g |
| 工作温度 | 0 ~ 55 摄氏度 |

电源注意：

- XT30 电源线：红色为正极，黑色为负极。
- 开发板有输入防反接、缓启动、过压保护。
- 输入超过 28 V 时后级电路会关断。
- USB 供电只供给 `VCC_5V`，不支持给 `VCC_5V_M` 供电，因此不能靠 USB 给 PWM 舵机接口供电。

## 2. 当前项目已实际启用的硬件资源

这张表以当前工程源码为准，不是单纯的手册能力表。

| 功能 | STM32 外设 / IO | 当前用途 | 当前工程位置 | 注意 |
|---|---|---|---|---|
| BMI088 SPI | `SPI1_SCK PB3`、`SPI1_MOSI PA7`、`SPI1_MISO PB4` | BMI088 读写 | `Src/spi.c`、`Application/ins/ins_task.c` | SPI1 已占用 |
| BMI088 片选 | `CS1_Accel PA4`、`CS1_Gyro PB0` | BMI088 片选 | `Src/gpio.c`、`bsp/bsp_spi.c` | GPIO 输出，默认置高 |
| BMI088 数据就绪 | `INT1_Accel PC4`、`INT1_Gyro PC5` | EXTI 下降沿中断 | `Src/gpio.c`、`Application/ins/ins_task.c` | EXTI4、EXTI9_5 已占用 |
| BMI088 加热 | `TIM10_CH1 PF6` | IMU 加热 PWM | `Src/tim.c` | 不要作为激光或普通 PWM 使用 |
| CAN1 | `CAN1_RX PD0`、`CAN1_TX PD1` | 双轴无刷云台电机通信 | `Src/can.c`、`bsp/bsp_can.c` | 当前用于 Yaw/Pitch 电机 |
| UART6 | `UART6_TX PG14`、`UART6_RX PG9` | MaixCam2 主通信 | `bsp/bsp_uart6.c` | 115200，RXNE 中断，TX DMA |
| USART1 | `UART1_TX PA9`、`UART1_RX PB7` | 蓝牙 / 备用调试 | `bsp/bsp_uart.c` | 115200，RXNE 中断，TX DMA |
| 用户按键 | `KEY PA0` | 模式切换 / 急停 / 发送测试命令 | `Src/gpio.c`、`Application/ins/ins_task.c` | 按下为低电平；当前逻辑有复用 |
| 用户 LED 绿灯 | `LED_G PH11` | 正常状态指示 | `Src/gpio.c`、`Application/ins/ins_task.c` | 高电平点亮 |
| 用户 LED 红灯 | `LED_R PH12` | 急停状态指示 | `Src/gpio.c`、`Application/ins/ins_task.c` | 高电平点亮 |

当前未启用但后续重点可能使用：

| 功能 | STM32 外设 / IO | 推荐用途 | 状态 |
|---|---|---|---|
| 板载 5V 激光接口控制 | `TIM3_CH3 PC8` | 激光开关或 PWM 调光 | 当前工程未配置 |
| 用户 LED 蓝灯 | `LED_B PH10` | 状态指示扩展 | 当前工程未配置 |
| 蜂鸣器 | `TIM4_CH3 PD14` | 蜂鸣提示 | 当前工程未配置 |
| 电池电压检测 | `ADC_BAT PF10` | 24V 输入电压检测 | 当前工程未配置 |

## 3. 串口接口

### 3.1 重要丝印差异

手册明确说明：开发板外壳丝印 `UART1` / `UART2` 与 STM32 实际串口不对应。

| 外壳丝印 | 物理接口 | STM32 实际串口 |
|---|---|---|
| `UART1` | 3-pin UART 接口 | `UART6` |
| `UART2` | 4-pin UART 接口 | `UART1` |

后续写代码必须以 STM32 外设名为准，不要以外壳丝印为准。

### 3.2 UART6，3-pin，当前用于 MaixCam2

手册线序：

| 物理 Pin | 信号 | STM32 IO |
|---:|---|---|
| 1 | GND | GND |
| 2 | TXD | `UART6_TX PG14` |
| 3 | RXD | `UART6_RX PG9` |

当前工程配置：

| 项目 | 配置 |
|---|---|
| 波特率 | 115200 |
| 数据格式 | 8N1 |
| TX | DMA2 Stream6 Channel5 |
| RX | RXNE 中断逐字节 |
| NVIC | `USART6_IRQn`、`DMA2_Stream6_IRQn` |

MaixCam2 接线建议：

```text
C 板 UART6 Pin1 GND  -> MaixCam2 GND
C 板 UART6 Pin2 TXD  -> MaixCam2 RX
C 板 UART6 Pin3 RXD  <- MaixCam2 TX
```

注意：

- UART6 接口没有 5V 供电脚，MaixCam2 需要独立供电。
- 必须共地。
- TX/RX 需要交叉。
- 该串口是后续主链路：`STM32 <-> MaixCam2 <-> 局域网上位机`。

### 3.3 UART1，4-pin，当前作为备用调试

手册线序：

| 物理 Pin | 信号 | STM32 IO |
|---:|---|---|
| 1 | RXD | `UART1_RX PB7` |
| 2 | TXD | `UART1_TX PA9` |
| 3 | GND | GND |
| 4 | 5V | VCC_5V |

当前工程配置：

| 项目 | 配置 |
|---|---|
| 波特率 | 115200 |
| 数据格式 | 8N1 |
| TX | DMA2 Stream7 Channel4 |
| RX | RXNE 中断逐字节 |
| NVIC | `USART1_IRQn`、`DMA2_Stream7_IRQn` |

用途：

- 保留为蓝牙或 USB-TTL 备用调试口。
- 后续主调试数据应汇总到 UART6 发给 MaixCam2。

## 4. CAN 接口

### 4.1 CAN1，2-pin，当前用于云台电机

手册线序：

| 物理 Pin | 信号 | STM32 IO |
|---:|---|---|
| 1 | CANL | `CAN1_RX/TX` 经收发器连接 |
| 2 | CANH | `CAN1_RX/TX` 经收发器连接 |

STM32 引脚：

| 网络名 | STM32 IO |
|---|---|
| `CAN1_RX` | `PD0` |
| `CAN1_TX` | `PD1` |

当前工程配置：

| 项目 | 配置 |
|---|---|
| 外设 | CAN1 |
| 模式 | Normal |
| 引脚复用 | AF9 |
| 中断 | `CAN1_RX0_IRQn`、`CAN1_RX1_IRQn` |
| 当前用途 | 双轴无刷云台电机 |

当前电机 ID 约定：

| 电机 | ID |
|---|---:|
| Yaw | `0x00` |
| Pitch | `0x01` |

### 4.2 CAN2，4-pin，当前未启用

手册线序：

| 物理 Pin | 信号 | STM32 IO |
|---:|---|---|
| 1 | 5V | VCC_5V |
| 2 | GND | GND |
| 3 | CANH | `CAN2_RX/TX` 经收发器连接 |
| 4 | CANL | `CAN2_RX/TX` 经收发器连接 |

STM32 引脚：

| 网络名 | STM32 IO |
|---|---|
| `CAN2_RX` | `PB5` |
| `CAN2_TX` | `PB6` |

注意：当前工程没有初始化 CAN2。

## 5. 板载 5V 激光接口

手册说明：开发板集成一个可控 5V 电源接口，可外接 RoboMaster 红点激光器；对应开关控制 IO 为 `PC8`，网络名为 `TIM3_CH3`。

| 功能 | 网络名 | STM32 IO |
|---|---|---|
| 5V 激光接口开关 / PWM | `TIM3_CH3` | `PC8` |

当前工程状态：

- `PC8` 当前未配置。
- `TIM3` 当前未初始化。
- 后续若使用板载 5V 激光接口，优先使用 `PC8`。
- 若只需要开关，配置 `PC8` 为 GPIO 输出即可。
- 若需要亮度调节，配置 `TIM3_CH3` PWM。

严禁误用：

- `PF6 / TIM10_CH1` 是 BMI088 加热控制，不是激光接口。

## 6. 用户 LED 和按键

### 6.1 RGB LED

手册说明：开发板集成 1 颗 RGB LED，对应 IO 如下；IO 输出高电平时对应 LED 点亮，输出低电平时熄灭。

| LED | STM32 IO | 当前工程状态 |
|---|---|---|
| 红灯 `LED_R` | `PH12` | 已配置，用于急停指示 |
| 绿灯 `LED_G` | `PH11` | 已配置，用于正常状态指示 |
| 蓝灯 `LED_B` | `PH10` | 当前未配置 |

### 6.2 用户按键 KEY

| 功能 | STM32 IO | 电平 |
|---|---|---|
| 用户按键 `KEY` | `PA0` | 按下为低电平 |

当前工程中 `PA0` 已被用于：

- 从 `GIMBAL_STATE_RELAX` 切入编码器硬锁 / 校准。
- 运行后再次按下触发急停。
- 发送视觉测试命令 `T`。

注意：后续如果增加按键逻辑，必须先整理 `PA0` 当前多重用途，避免同一次按键触发多个行为。

## 7. BMI088 IMU 与加热

手册附表与当前工程一致。

| 功能 | 网络名 | STM32 IO | 当前工程状态 |
|---|---|---|---|
| 加热 PWM | `TIM10_CH1` | `PF6` | 已配置 |
| 加速度计中断 | `INT1_Accel` | `PC4` | EXTI4，下降沿 |
| 陀螺仪中断 | `INT1_Gyro` | `PC5` | EXTI9_5，下降沿 |
| 加速度计片选 | `CS1_Accel` | `PA4` | GPIO 输出 |
| 陀螺仪片选 | `CS1_Gyro` | `PB0` | GPIO 输出 |
| SPI 时钟 | `SPI1_CLK` | `PB3` | SPI1 |
| SPI MOSI | `SPI1_MOSI` | `PA7` | SPI1 |
| SPI MISO | `SPI1_MISO` | `PB4` | SPI1 |

当前 SPI1 配置：

| 项目 | 配置 |
|---|---|
| SPI 模式 | Master |
| 数据位 | 8 bit |
| CPOL | High |
| CPHA | 2 Edge |
| NSS | Software |
| Prescaler | 8 |
| 计算速率 | 约 10.5 Mbit/s |
| RX DMA | DMA2 Stream2 Channel3 |
| TX DMA | DMA2 Stream3 Channel3 |

## 8. 可配置 I/O 接口，8-pin

手册线序：

| 物理 Pin | 信号 | STM32 IO |
|---:|---|---|
| 1 | `SPI2_CS` | `PB12` |
| 2 | GND | GND |
| 3 | `SPI2_CLK` | `PB13` |
| 4 | 3.3V | VCC_3V3 |
| 5 | `SPI2_MOSI` | `PB15` |
| 6 | `I2C2_SCL` | `PF1` |
| 7 | `SPI2_MISO` | `PB14` |
| 8 | `I2C2_SDA` | `PF0` |

注意：

- 默认通信电平为 3.3V。
- 若要使用 5V 外设，需要手动焊接 `R210` 并去除 `R209`。
- 当前工程未初始化 SPI2/I2C2。

## 9. PWM / DBUS 复合接口

PWM 与 DBUS 共用 24-pin 连接器。

电源排布：

| 排 | 功能 |
|---|---|
| A1 ~ A8 | GND |
| B1 ~ B8 | 5V，来自 `VCC_5V_M` |
| C1 ~ C7 | 7 路 PWM |
| C8 | DBUS |

PWM 信号：

| 连接器 Pin | 网络名 | STM32 IO |
|---|---|---|
| C1 | `TIM1_CH1` | `PE9` |
| C2 | `TIM1_CH2` | `PE11` |
| C3 | `TIM1_CH3` | `PE13` |
| C4 | `TIM1_CH4` | `PE14` |
| C5 | `TIM8_CH1` | `PC6` |
| C6 | `TIM8_CH2` | `PI6` |
| C7 | `TIM8_CH3` | `PI7` |

DBUS：

| 连接器 Pin | 网络名 | STM32 IO |
|---|---|---|
| C8 | `DBUS` | `UART3_RX PC11` |

注意：

- 7 路 PWM 接口总输出电流最大 5A。
- DBUS 信号经过反相电路后连接到 `UART3_RX PC11`。
- DBUS 常用波特率为 100 kbps。
- 当前工程未初始化 TIM1/TIM8/UART3。

## 10. 数字摄像头 FPC 接口，18-pin

当前项目使用 MaixCam2 外部视觉模块，不使用开发板 FPC 摄像头接口。后续除非明确切换到 STM32 DCMI 摄像头，否则不要启用这些引脚。

| 网络名 | STM32 IO |
|---|---|
| `I2C1_SCL` | `PB8` |
| `I2C1_SDA` | `PB9` |
| `PCLK_OUT` | `PA6` |
| `DCMI_HREF` | `PH8` |
| `DCMI_VSYNC` | `PI5` |
| `DCMI_D0` | `PH9` |
| `DCMI_D1` | `PC7` |
| `DCMI_D2` | `PE0` |
| `DCMI_D3` | `PE1` |
| `DCMI_D4` | `PE4` |
| `DCMI_D5` | `PI4` |
| `DCMI_D6` | `PE5` |
| `DCMI_D7` | `PE6` |

## 11. 其他板载资源

| 功能 | 网络名 | STM32 IO | 当前工程状态 |
|---|---|---|---|
| 蜂鸣器 | `TIM4_CH3` | `PD14` | 未配置 |
| 电压检测 | `ADC_BAT` | `PF10` | 未配置 |
| USB DM | `USB_DM` | `PA11` | 未配置 |
| USB DP | `USB_DP` | `PA12` | 未配置 |
| USB OTG ID | `USB_OTG` | `PA10` | 未配置 |
| 磁力计复位 | `RSTN_IST8310` | `PG6` | 未配置 |
| 磁力计数据就绪 | `DRDY_IST8310` | `PG3` | 未配置 |
| 磁力计 I2C 时钟 | `I2C3_SCL` | `PA8` | 未配置 |
| 磁力计 I2C 数据 | `I2C3_SDA` | `PC9` | 未配置 |

磁力计说明：

- 芯片为 IST8310。
- I2C 默认地址为 `0x0E`。
- 当前项目未使用磁力计。

## 12. SWD、BOOT、USB 下载

### 12.1 SWD 下载线线序

手册给出的 SWD 下载线线序：

| 线序 | 信号 |
|---|---|
| A | SWDIO |
| B | SWCLK |
| C | GND |
| D | 3.3V |

### 12.2 BOOT 配置

默认：

| BOOT1 | BOOT0 | 启动模式 |
|---|---|---|
| X | 0 | 用户 Flash |

DFU：

| BOOT1 | BOOT0 | 启动模式 |
|---|---|---|
| 0 | 1 | System Memory / DFU |

### 12.3 micro USB

用途：

- USB 通信。
- 配合 BOOT 配置进入 DFU 下载。
- USB 供电只供给 `VCC_5V`，不能驱动 `VCC_5V_M` 上的 PWM 舵机接口。

## 13. 后续 AI 写代码时的硬约束

1. 主通信链路使用 `UART6 PG14/PG9`，不是外壳丝印名。
2. `UART6` 物理 3-pin 接口线序是 `GND / TXD / RXD`。
3. MaixCam2 与 C 板必须共地，TX/RX 交叉。
4. `USART1 PA9/PB7` 保留为备用调试口，不要删除。
5. CAN 云台电机使用 `CAN1 PD0/PD1`。
6. `PF6 / TIM10_CH1` 已用于 BMI088 加热，不得作为激光控制。
7. 板载 5V 激光接口控制脚是 `PC8 / TIM3_CH3`，当前工程尚未配置。
8. `PA0` 当前已有按键逻辑复用，新增功能前必须先整理按键事件。
9. 当前工程没有初始化 CAN2、SPI2、I2C2、TIM1、TIM8、UART3、DCMI、ADC、TIM3。
10. 如果要新增外设，先检查是否和上表已占用引脚冲突。

