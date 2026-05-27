# B题 MaixCam2 + STM32 双轴云台分步实施路线

本文用于后续 AI 辅助开发。目标是把题目、当前工程、硬件边界、通信协议和分阶段实现路径固定下来，避免后续重复讨论架构。

## 1. 当前决策

### 1.1 题目实现范围

- 实现物体识别瞄准装置。
- 不实现语音播报。
- 目标识别、目标排序、ROI 划分、中心点计算由 MaixCam2 完成。
- STM32 不做 YOLO、不做 OpenCV，只负责接收 MaixCam2 数据、控制双轴无刷云台、控制激光、回传状态。
- 显示使用 MaixCam2 自带屏幕。
- 调试数据最终通过 MaixCam2 经局域网回传上位机。

### 1.2 系统架构

```text
目标板
  -> MaixCam2 摄像头
  -> YOLO 目标锁定
  -> ROI 划分
  -> OpenCV 中心点计算
  -> MaixCam2 目标排序和流程主控
  -> UART6 发送命令给 STM32
  -> STM32 控制双轴云台和激光笔
  -> STM32 UART6 回传状态给 MaixCam2
  -> MaixCam2 局域网回传上位机
```

### 1.3 控制边界

| 模块 | 职责 |
|---|---|
| MaixCam2 | YOLO 检测、ROI、OpenCV 中心点、目标排序、目标切换、屏幕显示、局域网上位机转发 |
| STM32F407 大疆 C 板 | 双轴云台控制、IMU 姿态解算、电机 CAN 控制、激光控制、状态回传 |
| 上位机 | 只做调试监控，不参与现场人工干预 |

## 2. 当前工程状态

工程路径：`C:\Users\Administrator\Desktop\jixun2`

当前工程基础：

- MCU：STM32F407。
- 工具链：Keil MDK-ARM。
- 框架：STM32 HAL + FreeRTOS。
- 已有模块：
  - `Application/ins`：BMI088 姿态解算。
  - `Application/gimbal`：云台控制任务。
  - `Device/vision`：UART6 视觉数据解析，目前只支持 `"dx,dy\n"`。
  - `Device/bluetooth`：USART1 蓝牙调试协议。
  - `bsp/bsp_can`：CAN 电机控制。
  - `bsp/bsp_uart6`：UART6 视觉通信。
  - `bsp/bsp_uart`：USART1 蓝牙调试。

关键现状：

- UART6 已作为视觉串口，115200，PG14/PG9。
- USART1 保留为备用调试口。
- 当前 `Gimbal_Task` 只实际控制 Yaw，Pitch 输出被硬编码为 0。
- 当前还没有题目专用状态机。
- 当前还没有激光控制抽象。
- 当前激光控制引脚未定，后续补充。

## 3. MaixCam2 串口依据

官方文档要点：

- MaixCam2 有多组 UART，`UART0` 是系统终端和日志串口，不建议作为主通信口。
- MaixCam2 常用串口波特率建议先使用 `115200`。
- 串口硬件需要 TX/RX 交叉连接，并且双方 GND 共地。
- 官方示例中 MaixCam2 可用 `A21 -> UART4_TX`、`A22 -> UART4_RX`，设备为 `/dev/ttyS4`。

参考：

- https://wiki.sipeed.com/maixpy/doc/zh/peripheral/uart.html
- https://wiki.sipeed.com/maixpy/doc/zh/peripheral/pinmap.html

## 4. 串口与调试链路

### 4.1 串口分工

| 串口 | 当前用途 | 后续用途 |
|---|---|---|
| UART6，PG14/PG9，115200 | 视觉模块通信 | 主通信链路：STM32 <-> MaixCam2 |
| USART1，PA9/PB7，115200 | 蓝牙调试 | 保留为备用调试口 |

后续主链路：

```text
STM32 UART6 <-> MaixCam2 <-> LAN/WiFi <-> 上位机
```

原本需要通过两个串口查看的数据，后续全部汇总到 UART6 发给 MaixCam2；USART1 保留备用，不作为主调试链路。

### 4.2 带宽策略

UART6 为 115200，不能把 1kHz IMU、1kHz 云台控制量全量输出。

建议：

- 状态包：20Hz。
- 关键事件日志：事件触发。
- 高频原始数据：默认关闭，需要调参时再临时打开。

## 5. UART6 协议

协议采用 ASCII 行协议，所有帧以 `\n` 结束，允许行尾带 `\r\n`。

### 5.1 MaixCam2 -> STM32

命令统一以 `C` 开头。

#### 设置模式

```text
C,MODE,mode\n
```

示例：

```text
C,MODE,AIM
C,MODE,CALIB
C,MODE,IDLE
```

#### 当前目标

```text
C,AIM,id,dx,dy,conf\n
```

示例：

```text
C,AIM,3,-12,8,91
```

字段：

| 字段 | 含义 |
|---|---|
| `id` | 目标类别或目标编号，由 MaixCam2 定义 |
| `dx` | 目标中心相对视觉参考中心的水平像素偏差 |
| `dy` | 目标中心相对视觉参考中心的垂直像素偏差 |
| `conf` | 置信度，0 到 100 |

坐标约定先固定为：

```text
dx > 0：目标在画面右侧
dx < 0：目标在画面左侧
dy > 0：目标在画面下方
dy < 0：目标在画面上方
```

如果实测方向相反，只改 STM32 侧极性参数，不改协议含义。

#### 激光控制

```text
C,LASER,onoff\n
```

示例：

```text
C,LASER,1
C,LASER,0
```

#### 停止

```text
C,STOP\n
```

STM32 收到后应停止云台目标更新，关闭激光，进入安全状态。

#### 后续激光校准

激光笔和摄像头不同轴，后续需要校准。建议保留命令：

```text
C,CALIB,step\n
C,LASER_ERR,dx,dy\n
```

其中 `C,LASER_ERR` 由 MaixCam2 在识别到激光光斑后发送，表示激光点相对目标中心的偏差。

### 5.2 STM32 -> MaixCam2

状态与日志统一以 `M` 开头。

#### 状态包

```text
M,STAT,state,yaw,pitch,dx,dy,target_id,locked,laser\n
```

示例：

```text
M,STAT,IMU_STAB,12.34,-3.20,-2,1,3,1,0
```

字段：

| 字段 | 含义 |
|---|---|
| `state` | STM32 当前状态 |
| `yaw` | 当前云台 Yaw 角 |
| `pitch` | 当前云台 Pitch 角 |
| `dx` | 最近一次收到的目标水平偏差 |
| `dy` | 最近一次收到的目标垂直偏差 |
| `target_id` | 最近一次目标 ID |
| `locked` | 是否进入锁定阈值，0/1 |
| `laser` | 激光状态，0/1 |

#### 日志包

```text
M,LOG,level,message\n
```

示例：

```text
M,LOG,I,uart6_ready
M,LOG,W,vision_timeout
M,LOG,E,can_error
```

#### 应答包

```text
M,ACK,cmd,result\n
```

示例：

```text
M,ACK,STOP,OK
M,ACK,LASER,OK
```

## 6. 分阶段实施路线

### 阶段 0：冻结边界

目标：

- 不动视觉侧代码。
- 不动云台闭环。
- 先把通信与状态汇总框架搭好。

输出：

- 本文档。
- 后续所有 AI 任务都以本文档为当前架构依据。

### 阶段 1：UART6 主通信框架

目标：

- 把 UART6 从简单 `"dx,dy"` 解析升级为 `C,...` 协议解析。
- STM32 能接收 MaixCam2 的 `C,AIM`、`C,MODE`、`C,LASER`、`C,STOP`。
- STM32 能缓存最近目标数据。
- STM32 20Hz 发送 `M,STAT` 给 MaixCam2。
- USART1 保留备用调试输出。

建议改动：

- 新增或重构 `Device/maixlink`，负责行协议解析和状态封装。
- `Device/vision` 可保留旧接口，或迁移为 MaixLink 的目标数据接口。
- 在 FreeRTOS 中新增低频通信/遥测任务，避免把业务继续塞进 `INS_Task`。

验收：

- MaixCam2 发送：

```text
C,AIM,3,-12,8,91
```

- STM32 回传状态中能看到 `target_id=3`、`dx=-12`、`dy=8`。
- 不要求云台跟随。
- 不要求激光动作。

### 阶段 2：汇总调试数据到 MaixCam2

目标：

- 原本 USART1 输出的关键调试信息，汇总到 UART6。
- MaixCam2 通过局域网转发给上位机。
- USART1 只保留为备用，不作为主链路。

建议：

- 统一封装 `Telemetry_SendStatus()`、`Telemetry_SendLog()`。
- 禁止多个任务直接抢 UART6 发送。
- 状态包默认 20Hz。
- 日志包只在事件发生时发送。

验收：

- 上位机能看到 STM32 的姿态、云台状态、目标数据和错误状态。
- UART6 不丢包、不阻塞控制任务。

### 阶段 3：Pitch 轴启用前检查

目标：

- 当前工程 Pitch 没启用，必须先单独验证。

检查项：

- Pitch 电机 CAN ID 是否为 `0x01`。
- Pitch 电机反馈角度是否正常。
- Pitch 正负电流方向是否正确。
- IMU Pitch 方向是否正确。
- `PITCH_POLARITY` 是否需要反向。
- Pitch 机械限位范围。

验收：

- Pitch 能在小角度范围内稳定锁定。
- 不出现正反馈甩动。
- 急停能清零输出。

### 阶段 4：Yaw 单轴 dx 闭环

目标：

- 只用 `dx` 控制 Yaw。
- 不启用 Pitch。
- 不控制激光。

建议公式：

```text
target_yaw += dx * K_YAW
```

保护：

- `abs(dx) < deadband_x` 时不更新。
- 单次角度增量限幅。
- 视觉超时后停止更新目标。
- `conf` 低于阈值时不更新。

验收：

- MaixCam2 固定发送目标中心偏差。
- 云台能让目标水平误差逐步收敛到阈值内。
- 检测跳变时云台不抽动。

### 阶段 5：Pitch 加入 dy 闭环

目标：

- 在 Yaw 稳定后加入 Pitch。

建议公式：

```text
target_yaw   += dx * K_YAW
target_pitch += dy * K_PITCH
```

注意：

- `dy` 的极性很可能需要反向。
- Pitch 的机械限位必须比 Yaw 更保守。
- 先低增益，后调速度。

验收：

- 目标中心能在二维方向进入误差阈值。
- 靶心瞄准能稳定保持。

### 阶段 6：激光控制抽象

目标：

- 先建立接口，不绑定具体引脚。

建议接口：

```c
void Laser_Init(void);
void Laser_SetOn(uint8_t on);
uint8_t Laser_IsOn(void);
```

后续等引脚确定后再落 BSP GPIO 或 PWM。

验收：

- MaixCam2 发送 `C,LASER,1`，STM32 状态包 `laser=1`。
- MaixCam2 发送 `C,LASER,0`，STM32 状态包 `laser=0`。
- 引脚未确定前只更新软件状态，不实际驱动硬件。

### 阶段 7：激光非同轴校准

背景：

- 摄像头和激光笔不是同轴，不能只靠目标中心 `dx,dy` 直接认为激光已命中。
- 后续必须让 MaixCam2 同时识别目标中心和激光光斑中心。

推荐流程：

```text
1. MaixCam2 识别目标中心。
2. STM32 控制云台让目标中心进入视觉中心。
3. 打开激光。
4. MaixCam2 识别激光光斑。
5. MaixCam2 计算 laser_dx, laser_dy。
6. STM32 根据 C,LASER_ERR 修正激光偏移补偿。
7. 保存补偿参数。
```

补偿参数：

```text
laser_offset_x
laser_offset_y
```

后续瞄准目标时使用：

```text
corrected_dx = target_dx + laser_offset_x
corrected_dy = target_dy + laser_offset_y
```

验收：

- 激光光斑能稳定落到目标中心附近。
- 目标移动后能重新收敛。

### 阶段 8：MaixCam2 主控目标切换

目标：

- 目标排序由 MaixCam2 完成。
- STM32 不管理目标顺序，只执行当前目标。

推荐流程：

```text
MaixCam2:
  识别所有目标
  从左到右排序
  发送当前目标 C,AIM
  等待 STM32 回传 locked=1
  保持或计时 3s
  切换下一个目标

STM32:
  接收 C,AIM
  控制云台收敛
  判断 locked
  回传 M,STAT
```

验收：

- MaixCam2 能依次切目标。
- STM32 能稳定跟随当前目标。
- 每个目标停留时间由 MaixCam2 主控。

### 阶段 9：完成基本要求

对应题目基本要求：

1. 初始靶心瞄准：
   - MaixCam2 识别靶心。
   - STM32 根据 `dx,dy` 闭环瞄准。
   - 激光补偿后落到靶心。

2. 从左到右识别并逐个瞄准：
   - MaixCam2 识别、排序、显示。
   - STM32 跟随当前目标。
   - MaixCam2 根据 `locked` 和计时切换目标。

3. 目标板左右随机移动后重新瞄准：
   - MaixCam2 持续识别靶心。
   - STM32 持续闭环修正。

验收指标：

- 激光光斑直径 <= 0.5 cm。
- 每个目标停留 >= 3 s。
- 基本要求总时间满足题目限制。

### 阶段 10：发挥部分

#### 10.1 沿最外围圆运动一周

建议两种方案：

方案 A：MaixCam2 主控轨迹点。

```text
MaixCam2 根据靶心和最外围圆半径生成一圈目标点
逐点发送 C,AIM 或后续扩展 C,TRACE
STM32 低层平滑跟随
```

方案 B：STM32 生成角度轨迹。

```text
完成板面坐标到云台角度标定后
STM32 根据圆心和半径生成 yaw/pitch 轨迹
```

当前更推荐方案 A，因为显示、视觉和流程已经由 MaixCam2 主控。

#### 10.2 记忆顺序自动瞄准

由 MaixCam2 存储和执行目标顺序，STM32 只执行当前目标。

#### 10.3 5 分钟自动停止与计数显示

由于当前不实现语音播报：

- MaixCam2 负责 5 分钟计时。
- MaixCam2 屏幕显示已瞄准数量。
- MaixCam2 到时发送 `C,STOP`。
- STM32 停止云台目标更新并关闭激光。

## 7. 后续代码实现建议

### 7.1 STM32 新增模块

建议新增：

```text
Device/maixlink/maixlink.h
Device/maixlink/maixlink.c
Application/telemetry/telemetry_task.h
Application/telemetry/telemetry_task.c
Application/laser/laser.h
Application/laser/laser.c
```

职责：

| 模块 | 职责 |
|---|---|
| `maixlink` | UART6 行协议解析、命令缓存、状态打包 |
| `telemetry_task` | 20Hz 状态回传、事件日志发送 |
| `laser` | 激光抽象接口，后续绑定 GPIO/PWM |

### 7.2 不建议继续扩张的文件

- 不建议继续把题目业务逻辑塞进 `Application/ins/ins_task.c`。
- 不建议让多个任务直接调用 `BSP_UART6_SendBytes()`。
- 不建议把 MaixCam2 协议继续混在旧 `vision.c` 的 `"dx,dy"` 简单解析里。

### 7.3 后续优先任务

优先级从高到低：

1. 实现 UART6 `C,... / M,...` 协议。
2. 汇总 STM32 状态到 MaixCam2。
3. 验证 Pitch 电机闭环。
4. Yaw 单轴 `dx` 闭环。
5. Pitch + Yaw 双轴 `dx,dy` 闭环。
6. 激光控制接口。
7. 激光非同轴校准。
8. MaixCam2 主控目标流程联调。
9. 基本要求验收。
10. 发挥部分轨迹和超时停止。

## 8. 给后续 AI 的任务提示词

```text
当前项目是 2026 杭州电子科技大学电子设计竞赛 B 题：物体智能识别瞄准装置。
请先阅读：
- B题_物体智能识别瞄准装置_题目总结.md
- B题_MaixCam2_STM32_分步实施路线.md

当前架构：
- MaixCam2 负责 YOLO、ROI、OpenCV 中心点、目标排序、目标切换、屏幕显示、局域网回传。
- STM32F407 大疆 C 板负责双轴无刷云台、激光控制、状态回传。
- 主通信为 UART6，PG14/PG9，115200。
- USART1 蓝牙调试口保留备用。
- 协议采用 ASCII 行：MaixCam2 发 C,...，STM32 发 M,...
- 当前不实现语音播报。
- 激光引脚未定，先做抽象接口。
- dx,dy 闭环后续实测再接入，第一步只做通信和状态汇总。

本次任务：
-

要求：
- 先检查相关源码和头文件。
- 最小侵入修改。
- 不破坏现有 INS、CAN、电机和 UART1 备用调试。
- 修改后说明验证方式。
```

