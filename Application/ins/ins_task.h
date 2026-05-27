#ifndef INS_TASK_H
#define INS_TASK_H

/**
 * @file    ins_task.h
 * @brief   Application层 — INS 惯性导航任务
 * @note    整合 BMI088 + Mahony + 蓝牙输出
 *          作为 FreeRTOS 任务运行
 *          严禁包含任何 MCU 相关头文件 (Rule 2)
 */

#include <stdint.h>

/* ======================== 任务接口 ======================== */

/**
 * @brief  INS 系统初始化 (调度器启动前调用)
 * @note   内部完成:
 *         1. BSP_SPI / BSP_UART6 初始化
 *         2. BMI088 初始化 + 5s零飘校准 (阻塞)
 *         3. Mahony 滤波器初始化
 *         4. 蓝牙实例初始化
 */
void INS_Init(void);

/**
 * @brief  INS FreeRTOS 任务函数 (200Hz)
 * @param  argument FreeRTOS 传入参数 (未使用)
 * @note   读 IMU → Mahony 解算 → 定期蓝牙发送
 */
extern void INS_Task(void *argument);
extern void BT_RxTask(void *argument);
extern void Gimbal_Task(void *argument);

// 跨任务暴露的云台全局姿态与急停标志
extern float global_gimbal_yaw;
extern float global_gimbal_pitch;
extern float global_imu_yaw_speed;
extern float global_imu_pitch_speed;
extern volatile uint8_t emergency_stop;

/* ======================== 数据获取 ======================== */

float INS_GetYaw(void);
float INS_GetPitch(void);
float INS_GetRoll(void);
float INS_GetYawTotal(void);
uint8_t INS_IsCalibrated(void);

#endif /* INS_TASK_H */
