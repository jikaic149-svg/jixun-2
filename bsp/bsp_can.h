#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "main.h"
#include "config.h"

// 定义电机的 ID
#define YAW_MOTOR_ID   0x00
#define PITCH_MOTOR_ID 0x01

// 电机反馈数据结构体
typedef struct {
    uint8_t enabled;   // 使能状态: 1-使能, 0-失能
    float current;     // 实际电流 (A)
    float speed;       // 实际转速 (rpm)
    float angle;       // 编码器当前单圈角度 (deg), 范围 [0, 360]
    float last_angle;  // 上一次更新的单圈角度 (deg)
    float total_angle; // 当前等价角度 (deg)
    uint32_t rx_count;
    uint32_t last_rx_tick;
    uint8_t init_flag; // 初始帧获取标志
} Motor_Measure_t;

extern Motor_Measure_t yaw_motor;
extern Motor_Measure_t pitch_motor;

void CAN_Filter_Init(void);
void Motor_Enable(uint8_t id);
void Motor_Disable(uint8_t id);
void Motor_SetCurrent(uint8_t id, float current);
void Motor_SetAngle(uint8_t id, float angle);
uint8_t Motor_FeedbackReady(const Motor_Measure_t *motor, uint32_t now_tick, uint32_t timeout_ms);

#endif /* __BSP_CAN_H */
