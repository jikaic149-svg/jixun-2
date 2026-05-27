/**
 * @file    freertos.c
 * @brief   FreeRTOS 任务创建与系统启动
 * @note    遵循 TI.md Rule 5: RMS 优先级调度
 *
 *          任务优先级 (值越大优先级越高):
 *          - INS_Task:  5 (高) — IMU 读取 + Mahony 解算 @ 200Hz
 *          - BT_RxTask: 3 (中) — 蓝牙接收处理
 *          - Idle:      0 (最低) — FreeRTOS 内建
 */

#include "FreeRTOS.h"
#include "task.h"
#include "ins_task.h"
#include "gimbal_task.h"

/* ======================== 任务句柄 ======================== */

static TaskHandle_t ins_task_handle   = NULL;
static TaskHandle_t bt_rx_task_handle = NULL;
static TaskHandle_t gimbal_task_handle = NULL;

/* ======================== 任务栈大小 ======================== */

#define INS_TASK_STACK_SIZE     512U    /* 2KB (含 Mahony 浮点运算) */
#define BT_RX_TASK_STACK_SIZE   256U    /* 1KB */
#define GIMBAL_TASK_STACK_SIZE  256U    /* 1KB 控制环 */

/* ======================== 任务优先级 ======================== */

#define INS_TASK_PRIORITY       5U      /* 最高: IMU 硬实时 */
#define GIMBAL_TASK_PRIORITY    4U      /* 次高: 云台控制 1000Hz */
#define BT_RX_TASK_PRIORITY     3U      /* 中: 蓝牙非实时 */

/* ======================== 创建任务 ======================== */

void FreeRTOS_Init(void)
{
    xTaskCreate(INS_Task,
                "INS",
                INS_TASK_STACK_SIZE,
                NULL,
                INS_TASK_PRIORITY,
                &ins_task_handle);

    xTaskCreate(BT_RxTask,
                "BT_RX",
                BT_RX_TASK_STACK_SIZE,
                NULL,
                BT_RX_TASK_PRIORITY,
                &bt_rx_task_handle);

    xTaskCreate(Gimbal_Task,
                "GIMBAL",
                GIMBAL_TASK_STACK_SIZE,
                NULL,
                GIMBAL_TASK_PRIORITY,
                &gimbal_task_handle);
}

/* ======================== 钩子函数 ======================== */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    /* 栈溢出: 死循环便于调试 */
    for (;;);
}

void vApplicationMallocFailedHook(void)
{
    /* 堆分配失败: 死循环便于调试 */
    for (;;);
}
