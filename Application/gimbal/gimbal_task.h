#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "stdint.h"

typedef enum {
    GIMBAL_STATE_RELAX = 0,
    GIMBAL_STATE_ENC_LOCK,
    GIMBAL_STATE_CALIBRATING,
    GIMBAL_STATE_IMU_STAB,
    GIMBAL_STATE_GRAVITY_CALIB,
    GIMBAL_STATE_IDLE,
    GIMBAL_STATE_ANGLE_TEST,
    GIMBAL_STATE_TORQUE_TEST,
    GIMBAL_STATE_VISION_STAB
} GimbalState_e;

extern volatile GimbalState_e gimbal_state;
extern float ext_target_yaw;
extern uint8_t flag_ext_yaw;
extern volatile uint8_t emergency_stop;
extern volatile uint8_t gimbal_stab_target_reset;
extern volatile int16_t vision_raw_dx;
extern volatile int16_t vision_raw_dy;
extern volatile uint8_t vision_raw_valid;
extern volatile uint32_t vision_raw_frame_count;
extern volatile uint32_t vision_raw_frame_seq;

extern void Gimbal_Task(void *argument);

#endif /* GIMBAL_TASK_H */
