#include "gimbal_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "bsp_can.h"
#include "main.h"
#include "MODE_GRAVITY_CALIB.h"
#include "gimbal_mode_scheduler.h"

volatile GimbalState_e gimbal_state = GIMBAL_STATE_IDLE;
float ext_target_yaw = 0.0f;
uint8_t flag_ext_yaw = 0;
volatile uint8_t emergency_stop = 0;
volatile uint8_t gimbal_stab_target_reset = 1U;
volatile int16_t vision_raw_dx = 0;
volatile int16_t vision_raw_dy = 0;
volatile uint8_t vision_raw_valid = 0;
volatile uint32_t vision_raw_frame_count = 0;
volatile uint32_t vision_raw_frame_seq = 0;

static float limit_float(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static float pitch_control_angle(float enc_deg)
{
    if (enc_deg > 180.0f) enc_deg -= 360.0f;
    return enc_deg;
}

static void apply_pitch_gravity_feedforward(GimbalModeOutput_t *out)
{
    if (gimbal_state == GIMBAL_STATE_IDLE ||
        gimbal_state == GIMBAL_STATE_RELAX ||
        emergency_stop) {
        return;
    }

    out->pitch_current += MODE_GRAVITY_CALIB_GetPitchGravityFeedforward(pitch_motor.angle);
}

static void update_state_led(GimbalState_e state)
{
    GPIO_PinState blue = GPIO_PIN_RESET;
    GPIO_PinState green = GPIO_PIN_RESET;
    GPIO_PinState red = GPIO_PIN_RESET;

    switch (state) {
        case GIMBAL_STATE_RELAX:
        case GIMBAL_STATE_IDLE:
            blue = GPIO_PIN_SET;
            break;
        case GIMBAL_STATE_ENC_LOCK:
            green = GPIO_PIN_SET;
            break;
        case GIMBAL_STATE_CALIBRATING:
            red = GPIO_PIN_SET;
            green = GPIO_PIN_SET;
            break;
        case GIMBAL_STATE_IMU_STAB:
            red = GPIO_PIN_SET;
            break;
        case GIMBAL_STATE_VISION_STAB:
            red = GPIO_PIN_SET;
            blue = GPIO_PIN_SET;
            break;
        case GIMBAL_STATE_GRAVITY_CALIB:
            blue = GPIO_PIN_SET;
            green = GPIO_PIN_SET;
            red = GPIO_PIN_SET;
            break;
        case GIMBAL_STATE_ANGLE_TEST:
            green = GPIO_PIN_SET;
            break;
        case GIMBAL_STATE_TORQUE_TEST:
            red = GPIO_PIN_SET;
            green = GPIO_PIN_SET;
            break;
        default:
            break;
    }

    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, blue);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, green);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, red);
}

static void apply_output_safety(GimbalModeOutput_t *out)
{
    if (gimbal_state == GIMBAL_STATE_IDLE ||
        gimbal_state == GIMBAL_STATE_RELAX ||
        emergency_stop) {
        out->pitch_current = 0.0f;
        out->yaw_current = 0.0f;
        return;
    }

    out->pitch_current = limit_float(out->pitch_current, GIMBAL_MAX_OUTPUT_CURRENT);
    out->yaw_current = limit_float(out->yaw_current, GIMBAL_MAX_OUTPUT_CURRENT);

    float pitch_ctrl_angle = pitch_control_angle(pitch_motor.angle);

    if (pitch_ctrl_angle <= PITCH_SOFT_LIMIT_MIN_DEG &&
        out->pitch_current * PITCH_CURRENT_POS_INCREASES_ANGLE < 0.0f) {
        out->pitch_current = 0.0f;
    }
    if (pitch_ctrl_angle >= PITCH_SOFT_LIMIT_MAX_DEG &&
        out->pitch_current * PITCH_CURRENT_POS_INCREASES_ANGLE > 0.0f) {
        out->pitch_current = 0.0f;
    }
}

void Gimbal_Task(void *argument)
{
    (void)argument;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);

    for (;;)
    {
        GimbalModeOutput_t out = {0.0f, 0.0f};

        update_state_led(gimbal_state);

        GimbalModeScheduler_Update(&out);

        apply_pitch_gravity_feedforward(&out);
        apply_output_safety(&out);

        Motor_SetCurrent(PITCH_MOTOR_ID, out.pitch_current);
        Motor_SetCurrent(YAW_MOTOR_ID, out.yaw_current);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
