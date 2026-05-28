#include "MODE_STAB_CALIB.h"
#include "MODE_ANGLE_LOOP.h"
#include "config.h"
#include "bsp_can.h"
#include "FreeRTOS.h"
#include "task.h"

#define MOTOR_FEEDBACK_TIMEOUT_MS 100U

static float target_enc_pitch = 0.0f;
static float target_enc_yaw = 0.0f;
static uint8_t target_ready = 0U;

void MODE_STAB_CALIB_Enter(void)
{
    uint32_t now_tick = (uint32_t)xTaskGetTickCount();
    target_ready =
        Motor_FeedbackReady(&pitch_motor, now_tick, MOTOR_FEEDBACK_TIMEOUT_MS) &&
        Motor_FeedbackReady(&yaw_motor, now_tick, MOTOR_FEEDBACK_TIMEOUT_MS);
    if (target_ready) {
        target_enc_pitch = pitch_motor.angle;
        target_enc_yaw = yaw_motor.angle;
    }
    MODE_ANGLE_LOOP_ResetAll();
}

void MODE_STAB_CALIB_Update(GimbalModeOutput_t *out)
{
    uint32_t now_tick = (uint32_t)xTaskGetTickCount();
    if (!target_ready ||
        !Motor_FeedbackReady(&pitch_motor, now_tick, MOTOR_FEEDBACK_TIMEOUT_MS) ||
        !Motor_FeedbackReady(&yaw_motor, now_tick, MOTOR_FEEDBACK_TIMEOUT_MS)) {
        out->pitch_current = 0.0f;
        out->yaw_current = 0.0f;
        target_ready =
            Motor_FeedbackReady(&pitch_motor, now_tick, MOTOR_FEEDBACK_TIMEOUT_MS) &&
            Motor_FeedbackReady(&yaw_motor, now_tick, MOTOR_FEEDBACK_TIMEOUT_MS);
        if (target_ready) {
            target_enc_pitch = pitch_motor.angle;
            target_enc_yaw = yaw_motor.angle;
            MODE_ANGLE_LOOP_ResetAll();
        }
        return;
    }

    out->pitch_current = MODE_ANGLE_LOOP_PitchUpdate(target_enc_pitch);
    out->yaw_current = MODE_ANGLE_LOOP_YawUpdate(target_enc_yaw);
}
