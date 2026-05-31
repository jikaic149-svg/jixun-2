#include "MODE_GIMBAL_STAB.h"
#include "config.h"
#include "bsp_can.h"
#include "ins_task.h"
#include "gimbal_task.h"
#include "MODE_ANGLE_LOOP.h"

static float target_yaw = 0.0f;
static float target_pitch = 0.0f;
static float target_enc_yaw = 0.0f;
static float pitch_i_out = 0.0f;
static float filtered_pitch_speed = 0.0f;
static uint8_t first_enter = 1U;

static float wrap_angle_error(float err)
{
    if (err > 180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;
    return err;
}

static float apply_deadband(float err, float deadband)
{
    if (err > deadband) {
        return err - deadband;
    }
    if (err < -deadband) {
        return err + deadband;
    }
    return 0.0f;
}

static float limit_float(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float wrap_0_360(float angle)
{
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

static void refresh_target_enc_yaw(void)
{
    float yaw_delta = wrap_angle_error(target_yaw - global_gimbal_yaw);
    target_enc_yaw = wrap_0_360(yaw_motor.angle +
                                yaw_delta * YAW_ENC_FROM_IMU_POLARITY);
}

static void reset_target_to_current(void)
{
    target_pitch = clamp_float(global_gimbal_pitch,
                               PITCH_SOFT_LIMIT_MIN_DEG,
                               PITCH_SOFT_LIMIT_MAX_DEG);
    target_yaw = wrap_0_360(global_gimbal_yaw);
    target_enc_yaw = yaw_motor.angle;
    pitch_i_out = 0.0f;
    first_enter = 0U;
    gimbal_stab_target_reset = 0U;
    MODE_ANGLE_LOOP_ResetAll();
}

void MODE_GIMBAL_STAB_ResetTarget(void)
{
    first_enter = 1U;
    pitch_i_out = 0.0f;
    MODE_ANGLE_LOOP_ResetAll();
}

void MODE_GIMBAL_STAB_SetTarget(float pitch_deg, float yaw_deg)
{
    target_pitch = clamp_float(pitch_deg,
                               PITCH_SOFT_LIMIT_MIN_DEG,
                               PITCH_SOFT_LIMIT_MAX_DEG);
    target_yaw = wrap_0_360(yaw_deg);
    refresh_target_enc_yaw();
    first_enter = 0U;
    gimbal_stab_target_reset = 0U;
}

float MODE_GIMBAL_STAB_GetTargetPitch(void)
{
    return target_pitch;
}

float MODE_GIMBAL_STAB_GetTargetYaw(void)
{
    return target_yaw;
}

void MODE_GIMBAL_STAB_Update(GimbalModeOutput_t *out)
{
    filtered_pitch_speed = 0.3f * filtered_pitch_speed + 0.7f * global_imu_pitch_speed;

    if (first_enter || gimbal_stab_target_reset) {
        reset_target_to_current();
    }

    if (flag_ext_yaw) {
        target_yaw = wrap_0_360(ext_target_yaw);
        MODE_ANGLE_LOOP_YawReset();
        flag_ext_yaw = 0U;
    }

    refresh_target_enc_yaw();

    float err_pitch = wrap_angle_error(target_pitch - global_gimbal_pitch);
    float original_err_pitch = err_pitch;

    err_pitch = limit_float(err_pitch, GIMBAL_STAB_PITCH_MAX_ERR_DEG);
    err_pitch = apply_deadband(err_pitch, GIMBAL_STAB_PITCH_DEADBAND_DEG);

#if GIMBAL_STAB_PITCH_I_ENABLE
    if (original_err_pitch > GIMBAL_STAB_PITCH_I_SEPARATE_ERR_DEG ||
        original_err_pitch < -GIMBAL_STAB_PITCH_I_SEPARATE_ERR_DEG ||
        filtered_pitch_speed > GIMBAL_STAB_PITCH_I_MAX_SPEED_DPS ||
        filtered_pitch_speed < -GIMBAL_STAB_PITCH_I_MAX_SPEED_DPS ||
        pitch_motor.angle <= PITCH_SOFT_LIMIT_MIN_DEG ||
        pitch_motor.angle >= PITCH_SOFT_LIMIT_MAX_DEG) {
        pitch_i_out = 0.0f;
    } else {
        pitch_i_out += GIMBAL_PID_PITCH_KI * original_err_pitch * ANGLE_LOOP_DT_SEC;
        pitch_i_out = limit_float(pitch_i_out, GIMBAL_MAX_I_PITCH);
    }
#else
    (void)original_err_pitch;
    pitch_i_out = 0.0f;
#endif

#if GIMBAL_STAB_PITCH_ENABLE
    {
        float pitch_stab_current;
#if GIMBAL_STAB_PITCH_ATTITUDE_P_ENABLE
        pitch_stab_current = (GIMBAL_PID_PITCH_KP * err_pitch + pitch_i_out -
                              GIMBAL_PID_PITCH_KD * filtered_pitch_speed) *
                             PITCH_POLARITY;
#else
        (void)err_pitch;
        (void)pitch_i_out;
        pitch_stab_current = (-GIMBAL_PID_PITCH_KD * filtered_pitch_speed) *
                             PITCH_POLARITY;
#endif
        out->pitch_current = limit_float(pitch_stab_current,
                                         GIMBAL_STAB_PITCH_MAX_CURRENT);
    }
#else
    pitch_i_out = 0.0f;
    out->pitch_current = 0.0f;
#endif

#if GIMBAL_STAB_YAW_ENABLE
    {
        out->yaw_current = MODE_ANGLE_LOOP_YawUpdate(target_enc_yaw);
    }
#else
    MODE_ANGLE_LOOP_YawReset();
    out->yaw_current = 0.0f;
#endif
}
