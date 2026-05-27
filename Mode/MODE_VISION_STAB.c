#include "MODE_VISION_STAB.h"
#include "MODE_GIMBAL_STAB.h"
#include "config.h"
#include "ins_task.h"
#include "gimbal_task.h"

static float target_yaw = 0.0f;
static float target_pitch = 0.0f;
static float vision_base_pitch = 0.0f;
static float vision_base_yaw = 0.0f;
static float vision_pitch_offset = 0.0f;
static float vision_yaw_offset = 0.0f;
static float vision_yaw_i = 0.0f;
static float vision_pitch_i = 0.0f;
static float vision_last_err_x = 0.0f;
static float vision_last_err_y = 0.0f;
static uint8_t vision_has_last_err = 0U;
static uint32_t vision_last_frame_seq = 0U;
static uint32_t vision_last_update_ms = 0U;
static uint32_t vision_loop_ms = 0U;
static uint32_t vision_control_div_ms = 0U;
static int16_t vision_sample_dx = 0;
static int16_t vision_sample_dy = 0;
static uint8_t vision_sample_valid = 0U;
static uint8_t vision_active_flag = 0U;
static uint8_t first_enter = 1U;

static float wrap_angle_error(float err)
{
    if (err > 180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;
    return err;
}

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float apply_pixel_deadband(float value, float deadband)
{
    if (value > deadband) return value - deadband;
    if (value < -deadband) return value + deadband;
    return 0.0f;
}

static float wrap_0_360(float angle)
{
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

static void reset_vision_filter(void)
{
    vision_yaw_i = 0.0f;
    vision_pitch_i = 0.0f;
    vision_last_err_x = 0.0f;
    vision_last_err_y = 0.0f;
    vision_has_last_err = 0U;
}

static void reset_to_current(void)
{
    target_pitch = clamp_float(global_gimbal_pitch,
                               PITCH_SOFT_LIMIT_MIN_DEG,
                               PITCH_SOFT_LIMIT_MAX_DEG);
    target_yaw = wrap_0_360(global_gimbal_yaw);
    vision_base_pitch = target_pitch;
    vision_base_yaw = target_yaw;
    vision_pitch_offset = 0.0f;
    vision_yaw_offset = 0.0f;
    vision_last_frame_seq = vision_raw_frame_seq;
    vision_last_update_ms = 0U;
    vision_loop_ms = 0U;
    vision_control_div_ms = 0U;
    vision_sample_dx = 0;
    vision_sample_dy = 0;
    vision_sample_valid = 0U;
    vision_active_flag = 0U;
    reset_vision_filter();
    first_enter = 0U;
    gimbal_stab_target_reset = 0U;
    MODE_GIMBAL_STAB_SetTarget(target_pitch, target_yaw);
}

static void freeze_target_to_current(void)
{
    target_pitch = clamp_float(global_gimbal_pitch,
                               PITCH_SOFT_LIMIT_MIN_DEG,
                               PITCH_SOFT_LIMIT_MAX_DEG);
    target_yaw = wrap_0_360(global_gimbal_yaw);
    vision_base_pitch = target_pitch;
    vision_base_yaw = target_yaw;
    vision_pitch_offset = 0.0f;
    vision_yaw_offset = 0.0f;
    reset_vision_filter();
}

void MODE_VISION_STAB_ResetTarget(void)
{
    first_enter = 1U;
    vision_pitch_offset = 0.0f;
    vision_yaw_offset = 0.0f;
    vision_last_frame_seq = 0U;
    vision_last_update_ms = 0U;
    vision_loop_ms = 0U;
    vision_control_div_ms = 0U;
    vision_sample_dx = 0;
    vision_sample_dy = 0;
    vision_sample_valid = 0U;
    vision_active_flag = 0U;
    reset_vision_filter();
    MODE_GIMBAL_STAB_ResetTarget();
}

uint8_t MODE_VISION_STAB_GetVisionActive(void)
{
    return vision_active_flag;
}

float MODE_VISION_STAB_GetVisionPitchOffset(void)
{
    return vision_pitch_offset;
}

float MODE_VISION_STAB_GetVisionYawOffset(void)
{
    return vision_yaw_offset;
}

void MODE_VISION_STAB_Update(GimbalModeOutput_t *out)
{
    if (first_enter || gimbal_stab_target_reset) {
        reset_to_current();
    }

    if (flag_ext_yaw) {
        target_yaw = wrap_0_360(ext_target_yaw);
        vision_base_yaw = target_yaw;
        vision_yaw_offset = 0.0f;
        reset_vision_filter();
        flag_ext_yaw = 0U;
    }

#if VISION_LOOP_ENABLE
    {
        uint8_t vision_active = 0U;
        uint8_t run_vision_step = 0U;
        float vision_err_x = 0.0f;
        float vision_err_y = 0.0f;
        float vision_i_err_x = 0.0f;
        float vision_i_err_y = 0.0f;
        float yaw_pid = 0.0f;
        float pitch_pid = 0.0f;

        vision_loop_ms += 1U;
        vision_control_div_ms += 1U;
        if (vision_control_div_ms >= VISION_LOOP_PERIOD_MS) {
            vision_control_div_ms = 0U;
            run_vision_step = 1U;
        }

        if (vision_raw_frame_seq != vision_last_frame_seq) {
            vision_last_frame_seq = vision_raw_frame_seq;
            if (vision_raw_valid) {
                vision_last_update_ms = vision_loop_ms;
                vision_sample_dx = vision_raw_dx;
                vision_sample_dy = vision_raw_dy;
                vision_sample_valid = 1U;
            } else {
                vision_sample_dx = 0;
                vision_sample_dy = 0;
                vision_sample_valid = 0U;
                reset_vision_filter();
            }
        }

        if (vision_last_update_ms > 0U &&
            (vision_loop_ms - vision_last_update_ms) <= VISION_INPUT_TIMEOUT_MS &&
            vision_sample_valid) {
            vision_active = 1U;
        }
        vision_active_flag = vision_active;

        if (run_vision_step) {
            if (vision_active) {
                vision_err_x = apply_pixel_deadband(
                    clamp_float((float)vision_sample_dx,
                                -(float)VISION_INPUT_MAX_DX,
                                (float)VISION_INPUT_MAX_DX),
                    VISION_INPUT_DEADBAND_X);
                vision_err_y = apply_pixel_deadband(
                    clamp_float((float)vision_sample_dy,
                                -(float)VISION_INPUT_MAX_DY,
                                (float)VISION_INPUT_MAX_DY),
                    VISION_INPUT_DEADBAND_Y);
                vision_i_err_x = apply_pixel_deadband(
                    clamp_float((float)vision_sample_dx,
                                -(float)VISION_INPUT_MAX_DX,
                                (float)VISION_INPUT_MAX_DX),
                    VISION_INTEGRAL_DEADBAND_X);
                vision_i_err_y = apply_pixel_deadband(
                    clamp_float((float)vision_sample_dy,
                                -(float)VISION_INPUT_MAX_DY,
                                (float)VISION_INPUT_MAX_DY),
                    VISION_INTEGRAL_DEADBAND_Y);

                if (vision_i_err_x == 0.0f) {
                    vision_yaw_i *= VISION_INTEGRAL_CENTER_DECAY;
                } else {
                    if ((vision_yaw_i > 0.0f && vision_i_err_x < 0.0f) ||
                        (vision_yaw_i < 0.0f && vision_i_err_x > 0.0f)) {
                        vision_yaw_i *= VISION_INTEGRAL_REVERSAL_DECAY;
                    }
                    vision_yaw_i = clamp_float(vision_yaw_i +
                                               vision_i_err_x * VISION_YAW_PID_KI_DEG_PER_PIXEL,
                                               -VISION_YAW_PID_I_MAX_DEG,
                                               VISION_YAW_PID_I_MAX_DEG);
                }
                if (vision_i_err_y == 0.0f) {
                    vision_pitch_i *= VISION_INTEGRAL_CENTER_DECAY;
                } else {
                    if ((vision_pitch_i > 0.0f && vision_i_err_y < 0.0f) ||
                        (vision_pitch_i < 0.0f && vision_i_err_y > 0.0f)) {
                        vision_pitch_i *= VISION_INTEGRAL_REVERSAL_DECAY;
                    }
                    vision_pitch_i = clamp_float(vision_pitch_i +
                                                 vision_i_err_y * VISION_PITCH_PID_KI_DEG_PER_PIXEL,
                                                 -VISION_PITCH_PID_I_MAX_DEG,
                                                 VISION_PITCH_PID_I_MAX_DEG);
                }

                yaw_pid = vision_err_x * VISION_YAW_PID_KP_DEG_PER_PIXEL + vision_yaw_i;
                pitch_pid = vision_err_y * VISION_PITCH_PID_KP_DEG_PER_PIXEL + vision_pitch_i;

                if (vision_has_last_err) {
                    yaw_pid += (vision_err_x - vision_last_err_x) *
                               VISION_YAW_PID_KD_DEG_PER_PIXEL;
                    pitch_pid += (vision_err_y - vision_last_err_y) *
                                 VISION_PITCH_PID_KD_DEG_PER_PIXEL;
                }

                target_yaw = wrap_0_360(target_yaw +
                                        clamp_float(yaw_pid * VISION_YAW_POLARITY,
                                                    -VISION_YAW_TRACK_MAX_STEP_DEG,
                                                    VISION_YAW_TRACK_MAX_STEP_DEG));
                target_pitch = clamp_float(target_pitch +
                                           clamp_float(pitch_pid * VISION_PITCH_POLARITY,
                                                       -VISION_PITCH_TRACK_MAX_STEP_DEG,
                                                       VISION_PITCH_TRACK_MAX_STEP_DEG),
                                           PITCH_SOFT_LIMIT_MIN_DEG,
                                           PITCH_SOFT_LIMIT_MAX_DEG);

                vision_yaw_offset = wrap_angle_error(target_yaw - vision_base_yaw);
                vision_pitch_offset = target_pitch - vision_base_pitch;
                vision_last_err_x = vision_err_x;
                vision_last_err_y = vision_err_y;
                vision_has_last_err = 1U;
            } else {
                if (vision_sample_valid &&
                    vision_last_update_ms > 0U &&
                    (vision_loop_ms - vision_last_update_ms) > VISION_INPUT_TIMEOUT_MS) {
                    vision_sample_valid = 0U;
                    reset_vision_filter();
                }
            }
        }
    }
#else
    vision_active_flag = 0U;
#endif

    MODE_GIMBAL_STAB_SetTarget(target_pitch, target_yaw);
    MODE_GIMBAL_STAB_Update(out);
}
