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
static uint8_t vision_sample_circle = 0U;
static uint8_t vision_active_flag = 0U;
static float vision_scan_offset = 0.0f;
static int8_t vision_scan_dir = 1;
static uint8_t vision_scan_ready = 0U;
static uint32_t vision_last_search_ms = 0U;
static uint8_t vision_search_requested = 0U;
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

static uint8_t integral_can_charge(float err, float integral, float kp, float max_step)
{
    float pid = err * kp + integral;
    if (pid > max_step && err > 0.0f && integral >= 0.0f) return 0U;
    if (pid < -max_step && err < 0.0f && integral <= 0.0f) return 0U;
    return 1U;
}

static float wrap_0_360(float angle)
{
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

static void update_yaw_offset(void)
{
    vision_yaw_offset = wrap_angle_error(target_yaw - vision_base_yaw);
}

static float clamp_yaw_to_search_limit(float yaw)
{
    float offset = wrap_angle_error(yaw - vision_base_yaw);
    offset = clamp_float(offset,
                         -VISION_LOST_SCAN_YAW_LIMIT_DEG,
                         VISION_LOST_SCAN_YAW_LIMIT_DEG);
    return wrap_0_360(vision_base_yaw + offset);
}

static void reset_search_scan(void)
{
    vision_scan_offset = 0.0f;
    vision_scan_dir = 1;
    vision_scan_ready = 0U;
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
    vision_sample_circle = 0U;
    vision_active_flag = 0U;
    vision_last_search_ms = 0U;
    vision_search_requested = 0U;
    reset_search_scan();
    reset_vision_filter();
    first_enter = 0U;
    gimbal_stab_target_reset = 0U;
    MODE_GIMBAL_STAB_SetTarget(target_pitch, target_yaw);
}

static uint8_t should_run_search_scan(void)
{
#if VISION_LOST_SCAN_ENABLE
    uint32_t quiet_ms = vision_loop_ms;

    if (vision_active_flag || !vision_search_requested || vision_last_search_ms == 0U) {
        return 0U;
    }
    if ((vision_loop_ms - vision_last_search_ms) > VISION_INPUT_TIMEOUT_MS) {
        return 0U;
    }
    if (vision_last_update_ms > 0U) {
        quiet_ms = vision_loop_ms - vision_last_update_ms;
    }
    return quiet_ms >= VISION_LOST_SCAN_DELAY_MS;
#else
    return 0U;
#endif
}

static void update_search_scan(void)
{
    float step;

    if (!vision_scan_ready) {
        vision_scan_offset = wrap_angle_error(target_yaw - vision_base_yaw);
        vision_scan_offset = clamp_float(vision_scan_offset,
                                         -VISION_LOST_SCAN_YAW_LIMIT_DEG,
                                         VISION_LOST_SCAN_YAW_LIMIT_DEG);
        if (vision_scan_offset >= VISION_LOST_SCAN_YAW_LIMIT_DEG) {
            vision_scan_dir = -1;
        } else if (vision_scan_offset <= -VISION_LOST_SCAN_YAW_LIMIT_DEG) {
            vision_scan_dir = 1;
        }
        vision_scan_ready = 1U;
    }

    step = VISION_LOST_SCAN_SPEED_DPS * ((float)VISION_LOOP_PERIOD_MS * 0.001f);
    vision_scan_offset += (float)vision_scan_dir * step;
    if (vision_scan_offset >= VISION_LOST_SCAN_YAW_LIMIT_DEG) {
        vision_scan_offset = VISION_LOST_SCAN_YAW_LIMIT_DEG;
        vision_scan_dir = -1;
    } else if (vision_scan_offset <= -VISION_LOST_SCAN_YAW_LIMIT_DEG) {
        vision_scan_offset = -VISION_LOST_SCAN_YAW_LIMIT_DEG;
        vision_scan_dir = 1;
    }

    target_yaw = wrap_0_360(vision_base_yaw + vision_scan_offset);
    update_yaw_offset();
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
    vision_sample_circle = 0U;
    vision_active_flag = 0U;
    vision_last_search_ms = 0U;
    vision_search_requested = 0U;
    reset_search_scan();
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
        vision_search_requested = 0U;
        reset_search_scan();
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
        float yaw_kp = VISION_YAW_PID_KP_DEG_PER_PIXEL;
        float yaw_ki = VISION_YAW_PID_KI_DEG_PER_PIXEL;
        float yaw_kd = VISION_YAW_PID_KD_DEG_PER_PIXEL;
        float yaw_i_max = VISION_YAW_PID_I_MAX_DEG;
        float yaw_max_step = VISION_YAW_TRACK_MAX_STEP_DEG;
        float pitch_kp = VISION_PITCH_PID_KP_DEG_PER_PIXEL;
        float pitch_ki = VISION_PITCH_PID_KI_DEG_PER_PIXEL;
        float pitch_kd = VISION_PITCH_PID_KD_DEG_PER_PIXEL;
        float pitch_i_max = VISION_PITCH_PID_I_MAX_DEG;
        float pitch_max_step = VISION_PITCH_TRACK_MAX_STEP_DEG;
        uint8_t yaw_integral_charge = 0U;
        uint8_t pitch_integral_charge = 0U;

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
                if (vision_sample_circle != vision_raw_circle) {
                    reset_vision_filter();
                }
                vision_sample_circle = vision_raw_circle;
                vision_sample_valid = 1U;
                vision_last_search_ms = 0U;
                vision_search_requested = 0U;
                reset_search_scan();
            } else {
                vision_sample_dx = 0;
                vision_sample_dy = 0;
                vision_sample_valid = 0U;
                vision_sample_circle = 0U;
                if (vision_raw_search) {
                    vision_search_requested = 1U;
                    vision_last_search_ms = vision_loop_ms;
                } else {
                    vision_search_requested = 0U;
                    vision_last_search_ms = 0U;
                    reset_search_scan();
                }
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
                if (vision_sample_circle) {
                    yaw_kp = VISION_CIRCLE_YAW_PID_KP_DEG_PER_PIXEL;
                    yaw_ki = VISION_CIRCLE_YAW_PID_KI_DEG_PER_PIXEL;
                    yaw_kd = VISION_CIRCLE_YAW_PID_KD_DEG_PER_PIXEL;
                    yaw_i_max = VISION_CIRCLE_YAW_PID_I_MAX_DEG;
                    yaw_max_step = VISION_CIRCLE_YAW_TRACK_MAX_STEP_DEG;
                    pitch_kp = VISION_CIRCLE_PITCH_PID_KP_DEG_PER_PIXEL;
                    pitch_ki = VISION_CIRCLE_PITCH_PID_KI_DEG_PER_PIXEL;
                    pitch_kd = VISION_CIRCLE_PITCH_PID_KD_DEG_PER_PIXEL;
                    pitch_i_max = VISION_CIRCLE_PITCH_PID_I_MAX_DEG;
                    pitch_max_step = VISION_CIRCLE_PITCH_TRACK_MAX_STEP_DEG;
                }
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
                yaw_integral_charge = integral_can_charge(vision_err_x,
                                                           vision_yaw_i,
                                                           yaw_kp,
                                                           yaw_max_step);
                pitch_integral_charge = integral_can_charge(vision_err_y,
                                                             vision_pitch_i,
                                                             pitch_kp,
                                                             pitch_max_step);

                if (vision_i_err_x == 0.0f || !yaw_integral_charge) {
                    vision_yaw_i *= VISION_INTEGRAL_CENTER_DECAY;
                } else {
                    if ((vision_yaw_i > 0.0f && vision_i_err_x < 0.0f) ||
                        (vision_yaw_i < 0.0f && vision_i_err_x > 0.0f)) {
                        vision_yaw_i *= VISION_INTEGRAL_REVERSAL_DECAY;
                    }
                    vision_yaw_i = clamp_float(vision_yaw_i +
                                               vision_i_err_x * yaw_ki,
                                               -yaw_i_max,
                                               yaw_i_max);
                }
                if (vision_i_err_y == 0.0f || !pitch_integral_charge) {
                    vision_pitch_i *= VISION_INTEGRAL_CENTER_DECAY;
                } else {
                    if ((vision_pitch_i > 0.0f && vision_i_err_y < 0.0f) ||
                        (vision_pitch_i < 0.0f && vision_i_err_y > 0.0f)) {
                        vision_pitch_i *= VISION_INTEGRAL_REVERSAL_DECAY;
                    }
                    vision_pitch_i = clamp_float(vision_pitch_i +
                                                 vision_i_err_y * pitch_ki,
                                                 -pitch_i_max,
                                                 pitch_i_max);
                }

                yaw_pid = vision_err_x * yaw_kp + vision_yaw_i;
                pitch_pid = vision_err_y * pitch_kp + vision_pitch_i;

                if (vision_has_last_err) {
                    yaw_pid += (vision_err_x - vision_last_err_x) *
                               yaw_kd;
                    pitch_pid += (vision_err_y - vision_last_err_y) *
                                 pitch_kd;
                }

                target_yaw = wrap_0_360(target_yaw +
                                        clamp_float(yaw_pid * VISION_YAW_POLARITY,
                                                    -yaw_max_step,
                                                    yaw_max_step));
                target_yaw = clamp_yaw_to_search_limit(target_yaw);
                target_pitch = clamp_float(target_pitch +
                                           clamp_float(pitch_pid * VISION_PITCH_POLARITY,
                                                       -pitch_max_step,
                                                       pitch_max_step),
                                           PITCH_SOFT_LIMIT_MIN_DEG,
                                           PITCH_SOFT_LIMIT_MAX_DEG);

                update_yaw_offset();
                vision_pitch_offset = target_pitch - vision_base_pitch;
                vision_scan_ready = 0U;
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
                if (should_run_search_scan()) {
                    update_search_scan();
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
