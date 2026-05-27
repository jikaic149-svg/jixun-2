#include "MODE_ANGLE_LOOP.h"
#include "config.h"
#include "bsp_can.h"

static float pitch_i_out = 0.0f;
static float yaw_i_out = 0.0f;

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

static float pitch_control_angle(float enc_deg)
{
    if (enc_deg > 180.0f) enc_deg -= 360.0f;
    return enc_deg;
}

void MODE_ANGLE_LOOP_PitchReset(void)
{
    pitch_i_out = 0.0f;
}

void MODE_ANGLE_LOOP_YawReset(void)
{
    yaw_i_out = 0.0f;
}

void MODE_ANGLE_LOOP_ResetAll(void)
{
    MODE_ANGLE_LOOP_PitchReset();
    MODE_ANGLE_LOOP_YawReset();
}

float MODE_ANGLE_LOOP_PitchUpdate(float target_enc_deg)
{
    float target_ctrl_deg = pitch_control_angle(target_enc_deg);
    float current_ctrl_deg = pitch_control_angle(pitch_motor.angle);
    float ff = 0.0f;

    target_ctrl_deg = clamp_float(target_ctrl_deg,
                                  PITCH_SOFT_LIMIT_MIN_DEG,
                                  PITCH_SOFT_LIMIT_MAX_DEG);

    float err = target_ctrl_deg - current_ctrl_deg;
    float original_err = err;

    err = limit_float(err, PITCH_ANGLE_LOOP_MAX_ERR_DEG);
    err = apply_deadband(err, PITCH_ANGLE_LOOP_DEADBAND_DEG);

    if (err > PITCH_ANGLE_LOOP_FF_ERR_DEG) {
        ff = PITCH_ANGLE_LOOP_POS_FF_CURRENT;
    } else if (err < -PITCH_ANGLE_LOOP_FF_ERR_DEG) {
        ff = -PITCH_ANGLE_LOOP_NEG_FF_CURRENT;
    }

    if (original_err > PITCH_ANGLE_LOOP_I_SEPARATE_ERR_DEG ||
        original_err < -PITCH_ANGLE_LOOP_I_SEPARATE_ERR_DEG) {
        pitch_i_out = 0.0f;
    } else {
        pitch_i_out += PITCH_ANGLE_LOOP_KI * original_err * ANGLE_LOOP_DT_SEC;
        pitch_i_out = limit_float(pitch_i_out, PITCH_ANGLE_LOOP_MAX_I_CURRENT);
    }

    return limit_float((PITCH_ANGLE_LOOP_KP * err + pitch_i_out + ff -
                        PITCH_ANGLE_LOOP_KD * pitch_motor.speed) *
                        PITCH_ANGLE_LOOP_POLARITY,
                        PITCH_ANGLE_LOOP_MAX_CURRENT);
}

float MODE_ANGLE_LOOP_YawUpdate(float target_enc_deg)
{
    float err = wrap_angle_error(target_enc_deg - yaw_motor.angle);
    float original_err = err;
    float ff = 0.0f;

    err = limit_float(err, YAW_ANGLE_LOOP_MAX_ERR_DEG);
    err = apply_deadband(err, YAW_ANGLE_LOOP_DEADBAND_DEG);

    if (err > YAW_ANGLE_LOOP_FF_ERR_DEG) {
        ff = YAW_ANGLE_LOOP_POS_FF_CURRENT;
    } else if (err < -YAW_ANGLE_LOOP_FF_ERR_DEG) {
        ff = -YAW_ANGLE_LOOP_NEG_FF_CURRENT;
    }

    if (original_err > YAW_ANGLE_LOOP_I_SEPARATE_ERR_DEG ||
        original_err < -YAW_ANGLE_LOOP_I_SEPARATE_ERR_DEG) {
        yaw_i_out = 0.0f;
    } else {
        yaw_i_out += YAW_ANGLE_LOOP_KI * original_err * ANGLE_LOOP_DT_SEC;
        yaw_i_out = limit_float(yaw_i_out, YAW_ANGLE_LOOP_MAX_I_CURRENT);
    }

    return limit_float((YAW_ANGLE_LOOP_KP * err + yaw_i_out + ff -
                        YAW_ANGLE_LOOP_KD * yaw_motor.speed) *
                        YAW_ANGLE_LOOP_POLARITY,
                        YAW_ANGLE_LOOP_MAX_CURRENT);
}
