#include "MODE_ANGLE_TEST.h"
#include "MODE_ANGLE_LOOP.h"
#include "config.h"
#include "bsp_can.h"

static float base_pitch = 0.0f;
static float base_yaw = 0.0f;
static float pitch_target = 0.0f;
static float yaw_target = 0.0f;
static float pitch_err = 0.0f;
static float yaw_err = 0.0f;
static unsigned int phase = 0U;
static unsigned int phase_tick = 0U;
static unsigned char first_enter = 1U;

static float pitch_control_angle(float enc_deg)
{
    if (enc_deg > 180.0f) enc_deg -= 360.0f;
    return enc_deg;
}

static float wrap_angle_error(float err)
{
    if (err > 180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;
    return err;
}

static float wrap_0_360(float angle)
{
    while (angle >= 360.0f) angle -= 360.0f;
    while (angle < 0.0f) angle += 360.0f;
    return angle;
}

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void set_phase_target(void)
{
    pitch_target = base_pitch;
    yaw_target = base_yaw;

    switch (phase) {
        case 0U:
            break;
        case 1U:
            pitch_target = base_pitch + ANGLE_TEST_STEP_DEG;
            break;
        case 2U:
            break;
        case 3U:
            pitch_target = base_pitch - ANGLE_TEST_STEP_DEG;
            break;
        case 4U:
            break;
        case 5U:
            yaw_target = wrap_0_360(base_yaw + ANGLE_TEST_STEP_DEG);
            break;
        case 6U:
            break;
        case 7U:
            yaw_target = wrap_0_360(base_yaw - ANGLE_TEST_STEP_DEG);
            break;
        default:
            phase = 0U;
            break;
    }

    pitch_target = clamp_float(pitch_target,
                               PITCH_SOFT_LIMIT_MIN_DEG + ANGLE_TEST_LIMIT_MARGIN_DEG,
                               PITCH_SOFT_LIMIT_MAX_DEG - ANGLE_TEST_LIMIT_MARGIN_DEG);
}

void MODE_ANGLE_TEST_Reset(void)
{
    first_enter = 1U;
    phase = 0U;
    phase_tick = 0U;
    pitch_err = 0.0f;
    yaw_err = 0.0f;
    MODE_ANGLE_LOOP_ResetAll();
}

void MODE_ANGLE_TEST_Update(GimbalModeOutput_t *out)
{
    if (first_enter) {
        base_pitch = pitch_control_angle(pitch_motor.angle);
        base_pitch = clamp_float(base_pitch,
                                 PITCH_SOFT_LIMIT_MIN_DEG + ANGLE_TEST_LIMIT_MARGIN_DEG,
                                 PITCH_SOFT_LIMIT_MAX_DEG - ANGLE_TEST_LIMIT_MARGIN_DEG);
        base_yaw = yaw_motor.angle;
        phase = 0U;
        phase_tick = 0U;
        set_phase_target();
        MODE_ANGLE_LOOP_ResetAll();
        first_enter = 0U;
    }

    phase_tick++;
    if (phase_tick >= ANGLE_TEST_PHASE_MS) {
        phase_tick = 0U;
        phase++;
        if (phase > 7U) phase = 0U;
        set_phase_target();
        MODE_ANGLE_LOOP_ResetAll();
    }

    pitch_err = pitch_target - pitch_control_angle(pitch_motor.angle);
    yaw_err = wrap_angle_error(yaw_target - yaw_motor.angle);

    out->pitch_current = MODE_ANGLE_LOOP_PitchUpdate(pitch_target);
    out->yaw_current = MODE_ANGLE_LOOP_YawUpdate(yaw_target);
}

float MODE_ANGLE_TEST_GetPitchTarget(void)
{
    return pitch_target;
}

float MODE_ANGLE_TEST_GetPitchError(void)
{
    return pitch_err;
}

float MODE_ANGLE_TEST_GetYawTarget(void)
{
    return yaw_target;
}

float MODE_ANGLE_TEST_GetYawError(void)
{
    return yaw_err;
}

unsigned int MODE_ANGLE_TEST_GetPhase(void)
{
    return phase;
}
