#include "MODE_TORQUE_TEST.h"
#include "config.h"
#include "bsp_can.h"

static unsigned int phase = 0U;
static unsigned int phase_tick = 0U;
static unsigned int done_mask = 0U;
static unsigned char first_enter = 1U;
static float start_pitch = 0.0f;
static float start_yaw = 0.0f;
static float ramp_current = 0.0f;
static float move_deg = 0.0f;
static float pitch_pos_current = 0.0f;
static float pitch_neg_current = 0.0f;
static float yaw_pos_current = 0.0f;
static float yaw_neg_current = 0.0f;

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

static void begin_phase(unsigned int next_phase)
{
    phase = next_phase;
    phase_tick = 0U;
    ramp_current = 0.0f;
    move_deg = 0.0f;
    start_pitch = pitch_control_angle(pitch_motor.angle);
    start_yaw = yaw_motor.angle;
}

void MODE_TORQUE_TEST_Reset(void)
{
    first_enter = 1U;
    phase = 0U;
    phase_tick = 0U;
    done_mask = 0U;
    ramp_current = 0.0f;
    move_deg = 0.0f;
    pitch_pos_current = 0.0f;
    pitch_neg_current = 0.0f;
    yaw_pos_current = 0.0f;
    yaw_neg_current = 0.0f;
}

void MODE_TORQUE_TEST_Update(GimbalModeOutput_t *out)
{
    out->pitch_current = 0.0f;
    out->yaw_current = 0.0f;

    if (first_enter) {
        begin_phase(0U);
        first_enter = 0U;
    }

    phase_tick++;
    if (phase_tick < TORQUE_TEST_SETTLE_MS) {
        return;
    }

    if (phase >= 4U) {
        return;
    }

    ramp_current += TORQUE_TEST_RAMP_STEP_CURRENT;
    if (ramp_current > TORQUE_TEST_MAX_CURRENT) {
        ramp_current = TORQUE_TEST_MAX_CURRENT;
        done_mask |= (1U << phase);
        begin_phase(phase + 1U);
        return;
    }

    switch (phase) {
        case 0U:
            out->pitch_current = ramp_current;
            move_deg = pitch_control_angle(pitch_motor.angle) - start_pitch;
            if (move_deg >= TORQUE_TEST_MOVE_DEG) {
                pitch_pos_current = ramp_current;
                done_mask |= 0x01U;
                begin_phase(1U);
            }
            break;
        case 1U:
            out->pitch_current = -ramp_current;
            move_deg = start_pitch - pitch_control_angle(pitch_motor.angle);
            if (move_deg >= TORQUE_TEST_MOVE_DEG) {
                pitch_neg_current = ramp_current;
                done_mask |= 0x02U;
                begin_phase(2U);
            }
            break;
        case 2U:
            out->yaw_current = ramp_current;
            move_deg = wrap_angle_error(yaw_motor.angle - start_yaw);
            if (move_deg >= TORQUE_TEST_MOVE_DEG) {
                yaw_pos_current = ramp_current;
                done_mask |= 0x04U;
                begin_phase(3U);
            }
            break;
        case 3U:
            out->yaw_current = -ramp_current;
            move_deg = wrap_angle_error(start_yaw - yaw_motor.angle);
            if (move_deg >= TORQUE_TEST_MOVE_DEG) {
                yaw_neg_current = ramp_current;
                done_mask |= 0x08U;
                begin_phase(4U);
            }
            break;
        default:
            break;
    }
}

float MODE_TORQUE_TEST_GetPitchPos(void)
{
    return pitch_pos_current;
}

float MODE_TORQUE_TEST_GetPitchNeg(void)
{
    return pitch_neg_current;
}

float MODE_TORQUE_TEST_GetYawPos(void)
{
    return yaw_pos_current;
}

float MODE_TORQUE_TEST_GetYawNeg(void)
{
    return yaw_neg_current;
}

float MODE_TORQUE_TEST_GetRampCurrent(void)
{
    return ramp_current;
}

float MODE_TORQUE_TEST_GetMoveDeg(void)
{
    return move_deg;
}

unsigned int MODE_TORQUE_TEST_GetPhase(void)
{
    return phase;
}

unsigned int MODE_TORQUE_TEST_GetDoneMask(void)
{
    return done_mask;
}
