#include "MODE_GRAVITY_CALIB.h"
#include "MODE_ANGLE_LOOP.h"
#include "config.h"
#include "bsp_can.h"
#include <stdint.h>
#include <math.h>

typedef enum {
    GRAV_PHASE_SETTLE = 0,
    GRAV_PHASE_SAMPLE,
    GRAV_PHASE_DONE
} GravCalibPhase_e;

static float target_enc_yaw = 0.0f;
static float pitch_target = PITCH_SOFT_LIMIT_MIN_DEG;
static float pitch_err = 0.0f;
static float sample_enc = 0.0f;
static float sample_current = 0.0f;
static float sum_enc = 0.0f;
static float sum_current = 0.0f;
static uint32_t sample_count = 0U;
static uint32_t phase_tick = 0U;
static uint8_t first_enter = 1U;
static uint8_t point_index = 0U;
static uint8_t sample_point = 0U;
static uint8_t done = 0U;
static GravCalibPhase_e phase = GRAV_PHASE_SETTLE;

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

static float point_to_target(uint8_t index)
{
    const float min_deg = PITCH_SOFT_LIMIT_MIN_DEG + PITCH_GRAVITY_CALIB_LIMIT_MARGIN_DEG;
    const float max_deg = PITCH_SOFT_LIMIT_MAX_DEG - PITCH_GRAVITY_CALIB_LIMIT_MARGIN_DEG;
    uint8_t point = index;

    if (point >= PITCH_GRAVITY_CALIB_POINT_COUNT) {
        point = (uint8_t)((PITCH_GRAVITY_CALIB_POINT_COUNT * 2U - 1U) - point);
    }

#if PITCH_GRAVITY_CALIB_POINT_COUNT <= 1U
    (void)point;
    return (min_deg + max_deg) * 0.5f;
#else
    return min_deg + (max_deg - min_deg) *
                     ((float)point / (float)(PITCH_GRAVITY_CALIB_POINT_COUNT - 1U));
#endif
}

static void begin_point(uint8_t index)
{
    point_index = index;
    pitch_target = point_to_target(index);
    pitch_err = 0.0f;
    sum_enc = 0.0f;
    sum_current = 0.0f;
    sample_count = 0U;
    phase_tick = 0U;
    phase = GRAV_PHASE_SETTLE;
    MODE_ANGLE_LOOP_PitchReset();
}

void MODE_GRAVITY_CALIB_Reset(void)
{
    first_enter = 1U;
    done = 0U;
    sample_enc = 0.0f;
    sample_current = 0.0f;
    sample_point = 0U;
    begin_point(0U);
    MODE_ANGLE_LOOP_ResetAll();
}

void MODE_GRAVITY_CALIB_Update(GimbalModeOutput_t *out)
{
    float current_pitch = pitch_control_angle(pitch_motor.angle);

    if (first_enter) {
        target_enc_yaw = yaw_motor.angle;
        begin_point(0U);
        MODE_ANGLE_LOOP_ResetAll();
        first_enter = 0U;
    }

    pitch_err = pitch_target - current_pitch;
    out->yaw_current = MODE_ANGLE_LOOP_YawUpdate(target_enc_yaw);
    out->pitch_current = MODE_ANGLE_LOOP_PitchUpdate(pitch_target);

    if (phase == GRAV_PHASE_DONE) {
        return;
    }

    phase_tick++;

    if (phase == GRAV_PHASE_SETTLE) {
        if (phase_tick >= PITCH_GRAVITY_CALIB_SETTLE_MS) {
            phase = GRAV_PHASE_SAMPLE;
            phase_tick = 0U;
            sum_enc = 0.0f;
            sum_current = 0.0f;
            sample_count = 0U;
            MODE_ANGLE_LOOP_PitchReset();
        }
        return;
    }

    if (phase == GRAV_PHASE_SAMPLE) {
        sum_enc += current_pitch;
        sum_current += pitch_motor.current;
        sample_count++;

        if (phase_tick >= PITCH_GRAVITY_CALIB_SAMPLE_MS) {
            if (sample_count > 0U) {
                sample_enc = sum_enc / (float)sample_count;
                sample_current = sum_current / (float)sample_count;
                sample_point = point_index;
            }

            if ((point_index + 1U) >= (PITCH_GRAVITY_CALIB_POINT_COUNT * 2U)) {
                done = 1U;
                phase = GRAV_PHASE_DONE;
                MODE_ANGLE_LOOP_ResetAll();
            } else {
                begin_point(point_index + 1U);
            }
        }
    }
}

float MODE_GRAVITY_CALIB_GetPitchGravityFeedforward(float enc_deg)
{
#if PITCH_GRAVITY_FF_ENABLE
    float ctrl_deg = pitch_control_angle(enc_deg);
    float rad = ctrl_deg * 0.01745329252f;
    float current = PITCH_GRAVITY_FF_SIN_K * sinf(rad) +
                    PITCH_GRAVITY_FF_COS_K * cosf(rad) +
                    PITCH_GRAVITY_FF_OFFSET;

    return limit_float(current, PITCH_GRAVITY_FF_MAX_CURRENT);
#else
    (void)enc_deg;
    return 0.0f;
#endif
}

float MODE_GRAVITY_CALIB_GetTarget(void)
{
    return pitch_target;
}

float MODE_GRAVITY_CALIB_GetError(void)
{
    return pitch_err;
}

float MODE_GRAVITY_CALIB_GetSampleEnc(void)
{
    return sample_enc;
}

float MODE_GRAVITY_CALIB_GetSampleCurrent(void)
{
    return sample_current;
}

unsigned int MODE_GRAVITY_CALIB_GetPoint(void)
{
    return sample_point;
}

unsigned int MODE_GRAVITY_CALIB_GetDone(void)
{
    return done;
}
