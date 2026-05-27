#include "gimbal_mode_scheduler.h"
#include "gimbal_task.h"
#include "config.h"
#include "MODE_STAB_CALIB.h"
#include "MODE_GIMBAL_STAB.h"
#include "MODE_VISION_STAB.h"
#include "MODE_GRAVITY_CALIB.h"
#include "MODE_ANGLE_TEST.h"
#include "MODE_TORQUE_TEST.h"

static GimbalState_e last_state = GIMBAL_STATE_IDLE;
static uint8_t first_update = 1U;

static void reset_control_modes(void)
{
    MODE_GIMBAL_STAB_ResetTarget();
    MODE_VISION_STAB_ResetTarget();
    MODE_GRAVITY_CALIB_Reset();
    MODE_ANGLE_TEST_Reset();
    MODE_TORQUE_TEST_Reset();
}

static void enter_state(GimbalState_e state)
{
    switch (state) {
        case GIMBAL_STATE_IDLE:
        case GIMBAL_STATE_RELAX:
            reset_control_modes();
            break;

        case GIMBAL_STATE_ENC_LOCK:
            MODE_STAB_CALIB_Enter();
            gimbal_state = GIMBAL_STATE_CALIBRATING;
            break;

        case GIMBAL_STATE_IMU_STAB:
            MODE_GIMBAL_STAB_ResetTarget();
            break;

        case GIMBAL_STATE_VISION_STAB:
            MODE_VISION_STAB_ResetTarget();
            break;

        case GIMBAL_STATE_GRAVITY_CALIB:
            MODE_GRAVITY_CALIB_Reset();
            break;

        case GIMBAL_STATE_ANGLE_TEST:
            MODE_ANGLE_TEST_Reset();
            break;

        case GIMBAL_STATE_TORQUE_TEST:
            MODE_TORQUE_TEST_Reset();
            break;

        default:
            gimbal_state = GIMBAL_STATE_RELAX;
            reset_control_modes();
            break;
    }
}

void GimbalModeScheduler_Reset(void)
{
    first_update = 1U;
    last_state = gimbal_state;
    reset_control_modes();
}

void GimbalModeScheduler_Update(GimbalModeOutput_t *out)
{
    if (first_update || gimbal_state != last_state) {
        enter_state(gimbal_state);
        last_state = gimbal_state;
        first_update = 0U;
    }

    switch (gimbal_state) {
        case GIMBAL_STATE_IDLE:
        case GIMBAL_STATE_RELAX:
            out->pitch_current = 0.0f;
            out->yaw_current = 0.0f;
            break;

        case GIMBAL_STATE_ENC_LOCK:
            MODE_STAB_CALIB_Enter();
            gimbal_state = GIMBAL_STATE_CALIBRATING;
            last_state = gimbal_state;
            MODE_STAB_CALIB_Update(out);
            break;

        case GIMBAL_STATE_CALIBRATING:
            MODE_STAB_CALIB_Update(out);
            break;

        case GIMBAL_STATE_IMU_STAB:
            MODE_GIMBAL_STAB_Update(out);
            break;

        case GIMBAL_STATE_VISION_STAB:
            MODE_VISION_STAB_Update(out);
            break;

        case GIMBAL_STATE_GRAVITY_CALIB:
            MODE_GRAVITY_CALIB_Update(out);
            break;

        case GIMBAL_STATE_ANGLE_TEST:
#if ENABLE_ANGLE_TEST_MODE
            MODE_ANGLE_TEST_Update(out);
#else
            gimbal_state = GIMBAL_STATE_RELAX;
            last_state = gimbal_state;
            reset_control_modes();
            out->pitch_current = 0.0f;
            out->yaw_current = 0.0f;
#endif
            break;

        case GIMBAL_STATE_TORQUE_TEST:
#if ENABLE_TORQUE_TEST_MODE
            MODE_TORQUE_TEST_Update(out);
#else
            gimbal_state = GIMBAL_STATE_RELAX;
            last_state = gimbal_state;
            reset_control_modes();
            out->pitch_current = 0.0f;
            out->yaw_current = 0.0f;
#endif
            break;

        default:
            gimbal_state = GIMBAL_STATE_RELAX;
            last_state = gimbal_state;
            reset_control_modes();
            out->pitch_current = 0.0f;
            out->yaw_current = 0.0f;
            break;
    }
}
