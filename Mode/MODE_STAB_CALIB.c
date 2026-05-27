#include "MODE_STAB_CALIB.h"
#include "MODE_ANGLE_LOOP.h"
#include "config.h"
#include "bsp_can.h"

static float target_enc_pitch = 0.0f;
static float target_enc_yaw = 0.0f;

void MODE_STAB_CALIB_Enter(void)
{
    target_enc_pitch = pitch_motor.angle;
    target_enc_yaw = yaw_motor.angle;
    MODE_ANGLE_LOOP_ResetAll();
}

void MODE_STAB_CALIB_Update(GimbalModeOutput_t *out)
{
    out->pitch_current = MODE_ANGLE_LOOP_PitchUpdate(target_enc_pitch);
    out->yaw_current = MODE_ANGLE_LOOP_YawUpdate(target_enc_yaw);
}
