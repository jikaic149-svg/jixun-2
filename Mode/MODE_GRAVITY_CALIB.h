#ifndef MODE_GRAVITY_CALIB_H
#define MODE_GRAVITY_CALIB_H

#include "gimbal_mode.h"

void MODE_GRAVITY_CALIB_Reset(void);
void MODE_GRAVITY_CALIB_Update(GimbalModeOutput_t *out);
float MODE_GRAVITY_CALIB_GetPitchGravityFeedforward(float enc_deg);
float MODE_GRAVITY_CALIB_GetTarget(void);
float MODE_GRAVITY_CALIB_GetError(void);
float MODE_GRAVITY_CALIB_GetSampleEnc(void);
float MODE_GRAVITY_CALIB_GetSampleCurrent(void);
unsigned int MODE_GRAVITY_CALIB_GetPoint(void);
unsigned int MODE_GRAVITY_CALIB_GetDone(void);

#endif /* MODE_GRAVITY_CALIB_H */
