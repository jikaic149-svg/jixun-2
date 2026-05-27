#ifndef MODE_ANGLE_TEST_H
#define MODE_ANGLE_TEST_H

#include "gimbal_mode.h"

void MODE_ANGLE_TEST_Reset(void);
void MODE_ANGLE_TEST_Update(GimbalModeOutput_t *out);
float MODE_ANGLE_TEST_GetPitchTarget(void);
float MODE_ANGLE_TEST_GetPitchError(void);
float MODE_ANGLE_TEST_GetYawTarget(void);
float MODE_ANGLE_TEST_GetYawError(void);
unsigned int MODE_ANGLE_TEST_GetPhase(void);

#endif /* MODE_ANGLE_TEST_H */
