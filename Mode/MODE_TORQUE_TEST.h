#ifndef MODE_TORQUE_TEST_H
#define MODE_TORQUE_TEST_H

#include "gimbal_mode.h"

void MODE_TORQUE_TEST_Reset(void);
void MODE_TORQUE_TEST_Update(GimbalModeOutput_t *out);
float MODE_TORQUE_TEST_GetPitchPos(void);
float MODE_TORQUE_TEST_GetPitchNeg(void);
float MODE_TORQUE_TEST_GetYawPos(void);
float MODE_TORQUE_TEST_GetYawNeg(void);
float MODE_TORQUE_TEST_GetRampCurrent(void);
float MODE_TORQUE_TEST_GetMoveDeg(void);
unsigned int MODE_TORQUE_TEST_GetPhase(void);
unsigned int MODE_TORQUE_TEST_GetDoneMask(void);

#endif /* MODE_TORQUE_TEST_H */
