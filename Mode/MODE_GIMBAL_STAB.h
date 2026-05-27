#ifndef MODE_GIMBAL_STAB_H
#define MODE_GIMBAL_STAB_H

#include <stdint.h>
#include "gimbal_mode.h"

void MODE_GIMBAL_STAB_ResetTarget(void);
void MODE_GIMBAL_STAB_SetTarget(float pitch_deg, float yaw_deg);
float MODE_GIMBAL_STAB_GetTargetPitch(void);
float MODE_GIMBAL_STAB_GetTargetYaw(void);
void MODE_GIMBAL_STAB_Update(GimbalModeOutput_t *out);

#endif /* MODE_GIMBAL_STAB_H */
