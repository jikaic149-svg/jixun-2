#ifndef MODE_VISION_STAB_H
#define MODE_VISION_STAB_H

#include <stdint.h>
#include "gimbal_mode.h"

void MODE_VISION_STAB_ResetTarget(void);
void MODE_VISION_STAB_Update(GimbalModeOutput_t *out);
uint8_t MODE_VISION_STAB_GetVisionActive(void);
float MODE_VISION_STAB_GetVisionPitchOffset(void);
float MODE_VISION_STAB_GetVisionYawOffset(void);

#endif /* MODE_VISION_STAB_H */
