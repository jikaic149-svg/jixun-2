#ifndef MODE_VISION_STAB_H
#define MODE_VISION_STAB_H

#include <stdint.h>
#include "gimbal_mode.h"

void MODE_VISION_STAB_ResetTarget(void);
void MODE_VISION_STAB_Update(GimbalModeOutput_t *out);
uint8_t MODE_VISION_STAB_GetVisionActive(void);
float MODE_VISION_STAB_GetVisionPitchOffset(void);
float MODE_VISION_STAB_GetVisionYawOffset(void);
float MODE_VISION_STAB_GetVisionInputDx(void);
float MODE_VISION_STAB_GetVisionInputDy(void);
float MODE_VISION_STAB_GetVisionPredictDx(void);
float MODE_VISION_STAB_GetVisionPredictDy(void);
uint32_t MODE_VISION_STAB_GetVisionPredictMs(void);

#endif /* MODE_VISION_STAB_H */
