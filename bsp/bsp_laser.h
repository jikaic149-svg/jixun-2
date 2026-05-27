#ifndef BSP_LASER_H
#define BSP_LASER_H

#include <stdint.h>

void BSP_Laser_Init(void);
void BSP_Laser_Set(uint8_t on);
uint8_t BSP_Laser_GetState(void);

#endif /* BSP_LASER_H */
