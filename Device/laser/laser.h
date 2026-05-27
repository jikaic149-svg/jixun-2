#ifndef LASER_H
#define LASER_H

#include <stdint.h>

typedef void (*Laser_SetOutput_t)(uint8_t on);
typedef uint8_t (*Laser_GetOutput_t)(void);

typedef struct
{
    Laser_SetOutput_t set_output;
    Laser_GetOutput_t get_output;
} Laser_Config_t;

typedef struct
{
    Laser_Config_t config;
} Laser_Instance_t;

void Laser_Init(Laser_Instance_t *inst, const Laser_Config_t *config);
void Laser_SetOn(Laser_Instance_t *inst, uint8_t on);
uint8_t Laser_IsOn(const Laser_Instance_t *inst);

#endif /* LASER_H */
