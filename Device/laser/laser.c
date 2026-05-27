#include "laser.h"
#include <string.h>

void Laser_Init(Laser_Instance_t *inst, const Laser_Config_t *config)
{
    memset(inst, 0, sizeof(*inst));
    if (config != 0) {
        inst->config = *config;
    }
}

void Laser_SetOn(Laser_Instance_t *inst, uint8_t on)
{
    if (inst->config.set_output == 0) {
        return;
    }

    inst->config.set_output(on ? 1U : 0U);
}

uint8_t Laser_IsOn(const Laser_Instance_t *inst)
{
    if (inst->config.get_output == 0) {
        return 0U;
    }

    return inst->config.get_output();
}
