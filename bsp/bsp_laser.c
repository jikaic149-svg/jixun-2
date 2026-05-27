#include "bsp_laser.h"
#include "main.h"

static uint8_t laser_state = 0U;

void BSP_Laser_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(LASER_CTRL_GPIO_Port, LASER_CTRL_Pin, GPIO_PIN_RESET);

    gpio.Pin = LASER_CTRL_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LASER_CTRL_GPIO_Port, &gpio);

    laser_state = 0U;
}

void BSP_Laser_Set(uint8_t on)
{
    laser_state = on ? 1U : 0U;
    HAL_GPIO_WritePin(LASER_CTRL_GPIO_Port,
                      LASER_CTRL_Pin,
                      laser_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t BSP_Laser_GetState(void)
{
    return laser_state;
}
