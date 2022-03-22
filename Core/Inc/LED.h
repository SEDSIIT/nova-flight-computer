/*
 * INFO:
 * This is a driver specifically made for a F103 chip to control the brightness of
 * an LED using the PWM peripheral multiple patterns of blinking are indicators of
 * different system status states.
 *
 * AUTHOR:
 * Michael Gromski
 *
 * UPDATED:
 * 3/17/22
 *
 */

#ifndef LED_PWM_DRIVER_H
#define LED_PWM_DRIVER_H

#include "stm32f4xx_hal.h"

typedef enum
{
	led_off 		= 0,
	led_steady_on	= 1,
	led_fast_blink	= 2,
	led_slow_blink 	= 3
} LED_STATUS;

typedef enum
{
	led_green = 0,
	led_blue = 1
} LED_SELECT;

typedef struct
{
	TIM_HandleTypeDef *pwmledHandle;
	LED_STATUS led_status;
} LED;

/*
 * Initialization
 */
uint8_t LED_init(LED *dev, TIM_HandleTypeDef *pwmledHandle);

/*
 * Set LED status
 */
uint8_t LED_set_status(LED *dev, uint8_t led_num, uint8_t led_status);

#endif
