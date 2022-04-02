/*
 * INFO:
 * This driver is specifically made for a STM32F103 chip but can be
 * easily converted to other STM32 platforms. This driver uses a
 * timer peripheral to create a PWM signal to drive a pizo-electric
 * buzzer.
 *
 * NOTE:
 * Assumes buzzer is 2048 Hz
 *
 * AUTHOR:
 * Michael Gromski
 *
 * UPDATED:
 * 04/02/22
 */

#ifndef BUZZER_PWM_DRIVER_H
#define BUZZER_PWM_DRIVER_H

#include "stm32f4xx_hal.h"

typedef enum
{
	buzzer_off = 0,
	buzzer_steady_on = 1,
	buzzer_short_tone = 2,
	buzzer_long_tone = 3,
	buzzer_fast_on_off = 4, // indicates failure somewhere in system
	buzzer_slow_on_off = 5, // indicates all systems are in good status/recovery tones

} BUZZER_TONE;

typedef struct
{
	TIM_HandleTypeDef *buzzerHandle;
	BUZZER_TONE buzzer_tone;
	uint32_t previous_time;
	uint32_t elapsed_time;
} BUZZER;



// Initialize the buzzer
uint8_t BUZZER_init(BUZZER *dev, TIM_HandleTypeDef *buzzerHandle);

// Set buzzer tone
uint8_t BUZZER_update(BUZZER *dev);


#endif
