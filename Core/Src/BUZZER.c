/*
 * INFO:
 * This driver is specifically made for a STM32F103 chip but can be
 * easily converted to other STM32 platforms.
 *
 * This driver uses a timer peripheral to create a PWM signal to
 * drive a pizo-electric buzzer.
 *
 * NOTE:
 * Assumes buzzer is 2048 Hz
 * Buzzer is on TIM_CHANNEL_2
 * Main loop is fast (<10 Hz)
 *
 * AUTHOR:
 * Michael Gromski
 *
 * UPDATED:
 * 04/02/22
 */

#include "BUZZER.h"

uint8_t BUZZER_init(BUZZER *dev, TIM_HandleTypeDef *buzzerHandle)
{
	/*
	 * This configures the buzzer for use with this driver
	 */
	// define struct
	dev->buzzerHandle = buzzerHandle;
	dev->buzzer_tone = buzzer_off;
	dev->previous_time = 0;

	// Make sure buzzer is set correctly
	dev->buzzerHandle->Instance->PSC = 43-1;
	dev->buzzerHandle->Instance->ARR = 2048-1;
	dev->buzzerHandle->Instance->CCR2 = 0;

	HAL_TIM_PWM_Start(buzzerHandle, TIM_CHANNEL_2);

	return 0;
}

uint8_t BUZZER_update(BUZZER *dev)
{
	/*
	 * This will need to run at the end of every loop due to
	 * the dependence on system tick for a non-blocking timer
	 */

	uint32_t current_time = HAL_GetTick(); // Get time since startup in milliseconds
	dev->elapsed_time = current_time - dev->previous_time; // note: overflow in ~50 days

	// steady on
	if (dev->buzzer_tone == buzzer_steady_on)
	{

		dev->buzzerHandle->Instance->CCR2 = 1024;
	}

	// short tone (activate for 100ms on a 1000ms interval)
	else if (dev->buzzer_tone == buzzer_short_tone)
	{
		if (dev->elapsed_time <= 100)
		{
			dev->buzzerHandle->Instance->CCR2 = 1024;
		}
		else if(dev->elapsed_time > 100 && dev->elapsed_time <= 1000)
		{
			dev->buzzerHandle->Instance->CCR2 = 0;
		}
		else
		{
			dev->previous_time = current_time;
		}
	}

	// long tone (activate for 800ms on a 1000ms interval)
	else if (dev->buzzer_tone == buzzer_long_tone)
	{
		if (dev->elapsed_time <= 800)
		{
			dev->buzzerHandle->Instance->CCR2 = 1024;
		}
		else if(dev->elapsed_time > 800 && dev->elapsed_time <= 1000)
		{
			dev->buzzerHandle->Instance->CCR2 = 0;
		}
		else
		{
			dev->previous_time = current_time;
		}
	}
	// fast on/off (activate for 125ms on 250ms interval)
	else if (dev->buzzer_tone == buzzer_fast_on_off)
	{
		if (dev->elapsed_time <= 125)
		{
			dev->buzzerHandle->Instance->CCR2 = 1024;
		}
		else if(dev->elapsed_time > 125 && dev->elapsed_time <= 250)
		{
			dev->buzzerHandle->Instance->CCR2 = 0;
		}
		else
		{
			dev->previous_time = current_time;
		}
	}

	// slow on/off (activate for 100ms on 4000ms interval)
	else if (dev->buzzer_tone == buzzer_slow_on_off)
	{
		if (dev->elapsed_time <= 100)
		{
			dev->buzzerHandle->Instance->CCR2 = 1024;
		}
		else if(dev->elapsed_time > 100 && dev->elapsed_time <= 4000)
		{
			dev->buzzerHandle->Instance->CCR2 = 0;
		}
		else
		{
			dev->previous_time = current_time;
		}
	}

	// steady off
	else
	{
		dev->buzzerHandle->Instance->CCR2 = 0; // faster to stop timer or make duty cycle 0%?
	}

	return 0;
}
