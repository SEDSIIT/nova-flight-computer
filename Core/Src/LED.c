/*
 * INFO:
 * This is a driver specifically made for a F103 chip to control the brightness of
 * an LED using the PWM peripheral multiple patterns of blinking are indicators of
 * different system status states.
 *
 * NOTE:
 * This assumes both LED are controlled by the same timer and the two LEDs are on
 * channels 1 and 2
 *
 * AUTHOR:
 * Michael Gromski
 *
 * UPDATED:
 * 3/17/22
 */

#include "LED.h"

uint8_t LED_init(LED *dev, TIM_HandleTypeDef *pwmledHandle)
{
	/*
	 * This configures a LED for use with this driver
	 */

	// define struct
	dev->pwmledHandle = pwmledHandle;
	dev->led_status = led_off;

	HAL_TIM_PWM_Start(pwmledHandle, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(pwmledHandle, TIM_CHANNEL_2);

	// set default LED status
	LED_set_status(dev, led_green, led_steady_on);
	LED_set_status(dev, led_blue, led_fast_blink);

	return 0;
}

uint8_t LED_set_status(LED *dev, uint8_t led_num, uint8_t led_status)
{
	/*
	 * This will set the blink pattern for a PWM LED
	 */

	// steady on
	if (led_status == led_steady_on)
	{
		if (led_num == led_blue)
		{
			dev->pwmledHandle->Instance->CCR1 = 65535;
		}
		else
		{
			dev->pwmledHandle->Instance->CCR2 = 65535;
		}
	}

	// blink fast
	else if (led_status == led_fast_blink)
	{
		dev->pwmledHandle->Instance->PSC = 669-1;
		if (led_num == led_blue)
		{
			dev->pwmledHandle->Instance->CCR1 = 32768;
		}
		else
		{
			dev->pwmledHandle->Instance->CCR2 = 32768;
		}
	}

	// blink slow
	else if (led_status == led_slow_blink)
	{
		dev->pwmledHandle->Instance->PSC = 2676-1;
		if (led_num == led_blue)
		{
			dev->pwmledHandle->Instance->CCR1 = 32768;
		}
		else
		{
			dev->pwmledHandle->Instance->CCR2 = 32768;
		}
	}

	// off
	else
	{
		if (led_num == led_blue)
		{
			dev->pwmledHandle->Instance->CCR1 = 0;
		}
		else
		{
			dev->pwmledHandle->Instance->CCR2 = 0;
		}
	}
	return 0;
}
