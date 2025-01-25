/*
 * 001_gpio_led_toggle.c
 *
 *  Created on: Jan 22, 2025
 *      Author: Mongoo
 */

#include "stm32f1xx.h"

void delay()
{
	for (int i = 0; i < 200000; i++);
}

int main(void)
{
	GPIO_Handle_t UserLedHandle = {0};

	UserLedHandle.pGPIOx = GPIOA;
	UserLedHandle.PinConfig.Pin = GPIO_PIN_5;
	UserLedHandle.PinConfig.Mode = GPIO_MODE_OUTPUT_PP;
	UserLedHandle.PinConfig.Pull = GPIO_NOPULL;
	UserLedHandle.PinConfig.Speed = GPIO_SPEED_FREQ_LOW;

	GPIO_Init(&UserLedHandle);
	GPIO_WritePin(UserLedHandle.pGPIOx, UserLedHandle.PinConfig.Pin, GPIO_PIN_RESET);

	while (1)
	{
		GPIO_TogglePin(UserLedHandle.pGPIOx, UserLedHandle.PinConfig.Pin);
		delay();
	}
}
