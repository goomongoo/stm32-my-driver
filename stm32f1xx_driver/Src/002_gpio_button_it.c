/*
 * 002_gpio_button_it.c
 *
 *  Created on: Jan 22, 2025
 *      Author: Mongoo
 */

#include "stm32f1xx.h"

void	delay()
{
	for (int i = 0; i < 200000; i++);
}

int	main(void)
{
	GPIO_Handle_t	LedHandle = {0};
	GPIO_Handle_t	BtnHandle = {0};

	LedHandle.pGPIOx = GPIOA;
	LedHandle.PinConfig.Pin = GPIO_PIN_5;
	LedHandle.PinConfig.Mode = GPIO_MODE_OUTPUT_PP;
	LedHandle.PinConfig.Pull = GPIO_NOPULL;
	LedHandle.PinConfig.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_Init(&LedHandle);
	GPIO_WritePin(LedHandle.pGPIOx, LedHandle.PinConfig.Pin, GPIO_PIN_RESET);

	BtnHandle.pGPIOx = GPIOC;
	BtnHandle.PinConfig.Pin = GPIO_PIN_13;
	BtnHandle.PinConfig.Mode = GPIO_MODE_IT_FALLING;
	BtnHandle.PinConfig.Pull = GPIO_PULLUP;
	GPIO_Init(&BtnHandle);

	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI47);

	while (1);
}

void	EXTI15_10_IRQHandler(void)
{
	delay();
	GPIO_EXTI_IRQHandler(GPIO_PIN_13);
	GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}
