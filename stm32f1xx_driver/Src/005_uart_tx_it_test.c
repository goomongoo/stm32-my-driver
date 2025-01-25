/*
 * 005_uart_tx_it_test.c
 *
 *  Created on: Jan 24, 2025
 *      Author: Mongoo
 */

#include <string.h>
#include "stm32f1xx.h"

char msg[1024] = "Hello from UART\n\r";

USART_Handle_t	USART2Handle;

void delay()
{
	for (int i = 0; i < 200000; i++);
}

void GPIO_BtnInit(GPIO_Handle_t *pGPIOHandle)
{
	pGPIOHandle->pGPIOx = GPIOC;
	pGPIOHandle->PinConfig.Pin = GPIO_PIN_13;
	pGPIOHandle->PinConfig.Mode = GPIO_MODE_INPUT;
	pGPIOHandle->PinConfig.Pull = GPIO_PULLUP;

	GPIO_Init(pGPIOHandle);
}

void USART2_GPIOInit()
{
	GPIO_Handle_t	USART2_GPIOHandle;

	USART2_GPIOHandle.pGPIOx = GPIOA;
	USART2_GPIOHandle.PinConfig.Pin = GPIO_PIN_2;
	USART2_GPIOHandle.PinConfig.Mode = GPIO_MODE_AF_PP;
	USART2_GPIOHandle.PinConfig.Pull = GPIO_NOPULL;
	USART2_GPIOHandle.PinConfig.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_Init(&USART2_GPIOHandle);
}

void USART2_Init(USART_Handle_t *pUSARTHandle)
{
	pUSARTHandle->pUSARTx = USART2;
	pUSARTHandle->Config.Mode = UART_MODE_TX;
	pUSARTHandle->Config.WordLength = UART_WORDLENGTH_8B;
	pUSARTHandle->Config.StopBits = UART_STOPBITS_1;
	pUSARTHandle->Config.Parity = UART_PARITY_NONE;
	pUSARTHandle->Config.HwFlowCtl = UART_HWCONTROL_NONE;
	pUSARTHandle->Config.BaudRate = UART_BAUDRATE_115200;

	USART_Init(pUSARTHandle);
}

int main(void)
{
	GPIO_Handle_t	BtnHandle;
	//USART_Handle_t	USART2Handle;

	GPIO_BtnInit(&BtnHandle);
	USART2_GPIOInit();
	USART2_Init(&USART2Handle);
	USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);
	USART_IRQPriorityConfig(IRQ_NO_USART2, NVIC_IRQ_PRI47);

	while (1)
	{
		if (GPIO_ReadPin(BtnHandle.pGPIOx, BtnHandle.PinConfig.Pin) == GPIO_PIN_RESET)
		{
			delay();
			USART_Transmit_IT(&USART2Handle, (uint8_t *)msg, strlen(msg));
		}
	}
}

void USART2_IRQHandler(void)
{
	USART_IRQHandler(&USART2Handle);
}
