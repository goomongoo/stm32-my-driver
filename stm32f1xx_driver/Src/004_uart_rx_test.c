/*
 * 004_uart_rx_test.c
 *
 *  Created on: Jan 24, 2025
 *      Author: Mongoo
 */

#include <stdio.h>
#include <string.h>
#include "stm32f1xx.h"

char data[128];

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
	pUSARTHandle->Config.Mode = UART_MODE_TX_RX;
	pUSARTHandle->Config.WordLength = UART_WORDLENGTH_8B;
	pUSARTHandle->Config.StopBits = UART_STOPBITS_1;
	pUSARTHandle->Config.Parity = UART_PARITY_NONE;
	pUSARTHandle->Config.HwFlowCtl = UART_HWCONTROL_NONE;
	pUSARTHandle->Config.BaudRate = UART_BAUDRATE_115200;

	USART_Init(pUSARTHandle);
}

int main(void)
{
	USART_Handle_t	USART2Handle;

	USART2_GPIOInit();
	USART2_Init(&USART2Handle);

	while (1)
	{
		USART_Receive(&USART2Handle, (uint8_t *)data, 6);
		USART_Transmit(&USART2Handle, (uint8_t *)data, strlen(data));
	}
}
