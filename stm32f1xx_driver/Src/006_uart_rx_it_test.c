/*
 * 006_uart_rx_it_test.c
 *
 *  Created on: Jan 24, 2025
 *      Author: Mongoo
 */

#include <stdio.h>
#include <string.h>
#include "stm32f1xx.h"

char rcv;
char buf[128];

//USART_Handle_t	USART2Handle;
USART_Handle_t	USART1Handle;

char *bufptr;

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

	USART2_GPIOHandle.PinConfig.Pin = GPIO_PIN_3;
	USART2_GPIOHandle.PinConfig.Mode = GPIO_MODE_INPUT;
	USART2_GPIOHandle.PinConfig.Pull = GPIO_PULLUP;
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

void USART1_GPIOInit()
{
	GPIO_Handle_t	USART1_GPIOHandle;

	USART1_GPIOHandle.pGPIOx = GPIOA;
	USART1_GPIOHandle.PinConfig.Pin = GPIO_PIN_9;
	USART1_GPIOHandle.PinConfig.Mode = GPIO_MODE_AF_PP;
	USART1_GPIOHandle.PinConfig.Pull = GPIO_NOPULL;
	USART1_GPIOHandle.PinConfig.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_Init(&USART1_GPIOHandle);

	USART1_GPIOHandle.PinConfig.Pin = GPIO_PIN_10;
	USART1_GPIOHandle.PinConfig.Mode = GPIO_MODE_INPUT;
	USART1_GPIOHandle.PinConfig.Pull = GPIO_PULLUP;
	USART1_GPIOHandle.PinConfig.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_Init(&USART1_GPIOHandle);
}

void USART1_Init(USART_Handle_t *pUSARTHandle)
{
	pUSARTHandle->pUSARTx = USART1;
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
	//GPIO_Handle_t	BtnHandle;
	//USART_Handle_t	USART2Handle;

	//GPIO_BtnInit(&BtnHandle);
	//USART2_GPIOInit();
	//USART2_Init(&USART2Handle);
	//USART_IRQInterruptConfig(IRQ_NO_USART2, ENABLE);
	//USART_IRQPriorityConfig(IRQ_NO_USART2, NVIC_IRQ_PRI47);

	USART1_GPIOInit();
	USART1_Init(&USART1Handle);
	USART_IRQInterruptConfig(IRQ_NO_USART1, ENABLE);
	USART_IRQPriorityConfig(IRQ_NO_USART1, NVIC_IRQ_PRI44);

	bufptr = buf;

	USART_Receive_IT(&USART1Handle, (uint8_t *)&rcv, 1);

	while (1)
	{

	}
}

void USART1_IRQHandler(void)
{
	USART_IRQHandler(&USART1Handle);
}

void USART_RxCpltCallback(USART_Handle_t *pUSARTHandle)
{
	if (rcv == '\r')
	{
		*bufptr = '\0';
		bufptr = buf;
		printf("%s\n", buf);
	}
	else
	{
		*bufptr++ = rcv;
	}
	USART_Receive_IT(pUSARTHandle, (uint8_t *)&rcv, 1);
}
