/*
 * 011_spi_rx_it.c
 *
 *  Created on: Feb 3, 2025
 *      Author: Mongoo
 */

#include <string.h>
#include <stdio.h>
#include "stm32f1xx.h"

/*
 * PB12 -> SPI2_NSS
 * PB13 -> SPI2_SCK
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 */

/*
 * PA4 -> SPI1_NSS
 * PA5 -> SPI1_SCK
 * PA6 -> SPI1_MISO
 * PA7 -> SPI1_MOSI
 */

/*
 * Arduino
 * SPI_SCK : 13
 * SPI_MISO : 12
 * SPI_MOSI : 11
 * SPI_SS : 10
 */

GPIO_Handle_t BtnHandle = {0};
SPI_Handle_t SPI1Handle = {0};

uint8_t rxdata;
uint8_t txdata = 0xAB;

void delay()
{
	for (int i = 0; i < 250000; i++);
}

void GPIO_BtnInit(GPIO_Handle_t *pGPIOHandle)
{
	pGPIOHandle->pGPIOx = GPIOC;
	pGPIOHandle->PinConfig.Pin = GPIO_PIN_13;
	pGPIOHandle->PinConfig.Mode = GPIO_MODE_INPUT;
	pGPIOHandle->PinConfig.Pull = GPIO_PULLUP;
	pGPIOHandle->PinConfig.Speed = GPIO_SPEED_FREQ_LOW;

	GPIO_Init(pGPIOHandle);
}

void SPI1_GPIOInit(void)
{
	GPIO_Handle_t SPIPin;

	SPIPin.pGPIOx = GPIOA;
	SPIPin.PinConfig.Mode = GPIO_MODE_AF_PP;
	SPIPin.PinConfig.Pull = GPIO_NOPULL;
	SPIPin.PinConfig.Speed = GPIO_SPEED_FREQ_HIGH;

	// SCK
	SPIPin.PinConfig.Pin = GPIO_PIN_5;
	GPIO_Init(&SPIPin);

	// MOSI
	SPIPin.PinConfig.Pin = GPIO_PIN_7;
	GPIO_Init(&SPIPin);

	// MISO
	SPIPin.PinConfig.Pin = GPIO_PIN_6;
	GPIO_Init(&SPIPin);

	// NSS
	SPIPin.PinConfig.Pin = GPIO_PIN_4;
	GPIO_Init(&SPIPin);
}

void SPI1_Init(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx = SPI1;
	pSPIHandle->Config.Mode = SPI_MODE_MASTER;
	pSPIHandle->Config.Direction = SPI_DIRECTION_2LINES;
	pSPIHandle->Config.DataSize = SPI_DATASIZE_8BIT;
	pSPIHandle->Config.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; // 2MHz
	pSPIHandle->Config.CLKPolarity = SPI_POLARITY_LOW;
	pSPIHandle->Config.CLKPhase = SPI_PHASE_1EDGE;
	//pSPIHandle->Config.NSS = SPI_NSS_HARD_INPUT;
	pSPIHandle->Config.NSS = SPI_NSS_SOFT;

	SPI_Init(pSPIHandle);
}

int main(void)
{
	GPIO_BtnInit(&BtnHandle);

	SPI1_GPIOInit();
	SPI1_Init(&SPI1Handle);
	SPI_SSIConfig(SPI1Handle.pSPIx, ENABLE);

	SPI_IRQInterruptConfig(IRQ_NO_SPI1, ENABLE);
	SPI_IRQPriorityConfig(IRQ_NO_SPI1, NVIC_IRQ_PRI42);

	SPI_PeripheralControl(SPI1Handle.pSPIx, ENABLE);
	SPI_Receive_IT(&SPI1Handle, &rxdata, 1);

	while (1)
	{
		if (GPIO_ReadPin(BtnHandle.pGPIOx, BtnHandle.PinConfig.Pin) == GPIO_PIN_RESET)
		{
			delay();

			SPI_Transmit(&SPI1Handle, &txdata, 1);
		}
	}
}

void SPI1_IRQHandler(void)
{
	SPI_IRQHandler(&SPI1Handle);
}

void SPI_RxCpltCallback(SPI_Handle_t *pSPIHandle)
{
	if (rxdata == txdata)
	{
		printf("Interrupt based transfer success : 0x%02X\n", rxdata);
	}
	else
	{
		printf("Interrupt based transfer failed : 0x%02X\n", rxdata);
	}
	rxdata = 0U;
	SPI_Receive_IT(&SPI1Handle, &rxdata, 1);
}
