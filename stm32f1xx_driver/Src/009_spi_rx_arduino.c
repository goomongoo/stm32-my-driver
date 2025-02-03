/*
 * 009_spi_rx_arduino.c
 *
 *  Created on: Feb 2, 2025
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
	pSPIHandle->Config.NSS = SPI_NSS_HARD_INPUT;
	//pSPIHandle->Config.NSS = SPI_NSS_SOFT;

	SPI_Init(pSPIHandle);
}

int main(void)
{
	uint8_t rxdummy;
	uint8_t dataLen;
	char buf[32];

	GPIO_Handle_t BtnHandle = {0};
	SPI_Handle_t SPI1Handle = {0};

	GPIO_BtnInit(&BtnHandle);
	SPI1_GPIOInit();
	SPI1_Init(&SPI1Handle);
	SPI_SSOEConfig(SPI1Handle.pSPIx, ENABLE);

	while (1)
	{
		if (GPIO_ReadPin(BtnHandle.pGPIOx, BtnHandle.PinConfig.Pin) == GPIO_PIN_RESET)
		{
			delay();

			SPI_Receive(&SPI1Handle, &rxdummy, 1);
			while (SPI_GetFlag(SPI1Handle.pSPIx, SPI_FLAG_BUSY) == FLAG_SET);
			SPI_Receive(&SPI1Handle, &dataLen, 1);
			while (SPI_GetFlag(SPI1Handle.pSPIx, SPI_FLAG_BUSY) == FLAG_SET);
			printf("Len : %d, Msg : ", (int)dataLen);
			SPI_Receive(&SPI1Handle, (uint8_t *)buf, dataLen);
			while (SPI_GetFlag(SPI1Handle.pSPIx, SPI_FLAG_BUSY) == FLAG_SET);
			SPI_PeripheralControl(SPI1Handle.pSPIx, DISABLE);
			buf[dataLen] = '\0';
			printf("%s\n", buf);
		}
	}
}
