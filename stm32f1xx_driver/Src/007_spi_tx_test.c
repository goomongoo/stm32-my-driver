/*
 * 007_spi_tx_test.c
 *
 *  Created on: Feb 2, 2025
 *      Author: Mongoo
 */

#include <string.h>
#include "stm32f1xx.h"

/*
 * PB12 -> SPI2_NSS
 * PB13 -> SPI2_SCK
 * PB14 -> SPI2_MISO
 * PB15 -> SPI2_MOSI
 */

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

void SPI2_GPIOInit(void)
{
	GPIO_Handle_t SPIPin;

	SPIPin.pGPIOx = GPIOB;
	SPIPin.PinConfig.Mode = GPIO_MODE_AF_PP;
	SPIPin.PinConfig.Pull = GPIO_NOPULL;
	SPIPin.PinConfig.Speed = GPIO_SPEED_FREQ_HIGH;

	// SCK
	SPIPin.PinConfig.Pin = GPIO_PIN_13;
	GPIO_Init(&SPIPin);

	// MOSI
	SPIPin.PinConfig.Pin = GPIO_PIN_15;
	GPIO_Init(&SPIPin);

	// MISO
	//SPIPin.PinConfig.Pin = GPIO_PIN_14;
	//GPIO_Init(&SPIPin);

	// NSS
	//SPIPin.PinConfig.Pin = GPIO_PIN_12;
	//GPIO_Init(&SPIPin);
}

void SPI2_Init(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx = SPI2;
	pSPIHandle->Config.Mode = SPI_MODE_MASTER;
	pSPIHandle->Config.Direction = SPI_DIRECTION_2LINES;
	pSPIHandle->Config.DataSize = SPI_DATASIZE_8BIT;
	pSPIHandle->Config.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	pSPIHandle->Config.CLKPolarity = SPI_POLARITY_LOW;
	pSPIHandle->Config.CLKPhase = SPI_PHASE_1EDGE;
	pSPIHandle->Config.NSS = SPI_NSS_SOFT;

	SPI_Init(pSPIHandle);
}

int main(void)
{
	char msg[] = "Hello world\n\r";
	SPI_Handle_t SPI2Handle = {0};
	GPIO_Handle_t BtnHandle = {0};

	GPIO_BtnInit(&BtnHandle);
	SPI2_GPIOInit();
	SPI2_Init(&SPI2Handle);
	SPI_SSIConfig(SPI2Handle.pSPIx, ENABLE);

	while (1)
	{
		SPI_Transmit(&SPI2Handle, (uint8_t *)msg, strlen(msg));
		delay();
		delay();
		delay();
		delay();
		delay();
		delay();
	}
}
