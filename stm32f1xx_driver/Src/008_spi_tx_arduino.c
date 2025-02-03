/*
 * 008_spi_tx_arduino.c
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

/*
 * Logic level converter
 * MOSI : 2
 * SCK : 3
 * NSS : 4
 */

char msg[] = "An Arduino Uno board is best suited for beginners who have just started using microcontrollers, on the other hand, Arduino Mega board is for enthusiasts who require a lot of I/O pins for their projects";

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
	pSPIHandle->Config.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4; // 2MHz
	pSPIHandle->Config.CLKPolarity = SPI_POLARITY_LOW;
	pSPIHandle->Config.CLKPhase = SPI_PHASE_1EDGE;
	//pSPIHandle->Config.NSS = SPI_NSS_HARD_INPUT;
	pSPIHandle->Config.NSS = SPI_NSS_SOFT;

	SPI_Init(pSPIHandle);
}

int main(void)
{
	char data[] = "hello world";

	GPIO_Handle_t BtnHandle = {0};
	SPI_Handle_t SPI2Handle = {0};

	GPIO_BtnInit(&BtnHandle);
	SPI2_GPIOInit();
	SPI2_Init(&SPI2Handle);

	SPI_SSIConfig(SPI2Handle.pSPIx, ENABLE);
	SPI_PeripheralControl(SPI2Handle.pSPIx, ENABLE);
	//SPI_SSOEConfig(SPI2Handle.pSPIx, ENABLE);

	while (1)
	{
		if (GPIO_ReadPin(BtnHandle.pGPIOx, BtnHandle.PinConfig.Pin) == GPIO_PIN_RESET)
		{
			delay();

			//SPI_PeripheralControl(SPI2Handle.pSPIx, ENABLE);
			uint8_t len = strlen(data);
			//SPI_Transmit(&SPI2Handle, &msglen, 1);
			//SPI_Transmit(&SPI2Handle, (uint8_t *)msg, (uint32_t)msglen);
			SPI_Transmit(&SPI2Handle, &len, 1);
			SPI_Transmit(&SPI2Handle, (uint8_t *)data, strlen(data));

			while (SPI_GetFlag(SPI2Handle.pSPIx, SPI_FLAG_BUSY) == FLAG_SET);

			//SPI_PeripheralControl(SPI2Handle.pSPIx, DISABLE);
		}
	}
}
