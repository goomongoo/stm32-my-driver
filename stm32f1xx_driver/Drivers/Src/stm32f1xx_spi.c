/*
 * stm32f1xx_spi.c
 *
 *  Created on: Jan 25, 2025
 *      Author: Mongoo
 */

#include "stm32f1xx_spi.h"

/**
  * @brief
  * @param
  * @retval
  */
void	SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_CLOCK_ENABLE();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_CLOCK_ENABLE();
		}
	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_CLOCK_DISABLE();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_CLOCK_DISABLE();
		}
	}
}

/**
  * @brief
  * @param
  * @retval
  */
void	SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PERI_ENABLE();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PERI_ENABLE();
		}
	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PERI_DISABLE();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PERI_DISABLE();
		}
	}
}

/**
  * @brief
  * @param
  * @retval
  */
void	SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t	config;

	/* SPI Peripheral Clock Enable */
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	/* ----- CR1 Register Configuration ----- */
	config = 0U;

	/* SPI Mode Configuration */
	config |= pSPIHandle->Config.Mode;

	/* SPI Direction Configuration */
	config |= pSPIHandle->Config.Direction;

	/* SPI Data Size Configuration */
	config |= pSPIHandle->Config.DataSize;

	/* SPI Clock Polarity Configuration */
	config |= pSPIHandle->Config.CLKPolarity;

	/* SPI Clock Phase Configuration */
	config |= pSPIHandle->Config.CLKPhase;

	/* SPI Slave Select Management Configuration */
	config |= pSPIHandle->Config.NSS;

	/* SPI Baud Rate Prescaler Configuration */
	config |= pSPIHandle->Config.BaudRatePrescaler;

	/* Apply configuration to CR1 Register */
	pSPIHandle->pSPIx->CR1 = (config & 0xFFFFU);
}

/**
  * @brief
  * @param
  * @retval
  */
void	SPI_DeInit(SPI_Handle_t *pSPIHandle)
{
	/* SPI Peripheral Disable */
	SPI_PeripheralControl(pSPIHandle->pSPIx, ENABLE);
}

/**
  * @brief
  * @param
  * @retval
  */
uint8_t	SPI_GetFlag(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	uint8_t	status;

	if (pSPIx->SR & FlagName)
	{
		status = FLAG_SET;
	}
	else
	{
		status = FLAG_RESET;
	}

	return status;
}

/**
  * @brief
  * @param
  * @retval
  */
void	SPI_Transmit(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	/* Configure communication direction : BIDIMODE */
	if (pSPIHandle->Config.Direction == SPI_DIRECTION_1LINE)
	{
		SPI_PeripheralControl(pSPIHandle->pSPIx, DISABLE);
		pSPIHandle->pSPIx->CR1 |= SPI_CR1_BIDIOE;
	}

	/* Enable SPI if disabled */
	if ((pSPIHandle->pSPIx->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
	{
		SPI_PeripheralControl(pSPIHandle->pSPIx, ENABLE);
	}

	/* Data Frame 16-bit */
	if (pSPIHandle->Config.DataSize == SPI_DATASIZE_16BIT)
	{
		while (Len > 0U)
		{
			/* Wait until TXE flag is set */
			while (SPI_GetFlag(pSPIHandle->pSPIx, SPI_FLAG_TXE) == FLAG_RESET);

			/* Load data into DR register */
			pSPIHandle->pSPIx->DR = *(uint16_t *)pTxBuffer;

			/* Move Buffer Pointer */
			pTxBuffer += 2U;

			/* Decrease Len */
			Len--;
		}
	}
	/* Data Frame 8-bit */
	else if (pSPIHandle->Config.DataSize == SPI_DATASIZE_8BIT)
	{
		while (Len > 0U)
		{
			/* Wait until TXE flag is set */
			while (SPI_GetFlag(pSPIHandle->pSPIx, SPI_FLAG_TXE) == FLAG_RESET);

			/* Load data into DR register */
			pSPIHandle->pSPIx->DR = *pTxBuffer;

			/* Move Buffer Pointer */
			pTxBuffer++;

			/* Decrease Len */
			Len--;
		}
	}

	/* Clear overrun flag in 2 Lines communication mode because received is not read */
	if (pSPIHandle->Config.Direction == SPI_DIRECTION_2LINES)
	{
		(void)pSPIHandle->pSPIx->DR;
		(void)pSPIHandle->pSPIx->SR;
	}
}

/**
  * @brief
  * @param
  * @retval
  */
void	SPI_Receive(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	/* Configure communication direction if BIDIMODE */
	if (pSPIHandle->Config.Direction == SPI_DIRECTION_1LINE)
	{
		SPI_PeripheralControl(pSPIHandle->pSPIx, DISABLE);
		pSPIHandle->pSPIx->CR1 &= ~SPI_CR1_BIDIOE;
	}

	/* Enable SPI if disabled */
	if ((pSPIHandle->pSPIx->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
	{
		SPI_PeripheralControl(pSPIHandle->pSPIx, ENABLE);
	}

	/* Data Frame 16-bit */
	if (pSPIHandle->Config.DataSize == SPI_DATASIZE_16BIT)
	{
		while (Len > 0U)
		{
			/* Wait until RXNE flag is set */
			while (SPI_GetFlag(pSPIHandle->pSPIx, SPI_FLAG_RXNE) == FLAG_RESET);

			/* Load data into RX Buffer */
			*(uint16_t *)pRxBuffer = (uint16_t)pSPIHandle->pSPIx->DR;

			/* Move Buffer Pointer */
			pRxBuffer += 2U;

			/* Decrease Len */
			Len--;
		}
	}
	/* Data Frame 8-bit */
	else if (pSPIHandle->Config.DataSize == SPI_DATASIZE_8BIT)
	{
		while (Len > 0U)
		{
			/* Wait until RXNE flag is set */
			while (SPI_GetFlag(pSPIHandle->pSPIx, SPI_FLAG_RXNE) == FLAG_RESET);

			/* Load data into RX Buffer */
			*pRxBuffer = (uint8_t)pSPIHandle->pSPIx->DR;

			/* Move Buffer Pointer */
			pRxBuffer++;

			/* Decrease Len */
			Len--;
		}
	}
}

/**
  * @brief
  * @param
  * @retval
  */
uint8_t	SPI_Transmit_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if (state != SPI_STATE_BUSY_TX)
	{
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		pSPIHandle->TxState = SPI_STATE_BUSY_TX;

		/* Configure communication direction : 1LINE */
		if (pSPIHandle->Config.Direction == SPI_DIRECTION_1LINE)
		{
			SPI_PeripheralControl(pSPIHandle->pSPIx, DISABLE);
			pSPIHandle->pSPIx->CR1 |= SPI_CR1_BIDIOE;
		}

		/* Enable SPI if disabled */
		if ((pSPIHandle->pSPIx->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
		{
			SPI_PeripheralControl(pSPIHandle->pSPIx, ENABLE);
		}

		/* Enable TXE, ERR interrupt */
		pSPIHandle->pSPIx->CR2 |= (SPI_CR2_TXEIE | SPI_CR2_ERRIE);
	}

	return state;
}

/**
  * @brief
  * @param
  * @retval
  */
uint8_t	SPI_Receive_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_STATE_BUSY_RX)
	{
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		pSPIHandle->RxState = SPI_STATE_BUSY_RX;

		/* Configure communication direction : 1LINE */
		if (pSPIHandle->Config.Direction == SPI_DIRECTION_1LINE)
		{
			SPI_PeripheralControl(pSPIHandle->pSPIx, DISABLE);
			pSPIHandle->pSPIx->CR1 &= ~SPI_CR1_BIDIOE;
		}

		/* Enable SPI if disabled */
		if ((pSPIHandle->pSPIx->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
		{
			SPI_PeripheralControl(pSPIHandle->pSPIx, ENABLE);
		}

		/* Enable RXNE, ERR interrupt */
		pSPIHandle->pSPIx->CR2 |= (SPI_CR2_RXNEIE | SPI_CR2_ERRIE);
	}

	return state;
}

/**
  * @brief
  * @param
  * @retval
  */
void	SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= SPI_CR1_SSI;
	}
	else
	{
		pSPIx->CR1 &= ~SPI_CR1_SSI;
	}
}

/**
  * @brief
  * @param
  * @retval
  */
void	SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR2 |= SPI_CR2_SSOE;
	}
	else
	{
		pSPIx->CR2 &= ~SPI_CR2_SSOE;
	}
}

/**
  * @brief  Configures the NVIC interrupt for a specified IRQ number.
  * @param  IRQNumber : Interrupt request (IRQ) number to be configured.
  * @param  EnorDi : ENABLE to enable the interrupt, DISABLE to disable it.
  * @retval None
  */
void	SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	/* Enable interrupt */
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber <= 59)
		{
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
	}
	/* Disable interrupt */
	else
	{
		if (IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber <= 59)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
	}
}

/**
  * @brief  Configures the priority of the specified IRQ.
  * @param  IRQNumber : Interrupt request (IRQ) number whose priority needs to be configured.
  * @param  IRQPriority : Priority level to be assigned to the IRQ (lower value indicates higher priority).
  * @retval None
  */
void	SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	__IO uint8_t	*pIprRegister;
	uint32_t		byteOffset;

	/* Get address of IPR register */
	pIprRegister = (__IO uint8_t *)(NVIC_IPR_BASE + (IRQNumber / 4));

	/* Get byte offset within the IPR register */
	byteOffset = IRQNumber % 4;

	/*
	 * Set the priority value in the appropriate byte of the IPR register
	 * 1. Shift the priority value to align with the upper bits of the 8-bit register.
	 * 2. Mask the value to ensure it does not exceed the valid range.
	 */
	*(pIprRegister + byteOffset) = (uint8_t)((IRQPriority << (8U - NVIC_PRIO_BITS)) & (uint32_t)0xFFUL);
}

/*
 * Function prototypes for interrupt handlers
 */
static void	_SPI_Transmit_IT(SPI_Handle_t *pSPIHandle);
static void	_SPI_Receive_IT(SPI_Handle_t *pSPIHandle);
static void	_SPI_ERR_OVRHandler(SPI_Handle_t *pSPIHandle);

/**
  * @brief
  * @param
  * @retval
  */
void	SPI_IRQHandler(SPI_Handle_t *pSPIHandle)
{
	/* TXE interrupt */
	if ((pSPIHandle->pSPIx->SR & SPI_SR_TXE) && (pSPIHandle->pSPIx->CR2 & SPI_CR2_TXEIE))
	{

		_SPI_Transmit_IT(pSPIHandle);
		return;
	}

	/* RXNE interrupt */
	if ((pSPIHandle->pSPIx->SR & SPI_SR_RXNE) && (pSPIHandle->pSPIx->CR2 & SPI_CR2_RXNEIE))
	{
		_SPI_Receive_IT(pSPIHandle);
		return;
	}

	/* OVR flag */
	if ((pSPIHandle->pSPIx->SR & SPI_SR_OVR) && (pSPIHandle->pSPIx->CR2 & SPI_CR2_ERRIE))
	{
		_SPI_ERR_OVRHandler(pSPIHandle);
		return;
	}
}

/**
  * @brief
  * @param
  * @retval
  */
static void	_SPI_Transmit_IT(SPI_Handle_t *pSPIHandle)
{
	/* Data Frame 16-bit */
	if (pSPIHandle->Config.DataSize == SPI_DATASIZE_16BIT)
	{
		pSPIHandle->pSPIx->DR = *(uint16_t *)pSPIHandle->pTxBuffer;
		pSPIHandle->pTxBuffer += 2U;
		pSPIHandle->TxLen--;
	}
	/* Data Frame 8-bit */
	else
	{
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->pTxBuffer++;
		pSPIHandle->TxLen--;
	}

	if (pSPIHandle->TxLen == 0U)
	{
		/* Disable TXE, ERR interrupt */
		pSPIHandle->pSPIx->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_ERRIE);

		/* Reset state */
		pSPIHandle->TxState = SPI_STATE_READY;

		/* Application Callback */
		SPI_TxCpltCallback(pSPIHandle);
	}
}

/**
  * @brief
  * @param
  * @retval
  */
static void	_SPI_Receive_IT(SPI_Handle_t *pSPIHandle)
{
	/* Data Frame 16-bit */
	if (pSPIHandle->Config.DataSize == SPI_DATASIZE_16BIT)
	{
		*(uint16_t *)pSPIHandle->pRxBuffer = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->pRxBuffer += 2U;
		pSPIHandle->RxLen--;
	}
	/* Data Frame 8-bit */
	else
	{
		*pSPIHandle->pRxBuffer = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen--;
	}

	if (pSPIHandle->RxLen == 0U)
	{
		/* Disable RXNE, ERR interrupt */
		pSPIHandle->pSPIx->CR2 &= ~(SPI_CR2_RXNEIE | SPI_CR2_ERRIE);

		/* Reset state */
		pSPIHandle->RxState = SPI_STATE_READY;

		/* Application Callback */
		SPI_RxCpltCallback(pSPIHandle);
	}
}

/**
  * @brief
  * @param
  * @retval
  */
static void	_SPI_ERR_OVRHandler(SPI_Handle_t *pSPIHandle)
{
	/* Clear OVR flag */
	if (pSPIHandle->TxState != SPI_STATE_BUSY_TX)
	{
		(void)pSPIHandle->pSPIx->DR;
		(void)pSPIHandle->pSPIx->SR;
	}

	pSPIHandle->ErrorState = SPI_ERROR_OVR;

	/* Application Callback */
	SPI_ErrorCallback(pSPIHandle);
}

/**
  * @brief
  * @param
  * @retval
  */
__weak void	SPI_TxCpltCallback(SPI_Handle_t *pSPIHandle)
{
	(void)pSPIHandle;
}

/**
  * @brief
  * @param
  * @retval
  */
__weak void	SPI_RxCpltCallback(SPI_Handle_t *pSPIHandle)
{
	(void)pSPIHandle;
}

/**
  * @brief
  * @param
  * @retval
  */
__weak void	SPI_ErrorCallback(SPI_Handle_t *pSPIHandle)
{
	(void)pSPIHandle;
}
