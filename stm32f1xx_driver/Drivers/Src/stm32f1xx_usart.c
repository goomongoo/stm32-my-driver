/*
 * stm32f1xx_usart.c
 *
 *  Created on: Jan 22, 2025
 *      Author: Mongoo
 */

#include "stm32f1xx_usart.h"

#define PCLK1	8000000U
#define PCLK2	PCLK1

/**
  * @brief
  * @param
  * @retval
  */
void	USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pUSARTx == USART1)
		{
			USART1_CLOCK_ENABLE();
		}
		else if (pUSARTx == USART2)
		{
			USART2_CLOCK_ENABLE();
		}
		else if (pUSARTx == USART3)
		{
			USART3_CLOCK_ENABLE();
		}
	}
	else
	{
		if (pUSARTx == USART1)
		{
			USART1_CLOCK_DISABLE();
		}
		else if (pUSARTx == USART2)
		{
			USART2_CLOCK_DISABLE();
		}
		else if (pUSARTx == USART3)
		{
			USART3_CLOCK_DISABLE();
		}
	}
}

/**
  * @brief
  * @param
  * @retval
  */
void	USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pUSARTx == USART1)
		{
			USART1_PERI_ENABLE();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PERI_ENABLE();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PERI_ENABLE();
		}
	}
	else
	{
		if (pUSARTx == USART1)
		{
			USART1_PERI_DISABLE();
		}
		else if (pUSARTx == USART2)
		{
			USART2_PERI_DISABLE();
		}
		else if (pUSARTx == USART3)
		{
			USART3_PERI_DISABLE();
		}
	}
}

/**
  * @brief
  * @param
  * @retval
  */
void	USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	uint32_t	clk;
	uint32_t	uartDiv;
	uint32_t	mantissa;
	uint32_t	fraction;

	/* Bus clock */
	clk = (pUSARTx == USART1) ? PCLK1 : PCLK2;

	/* Over sampling by 16 */
	uartDiv = (25U * clk) / (4U * BaudRate);

	/* Get mantissa */
	mantissa = uartDiv / 100U;

	/* Get fraction */
	fraction = ((uartDiv - (mantissa * 100U)) * 16U + 50U) / 100U;

	/* Configure BRR */
	/* BRR = mantissa + overflow + fraction */
	pUSARTx->BRR = (mantissa << 4U) + (fraction & 0xF0U) + (fraction & 0x0FU);
}

/**
  * @brief
  * @param
  * @retval
  */
void	USART_Init(USART_Handle_t *pUSARTHandle)
{
	uint32_t	config;

	/* USART Peripheral Clock Enable */
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	/* ----- CR1 Register Configuration ----- */
	config = 0U;

	/* UART Mode Configuration */
	config |= pUSARTHandle->Config.Mode;

	/* UART Parity Configuration */
	config |= pUSARTHandle->Config.Parity;

	/* UART Word Length Configuration */
	config |= pUSARTHandle->Config.WordLength;

	/* Apply configuration to CR1 Register */
	pUSARTHandle->pUSARTx->CR1 = config;

	/* ----- CR2 Register Configuration ----- */
	config = 0U;

	/* UART Stop Bits Configuration */
	config |= pUSARTHandle->Config.StopBits;

	/* Apply configuration to CR2 Register */
	pUSARTHandle->pUSARTx->CR2 = config;

	/* ----- CR3 Register Configuration ----- */
	config = 0U;

	/* UART Hardware Flow Control Configuration */
	config |= pUSARTHandle->Config.HwFlowCtl;

	/* Apply configuration to CR3 Register */
	pUSARTHandle->pUSARTx->CR3 = config;

	/* ----- BRR Register Configuration ----- */
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->Config.BaudRate);

	/* USART Peripheral Enable */
	USART_PeripheralControl(pUSARTHandle->pUSARTx, ENABLE);
}

/**
  * @brief
  * @param
  * @retval
  */
void	USART_DeInit(USART_Handle_t *pUSARTHandle)
{
	/* USART Peripheral Disable */
	USART_PeripheralControl(pUSARTHandle->pUSARTx, DISABLE);
}

/**
  * @brief
  * @param
  * @retval
  */
uint8_t	USART_GetFlag(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	uint8_t	status;

	if (pUSARTx->SR & FlagName)
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
void	USART_ClearFlag(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	pUSARTx->SR &= ~(FlagName);
}

/**
  * @brief
  * @param
  * @retval
  */
void	USART_Transmit(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t	*pTxBuffer16 = NULL;

	/* Use 16-bit buffer if 9-bit word length and parity disabled */
	if ((pUSARTHandle->Config.WordLength == UART_WORDLENGTH_9B) && (pUSARTHandle->Config.Parity == UART_PARITY_NONE))
	{
		pTxBuffer16 = (uint16_t *)pTxBuffer;
	}

	while (Len > 0U)
	{
		/* Wait until TXE flag is set */
		while (USART_GetFlag(pUSARTHandle->pUSARTx, UART_FLAG_TXE) == FLAG_RESET);

		/* 9-bit word length and parity disabled */
		if (pTxBuffer16 != NULL)
		{
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer16 & (0x01FFU));
			pTxBuffer16++;
		}
		/* 9-bit word length and parity enabled / 8-bit word length */
		else
		{
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (0xFFU));
			pTxBuffer++;
		}

		/* Decrement Len by 1 */
		Len--;
	}

	/* Wait until TC flag is set */
	while (USART_GetFlag(pUSARTHandle->pUSARTx, UART_FLAG_TC) == FLAG_RESET);
}

/**
  * @brief
  * @param
  * @retval
  */
void	USART_Receive(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint16_t	*pRxBuffer16 = NULL;

	/* Use 16-bit buffer if 9-bit word length and parity disabled */
	if ((pUSARTHandle->Config.WordLength == UART_WORDLENGTH_9B) && (pUSARTHandle->Config.Parity == UART_PARITY_NONE))
	{
		pRxBuffer16 = (uint16_t *)pRxBuffer;
	}

	while (Len > 0U)
	{
		/* Wait until RXNE flag is set */
		while (USART_GetFlag(pUSARTHandle->pUSARTx, UART_FLAG_RXNE) == FLAG_RESET);

		/* 9-bit word length and parity disabled */
		if (pRxBuffer16 != NULL)
		{
			*pRxBuffer16 = (uint16_t)(pUSARTHandle->pUSARTx->DR & (uint16_t)(0x01FFU));
			pRxBuffer16++;
		}
		else
		{
			/* 9-bit word length and parity enabled / 8-bit word length and parity disabled */
			if ((pUSARTHandle->Config.WordLength == UART_WORDLENGTH_9B) || ((pUSARTHandle->Config.WordLength == UART_WORDLENGTH_8B) && (pUSARTHandle->Config.Parity == UART_PARITY_NONE)))
			{
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)(0xFFU));
			}
			/* 8-bit word length and parity enabled */
			else
			{
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)(0x7FU));
			}
			pRxBuffer++;
		}

		/* Decrement Len by 1 */
		Len--;
	}
}

/**
  * @brief
  * @param
  * @retval
  */
uint8_t	USART_Transmit_IT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t	state = pUSARTHandle->TxBusy;

	if (state != UART_STATE_BUSY_TX)
	{
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->TxBusy = UART_STATE_BUSY_TX;

		/* Enable TXE interrupt */
		pUSARTHandle->pUSARTx->CR1 |= USART_CR1_TXEIE;
	}

	return state;
}

/**
  * @brief
  * @param
  * @retval
  */
uint8_t	USART_Receive_IT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t	state = pUSARTHandle->RxBusy;

	if (state != UART_STATE_BUSY_RX)
	{
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->RxLen = Len;
		pUSARTHandle->RxBusy = UART_STATE_BUSY_RX;

		/* Enable RXNE interrupt */
		pUSARTHandle->pUSARTx->CR1 |= USART_CR1_RXNEIE;
	}

	return state;
}

/**
  * @brief  Configures the NVIC interrupt for a specified IRQ number.
  * @param  IRQNumber : Interrupt request (IRQ) number to be configured.
  * @param  EnorDi : ENABLE to enable the interrupt, DISABLE to disable it.
  * @retval None
  */
void	USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
void	USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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
static void	_USART_Transmit_IT(USART_Handle_t *pUSARTHandle);
static void	_USART_EndTransmit_IT(USART_Handle_t *pUSARTHandle);
static void	_USART_Receive_IT(USART_Handle_t *pUSARTHandle);

/**
  * @brief
  * @param
  * @retval
  */
void	USART_IRQHandler(USART_Handle_t *pUSARTHandle)
{
	/* TC interrupt */
	if ((pUSARTHandle->pUSARTx->SR & USART_SR_TC) && (pUSARTHandle->pUSARTx->CR1 & USART_CR1_TCIE))
	{
		_USART_EndTransmit_IT(pUSARTHandle);
		return;
	}

	/* TXE interrupt */
	if ((pUSARTHandle->pUSARTx->SR & USART_SR_TXE) && (pUSARTHandle->pUSARTx->CR1 & USART_CR1_TXEIE))
	{
		_USART_Transmit_IT(pUSARTHandle);
		return;
	}

	/* RXNE interrupt */
	if ((pUSARTHandle->pUSARTx->SR & USART_SR_RXNE) && (pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE))
	{
		_USART_Receive_IT(pUSARTHandle);
		return;
	}
}


/**
  * @brief
  * @param
  * @retval
  */
static void	_USART_Transmit_IT(USART_Handle_t *pUSARTHandle)
{
	uint16_t	*temp;

	if (pUSARTHandle->TxBusy != UART_STATE_BUSY_TX)
	{
		return;
	}

	if (pUSARTHandle->TxLen > 0U)
	{
		/* 9-bit word length and parity disabled */
		if ((pUSARTHandle->Config.WordLength == UART_WORDLENGTH_9B) && (pUSARTHandle->Config.Parity == UART_PARITY_NONE))
		{
			temp = (uint16_t *)(pUSARTHandle->pTxBuffer);
			pUSARTHandle->pUSARTx->DR = (uint16_t)(*temp & (uint16_t)(0x01FFU));
			pUSARTHandle->pTxBuffer += 2U;
		}
		/* 9-bit word length and parity enabled / 8-bit word length */
		else
		{
			pUSARTHandle->pUSARTx->DR = (uint8_t)(*pUSARTHandle->pTxBuffer & (uint8_t)(0xFFU));
			pUSARTHandle->pTxBuffer++;
		}

		if (--pUSARTHandle->TxLen == 0U)
		{
			/* Disable TXE interrupt */
			pUSARTHandle->pUSARTx->CR1 &= ~USART_CR1_TXEIE;

			/* Enable TC interrupt */
			pUSARTHandle->pUSARTx->CR1 |= USART_CR1_TCIE;
		}
	}
}

/**
  * @brief
  * @param
  * @retval
  */
static void	_USART_EndTransmit_IT(USART_Handle_t *pUSARTHandle)
{
	/* Disable TC interrupt */
	pUSARTHandle->pUSARTx->CR1 &= ~USART_CR1_TCIE;

	/* Reset state to READY */
	pUSARTHandle->TxBusy = UART_STATE_READY;

	/* Application callback of TX complete */
	USART_TxCpltCallback(pUSARTHandle);
}

/**
  * @brief
  * @param
  * @retval
  */
static void	_USART_Receive_IT(USART_Handle_t *pUSARTHandle)
{
	uint16_t	*temp;

	if (pUSARTHandle->RxBusy != UART_STATE_BUSY_RX)
	{
		return;
	}

	if (pUSARTHandle->RxLen > 0U)
	{
		/* 9-bit word length and parity disabled */
		if ((pUSARTHandle->Config.WordLength == UART_WORDLENGTH_9B) && (pUSARTHandle->Config.Parity == UART_PARITY_NONE))
		{
			temp = (uint16_t *)(pUSARTHandle->pRxBuffer);
			*temp = (uint16_t)(pUSARTHandle->pUSARTx->DR & (uint16_t)(0x01FFU));
			pUSARTHandle->pRxBuffer += 2U;
		}
		else
		{
			/* 9-bit word length and parity enabled / 8-bit word length and parity disabled */
			if ((pUSARTHandle->Config.WordLength == UART_WORDLENGTH_9B) || ((pUSARTHandle->Config.WordLength == UART_WORDLENGTH_8B) && (pUSARTHandle->Config.Parity == UART_PARITY_NONE)))
			{
				*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)(0xFFU));
			}
			/* 8-bit word length and parity enabled */
			else
			{
				*pUSARTHandle->pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)(0x7FU));
			}
			pUSARTHandle->pRxBuffer++;
		}

		if (--pUSARTHandle->RxLen == 0U)
		{
			/* Disable RXNE interrupt */
			pUSARTHandle->pUSARTx->CR1 &= ~USART_CR1_RXNEIE;

			/* Reset state to READY */
			pUSARTHandle->RxBusy = UART_STATE_READY;

			/* Application callback of RX complete */
			USART_RxCpltCallback(pUSARTHandle);
		}
	}
}

/**
  * @brief
  * @param
  * @retval
  */
__weak void	USART_RxCpltCallback(USART_Handle_t *pUSARTHandle)
{
	(void)pUSARTHandle;
}

/**
  * @brief
  * @param
  * @retval
  */
__weak void	USART_TxCpltCallback(USART_Handle_t *pUSARTHandle)
{
	(void)pUSARTHandle;
}
