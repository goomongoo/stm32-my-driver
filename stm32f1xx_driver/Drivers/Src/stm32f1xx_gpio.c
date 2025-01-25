/*
 * stm32f1xx_gpio.c
 *
 *  Created on: Jan 21, 2025
 *      Author: Mongoo
 */

#include "stm32f1xx_gpio.h"

/* Definitions for bit manipulation of CRL and CRH register */
#define GPIO_CR_MODE_INPUT			0x00000000U
#define GPIO_CR_CNF_ANALOG			0x00000000U
#define GPIO_CR_CNF_INPUT_FLOATING	0x00000004U
#define GPIO_CR_CNF_INPUT_PUPD		0x00000008U
#define GPIO_CR_CNF_GP_OUTPUT_PP	0x00000000U
#define GPIO_CR_CNF_GP_OUTPUT_OD	0x00000004U
#define GPIO_CR_CNF_AF_OUTPUT_PP	0x00000008U
#define GPIO_CR_CNF_AF_OUTPUT_OD	0x0000000CU

/**
  * @brief  Enables or disables the peripheral clock for the specified GPIO port.
  * @param  pGPIOx : Pointer to the GPIO port base address (e.g., GPIOA, GPIOB, etc.).
  * @param  EnorDi : ENABLE to enable the clock, DISABLE to disable the clock.
  * @retval None
  */
void	GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_CLOCK_ENABLE();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_CLOCK_ENABLE();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_CLOCK_ENABLE();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_CLOCK_ENABLE();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_CLOCK_ENABLE();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_CLOCK_DISABLE();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_CLOCK_DISABLE();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_CLOCK_DISABLE();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_CLOCK_DISABLE();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_CLOCK_DISABLE();
		}
	}
}

/**
  * @brief  Initializes the GPIO pin based on the configuration provided in GPIO_Handle_t.
  * @param  pGPIOHandle : Pointer to GPIO_Handle_t structure that contains
  *                       the configuration information for the specified GPIO pin.
  * @retval None
  */
void	GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	__IO uint32_t	*pConfigRegister;
	uint32_t		config = 0U;
	uint32_t		registerOffset;

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	/* Get address and offset of GPIO_CR register determined by GPIO pin */
	pConfigRegister = (pGPIOHandle->PinConfig.Pin < 8) ? &(pGPIOHandle->pGPIOx->CRL) : &(pGPIOHandle->pGPIOx->CRH);
	registerOffset = (pGPIOHandle->PinConfig.Pin % 8) * 4;

	/* Clear CNF and MODE field */
	*pConfigRegister &= ~((GPIO_CRL_MODE0 | GPIO_CRL_CNF0) << registerOffset);

	switch (pGPIOHandle->PinConfig.Mode)
	{
		/* Output Mode */
		/* Output Push Pull */
		case GPIO_MODE_OUTPUT_PP:
			config = pGPIOHandle->PinConfig.Speed + GPIO_CR_CNF_GP_OUTPUT_PP;
			break;

		/* Output Open Drain */
		case GPIO_MODE_OUTPUT_OD:
			config = pGPIOHandle->PinConfig.Speed + GPIO_CR_CNF_GP_OUTPUT_OD;
			break;

		/* Alternative Function Output Push Pull */
		case GPIO_MODE_AF_PP:
			config = pGPIOHandle->PinConfig.Speed + GPIO_CR_CNF_AF_OUTPUT_PP;
			break;

		/* Alternative Function Output Open Drain */
		case GPIO_MODE_AF_OD:
			config = pGPIOHandle->PinConfig.Speed + GPIO_CR_CNF_AF_OUTPUT_OD;
			break;

		/* Input Mode */
		case GPIO_MODE_INPUT:
		case GPIO_MODE_IT_FALLING:
		case GPIO_MODE_IT_RISING:
		case GPIO_MODE_IT_RISING_FALLING:
			/* Input Pull-up */
			if (pGPIOHandle->PinConfig.Pull == GPIO_PULLUP)
			{
				config = GPIO_CR_MODE_INPUT + GPIO_CR_CNF_INPUT_PUPD;
				pGPIOHandle->pGPIOx->BSRR |= (1 << registerOffset);
			}
			/* Input Pull-down */
			else if (pGPIOHandle->PinConfig.Pull == GPIO_PULLDOWN)
			{
				config = GPIO_CR_MODE_INPUT + GPIO_CR_CNF_INPUT_PUPD;
				pGPIOHandle->pGPIOx->BRR |= (1 << registerOffset);
			}
			/* Input No pull (Floating) */
			else
			{
				config = GPIO_CR_MODE_INPUT + GPIO_CR_CNF_INPUT_FLOATING;
			}
			break;

		/* Input Analog */
		case GPIO_MODE_ANALOG:
			config = GPIO_CR_MODE_INPUT + GPIO_CR_CNF_ANALOG;
			break;

		default:
			break;
	}

	/* Apply new configuration to register */
	*pConfigRegister |= ((config << registerOffset) & ((GPIO_CRL_MODE0 | GPIO_CRL_CNF0) << registerOffset));

	/* EXTI Mode Configuration */
	if (pGPIOHandle->PinConfig.Mode >= GPIO_MODE_IT_RISING)
	{
		/* Rising edge trigger selection */
		if (pGPIOHandle->PinConfig.Mode == GPIO_MODE_IT_RISING)
		{
			EXTI->RTSR |= (1 << pGPIOHandle->PinConfig.Pin);
			EXTI->FTSR &= ~(1 << pGPIOHandle->PinConfig.Pin);
		}
		/* Falling edge trigger selection */
		else if (pGPIOHandle->PinConfig.Mode == GPIO_MODE_IT_FALLING)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->PinConfig.Pin);
			EXTI->RTSR &= ~(1 << pGPIOHandle->PinConfig.Pin);
		}
		/* Rising/Falling edge trigger selection */
		else if (pGPIOHandle->PinConfig.Mode == GPIO_MODE_IT_RISING_FALLING)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->PinConfig.Pin);
			EXTI->RTSR |= (1 << pGPIOHandle->PinConfig.Pin);
		}

		/* Enable clock for AFIO peripheral */
		AFIO_CLOCK_ENABLE();

		/* Get address and offset of AFIO_EXTICR register determined by GPIO pin */
		pConfigRegister = &(AFIO->EXTICR[(uint32_t)(pGPIOHandle->PinConfig.Pin / 4)]);
		registerOffset = (pGPIOHandle->PinConfig.Pin % 4) * 4;

		/* Apply EXTI configuration on AFIO_EXTICR register */
		config = (uint32_t)(GPIO_PORTCODE(pGPIOHandle->pGPIOx));
		*pConfigRegister |= ((config << registerOffset) & (AFIO_EXTICR1_EXTI0 << registerOffset));

		/* Enable interrupt request */
		EXTI->IMR |= (1 << pGPIOHandle->PinConfig.Pin);
	}
}

/**
  * @brief  Deinitializes the GPIO pin and resets it to its default state.
  * @param  pGPIOHandle : Pointer to GPIO_Handle_t structure that contains
  *                       the configuration information for the specified GPIO pin.
  * @retval None
  */
void	GPIO_DeInit(GPIO_Handle_t *pGPIOHandle)
{
	__IO uint32_t	*pConfigRegister;
	uint32_t		registerOffset;

	/* EXTI Mode Configuration */
	if (pGPIOHandle->PinConfig.Mode >= GPIO_MODE_IT_RISING)
	{
		/* Disable interrupt request */
		EXTI->IMR &= ~(1 << pGPIOHandle->PinConfig.Pin);

		/* Clear rising/falling edge trigger selection configuration */
		EXTI->FTSR &= ~(1 << pGPIOHandle->PinConfig.Pin);
		EXTI->RTSR &= ~(1 << pGPIOHandle->PinConfig.Pin);

		/* Get address and offset of AFIO_EXTICR register determined by GPIO pin */
		pConfigRegister = &(AFIO->EXTICR[(uint32_t)(pGPIOHandle->PinConfig.Pin / 4)]);
		registerOffset = (pGPIOHandle->PinConfig.Pin % 4) * 4;

		/* Clear EXTI configuration on AFIO_EXTICR register */
		*pConfigRegister &= ~(AFIO_EXTICR1_EXTI0 << registerOffset);
	}

	/* GPIO Mode Configuration */
	/* Get address and offset of GPIO_CR register determined by GPIO pin */
	pConfigRegister = (pGPIOHandle->PinConfig.Pin < 8) ? &(pGPIOHandle->pGPIOx->CRL) : &(pGPIOHandle->pGPIOx->CRH);
	registerOffset = (pGPIOHandle->PinConfig.Pin % 8) * 4;

	/* Clear CNF and MODE field */
	*pConfigRegister &= ~((GPIO_CRL_MODE0 | GPIO_CRL_CNF0) << registerOffset);

	/* Default configuration is Input Floating */
	*pConfigRegister |= ((GPIO_CR_MODE_INPUT + GPIO_CR_CNF_INPUT_FLOATING) << registerOffset);

	/* Default ODR value is zero */
	pGPIOHandle->pGPIOx->ODR &= ~(1 << pGPIOHandle->PinConfig.Pin);
}

/**
  * @brief  Reads the state of the specified GPIO pin.
  * @param  pGPIOx : Pointer to the GPIO port base address (e.g., GPIOA, GPIOB, etc.).
  * @param  GPIO_Pin : GPIO pin number to read (0-15 for 16-bit ports).
  * @retval GPIO_PinState : The state of the pin (GPIO_PIN_SET or GPIO_PIN_RESET).
  */
GPIO_PinState	GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint16_t GPIO_Pin)
{
	GPIO_PinState	status;

	if ((pGPIOx->IDR & (1 << GPIO_Pin)) != GPIO_PIN_RESET)
	{
		status = GPIO_PIN_SET;
	}
	else
	{
		status = GPIO_PIN_RESET;
	}

	return status;
}

/**
  * @brief  Writes the specified state to the selected GPIO pin.
  * @param  pGPIOx : Pointer to the GPIO port base address (e.g., GPIOA, GPIOB, etc.).
  * @param  GPIO_Pin : GPIO pin number to write (0-15 for 16-bit ports).
  * @param  PinState : State to write to the pin (GPIO_PIN_SET or GPIO_PIN_RESET).
  * @retval None
  */
void	GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)
{
	if (PinState != GPIO_PIN_RESET)
	{
		pGPIOx->BSRR |= (1 << GPIO_Pin);
	}
	else
	{
		pGPIOx->BSRR |= (1 << (GPIO_Pin + 16U));
	}
}

/**
  * @brief  Toggles the state of the specified GPIO pin.
  * @param  pGPIOx : Pointer to the GPIO port base address (e.g., GPIOA, GPIOB, etc.).
  * @param  GPIO_Pin : GPIO pin number to toggle (0-15 for 16-bit ports).
  * @retval None
  */
void	GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint16_t GPIO_Pin)
{
	if ((pGPIOx->ODR & (1 << GPIO_Pin)) != GPIO_PIN_RESET)
	{
		pGPIOx->BSRR |= (1 << (GPIO_Pin + 16U));
	}
	else
	{
		pGPIOx->BSRR |= (1 << GPIO_Pin);
	}
}

/**
  * @brief  Configures the NVIC interrupt for a specified IRQ number.
  * @param  IRQNumber : Interrupt request (IRQ) number to be configured.
  * @param  EnorDi : ENABLE to enable the interrupt, DISABLE to disable it.
  * @retval None
  */
void	GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
void	GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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

/**
  * @brief  Handles the interrupt request for the specified GPIO pin.
  * @param  GPIO_Pin : GPIO pin number that triggered the interrupt.
  * @retval None
  */
void	GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin)
{
	if (EXTI->PR & (1 << GPIO_Pin))
	{
		/* Clear pending bit */
		EXTI->PR |= (1 << GPIO_Pin);
	}
}
