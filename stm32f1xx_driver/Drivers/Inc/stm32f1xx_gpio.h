/*
 * stm32f1xx_gpio.h
 *
 *  Created on: Jan 21, 2025
 *      Author: Mongoo
 */

#ifndef INC_STM32F1XX_GPIO_H_
#define INC_STM32F1XX_GPIO_H_

#include "stm32f1xx.h"

/*
 * GPIO Pin Configuration Structure Definition
 */
typedef struct
{
	uint32_t	Pin;		/*!< @GPIO_pins_define */
	uint32_t	Mode;		/*!< @GPIO_mode_define */
	uint32_t	Pull;		/*!< @GPIO_pull_define */
	uint32_t	Speed;		/*!< @GPIO_speed_define */
} GPIO_PinConfig_t;

/*
 * GPIO Pin Handle Structure Definition
 */
typedef struct
{
	GPIO_RegDef_t		*pGPIOx;	/*!< Address of GPIO port to which the pin belongs >*/
	GPIO_PinConfig_t	PinConfig;	/*!< GPIO pin configuration settings >*/
} GPIO_Handle_t;

/*
 * GPIO Bit SET and RESET enumeration
 */
typedef enum
{
	GPIO_PIN_RESET = 0U,
	GPIO_PIN_SET
} GPIO_PinState;

/*
 * @GPIO_pins_define
 */
#define GPIO_PIN_0			0U
#define GPIO_PIN_1			1U
#define GPIO_PIN_2			2U
#define GPIO_PIN_3			3U
#define GPIO_PIN_4			4U
#define GPIO_PIN_5			5U
#define GPIO_PIN_6			6U
#define GPIO_PIN_7			7U
#define GPIO_PIN_8			8U
#define GPIO_PIN_9			9U
#define GPIO_PIN_10			10U
#define GPIO_PIN_11			11U
#define GPIO_PIN_12			12U
#define GPIO_PIN_13			13U
#define GPIO_PIN_14			14U
#define GPIO_PIN_15			15U

/*
 * @GPIO_mode_define
 */
#define GPIO_MODE_INPUT						0U
#define GPIO_MODE_OUTPUT_PP					1U
#define GPIO_MODE_OUTPUT_OD					2U
#define GPIO_MODE_AF_PP						3U
#define GPIO_MODE_AF_OD						4U

#define GPIO_MODE_ANALOG					5U

#define GPIO_MODE_IT_RISING					6U
#define GPIO_MODE_IT_FALLING				7U
#define GPIO_MODE_IT_RISING_FALLING			8U

/*
 * @GPIO_speed_define
 */
#define GPIO_SPEED_FREQ_LOW					2U
#define GPIO_SPEED_FREQ_MEDIUM				1U
#define GPIO_SPEED_FREQ_HIGH				3U

/*
 * @GPIO_pull_define
 */
#define GPIO_NOPULL							0U
#define GPIO_PULLUP							1U
#define GPIO_PULLDOWN						2U

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock Control
 */
void	GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void	GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void	GPIO_DeInit(GPIO_Handle_t *pGPIOHandle);

/*
 * I/O Operations
 */
GPIO_PinState	GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint16_t GPIO_Pin);
void			GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void			GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint16_t GPIO_Pin);

/*
 * IRQ configuration and ISR handling
 */
void	GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void	GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void	GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);

#endif /* INC_STM32F1XX_GPIO_H_ */
