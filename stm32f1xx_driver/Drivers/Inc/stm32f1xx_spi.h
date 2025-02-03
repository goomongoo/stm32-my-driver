/*
 * stm32f1xx_spi.h
 *
 *  Created on: Jan 25, 2025
 *      Author: Mongoo
 */

#ifndef INC_STM32F1XX_SPI_H_
#define INC_STM32F1XX_SPI_H_

#include "stm32f1xx.h"

/*
 * SPI Configuration Structure Definition
 */
typedef struct
{
	uint32_t	Mode;					/*!< @SPI_Mode */
	uint32_t	Direction;				/*!< @SPI_Direction */
	uint32_t	DataSize;				/*!< @SPI_Data_Size */
	uint32_t	CLKPolarity;			/*!< @SPI_Clock_Polarity */
	uint32_t	CLKPhase;				/*!< @SPI_Clock_Phase */
	uint32_t	NSS;					/*!< @SPI_Slave_Select_Management */
	uint32_t	BaudRatePrescaler;		/*!< @SPI_BaudRate_Prescaler */
} SPI_Config_t;

/*
 * SPI Handle Structure Definition
 */
typedef struct
{
	SPI_RegDef_t	*pSPIx;
	SPI_Config_t	Config;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t		TxLen;
	uint32_t		RxLen;
	uint8_t			TxState;
	uint8_t			RxState;
	uint8_t			ErrorState;
} SPI_Handle_t;

/*
 * @SPI_Mode
 * Only Master Mode Implemented
 */
#define SPI_MODE_SLAVE						0x00000000U
#define SPI_MODE_MASTER						(SPI_CR1_MSTR | SPI_CR1_SSI)

/*
 * @SPI_Direction
 */
#define SPI_DIRECTION_2LINES				0x00000000U
#define SPI_DIRECTION_2LINES_RXONLY			SPI_CR1_RXONLY
#define SPI_DIRECTION_1LINE					SPI_CR1_BIDIMODE

/*
 * @SPI_Data_Size
 */
#define SPI_DATASIZE_8BIT					0x00000000U
#define SPI_DATASIZE_16BIT					SPI_CR1_DFF

/*
 * @SPI_Clock_Polarity
 */
#define SPI_POLARITY_LOW					0x00000000U
#define SPI_POLARITY_HIGH					SPI_CR1_CPOL

/*
 * @SPI_Clock_Phase
 */
#define SPI_PHASE_1EDGE						0x00000000U
#define SPI_PHASE_2EDGE						SPI_CR1_CPHA

/*
 * @SPI_Slave_Select_Management
 */
#define SPI_NSS_HARD_INPUT					0x00000000U
#define SPI_NSS_SOFT						SPI_CR1_SSM

/*
 * @SPI_BaudRate_Prescaler
 */
#define SPI_BAUDRATEPRESCALER_2				(0x00000000U)
#define SPI_BAUDRATEPRESCALER_4				(SPI_CR1_BR_0)
#define SPI_BAUDRATEPRESCALER_8				(SPI_CR1_BR_1)
#define SPI_BAUDRATEPRESCALER_16			(SPI_CR1_BR_1 | SPI_CR1_BR_0)
#define SPI_BAUDRATEPRESCALER_32			(SPI_CR1_BR_2)
#define SPI_BAUDRATEPRESCALER_64			(SPI_CR1_BR_2 | SPI_CR1_BR_0)
#define SPI_BAUDRATEPRESCALER_128			(SPI_CR1_BR_2 | SPI_CR1_BR_1)
#define SPI_BAUDRATEPRESCALER_256			(SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0)

/*
 * SPI Flag definitions
 */
#define SPI_FLAG_RXNE						SPI_SR_RXNE
#define SPI_FLAG_TXE						SPI_SR_TXE
#define SPI_FLAG_BUSY						SPI_SR_BSY
#define SPI_FLAG_MODF						SPI_SR_MODF
#define SPI_FLAG_OVR						SPI_SR_OVR

/*
 * Application states
 */
#define SPI_STATE_READY						0U
#define SPI_STATE_BUSY_RX					1U
#define SPI_STATE_BUSY_TX					2U

/*
 * Error states
 */
#define SPI_ERROR_NONE						0U
#define SPI_ERROR_OVR						1U

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock Control
 */
void	SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void	SPI_Init(SPI_Handle_t *pSPIHandle);
void	SPI_DeInit(SPI_Handle_t *pSPIHandle);

/*
 * I/O Operations
 */
void	SPI_Transmit(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
void	SPI_Receive(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t	SPI_Transmit_IT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t	SPI_Receive_IT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ configuration and ISR handling
 */
void	SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void	SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void	SPI_IRQHandler(SPI_Handle_t *pSPIHandle);

/*
 * Other Peripheral Control APIs
 */
void	SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void	SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void	SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t	SPI_GetFlag(SPI_RegDef_t *pSPIx, uint32_t FlagName);

/*
 * Application Callbacks
 */
void	SPI_TxCpltCallback(SPI_Handle_t *pSPIHandle);
void	SPI_RxCpltCallback(SPI_Handle_t *pSPIHandle);
void	SPI_ErrorCallback(SPI_Handle_t *pSPIHandle);

#endif /* INC_STM32F1XX_SPI_H_ */
