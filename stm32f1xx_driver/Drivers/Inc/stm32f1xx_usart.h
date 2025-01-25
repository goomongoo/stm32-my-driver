/*
 * stm32f1xx_usart.h
 *
 *  Created on: Jan 22, 2025
 *      Author: Mongoo
 */

#ifndef INC_STM32F1XX_USART_H_
#define INC_STM32F1XX_USART_H_

#include "stm32f1xx.h"

/*
 * USART Configuration Structure Definition
 */
typedef struct
{
	uint32_t	BaudRate;		/*!< @UART_Baud_Rate */
	uint32_t	WordLength;		/*!< @UART_Word_Length */
	uint32_t	StopBits;		/*!< @UART_Stop_Bits */
	uint32_t	Parity;			/*!< @UART_Parity */
	uint32_t	Mode;			/*!< @UART_Mode */
	uint32_t	HwFlowCtl;		/*!< @UART_Hardware_Flow_Control */
} USART_Config_t;

/*
 * USART Handle Structure Definition
 */
typedef struct
{
	USART_RegDef_t	*pUSARTx;
	USART_Config_t	Config;
	uint8_t			*pTxBuffer;
	uint8_t			*pRxBuffer;
	uint32_t		TxLen;
	uint32_t		RxLen;
	uint8_t			TxBusy;
	uint8_t			RxBusy;
} USART_Handle_t;

/*
 * @UART_Baud_Rate
 */
#define UART_BAUDRATE_1200		1200U
#define UART_BAUDRATE_2400		2400U
#define UART_BAUDRATE_9600		9600U
#define UART_BAUDRATE_19200		19200U
#define UART_BAUDRATE_38400		38400U
#define UART_BAUDRATE_57600		57600U
#define UART_BAUDRATE_115200	115200U
#define UART_BAUDRATE_230400	230400U
#define UART_BAUDRATE_460800	460800U
#define UART_BAUDRATE_921600	921600U
#define UART_BAUDRATE_2M		2000000U
#define UART_BAUDRATE_3M		3000000U

/*
 * @UART_Word_Length
 */
#define UART_WORDLENGTH_8B		0x00000000U
#define UART_WORDLENGTH_9B		((uint32_t)USART_CR1_M)

/*
 * @UART_Stop_Bits
 */
#define UART_STOPBITS_1			0x00000000U
#define UART_STOPBITS_0_5		((uint32_t)USART_CR2_STOP_0)
#define UART_STOPBITS_2			((uint32_t)USART_CR2_STOP_1)
#define UART_STOPBITS_1_5		((uint32_t)(USART_CR2_STOP_0 | USART_CR2_STOP_1))

/*
 * @UART_Parity
 */
#define UART_PARITY_NONE		0x00000000U
#define UART_PARITY_EVEN		((uint32_t)USART_CR1_PCE)
#define UART_PARITY_ODD			((uint32_t)(USART_CR1_PCE | USART_CR1_PS))

/*
 * @UART_Hardware_Flow_Control
 */
#define UART_HWCONTROL_NONE			0x00000000U
#define UART_HWCONTROL_RTS			((uint32_t)USART_CR3_RTSE))
#define UART_HWCONTROL_CTS			((uint32_t)USART_CR3_CTSE))
#define UART_HWCONTROL_RTS_CTS		((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE))

/*
 * @UART_Mode
 */
#define UART_MODE_RX				((uint32_t)USART_CR1_RE)
#define UART_MODE_TX				((uint32_t)USART_CR1_TE)
#define UART_MODE_TX_RX				((uint32_t)(USART_CR1_TE | USART_CR1_RE))

/*
 * UART Flags
 */
#define UART_FLAG_TXE				((uint32_t)USART_SR_TXE)
#define UART_FLAG_RXNE				((uint32_t)USART_SR_RXNE)
#define UART_FLAG_TC				((uint32_t)USART_SR_TC)

/*
 * Application states
 */
#define UART_STATE_READY			0U
#define UART_STATE_BUSY_RX			1U
#define UART_STATE_BUSY_TX			2U

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock Control
 */
void	USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);

/*
 * Init and De-Init
 */
void	USART_Init(USART_Handle_t *pUSARTHandle);
void	USART_DeInit(USART_Handle_t *pUSARTHandle);

/*
 * I/O Operations
 */
void	USART_Transmit(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void	USART_Receive(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);
uint8_t	USART_Transmit_IT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t	USART_Receive_IT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ configuration and ISR handling
 */
void	USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void	USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void	USART_IRQHandler(USART_Handle_t *pUSARTHandle);

/*
 * Other Peripheral Control APIs
 */
uint8_t	USART_GetFlag(USART_RegDef_t *pUSARTx, uint32_t FlagName);
void	USART_ClearFlag(USART_RegDef_t *pUSARTx, uint32_t FlagName);
void	USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);
void	USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);

/*
 * Application Callbacks
 */
void	USART_RxCpltCallback(USART_Handle_t *pUSARTHandle);
void	USART_TxCpltCallback(USART_Handle_t *pUSARTHandle);

#endif /* INC_STM32F1XX_USART_H_ */
