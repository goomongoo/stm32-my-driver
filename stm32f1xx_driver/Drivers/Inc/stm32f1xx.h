/*
 * stm32f1xx.h
 *
 *  Created on: Jan 21, 2025
 *      Author: Mongoo
 */

#ifndef INC_STM32F1XX_H_
#define INC_STM32F1XX_H_

#include <stdint.h>
#include <stddef.h>

#define __IO	volatile
#define __weak	__attribute__((weak))

/*
 * Peripheral Registers Structures
 */

/*
 * External Interrupt/Event Controller
 */
typedef struct
{
	__IO uint32_t	IMR;
	__IO uint32_t	EMR;
	__IO uint32_t	RTSR;
	__IO uint32_t	FTSR;
	__IO uint32_t	SWIER;
	__IO uint32_t	PR;
} EXTI_RegDef_t;

/*
 * General Purpose I/O
 */
typedef struct
{
	__IO uint32_t	CRL;
	__IO uint32_t	CRH;
	__IO uint32_t	IDR;
	__IO uint32_t	ODR;
	__IO uint32_t	BSRR;
	__IO uint32_t	BRR;
	__IO uint32_t	LCKR;
} GPIO_RegDef_t;

/*
 * Alternative Function I/O
 */
typedef struct
{
	__IO uint32_t	EVCR;
	__IO uint32_t	MAPR;
	__IO uint32_t	EXTICR[4];
	uint32_t		RESERVED0;
	__IO uint32_t	MAPR2;
} AFIO_RegDef_t;

/*
 * Inter-Integrated Circuit Interface
 */
typedef struct
{
	__IO uint32_t	CR1;
	__IO uint32_t	CR2;
	__IO uint32_t	OAR1;
	__IO uint32_t	OAR2;
	__IO uint32_t	DR;
	__IO uint32_t	SR1;
	__IO uint32_t	SR2;
	__IO uint32_t	CCR;
	__IO uint32_t	TRISE;
} I2C_RegDef_t;

/*
 * Reset and Clock Control
 */
typedef struct
{
	__IO uint32_t	CR;
	__IO uint32_t	CFGR;
	__IO uint32_t	CIR;
	__IO uint32_t	APB2RSTR;
	__IO uint32_t	APB1RSTR;
	__IO uint32_t	AHBENR;
	__IO uint32_t	APB2ENR;
	__IO uint32_t	APB1ENR;
	__IO uint32_t	BDCR;
	__IO uint32_t	CSR;
} RCC_RegDef_t;

/*
 * Serial Peripheral Interface
 */
typedef struct
{
	__IO uint32_t	CR1;
	__IO uint32_t	CR2;
	__IO uint32_t	SR;
	__IO uint32_t	DR;
	__IO uint32_t	CRCPR;
	__IO uint32_t	RXCRCR;
	__IO uint32_t	TXCRCR;
	__IO uint32_t	I2SCFGR;
} SPI_RegDef_t;

/*
 * Universal Synchronous Asynchronous Receiver Transmitter
 */
typedef struct
{
	__IO uint32_t	SR;
	__IO uint32_t	DR;
	__IO uint32_t	BRR;
	__IO uint32_t	CR1;
	__IO uint32_t	CR2;
	__IO uint32_t	CR3;
	__IO uint32_t	GTPR;
} USART_RegDef_t;

/*
 * Peripheral Memory Map
 */
#define FLASH_BASE			0x08000000UL /*!< FLASH base address in the alias region */
#define SRAM_BASE			0x20000000UL /*!< SRAM base address in the alias region */
#define PERIPH_BASE			0x40000000UL /*!< Peripheral base address in the alias region */

#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		(PERIPH_BASE + 0x00010000UL)
#define AHBPERIPH_BASE		(PERIPH_BASE + 0x00020000UL)

#define SPI2_BASE			(APB1PERIPH_BASE + 0x00003800UL)
#define USART2_BASE			(APB1PERIPH_BASE + 0x00004400UL)
#define USART3_BASE			(APB1PERIPH_BASE + 0x00004800UL)
#define I2C1_BASE			(APB1PERIPH_BASE + 0x00005400UL)
#define I2C2_BASE			(APB1PERIPH_BASE + 0x00005800UL)
#define AFIO_BASE			(APB2PERIPH_BASE + 0x00000000UL)
#define EXTI_BASE			(APB2PERIPH_BASE + 0x00000400UL)
#define GPIOA_BASE			(APB2PERIPH_BASE + 0x00000800UL)
#define GPIOB_BASE			(APB2PERIPH_BASE + 0x00000C00UL)
#define GPIOC_BASE			(APB2PERIPH_BASE + 0x00001000UL)
#define GPIOD_BASE			(APB2PERIPH_BASE + 0x00001400UL)
#define GPIOE_BASE			(APB2PERIPH_BASE + 0x00001800UL)
#define SPI1_BASE			(APB2PERIPH_BASE + 0x00003000UL)
#define USART1_BASE			(APB2PERIPH_BASE + 0x00003800UL)

#define RCC_BASE			(AHBPERIPH_BASE + 0x00001000UL)

/*
 * Peripheral Declaration
 */
#define SPI2				((SPI_RegDef_t *)SPI2_BASE)
#define USART2				((USART_RegDef_t *)USART2_BASE)
#define USART3				((USART_RegDef_t *)USART3_BASE)
#define I2C1				((I2C_RegDef_t *)I2C1_BASE)
#define I2C2				((I2C_RegDef_t *)I2C2_BASE)
#define AFIO				((AFIO_RegDef_t *)AFIO_BASE)
#define EXTI				((EXTI_RegDef_t *)EXTI_BASE)
#define GPIOA				((GPIO_RegDef_t *)GPIOA_BASE)
#define GPIOB				((GPIO_RegDef_t *)GPIOB_BASE)
#define GPIOC				((GPIO_RegDef_t *)GPIOC_BASE)
#define GPIOD				((GPIO_RegDef_t *)GPIOD_BASE)
#define GPIOE				((GPIO_RegDef_t *)GPIOE_BASE)
#define SPI1				((SPI_RegDef_t *)SPI1_BASE)
#define USART1				((USART_RegDef_t *)USART1_BASE)
#define RCC					((RCC_RegDef_t *)RCC_BASE)

/***********Processor specific details***********/
/*
 * ARM Cortex-M3 Processor NVIC ISERx register Addresses
 */
#define NVIC_ISER0				(__IO uint32_t *)(0xE000E100UL)
#define NVIC_ISER1				(__IO uint32_t *)(0xE000E104UL)

/*
 * ARM Cortex-M3 Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0				(__IO uint32_t *)(0xE000E180UL)
#define NVIC_ICER1				(__IO uint32_t *)(0xE000E184UL)

/*
 * ARM Cortex-M3 Processor NVIC IPR register base address
 */
#define NVIC_IPR_BASE			(__IO uint32_t *)(0xE000E400UL)

#define NVIC_PRIO_BITS			4U

/*
 * Peripheral Registers Bit Definition
 */

/******************************************************************************/
/*                                                                            */
/*                General Purpose and Alternate Function I/O                  */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for GPIO_CRL register  *******************/
#define GPIO_CRL_MODE_Pos                    (0U)
#define GPIO_CRL_MODE_Msk                    (0x33333333UL << GPIO_CRL_MODE_Pos) /*!< 0x33333333 */
#define GPIO_CRL_MODE                        GPIO_CRL_MODE_Msk                 /*!< Port x mode bits */

#define GPIO_CRL_MODE0_Pos                   (0U)
#define GPIO_CRL_MODE0_Msk                   (0x3UL << GPIO_CRL_MODE0_Pos)      /*!< 0x00000003 */
#define GPIO_CRL_MODE0                       GPIO_CRL_MODE0_Msk                /*!< MODE0[1:0] bits (Port x mode bits, pin 0) */
#define GPIO_CRL_MODE0_0                     (0x1UL << GPIO_CRL_MODE0_Pos)      /*!< 0x00000001 */
#define GPIO_CRL_MODE0_1                     (0x2UL << GPIO_CRL_MODE0_Pos)      /*!< 0x00000002 */

#define GPIO_CRL_MODE1_Pos                   (4U)
#define GPIO_CRL_MODE1_Msk                   (0x3UL << GPIO_CRL_MODE1_Pos)      /*!< 0x00000030 */
#define GPIO_CRL_MODE1                       GPIO_CRL_MODE1_Msk                /*!< MODE1[1:0] bits (Port x mode bits, pin 1) */
#define GPIO_CRL_MODE1_0                     (0x1UL << GPIO_CRL_MODE1_Pos)      /*!< 0x00000010 */
#define GPIO_CRL_MODE1_1                     (0x2UL << GPIO_CRL_MODE1_Pos)      /*!< 0x00000020 */

#define GPIO_CRL_MODE2_Pos                   (8U)
#define GPIO_CRL_MODE2_Msk                   (0x3UL << GPIO_CRL_MODE2_Pos)      /*!< 0x00000300 */
#define GPIO_CRL_MODE2                       GPIO_CRL_MODE2_Msk                /*!< MODE2[1:0] bits (Port x mode bits, pin 2) */
#define GPIO_CRL_MODE2_0                     (0x1UL << GPIO_CRL_MODE2_Pos)      /*!< 0x00000100 */
#define GPIO_CRL_MODE2_1                     (0x2UL << GPIO_CRL_MODE2_Pos)      /*!< 0x00000200 */

#define GPIO_CRL_MODE3_Pos                   (12U)
#define GPIO_CRL_MODE3_Msk                   (0x3UL << GPIO_CRL_MODE3_Pos)      /*!< 0x00003000 */
#define GPIO_CRL_MODE3                       GPIO_CRL_MODE3_Msk                /*!< MODE3[1:0] bits (Port x mode bits, pin 3) */
#define GPIO_CRL_MODE3_0                     (0x1UL << GPIO_CRL_MODE3_Pos)      /*!< 0x00001000 */
#define GPIO_CRL_MODE3_1                     (0x2UL << GPIO_CRL_MODE3_Pos)      /*!< 0x00002000 */

#define GPIO_CRL_MODE4_Pos                   (16U)
#define GPIO_CRL_MODE4_Msk                   (0x3UL << GPIO_CRL_MODE4_Pos)      /*!< 0x00030000 */
#define GPIO_CRL_MODE4                       GPIO_CRL_MODE4_Msk                /*!< MODE4[1:0] bits (Port x mode bits, pin 4) */
#define GPIO_CRL_MODE4_0                     (0x1UL << GPIO_CRL_MODE4_Pos)      /*!< 0x00010000 */
#define GPIO_CRL_MODE4_1                     (0x2UL << GPIO_CRL_MODE4_Pos)      /*!< 0x00020000 */

#define GPIO_CRL_MODE5_Pos                   (20U)
#define GPIO_CRL_MODE5_Msk                   (0x3UL << GPIO_CRL_MODE5_Pos)      /*!< 0x00300000 */
#define GPIO_CRL_MODE5                       GPIO_CRL_MODE5_Msk                /*!< MODE5[1:0] bits (Port x mode bits, pin 5) */
#define GPIO_CRL_MODE5_0                     (0x1UL << GPIO_CRL_MODE5_Pos)      /*!< 0x00100000 */
#define GPIO_CRL_MODE5_1                     (0x2UL << GPIO_CRL_MODE5_Pos)      /*!< 0x00200000 */

#define GPIO_CRL_MODE6_Pos                   (24U)
#define GPIO_CRL_MODE6_Msk                   (0x3UL << GPIO_CRL_MODE6_Pos)      /*!< 0x03000000 */
#define GPIO_CRL_MODE6                       GPIO_CRL_MODE6_Msk                /*!< MODE6[1:0] bits (Port x mode bits, pin 6) */
#define GPIO_CRL_MODE6_0                     (0x1UL << GPIO_CRL_MODE6_Pos)      /*!< 0x01000000 */
#define GPIO_CRL_MODE6_1                     (0x2UL << GPIO_CRL_MODE6_Pos)      /*!< 0x02000000 */

#define GPIO_CRL_MODE7_Pos                   (28U)
#define GPIO_CRL_MODE7_Msk                   (0x3UL << GPIO_CRL_MODE7_Pos)      /*!< 0x30000000 */
#define GPIO_CRL_MODE7                       GPIO_CRL_MODE7_Msk                /*!< MODE7[1:0] bits (Port x mode bits, pin 7) */
#define GPIO_CRL_MODE7_0                     (0x1UL << GPIO_CRL_MODE7_Pos)      /*!< 0x10000000 */
#define GPIO_CRL_MODE7_1                     (0x2UL << GPIO_CRL_MODE7_Pos)      /*!< 0x20000000 */

#define GPIO_CRL_CNF_Pos                     (2U)
#define GPIO_CRL_CNF_Msk                     (0x33333333UL << GPIO_CRL_CNF_Pos) /*!< 0xCCCCCCCC */
#define GPIO_CRL_CNF                         GPIO_CRL_CNF_Msk                  /*!< Port x configuration bits */

#define GPIO_CRL_CNF0_Pos                    (2U)
#define GPIO_CRL_CNF0_Msk                    (0x3UL << GPIO_CRL_CNF0_Pos)       /*!< 0x0000000C */
#define GPIO_CRL_CNF0                        GPIO_CRL_CNF0_Msk                 /*!< CNF0[1:0] bits (Port x configuration bits, pin 0) */
#define GPIO_CRL_CNF0_0                      (0x1UL << GPIO_CRL_CNF0_Pos)       /*!< 0x00000004 */
#define GPIO_CRL_CNF0_1                      (0x2UL << GPIO_CRL_CNF0_Pos)       /*!< 0x00000008 */

#define GPIO_CRL_CNF1_Pos                    (6U)
#define GPIO_CRL_CNF1_Msk                    (0x3UL << GPIO_CRL_CNF1_Pos)       /*!< 0x000000C0 */
#define GPIO_CRL_CNF1                        GPIO_CRL_CNF1_Msk                 /*!< CNF1[1:0] bits (Port x configuration bits, pin 1) */
#define GPIO_CRL_CNF1_0                      (0x1UL << GPIO_CRL_CNF1_Pos)       /*!< 0x00000040 */
#define GPIO_CRL_CNF1_1                      (0x2UL << GPIO_CRL_CNF1_Pos)       /*!< 0x00000080 */

#define GPIO_CRL_CNF2_Pos                    (10U)
#define GPIO_CRL_CNF2_Msk                    (0x3UL << GPIO_CRL_CNF2_Pos)       /*!< 0x00000C00 */
#define GPIO_CRL_CNF2                        GPIO_CRL_CNF2_Msk                 /*!< CNF2[1:0] bits (Port x configuration bits, pin 2) */
#define GPIO_CRL_CNF2_0                      (0x1UL << GPIO_CRL_CNF2_Pos)       /*!< 0x00000400 */
#define GPIO_CRL_CNF2_1                      (0x2UL << GPIO_CRL_CNF2_Pos)       /*!< 0x00000800 */

#define GPIO_CRL_CNF3_Pos                    (14U)
#define GPIO_CRL_CNF3_Msk                    (0x3UL << GPIO_CRL_CNF3_Pos)       /*!< 0x0000C000 */
#define GPIO_CRL_CNF3                        GPIO_CRL_CNF3_Msk                 /*!< CNF3[1:0] bits (Port x configuration bits, pin 3) */
#define GPIO_CRL_CNF3_0                      (0x1UL << GPIO_CRL_CNF3_Pos)       /*!< 0x00004000 */
#define GPIO_CRL_CNF3_1                      (0x2UL << GPIO_CRL_CNF3_Pos)       /*!< 0x00008000 */

#define GPIO_CRL_CNF4_Pos                    (18U)
#define GPIO_CRL_CNF4_Msk                    (0x3UL << GPIO_CRL_CNF4_Pos)       /*!< 0x000C0000 */
#define GPIO_CRL_CNF4                        GPIO_CRL_CNF4_Msk                 /*!< CNF4[1:0] bits (Port x configuration bits, pin 4) */
#define GPIO_CRL_CNF4_0                      (0x1UL << GPIO_CRL_CNF4_Pos)       /*!< 0x00040000 */
#define GPIO_CRL_CNF4_1                      (0x2UL << GPIO_CRL_CNF4_Pos)       /*!< 0x00080000 */

#define GPIO_CRL_CNF5_Pos                    (22U)
#define GPIO_CRL_CNF5_Msk                    (0x3UL << GPIO_CRL_CNF5_Pos)       /*!< 0x00C00000 */
#define GPIO_CRL_CNF5                        GPIO_CRL_CNF5_Msk                 /*!< CNF5[1:0] bits (Port x configuration bits, pin 5) */
#define GPIO_CRL_CNF5_0                      (0x1UL << GPIO_CRL_CNF5_Pos)       /*!< 0x00400000 */
#define GPIO_CRL_CNF5_1                      (0x2UL << GPIO_CRL_CNF5_Pos)       /*!< 0x00800000 */

#define GPIO_CRL_CNF6_Pos                    (26U)
#define GPIO_CRL_CNF6_Msk                    (0x3UL << GPIO_CRL_CNF6_Pos)       /*!< 0x0C000000 */
#define GPIO_CRL_CNF6                        GPIO_CRL_CNF6_Msk                 /*!< CNF6[1:0] bits (Port x configuration bits, pin 6) */
#define GPIO_CRL_CNF6_0                      (0x1UL << GPIO_CRL_CNF6_Pos)       /*!< 0x04000000 */
#define GPIO_CRL_CNF6_1                      (0x2UL << GPIO_CRL_CNF6_Pos)       /*!< 0x08000000 */

#define GPIO_CRL_CNF7_Pos                    (30U)
#define GPIO_CRL_CNF7_Msk                    (0x3UL << GPIO_CRL_CNF7_Pos)       /*!< 0xC0000000 */
#define GPIO_CRL_CNF7                        GPIO_CRL_CNF7_Msk                 /*!< CNF7[1:0] bits (Port x configuration bits, pin 7) */
#define GPIO_CRL_CNF7_0                      (0x1UL << GPIO_CRL_CNF7_Pos)       /*!< 0x40000000 */
#define GPIO_CRL_CNF7_1                      (0x2UL << GPIO_CRL_CNF7_Pos)       /*!< 0x80000000 */

/*******************  Bit definition for GPIO_CRH register  *******************/
#define GPIO_CRH_MODE_Pos                    (0U)
#define GPIO_CRH_MODE_Msk                    (0x33333333UL << GPIO_CRH_MODE_Pos) /*!< 0x33333333 */
#define GPIO_CRH_MODE                        GPIO_CRH_MODE_Msk                 /*!< Port x mode bits */

#define GPIO_CRH_MODE8_Pos                   (0U)
#define GPIO_CRH_MODE8_Msk                   (0x3UL << GPIO_CRH_MODE8_Pos)      /*!< 0x00000003 */
#define GPIO_CRH_MODE8                       GPIO_CRH_MODE8_Msk                /*!< MODE8[1:0] bits (Port x mode bits, pin 8) */
#define GPIO_CRH_MODE8_0                     (0x1UL << GPIO_CRH_MODE8_Pos)      /*!< 0x00000001 */
#define GPIO_CRH_MODE8_1                     (0x2UL << GPIO_CRH_MODE8_Pos)      /*!< 0x00000002 */

#define GPIO_CRH_MODE9_Pos                   (4U)
#define GPIO_CRH_MODE9_Msk                   (0x3UL << GPIO_CRH_MODE9_Pos)      /*!< 0x00000030 */
#define GPIO_CRH_MODE9                       GPIO_CRH_MODE9_Msk                /*!< MODE9[1:0] bits (Port x mode bits, pin 9) */
#define GPIO_CRH_MODE9_0                     (0x1UL << GPIO_CRH_MODE9_Pos)      /*!< 0x00000010 */
#define GPIO_CRH_MODE9_1                     (0x2UL << GPIO_CRH_MODE9_Pos)      /*!< 0x00000020 */

#define GPIO_CRH_MODE10_Pos                  (8U)
#define GPIO_CRH_MODE10_Msk                  (0x3UL << GPIO_CRH_MODE10_Pos)     /*!< 0x00000300 */
#define GPIO_CRH_MODE10                      GPIO_CRH_MODE10_Msk               /*!< MODE10[1:0] bits (Port x mode bits, pin 10) */
#define GPIO_CRH_MODE10_0                    (0x1UL << GPIO_CRH_MODE10_Pos)     /*!< 0x00000100 */
#define GPIO_CRH_MODE10_1                    (0x2UL << GPIO_CRH_MODE10_Pos)     /*!< 0x00000200 */

#define GPIO_CRH_MODE11_Pos                  (12U)
#define GPIO_CRH_MODE11_Msk                  (0x3UL << GPIO_CRH_MODE11_Pos)     /*!< 0x00003000 */
#define GPIO_CRH_MODE11                      GPIO_CRH_MODE11_Msk               /*!< MODE11[1:0] bits (Port x mode bits, pin 11) */
#define GPIO_CRH_MODE11_0                    (0x1UL << GPIO_CRH_MODE11_Pos)     /*!< 0x00001000 */
#define GPIO_CRH_MODE11_1                    (0x2UL << GPIO_CRH_MODE11_Pos)     /*!< 0x00002000 */

#define GPIO_CRH_MODE12_Pos                  (16U)
#define GPIO_CRH_MODE12_Msk                  (0x3UL << GPIO_CRH_MODE12_Pos)     /*!< 0x00030000 */
#define GPIO_CRH_MODE12                      GPIO_CRH_MODE12_Msk               /*!< MODE12[1:0] bits (Port x mode bits, pin 12) */
#define GPIO_CRH_MODE12_0                    (0x1UL << GPIO_CRH_MODE12_Pos)     /*!< 0x00010000 */
#define GPIO_CRH_MODE12_1                    (0x2UL << GPIO_CRH_MODE12_Pos)     /*!< 0x00020000 */

#define GPIO_CRH_MODE13_Pos                  (20U)
#define GPIO_CRH_MODE13_Msk                  (0x3UL << GPIO_CRH_MODE13_Pos)     /*!< 0x00300000 */
#define GPIO_CRH_MODE13                      GPIO_CRH_MODE13_Msk               /*!< MODE13[1:0] bits (Port x mode bits, pin 13) */
#define GPIO_CRH_MODE13_0                    (0x1UL << GPIO_CRH_MODE13_Pos)     /*!< 0x00100000 */
#define GPIO_CRH_MODE13_1                    (0x2UL << GPIO_CRH_MODE13_Pos)     /*!< 0x00200000 */

#define GPIO_CRH_MODE14_Pos                  (24U)
#define GPIO_CRH_MODE14_Msk                  (0x3UL << GPIO_CRH_MODE14_Pos)     /*!< 0x03000000 */
#define GPIO_CRH_MODE14                      GPIO_CRH_MODE14_Msk               /*!< MODE14[1:0] bits (Port x mode bits, pin 14) */
#define GPIO_CRH_MODE14_0                    (0x1UL << GPIO_CRH_MODE14_Pos)     /*!< 0x01000000 */
#define GPIO_CRH_MODE14_1                    (0x2UL << GPIO_CRH_MODE14_Pos)     /*!< 0x02000000 */

#define GPIO_CRH_MODE15_Pos                  (28U)
#define GPIO_CRH_MODE15_Msk                  (0x3UL << GPIO_CRH_MODE15_Pos)     /*!< 0x30000000 */
#define GPIO_CRH_MODE15                      GPIO_CRH_MODE15_Msk               /*!< MODE15[1:0] bits (Port x mode bits, pin 15) */
#define GPIO_CRH_MODE15_0                    (0x1UL << GPIO_CRH_MODE15_Pos)     /*!< 0x10000000 */
#define GPIO_CRH_MODE15_1                    (0x2UL << GPIO_CRH_MODE15_Pos)     /*!< 0x20000000 */

#define GPIO_CRH_CNF_Pos                     (2U)
#define GPIO_CRH_CNF_Msk                     (0x33333333UL << GPIO_CRH_CNF_Pos) /*!< 0xCCCCCCCC */
#define GPIO_CRH_CNF                         GPIO_CRH_CNF_Msk                  /*!< Port x configuration bits */

#define GPIO_CRH_CNF8_Pos                    (2U)
#define GPIO_CRH_CNF8_Msk                    (0x3UL << GPIO_CRH_CNF8_Pos)       /*!< 0x0000000C */
#define GPIO_CRH_CNF8                        GPIO_CRH_CNF8_Msk                 /*!< CNF8[1:0] bits (Port x configuration bits, pin 8) */
#define GPIO_CRH_CNF8_0                      (0x1UL << GPIO_CRH_CNF8_Pos)       /*!< 0x00000004 */
#define GPIO_CRH_CNF8_1                      (0x2UL << GPIO_CRH_CNF8_Pos)       /*!< 0x00000008 */

#define GPIO_CRH_CNF9_Pos                    (6U)
#define GPIO_CRH_CNF9_Msk                    (0x3UL << GPIO_CRH_CNF9_Pos)       /*!< 0x000000C0 */
#define GPIO_CRH_CNF9                        GPIO_CRH_CNF9_Msk                 /*!< CNF9[1:0] bits (Port x configuration bits, pin 9) */
#define GPIO_CRH_CNF9_0                      (0x1UL << GPIO_CRH_CNF9_Pos)       /*!< 0x00000040 */
#define GPIO_CRH_CNF9_1                      (0x2UL << GPIO_CRH_CNF9_Pos)       /*!< 0x00000080 */

#define GPIO_CRH_CNF10_Pos                   (10U)
#define GPIO_CRH_CNF10_Msk                   (0x3UL << GPIO_CRH_CNF10_Pos)      /*!< 0x00000C00 */
#define GPIO_CRH_CNF10                       GPIO_CRH_CNF10_Msk                /*!< CNF10[1:0] bits (Port x configuration bits, pin 10) */
#define GPIO_CRH_CNF10_0                     (0x1UL << GPIO_CRH_CNF10_Pos)      /*!< 0x00000400 */
#define GPIO_CRH_CNF10_1                     (0x2UL << GPIO_CRH_CNF10_Pos)      /*!< 0x00000800 */

#define GPIO_CRH_CNF11_Pos                   (14U)
#define GPIO_CRH_CNF11_Msk                   (0x3UL << GPIO_CRH_CNF11_Pos)      /*!< 0x0000C000 */
#define GPIO_CRH_CNF11                       GPIO_CRH_CNF11_Msk                /*!< CNF11[1:0] bits (Port x configuration bits, pin 11) */
#define GPIO_CRH_CNF11_0                     (0x1UL << GPIO_CRH_CNF11_Pos)      /*!< 0x00004000 */
#define GPIO_CRH_CNF11_1                     (0x2UL << GPIO_CRH_CNF11_Pos)      /*!< 0x00008000 */

#define GPIO_CRH_CNF12_Pos                   (18U)
#define GPIO_CRH_CNF12_Msk                   (0x3UL << GPIO_CRH_CNF12_Pos)      /*!< 0x000C0000 */
#define GPIO_CRH_CNF12                       GPIO_CRH_CNF12_Msk                /*!< CNF12[1:0] bits (Port x configuration bits, pin 12) */
#define GPIO_CRH_CNF12_0                     (0x1UL << GPIO_CRH_CNF12_Pos)      /*!< 0x00040000 */
#define GPIO_CRH_CNF12_1                     (0x2UL << GPIO_CRH_CNF12_Pos)      /*!< 0x00080000 */

#define GPIO_CRH_CNF13_Pos                   (22U)
#define GPIO_CRH_CNF13_Msk                   (0x3UL << GPIO_CRH_CNF13_Pos)      /*!< 0x00C00000 */
#define GPIO_CRH_CNF13                       GPIO_CRH_CNF13_Msk                /*!< CNF13[1:0] bits (Port x configuration bits, pin 13) */
#define GPIO_CRH_CNF13_0                     (0x1UL << GPIO_CRH_CNF13_Pos)      /*!< 0x00400000 */
#define GPIO_CRH_CNF13_1                     (0x2UL << GPIO_CRH_CNF13_Pos)      /*!< 0x00800000 */

#define GPIO_CRH_CNF14_Pos                   (26U)
#define GPIO_CRH_CNF14_Msk                   (0x3UL << GPIO_CRH_CNF14_Pos)      /*!< 0x0C000000 */
#define GPIO_CRH_CNF14                       GPIO_CRH_CNF14_Msk                /*!< CNF14[1:0] bits (Port x configuration bits, pin 14) */
#define GPIO_CRH_CNF14_0                     (0x1UL << GPIO_CRH_CNF14_Pos)      /*!< 0x04000000 */
#define GPIO_CRH_CNF14_1                     (0x2UL << GPIO_CRH_CNF14_Pos)      /*!< 0x08000000 */

#define GPIO_CRH_CNF15_Pos                   (30U)
#define GPIO_CRH_CNF15_Msk                   (0x3UL << GPIO_CRH_CNF15_Pos)      /*!< 0xC0000000 */
#define GPIO_CRH_CNF15                       GPIO_CRH_CNF15_Msk                /*!< CNF15[1:0] bits (Port x configuration bits, pin 15) */
#define GPIO_CRH_CNF15_0                     (0x1UL << GPIO_CRH_CNF15_Pos)      /*!< 0x40000000 */
#define GPIO_CRH_CNF15_1                     (0x2UL << GPIO_CRH_CNF15_Pos)      /*!< 0x80000000 */

/*!<******************  Bit definition for GPIO_IDR register  *******************/
#define GPIO_IDR_IDR0_Pos                    (0U)
#define GPIO_IDR_IDR0_Msk                    (0x1UL << GPIO_IDR_IDR0_Pos)       /*!< 0x00000001 */
#define GPIO_IDR_IDR0                        GPIO_IDR_IDR0_Msk                 /*!< Port input data, bit 0 */
#define GPIO_IDR_IDR1_Pos                    (1U)
#define GPIO_IDR_IDR1_Msk                    (0x1UL << GPIO_IDR_IDR1_Pos)       /*!< 0x00000002 */
#define GPIO_IDR_IDR1                        GPIO_IDR_IDR1_Msk                 /*!< Port input data, bit 1 */
#define GPIO_IDR_IDR2_Pos                    (2U)
#define GPIO_IDR_IDR2_Msk                    (0x1UL << GPIO_IDR_IDR2_Pos)       /*!< 0x00000004 */
#define GPIO_IDR_IDR2                        GPIO_IDR_IDR2_Msk                 /*!< Port input data, bit 2 */
#define GPIO_IDR_IDR3_Pos                    (3U)
#define GPIO_IDR_IDR3_Msk                    (0x1UL << GPIO_IDR_IDR3_Pos)       /*!< 0x00000008 */
#define GPIO_IDR_IDR3                        GPIO_IDR_IDR3_Msk                 /*!< Port input data, bit 3 */
#define GPIO_IDR_IDR4_Pos                    (4U)
#define GPIO_IDR_IDR4_Msk                    (0x1UL << GPIO_IDR_IDR4_Pos)       /*!< 0x00000010 */
#define GPIO_IDR_IDR4                        GPIO_IDR_IDR4_Msk                 /*!< Port input data, bit 4 */
#define GPIO_IDR_IDR5_Pos                    (5U)
#define GPIO_IDR_IDR5_Msk                    (0x1UL << GPIO_IDR_IDR5_Pos)       /*!< 0x00000020 */
#define GPIO_IDR_IDR5                        GPIO_IDR_IDR5_Msk                 /*!< Port input data, bit 5 */
#define GPIO_IDR_IDR6_Pos                    (6U)
#define GPIO_IDR_IDR6_Msk                    (0x1UL << GPIO_IDR_IDR6_Pos)       /*!< 0x00000040 */
#define GPIO_IDR_IDR6                        GPIO_IDR_IDR6_Msk                 /*!< Port input data, bit 6 */
#define GPIO_IDR_IDR7_Pos                    (7U)
#define GPIO_IDR_IDR7_Msk                    (0x1UL << GPIO_IDR_IDR7_Pos)       /*!< 0x00000080 */
#define GPIO_IDR_IDR7                        GPIO_IDR_IDR7_Msk                 /*!< Port input data, bit 7 */
#define GPIO_IDR_IDR8_Pos                    (8U)
#define GPIO_IDR_IDR8_Msk                    (0x1UL << GPIO_IDR_IDR8_Pos)       /*!< 0x00000100 */
#define GPIO_IDR_IDR8                        GPIO_IDR_IDR8_Msk                 /*!< Port input data, bit 8 */
#define GPIO_IDR_IDR9_Pos                    (9U)
#define GPIO_IDR_IDR9_Msk                    (0x1UL << GPIO_IDR_IDR9_Pos)       /*!< 0x00000200 */
#define GPIO_IDR_IDR9                        GPIO_IDR_IDR9_Msk                 /*!< Port input data, bit 9 */
#define GPIO_IDR_IDR10_Pos                   (10U)
#define GPIO_IDR_IDR10_Msk                   (0x1UL << GPIO_IDR_IDR10_Pos)      /*!< 0x00000400 */
#define GPIO_IDR_IDR10                       GPIO_IDR_IDR10_Msk                /*!< Port input data, bit 10 */
#define GPIO_IDR_IDR11_Pos                   (11U)
#define GPIO_IDR_IDR11_Msk                   (0x1UL << GPIO_IDR_IDR11_Pos)      /*!< 0x00000800 */
#define GPIO_IDR_IDR11                       GPIO_IDR_IDR11_Msk                /*!< Port input data, bit 11 */
#define GPIO_IDR_IDR12_Pos                   (12U)
#define GPIO_IDR_IDR12_Msk                   (0x1UL << GPIO_IDR_IDR12_Pos)      /*!< 0x00001000 */
#define GPIO_IDR_IDR12                       GPIO_IDR_IDR12_Msk                /*!< Port input data, bit 12 */
#define GPIO_IDR_IDR13_Pos                   (13U)
#define GPIO_IDR_IDR13_Msk                   (0x1UL << GPIO_IDR_IDR13_Pos)      /*!< 0x00002000 */
#define GPIO_IDR_IDR13                       GPIO_IDR_IDR13_Msk                /*!< Port input data, bit 13 */
#define GPIO_IDR_IDR14_Pos                   (14U)
#define GPIO_IDR_IDR14_Msk                   (0x1UL << GPIO_IDR_IDR14_Pos)      /*!< 0x00004000 */
#define GPIO_IDR_IDR14                       GPIO_IDR_IDR14_Msk                /*!< Port input data, bit 14 */
#define GPIO_IDR_IDR15_Pos                   (15U)
#define GPIO_IDR_IDR15_Msk                   (0x1UL << GPIO_IDR_IDR15_Pos)      /*!< 0x00008000 */
#define GPIO_IDR_IDR15                       GPIO_IDR_IDR15_Msk                /*!< Port input data, bit 15 */

/*******************  Bit definition for GPIO_ODR register  *******************/
#define GPIO_ODR_ODR0_Pos                    (0U)
#define GPIO_ODR_ODR0_Msk                    (0x1UL << GPIO_ODR_ODR0_Pos)       /*!< 0x00000001 */
#define GPIO_ODR_ODR0                        GPIO_ODR_ODR0_Msk                 /*!< Port output data, bit 0 */
#define GPIO_ODR_ODR1_Pos                    (1U)
#define GPIO_ODR_ODR1_Msk                    (0x1UL << GPIO_ODR_ODR1_Pos)       /*!< 0x00000002 */
#define GPIO_ODR_ODR1                        GPIO_ODR_ODR1_Msk                 /*!< Port output data, bit 1 */
#define GPIO_ODR_ODR2_Pos                    (2U)
#define GPIO_ODR_ODR2_Msk                    (0x1UL << GPIO_ODR_ODR2_Pos)       /*!< 0x00000004 */
#define GPIO_ODR_ODR2                        GPIO_ODR_ODR2_Msk                 /*!< Port output data, bit 2 */
#define GPIO_ODR_ODR3_Pos                    (3U)
#define GPIO_ODR_ODR3_Msk                    (0x1UL << GPIO_ODR_ODR3_Pos)       /*!< 0x00000008 */
#define GPIO_ODR_ODR3                        GPIO_ODR_ODR3_Msk                 /*!< Port output data, bit 3 */
#define GPIO_ODR_ODR4_Pos                    (4U)
#define GPIO_ODR_ODR4_Msk                    (0x1UL << GPIO_ODR_ODR4_Pos)       /*!< 0x00000010 */
#define GPIO_ODR_ODR4                        GPIO_ODR_ODR4_Msk                 /*!< Port output data, bit 4 */
#define GPIO_ODR_ODR5_Pos                    (5U)
#define GPIO_ODR_ODR5_Msk                    (0x1UL << GPIO_ODR_ODR5_Pos)       /*!< 0x00000020 */
#define GPIO_ODR_ODR5                        GPIO_ODR_ODR5_Msk                 /*!< Port output data, bit 5 */
#define GPIO_ODR_ODR6_Pos                    (6U)
#define GPIO_ODR_ODR6_Msk                    (0x1UL << GPIO_ODR_ODR6_Pos)       /*!< 0x00000040 */
#define GPIO_ODR_ODR6                        GPIO_ODR_ODR6_Msk                 /*!< Port output data, bit 6 */
#define GPIO_ODR_ODR7_Pos                    (7U)
#define GPIO_ODR_ODR7_Msk                    (0x1UL << GPIO_ODR_ODR7_Pos)       /*!< 0x00000080 */
#define GPIO_ODR_ODR7                        GPIO_ODR_ODR7_Msk                 /*!< Port output data, bit 7 */
#define GPIO_ODR_ODR8_Pos                    (8U)
#define GPIO_ODR_ODR8_Msk                    (0x1UL << GPIO_ODR_ODR8_Pos)       /*!< 0x00000100 */
#define GPIO_ODR_ODR8                        GPIO_ODR_ODR8_Msk                 /*!< Port output data, bit 8 */
#define GPIO_ODR_ODR9_Pos                    (9U)
#define GPIO_ODR_ODR9_Msk                    (0x1UL << GPIO_ODR_ODR9_Pos)       /*!< 0x00000200 */
#define GPIO_ODR_ODR9                        GPIO_ODR_ODR9_Msk                 /*!< Port output data, bit 9 */
#define GPIO_ODR_ODR10_Pos                   (10U)
#define GPIO_ODR_ODR10_Msk                   (0x1UL << GPIO_ODR_ODR10_Pos)      /*!< 0x00000400 */
#define GPIO_ODR_ODR10                       GPIO_ODR_ODR10_Msk                /*!< Port output data, bit 10 */
#define GPIO_ODR_ODR11_Pos                   (11U)
#define GPIO_ODR_ODR11_Msk                   (0x1UL << GPIO_ODR_ODR11_Pos)      /*!< 0x00000800 */
#define GPIO_ODR_ODR11                       GPIO_ODR_ODR11_Msk                /*!< Port output data, bit 11 */
#define GPIO_ODR_ODR12_Pos                   (12U)
#define GPIO_ODR_ODR12_Msk                   (0x1UL << GPIO_ODR_ODR12_Pos)      /*!< 0x00001000 */
#define GPIO_ODR_ODR12                       GPIO_ODR_ODR12_Msk                /*!< Port output data, bit 12 */
#define GPIO_ODR_ODR13_Pos                   (13U)
#define GPIO_ODR_ODR13_Msk                   (0x1UL << GPIO_ODR_ODR13_Pos)      /*!< 0x00002000 */
#define GPIO_ODR_ODR13                       GPIO_ODR_ODR13_Msk                /*!< Port output data, bit 13 */
#define GPIO_ODR_ODR14_Pos                   (14U)
#define GPIO_ODR_ODR14_Msk                   (0x1UL << GPIO_ODR_ODR14_Pos)      /*!< 0x00004000 */
#define GPIO_ODR_ODR14                       GPIO_ODR_ODR14_Msk                /*!< Port output data, bit 14 */
#define GPIO_ODR_ODR15_Pos                   (15U)
#define GPIO_ODR_ODR15_Msk                   (0x1UL << GPIO_ODR_ODR15_Pos)      /*!< 0x00008000 */
#define GPIO_ODR_ODR15                       GPIO_ODR_ODR15_Msk                /*!< Port output data, bit 15 */

/******************  Bit definition for GPIO_BSRR register  *******************/
#define GPIO_BSRR_BS0_Pos                    (0U)
#define GPIO_BSRR_BS0_Msk                    (0x1UL << GPIO_BSRR_BS0_Pos)       /*!< 0x00000001 */
#define GPIO_BSRR_BS0                        GPIO_BSRR_BS0_Msk                 /*!< Port x Set bit 0 */
#define GPIO_BSRR_BS1_Pos                    (1U)
#define GPIO_BSRR_BS1_Msk                    (0x1UL << GPIO_BSRR_BS1_Pos)       /*!< 0x00000002 */
#define GPIO_BSRR_BS1                        GPIO_BSRR_BS1_Msk                 /*!< Port x Set bit 1 */
#define GPIO_BSRR_BS2_Pos                    (2U)
#define GPIO_BSRR_BS2_Msk                    (0x1UL << GPIO_BSRR_BS2_Pos)       /*!< 0x00000004 */
#define GPIO_BSRR_BS2                        GPIO_BSRR_BS2_Msk                 /*!< Port x Set bit 2 */
#define GPIO_BSRR_BS3_Pos                    (3U)
#define GPIO_BSRR_BS3_Msk                    (0x1UL << GPIO_BSRR_BS3_Pos)       /*!< 0x00000008 */
#define GPIO_BSRR_BS3                        GPIO_BSRR_BS3_Msk                 /*!< Port x Set bit 3 */
#define GPIO_BSRR_BS4_Pos                    (4U)
#define GPIO_BSRR_BS4_Msk                    (0x1UL << GPIO_BSRR_BS4_Pos)       /*!< 0x00000010 */
#define GPIO_BSRR_BS4                        GPIO_BSRR_BS4_Msk                 /*!< Port x Set bit 4 */
#define GPIO_BSRR_BS5_Pos                    (5U)
#define GPIO_BSRR_BS5_Msk                    (0x1UL << GPIO_BSRR_BS5_Pos)       /*!< 0x00000020 */
#define GPIO_BSRR_BS5                        GPIO_BSRR_BS5_Msk                 /*!< Port x Set bit 5 */
#define GPIO_BSRR_BS6_Pos                    (6U)
#define GPIO_BSRR_BS6_Msk                    (0x1UL << GPIO_BSRR_BS6_Pos)       /*!< 0x00000040 */
#define GPIO_BSRR_BS6                        GPIO_BSRR_BS6_Msk                 /*!< Port x Set bit 6 */
#define GPIO_BSRR_BS7_Pos                    (7U)
#define GPIO_BSRR_BS7_Msk                    (0x1UL << GPIO_BSRR_BS7_Pos)       /*!< 0x00000080 */
#define GPIO_BSRR_BS7                        GPIO_BSRR_BS7_Msk                 /*!< Port x Set bit 7 */
#define GPIO_BSRR_BS8_Pos                    (8U)
#define GPIO_BSRR_BS8_Msk                    (0x1UL << GPIO_BSRR_BS8_Pos)       /*!< 0x00000100 */
#define GPIO_BSRR_BS8                        GPIO_BSRR_BS8_Msk                 /*!< Port x Set bit 8 */
#define GPIO_BSRR_BS9_Pos                    (9U)
#define GPIO_BSRR_BS9_Msk                    (0x1UL << GPIO_BSRR_BS9_Pos)       /*!< 0x00000200 */
#define GPIO_BSRR_BS9                        GPIO_BSRR_BS9_Msk                 /*!< Port x Set bit 9 */
#define GPIO_BSRR_BS10_Pos                   (10U)
#define GPIO_BSRR_BS10_Msk                   (0x1UL << GPIO_BSRR_BS10_Pos)      /*!< 0x00000400 */
#define GPIO_BSRR_BS10                       GPIO_BSRR_BS10_Msk                /*!< Port x Set bit 10 */
#define GPIO_BSRR_BS11_Pos                   (11U)
#define GPIO_BSRR_BS11_Msk                   (0x1UL << GPIO_BSRR_BS11_Pos)      /*!< 0x00000800 */
#define GPIO_BSRR_BS11                       GPIO_BSRR_BS11_Msk                /*!< Port x Set bit 11 */
#define GPIO_BSRR_BS12_Pos                   (12U)
#define GPIO_BSRR_BS12_Msk                   (0x1UL << GPIO_BSRR_BS12_Pos)      /*!< 0x00001000 */
#define GPIO_BSRR_BS12                       GPIO_BSRR_BS12_Msk                /*!< Port x Set bit 12 */
#define GPIO_BSRR_BS13_Pos                   (13U)
#define GPIO_BSRR_BS13_Msk                   (0x1UL << GPIO_BSRR_BS13_Pos)      /*!< 0x00002000 */
#define GPIO_BSRR_BS13                       GPIO_BSRR_BS13_Msk                /*!< Port x Set bit 13 */
#define GPIO_BSRR_BS14_Pos                   (14U)
#define GPIO_BSRR_BS14_Msk                   (0x1UL << GPIO_BSRR_BS14_Pos)      /*!< 0x00004000 */
#define GPIO_BSRR_BS14                       GPIO_BSRR_BS14_Msk                /*!< Port x Set bit 14 */
#define GPIO_BSRR_BS15_Pos                   (15U)
#define GPIO_BSRR_BS15_Msk                   (0x1UL << GPIO_BSRR_BS15_Pos)      /*!< 0x00008000 */
#define GPIO_BSRR_BS15                       GPIO_BSRR_BS15_Msk                /*!< Port x Set bit 15 */

#define GPIO_BSRR_BR0_Pos                    (16U)
#define GPIO_BSRR_BR0_Msk                    (0x1UL << GPIO_BSRR_BR0_Pos)       /*!< 0x00010000 */
#define GPIO_BSRR_BR0                        GPIO_BSRR_BR0_Msk                 /*!< Port x Reset bit 0 */
#define GPIO_BSRR_BR1_Pos                    (17U)
#define GPIO_BSRR_BR1_Msk                    (0x1UL << GPIO_BSRR_BR1_Pos)       /*!< 0x00020000 */
#define GPIO_BSRR_BR1                        GPIO_BSRR_BR1_Msk                 /*!< Port x Reset bit 1 */
#define GPIO_BSRR_BR2_Pos                    (18U)
#define GPIO_BSRR_BR2_Msk                    (0x1UL << GPIO_BSRR_BR2_Pos)       /*!< 0x00040000 */
#define GPIO_BSRR_BR2                        GPIO_BSRR_BR2_Msk                 /*!< Port x Reset bit 2 */
#define GPIO_BSRR_BR3_Pos                    (19U)
#define GPIO_BSRR_BR3_Msk                    (0x1UL << GPIO_BSRR_BR3_Pos)       /*!< 0x00080000 */
#define GPIO_BSRR_BR3                        GPIO_BSRR_BR3_Msk                 /*!< Port x Reset bit 3 */
#define GPIO_BSRR_BR4_Pos                    (20U)
#define GPIO_BSRR_BR4_Msk                    (0x1UL << GPIO_BSRR_BR4_Pos)       /*!< 0x00100000 */
#define GPIO_BSRR_BR4                        GPIO_BSRR_BR4_Msk                 /*!< Port x Reset bit 4 */
#define GPIO_BSRR_BR5_Pos                    (21U)
#define GPIO_BSRR_BR5_Msk                    (0x1UL << GPIO_BSRR_BR5_Pos)       /*!< 0x00200000 */
#define GPIO_BSRR_BR5                        GPIO_BSRR_BR5_Msk                 /*!< Port x Reset bit 5 */
#define GPIO_BSRR_BR6_Pos                    (22U)
#define GPIO_BSRR_BR6_Msk                    (0x1UL << GPIO_BSRR_BR6_Pos)       /*!< 0x00400000 */
#define GPIO_BSRR_BR6                        GPIO_BSRR_BR6_Msk                 /*!< Port x Reset bit 6 */
#define GPIO_BSRR_BR7_Pos                    (23U)
#define GPIO_BSRR_BR7_Msk                    (0x1UL << GPIO_BSRR_BR7_Pos)       /*!< 0x00800000 */
#define GPIO_BSRR_BR7                        GPIO_BSRR_BR7_Msk                 /*!< Port x Reset bit 7 */
#define GPIO_BSRR_BR8_Pos                    (24U)
#define GPIO_BSRR_BR8_Msk                    (0x1UL << GPIO_BSRR_BR8_Pos)       /*!< 0x01000000 */
#define GPIO_BSRR_BR8                        GPIO_BSRR_BR8_Msk                 /*!< Port x Reset bit 8 */
#define GPIO_BSRR_BR9_Pos                    (25U)
#define GPIO_BSRR_BR9_Msk                    (0x1UL << GPIO_BSRR_BR9_Pos)       /*!< 0x02000000 */
#define GPIO_BSRR_BR9                        GPIO_BSRR_BR9_Msk                 /*!< Port x Reset bit 9 */
#define GPIO_BSRR_BR10_Pos                   (26U)
#define GPIO_BSRR_BR10_Msk                   (0x1UL << GPIO_BSRR_BR10_Pos)      /*!< 0x04000000 */
#define GPIO_BSRR_BR10                       GPIO_BSRR_BR10_Msk                /*!< Port x Reset bit 10 */
#define GPIO_BSRR_BR11_Pos                   (27U)
#define GPIO_BSRR_BR11_Msk                   (0x1UL << GPIO_BSRR_BR11_Pos)      /*!< 0x08000000 */
#define GPIO_BSRR_BR11                       GPIO_BSRR_BR11_Msk                /*!< Port x Reset bit 11 */
#define GPIO_BSRR_BR12_Pos                   (28U)
#define GPIO_BSRR_BR12_Msk                   (0x1UL << GPIO_BSRR_BR12_Pos)      /*!< 0x10000000 */
#define GPIO_BSRR_BR12                       GPIO_BSRR_BR12_Msk                /*!< Port x Reset bit 12 */
#define GPIO_BSRR_BR13_Pos                   (29U)
#define GPIO_BSRR_BR13_Msk                   (0x1UL << GPIO_BSRR_BR13_Pos)      /*!< 0x20000000 */
#define GPIO_BSRR_BR13                       GPIO_BSRR_BR13_Msk                /*!< Port x Reset bit 13 */
#define GPIO_BSRR_BR14_Pos                   (30U)
#define GPIO_BSRR_BR14_Msk                   (0x1UL << GPIO_BSRR_BR14_Pos)      /*!< 0x40000000 */
#define GPIO_BSRR_BR14                       GPIO_BSRR_BR14_Msk                /*!< Port x Reset bit 14 */
#define GPIO_BSRR_BR15_Pos                   (31U)
#define GPIO_BSRR_BR15_Msk                   (0x1UL << GPIO_BSRR_BR15_Pos)      /*!< 0x80000000 */
#define GPIO_BSRR_BR15                       GPIO_BSRR_BR15_Msk                /*!< Port x Reset bit 15 */

/*******************  Bit definition for GPIO_BRR register  *******************/
#define GPIO_BRR_BR0_Pos                     (0U)
#define GPIO_BRR_BR0_Msk                     (0x1UL << GPIO_BRR_BR0_Pos)        /*!< 0x00000001 */
#define GPIO_BRR_BR0                         GPIO_BRR_BR0_Msk                  /*!< Port x Reset bit 0 */
#define GPIO_BRR_BR1_Pos                     (1U)
#define GPIO_BRR_BR1_Msk                     (0x1UL << GPIO_BRR_BR1_Pos)        /*!< 0x00000002 */
#define GPIO_BRR_BR1                         GPIO_BRR_BR1_Msk                  /*!< Port x Reset bit 1 */
#define GPIO_BRR_BR2_Pos                     (2U)
#define GPIO_BRR_BR2_Msk                     (0x1UL << GPIO_BRR_BR2_Pos)        /*!< 0x00000004 */
#define GPIO_BRR_BR2                         GPIO_BRR_BR2_Msk                  /*!< Port x Reset bit 2 */
#define GPIO_BRR_BR3_Pos                     (3U)
#define GPIO_BRR_BR3_Msk                     (0x1UL << GPIO_BRR_BR3_Pos)        /*!< 0x00000008 */
#define GPIO_BRR_BR3                         GPIO_BRR_BR3_Msk                  /*!< Port x Reset bit 3 */
#define GPIO_BRR_BR4_Pos                     (4U)
#define GPIO_BRR_BR4_Msk                     (0x1UL << GPIO_BRR_BR4_Pos)        /*!< 0x00000010 */
#define GPIO_BRR_BR4                         GPIO_BRR_BR4_Msk                  /*!< Port x Reset bit 4 */
#define GPIO_BRR_BR5_Pos                     (5U)
#define GPIO_BRR_BR5_Msk                     (0x1UL << GPIO_BRR_BR5_Pos)        /*!< 0x00000020 */
#define GPIO_BRR_BR5                         GPIO_BRR_BR5_Msk                  /*!< Port x Reset bit 5 */
#define GPIO_BRR_BR6_Pos                     (6U)
#define GPIO_BRR_BR6_Msk                     (0x1UL << GPIO_BRR_BR6_Pos)        /*!< 0x00000040 */
#define GPIO_BRR_BR6                         GPIO_BRR_BR6_Msk                  /*!< Port x Reset bit 6 */
#define GPIO_BRR_BR7_Pos                     (7U)
#define GPIO_BRR_BR7_Msk                     (0x1UL << GPIO_BRR_BR7_Pos)        /*!< 0x00000080 */
#define GPIO_BRR_BR7                         GPIO_BRR_BR7_Msk                  /*!< Port x Reset bit 7 */
#define GPIO_BRR_BR8_Pos                     (8U)
#define GPIO_BRR_BR8_Msk                     (0x1UL << GPIO_BRR_BR8_Pos)        /*!< 0x00000100 */
#define GPIO_BRR_BR8                         GPIO_BRR_BR8_Msk                  /*!< Port x Reset bit 8 */
#define GPIO_BRR_BR9_Pos                     (9U)
#define GPIO_BRR_BR9_Msk                     (0x1UL << GPIO_BRR_BR9_Pos)        /*!< 0x00000200 */
#define GPIO_BRR_BR9                         GPIO_BRR_BR9_Msk                  /*!< Port x Reset bit 9 */
#define GPIO_BRR_BR10_Pos                    (10U)
#define GPIO_BRR_BR10_Msk                    (0x1UL << GPIO_BRR_BR10_Pos)       /*!< 0x00000400 */
#define GPIO_BRR_BR10                        GPIO_BRR_BR10_Msk                 /*!< Port x Reset bit 10 */
#define GPIO_BRR_BR11_Pos                    (11U)
#define GPIO_BRR_BR11_Msk                    (0x1UL << GPIO_BRR_BR11_Pos)       /*!< 0x00000800 */
#define GPIO_BRR_BR11                        GPIO_BRR_BR11_Msk                 /*!< Port x Reset bit 11 */
#define GPIO_BRR_BR12_Pos                    (12U)
#define GPIO_BRR_BR12_Msk                    (0x1UL << GPIO_BRR_BR12_Pos)       /*!< 0x00001000 */
#define GPIO_BRR_BR12                        GPIO_BRR_BR12_Msk                 /*!< Port x Reset bit 12 */
#define GPIO_BRR_BR13_Pos                    (13U)
#define GPIO_BRR_BR13_Msk                    (0x1UL << GPIO_BRR_BR13_Pos)       /*!< 0x00002000 */
#define GPIO_BRR_BR13                        GPIO_BRR_BR13_Msk                 /*!< Port x Reset bit 13 */
#define GPIO_BRR_BR14_Pos                    (14U)
#define GPIO_BRR_BR14_Msk                    (0x1UL << GPIO_BRR_BR14_Pos)       /*!< 0x00004000 */
#define GPIO_BRR_BR14                        GPIO_BRR_BR14_Msk                 /*!< Port x Reset bit 14 */
#define GPIO_BRR_BR15_Pos                    (15U)
#define GPIO_BRR_BR15_Msk                    (0x1UL << GPIO_BRR_BR15_Pos)       /*!< 0x00008000 */
#define GPIO_BRR_BR15                        GPIO_BRR_BR15_Msk                 /*!< Port x Reset bit 15 */

/******************  Bit definition for GPIO_LCKR register  *******************/
#define GPIO_LCKR_LCK0_Pos                   (0U)
#define GPIO_LCKR_LCK0_Msk                   (0x1UL << GPIO_LCKR_LCK0_Pos)      /*!< 0x00000001 */
#define GPIO_LCKR_LCK0                       GPIO_LCKR_LCK0_Msk                /*!< Port x Lock bit 0 */
#define GPIO_LCKR_LCK1_Pos                   (1U)
#define GPIO_LCKR_LCK1_Msk                   (0x1UL << GPIO_LCKR_LCK1_Pos)      /*!< 0x00000002 */
#define GPIO_LCKR_LCK1                       GPIO_LCKR_LCK1_Msk                /*!< Port x Lock bit 1 */
#define GPIO_LCKR_LCK2_Pos                   (2U)
#define GPIO_LCKR_LCK2_Msk                   (0x1UL << GPIO_LCKR_LCK2_Pos)      /*!< 0x00000004 */
#define GPIO_LCKR_LCK2                       GPIO_LCKR_LCK2_Msk                /*!< Port x Lock bit 2 */
#define GPIO_LCKR_LCK3_Pos                   (3U)
#define GPIO_LCKR_LCK3_Msk                   (0x1UL << GPIO_LCKR_LCK3_Pos)      /*!< 0x00000008 */
#define GPIO_LCKR_LCK3                       GPIO_LCKR_LCK3_Msk                /*!< Port x Lock bit 3 */
#define GPIO_LCKR_LCK4_Pos                   (4U)
#define GPIO_LCKR_LCK4_Msk                   (0x1UL << GPIO_LCKR_LCK4_Pos)      /*!< 0x00000010 */
#define GPIO_LCKR_LCK4                       GPIO_LCKR_LCK4_Msk                /*!< Port x Lock bit 4 */
#define GPIO_LCKR_LCK5_Pos                   (5U)
#define GPIO_LCKR_LCK5_Msk                   (0x1UL << GPIO_LCKR_LCK5_Pos)      /*!< 0x00000020 */
#define GPIO_LCKR_LCK5                       GPIO_LCKR_LCK5_Msk                /*!< Port x Lock bit 5 */
#define GPIO_LCKR_LCK6_Pos                   (6U)
#define GPIO_LCKR_LCK6_Msk                   (0x1UL << GPIO_LCKR_LCK6_Pos)      /*!< 0x00000040 */
#define GPIO_LCKR_LCK6                       GPIO_LCKR_LCK6_Msk                /*!< Port x Lock bit 6 */
#define GPIO_LCKR_LCK7_Pos                   (7U)
#define GPIO_LCKR_LCK7_Msk                   (0x1UL << GPIO_LCKR_LCK7_Pos)      /*!< 0x00000080 */
#define GPIO_LCKR_LCK7                       GPIO_LCKR_LCK7_Msk                /*!< Port x Lock bit 7 */
#define GPIO_LCKR_LCK8_Pos                   (8U)
#define GPIO_LCKR_LCK8_Msk                   (0x1UL << GPIO_LCKR_LCK8_Pos)      /*!< 0x00000100 */
#define GPIO_LCKR_LCK8                       GPIO_LCKR_LCK8_Msk                /*!< Port x Lock bit 8 */
#define GPIO_LCKR_LCK9_Pos                   (9U)
#define GPIO_LCKR_LCK9_Msk                   (0x1UL << GPIO_LCKR_LCK9_Pos)      /*!< 0x00000200 */
#define GPIO_LCKR_LCK9                       GPIO_LCKR_LCK9_Msk                /*!< Port x Lock bit 9 */
#define GPIO_LCKR_LCK10_Pos                  (10U)
#define GPIO_LCKR_LCK10_Msk                  (0x1UL << GPIO_LCKR_LCK10_Pos)     /*!< 0x00000400 */
#define GPIO_LCKR_LCK10                      GPIO_LCKR_LCK10_Msk               /*!< Port x Lock bit 10 */
#define GPIO_LCKR_LCK11_Pos                  (11U)
#define GPIO_LCKR_LCK11_Msk                  (0x1UL << GPIO_LCKR_LCK11_Pos)     /*!< 0x00000800 */
#define GPIO_LCKR_LCK11                      GPIO_LCKR_LCK11_Msk               /*!< Port x Lock bit 11 */
#define GPIO_LCKR_LCK12_Pos                  (12U)
#define GPIO_LCKR_LCK12_Msk                  (0x1UL << GPIO_LCKR_LCK12_Pos)     /*!< 0x00001000 */
#define GPIO_LCKR_LCK12                      GPIO_LCKR_LCK12_Msk               /*!< Port x Lock bit 12 */
#define GPIO_LCKR_LCK13_Pos                  (13U)
#define GPIO_LCKR_LCK13_Msk                  (0x1UL << GPIO_LCKR_LCK13_Pos)     /*!< 0x00002000 */
#define GPIO_LCKR_LCK13                      GPIO_LCKR_LCK13_Msk               /*!< Port x Lock bit 13 */
#define GPIO_LCKR_LCK14_Pos                  (14U)
#define GPIO_LCKR_LCK14_Msk                  (0x1UL << GPIO_LCKR_LCK14_Pos)     /*!< 0x00004000 */
#define GPIO_LCKR_LCK14                      GPIO_LCKR_LCK14_Msk               /*!< Port x Lock bit 14 */
#define GPIO_LCKR_LCK15_Pos                  (15U)
#define GPIO_LCKR_LCK15_Msk                  (0x1UL << GPIO_LCKR_LCK15_Pos)     /*!< 0x00008000 */
#define GPIO_LCKR_LCK15                      GPIO_LCKR_LCK15_Msk               /*!< Port x Lock bit 15 */
#define GPIO_LCKR_LCKK_Pos                   (16U)
#define GPIO_LCKR_LCKK_Msk                   (0x1UL << GPIO_LCKR_LCKK_Pos)      /*!< 0x00010000 */
#define GPIO_LCKR_LCKK                       GPIO_LCKR_LCKK_Msk                /*!< Lock key */

/*----------------------------------------------------------------------------*/

/******************  Bit definition for AFIO_EVCR register  *******************/
#define AFIO_EVCR_PIN_Pos                    (0U)
#define AFIO_EVCR_PIN_Msk                    (0xFUL << AFIO_EVCR_PIN_Pos)       /*!< 0x0000000F */
#define AFIO_EVCR_PIN                        AFIO_EVCR_PIN_Msk                 /*!< PIN[3:0] bits (Pin selection) */
#define AFIO_EVCR_PIN_0                      (0x1UL << AFIO_EVCR_PIN_Pos)       /*!< 0x00000001 */
#define AFIO_EVCR_PIN_1                      (0x2UL << AFIO_EVCR_PIN_Pos)       /*!< 0x00000002 */
#define AFIO_EVCR_PIN_2                      (0x4UL << AFIO_EVCR_PIN_Pos)       /*!< 0x00000004 */
#define AFIO_EVCR_PIN_3                      (0x8UL << AFIO_EVCR_PIN_Pos)       /*!< 0x00000008 */

/*!< PIN configuration */
#define AFIO_EVCR_PIN_PX0                    0x00000000U                       /*!< Pin 0 selected */
#define AFIO_EVCR_PIN_PX1_Pos                (0U)
#define AFIO_EVCR_PIN_PX1_Msk                (0x1UL << AFIO_EVCR_PIN_PX1_Pos)   /*!< 0x00000001 */
#define AFIO_EVCR_PIN_PX1                    AFIO_EVCR_PIN_PX1_Msk             /*!< Pin 1 selected */
#define AFIO_EVCR_PIN_PX2_Pos                (1U)
#define AFIO_EVCR_PIN_PX2_Msk                (0x1UL << AFIO_EVCR_PIN_PX2_Pos)   /*!< 0x00000002 */
#define AFIO_EVCR_PIN_PX2                    AFIO_EVCR_PIN_PX2_Msk             /*!< Pin 2 selected */
#define AFIO_EVCR_PIN_PX3_Pos                (0U)
#define AFIO_EVCR_PIN_PX3_Msk                (0x3UL << AFIO_EVCR_PIN_PX3_Pos)   /*!< 0x00000003 */
#define AFIO_EVCR_PIN_PX3                    AFIO_EVCR_PIN_PX3_Msk             /*!< Pin 3 selected */
#define AFIO_EVCR_PIN_PX4_Pos                (2U)
#define AFIO_EVCR_PIN_PX4_Msk                (0x1UL << AFIO_EVCR_PIN_PX4_Pos)   /*!< 0x00000004 */
#define AFIO_EVCR_PIN_PX4                    AFIO_EVCR_PIN_PX4_Msk             /*!< Pin 4 selected */
#define AFIO_EVCR_PIN_PX5_Pos                (0U)
#define AFIO_EVCR_PIN_PX5_Msk                (0x5UL << AFIO_EVCR_PIN_PX5_Pos)   /*!< 0x00000005 */
#define AFIO_EVCR_PIN_PX5                    AFIO_EVCR_PIN_PX5_Msk             /*!< Pin 5 selected */
#define AFIO_EVCR_PIN_PX6_Pos                (1U)
#define AFIO_EVCR_PIN_PX6_Msk                (0x3UL << AFIO_EVCR_PIN_PX6_Pos)   /*!< 0x00000006 */
#define AFIO_EVCR_PIN_PX6                    AFIO_EVCR_PIN_PX6_Msk             /*!< Pin 6 selected */
#define AFIO_EVCR_PIN_PX7_Pos                (0U)
#define AFIO_EVCR_PIN_PX7_Msk                (0x7UL << AFIO_EVCR_PIN_PX7_Pos)   /*!< 0x00000007 */
#define AFIO_EVCR_PIN_PX7                    AFIO_EVCR_PIN_PX7_Msk             /*!< Pin 7 selected */
#define AFIO_EVCR_PIN_PX8_Pos                (3U)
#define AFIO_EVCR_PIN_PX8_Msk                (0x1UL << AFIO_EVCR_PIN_PX8_Pos)   /*!< 0x00000008 */
#define AFIO_EVCR_PIN_PX8                    AFIO_EVCR_PIN_PX8_Msk             /*!< Pin 8 selected */
#define AFIO_EVCR_PIN_PX9_Pos                (0U)
#define AFIO_EVCR_PIN_PX9_Msk                (0x9UL << AFIO_EVCR_PIN_PX9_Pos)   /*!< 0x00000009 */
#define AFIO_EVCR_PIN_PX9                    AFIO_EVCR_PIN_PX9_Msk             /*!< Pin 9 selected */
#define AFIO_EVCR_PIN_PX10_Pos               (1U)
#define AFIO_EVCR_PIN_PX10_Msk               (0x5UL << AFIO_EVCR_PIN_PX10_Pos)  /*!< 0x0000000A */
#define AFIO_EVCR_PIN_PX10                   AFIO_EVCR_PIN_PX10_Msk            /*!< Pin 10 selected */
#define AFIO_EVCR_PIN_PX11_Pos               (0U)
#define AFIO_EVCR_PIN_PX11_Msk               (0xBUL << AFIO_EVCR_PIN_PX11_Pos)  /*!< 0x0000000B */
#define AFIO_EVCR_PIN_PX11                   AFIO_EVCR_PIN_PX11_Msk            /*!< Pin 11 selected */
#define AFIO_EVCR_PIN_PX12_Pos               (2U)
#define AFIO_EVCR_PIN_PX12_Msk               (0x3UL << AFIO_EVCR_PIN_PX12_Pos)  /*!< 0x0000000C */
#define AFIO_EVCR_PIN_PX12                   AFIO_EVCR_PIN_PX12_Msk            /*!< Pin 12 selected */
#define AFIO_EVCR_PIN_PX13_Pos               (0U)
#define AFIO_EVCR_PIN_PX13_Msk               (0xDUL << AFIO_EVCR_PIN_PX13_Pos)  /*!< 0x0000000D */
#define AFIO_EVCR_PIN_PX13                   AFIO_EVCR_PIN_PX13_Msk            /*!< Pin 13 selected */
#define AFIO_EVCR_PIN_PX14_Pos               (1U)
#define AFIO_EVCR_PIN_PX14_Msk               (0x7UL << AFIO_EVCR_PIN_PX14_Pos)  /*!< 0x0000000E */
#define AFIO_EVCR_PIN_PX14                   AFIO_EVCR_PIN_PX14_Msk            /*!< Pin 14 selected */
#define AFIO_EVCR_PIN_PX15_Pos               (0U)
#define AFIO_EVCR_PIN_PX15_Msk               (0xFUL << AFIO_EVCR_PIN_PX15_Pos)  /*!< 0x0000000F */
#define AFIO_EVCR_PIN_PX15                   AFIO_EVCR_PIN_PX15_Msk            /*!< Pin 15 selected */

#define AFIO_EVCR_PORT_Pos                   (4U)
#define AFIO_EVCR_PORT_Msk                   (0x7UL << AFIO_EVCR_PORT_Pos)      /*!< 0x00000070 */
#define AFIO_EVCR_PORT                       AFIO_EVCR_PORT_Msk                /*!< PORT[2:0] bits (Port selection) */
#define AFIO_EVCR_PORT_0                     (0x1UL << AFIO_EVCR_PORT_Pos)      /*!< 0x00000010 */
#define AFIO_EVCR_PORT_1                     (0x2UL << AFIO_EVCR_PORT_Pos)      /*!< 0x00000020 */
#define AFIO_EVCR_PORT_2                     (0x4UL << AFIO_EVCR_PORT_Pos)      /*!< 0x00000040 */

/*!< PORT configuration */
#define AFIO_EVCR_PORT_PA                    0x00000000                        /*!< Port A selected */
#define AFIO_EVCR_PORT_PB_Pos                (4U)
#define AFIO_EVCR_PORT_PB_Msk                (0x1UL << AFIO_EVCR_PORT_PB_Pos)   /*!< 0x00000010 */
#define AFIO_EVCR_PORT_PB                    AFIO_EVCR_PORT_PB_Msk             /*!< Port B selected */
#define AFIO_EVCR_PORT_PC_Pos                (5U)
#define AFIO_EVCR_PORT_PC_Msk                (0x1UL << AFIO_EVCR_PORT_PC_Pos)   /*!< 0x00000020 */
#define AFIO_EVCR_PORT_PC                    AFIO_EVCR_PORT_PC_Msk             /*!< Port C selected */
#define AFIO_EVCR_PORT_PD_Pos                (4U)
#define AFIO_EVCR_PORT_PD_Msk                (0x3UL << AFIO_EVCR_PORT_PD_Pos)   /*!< 0x00000030 */
#define AFIO_EVCR_PORT_PD                    AFIO_EVCR_PORT_PD_Msk             /*!< Port D selected */
#define AFIO_EVCR_PORT_PE_Pos                (6U)
#define AFIO_EVCR_PORT_PE_Msk                (0x1UL << AFIO_EVCR_PORT_PE_Pos)   /*!< 0x00000040 */
#define AFIO_EVCR_PORT_PE                    AFIO_EVCR_PORT_PE_Msk             /*!< Port E selected */

#define AFIO_EVCR_EVOE_Pos                   (7U)
#define AFIO_EVCR_EVOE_Msk                   (0x1UL << AFIO_EVCR_EVOE_Pos)      /*!< 0x00000080 */
#define AFIO_EVCR_EVOE                       AFIO_EVCR_EVOE_Msk                /*!< Event Output Enable */

/******************  Bit definition for AFIO_MAPR register  *******************/
#define AFIO_MAPR_SPI1_REMAP_Pos             (0U)
#define AFIO_MAPR_SPI1_REMAP_Msk             (0x1UL << AFIO_MAPR_SPI1_REMAP_Pos) /*!< 0x00000001 */
#define AFIO_MAPR_SPI1_REMAP                 AFIO_MAPR_SPI1_REMAP_Msk          /*!< SPI1 remapping */
#define AFIO_MAPR_I2C1_REMAP_Pos             (1U)
#define AFIO_MAPR_I2C1_REMAP_Msk             (0x1UL << AFIO_MAPR_I2C1_REMAP_Pos) /*!< 0x00000002 */
#define AFIO_MAPR_I2C1_REMAP                 AFIO_MAPR_I2C1_REMAP_Msk          /*!< I2C1 remapping */
#define AFIO_MAPR_USART1_REMAP_Pos           (2U)
#define AFIO_MAPR_USART1_REMAP_Msk           (0x1UL << AFIO_MAPR_USART1_REMAP_Pos) /*!< 0x00000004 */
#define AFIO_MAPR_USART1_REMAP               AFIO_MAPR_USART1_REMAP_Msk        /*!< USART1 remapping */
#define AFIO_MAPR_USART2_REMAP_Pos           (3U)
#define AFIO_MAPR_USART2_REMAP_Msk           (0x1UL << AFIO_MAPR_USART2_REMAP_Pos) /*!< 0x00000008 */
#define AFIO_MAPR_USART2_REMAP               AFIO_MAPR_USART2_REMAP_Msk        /*!< USART2 remapping */

#define AFIO_MAPR_USART3_REMAP_Pos           (4U)
#define AFIO_MAPR_USART3_REMAP_Msk           (0x3UL << AFIO_MAPR_USART3_REMAP_Pos) /*!< 0x00000030 */
#define AFIO_MAPR_USART3_REMAP               AFIO_MAPR_USART3_REMAP_Msk        /*!< USART3_REMAP[1:0] bits (USART3 remapping) */
#define AFIO_MAPR_USART3_REMAP_0             (0x1UL << AFIO_MAPR_USART3_REMAP_Pos) /*!< 0x00000010 */
#define AFIO_MAPR_USART3_REMAP_1             (0x2UL << AFIO_MAPR_USART3_REMAP_Pos) /*!< 0x00000020 */

/* USART3_REMAP configuration */
#define AFIO_MAPR_USART3_REMAP_NOREMAP       0x00000000U                          /*!< No remap (TX/PB10, RX/PB11, CK/PB12, CTS/PB13, RTS/PB14) */
#define AFIO_MAPR_USART3_REMAP_PARTIALREMAP_Pos (4U)
#define AFIO_MAPR_USART3_REMAP_PARTIALREMAP_Msk (0x1UL << AFIO_MAPR_USART3_REMAP_PARTIALREMAP_Pos) /*!< 0x00000010 */
#define AFIO_MAPR_USART3_REMAP_PARTIALREMAP  AFIO_MAPR_USART3_REMAP_PARTIALREMAP_Msk /*!< Partial remap (TX/PC10, RX/PC11, CK/PC12, CTS/PB13, RTS/PB14) */
#define AFIO_MAPR_USART3_REMAP_FULLREMAP_Pos (4U)
#define AFIO_MAPR_USART3_REMAP_FULLREMAP_Msk (0x3UL << AFIO_MAPR_USART3_REMAP_FULLREMAP_Pos) /*!< 0x00000030 */
#define AFIO_MAPR_USART3_REMAP_FULLREMAP     AFIO_MAPR_USART3_REMAP_FULLREMAP_Msk /*!< Full remap (TX/PD8, RX/PD9, CK/PD10, CTS/PD11, RTS/PD12) */

#define AFIO_MAPR_TIM1_REMAP_Pos             (6U)
#define AFIO_MAPR_TIM1_REMAP_Msk             (0x3UL << AFIO_MAPR_TIM1_REMAP_Pos) /*!< 0x000000C0 */
#define AFIO_MAPR_TIM1_REMAP                 AFIO_MAPR_TIM1_REMAP_Msk          /*!< TIM1_REMAP[1:0] bits (TIM1 remapping) */
#define AFIO_MAPR_TIM1_REMAP_0               (0x1UL << AFIO_MAPR_TIM1_REMAP_Pos) /*!< 0x00000040 */
#define AFIO_MAPR_TIM1_REMAP_1               (0x2UL << AFIO_MAPR_TIM1_REMAP_Pos) /*!< 0x00000080 */

/*!< TIM1_REMAP configuration */
#define AFIO_MAPR_TIM1_REMAP_NOREMAP         0x00000000U                          /*!< No remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PB12, CH1N/PB13, CH2N/PB14, CH3N/PB15) */
#define AFIO_MAPR_TIM1_REMAP_PARTIALREMAP_Pos (6U)
#define AFIO_MAPR_TIM1_REMAP_PARTIALREMAP_Msk (0x1UL << AFIO_MAPR_TIM1_REMAP_PARTIALREMAP_Pos) /*!< 0x00000040 */
#define AFIO_MAPR_TIM1_REMAP_PARTIALREMAP    AFIO_MAPR_TIM1_REMAP_PARTIALREMAP_Msk /*!< Partial remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PA6, CH1N/PA7, CH2N/PB0, CH3N/PB1) */
#define AFIO_MAPR_TIM1_REMAP_FULLREMAP_Pos   (6U)
#define AFIO_MAPR_TIM1_REMAP_FULLREMAP_Msk   (0x3UL << AFIO_MAPR_TIM1_REMAP_FULLREMAP_Pos) /*!< 0x000000C0 */
#define AFIO_MAPR_TIM1_REMAP_FULLREMAP       AFIO_MAPR_TIM1_REMAP_FULLREMAP_Msk /*!< Full remap (ETR/PE7, CH1/PE9, CH2/PE11, CH3/PE13, CH4/PE14, BKIN/PE15, CH1N/PE8, CH2N/PE10, CH3N/PE12) */

#define AFIO_MAPR_TIM2_REMAP_Pos             (8U)
#define AFIO_MAPR_TIM2_REMAP_Msk             (0x3UL << AFIO_MAPR_TIM2_REMAP_Pos) /*!< 0x00000300 */
#define AFIO_MAPR_TIM2_REMAP                 AFIO_MAPR_TIM2_REMAP_Msk          /*!< TIM2_REMAP[1:0] bits (TIM2 remapping) */
#define AFIO_MAPR_TIM2_REMAP_0               (0x1UL << AFIO_MAPR_TIM2_REMAP_Pos) /*!< 0x00000100 */
#define AFIO_MAPR_TIM2_REMAP_1               (0x2UL << AFIO_MAPR_TIM2_REMAP_Pos) /*!< 0x00000200 */

/*!< TIM2_REMAP configuration */
#define AFIO_MAPR_TIM2_REMAP_NOREMAP         0x00000000U                          /*!< No remap (CH1/ETR/PA0, CH2/PA1, CH3/PA2, CH4/PA3) */
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1_Pos (8U)
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1_Msk (0x1UL << AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1_Pos) /*!< 0x00000100 */
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1   AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1_Msk /*!< Partial remap (CH1/ETR/PA15, CH2/PB3, CH3/PA2, CH4/PA3) */
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2_Pos (9U)
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2_Msk (0x1UL << AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2_Pos) /*!< 0x00000200 */
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2   AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2_Msk /*!< Partial remap (CH1/ETR/PA0, CH2/PA1, CH3/PB10, CH4/PB11) */
#define AFIO_MAPR_TIM2_REMAP_FULLREMAP_Pos   (8U)
#define AFIO_MAPR_TIM2_REMAP_FULLREMAP_Msk   (0x3UL << AFIO_MAPR_TIM2_REMAP_FULLREMAP_Pos) /*!< 0x00000300 */
#define AFIO_MAPR_TIM2_REMAP_FULLREMAP       AFIO_MAPR_TIM2_REMAP_FULLREMAP_Msk /*!< Full remap (CH1/ETR/PA15, CH2/PB3, CH3/PB10, CH4/PB11) */

#define AFIO_MAPR_TIM3_REMAP_Pos             (10U)
#define AFIO_MAPR_TIM3_REMAP_Msk             (0x3UL << AFIO_MAPR_TIM3_REMAP_Pos) /*!< 0x00000C00 */
#define AFIO_MAPR_TIM3_REMAP                 AFIO_MAPR_TIM3_REMAP_Msk          /*!< TIM3_REMAP[1:0] bits (TIM3 remapping) */
#define AFIO_MAPR_TIM3_REMAP_0               (0x1UL << AFIO_MAPR_TIM3_REMAP_Pos) /*!< 0x00000400 */
#define AFIO_MAPR_TIM3_REMAP_1               (0x2UL << AFIO_MAPR_TIM3_REMAP_Pos) /*!< 0x00000800 */

/*!< TIM3_REMAP configuration */
#define AFIO_MAPR_TIM3_REMAP_NOREMAP         0x00000000U                          /*!< No remap (CH1/PA6, CH2/PA7, CH3/PB0, CH4/PB1) */
#define AFIO_MAPR_TIM3_REMAP_PARTIALREMAP_Pos (11U)
#define AFIO_MAPR_TIM3_REMAP_PARTIALREMAP_Msk (0x1UL << AFIO_MAPR_TIM3_REMAP_PARTIALREMAP_Pos) /*!< 0x00000800 */
#define AFIO_MAPR_TIM3_REMAP_PARTIALREMAP    AFIO_MAPR_TIM3_REMAP_PARTIALREMAP_Msk /*!< Partial remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1) */
#define AFIO_MAPR_TIM3_REMAP_FULLREMAP_Pos   (10U)
#define AFIO_MAPR_TIM3_REMAP_FULLREMAP_Msk   (0x3UL << AFIO_MAPR_TIM3_REMAP_FULLREMAP_Pos) /*!< 0x00000C00 */
#define AFIO_MAPR_TIM3_REMAP_FULLREMAP       AFIO_MAPR_TIM3_REMAP_FULLREMAP_Msk /*!< Full remap (CH1/PC6, CH2/PC7, CH3/PC8, CH4/PC9) */

#define AFIO_MAPR_TIM4_REMAP_Pos             (12U)
#define AFIO_MAPR_TIM4_REMAP_Msk             (0x1UL << AFIO_MAPR_TIM4_REMAP_Pos) /*!< 0x00001000 */
#define AFIO_MAPR_TIM4_REMAP                 AFIO_MAPR_TIM4_REMAP_Msk          /*!< TIM4_REMAP bit (TIM4 remapping) */

#define AFIO_MAPR_CAN_REMAP_Pos              (13U)
#define AFIO_MAPR_CAN_REMAP_Msk              (0x3UL << AFIO_MAPR_CAN_REMAP_Pos) /*!< 0x00006000 */
#define AFIO_MAPR_CAN_REMAP                  AFIO_MAPR_CAN_REMAP_Msk           /*!< CAN_REMAP[1:0] bits (CAN Alternate function remapping) */
#define AFIO_MAPR_CAN_REMAP_0                (0x1UL << AFIO_MAPR_CAN_REMAP_Pos) /*!< 0x00002000 */
#define AFIO_MAPR_CAN_REMAP_1                (0x2UL << AFIO_MAPR_CAN_REMAP_Pos) /*!< 0x00004000 */

/*!< CAN_REMAP configuration */
#define AFIO_MAPR_CAN_REMAP_REMAP1           0x00000000U                          /*!< CANRX mapped to PA11, CANTX mapped to PA12 */
#define AFIO_MAPR_CAN_REMAP_REMAP2_Pos       (14U)
#define AFIO_MAPR_CAN_REMAP_REMAP2_Msk       (0x1UL << AFIO_MAPR_CAN_REMAP_REMAP2_Pos) /*!< 0x00004000 */
#define AFIO_MAPR_CAN_REMAP_REMAP2           AFIO_MAPR_CAN_REMAP_REMAP2_Msk    /*!< CANRX mapped to PB8, CANTX mapped to PB9 */
#define AFIO_MAPR_CAN_REMAP_REMAP3_Pos       (13U)
#define AFIO_MAPR_CAN_REMAP_REMAP3_Msk       (0x3UL << AFIO_MAPR_CAN_REMAP_REMAP3_Pos) /*!< 0x00006000 */
#define AFIO_MAPR_CAN_REMAP_REMAP3           AFIO_MAPR_CAN_REMAP_REMAP3_Msk    /*!< CANRX mapped to PD0, CANTX mapped to PD1 */

#define AFIO_MAPR_PD01_REMAP_Pos             (15U)
#define AFIO_MAPR_PD01_REMAP_Msk             (0x1UL << AFIO_MAPR_PD01_REMAP_Pos) /*!< 0x00008000 */
#define AFIO_MAPR_PD01_REMAP                 AFIO_MAPR_PD01_REMAP_Msk          /*!< Port D0/Port D1 mapping on OSC_IN/OSC_OUT */

/*!< SWJ_CFG configuration */
#define AFIO_MAPR_SWJ_CFG_Pos                (24U)
#define AFIO_MAPR_SWJ_CFG_Msk                (0x7UL << AFIO_MAPR_SWJ_CFG_Pos)   /*!< 0x07000000 */
#define AFIO_MAPR_SWJ_CFG                    AFIO_MAPR_SWJ_CFG_Msk             /*!< SWJ_CFG[2:0] bits (Serial Wire JTAG configuration) */
#define AFIO_MAPR_SWJ_CFG_0                  (0x1UL << AFIO_MAPR_SWJ_CFG_Pos)   /*!< 0x01000000 */
#define AFIO_MAPR_SWJ_CFG_1                  (0x2UL << AFIO_MAPR_SWJ_CFG_Pos)   /*!< 0x02000000 */
#define AFIO_MAPR_SWJ_CFG_2                  (0x4UL << AFIO_MAPR_SWJ_CFG_Pos)   /*!< 0x04000000 */

#define AFIO_MAPR_SWJ_CFG_RESET              0x00000000U                          /*!< Full SWJ (JTAG-DP + SW-DP) : Reset State */
#define AFIO_MAPR_SWJ_CFG_NOJNTRST_Pos       (24U)
#define AFIO_MAPR_SWJ_CFG_NOJNTRST_Msk       (0x1UL << AFIO_MAPR_SWJ_CFG_NOJNTRST_Pos) /*!< 0x01000000 */
#define AFIO_MAPR_SWJ_CFG_NOJNTRST           AFIO_MAPR_SWJ_CFG_NOJNTRST_Msk    /*!< Full SWJ (JTAG-DP + SW-DP) but without JNTRST */
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE_Pos    (25U)
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE_Msk    (0x1UL << AFIO_MAPR_SWJ_CFG_JTAGDISABLE_Pos) /*!< 0x02000000 */
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE        AFIO_MAPR_SWJ_CFG_JTAGDISABLE_Msk /*!< JTAG-DP Disabled and SW-DP Enabled */
#define AFIO_MAPR_SWJ_CFG_DISABLE_Pos        (26U)
#define AFIO_MAPR_SWJ_CFG_DISABLE_Msk        (0x1UL << AFIO_MAPR_SWJ_CFG_DISABLE_Pos) /*!< 0x04000000 */
#define AFIO_MAPR_SWJ_CFG_DISABLE            AFIO_MAPR_SWJ_CFG_DISABLE_Msk     /*!< JTAG-DP Disabled and SW-DP Disabled */


/*****************  Bit definition for AFIO_EXTICR1 register  *****************/
#define AFIO_EXTICR1_EXTI0_Pos               (0U)
#define AFIO_EXTICR1_EXTI0_Msk               (0xFUL << AFIO_EXTICR1_EXTI0_Pos)  /*!< 0x0000000F */
#define AFIO_EXTICR1_EXTI0                   AFIO_EXTICR1_EXTI0_Msk            /*!< EXTI 0 configuration */
#define AFIO_EXTICR1_EXTI1_Pos               (4U)
#define AFIO_EXTICR1_EXTI1_Msk               (0xFUL << AFIO_EXTICR1_EXTI1_Pos)  /*!< 0x000000F0 */
#define AFIO_EXTICR1_EXTI1                   AFIO_EXTICR1_EXTI1_Msk            /*!< EXTI 1 configuration */
#define AFIO_EXTICR1_EXTI2_Pos               (8U)
#define AFIO_EXTICR1_EXTI2_Msk               (0xFUL << AFIO_EXTICR1_EXTI2_Pos)  /*!< 0x00000F00 */
#define AFIO_EXTICR1_EXTI2                   AFIO_EXTICR1_EXTI2_Msk            /*!< EXTI 2 configuration */
#define AFIO_EXTICR1_EXTI3_Pos               (12U)
#define AFIO_EXTICR1_EXTI3_Msk               (0xFUL << AFIO_EXTICR1_EXTI3_Pos)  /*!< 0x0000F000 */
#define AFIO_EXTICR1_EXTI3                   AFIO_EXTICR1_EXTI3_Msk            /*!< EXTI 3 configuration */

/*!< EXTI0 configuration */
#define AFIO_EXTICR1_EXTI0_PA                0x00000000U                          /*!< PA[0] pin */
#define AFIO_EXTICR1_EXTI0_PB_Pos            (0U)
#define AFIO_EXTICR1_EXTI0_PB_Msk            (0x1UL << AFIO_EXTICR1_EXTI0_PB_Pos) /*!< 0x00000001 */
#define AFIO_EXTICR1_EXTI0_PB                AFIO_EXTICR1_EXTI0_PB_Msk         /*!< PB[0] pin */
#define AFIO_EXTICR1_EXTI0_PC_Pos            (1U)
#define AFIO_EXTICR1_EXTI0_PC_Msk            (0x1UL << AFIO_EXTICR1_EXTI0_PC_Pos) /*!< 0x00000002 */
#define AFIO_EXTICR1_EXTI0_PC                AFIO_EXTICR1_EXTI0_PC_Msk         /*!< PC[0] pin */
#define AFIO_EXTICR1_EXTI0_PD_Pos            (0U)
#define AFIO_EXTICR1_EXTI0_PD_Msk            (0x3UL << AFIO_EXTICR1_EXTI0_PD_Pos) /*!< 0x00000003 */
#define AFIO_EXTICR1_EXTI0_PD                AFIO_EXTICR1_EXTI0_PD_Msk         /*!< PD[0] pin */
#define AFIO_EXTICR1_EXTI0_PE_Pos            (2U)
#define AFIO_EXTICR1_EXTI0_PE_Msk            (0x1UL << AFIO_EXTICR1_EXTI0_PE_Pos) /*!< 0x00000004 */
#define AFIO_EXTICR1_EXTI0_PE                AFIO_EXTICR1_EXTI0_PE_Msk         /*!< PE[0] pin */
#define AFIO_EXTICR1_EXTI0_PF_Pos            (0U)
#define AFIO_EXTICR1_EXTI0_PF_Msk            (0x5UL << AFIO_EXTICR1_EXTI0_PF_Pos) /*!< 0x00000005 */
#define AFIO_EXTICR1_EXTI0_PF                AFIO_EXTICR1_EXTI0_PF_Msk         /*!< PF[0] pin */
#define AFIO_EXTICR1_EXTI0_PG_Pos            (1U)
#define AFIO_EXTICR1_EXTI0_PG_Msk            (0x3UL << AFIO_EXTICR1_EXTI0_PG_Pos) /*!< 0x00000006 */
#define AFIO_EXTICR1_EXTI0_PG                AFIO_EXTICR1_EXTI0_PG_Msk         /*!< PG[0] pin */

/*!< EXTI1 configuration */
#define AFIO_EXTICR1_EXTI1_PA                0x00000000U                          /*!< PA[1] pin */
#define AFIO_EXTICR1_EXTI1_PB_Pos            (4U)
#define AFIO_EXTICR1_EXTI1_PB_Msk            (0x1UL << AFIO_EXTICR1_EXTI1_PB_Pos) /*!< 0x00000010 */
#define AFIO_EXTICR1_EXTI1_PB                AFIO_EXTICR1_EXTI1_PB_Msk         /*!< PB[1] pin */
#define AFIO_EXTICR1_EXTI1_PC_Pos            (5U)
#define AFIO_EXTICR1_EXTI1_PC_Msk            (0x1UL << AFIO_EXTICR1_EXTI1_PC_Pos) /*!< 0x00000020 */
#define AFIO_EXTICR1_EXTI1_PC                AFIO_EXTICR1_EXTI1_PC_Msk         /*!< PC[1] pin */
#define AFIO_EXTICR1_EXTI1_PD_Pos            (4U)
#define AFIO_EXTICR1_EXTI1_PD_Msk            (0x3UL << AFIO_EXTICR1_EXTI1_PD_Pos) /*!< 0x00000030 */
#define AFIO_EXTICR1_EXTI1_PD                AFIO_EXTICR1_EXTI1_PD_Msk         /*!< PD[1] pin */
#define AFIO_EXTICR1_EXTI1_PE_Pos            (6U)
#define AFIO_EXTICR1_EXTI1_PE_Msk            (0x1UL << AFIO_EXTICR1_EXTI1_PE_Pos) /*!< 0x00000040 */
#define AFIO_EXTICR1_EXTI1_PE                AFIO_EXTICR1_EXTI1_PE_Msk         /*!< PE[1] pin */
#define AFIO_EXTICR1_EXTI1_PF_Pos            (4U)
#define AFIO_EXTICR1_EXTI1_PF_Msk            (0x5UL << AFIO_EXTICR1_EXTI1_PF_Pos) /*!< 0x00000050 */
#define AFIO_EXTICR1_EXTI1_PF                AFIO_EXTICR1_EXTI1_PF_Msk         /*!< PF[1] pin */
#define AFIO_EXTICR1_EXTI1_PG_Pos            (5U)
#define AFIO_EXTICR1_EXTI1_PG_Msk            (0x3UL << AFIO_EXTICR1_EXTI1_PG_Pos) /*!< 0x00000060 */
#define AFIO_EXTICR1_EXTI1_PG                AFIO_EXTICR1_EXTI1_PG_Msk         /*!< PG[1] pin */

/*!< EXTI2 configuration */
#define AFIO_EXTICR1_EXTI2_PA                0x00000000U                          /*!< PA[2] pin */
#define AFIO_EXTICR1_EXTI2_PB_Pos            (8U)
#define AFIO_EXTICR1_EXTI2_PB_Msk            (0x1UL << AFIO_EXTICR1_EXTI2_PB_Pos) /*!< 0x00000100 */
#define AFIO_EXTICR1_EXTI2_PB                AFIO_EXTICR1_EXTI2_PB_Msk         /*!< PB[2] pin */
#define AFIO_EXTICR1_EXTI2_PC_Pos            (9U)
#define AFIO_EXTICR1_EXTI2_PC_Msk            (0x1UL << AFIO_EXTICR1_EXTI2_PC_Pos) /*!< 0x00000200 */
#define AFIO_EXTICR1_EXTI2_PC                AFIO_EXTICR1_EXTI2_PC_Msk         /*!< PC[2] pin */
#define AFIO_EXTICR1_EXTI2_PD_Pos            (8U)
#define AFIO_EXTICR1_EXTI2_PD_Msk            (0x3UL << AFIO_EXTICR1_EXTI2_PD_Pos) /*!< 0x00000300 */
#define AFIO_EXTICR1_EXTI2_PD                AFIO_EXTICR1_EXTI2_PD_Msk         /*!< PD[2] pin */
#define AFIO_EXTICR1_EXTI2_PE_Pos            (10U)
#define AFIO_EXTICR1_EXTI2_PE_Msk            (0x1UL << AFIO_EXTICR1_EXTI2_PE_Pos) /*!< 0x00000400 */
#define AFIO_EXTICR1_EXTI2_PE                AFIO_EXTICR1_EXTI2_PE_Msk         /*!< PE[2] pin */
#define AFIO_EXTICR1_EXTI2_PF_Pos            (8U)
#define AFIO_EXTICR1_EXTI2_PF_Msk            (0x5UL << AFIO_EXTICR1_EXTI2_PF_Pos) /*!< 0x00000500 */
#define AFIO_EXTICR1_EXTI2_PF                AFIO_EXTICR1_EXTI2_PF_Msk         /*!< PF[2] pin */
#define AFIO_EXTICR1_EXTI2_PG_Pos            (9U)
#define AFIO_EXTICR1_EXTI2_PG_Msk            (0x3UL << AFIO_EXTICR1_EXTI2_PG_Pos) /*!< 0x00000600 */
#define AFIO_EXTICR1_EXTI2_PG                AFIO_EXTICR1_EXTI2_PG_Msk         /*!< PG[2] pin */

/*!< EXTI3 configuration */
#define AFIO_EXTICR1_EXTI3_PA                0x00000000U                          /*!< PA[3] pin */
#define AFIO_EXTICR1_EXTI3_PB_Pos            (12U)
#define AFIO_EXTICR1_EXTI3_PB_Msk            (0x1UL << AFIO_EXTICR1_EXTI3_PB_Pos) /*!< 0x00001000 */
#define AFIO_EXTICR1_EXTI3_PB                AFIO_EXTICR1_EXTI3_PB_Msk         /*!< PB[3] pin */
#define AFIO_EXTICR1_EXTI3_PC_Pos            (13U)
#define AFIO_EXTICR1_EXTI3_PC_Msk            (0x1UL << AFIO_EXTICR1_EXTI3_PC_Pos) /*!< 0x00002000 */
#define AFIO_EXTICR1_EXTI3_PC                AFIO_EXTICR1_EXTI3_PC_Msk         /*!< PC[3] pin */
#define AFIO_EXTICR1_EXTI3_PD_Pos            (12U)
#define AFIO_EXTICR1_EXTI3_PD_Msk            (0x3UL << AFIO_EXTICR1_EXTI3_PD_Pos) /*!< 0x00003000 */
#define AFIO_EXTICR1_EXTI3_PD                AFIO_EXTICR1_EXTI3_PD_Msk         /*!< PD[3] pin */
#define AFIO_EXTICR1_EXTI3_PE_Pos            (14U)
#define AFIO_EXTICR1_EXTI3_PE_Msk            (0x1UL << AFIO_EXTICR1_EXTI3_PE_Pos) /*!< 0x00004000 */
#define AFIO_EXTICR1_EXTI3_PE                AFIO_EXTICR1_EXTI3_PE_Msk         /*!< PE[3] pin */
#define AFIO_EXTICR1_EXTI3_PF_Pos            (12U)
#define AFIO_EXTICR1_EXTI3_PF_Msk            (0x5UL << AFIO_EXTICR1_EXTI3_PF_Pos) /*!< 0x00005000 */
#define AFIO_EXTICR1_EXTI3_PF                AFIO_EXTICR1_EXTI3_PF_Msk         /*!< PF[3] pin */
#define AFIO_EXTICR1_EXTI3_PG_Pos            (13U)
#define AFIO_EXTICR1_EXTI3_PG_Msk            (0x3UL << AFIO_EXTICR1_EXTI3_PG_Pos) /*!< 0x00006000 */
#define AFIO_EXTICR1_EXTI3_PG                AFIO_EXTICR1_EXTI3_PG_Msk         /*!< PG[3] pin */

/*****************  Bit definition for AFIO_EXTICR2 register  *****************/
#define AFIO_EXTICR2_EXTI4_Pos               (0U)
#define AFIO_EXTICR2_EXTI4_Msk               (0xFUL << AFIO_EXTICR2_EXTI4_Pos)  /*!< 0x0000000F */
#define AFIO_EXTICR2_EXTI4                   AFIO_EXTICR2_EXTI4_Msk            /*!< EXTI 4 configuration */
#define AFIO_EXTICR2_EXTI5_Pos               (4U)
#define AFIO_EXTICR2_EXTI5_Msk               (0xFUL << AFIO_EXTICR2_EXTI5_Pos)  /*!< 0x000000F0 */
#define AFIO_EXTICR2_EXTI5                   AFIO_EXTICR2_EXTI5_Msk            /*!< EXTI 5 configuration */
#define AFIO_EXTICR2_EXTI6_Pos               (8U)
#define AFIO_EXTICR2_EXTI6_Msk               (0xFUL << AFIO_EXTICR2_EXTI6_Pos)  /*!< 0x00000F00 */
#define AFIO_EXTICR2_EXTI6                   AFIO_EXTICR2_EXTI6_Msk            /*!< EXTI 6 configuration */
#define AFIO_EXTICR2_EXTI7_Pos               (12U)
#define AFIO_EXTICR2_EXTI7_Msk               (0xFUL << AFIO_EXTICR2_EXTI7_Pos)  /*!< 0x0000F000 */
#define AFIO_EXTICR2_EXTI7                   AFIO_EXTICR2_EXTI7_Msk            /*!< EXTI 7 configuration */

/*!< EXTI4 configuration */
#define AFIO_EXTICR2_EXTI4_PA                0x00000000U                          /*!< PA[4] pin */
#define AFIO_EXTICR2_EXTI4_PB_Pos            (0U)
#define AFIO_EXTICR2_EXTI4_PB_Msk            (0x1UL << AFIO_EXTICR2_EXTI4_PB_Pos) /*!< 0x00000001 */
#define AFIO_EXTICR2_EXTI4_PB                AFIO_EXTICR2_EXTI4_PB_Msk         /*!< PB[4] pin */
#define AFIO_EXTICR2_EXTI4_PC_Pos            (1U)
#define AFIO_EXTICR2_EXTI4_PC_Msk            (0x1UL << AFIO_EXTICR2_EXTI4_PC_Pos) /*!< 0x00000002 */
#define AFIO_EXTICR2_EXTI4_PC                AFIO_EXTICR2_EXTI4_PC_Msk         /*!< PC[4] pin */
#define AFIO_EXTICR2_EXTI4_PD_Pos            (0U)
#define AFIO_EXTICR2_EXTI4_PD_Msk            (0x3UL << AFIO_EXTICR2_EXTI4_PD_Pos) /*!< 0x00000003 */
#define AFIO_EXTICR2_EXTI4_PD                AFIO_EXTICR2_EXTI4_PD_Msk         /*!< PD[4] pin */
#define AFIO_EXTICR2_EXTI4_PE_Pos            (2U)
#define AFIO_EXTICR2_EXTI4_PE_Msk            (0x1UL << AFIO_EXTICR2_EXTI4_PE_Pos) /*!< 0x00000004 */
#define AFIO_EXTICR2_EXTI4_PE                AFIO_EXTICR2_EXTI4_PE_Msk         /*!< PE[4] pin */
#define AFIO_EXTICR2_EXTI4_PF_Pos            (0U)
#define AFIO_EXTICR2_EXTI4_PF_Msk            (0x5UL << AFIO_EXTICR2_EXTI4_PF_Pos) /*!< 0x00000005 */
#define AFIO_EXTICR2_EXTI4_PF                AFIO_EXTICR2_EXTI4_PF_Msk         /*!< PF[4] pin */
#define AFIO_EXTICR2_EXTI4_PG_Pos            (1U)
#define AFIO_EXTICR2_EXTI4_PG_Msk            (0x3UL << AFIO_EXTICR2_EXTI4_PG_Pos) /*!< 0x00000006 */
#define AFIO_EXTICR2_EXTI4_PG                AFIO_EXTICR2_EXTI4_PG_Msk         /*!< PG[4] pin */

/* EXTI5 configuration */
#define AFIO_EXTICR2_EXTI5_PA                0x00000000U                          /*!< PA[5] pin */
#define AFIO_EXTICR2_EXTI5_PB_Pos            (4U)
#define AFIO_EXTICR2_EXTI5_PB_Msk            (0x1UL << AFIO_EXTICR2_EXTI5_PB_Pos) /*!< 0x00000010 */
#define AFIO_EXTICR2_EXTI5_PB                AFIO_EXTICR2_EXTI5_PB_Msk         /*!< PB[5] pin */
#define AFIO_EXTICR2_EXTI5_PC_Pos            (5U)
#define AFIO_EXTICR2_EXTI5_PC_Msk            (0x1UL << AFIO_EXTICR2_EXTI5_PC_Pos) /*!< 0x00000020 */
#define AFIO_EXTICR2_EXTI5_PC                AFIO_EXTICR2_EXTI5_PC_Msk         /*!< PC[5] pin */
#define AFIO_EXTICR2_EXTI5_PD_Pos            (4U)
#define AFIO_EXTICR2_EXTI5_PD_Msk            (0x3UL << AFIO_EXTICR2_EXTI5_PD_Pos) /*!< 0x00000030 */
#define AFIO_EXTICR2_EXTI5_PD                AFIO_EXTICR2_EXTI5_PD_Msk         /*!< PD[5] pin */
#define AFIO_EXTICR2_EXTI5_PE_Pos            (6U)
#define AFIO_EXTICR2_EXTI5_PE_Msk            (0x1UL << AFIO_EXTICR2_EXTI5_PE_Pos) /*!< 0x00000040 */
#define AFIO_EXTICR2_EXTI5_PE                AFIO_EXTICR2_EXTI5_PE_Msk         /*!< PE[5] pin */
#define AFIO_EXTICR2_EXTI5_PF_Pos            (4U)
#define AFIO_EXTICR2_EXTI5_PF_Msk            (0x5UL << AFIO_EXTICR2_EXTI5_PF_Pos) /*!< 0x00000050 */
#define AFIO_EXTICR2_EXTI5_PF                AFIO_EXTICR2_EXTI5_PF_Msk         /*!< PF[5] pin */
#define AFIO_EXTICR2_EXTI5_PG_Pos            (5U)
#define AFIO_EXTICR2_EXTI5_PG_Msk            (0x3UL << AFIO_EXTICR2_EXTI5_PG_Pos) /*!< 0x00000060 */
#define AFIO_EXTICR2_EXTI5_PG                AFIO_EXTICR2_EXTI5_PG_Msk         /*!< PG[5] pin */

/*!< EXTI6 configuration */
#define AFIO_EXTICR2_EXTI6_PA                0x00000000U                          /*!< PA[6] pin */
#define AFIO_EXTICR2_EXTI6_PB_Pos            (8U)
#define AFIO_EXTICR2_EXTI6_PB_Msk            (0x1UL << AFIO_EXTICR2_EXTI6_PB_Pos) /*!< 0x00000100 */
#define AFIO_EXTICR2_EXTI6_PB                AFIO_EXTICR2_EXTI6_PB_Msk         /*!< PB[6] pin */
#define AFIO_EXTICR2_EXTI6_PC_Pos            (9U)
#define AFIO_EXTICR2_EXTI6_PC_Msk            (0x1UL << AFIO_EXTICR2_EXTI6_PC_Pos) /*!< 0x00000200 */
#define AFIO_EXTICR2_EXTI6_PC                AFIO_EXTICR2_EXTI6_PC_Msk         /*!< PC[6] pin */
#define AFIO_EXTICR2_EXTI6_PD_Pos            (8U)
#define AFIO_EXTICR2_EXTI6_PD_Msk            (0x3UL << AFIO_EXTICR2_EXTI6_PD_Pos) /*!< 0x00000300 */
#define AFIO_EXTICR2_EXTI6_PD                AFIO_EXTICR2_EXTI6_PD_Msk         /*!< PD[6] pin */
#define AFIO_EXTICR2_EXTI6_PE_Pos            (10U)
#define AFIO_EXTICR2_EXTI6_PE_Msk            (0x1UL << AFIO_EXTICR2_EXTI6_PE_Pos) /*!< 0x00000400 */
#define AFIO_EXTICR2_EXTI6_PE                AFIO_EXTICR2_EXTI6_PE_Msk         /*!< PE[6] pin */
#define AFIO_EXTICR2_EXTI6_PF_Pos            (8U)
#define AFIO_EXTICR2_EXTI6_PF_Msk            (0x5UL << AFIO_EXTICR2_EXTI6_PF_Pos) /*!< 0x00000500 */
#define AFIO_EXTICR2_EXTI6_PF                AFIO_EXTICR2_EXTI6_PF_Msk         /*!< PF[6] pin */
#define AFIO_EXTICR2_EXTI6_PG_Pos            (9U)
#define AFIO_EXTICR2_EXTI6_PG_Msk            (0x3UL << AFIO_EXTICR2_EXTI6_PG_Pos) /*!< 0x00000600 */
#define AFIO_EXTICR2_EXTI6_PG                AFIO_EXTICR2_EXTI6_PG_Msk         /*!< PG[6] pin */

/*!< EXTI7 configuration */
#define AFIO_EXTICR2_EXTI7_PA                0x00000000U                          /*!< PA[7] pin */
#define AFIO_EXTICR2_EXTI7_PB_Pos            (12U)
#define AFIO_EXTICR2_EXTI7_PB_Msk            (0x1UL << AFIO_EXTICR2_EXTI7_PB_Pos) /*!< 0x00001000 */
#define AFIO_EXTICR2_EXTI7_PB                AFIO_EXTICR2_EXTI7_PB_Msk         /*!< PB[7] pin */
#define AFIO_EXTICR2_EXTI7_PC_Pos            (13U)
#define AFIO_EXTICR2_EXTI7_PC_Msk            (0x1UL << AFIO_EXTICR2_EXTI7_PC_Pos) /*!< 0x00002000 */
#define AFIO_EXTICR2_EXTI7_PC                AFIO_EXTICR2_EXTI7_PC_Msk         /*!< PC[7] pin */
#define AFIO_EXTICR2_EXTI7_PD_Pos            (12U)
#define AFIO_EXTICR2_EXTI7_PD_Msk            (0x3UL << AFIO_EXTICR2_EXTI7_PD_Pos) /*!< 0x00003000 */
#define AFIO_EXTICR2_EXTI7_PD                AFIO_EXTICR2_EXTI7_PD_Msk         /*!< PD[7] pin */
#define AFIO_EXTICR2_EXTI7_PE_Pos            (14U)
#define AFIO_EXTICR2_EXTI7_PE_Msk            (0x1UL << AFIO_EXTICR2_EXTI7_PE_Pos) /*!< 0x00004000 */
#define AFIO_EXTICR2_EXTI7_PE                AFIO_EXTICR2_EXTI7_PE_Msk         /*!< PE[7] pin */
#define AFIO_EXTICR2_EXTI7_PF_Pos            (12U)
#define AFIO_EXTICR2_EXTI7_PF_Msk            (0x5UL << AFIO_EXTICR2_EXTI7_PF_Pos) /*!< 0x00005000 */
#define AFIO_EXTICR2_EXTI7_PF                AFIO_EXTICR2_EXTI7_PF_Msk         /*!< PF[7] pin */
#define AFIO_EXTICR2_EXTI7_PG_Pos            (13U)
#define AFIO_EXTICR2_EXTI7_PG_Msk            (0x3UL << AFIO_EXTICR2_EXTI7_PG_Pos) /*!< 0x00006000 */
#define AFIO_EXTICR2_EXTI7_PG                AFIO_EXTICR2_EXTI7_PG_Msk         /*!< PG[7] pin */

/*****************  Bit definition for AFIO_EXTICR3 register  *****************/
#define AFIO_EXTICR3_EXTI8_Pos               (0U)
#define AFIO_EXTICR3_EXTI8_Msk               (0xFUL << AFIO_EXTICR3_EXTI8_Pos)  /*!< 0x0000000F */
#define AFIO_EXTICR3_EXTI8                   AFIO_EXTICR3_EXTI8_Msk            /*!< EXTI 8 configuration */
#define AFIO_EXTICR3_EXTI9_Pos               (4U)
#define AFIO_EXTICR3_EXTI9_Msk               (0xFUL << AFIO_EXTICR3_EXTI9_Pos)  /*!< 0x000000F0 */
#define AFIO_EXTICR3_EXTI9                   AFIO_EXTICR3_EXTI9_Msk            /*!< EXTI 9 configuration */
#define AFIO_EXTICR3_EXTI10_Pos              (8U)
#define AFIO_EXTICR3_EXTI10_Msk              (0xFUL << AFIO_EXTICR3_EXTI10_Pos) /*!< 0x00000F00 */
#define AFIO_EXTICR3_EXTI10                  AFIO_EXTICR3_EXTI10_Msk           /*!< EXTI 10 configuration */
#define AFIO_EXTICR3_EXTI11_Pos              (12U)
#define AFIO_EXTICR3_EXTI11_Msk              (0xFUL << AFIO_EXTICR3_EXTI11_Pos) /*!< 0x0000F000 */
#define AFIO_EXTICR3_EXTI11                  AFIO_EXTICR3_EXTI11_Msk           /*!< EXTI 11 configuration */

/*!< EXTI8 configuration */
#define AFIO_EXTICR3_EXTI8_PA                0x00000000U                          /*!< PA[8] pin */
#define AFIO_EXTICR3_EXTI8_PB_Pos            (0U)
#define AFIO_EXTICR3_EXTI8_PB_Msk            (0x1UL << AFIO_EXTICR3_EXTI8_PB_Pos) /*!< 0x00000001 */
#define AFIO_EXTICR3_EXTI8_PB                AFIO_EXTICR3_EXTI8_PB_Msk         /*!< PB[8] pin */
#define AFIO_EXTICR3_EXTI8_PC_Pos            (1U)
#define AFIO_EXTICR3_EXTI8_PC_Msk            (0x1UL << AFIO_EXTICR3_EXTI8_PC_Pos) /*!< 0x00000002 */
#define AFIO_EXTICR3_EXTI8_PC                AFIO_EXTICR3_EXTI8_PC_Msk         /*!< PC[8] pin */
#define AFIO_EXTICR3_EXTI8_PD_Pos            (0U)
#define AFIO_EXTICR3_EXTI8_PD_Msk            (0x3UL << AFIO_EXTICR3_EXTI8_PD_Pos) /*!< 0x00000003 */
#define AFIO_EXTICR3_EXTI8_PD                AFIO_EXTICR3_EXTI8_PD_Msk         /*!< PD[8] pin */
#define AFIO_EXTICR3_EXTI8_PE_Pos            (2U)
#define AFIO_EXTICR3_EXTI8_PE_Msk            (0x1UL << AFIO_EXTICR3_EXTI8_PE_Pos) /*!< 0x00000004 */
#define AFIO_EXTICR3_EXTI8_PE                AFIO_EXTICR3_EXTI8_PE_Msk         /*!< PE[8] pin */
#define AFIO_EXTICR3_EXTI8_PF_Pos            (0U)
#define AFIO_EXTICR3_EXTI8_PF_Msk            (0x5UL << AFIO_EXTICR3_EXTI8_PF_Pos) /*!< 0x00000005 */
#define AFIO_EXTICR3_EXTI8_PF                AFIO_EXTICR3_EXTI8_PF_Msk         /*!< PF[8] pin */
#define AFIO_EXTICR3_EXTI8_PG_Pos            (1U)
#define AFIO_EXTICR3_EXTI8_PG_Msk            (0x3UL << AFIO_EXTICR3_EXTI8_PG_Pos) /*!< 0x00000006 */
#define AFIO_EXTICR3_EXTI8_PG                AFIO_EXTICR3_EXTI8_PG_Msk         /*!< PG[8] pin */

/*!< EXTI9 configuration */
#define AFIO_EXTICR3_EXTI9_PA                0x00000000U                          /*!< PA[9] pin */
#define AFIO_EXTICR3_EXTI9_PB_Pos            (4U)
#define AFIO_EXTICR3_EXTI9_PB_Msk            (0x1UL << AFIO_EXTICR3_EXTI9_PB_Pos) /*!< 0x00000010 */
#define AFIO_EXTICR3_EXTI9_PB                AFIO_EXTICR3_EXTI9_PB_Msk         /*!< PB[9] pin */
#define AFIO_EXTICR3_EXTI9_PC_Pos            (5U)
#define AFIO_EXTICR3_EXTI9_PC_Msk            (0x1UL << AFIO_EXTICR3_EXTI9_PC_Pos) /*!< 0x00000020 */
#define AFIO_EXTICR3_EXTI9_PC                AFIO_EXTICR3_EXTI9_PC_Msk         /*!< PC[9] pin */
#define AFIO_EXTICR3_EXTI9_PD_Pos            (4U)
#define AFIO_EXTICR3_EXTI9_PD_Msk            (0x3UL << AFIO_EXTICR3_EXTI9_PD_Pos) /*!< 0x00000030 */
#define AFIO_EXTICR3_EXTI9_PD                AFIO_EXTICR3_EXTI9_PD_Msk         /*!< PD[9] pin */
#define AFIO_EXTICR3_EXTI9_PE_Pos            (6U)
#define AFIO_EXTICR3_EXTI9_PE_Msk            (0x1UL << AFIO_EXTICR3_EXTI9_PE_Pos) /*!< 0x00000040 */
#define AFIO_EXTICR3_EXTI9_PE                AFIO_EXTICR3_EXTI9_PE_Msk         /*!< PE[9] pin */
#define AFIO_EXTICR3_EXTI9_PF_Pos            (4U)
#define AFIO_EXTICR3_EXTI9_PF_Msk            (0x5UL << AFIO_EXTICR3_EXTI9_PF_Pos) /*!< 0x00000050 */
#define AFIO_EXTICR3_EXTI9_PF                AFIO_EXTICR3_EXTI9_PF_Msk         /*!< PF[9] pin */
#define AFIO_EXTICR3_EXTI9_PG_Pos            (5U)
#define AFIO_EXTICR3_EXTI9_PG_Msk            (0x3UL << AFIO_EXTICR3_EXTI9_PG_Pos) /*!< 0x00000060 */
#define AFIO_EXTICR3_EXTI9_PG                AFIO_EXTICR3_EXTI9_PG_Msk         /*!< PG[9] pin */

/*!< EXTI10 configuration */
#define AFIO_EXTICR3_EXTI10_PA               0x00000000U                          /*!< PA[10] pin */
#define AFIO_EXTICR3_EXTI10_PB_Pos           (8U)
#define AFIO_EXTICR3_EXTI10_PB_Msk           (0x1UL << AFIO_EXTICR3_EXTI10_PB_Pos) /*!< 0x00000100 */
#define AFIO_EXTICR3_EXTI10_PB               AFIO_EXTICR3_EXTI10_PB_Msk        /*!< PB[10] pin */
#define AFIO_EXTICR3_EXTI10_PC_Pos           (9U)
#define AFIO_EXTICR3_EXTI10_PC_Msk           (0x1UL << AFIO_EXTICR3_EXTI10_PC_Pos) /*!< 0x00000200 */
#define AFIO_EXTICR3_EXTI10_PC               AFIO_EXTICR3_EXTI10_PC_Msk        /*!< PC[10] pin */
#define AFIO_EXTICR3_EXTI10_PD_Pos           (8U)
#define AFIO_EXTICR3_EXTI10_PD_Msk           (0x3UL << AFIO_EXTICR3_EXTI10_PD_Pos) /*!< 0x00000300 */
#define AFIO_EXTICR3_EXTI10_PD               AFIO_EXTICR3_EXTI10_PD_Msk        /*!< PD[10] pin */
#define AFIO_EXTICR3_EXTI10_PE_Pos           (10U)
#define AFIO_EXTICR3_EXTI10_PE_Msk           (0x1UL << AFIO_EXTICR3_EXTI10_PE_Pos) /*!< 0x00000400 */
#define AFIO_EXTICR3_EXTI10_PE               AFIO_EXTICR3_EXTI10_PE_Msk        /*!< PE[10] pin */
#define AFIO_EXTICR3_EXTI10_PF_Pos           (8U)
#define AFIO_EXTICR3_EXTI10_PF_Msk           (0x5UL << AFIO_EXTICR3_EXTI10_PF_Pos) /*!< 0x00000500 */
#define AFIO_EXTICR3_EXTI10_PF               AFIO_EXTICR3_EXTI10_PF_Msk        /*!< PF[10] pin */
#define AFIO_EXTICR3_EXTI10_PG_Pos           (9U)
#define AFIO_EXTICR3_EXTI10_PG_Msk           (0x3UL << AFIO_EXTICR3_EXTI10_PG_Pos) /*!< 0x00000600 */
#define AFIO_EXTICR3_EXTI10_PG               AFIO_EXTICR3_EXTI10_PG_Msk        /*!< PG[10] pin */

/*!< EXTI11 configuration */
#define AFIO_EXTICR3_EXTI11_PA               0x00000000U                          /*!< PA[11] pin */
#define AFIO_EXTICR3_EXTI11_PB_Pos           (12U)
#define AFIO_EXTICR3_EXTI11_PB_Msk           (0x1UL << AFIO_EXTICR3_EXTI11_PB_Pos) /*!< 0x00001000 */
#define AFIO_EXTICR3_EXTI11_PB               AFIO_EXTICR3_EXTI11_PB_Msk        /*!< PB[11] pin */
#define AFIO_EXTICR3_EXTI11_PC_Pos           (13U)
#define AFIO_EXTICR3_EXTI11_PC_Msk           (0x1UL << AFIO_EXTICR3_EXTI11_PC_Pos) /*!< 0x00002000 */
#define AFIO_EXTICR3_EXTI11_PC               AFIO_EXTICR3_EXTI11_PC_Msk        /*!< PC[11] pin */
#define AFIO_EXTICR3_EXTI11_PD_Pos           (12U)
#define AFIO_EXTICR3_EXTI11_PD_Msk           (0x3UL << AFIO_EXTICR3_EXTI11_PD_Pos) /*!< 0x00003000 */
#define AFIO_EXTICR3_EXTI11_PD               AFIO_EXTICR3_EXTI11_PD_Msk        /*!< PD[11] pin */
#define AFIO_EXTICR3_EXTI11_PE_Pos           (14U)
#define AFIO_EXTICR3_EXTI11_PE_Msk           (0x1UL << AFIO_EXTICR3_EXTI11_PE_Pos) /*!< 0x00004000 */
#define AFIO_EXTICR3_EXTI11_PE               AFIO_EXTICR3_EXTI11_PE_Msk        /*!< PE[11] pin */
#define AFIO_EXTICR3_EXTI11_PF_Pos           (12U)
#define AFIO_EXTICR3_EXTI11_PF_Msk           (0x5UL << AFIO_EXTICR3_EXTI11_PF_Pos) /*!< 0x00005000 */
#define AFIO_EXTICR3_EXTI11_PF               AFIO_EXTICR3_EXTI11_PF_Msk        /*!< PF[11] pin */
#define AFIO_EXTICR3_EXTI11_PG_Pos           (13U)
#define AFIO_EXTICR3_EXTI11_PG_Msk           (0x3UL << AFIO_EXTICR3_EXTI11_PG_Pos) /*!< 0x00006000 */
#define AFIO_EXTICR3_EXTI11_PG               AFIO_EXTICR3_EXTI11_PG_Msk        /*!< PG[11] pin */

/*****************  Bit definition for AFIO_EXTICR4 register  *****************/
#define AFIO_EXTICR4_EXTI12_Pos              (0U)
#define AFIO_EXTICR4_EXTI12_Msk              (0xFUL << AFIO_EXTICR4_EXTI12_Pos) /*!< 0x0000000F */
#define AFIO_EXTICR4_EXTI12                  AFIO_EXTICR4_EXTI12_Msk           /*!< EXTI 12 configuration */
#define AFIO_EXTICR4_EXTI13_Pos              (4U)
#define AFIO_EXTICR4_EXTI13_Msk              (0xFUL << AFIO_EXTICR4_EXTI13_Pos) /*!< 0x000000F0 */
#define AFIO_EXTICR4_EXTI13                  AFIO_EXTICR4_EXTI13_Msk           /*!< EXTI 13 configuration */
#define AFIO_EXTICR4_EXTI14_Pos              (8U)
#define AFIO_EXTICR4_EXTI14_Msk              (0xFUL << AFIO_EXTICR4_EXTI14_Pos) /*!< 0x00000F00 */
#define AFIO_EXTICR4_EXTI14                  AFIO_EXTICR4_EXTI14_Msk           /*!< EXTI 14 configuration */
#define AFIO_EXTICR4_EXTI15_Pos              (12U)
#define AFIO_EXTICR4_EXTI15_Msk              (0xFUL << AFIO_EXTICR4_EXTI15_Pos) /*!< 0x0000F000 */
#define AFIO_EXTICR4_EXTI15                  AFIO_EXTICR4_EXTI15_Msk           /*!< EXTI 15 configuration */

/* EXTI12 configuration */
#define AFIO_EXTICR4_EXTI12_PA               0x00000000U                          /*!< PA[12] pin */
#define AFIO_EXTICR4_EXTI12_PB_Pos           (0U)
#define AFIO_EXTICR4_EXTI12_PB_Msk           (0x1UL << AFIO_EXTICR4_EXTI12_PB_Pos) /*!< 0x00000001 */
#define AFIO_EXTICR4_EXTI12_PB               AFIO_EXTICR4_EXTI12_PB_Msk        /*!< PB[12] pin */
#define AFIO_EXTICR4_EXTI12_PC_Pos           (1U)
#define AFIO_EXTICR4_EXTI12_PC_Msk           (0x1UL << AFIO_EXTICR4_EXTI12_PC_Pos) /*!< 0x00000002 */
#define AFIO_EXTICR4_EXTI12_PC               AFIO_EXTICR4_EXTI12_PC_Msk        /*!< PC[12] pin */
#define AFIO_EXTICR4_EXTI12_PD_Pos           (0U)
#define AFIO_EXTICR4_EXTI12_PD_Msk           (0x3UL << AFIO_EXTICR4_EXTI12_PD_Pos) /*!< 0x00000003 */
#define AFIO_EXTICR4_EXTI12_PD               AFIO_EXTICR4_EXTI12_PD_Msk        /*!< PD[12] pin */
#define AFIO_EXTICR4_EXTI12_PE_Pos           (2U)
#define AFIO_EXTICR4_EXTI12_PE_Msk           (0x1UL << AFIO_EXTICR4_EXTI12_PE_Pos) /*!< 0x00000004 */
#define AFIO_EXTICR4_EXTI12_PE               AFIO_EXTICR4_EXTI12_PE_Msk        /*!< PE[12] pin */
#define AFIO_EXTICR4_EXTI12_PF_Pos           (0U)
#define AFIO_EXTICR4_EXTI12_PF_Msk           (0x5UL << AFIO_EXTICR4_EXTI12_PF_Pos) /*!< 0x00000005 */
#define AFIO_EXTICR4_EXTI12_PF               AFIO_EXTICR4_EXTI12_PF_Msk        /*!< PF[12] pin */
#define AFIO_EXTICR4_EXTI12_PG_Pos           (1U)
#define AFIO_EXTICR4_EXTI12_PG_Msk           (0x3UL << AFIO_EXTICR4_EXTI12_PG_Pos) /*!< 0x00000006 */
#define AFIO_EXTICR4_EXTI12_PG               AFIO_EXTICR4_EXTI12_PG_Msk        /*!< PG[12] pin */

/* EXTI13 configuration */
#define AFIO_EXTICR4_EXTI13_PA               0x00000000U                          /*!< PA[13] pin */
#define AFIO_EXTICR4_EXTI13_PB_Pos           (4U)
#define AFIO_EXTICR4_EXTI13_PB_Msk           (0x1UL << AFIO_EXTICR4_EXTI13_PB_Pos) /*!< 0x00000010 */
#define AFIO_EXTICR4_EXTI13_PB               AFIO_EXTICR4_EXTI13_PB_Msk        /*!< PB[13] pin */
#define AFIO_EXTICR4_EXTI13_PC_Pos           (5U)
#define AFIO_EXTICR4_EXTI13_PC_Msk           (0x1UL << AFIO_EXTICR4_EXTI13_PC_Pos) /*!< 0x00000020 */
#define AFIO_EXTICR4_EXTI13_PC               AFIO_EXTICR4_EXTI13_PC_Msk        /*!< PC[13] pin */
#define AFIO_EXTICR4_EXTI13_PD_Pos           (4U)
#define AFIO_EXTICR4_EXTI13_PD_Msk           (0x3UL << AFIO_EXTICR4_EXTI13_PD_Pos) /*!< 0x00000030 */
#define AFIO_EXTICR4_EXTI13_PD               AFIO_EXTICR4_EXTI13_PD_Msk        /*!< PD[13] pin */
#define AFIO_EXTICR4_EXTI13_PE_Pos           (6U)
#define AFIO_EXTICR4_EXTI13_PE_Msk           (0x1UL << AFIO_EXTICR4_EXTI13_PE_Pos) /*!< 0x00000040 */
#define AFIO_EXTICR4_EXTI13_PE               AFIO_EXTICR4_EXTI13_PE_Msk        /*!< PE[13] pin */
#define AFIO_EXTICR4_EXTI13_PF_Pos           (4U)
#define AFIO_EXTICR4_EXTI13_PF_Msk           (0x5UL << AFIO_EXTICR4_EXTI13_PF_Pos) /*!< 0x00000050 */
#define AFIO_EXTICR4_EXTI13_PF               AFIO_EXTICR4_EXTI13_PF_Msk        /*!< PF[13] pin */
#define AFIO_EXTICR4_EXTI13_PG_Pos           (5U)
#define AFIO_EXTICR4_EXTI13_PG_Msk           (0x3UL << AFIO_EXTICR4_EXTI13_PG_Pos) /*!< 0x00000060 */
#define AFIO_EXTICR4_EXTI13_PG               AFIO_EXTICR4_EXTI13_PG_Msk        /*!< PG[13] pin */

/*!< EXTI14 configuration */
#define AFIO_EXTICR4_EXTI14_PA               0x00000000U                          /*!< PA[14] pin */
#define AFIO_EXTICR4_EXTI14_PB_Pos           (8U)
#define AFIO_EXTICR4_EXTI14_PB_Msk           (0x1UL << AFIO_EXTICR4_EXTI14_PB_Pos) /*!< 0x00000100 */
#define AFIO_EXTICR4_EXTI14_PB               AFIO_EXTICR4_EXTI14_PB_Msk        /*!< PB[14] pin */
#define AFIO_EXTICR4_EXTI14_PC_Pos           (9U)
#define AFIO_EXTICR4_EXTI14_PC_Msk           (0x1UL << AFIO_EXTICR4_EXTI14_PC_Pos) /*!< 0x00000200 */
#define AFIO_EXTICR4_EXTI14_PC               AFIO_EXTICR4_EXTI14_PC_Msk        /*!< PC[14] pin */
#define AFIO_EXTICR4_EXTI14_PD_Pos           (8U)
#define AFIO_EXTICR4_EXTI14_PD_Msk           (0x3UL << AFIO_EXTICR4_EXTI14_PD_Pos) /*!< 0x00000300 */
#define AFIO_EXTICR4_EXTI14_PD               AFIO_EXTICR4_EXTI14_PD_Msk        /*!< PD[14] pin */
#define AFIO_EXTICR4_EXTI14_PE_Pos           (10U)
#define AFIO_EXTICR4_EXTI14_PE_Msk           (0x1UL << AFIO_EXTICR4_EXTI14_PE_Pos) /*!< 0x00000400 */
#define AFIO_EXTICR4_EXTI14_PE               AFIO_EXTICR4_EXTI14_PE_Msk        /*!< PE[14] pin */
#define AFIO_EXTICR4_EXTI14_PF_Pos           (8U)
#define AFIO_EXTICR4_EXTI14_PF_Msk           (0x5UL << AFIO_EXTICR4_EXTI14_PF_Pos) /*!< 0x00000500 */
#define AFIO_EXTICR4_EXTI14_PF               AFIO_EXTICR4_EXTI14_PF_Msk        /*!< PF[14] pin */
#define AFIO_EXTICR4_EXTI14_PG_Pos           (9U)
#define AFIO_EXTICR4_EXTI14_PG_Msk           (0x3UL << AFIO_EXTICR4_EXTI14_PG_Pos) /*!< 0x00000600 */
#define AFIO_EXTICR4_EXTI14_PG               AFIO_EXTICR4_EXTI14_PG_Msk        /*!< PG[14] pin */

/*!< EXTI15 configuration */
#define AFIO_EXTICR4_EXTI15_PA               0x00000000U                          /*!< PA[15] pin */
#define AFIO_EXTICR4_EXTI15_PB_Pos           (12U)
#define AFIO_EXTICR4_EXTI15_PB_Msk           (0x1UL << AFIO_EXTICR4_EXTI15_PB_Pos) /*!< 0x00001000 */
#define AFIO_EXTICR4_EXTI15_PB               AFIO_EXTICR4_EXTI15_PB_Msk        /*!< PB[15] pin */
#define AFIO_EXTICR4_EXTI15_PC_Pos           (13U)
#define AFIO_EXTICR4_EXTI15_PC_Msk           (0x1UL << AFIO_EXTICR4_EXTI15_PC_Pos) /*!< 0x00002000 */
#define AFIO_EXTICR4_EXTI15_PC               AFIO_EXTICR4_EXTI15_PC_Msk        /*!< PC[15] pin */
#define AFIO_EXTICR4_EXTI15_PD_Pos           (12U)
#define AFIO_EXTICR4_EXTI15_PD_Msk           (0x3UL << AFIO_EXTICR4_EXTI15_PD_Pos) /*!< 0x00003000 */
#define AFIO_EXTICR4_EXTI15_PD               AFIO_EXTICR4_EXTI15_PD_Msk        /*!< PD[15] pin */
#define AFIO_EXTICR4_EXTI15_PE_Pos           (14U)
#define AFIO_EXTICR4_EXTI15_PE_Msk           (0x1UL << AFIO_EXTICR4_EXTI15_PE_Pos) /*!< 0x00004000 */
#define AFIO_EXTICR4_EXTI15_PE               AFIO_EXTICR4_EXTI15_PE_Msk        /*!< PE[15] pin */
#define AFIO_EXTICR4_EXTI15_PF_Pos           (12U)
#define AFIO_EXTICR4_EXTI15_PF_Msk           (0x5UL << AFIO_EXTICR4_EXTI15_PF_Pos) /*!< 0x00005000 */
#define AFIO_EXTICR4_EXTI15_PF               AFIO_EXTICR4_EXTI15_PF_Msk        /*!< PF[15] pin */
#define AFIO_EXTICR4_EXTI15_PG_Pos           (13U)
#define AFIO_EXTICR4_EXTI15_PG_Msk           (0x3UL << AFIO_EXTICR4_EXTI15_PG_Pos) /*!< 0x00006000 */
#define AFIO_EXTICR4_EXTI15_PG               AFIO_EXTICR4_EXTI15_PG_Msk        /*!< PG[15] pin */

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for RCC_CR register  ********************/
#define RCC_CR_HSION_Pos                     (0U)
#define RCC_CR_HSION_Msk                     (0x1UL << RCC_CR_HSION_Pos)        /*!< 0x00000001 */
#define RCC_CR_HSION                         RCC_CR_HSION_Msk                  /*!< Internal High Speed clock enable */
#define RCC_CR_HSIRDY_Pos                    (1U)
#define RCC_CR_HSIRDY_Msk                    (0x1UL << RCC_CR_HSIRDY_Pos)       /*!< 0x00000002 */
#define RCC_CR_HSIRDY                        RCC_CR_HSIRDY_Msk                 /*!< Internal High Speed clock ready flag */
#define RCC_CR_HSITRIM_Pos                   (3U)
#define RCC_CR_HSITRIM_Msk                   (0x1FUL << RCC_CR_HSITRIM_Pos)     /*!< 0x000000F8 */
#define RCC_CR_HSITRIM                       RCC_CR_HSITRIM_Msk                /*!< Internal High Speed clock trimming */
#define RCC_CR_HSICAL_Pos                    (8U)
#define RCC_CR_HSICAL_Msk                    (0xFFUL << RCC_CR_HSICAL_Pos)      /*!< 0x0000FF00 */
#define RCC_CR_HSICAL                        RCC_CR_HSICAL_Msk                 /*!< Internal High Speed clock Calibration */
#define RCC_CR_HSEON_Pos                     (16U)
#define RCC_CR_HSEON_Msk                     (0x1UL << RCC_CR_HSEON_Pos)        /*!< 0x00010000 */
#define RCC_CR_HSEON                         RCC_CR_HSEON_Msk                  /*!< External High Speed clock enable */
#define RCC_CR_HSERDY_Pos                    (17U)
#define RCC_CR_HSERDY_Msk                    (0x1UL << RCC_CR_HSERDY_Pos)       /*!< 0x00020000 */
#define RCC_CR_HSERDY                        RCC_CR_HSERDY_Msk                 /*!< External High Speed clock ready flag */
#define RCC_CR_HSEBYP_Pos                    (18U)
#define RCC_CR_HSEBYP_Msk                    (0x1UL << RCC_CR_HSEBYP_Pos)       /*!< 0x00040000 */
#define RCC_CR_HSEBYP                        RCC_CR_HSEBYP_Msk                 /*!< External High Speed clock Bypass */
#define RCC_CR_CSSON_Pos                     (19U)
#define RCC_CR_CSSON_Msk                     (0x1UL << RCC_CR_CSSON_Pos)        /*!< 0x00080000 */
#define RCC_CR_CSSON                         RCC_CR_CSSON_Msk                  /*!< Clock Security System enable */
#define RCC_CR_PLLON_Pos                     (24U)
#define RCC_CR_PLLON_Msk                     (0x1UL << RCC_CR_PLLON_Pos)        /*!< 0x01000000 */
#define RCC_CR_PLLON                         RCC_CR_PLLON_Msk                  /*!< PLL enable */
#define RCC_CR_PLLRDY_Pos                    (25U)
#define RCC_CR_PLLRDY_Msk                    (0x1UL << RCC_CR_PLLRDY_Pos)       /*!< 0x02000000 */
#define RCC_CR_PLLRDY                        RCC_CR_PLLRDY_Msk                 /*!< PLL clock ready flag */


/*******************  Bit definition for RCC_CFGR register  *******************/
/*!< SW configuration */
#define RCC_CFGR_SW_Pos                      (0U)
#define RCC_CFGR_SW_Msk                      (0x3UL << RCC_CFGR_SW_Pos)         /*!< 0x00000003 */
#define RCC_CFGR_SW                          RCC_CFGR_SW_Msk                   /*!< SW[1:0] bits (System clock Switch) */
#define RCC_CFGR_SW_0                        (0x1UL << RCC_CFGR_SW_Pos)         /*!< 0x00000001 */
#define RCC_CFGR_SW_1                        (0x2UL << RCC_CFGR_SW_Pos)         /*!< 0x00000002 */

#define RCC_CFGR_SW_HSI                      0x00000000U                       /*!< HSI selected as system clock */
#define RCC_CFGR_SW_HSE                      0x00000001U                       /*!< HSE selected as system clock */
#define RCC_CFGR_SW_PLL                      0x00000002U                       /*!< PLL selected as system clock */

/*!< SWS configuration */
#define RCC_CFGR_SWS_Pos                     (2U)
#define RCC_CFGR_SWS_Msk                     (0x3UL << RCC_CFGR_SWS_Pos)        /*!< 0x0000000C */
#define RCC_CFGR_SWS                         RCC_CFGR_SWS_Msk                  /*!< SWS[1:0] bits (System Clock Switch Status) */
#define RCC_CFGR_SWS_0                       (0x1UL << RCC_CFGR_SWS_Pos)        /*!< 0x00000004 */
#define RCC_CFGR_SWS_1                       (0x2UL << RCC_CFGR_SWS_Pos)        /*!< 0x00000008 */

#define RCC_CFGR_SWS_HSI                     0x00000000U                       /*!< HSI oscillator used as system clock */
#define RCC_CFGR_SWS_HSE                     0x00000004U                       /*!< HSE oscillator used as system clock */
#define RCC_CFGR_SWS_PLL                     0x00000008U                       /*!< PLL used as system clock */

/*!< HPRE configuration */
#define RCC_CFGR_HPRE_Pos                    (4U)
#define RCC_CFGR_HPRE_Msk                    (0xFUL << RCC_CFGR_HPRE_Pos)       /*!< 0x000000F0 */
#define RCC_CFGR_HPRE                        RCC_CFGR_HPRE_Msk                 /*!< HPRE[3:0] bits (AHB prescaler) */
#define RCC_CFGR_HPRE_0                      (0x1UL << RCC_CFGR_HPRE_Pos)       /*!< 0x00000010 */
#define RCC_CFGR_HPRE_1                      (0x2UL << RCC_CFGR_HPRE_Pos)       /*!< 0x00000020 */
#define RCC_CFGR_HPRE_2                      (0x4UL << RCC_CFGR_HPRE_Pos)       /*!< 0x00000040 */
#define RCC_CFGR_HPRE_3                      (0x8UL << RCC_CFGR_HPRE_Pos)       /*!< 0x00000080 */

#define RCC_CFGR_HPRE_DIV1                   0x00000000U                       /*!< SYSCLK not divided */
#define RCC_CFGR_HPRE_DIV2                   0x00000080U                       /*!< SYSCLK divided by 2 */
#define RCC_CFGR_HPRE_DIV4                   0x00000090U                       /*!< SYSCLK divided by 4 */
#define RCC_CFGR_HPRE_DIV8                   0x000000A0U                       /*!< SYSCLK divided by 8 */
#define RCC_CFGR_HPRE_DIV16                  0x000000B0U                       /*!< SYSCLK divided by 16 */
#define RCC_CFGR_HPRE_DIV64                  0x000000C0U                       /*!< SYSCLK divided by 64 */
#define RCC_CFGR_HPRE_DIV128                 0x000000D0U                       /*!< SYSCLK divided by 128 */
#define RCC_CFGR_HPRE_DIV256                 0x000000E0U                       /*!< SYSCLK divided by 256 */
#define RCC_CFGR_HPRE_DIV512                 0x000000F0U                       /*!< SYSCLK divided by 512 */

/*!< PPRE1 configuration */
#define RCC_CFGR_PPRE1_Pos                   (8U)
#define RCC_CFGR_PPRE1_Msk                   (0x7UL << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000700 */
#define RCC_CFGR_PPRE1                       RCC_CFGR_PPRE1_Msk                /*!< PRE1[2:0] bits (APB1 prescaler) */
#define RCC_CFGR_PPRE1_0                     (0x1UL << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000100 */
#define RCC_CFGR_PPRE1_1                     (0x2UL << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000200 */
#define RCC_CFGR_PPRE1_2                     (0x4UL << RCC_CFGR_PPRE1_Pos)      /*!< 0x00000400 */

#define RCC_CFGR_PPRE1_DIV1                  0x00000000U                       /*!< HCLK not divided */
#define RCC_CFGR_PPRE1_DIV2                  0x00000400U                       /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE1_DIV4                  0x00000500U                       /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE1_DIV8                  0x00000600U                       /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE1_DIV16                 0x00000700U                       /*!< HCLK divided by 16 */

/*!< PPRE2 configuration */
#define RCC_CFGR_PPRE2_Pos                   (11U)
#define RCC_CFGR_PPRE2_Msk                   (0x7UL << RCC_CFGR_PPRE2_Pos)      /*!< 0x00003800 */
#define RCC_CFGR_PPRE2                       RCC_CFGR_PPRE2_Msk                /*!< PRE2[2:0] bits (APB2 prescaler) */
#define RCC_CFGR_PPRE2_0                     (0x1UL << RCC_CFGR_PPRE2_Pos)      /*!< 0x00000800 */
#define RCC_CFGR_PPRE2_1                     (0x2UL << RCC_CFGR_PPRE2_Pos)      /*!< 0x00001000 */
#define RCC_CFGR_PPRE2_2                     (0x4UL << RCC_CFGR_PPRE2_Pos)      /*!< 0x00002000 */

#define RCC_CFGR_PPRE2_DIV1                  0x00000000U                       /*!< HCLK not divided */
#define RCC_CFGR_PPRE2_DIV2                  0x00002000U                       /*!< HCLK divided by 2 */
#define RCC_CFGR_PPRE2_DIV4                  0x00002800U                       /*!< HCLK divided by 4 */
#define RCC_CFGR_PPRE2_DIV8                  0x00003000U                       /*!< HCLK divided by 8 */
#define RCC_CFGR_PPRE2_DIV16                 0x00003800U                       /*!< HCLK divided by 16 */

/*!< ADCPPRE configuration */
#define RCC_CFGR_ADCPRE_Pos                  (14U)
#define RCC_CFGR_ADCPRE_Msk                  (0x3UL << RCC_CFGR_ADCPRE_Pos)     /*!< 0x0000C000 */
#define RCC_CFGR_ADCPRE                      RCC_CFGR_ADCPRE_Msk               /*!< ADCPRE[1:0] bits (ADC prescaler) */
#define RCC_CFGR_ADCPRE_0                    (0x1UL << RCC_CFGR_ADCPRE_Pos)     /*!< 0x00004000 */
#define RCC_CFGR_ADCPRE_1                    (0x2UL << RCC_CFGR_ADCPRE_Pos)     /*!< 0x00008000 */

#define RCC_CFGR_ADCPRE_DIV2                 0x00000000U                       /*!< PCLK2 divided by 2 */
#define RCC_CFGR_ADCPRE_DIV4                 0x00004000U                       /*!< PCLK2 divided by 4 */
#define RCC_CFGR_ADCPRE_DIV6                 0x00008000U                       /*!< PCLK2 divided by 6 */
#define RCC_CFGR_ADCPRE_DIV8                 0x0000C000U                       /*!< PCLK2 divided by 8 */

#define RCC_CFGR_PLLSRC_Pos                  (16U)
#define RCC_CFGR_PLLSRC_Msk                  (0x1UL << RCC_CFGR_PLLSRC_Pos)     /*!< 0x00010000 */
#define RCC_CFGR_PLLSRC                      RCC_CFGR_PLLSRC_Msk               /*!< PLL entry clock source */

#define RCC_CFGR_PLLXTPRE_Pos                (17U)
#define RCC_CFGR_PLLXTPRE_Msk                (0x1UL << RCC_CFGR_PLLXTPRE_Pos)   /*!< 0x00020000 */
#define RCC_CFGR_PLLXTPRE                    RCC_CFGR_PLLXTPRE_Msk             /*!< HSE divider for PLL entry */

/*!< PLLMUL configuration */
#define RCC_CFGR_PLLMULL_Pos                 (18U)
#define RCC_CFGR_PLLMULL_Msk                 (0xFUL << RCC_CFGR_PLLMULL_Pos)    /*!< 0x003C0000 */
#define RCC_CFGR_PLLMULL                     RCC_CFGR_PLLMULL_Msk              /*!< PLLMUL[3:0] bits (PLL multiplication factor) */
#define RCC_CFGR_PLLMULL_0                   (0x1UL << RCC_CFGR_PLLMULL_Pos)    /*!< 0x00040000 */
#define RCC_CFGR_PLLMULL_1                   (0x2UL << RCC_CFGR_PLLMULL_Pos)    /*!< 0x00080000 */
#define RCC_CFGR_PLLMULL_2                   (0x4UL << RCC_CFGR_PLLMULL_Pos)    /*!< 0x00100000 */
#define RCC_CFGR_PLLMULL_3                   (0x8UL << RCC_CFGR_PLLMULL_Pos)    /*!< 0x00200000 */

#define RCC_CFGR_PLLXTPRE_HSE                0x00000000U                      /*!< HSE clock not divided for PLL entry */
#define RCC_CFGR_PLLXTPRE_HSE_DIV2           0x00020000U                      /*!< HSE clock divided by 2 for PLL entry */

#define RCC_CFGR_PLLMULL2                    0x00000000U                       /*!< PLL input clock*2 */
#define RCC_CFGR_PLLMULL3_Pos                (18U)
#define RCC_CFGR_PLLMULL3_Msk                (0x1UL << RCC_CFGR_PLLMULL3_Pos)   /*!< 0x00040000 */
#define RCC_CFGR_PLLMULL3                    RCC_CFGR_PLLMULL3_Msk             /*!< PLL input clock*3 */
#define RCC_CFGR_PLLMULL4_Pos                (19U)
#define RCC_CFGR_PLLMULL4_Msk                (0x1UL << RCC_CFGR_PLLMULL4_Pos)   /*!< 0x00080000 */
#define RCC_CFGR_PLLMULL4                    RCC_CFGR_PLLMULL4_Msk             /*!< PLL input clock*4 */
#define RCC_CFGR_PLLMULL5_Pos                (18U)
#define RCC_CFGR_PLLMULL5_Msk                (0x3UL << RCC_CFGR_PLLMULL5_Pos)   /*!< 0x000C0000 */
#define RCC_CFGR_PLLMULL5                    RCC_CFGR_PLLMULL5_Msk             /*!< PLL input clock*5 */
#define RCC_CFGR_PLLMULL6_Pos                (20U)
#define RCC_CFGR_PLLMULL6_Msk                (0x1UL << RCC_CFGR_PLLMULL6_Pos)   /*!< 0x00100000 */
#define RCC_CFGR_PLLMULL6                    RCC_CFGR_PLLMULL6_Msk             /*!< PLL input clock*6 */
#define RCC_CFGR_PLLMULL7_Pos                (18U)
#define RCC_CFGR_PLLMULL7_Msk                (0x5UL << RCC_CFGR_PLLMULL7_Pos)   /*!< 0x00140000 */
#define RCC_CFGR_PLLMULL7                    RCC_CFGR_PLLMULL7_Msk             /*!< PLL input clock*7 */
#define RCC_CFGR_PLLMULL8_Pos                (19U)
#define RCC_CFGR_PLLMULL8_Msk                (0x3UL << RCC_CFGR_PLLMULL8_Pos)   /*!< 0x00180000 */
#define RCC_CFGR_PLLMULL8                    RCC_CFGR_PLLMULL8_Msk             /*!< PLL input clock*8 */
#define RCC_CFGR_PLLMULL9_Pos                (18U)
#define RCC_CFGR_PLLMULL9_Msk                (0x7UL << RCC_CFGR_PLLMULL9_Pos)   /*!< 0x001C0000 */
#define RCC_CFGR_PLLMULL9                    RCC_CFGR_PLLMULL9_Msk             /*!< PLL input clock*9 */
#define RCC_CFGR_PLLMULL10_Pos               (21U)
#define RCC_CFGR_PLLMULL10_Msk               (0x1UL << RCC_CFGR_PLLMULL10_Pos)  /*!< 0x00200000 */
#define RCC_CFGR_PLLMULL10                   RCC_CFGR_PLLMULL10_Msk            /*!< PLL input clock10 */
#define RCC_CFGR_PLLMULL11_Pos               (18U)
#define RCC_CFGR_PLLMULL11_Msk               (0x9UL << RCC_CFGR_PLLMULL11_Pos)  /*!< 0x00240000 */
#define RCC_CFGR_PLLMULL11                   RCC_CFGR_PLLMULL11_Msk            /*!< PLL input clock*11 */
#define RCC_CFGR_PLLMULL12_Pos               (19U)
#define RCC_CFGR_PLLMULL12_Msk               (0x5UL << RCC_CFGR_PLLMULL12_Pos)  /*!< 0x00280000 */
#define RCC_CFGR_PLLMULL12                   RCC_CFGR_PLLMULL12_Msk            /*!< PLL input clock*12 */
#define RCC_CFGR_PLLMULL13_Pos               (18U)
#define RCC_CFGR_PLLMULL13_Msk               (0xBUL << RCC_CFGR_PLLMULL13_Pos)  /*!< 0x002C0000 */
#define RCC_CFGR_PLLMULL13                   RCC_CFGR_PLLMULL13_Msk            /*!< PLL input clock*13 */
#define RCC_CFGR_PLLMULL14_Pos               (20U)
#define RCC_CFGR_PLLMULL14_Msk               (0x3UL << RCC_CFGR_PLLMULL14_Pos)  /*!< 0x00300000 */
#define RCC_CFGR_PLLMULL14                   RCC_CFGR_PLLMULL14_Msk            /*!< PLL input clock*14 */
#define RCC_CFGR_PLLMULL15_Pos               (18U)
#define RCC_CFGR_PLLMULL15_Msk               (0xDUL << RCC_CFGR_PLLMULL15_Pos)  /*!< 0x00340000 */
#define RCC_CFGR_PLLMULL15                   RCC_CFGR_PLLMULL15_Msk            /*!< PLL input clock*15 */
#define RCC_CFGR_PLLMULL16_Pos               (19U)
#define RCC_CFGR_PLLMULL16_Msk               (0x7UL << RCC_CFGR_PLLMULL16_Pos)  /*!< 0x00380000 */
#define RCC_CFGR_PLLMULL16                   RCC_CFGR_PLLMULL16_Msk            /*!< PLL input clock*16 */
#define RCC_CFGR_USBPRE_Pos                  (22U)
#define RCC_CFGR_USBPRE_Msk                  (0x1UL << RCC_CFGR_USBPRE_Pos)     /*!< 0x00400000 */
#define RCC_CFGR_USBPRE                      RCC_CFGR_USBPRE_Msk               /*!< USB Device prescaler */

/*!< MCO configuration */
#define RCC_CFGR_MCO_Pos                     (24U)
#define RCC_CFGR_MCO_Msk                     (0x7UL << RCC_CFGR_MCO_Pos)        /*!< 0x07000000 */
#define RCC_CFGR_MCO                         RCC_CFGR_MCO_Msk                  /*!< MCO[2:0] bits (Microcontroller Clock Output) */
#define RCC_CFGR_MCO_0                       (0x1UL << RCC_CFGR_MCO_Pos)        /*!< 0x01000000 */
#define RCC_CFGR_MCO_1                       (0x2UL << RCC_CFGR_MCO_Pos)        /*!< 0x02000000 */
#define RCC_CFGR_MCO_2                       (0x4UL << RCC_CFGR_MCO_Pos)        /*!< 0x04000000 */

#define RCC_CFGR_MCO_NOCLOCK                 0x00000000U                        /*!< No clock */
#define RCC_CFGR_MCO_SYSCLK                  0x04000000U                        /*!< System clock selected as MCO source */
#define RCC_CFGR_MCO_HSI                     0x05000000U                        /*!< HSI clock selected as MCO source */
#define RCC_CFGR_MCO_HSE                     0x06000000U                        /*!< HSE clock selected as MCO source  */
#define RCC_CFGR_MCO_PLLCLK_DIV2             0x07000000U                        /*!< PLL clock divided by 2 selected as MCO source */

 /* Reference defines */
 #define RCC_CFGR_MCOSEL                      RCC_CFGR_MCO
 #define RCC_CFGR_MCOSEL_0                    RCC_CFGR_MCO_0
 #define RCC_CFGR_MCOSEL_1                    RCC_CFGR_MCO_1
 #define RCC_CFGR_MCOSEL_2                    RCC_CFGR_MCO_2
 #define RCC_CFGR_MCOSEL_NOCLOCK              RCC_CFGR_MCO_NOCLOCK
 #define RCC_CFGR_MCOSEL_SYSCLK               RCC_CFGR_MCO_SYSCLK
 #define RCC_CFGR_MCOSEL_HSI                  RCC_CFGR_MCO_HSI
 #define RCC_CFGR_MCOSEL_HSE                  RCC_CFGR_MCO_HSE
 #define RCC_CFGR_MCOSEL_PLL_DIV2             RCC_CFGR_MCO_PLLCLK_DIV2

/*!<******************  Bit definition for RCC_CIR register  ********************/
#define RCC_CIR_LSIRDYF_Pos                  (0U)
#define RCC_CIR_LSIRDYF_Msk                  (0x1UL << RCC_CIR_LSIRDYF_Pos)     /*!< 0x00000001 */
#define RCC_CIR_LSIRDYF                      RCC_CIR_LSIRDYF_Msk               /*!< LSI Ready Interrupt flag */
#define RCC_CIR_LSERDYF_Pos                  (1U)
#define RCC_CIR_LSERDYF_Msk                  (0x1UL << RCC_CIR_LSERDYF_Pos)     /*!< 0x00000002 */
#define RCC_CIR_LSERDYF                      RCC_CIR_LSERDYF_Msk               /*!< LSE Ready Interrupt flag */
#define RCC_CIR_HSIRDYF_Pos                  (2U)
#define RCC_CIR_HSIRDYF_Msk                  (0x1UL << RCC_CIR_HSIRDYF_Pos)     /*!< 0x00000004 */
#define RCC_CIR_HSIRDYF                      RCC_CIR_HSIRDYF_Msk               /*!< HSI Ready Interrupt flag */
#define RCC_CIR_HSERDYF_Pos                  (3U)
#define RCC_CIR_HSERDYF_Msk                  (0x1UL << RCC_CIR_HSERDYF_Pos)     /*!< 0x00000008 */
#define RCC_CIR_HSERDYF                      RCC_CIR_HSERDYF_Msk               /*!< HSE Ready Interrupt flag */
#define RCC_CIR_PLLRDYF_Pos                  (4U)
#define RCC_CIR_PLLRDYF_Msk                  (0x1UL << RCC_CIR_PLLRDYF_Pos)     /*!< 0x00000010 */
#define RCC_CIR_PLLRDYF                      RCC_CIR_PLLRDYF_Msk               /*!< PLL Ready Interrupt flag */
#define RCC_CIR_CSSF_Pos                     (7U)
#define RCC_CIR_CSSF_Msk                     (0x1UL << RCC_CIR_CSSF_Pos)        /*!< 0x00000080 */
#define RCC_CIR_CSSF                         RCC_CIR_CSSF_Msk                  /*!< Clock Security System Interrupt flag */
#define RCC_CIR_LSIRDYIE_Pos                 (8U)
#define RCC_CIR_LSIRDYIE_Msk                 (0x1UL << RCC_CIR_LSIRDYIE_Pos)    /*!< 0x00000100 */
#define RCC_CIR_LSIRDYIE                     RCC_CIR_LSIRDYIE_Msk              /*!< LSI Ready Interrupt Enable */
#define RCC_CIR_LSERDYIE_Pos                 (9U)
#define RCC_CIR_LSERDYIE_Msk                 (0x1UL << RCC_CIR_LSERDYIE_Pos)    /*!< 0x00000200 */
#define RCC_CIR_LSERDYIE                     RCC_CIR_LSERDYIE_Msk              /*!< LSE Ready Interrupt Enable */
#define RCC_CIR_HSIRDYIE_Pos                 (10U)
#define RCC_CIR_HSIRDYIE_Msk                 (0x1UL << RCC_CIR_HSIRDYIE_Pos)    /*!< 0x00000400 */
#define RCC_CIR_HSIRDYIE                     RCC_CIR_HSIRDYIE_Msk              /*!< HSI Ready Interrupt Enable */
#define RCC_CIR_HSERDYIE_Pos                 (11U)
#define RCC_CIR_HSERDYIE_Msk                 (0x1UL << RCC_CIR_HSERDYIE_Pos)    /*!< 0x00000800 */
#define RCC_CIR_HSERDYIE                     RCC_CIR_HSERDYIE_Msk              /*!< HSE Ready Interrupt Enable */
#define RCC_CIR_PLLRDYIE_Pos                 (12U)
#define RCC_CIR_PLLRDYIE_Msk                 (0x1UL << RCC_CIR_PLLRDYIE_Pos)    /*!< 0x00001000 */
#define RCC_CIR_PLLRDYIE                     RCC_CIR_PLLRDYIE_Msk              /*!< PLL Ready Interrupt Enable */
#define RCC_CIR_LSIRDYC_Pos                  (16U)
#define RCC_CIR_LSIRDYC_Msk                  (0x1UL << RCC_CIR_LSIRDYC_Pos)     /*!< 0x00010000 */
#define RCC_CIR_LSIRDYC                      RCC_CIR_LSIRDYC_Msk               /*!< LSI Ready Interrupt Clear */
#define RCC_CIR_LSERDYC_Pos                  (17U)
#define RCC_CIR_LSERDYC_Msk                  (0x1UL << RCC_CIR_LSERDYC_Pos)     /*!< 0x00020000 */
#define RCC_CIR_LSERDYC                      RCC_CIR_LSERDYC_Msk               /*!< LSE Ready Interrupt Clear */
#define RCC_CIR_HSIRDYC_Pos                  (18U)
#define RCC_CIR_HSIRDYC_Msk                  (0x1UL << RCC_CIR_HSIRDYC_Pos)     /*!< 0x00040000 */
#define RCC_CIR_HSIRDYC                      RCC_CIR_HSIRDYC_Msk               /*!< HSI Ready Interrupt Clear */
#define RCC_CIR_HSERDYC_Pos                  (19U)
#define RCC_CIR_HSERDYC_Msk                  (0x1UL << RCC_CIR_HSERDYC_Pos)     /*!< 0x00080000 */
#define RCC_CIR_HSERDYC                      RCC_CIR_HSERDYC_Msk               /*!< HSE Ready Interrupt Clear */
#define RCC_CIR_PLLRDYC_Pos                  (20U)
#define RCC_CIR_PLLRDYC_Msk                  (0x1UL << RCC_CIR_PLLRDYC_Pos)     /*!< 0x00100000 */
#define RCC_CIR_PLLRDYC                      RCC_CIR_PLLRDYC_Msk               /*!< PLL Ready Interrupt Clear */
#define RCC_CIR_CSSC_Pos                     (23U)
#define RCC_CIR_CSSC_Msk                     (0x1UL << RCC_CIR_CSSC_Pos)        /*!< 0x00800000 */
#define RCC_CIR_CSSC                         RCC_CIR_CSSC_Msk                  /*!< Clock Security System Interrupt Clear */


/*****************  Bit definition for RCC_APB2RSTR register  *****************/
#define RCC_APB2RSTR_AFIORST_Pos             (0U)
#define RCC_APB2RSTR_AFIORST_Msk             (0x1UL << RCC_APB2RSTR_AFIORST_Pos) /*!< 0x00000001 */
#define RCC_APB2RSTR_AFIORST                 RCC_APB2RSTR_AFIORST_Msk          /*!< Alternate Function I/O reset */
#define RCC_APB2RSTR_IOPARST_Pos             (2U)
#define RCC_APB2RSTR_IOPARST_Msk             (0x1UL << RCC_APB2RSTR_IOPARST_Pos) /*!< 0x00000004 */
#define RCC_APB2RSTR_IOPARST                 RCC_APB2RSTR_IOPARST_Msk          /*!< I/O port A reset */
#define RCC_APB2RSTR_IOPBRST_Pos             (3U)
#define RCC_APB2RSTR_IOPBRST_Msk             (0x1UL << RCC_APB2RSTR_IOPBRST_Pos) /*!< 0x00000008 */
#define RCC_APB2RSTR_IOPBRST                 RCC_APB2RSTR_IOPBRST_Msk          /*!< I/O port B reset */
#define RCC_APB2RSTR_IOPCRST_Pos             (4U)
#define RCC_APB2RSTR_IOPCRST_Msk             (0x1UL << RCC_APB2RSTR_IOPCRST_Pos) /*!< 0x00000010 */
#define RCC_APB2RSTR_IOPCRST                 RCC_APB2RSTR_IOPCRST_Msk          /*!< I/O port C reset */
#define RCC_APB2RSTR_IOPDRST_Pos             (5U)
#define RCC_APB2RSTR_IOPDRST_Msk             (0x1UL << RCC_APB2RSTR_IOPDRST_Pos) /*!< 0x00000020 */
#define RCC_APB2RSTR_IOPDRST                 RCC_APB2RSTR_IOPDRST_Msk          /*!< I/O port D reset */
#define RCC_APB2RSTR_ADC1RST_Pos             (9U)
#define RCC_APB2RSTR_ADC1RST_Msk             (0x1UL << RCC_APB2RSTR_ADC1RST_Pos) /*!< 0x00000200 */
#define RCC_APB2RSTR_ADC1RST                 RCC_APB2RSTR_ADC1RST_Msk          /*!< ADC 1 interface reset */

#define RCC_APB2RSTR_ADC2RST_Pos             (10U)
#define RCC_APB2RSTR_ADC2RST_Msk             (0x1UL << RCC_APB2RSTR_ADC2RST_Pos) /*!< 0x00000400 */
#define RCC_APB2RSTR_ADC2RST                 RCC_APB2RSTR_ADC2RST_Msk          /*!< ADC 2 interface reset */

#define RCC_APB2RSTR_TIM1RST_Pos             (11U)
#define RCC_APB2RSTR_TIM1RST_Msk             (0x1UL << RCC_APB2RSTR_TIM1RST_Pos) /*!< 0x00000800 */
#define RCC_APB2RSTR_TIM1RST                 RCC_APB2RSTR_TIM1RST_Msk          /*!< TIM1 Timer reset */
#define RCC_APB2RSTR_SPI1RST_Pos             (12U)
#define RCC_APB2RSTR_SPI1RST_Msk             (0x1UL << RCC_APB2RSTR_SPI1RST_Pos) /*!< 0x00001000 */
#define RCC_APB2RSTR_SPI1RST                 RCC_APB2RSTR_SPI1RST_Msk          /*!< SPI 1 reset */
#define RCC_APB2RSTR_USART1RST_Pos           (14U)
#define RCC_APB2RSTR_USART1RST_Msk           (0x1UL << RCC_APB2RSTR_USART1RST_Pos) /*!< 0x00004000 */
#define RCC_APB2RSTR_USART1RST               RCC_APB2RSTR_USART1RST_Msk        /*!< USART1 reset */


#define RCC_APB2RSTR_IOPERST_Pos             (6U)
#define RCC_APB2RSTR_IOPERST_Msk             (0x1UL << RCC_APB2RSTR_IOPERST_Pos) /*!< 0x00000040 */
#define RCC_APB2RSTR_IOPERST                 RCC_APB2RSTR_IOPERST_Msk          /*!< I/O port E reset */




/*****************  Bit definition for RCC_APB1RSTR register  *****************/
#define RCC_APB1RSTR_TIM2RST_Pos             (0U)
#define RCC_APB1RSTR_TIM2RST_Msk             (0x1UL << RCC_APB1RSTR_TIM2RST_Pos) /*!< 0x00000001 */
#define RCC_APB1RSTR_TIM2RST                 RCC_APB1RSTR_TIM2RST_Msk          /*!< Timer 2 reset */
#define RCC_APB1RSTR_TIM3RST_Pos             (1U)
#define RCC_APB1RSTR_TIM3RST_Msk             (0x1UL << RCC_APB1RSTR_TIM3RST_Pos) /*!< 0x00000002 */
#define RCC_APB1RSTR_TIM3RST                 RCC_APB1RSTR_TIM3RST_Msk          /*!< Timer 3 reset */
#define RCC_APB1RSTR_WWDGRST_Pos             (11U)
#define RCC_APB1RSTR_WWDGRST_Msk             (0x1UL << RCC_APB1RSTR_WWDGRST_Pos) /*!< 0x00000800 */
#define RCC_APB1RSTR_WWDGRST                 RCC_APB1RSTR_WWDGRST_Msk          /*!< Window Watchdog reset */
#define RCC_APB1RSTR_USART2RST_Pos           (17U)
#define RCC_APB1RSTR_USART2RST_Msk           (0x1UL << RCC_APB1RSTR_USART2RST_Pos) /*!< 0x00020000 */
#define RCC_APB1RSTR_USART2RST               RCC_APB1RSTR_USART2RST_Msk        /*!< USART 2 reset */
#define RCC_APB1RSTR_I2C1RST_Pos             (21U)
#define RCC_APB1RSTR_I2C1RST_Msk             (0x1UL << RCC_APB1RSTR_I2C1RST_Pos) /*!< 0x00200000 */
#define RCC_APB1RSTR_I2C1RST                 RCC_APB1RSTR_I2C1RST_Msk          /*!< I2C 1 reset */

#define RCC_APB1RSTR_CAN1RST_Pos             (25U)
#define RCC_APB1RSTR_CAN1RST_Msk             (0x1UL << RCC_APB1RSTR_CAN1RST_Pos) /*!< 0x02000000 */
#define RCC_APB1RSTR_CAN1RST                 RCC_APB1RSTR_CAN1RST_Msk          /*!< CAN1 reset */

#define RCC_APB1RSTR_BKPRST_Pos              (27U)
#define RCC_APB1RSTR_BKPRST_Msk              (0x1UL << RCC_APB1RSTR_BKPRST_Pos) /*!< 0x08000000 */
#define RCC_APB1RSTR_BKPRST                  RCC_APB1RSTR_BKPRST_Msk           /*!< Backup interface reset */
#define RCC_APB1RSTR_PWRRST_Pos              (28U)
#define RCC_APB1RSTR_PWRRST_Msk              (0x1UL << RCC_APB1RSTR_PWRRST_Pos) /*!< 0x10000000 */
#define RCC_APB1RSTR_PWRRST                  RCC_APB1RSTR_PWRRST_Msk           /*!< Power interface reset */

#define RCC_APB1RSTR_TIM4RST_Pos             (2U)
#define RCC_APB1RSTR_TIM4RST_Msk             (0x1UL << RCC_APB1RSTR_TIM4RST_Pos) /*!< 0x00000004 */
#define RCC_APB1RSTR_TIM4RST                 RCC_APB1RSTR_TIM4RST_Msk          /*!< Timer 4 reset */
#define RCC_APB1RSTR_SPI2RST_Pos             (14U)
#define RCC_APB1RSTR_SPI2RST_Msk             (0x1UL << RCC_APB1RSTR_SPI2RST_Pos) /*!< 0x00004000 */
#define RCC_APB1RSTR_SPI2RST                 RCC_APB1RSTR_SPI2RST_Msk          /*!< SPI 2 reset */
#define RCC_APB1RSTR_USART3RST_Pos           (18U)
#define RCC_APB1RSTR_USART3RST_Msk           (0x1UL << RCC_APB1RSTR_USART3RST_Pos) /*!< 0x00040000 */
#define RCC_APB1RSTR_USART3RST               RCC_APB1RSTR_USART3RST_Msk        /*!< USART 3 reset */
#define RCC_APB1RSTR_I2C2RST_Pos             (22U)
#define RCC_APB1RSTR_I2C2RST_Msk             (0x1UL << RCC_APB1RSTR_I2C2RST_Pos) /*!< 0x00400000 */
#define RCC_APB1RSTR_I2C2RST                 RCC_APB1RSTR_I2C2RST_Msk          /*!< I2C 2 reset */

#define RCC_APB1RSTR_USBRST_Pos              (23U)
#define RCC_APB1RSTR_USBRST_Msk              (0x1UL << RCC_APB1RSTR_USBRST_Pos) /*!< 0x00800000 */
#define RCC_APB1RSTR_USBRST                  RCC_APB1RSTR_USBRST_Msk           /*!< USB Device reset */






/******************  Bit definition for RCC_AHBENR register  ******************/
#define RCC_AHBENR_DMA1EN_Pos                (0U)
#define RCC_AHBENR_DMA1EN_Msk                (0x1UL << RCC_AHBENR_DMA1EN_Pos)   /*!< 0x00000001 */
#define RCC_AHBENR_DMA1EN                    RCC_AHBENR_DMA1EN_Msk             /*!< DMA1 clock enable */
#define RCC_AHBENR_SRAMEN_Pos                (2U)
#define RCC_AHBENR_SRAMEN_Msk                (0x1UL << RCC_AHBENR_SRAMEN_Pos)   /*!< 0x00000004 */
#define RCC_AHBENR_SRAMEN                    RCC_AHBENR_SRAMEN_Msk             /*!< SRAM interface clock enable */
#define RCC_AHBENR_FLITFEN_Pos               (4U)
#define RCC_AHBENR_FLITFEN_Msk               (0x1UL << RCC_AHBENR_FLITFEN_Pos)  /*!< 0x00000010 */
#define RCC_AHBENR_FLITFEN                   RCC_AHBENR_FLITFEN_Msk            /*!< FLITF clock enable */
#define RCC_AHBENR_CRCEN_Pos                 (6U)
#define RCC_AHBENR_CRCEN_Msk                 (0x1UL << RCC_AHBENR_CRCEN_Pos)    /*!< 0x00000040 */
#define RCC_AHBENR_CRCEN                     RCC_AHBENR_CRCEN_Msk              /*!< CRC clock enable */




/******************  Bit definition for RCC_APB2ENR register  *****************/
#define RCC_APB2ENR_AFIOEN_Pos               (0U)
#define RCC_APB2ENR_AFIOEN_Msk               (0x1UL << RCC_APB2ENR_AFIOEN_Pos)  /*!< 0x00000001 */
#define RCC_APB2ENR_AFIOEN                   RCC_APB2ENR_AFIOEN_Msk            /*!< Alternate Function I/O clock enable */
#define RCC_APB2ENR_IOPAEN_Pos               (2U)
#define RCC_APB2ENR_IOPAEN_Msk               (0x1UL << RCC_APB2ENR_IOPAEN_Pos)  /*!< 0x00000004 */
#define RCC_APB2ENR_IOPAEN                   RCC_APB2ENR_IOPAEN_Msk            /*!< I/O port A clock enable */
#define RCC_APB2ENR_IOPBEN_Pos               (3U)
#define RCC_APB2ENR_IOPBEN_Msk               (0x1UL << RCC_APB2ENR_IOPBEN_Pos)  /*!< 0x00000008 */
#define RCC_APB2ENR_IOPBEN                   RCC_APB2ENR_IOPBEN_Msk            /*!< I/O port B clock enable */
#define RCC_APB2ENR_IOPCEN_Pos               (4U)
#define RCC_APB2ENR_IOPCEN_Msk               (0x1UL << RCC_APB2ENR_IOPCEN_Pos)  /*!< 0x00000010 */
#define RCC_APB2ENR_IOPCEN                   RCC_APB2ENR_IOPCEN_Msk            /*!< I/O port C clock enable */
#define RCC_APB2ENR_IOPDEN_Pos               (5U)
#define RCC_APB2ENR_IOPDEN_Msk               (0x1UL << RCC_APB2ENR_IOPDEN_Pos)  /*!< 0x00000020 */
#define RCC_APB2ENR_IOPDEN                   RCC_APB2ENR_IOPDEN_Msk            /*!< I/O port D clock enable */
#define RCC_APB2ENR_ADC1EN_Pos               (9U)
#define RCC_APB2ENR_ADC1EN_Msk               (0x1UL << RCC_APB2ENR_ADC1EN_Pos)  /*!< 0x00000200 */
#define RCC_APB2ENR_ADC1EN                   RCC_APB2ENR_ADC1EN_Msk            /*!< ADC 1 interface clock enable */

#define RCC_APB2ENR_ADC2EN_Pos               (10U)
#define RCC_APB2ENR_ADC2EN_Msk               (0x1UL << RCC_APB2ENR_ADC2EN_Pos)  /*!< 0x00000400 */
#define RCC_APB2ENR_ADC2EN                   RCC_APB2ENR_ADC2EN_Msk            /*!< ADC 2 interface clock enable */

#define RCC_APB2ENR_TIM1EN_Pos               (11U)
#define RCC_APB2ENR_TIM1EN_Msk               (0x1UL << RCC_APB2ENR_TIM1EN_Pos)  /*!< 0x00000800 */
#define RCC_APB2ENR_TIM1EN                   RCC_APB2ENR_TIM1EN_Msk            /*!< TIM1 Timer clock enable */
#define RCC_APB2ENR_SPI1EN_Pos               (12U)
#define RCC_APB2ENR_SPI1EN_Msk               (0x1UL << RCC_APB2ENR_SPI1EN_Pos)  /*!< 0x00001000 */
#define RCC_APB2ENR_SPI1EN                   RCC_APB2ENR_SPI1EN_Msk            /*!< SPI 1 clock enable */
#define RCC_APB2ENR_USART1EN_Pos             (14U)
#define RCC_APB2ENR_USART1EN_Msk             (0x1UL << RCC_APB2ENR_USART1EN_Pos) /*!< 0x00004000 */
#define RCC_APB2ENR_USART1EN                 RCC_APB2ENR_USART1EN_Msk          /*!< USART1 clock enable */


#define RCC_APB2ENR_IOPEEN_Pos               (6U)
#define RCC_APB2ENR_IOPEEN_Msk               (0x1UL << RCC_APB2ENR_IOPEEN_Pos)  /*!< 0x00000040 */
#define RCC_APB2ENR_IOPEEN                   RCC_APB2ENR_IOPEEN_Msk            /*!< I/O port E clock enable */




/*****************  Bit definition for RCC_APB1ENR register  ******************/
#define RCC_APB1ENR_TIM2EN_Pos               (0U)
#define RCC_APB1ENR_TIM2EN_Msk               (0x1UL << RCC_APB1ENR_TIM2EN_Pos)  /*!< 0x00000001 */
#define RCC_APB1ENR_TIM2EN                   RCC_APB1ENR_TIM2EN_Msk            /*!< Timer 2 clock enabled*/
#define RCC_APB1ENR_TIM3EN_Pos               (1U)
#define RCC_APB1ENR_TIM3EN_Msk               (0x1UL << RCC_APB1ENR_TIM3EN_Pos)  /*!< 0x00000002 */
#define RCC_APB1ENR_TIM3EN                   RCC_APB1ENR_TIM3EN_Msk            /*!< Timer 3 clock enable */
#define RCC_APB1ENR_WWDGEN_Pos               (11U)
#define RCC_APB1ENR_WWDGEN_Msk               (0x1UL << RCC_APB1ENR_WWDGEN_Pos)  /*!< 0x00000800 */
#define RCC_APB1ENR_WWDGEN                   RCC_APB1ENR_WWDGEN_Msk            /*!< Window Watchdog clock enable */
#define RCC_APB1ENR_USART2EN_Pos             (17U)
#define RCC_APB1ENR_USART2EN_Msk             (0x1UL << RCC_APB1ENR_USART2EN_Pos) /*!< 0x00020000 */
#define RCC_APB1ENR_USART2EN                 RCC_APB1ENR_USART2EN_Msk          /*!< USART 2 clock enable */
#define RCC_APB1ENR_I2C1EN_Pos               (21U)
#define RCC_APB1ENR_I2C1EN_Msk               (0x1UL << RCC_APB1ENR_I2C1EN_Pos)  /*!< 0x00200000 */
#define RCC_APB1ENR_I2C1EN                   RCC_APB1ENR_I2C1EN_Msk            /*!< I2C 1 clock enable */

#define RCC_APB1ENR_CAN1EN_Pos               (25U)
#define RCC_APB1ENR_CAN1EN_Msk               (0x1UL << RCC_APB1ENR_CAN1EN_Pos)  /*!< 0x02000000 */
#define RCC_APB1ENR_CAN1EN                   RCC_APB1ENR_CAN1EN_Msk            /*!< CAN1 clock enable */

#define RCC_APB1ENR_BKPEN_Pos                (27U)
#define RCC_APB1ENR_BKPEN_Msk                (0x1UL << RCC_APB1ENR_BKPEN_Pos)   /*!< 0x08000000 */
#define RCC_APB1ENR_BKPEN                    RCC_APB1ENR_BKPEN_Msk             /*!< Backup interface clock enable */
#define RCC_APB1ENR_PWREN_Pos                (28U)
#define RCC_APB1ENR_PWREN_Msk                (0x1UL << RCC_APB1ENR_PWREN_Pos)   /*!< 0x10000000 */
#define RCC_APB1ENR_PWREN                    RCC_APB1ENR_PWREN_Msk             /*!< Power interface clock enable */

#define RCC_APB1ENR_TIM4EN_Pos               (2U)
#define RCC_APB1ENR_TIM4EN_Msk               (0x1UL << RCC_APB1ENR_TIM4EN_Pos)  /*!< 0x00000004 */
#define RCC_APB1ENR_TIM4EN                   RCC_APB1ENR_TIM4EN_Msk            /*!< Timer 4 clock enable */
#define RCC_APB1ENR_SPI2EN_Pos               (14U)
#define RCC_APB1ENR_SPI2EN_Msk               (0x1UL << RCC_APB1ENR_SPI2EN_Pos)  /*!< 0x00004000 */
#define RCC_APB1ENR_SPI2EN                   RCC_APB1ENR_SPI2EN_Msk            /*!< SPI 2 clock enable */
#define RCC_APB1ENR_USART3EN_Pos             (18U)
#define RCC_APB1ENR_USART3EN_Msk             (0x1UL << RCC_APB1ENR_USART3EN_Pos) /*!< 0x00040000 */
#define RCC_APB1ENR_USART3EN                 RCC_APB1ENR_USART3EN_Msk          /*!< USART 3 clock enable */
#define RCC_APB1ENR_I2C2EN_Pos               (22U)
#define RCC_APB1ENR_I2C2EN_Msk               (0x1UL << RCC_APB1ENR_I2C2EN_Pos)  /*!< 0x00400000 */
#define RCC_APB1ENR_I2C2EN                   RCC_APB1ENR_I2C2EN_Msk            /*!< I2C 2 clock enable */

#define RCC_APB1ENR_USBEN_Pos                (23U)
#define RCC_APB1ENR_USBEN_Msk                (0x1UL << RCC_APB1ENR_USBEN_Pos)   /*!< 0x00800000 */
#define RCC_APB1ENR_USBEN                    RCC_APB1ENR_USBEN_Msk             /*!< USB Device clock enable */






/*******************  Bit definition for RCC_BDCR register  *******************/
#define RCC_BDCR_LSEON_Pos                   (0U)
#define RCC_BDCR_LSEON_Msk                   (0x1UL << RCC_BDCR_LSEON_Pos)      /*!< 0x00000001 */
#define RCC_BDCR_LSEON                       RCC_BDCR_LSEON_Msk                /*!< External Low Speed oscillator enable */
#define RCC_BDCR_LSERDY_Pos                  (1U)
#define RCC_BDCR_LSERDY_Msk                  (0x1UL << RCC_BDCR_LSERDY_Pos)     /*!< 0x00000002 */
#define RCC_BDCR_LSERDY                      RCC_BDCR_LSERDY_Msk               /*!< External Low Speed oscillator Ready */
#define RCC_BDCR_LSEBYP_Pos                  (2U)
#define RCC_BDCR_LSEBYP_Msk                  (0x1UL << RCC_BDCR_LSEBYP_Pos)     /*!< 0x00000004 */
#define RCC_BDCR_LSEBYP                      RCC_BDCR_LSEBYP_Msk               /*!< External Low Speed oscillator Bypass */

#define RCC_BDCR_RTCSEL_Pos                  (8U)
#define RCC_BDCR_RTCSEL_Msk                  (0x3UL << RCC_BDCR_RTCSEL_Pos)     /*!< 0x00000300 */
#define RCC_BDCR_RTCSEL                      RCC_BDCR_RTCSEL_Msk               /*!< RTCSEL[1:0] bits (RTC clock source selection) */
#define RCC_BDCR_RTCSEL_0                    (0x1UL << RCC_BDCR_RTCSEL_Pos)     /*!< 0x00000100 */
#define RCC_BDCR_RTCSEL_1                    (0x2UL << RCC_BDCR_RTCSEL_Pos)     /*!< 0x00000200 */

/*!< RTC configuration */
#define RCC_BDCR_RTCSEL_NOCLOCK              0x00000000U                       /*!< No clock */
#define RCC_BDCR_RTCSEL_LSE                  0x00000100U                       /*!< LSE oscillator clock used as RTC clock */
#define RCC_BDCR_RTCSEL_LSI                  0x00000200U                       /*!< LSI oscillator clock used as RTC clock */
#define RCC_BDCR_RTCSEL_HSE                  0x00000300U                       /*!< HSE oscillator clock divided by 128 used as RTC clock */

#define RCC_BDCR_RTCEN_Pos                   (15U)
#define RCC_BDCR_RTCEN_Msk                   (0x1UL << RCC_BDCR_RTCEN_Pos)      /*!< 0x00008000 */
#define RCC_BDCR_RTCEN                       RCC_BDCR_RTCEN_Msk                /*!< RTC clock enable */
#define RCC_BDCR_BDRST_Pos                   (16U)
#define RCC_BDCR_BDRST_Msk                   (0x1UL << RCC_BDCR_BDRST_Pos)      /*!< 0x00010000 */
#define RCC_BDCR_BDRST                       RCC_BDCR_BDRST_Msk                /*!< Backup domain software reset  */

/*******************  Bit definition for RCC_CSR register  ********************/
#define RCC_CSR_LSION_Pos                    (0U)
#define RCC_CSR_LSION_Msk                    (0x1UL << RCC_CSR_LSION_Pos)       /*!< 0x00000001 */
#define RCC_CSR_LSION                        RCC_CSR_LSION_Msk                 /*!< Internal Low Speed oscillator enable */
#define RCC_CSR_LSIRDY_Pos                   (1U)
#define RCC_CSR_LSIRDY_Msk                   (0x1UL << RCC_CSR_LSIRDY_Pos)      /*!< 0x00000002 */
#define RCC_CSR_LSIRDY                       RCC_CSR_LSIRDY_Msk                /*!< Internal Low Speed oscillator Ready */
#define RCC_CSR_RMVF_Pos                     (24U)
#define RCC_CSR_RMVF_Msk                     (0x1UL << RCC_CSR_RMVF_Pos)        /*!< 0x01000000 */
#define RCC_CSR_RMVF                         RCC_CSR_RMVF_Msk                  /*!< Remove reset flag */
#define RCC_CSR_PINRSTF_Pos                  (26U)
#define RCC_CSR_PINRSTF_Msk                  (0x1UL << RCC_CSR_PINRSTF_Pos)     /*!< 0x04000000 */
#define RCC_CSR_PINRSTF                      RCC_CSR_PINRSTF_Msk               /*!< PIN reset flag */
#define RCC_CSR_PORRSTF_Pos                  (27U)
#define RCC_CSR_PORRSTF_Msk                  (0x1UL << RCC_CSR_PORRSTF_Pos)     /*!< 0x08000000 */
#define RCC_CSR_PORRSTF                      RCC_CSR_PORRSTF_Msk               /*!< POR/PDR reset flag */
#define RCC_CSR_SFTRSTF_Pos                  (28U)
#define RCC_CSR_SFTRSTF_Msk                  (0x1UL << RCC_CSR_SFTRSTF_Pos)     /*!< 0x10000000 */
#define RCC_CSR_SFTRSTF                      RCC_CSR_SFTRSTF_Msk               /*!< Software Reset flag */
#define RCC_CSR_IWDGRSTF_Pos                 (29U)
#define RCC_CSR_IWDGRSTF_Msk                 (0x1UL << RCC_CSR_IWDGRSTF_Pos)    /*!< 0x20000000 */
#define RCC_CSR_IWDGRSTF                     RCC_CSR_IWDGRSTF_Msk              /*!< Independent Watchdog reset flag */
#define RCC_CSR_WWDGRSTF_Pos                 (30U)
#define RCC_CSR_WWDGRSTF_Msk                 (0x1UL << RCC_CSR_WWDGRSTF_Pos)    /*!< 0x40000000 */
#define RCC_CSR_WWDGRSTF                     RCC_CSR_WWDGRSTF_Msk              /*!< Window watchdog reset flag */
#define RCC_CSR_LPWRRSTF_Pos                 (31U)
#define RCC_CSR_LPWRRSTF_Msk                 (0x1UL << RCC_CSR_LPWRRSTF_Pos)    /*!< 0x80000000 */
#define RCC_CSR_LPWRRSTF                     RCC_CSR_LPWRRSTF_Msk              /*!< Low-Power reset flag */

/******************************************************************************/
/*                                                                            */
/*                    External Interrupt/Event Controller                     */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for EXTI_IMR register  *******************/
#define EXTI_IMR_MR0_Pos                    (0U)
#define EXTI_IMR_MR0_Msk                    (0x1UL << EXTI_IMR_MR0_Pos)         /*!< 0x00000001 */
#define EXTI_IMR_MR0                        EXTI_IMR_MR0_Msk                   /*!< Interrupt Mask on line 0 */
#define EXTI_IMR_MR1_Pos                    (1U)
#define EXTI_IMR_MR1_Msk                    (0x1UL << EXTI_IMR_MR1_Pos)         /*!< 0x00000002 */
#define EXTI_IMR_MR1                        EXTI_IMR_MR1_Msk                   /*!< Interrupt Mask on line 1 */
#define EXTI_IMR_MR2_Pos                    (2U)
#define EXTI_IMR_MR2_Msk                    (0x1UL << EXTI_IMR_MR2_Pos)         /*!< 0x00000004 */
#define EXTI_IMR_MR2                        EXTI_IMR_MR2_Msk                   /*!< Interrupt Mask on line 2 */
#define EXTI_IMR_MR3_Pos                    (3U)
#define EXTI_IMR_MR3_Msk                    (0x1UL << EXTI_IMR_MR3_Pos)         /*!< 0x00000008 */
#define EXTI_IMR_MR3                        EXTI_IMR_MR3_Msk                   /*!< Interrupt Mask on line 3 */
#define EXTI_IMR_MR4_Pos                    (4U)
#define EXTI_IMR_MR4_Msk                    (0x1UL << EXTI_IMR_MR4_Pos)         /*!< 0x00000010 */
#define EXTI_IMR_MR4                        EXTI_IMR_MR4_Msk                   /*!< Interrupt Mask on line 4 */
#define EXTI_IMR_MR5_Pos                    (5U)
#define EXTI_IMR_MR5_Msk                    (0x1UL << EXTI_IMR_MR5_Pos)         /*!< 0x00000020 */
#define EXTI_IMR_MR5                        EXTI_IMR_MR5_Msk                   /*!< Interrupt Mask on line 5 */
#define EXTI_IMR_MR6_Pos                    (6U)
#define EXTI_IMR_MR6_Msk                    (0x1UL << EXTI_IMR_MR6_Pos)         /*!< 0x00000040 */
#define EXTI_IMR_MR6                        EXTI_IMR_MR6_Msk                   /*!< Interrupt Mask on line 6 */
#define EXTI_IMR_MR7_Pos                    (7U)
#define EXTI_IMR_MR7_Msk                    (0x1UL << EXTI_IMR_MR7_Pos)         /*!< 0x00000080 */
#define EXTI_IMR_MR7                        EXTI_IMR_MR7_Msk                   /*!< Interrupt Mask on line 7 */
#define EXTI_IMR_MR8_Pos                    (8U)
#define EXTI_IMR_MR8_Msk                    (0x1UL << EXTI_IMR_MR8_Pos)         /*!< 0x00000100 */
#define EXTI_IMR_MR8                        EXTI_IMR_MR8_Msk                   /*!< Interrupt Mask on line 8 */
#define EXTI_IMR_MR9_Pos                    (9U)
#define EXTI_IMR_MR9_Msk                    (0x1UL << EXTI_IMR_MR9_Pos)         /*!< 0x00000200 */
#define EXTI_IMR_MR9                        EXTI_IMR_MR9_Msk                   /*!< Interrupt Mask on line 9 */
#define EXTI_IMR_MR10_Pos                   (10U)
#define EXTI_IMR_MR10_Msk                   (0x1UL << EXTI_IMR_MR10_Pos)        /*!< 0x00000400 */
#define EXTI_IMR_MR10                       EXTI_IMR_MR10_Msk                  /*!< Interrupt Mask on line 10 */
#define EXTI_IMR_MR11_Pos                   (11U)
#define EXTI_IMR_MR11_Msk                   (0x1UL << EXTI_IMR_MR11_Pos)        /*!< 0x00000800 */
#define EXTI_IMR_MR11                       EXTI_IMR_MR11_Msk                  /*!< Interrupt Mask on line 11 */
#define EXTI_IMR_MR12_Pos                   (12U)
#define EXTI_IMR_MR12_Msk                   (0x1UL << EXTI_IMR_MR12_Pos)        /*!< 0x00001000 */
#define EXTI_IMR_MR12                       EXTI_IMR_MR12_Msk                  /*!< Interrupt Mask on line 12 */
#define EXTI_IMR_MR13_Pos                   (13U)
#define EXTI_IMR_MR13_Msk                   (0x1UL << EXTI_IMR_MR13_Pos)        /*!< 0x00002000 */
#define EXTI_IMR_MR13                       EXTI_IMR_MR13_Msk                  /*!< Interrupt Mask on line 13 */
#define EXTI_IMR_MR14_Pos                   (14U)
#define EXTI_IMR_MR14_Msk                   (0x1UL << EXTI_IMR_MR14_Pos)        /*!< 0x00004000 */
#define EXTI_IMR_MR14                       EXTI_IMR_MR14_Msk                  /*!< Interrupt Mask on line 14 */
#define EXTI_IMR_MR15_Pos                   (15U)
#define EXTI_IMR_MR15_Msk                   (0x1UL << EXTI_IMR_MR15_Pos)        /*!< 0x00008000 */
#define EXTI_IMR_MR15                       EXTI_IMR_MR15_Msk                  /*!< Interrupt Mask on line 15 */
#define EXTI_IMR_MR16_Pos                   (16U)
#define EXTI_IMR_MR16_Msk                   (0x1UL << EXTI_IMR_MR16_Pos)        /*!< 0x00010000 */
#define EXTI_IMR_MR16                       EXTI_IMR_MR16_Msk                  /*!< Interrupt Mask on line 16 */
#define EXTI_IMR_MR17_Pos                   (17U)
#define EXTI_IMR_MR17_Msk                   (0x1UL << EXTI_IMR_MR17_Pos)        /*!< 0x00020000 */
#define EXTI_IMR_MR17                       EXTI_IMR_MR17_Msk                  /*!< Interrupt Mask on line 17 */
#define EXTI_IMR_MR18_Pos                   (18U)
#define EXTI_IMR_MR18_Msk                   (0x1UL << EXTI_IMR_MR18_Pos)        /*!< 0x00040000 */
#define EXTI_IMR_MR18                       EXTI_IMR_MR18_Msk                  /*!< Interrupt Mask on line 18 */

/* References Defines */
#define  EXTI_IMR_IM0 EXTI_IMR_MR0
#define  EXTI_IMR_IM1 EXTI_IMR_MR1
#define  EXTI_IMR_IM2 EXTI_IMR_MR2
#define  EXTI_IMR_IM3 EXTI_IMR_MR3
#define  EXTI_IMR_IM4 EXTI_IMR_MR4
#define  EXTI_IMR_IM5 EXTI_IMR_MR5
#define  EXTI_IMR_IM6 EXTI_IMR_MR6
#define  EXTI_IMR_IM7 EXTI_IMR_MR7
#define  EXTI_IMR_IM8 EXTI_IMR_MR8
#define  EXTI_IMR_IM9 EXTI_IMR_MR9
#define  EXTI_IMR_IM10 EXTI_IMR_MR10
#define  EXTI_IMR_IM11 EXTI_IMR_MR11
#define  EXTI_IMR_IM12 EXTI_IMR_MR12
#define  EXTI_IMR_IM13 EXTI_IMR_MR13
#define  EXTI_IMR_IM14 EXTI_IMR_MR14
#define  EXTI_IMR_IM15 EXTI_IMR_MR15
#define  EXTI_IMR_IM16 EXTI_IMR_MR16
#define  EXTI_IMR_IM17 EXTI_IMR_MR17
#define  EXTI_IMR_IM18 EXTI_IMR_MR18
#define  EXTI_IMR_IM   0x0007FFFFU        /*!< Interrupt Mask All */

/*******************  Bit definition for EXTI_EMR register  *******************/
#define EXTI_EMR_MR0_Pos                    (0U)
#define EXTI_EMR_MR0_Msk                    (0x1UL << EXTI_EMR_MR0_Pos)         /*!< 0x00000001 */
#define EXTI_EMR_MR0                        EXTI_EMR_MR0_Msk                   /*!< Event Mask on line 0 */
#define EXTI_EMR_MR1_Pos                    (1U)
#define EXTI_EMR_MR1_Msk                    (0x1UL << EXTI_EMR_MR1_Pos)         /*!< 0x00000002 */
#define EXTI_EMR_MR1                        EXTI_EMR_MR1_Msk                   /*!< Event Mask on line 1 */
#define EXTI_EMR_MR2_Pos                    (2U)
#define EXTI_EMR_MR2_Msk                    (0x1UL << EXTI_EMR_MR2_Pos)         /*!< 0x00000004 */
#define EXTI_EMR_MR2                        EXTI_EMR_MR2_Msk                   /*!< Event Mask on line 2 */
#define EXTI_EMR_MR3_Pos                    (3U)
#define EXTI_EMR_MR3_Msk                    (0x1UL << EXTI_EMR_MR3_Pos)         /*!< 0x00000008 */
#define EXTI_EMR_MR3                        EXTI_EMR_MR3_Msk                   /*!< Event Mask on line 3 */
#define EXTI_EMR_MR4_Pos                    (4U)
#define EXTI_EMR_MR4_Msk                    (0x1UL << EXTI_EMR_MR4_Pos)         /*!< 0x00000010 */
#define EXTI_EMR_MR4                        EXTI_EMR_MR4_Msk                   /*!< Event Mask on line 4 */
#define EXTI_EMR_MR5_Pos                    (5U)
#define EXTI_EMR_MR5_Msk                    (0x1UL << EXTI_EMR_MR5_Pos)         /*!< 0x00000020 */
#define EXTI_EMR_MR5                        EXTI_EMR_MR5_Msk                   /*!< Event Mask on line 5 */
#define EXTI_EMR_MR6_Pos                    (6U)
#define EXTI_EMR_MR6_Msk                    (0x1UL << EXTI_EMR_MR6_Pos)         /*!< 0x00000040 */
#define EXTI_EMR_MR6                        EXTI_EMR_MR6_Msk                   /*!< Event Mask on line 6 */
#define EXTI_EMR_MR7_Pos                    (7U)
#define EXTI_EMR_MR7_Msk                    (0x1UL << EXTI_EMR_MR7_Pos)         /*!< 0x00000080 */
#define EXTI_EMR_MR7                        EXTI_EMR_MR7_Msk                   /*!< Event Mask on line 7 */
#define EXTI_EMR_MR8_Pos                    (8U)
#define EXTI_EMR_MR8_Msk                    (0x1UL << EXTI_EMR_MR8_Pos)         /*!< 0x00000100 */
#define EXTI_EMR_MR8                        EXTI_EMR_MR8_Msk                   /*!< Event Mask on line 8 */
#define EXTI_EMR_MR9_Pos                    (9U)
#define EXTI_EMR_MR9_Msk                    (0x1UL << EXTI_EMR_MR9_Pos)         /*!< 0x00000200 */
#define EXTI_EMR_MR9                        EXTI_EMR_MR9_Msk                   /*!< Event Mask on line 9 */
#define EXTI_EMR_MR10_Pos                   (10U)
#define EXTI_EMR_MR10_Msk                   (0x1UL << EXTI_EMR_MR10_Pos)        /*!< 0x00000400 */
#define EXTI_EMR_MR10                       EXTI_EMR_MR10_Msk                  /*!< Event Mask on line 10 */
#define EXTI_EMR_MR11_Pos                   (11U)
#define EXTI_EMR_MR11_Msk                   (0x1UL << EXTI_EMR_MR11_Pos)        /*!< 0x00000800 */
#define EXTI_EMR_MR11                       EXTI_EMR_MR11_Msk                  /*!< Event Mask on line 11 */
#define EXTI_EMR_MR12_Pos                   (12U)
#define EXTI_EMR_MR12_Msk                   (0x1UL << EXTI_EMR_MR12_Pos)        /*!< 0x00001000 */
#define EXTI_EMR_MR12                       EXTI_EMR_MR12_Msk                  /*!< Event Mask on line 12 */
#define EXTI_EMR_MR13_Pos                   (13U)
#define EXTI_EMR_MR13_Msk                   (0x1UL << EXTI_EMR_MR13_Pos)        /*!< 0x00002000 */
#define EXTI_EMR_MR13                       EXTI_EMR_MR13_Msk                  /*!< Event Mask on line 13 */
#define EXTI_EMR_MR14_Pos                   (14U)
#define EXTI_EMR_MR14_Msk                   (0x1UL << EXTI_EMR_MR14_Pos)        /*!< 0x00004000 */
#define EXTI_EMR_MR14                       EXTI_EMR_MR14_Msk                  /*!< Event Mask on line 14 */
#define EXTI_EMR_MR15_Pos                   (15U)
#define EXTI_EMR_MR15_Msk                   (0x1UL << EXTI_EMR_MR15_Pos)        /*!< 0x00008000 */
#define EXTI_EMR_MR15                       EXTI_EMR_MR15_Msk                  /*!< Event Mask on line 15 */
#define EXTI_EMR_MR16_Pos                   (16U)
#define EXTI_EMR_MR16_Msk                   (0x1UL << EXTI_EMR_MR16_Pos)        /*!< 0x00010000 */
#define EXTI_EMR_MR16                       EXTI_EMR_MR16_Msk                  /*!< Event Mask on line 16 */
#define EXTI_EMR_MR17_Pos                   (17U)
#define EXTI_EMR_MR17_Msk                   (0x1UL << EXTI_EMR_MR17_Pos)        /*!< 0x00020000 */
#define EXTI_EMR_MR17                       EXTI_EMR_MR17_Msk                  /*!< Event Mask on line 17 */
#define EXTI_EMR_MR18_Pos                   (18U)
#define EXTI_EMR_MR18_Msk                   (0x1UL << EXTI_EMR_MR18_Pos)        /*!< 0x00040000 */
#define EXTI_EMR_MR18                       EXTI_EMR_MR18_Msk                  /*!< Event Mask on line 18 */

/* References Defines */
#define  EXTI_EMR_EM0 EXTI_EMR_MR0
#define  EXTI_EMR_EM1 EXTI_EMR_MR1
#define  EXTI_EMR_EM2 EXTI_EMR_MR2
#define  EXTI_EMR_EM3 EXTI_EMR_MR3
#define  EXTI_EMR_EM4 EXTI_EMR_MR4
#define  EXTI_EMR_EM5 EXTI_EMR_MR5
#define  EXTI_EMR_EM6 EXTI_EMR_MR6
#define  EXTI_EMR_EM7 EXTI_EMR_MR7
#define  EXTI_EMR_EM8 EXTI_EMR_MR8
#define  EXTI_EMR_EM9 EXTI_EMR_MR9
#define  EXTI_EMR_EM10 EXTI_EMR_MR10
#define  EXTI_EMR_EM11 EXTI_EMR_MR11
#define  EXTI_EMR_EM12 EXTI_EMR_MR12
#define  EXTI_EMR_EM13 EXTI_EMR_MR13
#define  EXTI_EMR_EM14 EXTI_EMR_MR14
#define  EXTI_EMR_EM15 EXTI_EMR_MR15
#define  EXTI_EMR_EM16 EXTI_EMR_MR16
#define  EXTI_EMR_EM17 EXTI_EMR_MR17
#define  EXTI_EMR_EM18 EXTI_EMR_MR18

/******************  Bit definition for EXTI_RTSR register  *******************/
#define EXTI_RTSR_TR0_Pos                   (0U)
#define EXTI_RTSR_TR0_Msk                   (0x1UL << EXTI_RTSR_TR0_Pos)        /*!< 0x00000001 */
#define EXTI_RTSR_TR0                       EXTI_RTSR_TR0_Msk                  /*!< Rising trigger event configuration bit of line 0 */
#define EXTI_RTSR_TR1_Pos                   (1U)
#define EXTI_RTSR_TR1_Msk                   (0x1UL << EXTI_RTSR_TR1_Pos)        /*!< 0x00000002 */
#define EXTI_RTSR_TR1                       EXTI_RTSR_TR1_Msk                  /*!< Rising trigger event configuration bit of line 1 */
#define EXTI_RTSR_TR2_Pos                   (2U)
#define EXTI_RTSR_TR2_Msk                   (0x1UL << EXTI_RTSR_TR2_Pos)        /*!< 0x00000004 */
#define EXTI_RTSR_TR2                       EXTI_RTSR_TR2_Msk                  /*!< Rising trigger event configuration bit of line 2 */
#define EXTI_RTSR_TR3_Pos                   (3U)
#define EXTI_RTSR_TR3_Msk                   (0x1UL << EXTI_RTSR_TR3_Pos)        /*!< 0x00000008 */
#define EXTI_RTSR_TR3                       EXTI_RTSR_TR3_Msk                  /*!< Rising trigger event configuration bit of line 3 */
#define EXTI_RTSR_TR4_Pos                   (4U)
#define EXTI_RTSR_TR4_Msk                   (0x1UL << EXTI_RTSR_TR4_Pos)        /*!< 0x00000010 */
#define EXTI_RTSR_TR4                       EXTI_RTSR_TR4_Msk                  /*!< Rising trigger event configuration bit of line 4 */
#define EXTI_RTSR_TR5_Pos                   (5U)
#define EXTI_RTSR_TR5_Msk                   (0x1UL << EXTI_RTSR_TR5_Pos)        /*!< 0x00000020 */
#define EXTI_RTSR_TR5                       EXTI_RTSR_TR5_Msk                  /*!< Rising trigger event configuration bit of line 5 */
#define EXTI_RTSR_TR6_Pos                   (6U)
#define EXTI_RTSR_TR6_Msk                   (0x1UL << EXTI_RTSR_TR6_Pos)        /*!< 0x00000040 */
#define EXTI_RTSR_TR6                       EXTI_RTSR_TR6_Msk                  /*!< Rising trigger event configuration bit of line 6 */
#define EXTI_RTSR_TR7_Pos                   (7U)
#define EXTI_RTSR_TR7_Msk                   (0x1UL << EXTI_RTSR_TR7_Pos)        /*!< 0x00000080 */
#define EXTI_RTSR_TR7                       EXTI_RTSR_TR7_Msk                  /*!< Rising trigger event configuration bit of line 7 */
#define EXTI_RTSR_TR8_Pos                   (8U)
#define EXTI_RTSR_TR8_Msk                   (0x1UL << EXTI_RTSR_TR8_Pos)        /*!< 0x00000100 */
#define EXTI_RTSR_TR8                       EXTI_RTSR_TR8_Msk                  /*!< Rising trigger event configuration bit of line 8 */
#define EXTI_RTSR_TR9_Pos                   (9U)
#define EXTI_RTSR_TR9_Msk                   (0x1UL << EXTI_RTSR_TR9_Pos)        /*!< 0x00000200 */
#define EXTI_RTSR_TR9                       EXTI_RTSR_TR9_Msk                  /*!< Rising trigger event configuration bit of line 9 */
#define EXTI_RTSR_TR10_Pos                  (10U)
#define EXTI_RTSR_TR10_Msk                  (0x1UL << EXTI_RTSR_TR10_Pos)       /*!< 0x00000400 */
#define EXTI_RTSR_TR10                      EXTI_RTSR_TR10_Msk                 /*!< Rising trigger event configuration bit of line 10 */
#define EXTI_RTSR_TR11_Pos                  (11U)
#define EXTI_RTSR_TR11_Msk                  (0x1UL << EXTI_RTSR_TR11_Pos)       /*!< 0x00000800 */
#define EXTI_RTSR_TR11                      EXTI_RTSR_TR11_Msk                 /*!< Rising trigger event configuration bit of line 11 */
#define EXTI_RTSR_TR12_Pos                  (12U)
#define EXTI_RTSR_TR12_Msk                  (0x1UL << EXTI_RTSR_TR12_Pos)       /*!< 0x00001000 */
#define EXTI_RTSR_TR12                      EXTI_RTSR_TR12_Msk                 /*!< Rising trigger event configuration bit of line 12 */
#define EXTI_RTSR_TR13_Pos                  (13U)
#define EXTI_RTSR_TR13_Msk                  (0x1UL << EXTI_RTSR_TR13_Pos)       /*!< 0x00002000 */
#define EXTI_RTSR_TR13                      EXTI_RTSR_TR13_Msk                 /*!< Rising trigger event configuration bit of line 13 */
#define EXTI_RTSR_TR14_Pos                  (14U)
#define EXTI_RTSR_TR14_Msk                  (0x1UL << EXTI_RTSR_TR14_Pos)       /*!< 0x00004000 */
#define EXTI_RTSR_TR14                      EXTI_RTSR_TR14_Msk                 /*!< Rising trigger event configuration bit of line 14 */
#define EXTI_RTSR_TR15_Pos                  (15U)
#define EXTI_RTSR_TR15_Msk                  (0x1UL << EXTI_RTSR_TR15_Pos)       /*!< 0x00008000 */
#define EXTI_RTSR_TR15                      EXTI_RTSR_TR15_Msk                 /*!< Rising trigger event configuration bit of line 15 */
#define EXTI_RTSR_TR16_Pos                  (16U)
#define EXTI_RTSR_TR16_Msk                  (0x1UL << EXTI_RTSR_TR16_Pos)       /*!< 0x00010000 */
#define EXTI_RTSR_TR16                      EXTI_RTSR_TR16_Msk                 /*!< Rising trigger event configuration bit of line 16 */
#define EXTI_RTSR_TR17_Pos                  (17U)
#define EXTI_RTSR_TR17_Msk                  (0x1UL << EXTI_RTSR_TR17_Pos)       /*!< 0x00020000 */
#define EXTI_RTSR_TR17                      EXTI_RTSR_TR17_Msk                 /*!< Rising trigger event configuration bit of line 17 */
#define EXTI_RTSR_TR18_Pos                  (18U)
#define EXTI_RTSR_TR18_Msk                  (0x1UL << EXTI_RTSR_TR18_Pos)       /*!< 0x00040000 */
#define EXTI_RTSR_TR18                      EXTI_RTSR_TR18_Msk                 /*!< Rising trigger event configuration bit of line 18 */

/* References Defines */
#define  EXTI_RTSR_RT0 EXTI_RTSR_TR0
#define  EXTI_RTSR_RT1 EXTI_RTSR_TR1
#define  EXTI_RTSR_RT2 EXTI_RTSR_TR2
#define  EXTI_RTSR_RT3 EXTI_RTSR_TR3
#define  EXTI_RTSR_RT4 EXTI_RTSR_TR4
#define  EXTI_RTSR_RT5 EXTI_RTSR_TR5
#define  EXTI_RTSR_RT6 EXTI_RTSR_TR6
#define  EXTI_RTSR_RT7 EXTI_RTSR_TR7
#define  EXTI_RTSR_RT8 EXTI_RTSR_TR8
#define  EXTI_RTSR_RT9 EXTI_RTSR_TR9
#define  EXTI_RTSR_RT10 EXTI_RTSR_TR10
#define  EXTI_RTSR_RT11 EXTI_RTSR_TR11
#define  EXTI_RTSR_RT12 EXTI_RTSR_TR12
#define  EXTI_RTSR_RT13 EXTI_RTSR_TR13
#define  EXTI_RTSR_RT14 EXTI_RTSR_TR14
#define  EXTI_RTSR_RT15 EXTI_RTSR_TR15
#define  EXTI_RTSR_RT16 EXTI_RTSR_TR16
#define  EXTI_RTSR_RT17 EXTI_RTSR_TR17
#define  EXTI_RTSR_RT18 EXTI_RTSR_TR18

/******************  Bit definition for EXTI_FTSR register  *******************/
#define EXTI_FTSR_TR0_Pos                   (0U)
#define EXTI_FTSR_TR0_Msk                   (0x1UL << EXTI_FTSR_TR0_Pos)        /*!< 0x00000001 */
#define EXTI_FTSR_TR0                       EXTI_FTSR_TR0_Msk                  /*!< Falling trigger event configuration bit of line 0 */
#define EXTI_FTSR_TR1_Pos                   (1U)
#define EXTI_FTSR_TR1_Msk                   (0x1UL << EXTI_FTSR_TR1_Pos)        /*!< 0x00000002 */
#define EXTI_FTSR_TR1                       EXTI_FTSR_TR1_Msk                  /*!< Falling trigger event configuration bit of line 1 */
#define EXTI_FTSR_TR2_Pos                   (2U)
#define EXTI_FTSR_TR2_Msk                   (0x1UL << EXTI_FTSR_TR2_Pos)        /*!< 0x00000004 */
#define EXTI_FTSR_TR2                       EXTI_FTSR_TR2_Msk                  /*!< Falling trigger event configuration bit of line 2 */
#define EXTI_FTSR_TR3_Pos                   (3U)
#define EXTI_FTSR_TR3_Msk                   (0x1UL << EXTI_FTSR_TR3_Pos)        /*!< 0x00000008 */
#define EXTI_FTSR_TR3                       EXTI_FTSR_TR3_Msk                  /*!< Falling trigger event configuration bit of line 3 */
#define EXTI_FTSR_TR4_Pos                   (4U)
#define EXTI_FTSR_TR4_Msk                   (0x1UL << EXTI_FTSR_TR4_Pos)        /*!< 0x00000010 */
#define EXTI_FTSR_TR4                       EXTI_FTSR_TR4_Msk                  /*!< Falling trigger event configuration bit of line 4 */
#define EXTI_FTSR_TR5_Pos                   (5U)
#define EXTI_FTSR_TR5_Msk                   (0x1UL << EXTI_FTSR_TR5_Pos)        /*!< 0x00000020 */
#define EXTI_FTSR_TR5                       EXTI_FTSR_TR5_Msk                  /*!< Falling trigger event configuration bit of line 5 */
#define EXTI_FTSR_TR6_Pos                   (6U)
#define EXTI_FTSR_TR6_Msk                   (0x1UL << EXTI_FTSR_TR6_Pos)        /*!< 0x00000040 */
#define EXTI_FTSR_TR6                       EXTI_FTSR_TR6_Msk                  /*!< Falling trigger event configuration bit of line 6 */
#define EXTI_FTSR_TR7_Pos                   (7U)
#define EXTI_FTSR_TR7_Msk                   (0x1UL << EXTI_FTSR_TR7_Pos)        /*!< 0x00000080 */
#define EXTI_FTSR_TR7                       EXTI_FTSR_TR7_Msk                  /*!< Falling trigger event configuration bit of line 7 */
#define EXTI_FTSR_TR8_Pos                   (8U)
#define EXTI_FTSR_TR8_Msk                   (0x1UL << EXTI_FTSR_TR8_Pos)        /*!< 0x00000100 */
#define EXTI_FTSR_TR8                       EXTI_FTSR_TR8_Msk                  /*!< Falling trigger event configuration bit of line 8 */
#define EXTI_FTSR_TR9_Pos                   (9U)
#define EXTI_FTSR_TR9_Msk                   (0x1UL << EXTI_FTSR_TR9_Pos)        /*!< 0x00000200 */
#define EXTI_FTSR_TR9                       EXTI_FTSR_TR9_Msk                  /*!< Falling trigger event configuration bit of line 9 */
#define EXTI_FTSR_TR10_Pos                  (10U)
#define EXTI_FTSR_TR10_Msk                  (0x1UL << EXTI_FTSR_TR10_Pos)       /*!< 0x00000400 */
#define EXTI_FTSR_TR10                      EXTI_FTSR_TR10_Msk                 /*!< Falling trigger event configuration bit of line 10 */
#define EXTI_FTSR_TR11_Pos                  (11U)
#define EXTI_FTSR_TR11_Msk                  (0x1UL << EXTI_FTSR_TR11_Pos)       /*!< 0x00000800 */
#define EXTI_FTSR_TR11                      EXTI_FTSR_TR11_Msk                 /*!< Falling trigger event configuration bit of line 11 */
#define EXTI_FTSR_TR12_Pos                  (12U)
#define EXTI_FTSR_TR12_Msk                  (0x1UL << EXTI_FTSR_TR12_Pos)       /*!< 0x00001000 */
#define EXTI_FTSR_TR12                      EXTI_FTSR_TR12_Msk                 /*!< Falling trigger event configuration bit of line 12 */
#define EXTI_FTSR_TR13_Pos                  (13U)
#define EXTI_FTSR_TR13_Msk                  (0x1UL << EXTI_FTSR_TR13_Pos)       /*!< 0x00002000 */
#define EXTI_FTSR_TR13                      EXTI_FTSR_TR13_Msk                 /*!< Falling trigger event configuration bit of line 13 */
#define EXTI_FTSR_TR14_Pos                  (14U)
#define EXTI_FTSR_TR14_Msk                  (0x1UL << EXTI_FTSR_TR14_Pos)       /*!< 0x00004000 */
#define EXTI_FTSR_TR14                      EXTI_FTSR_TR14_Msk                 /*!< Falling trigger event configuration bit of line 14 */
#define EXTI_FTSR_TR15_Pos                  (15U)
#define EXTI_FTSR_TR15_Msk                  (0x1UL << EXTI_FTSR_TR15_Pos)       /*!< 0x00008000 */
#define EXTI_FTSR_TR15                      EXTI_FTSR_TR15_Msk                 /*!< Falling trigger event configuration bit of line 15 */
#define EXTI_FTSR_TR16_Pos                  (16U)
#define EXTI_FTSR_TR16_Msk                  (0x1UL << EXTI_FTSR_TR16_Pos)       /*!< 0x00010000 */
#define EXTI_FTSR_TR16                      EXTI_FTSR_TR16_Msk                 /*!< Falling trigger event configuration bit of line 16 */
#define EXTI_FTSR_TR17_Pos                  (17U)
#define EXTI_FTSR_TR17_Msk                  (0x1UL << EXTI_FTSR_TR17_Pos)       /*!< 0x00020000 */
#define EXTI_FTSR_TR17                      EXTI_FTSR_TR17_Msk                 /*!< Falling trigger event configuration bit of line 17 */
#define EXTI_FTSR_TR18_Pos                  (18U)
#define EXTI_FTSR_TR18_Msk                  (0x1UL << EXTI_FTSR_TR18_Pos)       /*!< 0x00040000 */
#define EXTI_FTSR_TR18                      EXTI_FTSR_TR18_Msk                 /*!< Falling trigger event configuration bit of line 18 */

/* References Defines */
#define  EXTI_FTSR_FT0 EXTI_FTSR_TR0
#define  EXTI_FTSR_FT1 EXTI_FTSR_TR1
#define  EXTI_FTSR_FT2 EXTI_FTSR_TR2
#define  EXTI_FTSR_FT3 EXTI_FTSR_TR3
#define  EXTI_FTSR_FT4 EXTI_FTSR_TR4
#define  EXTI_FTSR_FT5 EXTI_FTSR_TR5
#define  EXTI_FTSR_FT6 EXTI_FTSR_TR6
#define  EXTI_FTSR_FT7 EXTI_FTSR_TR7
#define  EXTI_FTSR_FT8 EXTI_FTSR_TR8
#define  EXTI_FTSR_FT9 EXTI_FTSR_TR9
#define  EXTI_FTSR_FT10 EXTI_FTSR_TR10
#define  EXTI_FTSR_FT11 EXTI_FTSR_TR11
#define  EXTI_FTSR_FT12 EXTI_FTSR_TR12
#define  EXTI_FTSR_FT13 EXTI_FTSR_TR13
#define  EXTI_FTSR_FT14 EXTI_FTSR_TR14
#define  EXTI_FTSR_FT15 EXTI_FTSR_TR15
#define  EXTI_FTSR_FT16 EXTI_FTSR_TR16
#define  EXTI_FTSR_FT17 EXTI_FTSR_TR17
#define  EXTI_FTSR_FT18 EXTI_FTSR_TR18

/******************  Bit definition for EXTI_SWIER register  ******************/
#define EXTI_SWIER_SWIER0_Pos               (0U)
#define EXTI_SWIER_SWIER0_Msk               (0x1UL << EXTI_SWIER_SWIER0_Pos)    /*!< 0x00000001 */
#define EXTI_SWIER_SWIER0                   EXTI_SWIER_SWIER0_Msk              /*!< Software Interrupt on line 0 */
#define EXTI_SWIER_SWIER1_Pos               (1U)
#define EXTI_SWIER_SWIER1_Msk               (0x1UL << EXTI_SWIER_SWIER1_Pos)    /*!< 0x00000002 */
#define EXTI_SWIER_SWIER1                   EXTI_SWIER_SWIER1_Msk              /*!< Software Interrupt on line 1 */
#define EXTI_SWIER_SWIER2_Pos               (2U)
#define EXTI_SWIER_SWIER2_Msk               (0x1UL << EXTI_SWIER_SWIER2_Pos)    /*!< 0x00000004 */
#define EXTI_SWIER_SWIER2                   EXTI_SWIER_SWIER2_Msk              /*!< Software Interrupt on line 2 */
#define EXTI_SWIER_SWIER3_Pos               (3U)
#define EXTI_SWIER_SWIER3_Msk               (0x1UL << EXTI_SWIER_SWIER3_Pos)    /*!< 0x00000008 */
#define EXTI_SWIER_SWIER3                   EXTI_SWIER_SWIER3_Msk              /*!< Software Interrupt on line 3 */
#define EXTI_SWIER_SWIER4_Pos               (4U)
#define EXTI_SWIER_SWIER4_Msk               (0x1UL << EXTI_SWIER_SWIER4_Pos)    /*!< 0x00000010 */
#define EXTI_SWIER_SWIER4                   EXTI_SWIER_SWIER4_Msk              /*!< Software Interrupt on line 4 */
#define EXTI_SWIER_SWIER5_Pos               (5U)
#define EXTI_SWIER_SWIER5_Msk               (0x1UL << EXTI_SWIER_SWIER5_Pos)    /*!< 0x00000020 */
#define EXTI_SWIER_SWIER5                   EXTI_SWIER_SWIER5_Msk              /*!< Software Interrupt on line 5 */
#define EXTI_SWIER_SWIER6_Pos               (6U)
#define EXTI_SWIER_SWIER6_Msk               (0x1UL << EXTI_SWIER_SWIER6_Pos)    /*!< 0x00000040 */
#define EXTI_SWIER_SWIER6                   EXTI_SWIER_SWIER6_Msk              /*!< Software Interrupt on line 6 */
#define EXTI_SWIER_SWIER7_Pos               (7U)
#define EXTI_SWIER_SWIER7_Msk               (0x1UL << EXTI_SWIER_SWIER7_Pos)    /*!< 0x00000080 */
#define EXTI_SWIER_SWIER7                   EXTI_SWIER_SWIER7_Msk              /*!< Software Interrupt on line 7 */
#define EXTI_SWIER_SWIER8_Pos               (8U)
#define EXTI_SWIER_SWIER8_Msk               (0x1UL << EXTI_SWIER_SWIER8_Pos)    /*!< 0x00000100 */
#define EXTI_SWIER_SWIER8                   EXTI_SWIER_SWIER8_Msk              /*!< Software Interrupt on line 8 */
#define EXTI_SWIER_SWIER9_Pos               (9U)
#define EXTI_SWIER_SWIER9_Msk               (0x1UL << EXTI_SWIER_SWIER9_Pos)    /*!< 0x00000200 */
#define EXTI_SWIER_SWIER9                   EXTI_SWIER_SWIER9_Msk              /*!< Software Interrupt on line 9 */
#define EXTI_SWIER_SWIER10_Pos              (10U)
#define EXTI_SWIER_SWIER10_Msk              (0x1UL << EXTI_SWIER_SWIER10_Pos)   /*!< 0x00000400 */
#define EXTI_SWIER_SWIER10                  EXTI_SWIER_SWIER10_Msk             /*!< Software Interrupt on line 10 */
#define EXTI_SWIER_SWIER11_Pos              (11U)
#define EXTI_SWIER_SWIER11_Msk              (0x1UL << EXTI_SWIER_SWIER11_Pos)   /*!< 0x00000800 */
#define EXTI_SWIER_SWIER11                  EXTI_SWIER_SWIER11_Msk             /*!< Software Interrupt on line 11 */
#define EXTI_SWIER_SWIER12_Pos              (12U)
#define EXTI_SWIER_SWIER12_Msk              (0x1UL << EXTI_SWIER_SWIER12_Pos)   /*!< 0x00001000 */
#define EXTI_SWIER_SWIER12                  EXTI_SWIER_SWIER12_Msk             /*!< Software Interrupt on line 12 */
#define EXTI_SWIER_SWIER13_Pos              (13U)
#define EXTI_SWIER_SWIER13_Msk              (0x1UL << EXTI_SWIER_SWIER13_Pos)   /*!< 0x00002000 */
#define EXTI_SWIER_SWIER13                  EXTI_SWIER_SWIER13_Msk             /*!< Software Interrupt on line 13 */
#define EXTI_SWIER_SWIER14_Pos              (14U)
#define EXTI_SWIER_SWIER14_Msk              (0x1UL << EXTI_SWIER_SWIER14_Pos)   /*!< 0x00004000 */
#define EXTI_SWIER_SWIER14                  EXTI_SWIER_SWIER14_Msk             /*!< Software Interrupt on line 14 */
#define EXTI_SWIER_SWIER15_Pos              (15U)
#define EXTI_SWIER_SWIER15_Msk              (0x1UL << EXTI_SWIER_SWIER15_Pos)   /*!< 0x00008000 */
#define EXTI_SWIER_SWIER15                  EXTI_SWIER_SWIER15_Msk             /*!< Software Interrupt on line 15 */
#define EXTI_SWIER_SWIER16_Pos              (16U)
#define EXTI_SWIER_SWIER16_Msk              (0x1UL << EXTI_SWIER_SWIER16_Pos)   /*!< 0x00010000 */
#define EXTI_SWIER_SWIER16                  EXTI_SWIER_SWIER16_Msk             /*!< Software Interrupt on line 16 */
#define EXTI_SWIER_SWIER17_Pos              (17U)
#define EXTI_SWIER_SWIER17_Msk              (0x1UL << EXTI_SWIER_SWIER17_Pos)   /*!< 0x00020000 */
#define EXTI_SWIER_SWIER17                  EXTI_SWIER_SWIER17_Msk             /*!< Software Interrupt on line 17 */
#define EXTI_SWIER_SWIER18_Pos              (18U)
#define EXTI_SWIER_SWIER18_Msk              (0x1UL << EXTI_SWIER_SWIER18_Pos)   /*!< 0x00040000 */
#define EXTI_SWIER_SWIER18                  EXTI_SWIER_SWIER18_Msk             /*!< Software Interrupt on line 18 */

/* References Defines */
#define  EXTI_SWIER_SWI0 EXTI_SWIER_SWIER0
#define  EXTI_SWIER_SWI1 EXTI_SWIER_SWIER1
#define  EXTI_SWIER_SWI2 EXTI_SWIER_SWIER2
#define  EXTI_SWIER_SWI3 EXTI_SWIER_SWIER3
#define  EXTI_SWIER_SWI4 EXTI_SWIER_SWIER4
#define  EXTI_SWIER_SWI5 EXTI_SWIER_SWIER5
#define  EXTI_SWIER_SWI6 EXTI_SWIER_SWIER6
#define  EXTI_SWIER_SWI7 EXTI_SWIER_SWIER7
#define  EXTI_SWIER_SWI8 EXTI_SWIER_SWIER8
#define  EXTI_SWIER_SWI9 EXTI_SWIER_SWIER9
#define  EXTI_SWIER_SWI10 EXTI_SWIER_SWIER10
#define  EXTI_SWIER_SWI11 EXTI_SWIER_SWIER11
#define  EXTI_SWIER_SWI12 EXTI_SWIER_SWIER12
#define  EXTI_SWIER_SWI13 EXTI_SWIER_SWIER13
#define  EXTI_SWIER_SWI14 EXTI_SWIER_SWIER14
#define  EXTI_SWIER_SWI15 EXTI_SWIER_SWIER15
#define  EXTI_SWIER_SWI16 EXTI_SWIER_SWIER16
#define  EXTI_SWIER_SWI17 EXTI_SWIER_SWIER17
#define  EXTI_SWIER_SWI18 EXTI_SWIER_SWIER18

/*******************  Bit definition for EXTI_PR register  ********************/
#define EXTI_PR_PR0_Pos                     (0U)
#define EXTI_PR_PR0_Msk                     (0x1UL << EXTI_PR_PR0_Pos)          /*!< 0x00000001 */
#define EXTI_PR_PR0                         EXTI_PR_PR0_Msk                    /*!< Pending bit for line 0 */
#define EXTI_PR_PR1_Pos                     (1U)
#define EXTI_PR_PR1_Msk                     (0x1UL << EXTI_PR_PR1_Pos)          /*!< 0x00000002 */
#define EXTI_PR_PR1                         EXTI_PR_PR1_Msk                    /*!< Pending bit for line 1 */
#define EXTI_PR_PR2_Pos                     (2U)
#define EXTI_PR_PR2_Msk                     (0x1UL << EXTI_PR_PR2_Pos)          /*!< 0x00000004 */
#define EXTI_PR_PR2                         EXTI_PR_PR2_Msk                    /*!< Pending bit for line 2 */
#define EXTI_PR_PR3_Pos                     (3U)
#define EXTI_PR_PR3_Msk                     (0x1UL << EXTI_PR_PR3_Pos)          /*!< 0x00000008 */
#define EXTI_PR_PR3                         EXTI_PR_PR3_Msk                    /*!< Pending bit for line 3 */
#define EXTI_PR_PR4_Pos                     (4U)
#define EXTI_PR_PR4_Msk                     (0x1UL << EXTI_PR_PR4_Pos)          /*!< 0x00000010 */
#define EXTI_PR_PR4                         EXTI_PR_PR4_Msk                    /*!< Pending bit for line 4 */
#define EXTI_PR_PR5_Pos                     (5U)
#define EXTI_PR_PR5_Msk                     (0x1UL << EXTI_PR_PR5_Pos)          /*!< 0x00000020 */
#define EXTI_PR_PR5                         EXTI_PR_PR5_Msk                    /*!< Pending bit for line 5 */
#define EXTI_PR_PR6_Pos                     (6U)
#define EXTI_PR_PR6_Msk                     (0x1UL << EXTI_PR_PR6_Pos)          /*!< 0x00000040 */
#define EXTI_PR_PR6                         EXTI_PR_PR6_Msk                    /*!< Pending bit for line 6 */
#define EXTI_PR_PR7_Pos                     (7U)
#define EXTI_PR_PR7_Msk                     (0x1UL << EXTI_PR_PR7_Pos)          /*!< 0x00000080 */
#define EXTI_PR_PR7                         EXTI_PR_PR7_Msk                    /*!< Pending bit for line 7 */
#define EXTI_PR_PR8_Pos                     (8U)
#define EXTI_PR_PR8_Msk                     (0x1UL << EXTI_PR_PR8_Pos)          /*!< 0x00000100 */
#define EXTI_PR_PR8                         EXTI_PR_PR8_Msk                    /*!< Pending bit for line 8 */
#define EXTI_PR_PR9_Pos                     (9U)
#define EXTI_PR_PR9_Msk                     (0x1UL << EXTI_PR_PR9_Pos)          /*!< 0x00000200 */
#define EXTI_PR_PR9                         EXTI_PR_PR9_Msk                    /*!< Pending bit for line 9 */
#define EXTI_PR_PR10_Pos                    (10U)
#define EXTI_PR_PR10_Msk                    (0x1UL << EXTI_PR_PR10_Pos)         /*!< 0x00000400 */
#define EXTI_PR_PR10                        EXTI_PR_PR10_Msk                   /*!< Pending bit for line 10 */
#define EXTI_PR_PR11_Pos                    (11U)
#define EXTI_PR_PR11_Msk                    (0x1UL << EXTI_PR_PR11_Pos)         /*!< 0x00000800 */
#define EXTI_PR_PR11                        EXTI_PR_PR11_Msk                   /*!< Pending bit for line 11 */
#define EXTI_PR_PR12_Pos                    (12U)
#define EXTI_PR_PR12_Msk                    (0x1UL << EXTI_PR_PR12_Pos)         /*!< 0x00001000 */
#define EXTI_PR_PR12                        EXTI_PR_PR12_Msk                   /*!< Pending bit for line 12 */
#define EXTI_PR_PR13_Pos                    (13U)
#define EXTI_PR_PR13_Msk                    (0x1UL << EXTI_PR_PR13_Pos)         /*!< 0x00002000 */
#define EXTI_PR_PR13                        EXTI_PR_PR13_Msk                   /*!< Pending bit for line 13 */
#define EXTI_PR_PR14_Pos                    (14U)
#define EXTI_PR_PR14_Msk                    (0x1UL << EXTI_PR_PR14_Pos)         /*!< 0x00004000 */
#define EXTI_PR_PR14                        EXTI_PR_PR14_Msk                   /*!< Pending bit for line 14 */
#define EXTI_PR_PR15_Pos                    (15U)
#define EXTI_PR_PR15_Msk                    (0x1UL << EXTI_PR_PR15_Pos)         /*!< 0x00008000 */
#define EXTI_PR_PR15                        EXTI_PR_PR15_Msk                   /*!< Pending bit for line 15 */
#define EXTI_PR_PR16_Pos                    (16U)
#define EXTI_PR_PR16_Msk                    (0x1UL << EXTI_PR_PR16_Pos)         /*!< 0x00010000 */
#define EXTI_PR_PR16                        EXTI_PR_PR16_Msk                   /*!< Pending bit for line 16 */
#define EXTI_PR_PR17_Pos                    (17U)
#define EXTI_PR_PR17_Msk                    (0x1UL << EXTI_PR_PR17_Pos)         /*!< 0x00020000 */
#define EXTI_PR_PR17                        EXTI_PR_PR17_Msk                   /*!< Pending bit for line 17 */
#define EXTI_PR_PR18_Pos                    (18U)
#define EXTI_PR_PR18_Msk                    (0x1UL << EXTI_PR_PR18_Pos)         /*!< 0x00040000 */
#define EXTI_PR_PR18                        EXTI_PR_PR18_Msk                   /*!< Pending bit for line 18 */

/* References Defines */
#define  EXTI_PR_PIF0 EXTI_PR_PR0
#define  EXTI_PR_PIF1 EXTI_PR_PR1
#define  EXTI_PR_PIF2 EXTI_PR_PR2
#define  EXTI_PR_PIF3 EXTI_PR_PR3
#define  EXTI_PR_PIF4 EXTI_PR_PR4
#define  EXTI_PR_PIF5 EXTI_PR_PR5
#define  EXTI_PR_PIF6 EXTI_PR_PR6
#define  EXTI_PR_PIF7 EXTI_PR_PR7
#define  EXTI_PR_PIF8 EXTI_PR_PR8
#define  EXTI_PR_PIF9 EXTI_PR_PR9
#define  EXTI_PR_PIF10 EXTI_PR_PR10
#define  EXTI_PR_PIF11 EXTI_PR_PR11
#define  EXTI_PR_PIF12 EXTI_PR_PR12
#define  EXTI_PR_PIF13 EXTI_PR_PR13
#define  EXTI_PR_PIF14 EXTI_PR_PR14
#define  EXTI_PR_PIF15 EXTI_PR_PR15
#define  EXTI_PR_PIF16 EXTI_PR_PR16
#define  EXTI_PR_PIF17 EXTI_PR_PR17
#define  EXTI_PR_PIF18 EXTI_PR_PR18

/******************************************************************************/
/*                                                                            */
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for USART_SR register  *******************/
#define USART_SR_PE_Pos                     (0U)
#define USART_SR_PE_Msk                     (0x1UL << USART_SR_PE_Pos)          /*!< 0x00000001 */
#define USART_SR_PE                         USART_SR_PE_Msk                    /*!< Parity Error */
#define USART_SR_FE_Pos                     (1U)
#define USART_SR_FE_Msk                     (0x1UL << USART_SR_FE_Pos)          /*!< 0x00000002 */
#define USART_SR_FE                         USART_SR_FE_Msk                    /*!< Framing Error */
#define USART_SR_NE_Pos                     (2U)
#define USART_SR_NE_Msk                     (0x1UL << USART_SR_NE_Pos)          /*!< 0x00000004 */
#define USART_SR_NE                         USART_SR_NE_Msk                    /*!< Noise Error Flag */
#define USART_SR_ORE_Pos                    (3U)
#define USART_SR_ORE_Msk                    (0x1UL << USART_SR_ORE_Pos)         /*!< 0x00000008 */
#define USART_SR_ORE                        USART_SR_ORE_Msk                   /*!< OverRun Error */
#define USART_SR_IDLE_Pos                   (4U)
#define USART_SR_IDLE_Msk                   (0x1UL << USART_SR_IDLE_Pos)        /*!< 0x00000010 */
#define USART_SR_IDLE                       USART_SR_IDLE_Msk                  /*!< IDLE line detected */
#define USART_SR_RXNE_Pos                   (5U)
#define USART_SR_RXNE_Msk                   (0x1UL << USART_SR_RXNE_Pos)        /*!< 0x00000020 */
#define USART_SR_RXNE                       USART_SR_RXNE_Msk                  /*!< Read Data Register Not Empty */
#define USART_SR_TC_Pos                     (6U)
#define USART_SR_TC_Msk                     (0x1UL << USART_SR_TC_Pos)          /*!< 0x00000040 */
#define USART_SR_TC                         USART_SR_TC_Msk                    /*!< Transmission Complete */
#define USART_SR_TXE_Pos                    (7U)
#define USART_SR_TXE_Msk                    (0x1UL << USART_SR_TXE_Pos)         /*!< 0x00000080 */
#define USART_SR_TXE                        USART_SR_TXE_Msk                   /*!< Transmit Data Register Empty */
#define USART_SR_LBD_Pos                    (8U)
#define USART_SR_LBD_Msk                    (0x1UL << USART_SR_LBD_Pos)         /*!< 0x00000100 */
#define USART_SR_LBD                        USART_SR_LBD_Msk                   /*!< LIN Break Detection Flag */
#define USART_SR_CTS_Pos                    (9U)
#define USART_SR_CTS_Msk                    (0x1UL << USART_SR_CTS_Pos)         /*!< 0x00000200 */
#define USART_SR_CTS                        USART_SR_CTS_Msk                   /*!< CTS Flag */

/*******************  Bit definition for USART_DR register  *******************/
#define USART_DR_DR_Pos                     (0U)
#define USART_DR_DR_Msk                     (0x1FFUL << USART_DR_DR_Pos)        /*!< 0x000001FF */
#define USART_DR_DR                         USART_DR_DR_Msk                    /*!< Data value */

/******************  Bit definition for USART_BRR register  *******************/
#define USART_BRR_DIV_Fraction_Pos          (0U)
#define USART_BRR_DIV_Fraction_Msk          (0xFUL << USART_BRR_DIV_Fraction_Pos) /*!< 0x0000000F */
#define USART_BRR_DIV_Fraction              USART_BRR_DIV_Fraction_Msk         /*!< Fraction of USARTDIV */
#define USART_BRR_DIV_Mantissa_Pos          (4U)
#define USART_BRR_DIV_Mantissa_Msk          (0xFFFUL << USART_BRR_DIV_Mantissa_Pos) /*!< 0x0000FFF0 */
#define USART_BRR_DIV_Mantissa              USART_BRR_DIV_Mantissa_Msk         /*!< Mantissa of USARTDIV */

/******************  Bit definition for USART_CR1 register  *******************/
#define USART_CR1_SBK_Pos                   (0U)
#define USART_CR1_SBK_Msk                   (0x1UL << USART_CR1_SBK_Pos)        /*!< 0x00000001 */
#define USART_CR1_SBK                       USART_CR1_SBK_Msk                  /*!< Send Break */
#define USART_CR1_RWU_Pos                   (1U)
#define USART_CR1_RWU_Msk                   (0x1UL << USART_CR1_RWU_Pos)        /*!< 0x00000002 */
#define USART_CR1_RWU                       USART_CR1_RWU_Msk                  /*!< Receiver wakeup */
#define USART_CR1_RE_Pos                    (2U)
#define USART_CR1_RE_Msk                    (0x1UL << USART_CR1_RE_Pos)         /*!< 0x00000004 */
#define USART_CR1_RE                        USART_CR1_RE_Msk                   /*!< Receiver Enable */
#define USART_CR1_TE_Pos                    (3U)
#define USART_CR1_TE_Msk                    (0x1UL << USART_CR1_TE_Pos)         /*!< 0x00000008 */
#define USART_CR1_TE                        USART_CR1_TE_Msk                   /*!< Transmitter Enable */
#define USART_CR1_IDLEIE_Pos                (4U)
#define USART_CR1_IDLEIE_Msk                (0x1UL << USART_CR1_IDLEIE_Pos)     /*!< 0x00000010 */
#define USART_CR1_IDLEIE                    USART_CR1_IDLEIE_Msk               /*!< IDLE Interrupt Enable */
#define USART_CR1_RXNEIE_Pos                (5U)
#define USART_CR1_RXNEIE_Msk                (0x1UL << USART_CR1_RXNEIE_Pos)     /*!< 0x00000020 */
#define USART_CR1_RXNEIE                    USART_CR1_RXNEIE_Msk               /*!< RXNE Interrupt Enable */
#define USART_CR1_TCIE_Pos                  (6U)
#define USART_CR1_TCIE_Msk                  (0x1UL << USART_CR1_TCIE_Pos)       /*!< 0x00000040 */
#define USART_CR1_TCIE                      USART_CR1_TCIE_Msk                 /*!< Transmission Complete Interrupt Enable */
#define USART_CR1_TXEIE_Pos                 (7U)
#define USART_CR1_TXEIE_Msk                 (0x1UL << USART_CR1_TXEIE_Pos)      /*!< 0x00000080 */
#define USART_CR1_TXEIE                     USART_CR1_TXEIE_Msk                /*!< PE Interrupt Enable */
#define USART_CR1_PEIE_Pos                  (8U)
#define USART_CR1_PEIE_Msk                  (0x1UL << USART_CR1_PEIE_Pos)       /*!< 0x00000100 */
#define USART_CR1_PEIE                      USART_CR1_PEIE_Msk                 /*!< PE Interrupt Enable */
#define USART_CR1_PS_Pos                    (9U)
#define USART_CR1_PS_Msk                    (0x1UL << USART_CR1_PS_Pos)         /*!< 0x00000200 */
#define USART_CR1_PS                        USART_CR1_PS_Msk                   /*!< Parity Selection */
#define USART_CR1_PCE_Pos                   (10U)
#define USART_CR1_PCE_Msk                   (0x1UL << USART_CR1_PCE_Pos)        /*!< 0x00000400 */
#define USART_CR1_PCE                       USART_CR1_PCE_Msk                  /*!< Parity Control Enable */
#define USART_CR1_WAKE_Pos                  (11U)
#define USART_CR1_WAKE_Msk                  (0x1UL << USART_CR1_WAKE_Pos)       /*!< 0x00000800 */
#define USART_CR1_WAKE                      USART_CR1_WAKE_Msk                 /*!< Wakeup method */
#define USART_CR1_M_Pos                     (12U)
#define USART_CR1_M_Msk                     (0x1UL << USART_CR1_M_Pos)          /*!< 0x00001000 */
#define USART_CR1_M                         USART_CR1_M_Msk                    /*!< Word length */
#define USART_CR1_UE_Pos                    (13U)
#define USART_CR1_UE_Msk                    (0x1UL << USART_CR1_UE_Pos)         /*!< 0x00002000 */
#define USART_CR1_UE                        USART_CR1_UE_Msk                   /*!< USART Enable */

/******************  Bit definition for USART_CR2 register  *******************/
#define USART_CR2_ADD_Pos                   (0U)
#define USART_CR2_ADD_Msk                   (0xFUL << USART_CR2_ADD_Pos)        /*!< 0x0000000F */
#define USART_CR2_ADD                       USART_CR2_ADD_Msk                  /*!< Address of the USART node */
#define USART_CR2_LBDL_Pos                  (5U)
#define USART_CR2_LBDL_Msk                  (0x1UL << USART_CR2_LBDL_Pos)       /*!< 0x00000020 */
#define USART_CR2_LBDL                      USART_CR2_LBDL_Msk                 /*!< LIN Break Detection Length */
#define USART_CR2_LBDIE_Pos                 (6U)
#define USART_CR2_LBDIE_Msk                 (0x1UL << USART_CR2_LBDIE_Pos)      /*!< 0x00000040 */
#define USART_CR2_LBDIE                     USART_CR2_LBDIE_Msk                /*!< LIN Break Detection Interrupt Enable */
#define USART_CR2_LBCL_Pos                  (8U)
#define USART_CR2_LBCL_Msk                  (0x1UL << USART_CR2_LBCL_Pos)       /*!< 0x00000100 */
#define USART_CR2_LBCL                      USART_CR2_LBCL_Msk                 /*!< Last Bit Clock pulse */
#define USART_CR2_CPHA_Pos                  (9U)
#define USART_CR2_CPHA_Msk                  (0x1UL << USART_CR2_CPHA_Pos)       /*!< 0x00000200 */
#define USART_CR2_CPHA                      USART_CR2_CPHA_Msk                 /*!< Clock Phase */
#define USART_CR2_CPOL_Pos                  (10U)
#define USART_CR2_CPOL_Msk                  (0x1UL << USART_CR2_CPOL_Pos)       /*!< 0x00000400 */
#define USART_CR2_CPOL                      USART_CR2_CPOL_Msk                 /*!< Clock Polarity */
#define USART_CR2_CLKEN_Pos                 (11U)
#define USART_CR2_CLKEN_Msk                 (0x1UL << USART_CR2_CLKEN_Pos)      /*!< 0x00000800 */
#define USART_CR2_CLKEN                     USART_CR2_CLKEN_Msk                /*!< Clock Enable */

#define USART_CR2_STOP_Pos                  (12U)
#define USART_CR2_STOP_Msk                  (0x3UL << USART_CR2_STOP_Pos)       /*!< 0x00003000 */
#define USART_CR2_STOP                      USART_CR2_STOP_Msk                 /*!< STOP[1:0] bits (STOP bits) */
#define USART_CR2_STOP_0                    (0x1UL << USART_CR2_STOP_Pos)       /*!< 0x00001000 */
#define USART_CR2_STOP_1                    (0x2UL << USART_CR2_STOP_Pos)       /*!< 0x00002000 */

#define USART_CR2_LINEN_Pos                 (14U)
#define USART_CR2_LINEN_Msk                 (0x1UL << USART_CR2_LINEN_Pos)      /*!< 0x00004000 */
#define USART_CR2_LINEN                     USART_CR2_LINEN_Msk                /*!< LIN mode enable */

/******************  Bit definition for USART_CR3 register  *******************/
#define USART_CR3_EIE_Pos                   (0U)
#define USART_CR3_EIE_Msk                   (0x1UL << USART_CR3_EIE_Pos)        /*!< 0x00000001 */
#define USART_CR3_EIE                       USART_CR3_EIE_Msk                  /*!< Error Interrupt Enable */
#define USART_CR3_IREN_Pos                  (1U)
#define USART_CR3_IREN_Msk                  (0x1UL << USART_CR3_IREN_Pos)       /*!< 0x00000002 */
#define USART_CR3_IREN                      USART_CR3_IREN_Msk                 /*!< IrDA mode Enable */
#define USART_CR3_IRLP_Pos                  (2U)
#define USART_CR3_IRLP_Msk                  (0x1UL << USART_CR3_IRLP_Pos)       /*!< 0x00000004 */
#define USART_CR3_IRLP                      USART_CR3_IRLP_Msk                 /*!< IrDA Low-Power */
#define USART_CR3_HDSEL_Pos                 (3U)
#define USART_CR3_HDSEL_Msk                 (0x1UL << USART_CR3_HDSEL_Pos)      /*!< 0x00000008 */
#define USART_CR3_HDSEL                     USART_CR3_HDSEL_Msk                /*!< Half-Duplex Selection */
#define USART_CR3_NACK_Pos                  (4U)
#define USART_CR3_NACK_Msk                  (0x1UL << USART_CR3_NACK_Pos)       /*!< 0x00000010 */
#define USART_CR3_NACK                      USART_CR3_NACK_Msk                 /*!< Smartcard NACK enable */
#define USART_CR3_SCEN_Pos                  (5U)
#define USART_CR3_SCEN_Msk                  (0x1UL << USART_CR3_SCEN_Pos)       /*!< 0x00000020 */
#define USART_CR3_SCEN                      USART_CR3_SCEN_Msk                 /*!< Smartcard mode enable */
#define USART_CR3_DMAR_Pos                  (6U)
#define USART_CR3_DMAR_Msk                  (0x1UL << USART_CR3_DMAR_Pos)       /*!< 0x00000040 */
#define USART_CR3_DMAR                      USART_CR3_DMAR_Msk                 /*!< DMA Enable Receiver */
#define USART_CR3_DMAT_Pos                  (7U)
#define USART_CR3_DMAT_Msk                  (0x1UL << USART_CR3_DMAT_Pos)       /*!< 0x00000080 */
#define USART_CR3_DMAT                      USART_CR3_DMAT_Msk                 /*!< DMA Enable Transmitter */
#define USART_CR3_RTSE_Pos                  (8U)
#define USART_CR3_RTSE_Msk                  (0x1UL << USART_CR3_RTSE_Pos)       /*!< 0x00000100 */
#define USART_CR3_RTSE                      USART_CR3_RTSE_Msk                 /*!< RTS Enable */
#define USART_CR3_CTSE_Pos                  (9U)
#define USART_CR3_CTSE_Msk                  (0x1UL << USART_CR3_CTSE_Pos)       /*!< 0x00000200 */
#define USART_CR3_CTSE                      USART_CR3_CTSE_Msk                 /*!< CTS Enable */
#define USART_CR3_CTSIE_Pos                 (10U)
#define USART_CR3_CTSIE_Msk                 (0x1UL << USART_CR3_CTSIE_Pos)      /*!< 0x00000400 */
#define USART_CR3_CTSIE                     USART_CR3_CTSIE_Msk                /*!< CTS Interrupt Enable */

/******************  Bit definition for USART_GTPR register  ******************/
#define USART_GTPR_PSC_Pos                  (0U)
#define USART_GTPR_PSC_Msk                  (0xFFUL << USART_GTPR_PSC_Pos)      /*!< 0x000000FF */
#define USART_GTPR_PSC                      USART_GTPR_PSC_Msk                 /*!< PSC[7:0] bits (Prescaler value) */
#define USART_GTPR_PSC_0                    (0x01UL << USART_GTPR_PSC_Pos)      /*!< 0x00000001 */
#define USART_GTPR_PSC_1                    (0x02UL << USART_GTPR_PSC_Pos)      /*!< 0x00000002 */
#define USART_GTPR_PSC_2                    (0x04UL << USART_GTPR_PSC_Pos)      /*!< 0x00000004 */
#define USART_GTPR_PSC_3                    (0x08UL << USART_GTPR_PSC_Pos)      /*!< 0x00000008 */
#define USART_GTPR_PSC_4                    (0x10UL << USART_GTPR_PSC_Pos)      /*!< 0x00000010 */
#define USART_GTPR_PSC_5                    (0x20UL << USART_GTPR_PSC_Pos)      /*!< 0x00000020 */
#define USART_GTPR_PSC_6                    (0x40UL << USART_GTPR_PSC_Pos)      /*!< 0x00000040 */
#define USART_GTPR_PSC_7                    (0x80UL << USART_GTPR_PSC_Pos)      /*!< 0x00000080 */

#define USART_GTPR_GT_Pos                   (8U)
#define USART_GTPR_GT_Msk                   (0xFFUL << USART_GTPR_GT_Pos)       /*!< 0x0000FF00 */
#define USART_GTPR_GT                       USART_GTPR_GT_Msk                  /*!< Guard time value */

/*
 * Clock enable/disable macros for GPIOx peripheral
 */
#define GPIOA_CLOCK_ENABLE()		(RCC->APB2ENR |= RCC_APB2ENR_IOPAEN)
#define GPIOB_CLOCK_ENABLE()		(RCC->APB2ENR |= RCC_APB2ENR_IOPBEN)
#define GPIOC_CLOCK_ENABLE()		(RCC->APB2ENR |= RCC_APB2ENR_IOPCEN)
#define GPIOD_CLOCK_ENABLE()		(RCC->APB2ENR |= RCC_APB2ENR_IOPDEN)
#define GPIOE_CLOCK_ENABLE()		(RCC->APB2ENR |= RCC_APB2ENR_IOPEEN)

#define GPIOA_CLOCK_DISABLE()		(RCC->APB2ENR &= ~RCC_APB2ENR_IOPAEN)
#define GPIOB_CLOCK_DISABLE()		(RCC->APB2ENR &= ~RCC_APB2ENR_IOPBEN)
#define GPIOC_CLOCK_DISABLE()		(RCC->APB2ENR &= ~RCC_APB2ENR_IOPCEN)
#define GPIOD_CLOCK_DISABLE()		(RCC->APB2ENR &= ~RCC_APB2ENR_IOPDEN)
#define GPIOE_CLOCK_DISABLE()		(RCC->APB2ENR &= ~RCC_APB2ENR_IOPEEN)

/*
 * Portcode macro for GPIOx peripheral
 */
#define GPIO_PORTCODE(x)		( (x == GPIOA) ? 0 : \
								  (x == GPIOB) ? 1 : \
								  (x == GPIOC) ? 2 : \
								  (x == GPIOD) ? 3 : \
								  (x == GPIOE) ? 4 : 0 )

/*
 * Clock enable/disable macros for AFIO peripheral
 */
#define AFIO_CLOCK_ENABLE()			(RCC->APB2ENR |= RCC_APB2ENR_AFIOEN)
#define AFIO_CLOCK_DISABLE()		(RCC->APB2ENR &= ~RCC_APB2ENR_AFIOEN)

/*
 * Clock enable/disable macros for USARTx peripheral
 */
#define USART1_CLOCK_ENABLE()		(RCC->APB2ENR |= RCC_APB2ENR_USART1EN)
#define USART2_CLOCK_ENABLE()		(RCC->APB1ENR |= RCC_APB1ENR_USART2EN)
#define USART3_CLOCK_ENABLE()		(RCC->APB1ENR |= RCC_APB1ENR_USART3EN)

#define USART1_CLOCK_DISABLE()		(RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN)
#define USART2_CLOCK_DISABLE()		(RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN)
#define USART3_CLOCK_DISABLE()		(RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN)

/*
 * Peripheral enable/disable macros for USARTx peripheral
 */
#define USART1_PERI_ENABLE()		(USART1->CR1 |= USART_CR1_UE)
#define USART2_PERI_ENABLE()		(USART2->CR1 |= USART_CR1_UE)
#define USART3_PERI_ENABLE()		(USART3->CR1 |= USART_CR1_UE)

#define USART1_PERI_DISABLE()		(USART1->CR1 &= ~USART_CR1_UE)
#define USART2_PERI_DISABLE()		(USART2->CR1 &= ~USART_CR1_UE)
#define USART3_PERI_DISABLE()		(USART3->CR1 &= ~USART_CR1_UE)

/*
 * IRQ(Interrupt Request) Numbers of STM32F103RB MCU
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36

#define IRQ_NO_I2C1_EV		31
#define IRQ_NO_I2C1_ER		32
#define IRQ_NO_I2C2_EV		33
#define IRQ_NO_I2C2_ER		34

#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART3		39
#define IRQ_NO_UART4		52
#define IRQ_NO_UART5		53

/*
 * IRQ Priority Macros
 */
#define NVIC_IRQ_PRI47		47
#define NVIC_IRQ_PRI45		45

/*
 * Other macros
 */
#define ENABLE		1U
#define DISABLE		0U
#define FLAG_SET	1U
#define FLAG_RESET	0U

#include "stm32f1xx_gpio.h"
#include "stm32f1xx_usart.h"

#endif /* INC_STM32F1XX_H_ */
