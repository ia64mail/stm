/**
  ******************************************************************************
  * @file    main.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4_DISCOVERY_DEMO_H
#define __STM32F4_DISCOVERY_DEMO_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <stdio.h>

/** @addtogroup STM32F4_DISCOVERY_LOW_LEVEL_LED
  * @{
  */

/* RX operation in progress  - orange */
#define LED_RX_PROGRESS_GPIO_PIN          GPIO_Pin_13
#define LED_RX_PROGRESS_GPIO_PORT         GPIOD
#define LED_RX_PROGRESS_GPIO_BUS          RCC_AHB1Periph_GPIOD  
#define LED_RX_PROGRESS_GPIO_MODE					GPIO_Mode_OUT
#define LED_RX_PROGRESS_GPIO_SPEED				GPIO_Speed_25MHz
#define LED_RX_PROGRESS_GPIO_OTYPE				GPIO_OType_PP
#define LED_RX_PROGRESS_GPIO_PTYPE				GPIO_PuPd_UP

/* AT Command pass - green */
#define LED_AT_PASS_GPIO_PIN              GPIO_Pin_12
#define LED_AT_PASS_GPIO_PORT             GPIOD
#define LED_AT_PASS_GPIO_BUS              RCC_AHB1Periph_GPIOD  
#define LED_AT_PASS_GPIO_MODE							GPIO_Mode_OUT
#define LED_AT_PASS_GPIO_SPEED						GPIO_Speed_25MHz
#define LED_AT_PASS_GPIO_OTYPE						GPIO_OType_PP
#define LED_AT_PASS_GPIO_PTYPE						GPIO_PuPd_UP

/* AT Command fail - red */
#define LED_AT_FAIL_GPIO_PIN              GPIO_Pin_14
#define LED_AT_FAIL_GPIO_PORT             GPIOD
#define LED_AT_FAIL_GPIO_BUS              RCC_AHB1Periph_GPIOD  
#define LED_AT_FAIL_GPIO_MODE							GPIO_Mode_OUT
#define LED_AT_FAIL_GPIO_SPEED						GPIO_Speed_25MHz
#define LED_AT_FAIL_GPIO_OTYPE						GPIO_OType_PP
#define LED_AT_FAIL_GPIO_PTYPE						GPIO_PuPd_UP

/* TX operation in progress  - blue */
#define LED_TX_PROGRESS_GPIO_PIN          GPIO_Pin_15
#define LED_TX_PROGRESS_GPIO_PORT         GPIOD
#define LED_TX_PROGRESS_GPIO_BUS          RCC_AHB1Periph_GPIOD  
#define LED_TX_PROGRESS_GPIO_MODE					GPIO_Mode_OUT
#define LED_TX_PROGRESS_GPIO_SPEED				GPIO_Speed_25MHz
#define LED_TX_PROGRESS_GPIO_OTYPE				GPIO_OType_PP
#define LED_TX_PROGRESS_GPIO_PTYPE				GPIO_PuPd_UP

/**
 * @brief Wakeup push-button
 */
#define USER_BUTTON_GPIO_PIN           GPIO_Pin_0
#define USER_BUTTON_GPIO_PORT          GPIOA
#define USER_BUTTON_GPIO_BUS           RCC_AHB1Periph_GPIOA
#define USER_BUTTON_GPIO_MODE					 GPIO_Mode_IN
#define USER_BUTTON_GPIO_SPEED				 GPIO_Speed_25MHz
#define USER_BUTTON_GPIO_OTYPE				 GPIO_OType_PP
#define USER_BUTTON_GPIO_PTYPE				 GPIO_PuPd_NOPULL
#define USER_BUTTON_EXTI_LINE          EXTI_Line0
#define USER_BUTTON_EXTI_MODE          EXTI_Mode_Interrupt
#define USER_BUTTON_EXTI_TRIGER        EXTI_Trigger_Rising
#define USER_BUTTON_EXTI_PORT_SOURCE   EXTI_PortSourceGPIOA
#define USER_BUTTON_EXTI_PIN_SOURCE    EXTI_PinSource0
#define USER_BUTTON_NVIC_IRQn          EXTI0_IRQn
#define USER_BUTTON_NVIC_PRIORITY      0x0
#define USER_BUTTON_NVIC_SUBPRIORITY   0x0

/**
 * @brief USATR3 port pins
 */
 
/* RX input pin  - PD9 */
#define USART3_RX_GPIO_PIN							GPIO_Pin_9
#define USART3_RX_GPIO_PORT          		GPIOD
#define USART3_RX_GPIO_BUS           		RCC_AHB1Periph_GPIOD
#define USART3_RX_GPIO_MODE							GPIO_Mode_AF
#define USART3_RX_GPIO_SPEED						GPIO_Speed_25MHz
#define USART3_RX_GPIO_OTYPE						GPIO_OType_PP
#define USART3_RX_GPIO_PTYPE						GPIO_PuPd_NOPULL
#define USART3_RX_GPIO_PIN_SOURCE				GPIO_PinSource9
#define USART3_RX_GPIO_AF_MODE					GPIO_AF_USART3

/* DCD input pin - PD10 */
#define USART3_DCD_GPIO_PIN							GPIO_Pin_10
#define USART3_DCD_GPIO_PORT          	GPIOD
#define USART3_DCD_GPIO_BUS           	RCC_AHB1Periph_GPIOD
#define USART3_DCD_GPIO_MODE						GPIO_Mode_IN
#define USART3_DCD_GPIO_SPEED						GPIO_Speed_25MHz
#define USART3_DCD_GPIO_OTYPE						GPIO_OType_PP
#define USART3_DCD_GPIO_PTYPE						GPIO_PuPd_NOPULL
#define USART3_DCD_EXTI_LINE          	EXTI_Line10
#define USART3_DCD_EXTI_MODE          	EXTI_Mode_Interrupt
#define USART3_DCD_EXTI_TRIGER          EXTI_Trigger_Rising
#define USART3_DCD_EXTI_PORT_SOURCE   	EXTI_PortSourceGPIOD
#define USART3_DCD_EXTI_PIN_SOURCE    	EXTI_PinSource10
#define USART3_DCD_NVIC_IRQn          	EXTI15_10_IRQn
#define USART3_DCD_NVIC_PRIORITY        0x0
#define USART3_DCD_NVIC_SUBPRIORITY     0x0

/* CTS input pin - PD11 */
#define USART3_CTS_GPIO_PIN							GPIO_Pin_11
#define USART3_CTS_GPIO_PORT          	GPIOD
#define USART3_CTS_GPIO_BUS           	RCC_AHB1Periph_GPIOD
#define USART3_CTS_GPIO_MODE						GPIO_Mode_AF
#define USART3_CTS_GPIO_SPEED						GPIO_Speed_25MHz
#define USART3_CTS_GPIO_OTYPE						GPIO_OType_PP
#define USART3_CTS_GPIO_PTYPE						GPIO_PuPd_NOPULL
#define USART3_CTS_GPIO_PIN_SOURCE			GPIO_PinSource11
#define USART3_CTS_GPIO_AF_MODE					GPIO_AF_USART3

/* RI input pin - PB13 */
#define USART3_RI_GPIO_PIN							GPIO_Pin_13
#define USART3_RI_GPIO_PORT          		GPIOB
#define USART3_RI_GPIO_BUS           		RCC_AHB1Periph_GPIOB
#define USART3_RI_GPIO_MODE							GPIO_Mode_IN
#define USART3_RI_GPIO_SPEED						GPIO_Speed_25MHz
#define USART3_RI_GPIO_OTYPE						GPIO_OType_PP
#define USART3_RI_GPIO_PTYPE						GPIO_PuPd_NOPULL
#define USART3_RI_EXTI_LINE          		EXTI_Line13
#define USART3_RI_EXTI_MODE          		EXTI_Mode_Interrupt
#define USART3_RI_EXTI_TRIGER          	EXTI_Trigger_Rising
#define USART3_RI_EXTI_PORT_SOURCE   		EXTI_PortSourceGPIOB
#define USART3_RI_EXTI_PIN_SOURCE    		EXTI_PinSource13
#define USART3_RI_NVIC_IRQn          		EXTI15_10_IRQn
#define USART3_RI_NVIC_PRIORITY        	0x0
#define USART3_RI_NVIC_SUBPRIORITY     	0x0

/* TX output pin - PD8 */
#define USART3_TX_GPIO_PIN							GPIO_Pin_8
#define USART3_TX_GPIO_PORT          		GPIOD
#define USART3_TX_GPIO_BUS           		RCC_AHB1Periph_GPIOD
#define USART3_TX_GPIO_MODE							GPIO_Mode_AF
#define USART3_TX_GPIO_SPEED						GPIO_Speed_25MHz
#define USART3_TX_GPIO_OTYPE						GPIO_OType_OD
#define USART3_TX_GPIO_PTYPE						GPIO_PuPd_NOPULL
#define USART3_TX_GPIO_PIN_SOURCE				GPIO_PinSource8
#define USART3_TX_GPIO_AF_MODE					GPIO_AF_USART3

/* RTS output pin - PB14 */
#define USART3_RTS_GPIO_PIN							GPIO_Pin_14
#define USART3_RTS_GPIO_PORT          	GPIOB
#define USART3_RTS_GPIO_BUS           	RCC_AHB1Periph_GPIOB
#define USART3_RTS_GPIO_MODE						GPIO_Mode_AF
#define USART3_RTS_GPIO_SPEED						GPIO_Speed_25MHz
#define USART3_RTS_GPIO_OTYPE						GPIO_OType_OD
#define USART3_RTS_GPIO_PTYPE						GPIO_PuPd_NOPULL
#define USART3_RTS_GPIO_PIN_SOURCE			GPIO_PinSource14
#define USART3_RTS_GPIO_AF_MODE					GPIO_AF_USART3

/* DTR output pin - PB15 */
#define USART3_DTR_GPIO_PIN							GPIO_Pin_15
#define USART3_DTR_GPIO_PORT          	GPIOB
#define USART3_DTR_GPIO_BUS           	RCC_AHB1Periph_GPIOB
#define USART3_DTR_GPIO_MODE						GPIO_Mode_OUT
#define USART3_DTR_GPIO_SPEED						GPIO_Speed_25MHz
#define USART3_DTR_GPIO_OTYPE						GPIO_OType_OD
#define USART3_DTR_GPIO_PTYPE						GPIO_PuPd_NOPULL

/**
 * @brief USATR3 device
 */

#define USART                           USART3
#define USART_BUS                       RCC_APB1Periph_USART3
#define USART_BAUND_RATE								9600
#define USART_WORD_LENGHT								USART_WordLength_8b
#define USART_STOP_BIT									USART_StopBits_1								
#define USART_PARITY										USART_Parity_No							
#define USART_FLOW_CONTROL							USART_HardwareFlowControl_RTS_CTS
#define USART_OPERATION_MODE						USART_Mode_Rx | USART_Mode_Tx
#define USART_NVIC_IRQn                 USART3_IRQn
#define USART_NVIC_PRIORITY             0x0
#define USART_NVIC_SUBPRIORITY          0x0


void enableLed(GPIO_TypeDef * port, uint16_t pin);
void disableLed(GPIO_TypeDef * port, uint16_t pin);
void disabelAllLeds(void);

#endif
