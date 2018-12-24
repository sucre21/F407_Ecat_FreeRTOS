/****************************************Copyright (c)****************************************************
**                                      
**                                 http://www.powermcu.com
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               TouchPanel.h
** Descriptions:            The TouchPanel application function
**
**--------------------------------------------------------------------------------------------------------
** Created by:              AVRman
** Created date:            2010-11-7
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

#ifndef _TOUCHPANEL_H_
#define _TOUCHPANEL_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/*
  #define SPIx                           SPI1
  #define SPIx_CLK                       RCC_APB2Periph_SPI1
  #define SPIx_CLK_INIT                  RCC_APB2PeriphClockCmd

  #define SPIx_SCK_PIN                   GPIO_Pin_3
  #define SPIx_SCK_GPIO_PORT             GPIOB
  #define SPIx_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOB
  #define SPIx_SCK_SOURCE                GPIO_PinSource3
  #define SPIx_SCK_AF                    GPIO_AF_SPI1

  #define SPIx_MISO_PIN                  GPIO_Pin_4
  #define SPIx_MISO_GPIO_PORT            GPIOB
  #define SPIx_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOB
  #define SPIx_MISO_SOURCE               GPIO_PinSource4
  #define SPIx_MISO_AF                   GPIO_AF_SPI1

  #define SPIx_MOSI_PIN                  GPIO_Pin_5
  #define SPIx_MOSI_GPIO_PORT            GPIOB
  #define SPIx_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOB
  #define SPIx_MOSI_SOURCE               GPIO_PinSource5
  #define SPIx_MOSI_AF                   GPIO_AF_SPI1

	#define DESELECT_SPI GPIO_SetBits(GPIOE,GPIO_Pin_2) 
	#define SELECT_SPI GPIO_ResetBits(GPIOE,GPIO_Pin_2) 
*/

  #define SPIx                           SPI3
  #define SPIx_CLK                       RCC_APB1Periph_SPI3
  #define SPIx_CLK_INIT                  RCC_APB1PeriphClockCmd

  #define SPIx_SCK_PIN                   GPIO_Pin_10
  #define SPIx_SCK_GPIO_PORT             GPIOC
  #define SPIx_SCK_GPIO_CLK              RCC_AHB1Periph_GPIOC
  #define SPIx_SCK_SOURCE                GPIO_PinSource3
  #define SPIx_SCK_AF                    GPIO_AF_SPI3

  #define SPIx_MISO_PIN                  GPIO_Pin_11
  #define SPIx_MISO_GPIO_PORT            GPIOC
  #define SPIx_MISO_GPIO_CLK             RCC_AHB1Periph_GPIOC
  #define SPIx_MISO_SOURCE               GPIO_PinSource4
  #define SPIx_MISO_AF                   GPIO_AF_SPI3

  #define SPIx_MOSI_PIN                  GPIO_Pin_12
  #define SPIx_MOSI_GPIO_PORT            GPIOC
  #define SPIx_MOSI_GPIO_CLK             RCC_AHB1Periph_GPIOC
  #define SPIx_MOSI_SOURCE               GPIO_PinSource5
  #define SPIx_MOSI_AF                   GPIO_AF_SPI3

	#define DESELECT_SPI GPIO_SetBits(GPIOA,GPIO_Pin_15) 
	#define SELECT_SPI GPIO_ResetBits(GPIOA,GPIO_Pin_15) 



/* Private function prototypes -----------------------------------------------*/				
void SPI1_GPIO_Init(void);	
uint8_t WR_CMD (uint8_t cmd)  ;
void ADC_GPIO_Configuration(void);
void ADC_Configuration(void);
void Uart_Configuration(void);
void NVIC_Configuration(void);
void TIM_Configuration(uint8_t period)	;
void EXTI2_Configuration(void);
void EXTI9_Configuration(void);
void EXTI12_Configuration(void);
#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/


