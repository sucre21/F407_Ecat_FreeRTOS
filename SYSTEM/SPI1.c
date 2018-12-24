/****************************************Copyright (c)****************************************************
**                                      
**                                 
**
**--------------File Info---------------------------------------------------------------------------------
** File name:               .c
** Descriptions:           SPI1 hardware driver
**
**--------------------------------------------------------------------------------------------------------
** Created by:             	 lyd
** Created date:            2015-05-01
** Version:                 		v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "SPI1.h"

//#include "el9800hw.h"
/*******************************************************************************
* Function Name  : SPI1_Init
* Description    : SPI1 initialize
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void SPI1_Init(void) 
{ 
	
//  SPI_InitTypeDef  SPI_InitStructure;
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
//  /* DISABLE SPI1 */ 
//  SPI_Cmd(SPI1, DISABLE); 
//  /* SPI1 Config -------------------------------------------------------------*/ 
//  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; 
//  SPI_InitStructure.SPI_Mode = SPI_Mode_Master; 
//  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; 
//  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //
//  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //
//  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft; 
//  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; 
//  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
//  SPI_InitStructure.SPI_CRCPolynomial = 7; 
//  SPI_Init(SPI1, &SPI_InitStructure); 
//  /* Enable SPI1 */ 
//  SPI_Cmd(SPI1, ENABLE); 
	
	SPI_InitTypeDef  SPI_InitStructure;

	  /* SPI configuration -------------------------------------------------------*/
  SPI_I2S_DeInit(SPIx);
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_Init(SPIx, &SPI_InitStructure);
  
  /* Enable the SPI peripheral */
  SPI_Cmd(SPIx, ENABLE);

                       
} 

/*******************************************************************************
* Function Name  : SPI_GPIO_Init
* Description    : SPI Port initialize
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void SPI1_GPIO_Init(void) 
{ 
	GPIO_InitTypeDef GPIO_InitStructure;

  /* Peripheral Clock Enable -------------------------------------------------*/
  /* Enable the SPI clock */
  //SPIx_CLK_INIT(SPIx_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
  /* Enable GPIO clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  GPIO_StructInit(&GPIO_InitStructure);
  /* SPI GPIO Configuration --------------------------------------------------*/
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_SPI3);
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_SPI3);
 
 	 /* CS */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	

	
	SPI1_Init(); 
  
} 



/*******************************************************************************
* Function Name  : WR_CMD
* Description    : Read and Wire data to ET1100 
* Input          : - cmd: the data send to ET1100
* Output         : none
* Return         : temp: the data read from ET1100
* Attention		 : None
*******************************************************************************/
 uint8_t WR_CMD (uint8_t cmd)  
{ 
	 uint8_t temp; 

  /* Wait for SPI1 Tx buffer empty */ 
  while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET); //·¢ËÍbufferÎª¿Õ
  /* Send SPI1 data */ 
  SPI_I2S_SendData(SPI3,cmd); 
  /* Wait for SPI1 data reception */ 
  while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET); 
  /* Read SPI1 received data */ 
	temp =  SPI_I2S_ReceiveData(SPI3); 
	return temp;
	
} 


/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
