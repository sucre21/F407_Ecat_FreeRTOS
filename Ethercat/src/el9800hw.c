/**
note:stm32f407硬件初始化，底层HAL函数操作
author:su'ang
date:20180604
*/


/*--------------------------------------------------------------------------------------
------
------    Includes
------
--------------------------------------------------------------------------------------*/
#include "ecat_def.h"
#if EL9800_HW
#include "SPI1.h"
#include "ecatslv.h"

#define    _EL9800HW_ 1
#include "el9800hw.h"
#undef    _EL9800HW_
/* ECATCHANGE_START(V5.11) ECAT10*/
/*remove definition of _EL9800HW_ (#ifdef is used in el9800hw.h)*/
/* ECATCHANGE_END(V5.11) ECAT10*/

#include "ecatappl.h"
//#include "sys.h"

/*--------------------------------------------------------------------------------------
------
------    internal Types and Defines
------
--------------------------------------------------------------------------------------*/

typedef union
{
    unsigned short    Word;
    unsigned char    Byte[2];
} UBYTETOWORD;

typedef union 
{
    UINT8           Byte[2];
    UINT16          Word;
}
UALEVENT;

/*-----------------------------------------------------------------------------------------
------
------    SPI defines/macros
------
-----------------------------------------------------------------------------------------*/
#define SPI_DEACTIVE                    1
#define SPI_ACTIVE                        0


#if INTERRUPTS_SUPPORTED
/*-----------------------------------------------------------------------------------------
------
------    Global Interrupt setting
------
-----------------------------------------------------------------------------------------*/

#define DISABLE_GLOBAL_INT           		   __disable_irq()			
#define ENABLE_GLOBAL_INT           		    __enable_irq()				
#define    DISABLE_AL_EVENT_INT           DISABLE_GLOBAL_INT
#define    ENABLE_AL_EVENT_INT            ENABLE_GLOBAL_INT



/*-----------------------------------------------------------------------------------------
------
------    ESC Interrupt
------
-----------------------------------------------------------------------------------------*/
#if AL_EVENT_ENABLED
#define    INIT_ESC_INT           EXTI2_Configuration();					
#define    EcatIsr                EXTI2_IRQHandler
#define    ACK_ESC_INT         		EXTI_ClearITPendingBit(EXTI_Line2);  

#endif //#if AL_EVENT_ENABLED

/*-----------------------------------------------------------------------------------------
------
------    SYNC0 Interrupt
------
-----------------------------------------------------------------------------------------*/
#if DC_SUPPORTED && _STM32_IO8
#define    INIT_SYNC0_INT                  EXTI9_Configuration();		
#define    Sync0Isr                        				EXTI9_5_IRQHandler // primary interrupt vector name
#define    DISABLE_SYNC0_INT             NVIC_DisableIRQ(EXTI9_5_IRQn);	  // {(_INT3IE)=0;}//disable interrupt source INT3
#define    ENABLE_SYNC0_INT               NVIC_EnableIRQ(EXTI9_5_IRQn);	// {(_INT3IE) = 1;} //enable interrupt source INT3
#define    ACK_SYNC0_INT                  EXTI_ClearITPendingBit(EXTI_Line9);//  {(SYNC0_INT_REQ) = 0;}


/*ECATCHANGE_START(V5.10) HW3*/

#define    INIT_SYNC1_INT                  EXTI12_Configuration();
#define    Sync1Isr                        						EXTI15_10_IRQHandler
#define    DISABLE_SYNC1_INT                 NVIC_DisableIRQ(EXTI15_10_IRQn);// {(_INT4IE)=0;}//disable interrupt source INT4
#define    ENABLE_SYNC1_INT                 NVIC_EnableIRQ(EXTI15_10_IRQn); //{(_INT4IE) = 1;} //enable interrupt source INT4
#define    ACK_SYNC1_INT                  	 EXTI_ClearITPendingBit(EXTI_Line12);// {(SYNC1_INT_REQ) = 0;}

/*ECATCHANGE_END(V5.10) HW3*/

#endif //#if DC_SUPPORTED && _STM32_IO8

#endif	//#if INTERRUPTS_SUPPORTED
/*-----------------------------------------------------------------------------------------
------
------    Hardware timer
------
-----------------------------------------------------------------------------------------*/
#if _STM32_IO8
#if ECAT_TIMER_INT
#define ECAT_TIMER_INT_STATE       
#define ECAT_TIMER_ACK_INT        		 	TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);	
#define    TimerIsr                    	TIM2_IRQHandler //	SysTick_Handler//						
#define    ENABLE_ECAT_TIMER_INT        NVIC_EnableIRQ(TIM2_IRQn) ;	//SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;//NVIC_EnableIRQ(TIM2_IRQn) ;	
#define    DISABLE_ECAT_TIMER_INT       NVIC_DisableIRQ(TIM2_IRQn) ;//SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;//NVIC_DisableIRQ(SysTick_IRQn/*TIM2_IRQn*/) ;

#define INIT_ECAT_TIMER            TIM_Configuration(10) ;  // SysTick_Config(SystemCoreClock/1000);  

#define STOP_ECAT_TIMER            DISABLE_ECAT_TIMER_INT;/*disable timer interrupt*/ \

#define START_ECAT_TIMER          ENABLE_ECAT_TIMER_INT


#else    //#if ECAT_TIMER_INT

#define INIT_ECAT_TIMER      					TIM_Configuration(10) //SysTick_Config(SystemCoreClock/1000);//          {(PR7) = 2000;/*set period*/ \

#define STOP_ECAT_TIMER              TIM_Cmd(TIM2, DISABLE);	// SysTick->CTRL  &=  ~SysTick_CTRL_ENABLE_Msk;      //			 

#define START_ECAT_TIMER              TIM_Cmd(TIM2, ENABLE);// SysTick->CTRL  |=  SysTick_CTRL_ENABLE_Msk;      			

#endif //#else #if ECAT_TIMER_INT

#elif _STM32_IO4

#if !ECAT_TIMER_INT
#define    ENABLE_ECAT_TIMER_INT       NVIC_EnableIRQ(TIM2_IRQn) ;	// SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;//
#define    DISABLE_ECAT_TIMER_INT      NVIC_DisableIRQ(TIM2_IRQn) ;//SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;//
#define INIT_ECAT_TIMER               TIM_Configuration(10) ;//	SysTick_Config(SystemCoreClock/1000);//			
#define STOP_ECAT_TIMER              	TIM_Cmd(TIM2, DISABLE);//	SysTick->CTRL  &=  ~SysTick_CTRL_ENABLE_Msk; //		
#define START_ECAT_TIMER           		TIM_Cmd(TIM2, ENABLE);	//	SysTick->CTRL  |=  SysTick_CTRL_ENABLE_Msk; //		

#else    //#if !ECAT_TIMER_INT

#warning "define Timer Interrupt Macros"

#endif //#else #if !ECAT_TIMER_INT
#endif //#elif _STM32_IO4

/*-----------------------------------------------------------------------------------------
------
------    Configuration Bits
------
-----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
------
------    LED defines
------
-----------------------------------------------------------------------------------------*/
#if _STM32_IO8
// EtherCAT Status LEDs -> StateMachine
#define LED_ECATGREEN               
#define LED_ECATRED                   
#endif //_STM32_IO8


/*--------------------------------------------------------------------------------------
------
------    internal Variables
------
--------------------------------------------------------------------------------------*/
UALEVENT         EscALEvent;            //contains the content of the ALEvent register (0x220), this variable is updated on each Access to the Esc

/*--------------------------------------------------------------------------------------
------
------    internal functions
------
--------------------------------------------------------------------------------------*/


/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief  The function operates a SPI access without addressing.

        The first two bytes of an access to the EtherCAT ASIC always deliver the AL_Event register (0x220).
        It will be saved in the global "EscALEvent"
*////////////////////////////////////////////////////////////////////////////////////////
static void GetInterruptRegister(void)
{

    #if AL_EVENT_ENABLED
    DISABLE_AL_EVENT_INT;
#endif
	
    /* select the SPI */
    SELECT_SPI;
	
	 HW_EscReadIsr((MEM_ADDR *)&EscALEvent.Word, 0x220, 2);
	/* if the SPI transmission rate is higher than 15 MBaud, the Busy detection shall be
       done here */
	
  DESELECT_SPI;
 #if AL_EVENT_ENABLED
    ENABLE_AL_EVENT_INT;
#endif
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief  The function operates a SPI access without addressing.
        Shall be implemented if interrupts are supported else this function is equal to "GetInterruptRegsiter()"

        The first two bytes of an access to the EtherCAT ASIC always deliver the AL_Event register (0x220).
        It will be saved in the global "EscALEvent"
*////////////////////////////////////////////////////////////////////////////////////////
#if !INTERRUPTS_SUPPORTED
#define ISR_GetInterruptRegister GetInterruptRegister
#else
static void ISR_GetInterruptRegister(void)
{
    /* SPI should be deactivated to interrupt a possible transmission */
    DESELECT_SPI;

    /* select the SPI */
    SELECT_SPI;

		 HW_EscReadIsr((MEM_ADDR *)&EscALEvent.Word, 0x220, 2);

  /* if the SPI transmission rate is higher than 15 MBaud, the Busy detection shall be
       done here */

    DESELECT_SPI;
}
#endif //#else #if !INTERRUPTS_SUPPORTED

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief The function addresses the EtherCAT ASIC via SPI for a following SPI access.
*////////////////////////////////////////////////////////////////////////////////////////
static void AddressingEsc( UINT16 Address, UINT8 Command )
{
    UBYTETOWORD tmp;
    VARVOLATILE UINT8 dummy;
    tmp.Word = ( Address << 3 ) | Command;
    /* select the SPI */
    SELECT_SPI;


   /* there have to be at least 15 ns after the SPI1_SEL signal was active (0) before
       the transmission shall be started */
    /* send the first address/command byte to the ESC */
	
		dummy = 	WR_CMD(tmp.Byte[1]);
	
    /* send the second address/command byte to the ESC */
 
	dummy =	WR_CMD(tmp.Byte[0]);
	
    /* if the SPI transmission rate is higher than 15 MBaud, the Busy detection shall be
       done here */
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param Address     EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
 \param Command    ESC_WR performs a write access; ESC_RD performs a read access.

 \brief The function addresses the EtherCAT ASIC via SPI for a following SPI access.
        Shall be implemented if interrupts are supported else this function is equal to "AddressingEsc()"
*////////////////////////////////////////////////////////////////////////////////////////

#if !INTERRUPTS_SUPPORTED
#define ISR_AddressingEsc AddressingEsc
#else
static void ISR_AddressingEsc( UINT16 Address, UINT8 Command )
{
    VARVOLATILE UINT8 dummy;
    UBYTETOWORD tmp;
    tmp.Word = ( Address << 3 ) | Command;

    /* select the SPI */
    SELECT_SPI;

    /* there have to be at least 15 ns after the SPI1_SEL signal was active (0) before
      the transmission shall be started */
	
    /* send the first address/command byte to the ESC */
	dummy= WR_CMD(tmp.Byte[1]);
	
   /* send the second address/command byte to the ESC */

	dummy= WR_CMD(tmp.Byte[0]);
    /* if the SPI transmission rate is higher than 15 MBaud, the Busy detection shall be
       done here */
}
#endif //#else #if !INTERRUPTS_SUPPORTED
/*--------------------------------------------------------------------------------------
------
------    exported hardware access functions
------
--------------------------------------------------------------------------------------*/
/*******************************************************************************
* Function Name  : GPIO_Config
  PE11:LED_WORK
*******************************************************************************/
void GPIO_Config(void) 
{ 
//  GPIO_InitTypeDef GPIO_InitStructure;

//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
//  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11; 
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
//  GPIO_Init(GPIOE, &GPIO_InitStructure); 
//	
} 

/////////////////////////////////////////////////////////////////////////////////////////
/**
  硬件初始化函数
*////////////////////////////////////////////////////////////////////////////////////////
UINT8 HW_Init(void)
{
  UINT16 intMask;
 
	//GPIO_Config();              //LED
	//SPI1_GPIO_Init();           //SPI1
	ADC_Configuration();        //ADC1
	//Uart_Configuration();       //Uart2

   do
   {
        intMask = 0x93;
        HW_EscWriteWord(intMask, ESC_AL_EVENTMASK_OFFSET);
        intMask = 0;
        HW_EscReadWord(intMask, ESC_AL_EVENTMASK_OFFSET);
    } while (intMask != 0x93);
		
		

		intMask = 0x00;
	  
    HW_EscWriteDWord(intMask, ESC_AL_EVENTMASK_OFFSET);

    INIT_ESC_INT          //外部中断3 ，spi_irq
		ENABLE_ESC_INT();

    INIT_SYNC0_INT         //外部中断1,sync0
    INIT_SYNC1_INT         //外部中断2,sync1

    ENABLE_SYNC0_INT;
    ENABLE_SYNC1_INT;

    INIT_ECAT_TIMER;        //TIM2，定时中断
    START_ECAT_TIMER;

    ENABLE_GLOBAL_INT;

    return 0;
}



/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    This function shall be implemented if hardware resources need to be release
        when the sample application stops
*////////////////////////////////////////////////////////////////////////////////////////
void HW_Release(void)
{

}

/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    first two Bytes of ALEvent register (0x220)

 \brief  This function gets the current content of ALEvent register
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 HW_GetALEventRegister(void)
{
    GetInterruptRegister();
    return EscALEvent.Word;
}
#if INTERRUPTS_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \return    first two Bytes of ALEvent register (0x220)

 \brief  The SPI PDI requires an extra ESC read access functions from interrupts service routines.
        The behaviour is equal to "HW_GetALEventRegister()"
*////////////////////////////////////////////////////////////////////////////////////////
#if _STM32_IO4  && AL_EVENT_ENABLED
/* the pragma interrupt_level is used to tell the compiler that these functions will not
   be called at the same time from the main function and the interrupt routine */
//#pragma interrupt_level 1
#endif
UINT16 HW_GetALEventRegister_Isr(void)
{
     ISR_GetInterruptRegister();
    return EscALEvent.Word;
}
#endif


#if UC_SET_ECAT_LED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param RunLed            desired EtherCAT Run led state
 \param ErrLed            desired EtherCAT Error led state

  \brief    This function updates the EtherCAT run and error led
*////////////////////////////////////////////////////////////////////////////////////////
void HW_SetLed(UINT8 RunLed,UINT8 ErrLed)
{
#if _STM32_IO8
 //     LED_ECATGREEN = RunLed;
//      LED_ECATRED   = ErrLed;
#endif
}
#endif //#if UC_SET_ECAT_LED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param pData        Pointer to a byte array which holds data to write or saves read data.
 \param Address     EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
 \param Len            Access size in Bytes.

 \brief  This function operates the SPI read access to the EtherCAT ASIC.
*////////////////////////////////////////////////////////////////////////////////////////
void HW_EscRead( MEM_ADDR *pData, UINT16 Address, UINT16 Len )
{
    /* HBu 24.01.06: if the SPI will be read by an interrupt routine too the
                     mailbox reading may be interrupted but an interrupted
                     reading will remain in a SPI transmission fault that will
                     reset the internal Sync Manager status. Therefore the reading
                     will be divided in 1-byte reads with disabled interrupt */
    UINT16 i = Len;
    UINT8 *pTmpData = (UINT8 *)pData;

    /* loop for all bytes to be read */
    while ( i-- > 0 )
    {
			#if AL_EVENT_ENABLED
        /* the reading of data from the ESC can be interrupted by the
           AL Event ISR, in that case the address has to be reinitialized,
           in that case the status flag will indicate an error because
           the reading operation was interrupted without setting the last
           sent byte to 0xFF */
        DISABLE_AL_EVENT_INT;
#endif
         AddressingEsc( Address, ESC_RD );

        /* when reading the last byte the DI pin shall be 1 */
			*pTmpData++= WR_CMD(0xFF);
        /* enable the ESC interrupt to get the AL Event ISR the chance to interrupt,
           if the next byte is the last the transmission shall not be interrupted,
           otherwise a sync manager could unlock the buffer, because the last was
           read internally */
   #if AL_EVENT_ENABLED
        ENABLE_AL_EVENT_INT;
#endif
        /* there has to be at least 15 ns + CLK/2 after the transmission is finished
           before the SPI1_SEL signal shall be 1 */
        DESELECT_SPI;
        /* next address */
        Address++;
//        /* reset transmission flag */
//        SPI1_IF = 0;
    }
}
#if INTERRUPTS_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param pData        Pointer to a byte array which holds data to write or saves read data.
 \param Address     EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
 \param Len            Access size in Bytes.

\brief  The SPI PDI requires an extra ESC read access functions from interrupts service routines.
        The behaviour is equal to "HW_EscRead()"
*////////////////////////////////////////////////////////////////////////////////////////
#if _STM32_IO4  && AL_EVENT_ENABLED
/* the pragma interrupt_level is used to tell the compiler that these functions will not
   be called at the same time from the main function and the interrupt routine */
//#pragma interrupt_level 1
#endif
void HW_EscReadIsr( MEM_ADDR *pData, UINT16 Address, UINT16 Len )     //硬件层的读取数据操作
{
    UINT16 i = Len;
    UINT8 data = 0;

   UINT8 *pTmpData = (UINT8 *)pData;

    /* send the address and command to the ESC */
     ISR_AddressingEsc( Address, ESC_RD );
    /* loop for all bytes to be read */
    while ( i-- > 0 )
    {
        if ( i == 0 )
        {
            /* when reading the last byte the DI pin shall be 1 */
            data = 0xFF;
        }

		*pTmpData++= WR_CMD(data);
    }

    /* there has to be at least 15 ns + CLK/2 after the transmission is finished
       before the SPI1_SEL signal shall be 1 */
    DESELECT_SPI;
}
#endif //#if INTERRUPTS_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param pData        Pointer to a byte array which holds data to write or saves write data.
 \param Address     EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
 \param Len            Access size in Bytes.

  \brief  This function operates the SPI write access to the EtherCAT ASIC.
*////////////////////////////////////////////////////////////////////////////////////////
void HW_EscWrite( MEM_ADDR *pData, UINT16 Address, UINT16 Len )    
{
    UINT16 i = Len;
    VARVOLATILE UINT8 dummy;

    UINT8 *pTmpData = (UINT8 *)pData;

    /* loop for all bytes to be written */
    while ( i-- > 0 )
    {
   #if AL_EVENT_ENABLED
        /* the reading of data from the ESC can be interrupted by the
           AL Event ISR, so every byte will be written separate */
        DISABLE_AL_EVENT_INT;
#endif
        /* HBu 24.01.06: wrong parameter ESC_RD */
         AddressingEsc( Address, ESC_WR );

        /* enable the ESC interrupt to get the AL Event ISR the chance to interrupt */
        /* SPI1_BUF must be read, otherwise the module will not transfer the next received data from SPIxSR to SPIxRXB.*/
			dummy= WR_CMD(*pTmpData++);
			
    #if AL_EVENT_ENABLED
        ENABLE_AL_EVENT_INT;
#endif

        DESELECT_SPI;
        /* next address */
        Address++;
    }
}
#if INTERRUPTS_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \param pData        Pointer to a byte array which holds data to write or saves write data.
 \param Address     EtherCAT ASIC address ( upper limit is 0x1FFF )    for access.
 \param Len            Access size in Bytes.

 \brief  The SPI PDI requires an extra ESC write access functions from interrupts service routines.
        The behaviour is equal to "HW_EscWrite()"
*////////////////////////////////////////////////////////////////////////////////////////
#if _STM32_IO4  && AL_EVENT_ENABLED
/* the pragma interrupt_level is used to tell the compiler that these functions will not
   be called at the same time from the main function and the interrupt routine */
//#pragma interrupt_level 1
#endif
void HW_EscWriteIsr( MEM_ADDR *pData, UINT16 Address, UINT16 Len )   //硬件层发送操作
{
    UINT16 i = Len;
    VARVOLATILE UINT16 dummy;
    UINT8 *pTmpData = (UINT8 *)pData;

    /* send the address and command to the ESC */
     ISR_AddressingEsc( Address, ESC_WR );
    /* loop for all bytes to be written */
    while ( i-- > 0 )
    {
        /* start transmission */
			dummy= WR_CMD(*pTmpData);
      /* increment data pointer */
        pTmpData++;
    }

    /* there has to be at least 15 ns + CLK/2 after the transmission is finished
       before the SPI1_SEL signal shall be 1 */
    DESELECT_SPI;
}
#endif //#if INTERRUPTS_SUPPORTED


#if BOOTSTRAPMODE_SUPPORTED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    This function resets the hardware
*////////////////////////////////////////////////////////////////////////////////////////

void HW_RestartTarget(void)
{
}
#endif

#if ESC_EEPROM_EMULATION
/////////////////////////////////////////////////////////////////////////////////////////
/**
\return     0 if reload was successful

 \brief    This function is called when the master has request an EEPROM reload during EEPROM emulation

*////////////////////////////////////////////////////////////////////////////////////////
UINT16 HW_EepromReload (void)
{
    return 0;
}
#endif

#if AL_EVENT_ENABLED
/////////////////////////////////////////////////////////////////////////////////////////
/**
 \brief    Interrupt service routine for the PDI interrupt from the EtherCAT Slave Controller
*////////////////////////////////////////////////////////////////////////////////////////

#if _STM32_IO4

void EcatIsr(void)
#else
void EcatIsr(void)                                              //spi_irq中断，读写操作
#endif
{

			
    PDI_Isr();
	

    /* reset the interrupt flag */
		ACK_ESC_INT; 
}
#endif     // AL_EVENT_ENABLED

void Sync0Isr(void)                                                                   //同步0事件中断
{
    Sync0_Isr();
    /* reset the interrupt flag */
    ACK_SYNC0_INT;
}

void Sync1Isr(void)                                                                 //同步1事件中断
{	
	GPIO_ToggleBits(GPIOE, GPIO_Pin_11);	
	
	Sync1_Isr();
	
	ACK_SYNC1_INT;
}


// Timer 2 ISR (0.1ms)
void TimerIsr(void)                                                               //TIM2定时中断
{
			DISABLE_ESC_INT();
		
		  ECAT_CheckTimer();

			ECAT_TIMER_ACK_INT;
		
			ENABLE_ESC_INT();
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
  串口空闲中断，读取来自IMU的数据，52字节，读完一帧后中断，1ms周期，中断内处理数据更新
*////////////////////////////////////////////////////////////////////////////////////////

UINT32 readdata[11]={0};                             //传感器数据float
UINT16 readerr=0;

UINT16 getcheck()                                //校验
{
	UINT8 check1=0,check2=0;
	UINT8 i=0;
	for(i=0;i<50;i++)
	{
		check1+=read[i];
		check2+=check1;
	}
	if((check1==read[50])&&(check2==read[51]))  return 1;
	else  return 0;
}
/*
void USART2_IRQHandler(void)                                                        //串口接收中断
{
	UINT16 checkstatus=0;

	if(USART_GetITStatus(USART2,USART_IT_IDLE)!=RESET)
	{
		
		USART2->SR;
		USART2->DR;  //清标志位
		
    DMA_Cmd(DMA1_Stream5, DISABLE);
		DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5);
		DMA_Cmd(DMA1_Stream5, ENABLE);
		checkstatus=getcheck();
		if((checkstatus==1)&&(read[0]==0x75))   //校验及帧头正确
		{
			readdata[0]=(read[6]<<24)+(read[7]<<16)+(read[8]<<8)+read[9];
			readdata[1]=(read[10]<<24)+(read[11]<<16)+(read[12]<<8)+read[13];
			readdata[2]=(read[14]<<24)+(read[15]<<16)+(read[16]<<8)+read[17];
			readdata[3]=(read[20]<<24)+(read[21]<<16)+(read[22]<<8)+read[23];
			readdata[4]=(read[24]<<24)+(read[25]<<16)+(read[26]<<8)+read[27];
			readdata[5]=(read[28]<<24)+(read[29]<<16)+(read[30]<<8)+read[31];
			readdata[6]=(read[34]<<24)+(read[35]<<16)+(read[36]<<8)+read[37];
			readdata[7]=(read[38]<<24)+(read[39]<<16)+(read[40]<<8)+read[41];
			readdata[8]=(read[42]<<24)+(read[43]<<16)+(read[44]<<8)+read[45];
			readdata[9]=(read[46]<<24)+(read[47]<<16)+(read[48]<<8)+read[49];
			readdata[10]=readerr;
		}
		else
		{
			//USART_Cmd(USART2, DISABLE);
			//DMA_Cmd(DMA1_Stream5, DISABLE);
			readerr++;
			//USART_Cmd(USART2, ENABLE);
			//DMA_Cmd(DMA1_Stream5, ENABLE);
		}
	}
}
*/

#endif //#if EL9800_HW
/** @} */




