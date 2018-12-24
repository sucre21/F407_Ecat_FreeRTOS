
/*-----------------------------------------------------------------------------------------
note:        工程为传感器从站软件，包含ecat的spi底层函数操作，协议栈和coe协议，需修改spi底层接口，数据mapping函数，
             中断接口，coe对象字典和对应xml文件，DC触发函数；修改传感器外设接口及数据处理操作。
author：     su'ang
date:        2018.6.4
vision：     V1.0.0
-----------------------------------------------------------------------------------------*/

//包含头文件
#include "ecat_def.h"

#if EL9800_APPLICATION

/* ECATCHANGE_START(V5.11) ECAT11*/
#include "applInterface.h"
/* ECATCHANGE_END(V5.11) ECAT11*/

//#include "el9800hw.h"

#define _EVALBOARD_
#include "el9800appl.h"
#undef _EVALBOARD_

#if MCI_HW
#include "mcihw.h"
#endif
#if EL9800_HW
#include "el9800hw.h"
#endif

#include "delay.h"
//#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"

//应用函数
void    APPL_AckErrorInd(UINT16 stateTrans)
{ 
}

UINT16 APPL_StartMailboxHandler(void)                            //状态机，初始化到预运行
{
    return ALSTATUSCODE_NOERROR;
}

UINT16 APPL_StopMailboxHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

UINT16 APPL_StartInputHandler(UINT16 *pIntMask)               //状态机，预运行到安全运行
{
    return ALSTATUSCODE_NOERROR;
}

UINT16 APPL_StopInputHandler(void)
{
    return ALSTATUSCODE_NOERROR;
}

UINT16 APPL_StartOutputHandler(void)                      //状态机，安全运行到运行
{
    return ALSTATUSCODE_NOERROR;
}

UINT16 APPL_StopOutputHandler(void)
{
/*ECATCHANGE_START(V5.11) EL9800 1*/
    sDOOutputs.bLED1 = 0;
    sDOOutputs.bLED2 = 0;
    sDOOutputs.bLED3 = 0;
    sDOOutputs.bLED4 = 0;
#if _STM32_IO8
    sDOOutputs.bLED5 = 0;
    sDOOutputs.bLED7 = 0;
    sDOOutputs.bLED6 = 0;
    sDOOutputs.bLED8 = 0;
#endif
    

#endif
/*ECATCHANGE_END(V5.11) EL9800 1*/
    return ALSTATUSCODE_NOERROR;
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\param      计算pInputSize 和pOutputSize个数
*////////////////////////////////////////////////////////////////////////////////////////
UINT16 APPL_GenerateMapping(UINT16* pInputSize,UINT16* pOutputSize)
{
#if COE_SUPPORTED
    UINT16 result = ALSTATUSCODE_NOERROR;
    UINT16 PDOAssignEntryCnt = 0;
    OBJCONST TOBJECT OBJMEM * pPDO = NULL;
    UINT16 PDOSubindex0 = 0;
    UINT32 *pPDOEntry = NULL;
    UINT16 PDOEntryCnt = 0;
    UINT16 InputSize = 0;
    UINT16 OutputSize = 0;

    /*Scan object 0x1C12 RXPDO assign*/
    for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sRxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
    {
        pPDO = OBJ_GetObjectHandle(sRxPDOassign.aEntries[PDOAssignEntryCnt]);
        if(pPDO != NULL)
        {
            PDOSubindex0 = *((UINT16 *)pPDO->pVarPtr);
            for(PDOEntryCnt = 0; PDOEntryCnt < PDOSubindex0; PDOEntryCnt++)
            {
			//				result = *(UINT8 *)pPDO->pVarPtr;
			//				result=(OBJ_GetEntryOffset((PDOEntryCnt+1),pPDO)>>3);
                pPDOEntry = (UINT32 *)((UINT8 *)pPDO->pVarPtr + (OBJ_GetEntryOffset((PDOEntryCnt+1),pPDO)>>3));    //goto PDO entry
                // we increment the expected output size depending on the mapped Entry
                OutputSize += (UINT16) ((*pPDOEntry) & 0xFF);
            }
        }
        else
        {
            /*assigned PDO was not found in object dictionary. return invalid mapping*/
            OutputSize = 0;
            result = ALSTATUSCODE_INVALIDOUTPUTMAPPING;
            break;
        }
    }

    OutputSize = (OutputSize + 7) >> 3;

    if(result == 0)
    {
        /*Scan Object 0x1C13 TXPDO assign*/
        for(PDOAssignEntryCnt = 0; PDOAssignEntryCnt < sTxPDOassign.u16SubIndex0; PDOAssignEntryCnt++)
        {
            pPDO = OBJ_GetObjectHandle(sTxPDOassign.aEntries[PDOAssignEntryCnt]);
            if(pPDO != NULL)
            {
                PDOSubindex0 = *((UINT16 *)pPDO->pVarPtr);
                for(PDOEntryCnt = 0; PDOEntryCnt < PDOSubindex0; PDOEntryCnt++)
                {
                    pPDOEntry = (UINT32 *)((UINT8 *)pPDO->pVarPtr + (OBJ_GetEntryOffset((PDOEntryCnt+1),pPDO)>>3));    //goto PDO entry
                    // we increment the expected output size depending on the mapped Entry
                    InputSize += (UINT16) ((*pPDOEntry) & 0xFF);
                }
            }
            else
            {
                /*assigned PDO was not found in object dictionary. return invalid mapping*/
                InputSize = 0;
                result = ALSTATUSCODE_INVALIDINPUTMAPPING;
                break;
            }
        }
    }
    InputSize = (InputSize + 7) >> 3;

    *pInputSize = InputSize;
    *pOutputSize = OutputSize;
    return result;
#else
    *pInputSize = 6;
    *pOutputSize = 2;
    return ALSTATUSCODE_NOERROR;
#endif

}


/////////////////////////////////////////////////////////////////////////////////////////
/**
从应用变量传入中间数组中.对象字典的操作
*////////////////////////////////////////////////////////////////////////////////////////
#if _STM32_IO4 && AL_EVENT_ENABLED
#pragma interrupt_level 1
#endif
void APPL_InputMapping(UINT16* pData)
{
    UINT16 j = 0;
    UINT16 *pTmpData = (UINT16 *)pData;

   for (j = 0; j < sTxPDOassign.u16SubIndex0; j++)  //0x02,｛0x1A00,0x1A02｝
   {
      switch (sTxPDOassign.aEntries[j])//j=0,1,依次上传
      {
      /* TxPDO 1 */
      case 0x1A00:
         *pTmpData++ = SWAPWORD(((UINT16 *) &sDIInputs)[1]); //第二个元素按位定义,存入aPdInputData[0]
         break;
      /* TxPDO 3 */
      case 0x1A02:
         *pTmpData++ = SWAPWORD(((UINT16 *) &sAIInputs)[1]); //analog input 0
         *pTmpData++ = SWAPWORD(((UINT16 *) &sAIInputs)[2]); //analog input 1
			   *pTmpData++ = sAIInputs.Imuinput0 & 0x0000FFFF;
			   *pTmpData++ = (sAIInputs.Imuinput0 & 0xFFFF0000)>>16; //imuinput0
			   *pTmpData++ = sAIInputs.Imuinput1 & 0x0000FFFF;
			   *pTmpData++ = (sAIInputs.Imuinput1 & 0xFFFF0000)>>16; //imuinput0
			   *pTmpData++ = sAIInputs.Imuinput2 & 0x0000FFFF;
			   *pTmpData++ = (sAIInputs.Imuinput2 & 0xFFFF0000)>>16; //imuinput0
			   *pTmpData++ = sAIInputs.Imuinput3 & 0x0000FFFF;
			   *pTmpData++ = (sAIInputs.Imuinput3 & 0xFFFF0000)>>16; //imuinput0
			   *pTmpData++ = sAIInputs.Imuinput4 & 0x0000FFFF;
			   *pTmpData++ = (sAIInputs.Imuinput4 & 0xFFFF0000)>>16; //imuinput0
			   *pTmpData++ = sAIInputs.Imuinput5 & 0x0000FFFF;
			   *pTmpData++ = (sAIInputs.Imuinput5 & 0xFFFF0000)>>16; //imuinput0
			   *pTmpData++ = sAIInputs.Imuinput6 & 0x0000FFFF;
			   *pTmpData++ = (sAIInputs.Imuinput6 & 0xFFFF0000)>>16; //imuinput0
			   *pTmpData++ = sAIInputs.Imuinput7 & 0x0000FFFF;
			   *pTmpData++ = (sAIInputs.Imuinput7 & 0xFFFF0000)>>16; //imuinput0
			   *pTmpData++ = sAIInputs.Imuinput8 & 0x0000FFFF;
			   *pTmpData++ = (sAIInputs.Imuinput8 & 0xFFFF0000)>>16; //imuinput0
			   *pTmpData++ = sAIInputs.Imuinput9 & 0x0000FFFF;
			   *pTmpData++ = (sAIInputs.Imuinput9 & 0xFFFF0000)>>16; //imuinput0
				 *pTmpData++ = sAIInputs.Imuinput10 & 0x0000FFFF;
			   *pTmpData++ = (sAIInputs.Imuinput10 & 0xFFFF0000)>>16; //imuinput0
				 *pTmpData++ = sAIInputs.Imuinput11 & 0x0000FFFF;
			   *pTmpData++ = (sAIInputs.Imuinput11 & 0xFFFF0000)>>16; //imuinput0
				 *pTmpData++ = sAIInputs.Imuinput12 & 0x0000FFFF;
			   *pTmpData++ = (sAIInputs.Imuinput12 & 0xFFFF0000)>>16; //imuinput0
				 *pTmpData++ = sAIInputs.Imuinput13 & 0x0000FFFF;
			   *pTmpData++ = (sAIInputs.Imuinput13 & 0xFFFF0000)>>16; //imuinput0
				 *pTmpData++ = sAIInputs.Imuinput14 & 0x0000FFFF;
			   *pTmpData++ = (sAIInputs.Imuinput14 & 0xFFFF0000)>>16; //imuinput14
         break;
      }
   }
}

#if _STM32_IO4  && AL_EVENT_ENABLED
/* the pragma interrupt_level is used to tell the compiler that these functions will not
   be called at the same time from the main function and the interrupt routine */
#pragma interrupt_level 1
#endif
void APPL_OutputMapping(UINT16* pData)
{
    UINT16 j = 0;
    UINT16 *pTmpData = (UINT16 *)pData;

    /* we go through all entries of the RxPDO Assign object to get the assigned RxPDOs */
    for (j = 0; j < sRxPDOassign.u16SubIndex0; j++)  //描述个数和类型tobj，0x01,0x1601,已定义，固定
    {
        switch (sRxPDOassign.aEntries[j])
        {
        /* RxPDO 2 */
        case 0x1601:
            ((UINT16 *) &sDOOutputs)[1] = SWAPWORD(*pTmpData++);  //此处只用了第一个字节，sDOOutputs为用户tobj
            break;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
/**
\brief  用户变量更新，应用层输入输出是针对主站的
*////////////////////////////////////////////////////////////////////////////////////////
void APPL_Application(void)
{


    sDIInputs.bSwitch1    = 1;  //上传开关量
    sDIInputs.bSwitch2    = 0;
    sDIInputs.bSwitch3    = 0;
    sDIInputs.bSwitch4    = 0;
#if _STM32_IO8
    sDIInputs.bSwitch5    = 0;
    sDIInputs.bSwitch6    = 0;
    sDIInputs.bSwitch7    = 0;
    sDIInputs.bSwitch8    = 1;
#endif
    sAIInputs.i16Analoginput0 =  uhADCxConvertedValue[0];
		sAIInputs.i16Analoginput  =  uhADCxConvertedValue[1]; //上传模拟量
    
    sAIInputs.Imuinput0 =readdata[0];
		sAIInputs.Imuinput1 =readdata[1];
		sAIInputs.Imuinput2 =readdata[2];
		sAIInputs.Imuinput3 =readdata[3];
		sAIInputs.Imuinput4 =readdata[4];
		sAIInputs.Imuinput5 =readdata[5];
		sAIInputs.Imuinput6 =readdata[6];
		sAIInputs.Imuinput7 =readdata[7];
		sAIInputs.Imuinput8 =readdata[8];
		sAIInputs.Imuinput9 =readdata[9];
		sAIInputs.Imuinput10 =0x00000001;
		sAIInputs.Imuinput11 =0x00000002;
		sAIInputs.Imuinput12 =0x00000003;
		sAIInputs.Imuinput13 =0x00000004;
		sAIInputs.Imuinput14 =readdata[10]; //err

}

UINT8 ReadObject0x1802( UINT16 index, UINT8 subindex, UINT32 dataSize, UINT16 MBXMEM * pData, UINT8 bCompleteAccess )
{

    if(bCompleteAccess)
        return ABORTIDX_UNSUPPORTED_ACCESS;

    if(subindex == 0)
    {
        *pData = TxPDO1802Subindex0;
    }
    else if(subindex == 6)
    {
        /*clear destination buffer (no excluded TxPDO set)*/
        if(dataSize > 0)
            MBXMEMSET(pData,0x00,dataSize);
    }
    else if(subindex == 7)
    {
        /*min size is one Byte*/
        UINT8 *pu8Data = (UINT8*)pData;
        
        //Reset Buffer
        *pu8Data = 0; 

        //*pu8Data = sAIInputs.bTxPDOState;
    }
    else if(subindex == 9)
    {
        /*min size is one Byte*/
        UINT8 *pu8Data = (UINT8*)pData;
        
        //Reset Buffer
        *pu8Data = 0; 

        //*pu8Data = sAIInputs.bTxPDOToggle;
    }
    else
        return ABORTIDX_SUBINDEX_NOT_EXISTING;

    return 0;
}

/*
 
//任务优先级
#define START_TASK_PRIO		1
//任务堆栈大小	
#define START_STK_SIZE 		128  
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

//任务优先级
#define LED0_TASK_PRIO		2
//任务堆栈大小	
#define LED0_STK_SIZE 		50  
//任务句柄
TaskHandle_t LED0Task_Handler;
//任务函数
void led0_task(void *pvParameters);

//任务优先级
#define LED1_TASK_PRIO		3
//任务堆栈大小	
#define LED1_STK_SIZE 		50  
//任务句柄
TaskHandle_t LED1Task_Handler;
//任务函数
void led1_task(void *pvParameters);

//任务优先级
#define FLOAT_TASK_PRIO		4
//任务堆栈大小	
#define FLOAT_STK_SIZE 		128
//任务句柄
TaskHandle_t FLOATTask_Handler;
//任务函数
void float_task(void *pvParameters);
//main函数
int main(void)
{
    HW_Init();//stm32初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	  delay_init(168);		//初始化延时函数
	  //LED_Init();		        //初始化LED端口
	
    MainInit();//ECAT初始化

	//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
}
 
//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
    //创建LED0任务
    xTaskCreate((TaskFunction_t )led0_task,     	
                (const char*    )"led0_task",   	
                (uint16_t       )LED0_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )LED0_TASK_PRIO,	
                (TaskHandle_t*  )&LED0Task_Handler);   
    //创建LED1任务
    xTaskCreate((TaskFunction_t )led1_task,     
                (const char*    )"led1_task",   
                (uint16_t       )LED1_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )LED1_TASK_PRIO,
                (TaskHandle_t*  )&LED1Task_Handler);        
    //浮点测试任务
    xTaskCreate((TaskFunction_t )float_task,     
                (const char*    )"float_task",   
                (uint16_t       )FLOAT_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )FLOAT_TASK_PRIO,
                (TaskHandle_t*  )&FLOATTask_Handler);  
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

//LED0任务函数 
void led0_task(void *pvParameters)
{
    while(1)
    {
        LED0=~LED0;
        vTaskDelay(500);
			  MainLoop();//主循环程序，查询模式未用
    }
}   

//LED1任务函数
void led1_task(void *pvParameters)
{
    while(1)
    {
        LED1=0;
        vTaskDelay(200);
        LED1=1;
        vTaskDelay(800);
    }
}

//浮点测试任务
void float_task(void *pvParameters)
{
	static float float_num=0.00;
	while(1)
	{
		float_num+=0.01f;
		printf("float_num的值为: %.4f\r\n",float_num);
        vTaskDelay(1000);
	}
}
*/
/** @} */
