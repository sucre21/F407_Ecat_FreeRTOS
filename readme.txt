实验器材:
	STM32F407VET6开发板
	
实验目的:
	学习FreeRTOS在STM32F407上的移植
	
硬件资源:
	1,DS0(连接在PB12)，DS1(连接在PB13上)
	2,串口2(波特率:115200,PA2/PA3连接在板载USB转串口芯片CH340上面) 
	
实验现象:
	本实验在STM32F407开发板上移植了FreeRTOS，版本为V9.0.0。例程中创建了四个任务来测试移植是否成功。
	start_task任务创建其他3个测试任务，led0_task任务中LED0闪烁，led1_task任务中LED1闪烁，	
	float_task测试浮点运算。