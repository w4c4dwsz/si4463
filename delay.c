/*
 * delay.c
 *
 *  Created on: 2019年9月22日
 *      Author: NEUQ
 */
#include "xparameters.h"
#include "xgpio.h"
#include "xscutimer.h"
#include "delay.h"

int delay_init(void)
{
	int Status;
		XScuTimer *TimerInstancePtr = &Timer;
		XScuTimer_Config *ConfigPtr;
		//初始化计数器
		ConfigPtr = XScuTimer_LookupConfig(TIMER_DEVICE_ID);
		Status = XScuTimer_CfgInitialize(TimerInstancePtr, ConfigPtr,
						 ConfigPtr->BaseAddr);
		if (Status != XST_SUCCESS) {
				return Status;
			}
		return Status;
}
int s_delay(volatile int count_number)
{
	int time_load_value;
	XScuTimer *TimerInstancePtr = &Timer;
	volatile u32 count;
	delay_init();
	time_load_value = (XPAR_PS7_CORTEXA9_0_CPU_CLK_FREQ_HZ / 2)*count_number;
	//装载计数器寄存器

	XScuTimer_LoadTimer(TimerInstancePtr,time_load_value);

	//开始计时
	XScuTimer_Start(TimerInstancePtr);

	while(1)
	{
		count = XScuTimer_GetCounterValue(TimerInstancePtr);
		if (count == 0)
			break;
	}
	return 0;
}

int ms_delay(volatile int count_number)
{
	int time_load_value;
	XScuTimer *TimerInstancePtr = &Timer;
	volatile u32 count;
	delay_init();
	time_load_value = (XPAR_PS7_CORTEXA9_0_CPU_CLK_FREQ_HZ / 2000)*count_number;
	//装载计数器寄存器

	XScuTimer_LoadTimer(TimerInstancePtr,time_load_value);

	//开始计时
	XScuTimer_Start(TimerInstancePtr);

	while(1)
	{
		count = XScuTimer_GetCounterValue(TimerInstancePtr);
		if (count == 0)
			break;
	}
	return 0;
}
int us_delay(volatile int count_number)
{
	int time_load_value;
	XScuTimer *TimerInstancePtr = &Timer;
	volatile u32 count;
	delay_init();
	time_load_value = (XPAR_PS7_CORTEXA9_0_CPU_CLK_FREQ_HZ / 2000000)*count_number;
	//装载计数器寄存器

	XScuTimer_LoadTimer(TimerInstancePtr,time_load_value);

	//开始计时
	XScuTimer_Start(TimerInstancePtr);

	while(1)
	{
		count = XScuTimer_GetCounterValue(TimerInstancePtr);
		if (count == 0)
			break;
	}
	return 0;
}

