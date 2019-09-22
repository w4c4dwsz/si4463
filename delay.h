/*
 * si4463_test.h
 *
 *  Created on: 2019Äê9ÔÂ21ÈÕ
 *      Author: NEUQ
 */
/***************************** Include Files *********************************/

#include "xparameters.h"
#include "xgpio.h"
#include "xscutimer.h"
#define Frequency 	660000000
#define TIMER_DEVICE_ID		XPAR_XSCUTIMER_0_DEVICE_ID

/***************************** define var *********************************/

XScuTimer Timer;

/************************** Function Prototypes ******************************/

int ms_delay(int);
int us_delay(int);
int s_delay (int);
int delay_init(void);
/***************************** Define Function *********************************/





