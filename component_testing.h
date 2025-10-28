/*
 * component_testing.h
 *
 *  Created on: Oct 26, 2025
 *      Author: prime
 */

#ifndef COMPONENT_TESTING_H_
#define COMPONENT_TESTING_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "string.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"

void BrightnessLevel();

//*****************************************************************************
//
// PWM Brightness Level Function
//
//*****************************************************************************

void BrightnessLevel(){

	int brightness = 1;
	int dutyCycle = 50;
	int Load = ( (SysCtlClockGet() / 64) / PWM_FREQUENCY) - 1;

	while(1){

		dutyCycle += brightness;

		if(dutyCycle >= 100){
			dutyCycle = 90;
			brightness = -1;
		}
		else if(dutyCycle <= 0){
			dutyCycle = 10;
			brightness = 1;
		}

		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (Load * dutyCycle) / 100);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (Load * dutyCycle) / 100);
		SysCtlDelay(SysCtlClockGet() / 200);
	}
}



#endif /* COMPONENT_TESTING_H_ */
