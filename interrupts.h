/*
 * interrupts.h
 *
 *  Created on: Oct 25, 2025
 *      Author: prime
 */

#ifndef INTERRUPTS_H_
#define INTERRUPTS_H_

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


// LED Switching on Terminal Inputs
uint8_t pins[] = {GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1}; // FP1 = RED, PF2 = BLUE, PF3 = GREEN
int led_count = 1;

// adc
uint32_t ADC0Val[8], Int_status;
volatile uint32_t ADCAvVal;
volatile uint32_t ADCAvVal2;
bool front_block_flag = false;
bool uTurnStatus = false, right_turn_status;

volatile float volts1, right_distance; // right sensor
volatile float volts2, front_distance; // front sensor

// pwm
#define PWM_FREQUENCY 5000
uint32_t ui32PWMClock;
uint32_t ui32Load;
volatile int baseDuty = 80;   // duty cycle speed %

// pid
volatile float target_distance = 8.5, right_turn_distance = 15.0;  // target distance in cm
volatile int right_turn_count = 0.0, uturn_count = 0.0;
volatile float curr_error = 0.0, prev_error = 0.0, integral_error = 0.0, sum_error = 0.0;
volatile float Kp = 0.8, Ki = 0.02, Kd = 0.08;
volatile float proportional = 0.0, integral = 0.0, derivative = 0.0, adjustment = 0.0;


//*****************************************************************************
//
// UART interrupt handler.
//
//*****************************************************************************
void
UART0IntHandler(void)
{
    uint32_t ui32Status;

    // get and clear interrupt status
    ui32Status = ROM_UARTIntStatus(UART0_BASE, true);
    ROM_UARTIntClear(UART0_BASE, ui32Status);

    while (ROM_UARTCharsAvail(UART0_BASE))
    {
    	char c = ROM_UARTCharGetNonBlocking(UART0_BASE);

        // send to UART1
        ROM_UARTCharPutNonBlocking(UART1_BASE, c);

        // echo to UART0
        ROM_UARTCharPutNonBlocking(UART0_BASE, c);
    }
}


void
UART1IntHandler(void)
{
    uint32_t ui32Status;

    // get and clear interrupt status
    ui32Status = ROM_UARTIntStatus(UART1_BASE, true);
    ROM_UARTIntClear(UART1_BASE, ui32Status);

    while(ROM_UARTCharsAvail(UART1_BASE))
    {

        char c = ROM_UARTCharGetNonBlocking(UART1_BASE);

        // send to UART0
        ROM_UARTCharPutNonBlocking(UART0_BASE, c);

        // echo to UART1
        ROM_UARTCharPutNonBlocking(UART1_BASE, c);

        //turn on current LED, turn off the others
        GPIOPinWrite(GPIO_PORTF_BASE, pins[led_count], pins[led_count]);
        GPIOPinWrite(GPIO_PORTF_BASE, pins[(led_count+1)%3], 0);
        GPIOPinWrite(GPIO_PORTF_BASE, pins[(led_count+2)%3], 0);

        // Advance the counter
        led_count = (led_count + 1) % 3;


        input_cmd[command_char_count] = c;

        command_char_count++;

        if (command_char_count == 3){
        	input_cmd[3] = '\0';
        	command_received = true;
        }
    }
}

//*****************************************************************************
//
// functions used by Interrupts
//
//*****************************************************************************

void stop_checker(void){ // a quick checker to see if the front sensor actually works

	 if (front_block_flag){
		 int rightDuty = 10;
	 	 int leftDuty = 10;
	 	 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * rightDuty) / 100);
	 	 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * leftDuty) / 100);
	 }

}

void pid_func(void){

	// enable green LED
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_3);

    // --- PID ---
    prev_error = curr_error;
    curr_error = target_distance - right_distance;

    // clamp integral error
    integral_error += curr_error * .05;
    if(integral_error > 100) integral_error = 100;
    if(integral_error < -100) integral_error = -100;

    proportional = Kp * curr_error;
    derivative = Kd * (curr_error - prev_error) / .05; // d/dt
    integral = Ki * integral_error;

    adjustment = proportional + integral + derivative;

    // --- Motor Duty Cycles ---
    int rightDuty = baseDuty + adjustment;
    int leftDuty  = baseDuty - adjustment;

    // clamps

    if (rightDuty > 99) rightDuty = baseDuty;
    if (rightDuty < 10) rightDuty = 10;

    if (leftDuty  > 99) leftDuty = baseDuty;
    if (leftDuty  < 10) leftDuty = 10;


    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * rightDuty) / 100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * leftDuty) / 100);

}

void rightTurn(void){

	// enable BLUE LED
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_2);

    int rightDuty = 30;
    int leftDuty  = 80;

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * rightDuty) / 100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * leftDuty) / 100);

}

void uTurn(void){

	// enable RED LED
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_1);

	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5 , 0); // right motor phase - forward
	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 , GPIO_PIN_4); // left motor phase - backwards

    // experiment with duty cycles - try to get the robot to spin in place
    int rightDuty = 85;
    int leftDuty  = 85;

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * rightDuty) / 100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * leftDuty) / 100);

    //TimerDisable(TIMER0_BASE, TIMER_A);
    //SysCtlDelay(40000000 / 7.0);
    //TimerEnable(TIMER0_BASE, TIMER_A);

}

//*****************************************************************************
//
// Timer0A Interrupt Handler
//
//*****************************************************************************

void Timer0AIntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // front sensor handler call
   	ADCProcessorTrigger(ADC0_BASE, 2);



}

//*****************************************************************************
//
// ADC Interrupt Handler
//
//*****************************************************************************

void ADCSeq3IntHandler(void) // right sensor
{
    // Clear flag
    ADCIntClear(ADC0_BASE, 3);

    // distance calculation
    ADCSequenceDataGet(ADC0_BASE, 3, &ADCAvVal);
    volts1 = ADCAvVal * (3.3 / 4096.0);
    right_distance = 5.0685 * pow(volts1, 2) - 23.329 * volts1 + 31.152;

    if(right_distance > right_turn_distance){
    	right_turn_status = true;
    	rightTurn();
    }
    else{
    	right_turn_status = false;

    	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5 , 0);
		GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 , 0);

		int rightDuty = baseDuty;
		int leftDuty  = baseDuty;

		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * rightDuty) / 100);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * leftDuty) / 100);

        pid_func();
    }

    ADCIntEnable(ADC0_BASE, 3); // re-enable ADC interrupt
}

void ADCSeq2IntHandler(void){ // front sensor

	// clear flag
	ADCIntClear(ADC0_BASE, 2);

	ADCSequenceDataGet(ADC0_BASE, 2, &ADCAvVal2);
	volts2 = ADCAvVal2 * (3.3/4096.0);
	front_distance = 5.0685 * pow(volts2, 2) - 23.329 * volts2 + 31.152;

	if (front_distance <= 8 && !right_turn_status && uturn_count <= 0){
		// front_block_flag = true;
		uturn_count = 2;
		uTurnStatus = true;
		uTurn();
	}
	else if (uTurnStatus && front_distance <= 14.0){
	    uTurn();
	}
    else if (uTurnStatus){

    	uturn_count--;

    	if (uturn_count <= 0)
    	{
    		uTurnStatus = false;

			// set wheels forward again
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5 , 0);
			GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4 , 0);

			int rightDuty = baseDuty;
			int leftDuty  = baseDuty;

			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * rightDuty) / 100);
			PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * leftDuty) / 100);
    	}
    }
    else{
        ADCProcessorTrigger(ADC0_BASE, 3);
    }

	// re-enable ADC interrupt for next conversion
	ADCIntEnable(ADC0_BASE, 2);

}

#endif /* INTERRUPTS_H_ */
