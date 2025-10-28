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
volatile bool front_block_flag = false;

volatile float volts1, right_distance; // right sensor
volatile float volts2, front_distance; // front sensor

// pwm
#define PWM_FREQUENCY 5000
uint32_t ui32PWMClock;
uint32_t ui32Load;
volatile int baseDuty = 50;   // duty cycle speed (%)

// pid
volatile float target_distance = 15.0;  // target distance in cm
volatile float curr_error = 0.0, prev_error = 0.0, integral_error = 0.0;
volatile float Kp = 4, Ki = 3, Kd = 2;
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
// Timer0A Interrupt Handler
//
//*****************************************************************************

void Timer0AIntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //ADCProcessorTrigger(ADC0_BASE, 2);

	 if (front_block_flag){
		 int rightDuty = 0;
	 	 int leftDuty = 0;
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * rightDuty) / 100);
		 PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * leftDuty) / 100);
	 }

    // right sensor distance measurement every 50 ms
    ADCProcessorTrigger(ADC0_BASE, 3);
}

//*****************************************************************************
//
// ADC Interrupt Handler
//
//*****************************************************************************

void ADCSeq3IntHandler(void) // left sensor
{
    // Clear flag
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, &ADCAvVal);

    volts1 = ADCAvVal * (3.3 / 4096.0);
//    volts2 = volts1 * volts1; //
//    right_distance = 5.0685 * volts2 - 23.329 * volts1 + 31.152;
    right_distance = 5.0685 * pow(volts1, 2) - 23.329 * volts1 + 31.152;

    // --- LED Logic ---
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

    float diff = fabs(target_distance - right_distance);

    if (diff <= 1.0f)
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);   // GREEN
    else if (diff < 2.0f)
    	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3, GPIO_PIN_1 | GPIO_PIN_3);   // YELLOW
    else
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);   // RED

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
    if (rightDuty > 90) rightDuty = 90;
    if (rightDuty < 10) rightDuty = 10;

    if (leftDuty  > 90) leftDuty = 90;
    if (leftDuty  < 10) leftDuty = 10;

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * rightDuty) / 100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * leftDuty) / 100);

    ADCIntEnable(ADC0_BASE, 3); // re-enable ADC interrupt
}

void ADCSeq2IntHandler(void){ // front sensor

	// clear flag
	ADCIntClear(ADC0_BASE, 2);
	ADCSequenceDataGet(ADC0_BASE, 2, &ADCAvVal2);

	volts2 = ADCAvVal2 * (3.3/4096.0);
	front_distance = 5.0685 * pow(volts2, 2) - 23.329 * volts2 + 31.152;

	if (front_distance <= 3){
		 front_block_flag = true;
	}
	else{
	  	  front_block_flag = false;
	}

	// re-enable ADC interrupt for next conversion
	ADCIntEnable(ADC0_BASE, 2);

}

#endif /* INTERRUPTS_H_ */
