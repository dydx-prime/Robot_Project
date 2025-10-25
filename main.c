//*****************************************************************************
//
// uart_echo.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
//
// Copyright (c) 2012-2016 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.3.156 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************
// what
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
#include "driverlib/fpu.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"

// global variables
uint8_t pins[] = {GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_1}; // FP1 = RED, PF2 = BLUE, PF3 = GREEN
int led_count = 1;

// command variables
volatile bool command_received = false;
volatile int command_char_count = 0;
volatile char input_cmd[4];
volatile char prev_input_cmd[4] = "NA_";

// adc
uint32_t ADC0Val[8], Int_status;
volatile uint32_t ADCAvVal;
volatile uint32_t ADCAvVal2;


volatile float volts1, distance1;
volatile float volts2, distance2;

// pwm
#define PWM_FREQUENCY 5000
uint32_t ui32PWMClock;
uint32_t ui32Load;
volatile uint32_t dutyCycle = 50;

// pid
volatile float target_distance = 15.0;  // target distance in cm
volatile float curr_error = 0.0, prev_error = 0.0, sum_error = 0.0;
volatile float Kp = 5, Ki = 20, Kd = 5;
volatile float proportional = 0.0, integral = 0.0, derivative = 0.0, adjustment = 0.0;

volatile int baseDuty = 50;   // base forward speed (%)
volatile float pid_output = 0.0;

#define TIMER_PERIOD_MS 50

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART Echo (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the USB debug virtual serial port on the evaluation board)
//! will be configured in 115,200 baud, 8-n-1 mode.  All characters received on
//! the UART are transmitted back to the UART.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The UART interrupt handler.
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

    // Trigger the right-side distance sensor measurement every 50 ms
    ADCProcessorTrigger(ADC0_BASE, 3);
}


//*****************************************************************************
//
// ADC interrupt handler.
//
//*****************************************************************************

// right sensor
//void ADCSeq3IntHandler(void){
//
//	// clear flag
//	ADCIntClear(ADC0_BASE, 3);
//
//	ADCSequenceDataGet(ADC0_BASE, 3, &ADCAvVal);
//
//	volts1 = ADCAvVal * (3.3/4096.0);
//
//	distance1 = 5.0685 * pow(volts1, 2) - 23.329 * volts1 + 31.152;
//
//
//	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
//
//	if (distance1 < 8) {
//		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
//	}
//
//	else if (distance1 < 10) {
//		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3, GPIO_PIN_1 | GPIO_PIN_3);
//	}
//
//	else {
//		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
//	}
//
//	// re-enable ADC interrupt for next conversion
//	ADCIntEnable(ADC0_BASE, 3);
//
//}

void ADCSeq3IntHandler(void)
{
    // Clear flag
    ADCIntClear(ADC0_BASE, 3);
    ADCSequenceDataGet(ADC0_BASE, 3, &ADCAvVal);

    volts1 = ADCAvVal * (3.3 / 4096.0);
    volts2 = volts1 * volts1;
    distance1 = 5.0685 * volts2 - 23.329 * volts1 + 31.152;

    // --- LED Logic ---
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
    //float diff = fabs(target_distance - distance1);
    if (distance1 >= 15)
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);   // GREEN
    else if (distance1 < 15 && distance1 > 5)
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);   // BLUE/YELLOW-ish
    else
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);   // RED

    // --- PID Controller ---
    /*
    error = target_distance - distance1;
    integral += error * 0.05;  // 50 ms period = 0.05 s
    float derivative = (error - prev_error) / 0.05;
    pid_output = Kp * error + Ki * integral + Kd * derivative;
    prev_error = error;
    */
    prev_error = curr_error;
    curr_error = target_distance - distance1;
    sum_error += curr_error;

    proportional = Kp * curr_error;
    derivative = Kd * (curr_error - prev_error);
    integral = Ki * sum_error;



    adjustment = proportional + integral + derivative;

    // --- Adjust Motor Duty Cycles ---
    int rightDuty = baseDuty + adjustment;
    int leftDuty  = baseDuty - adjustment;

    // Clamp duty cycles
    if (rightDuty > 90) rightDuty = 90;
    if (rightDuty < 10)  rightDuty = 40;
    if (leftDuty  > 90) leftDuty = 90;
    if (leftDuty  < 10)  leftDuty = 40;

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * rightDuty) / 100);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * leftDuty) / 100);

    ADCIntEnable(ADC0_BASE, 3); // re-enable ADC interrupt
}

// front sensor
void ADCSeq2IntHandler(void){

	// clear flag
	ADCIntClear(ADC0_BASE, 2);

	ADCSequenceDataGet(ADC0_BASE, 2, &ADCAvVal2);

	volts2 = ADCAvVal2 * (3.3/4096.0);

	distance2 = 5.0685 * pow(volts2, 2) - 23.329 * volts2 + 31.152;

//	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);


	// re-enable ADC interrupt for next conversion
	ADCIntEnable(ADC0_BASE, 2);

}

//*****************************************************************************
//
// ADC initialization function
//
//*****************************************************************************


void ADC_init(void){

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

	//-----Sensor1----- // Right Sensor
	// PE2 as an analog input pin
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
	ADCSequenceDisable(ADC0_BASE, 3);

	// ADC0 sequence 3 triggered by processor
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH1);
	ADCSequenceEnable(ADC0_BASE, 3);
	// register interrupt handler for ADC0 sequence 3
	ADCIntRegister(ADC0_BASE, 3, &ADCSeq3IntHandler);
	ADCIntEnable(ADC0_BASE, 3);


	//-----Sensor2----- // Front Sensor
	// PE1 as an analog input pin
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);
	ADCSequenceDisable(ADC0_BASE, 2);

	// ADC0 sequence 2 triggered by processor
	ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH2);
	ADCSequenceEnable(ADC0_BASE, 2);
	// register interrupt handler for ADC0 sequence 2
	ADCIntRegister(ADC0_BASE, 2, &ADCSeq2IntHandler);
	ADCIntEnable(ADC0_BASE, 2);


}


//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count != 0)
    {
    	ui32Count--;
        //
        // Write the next character to the UART.
        //
    	//ROM_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
        ROM_UARTCharPut(UART0_BASE, *pui8Buffer++);
    }
}

void
UARTSend1(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count != 0)
    {
    	ui32Count--;
        //
        // Write the next character to the UART.
        //
    	//ROM_UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
        ROM_UARTCharPut(UART1_BASE, *pui8Buffer++);
    }
}


//*****************************************************************************
//
// LookUp Table & CMD Logic
//
//*****************************************************************************
void forward(void);
void scan(void);
void stop(void);
void left(void);
void right(void);
void backward(void);
void clear(void);
void BrightnessLevel(void);
void adc_func(void);
void start(void);

typedef void (*Fn)(void);
typedef struct {
	char cmd[4]; // command + '/0'
	Fn cmd_funct; // pointer to command function
}lookup;

lookup lookup_table[] = {
	{"STR", start},
    {"FWD", forward},
    {"SCN", scan},
    {"STP", stop},
    {"LFT", left},
    {"RGT", right},
    {"BAC", backward},
	{"CLC", clear},
	{"LED", BrightnessLevel},
    {"ADC", adc_func}
};

int table_entries = sizeof(lookup_table) / sizeof(lookup_table[0]);

// function calls based on lookup_table
void forward() {
	char terminal_string[] = "\n\rMoving Forward!\r\n";
    UARTSend((uint8_t *) terminal_string, strlen(terminal_string));
}

void scan() {
	char terminal_string[] = "\n\rScanning!\r\n";
    UARTSend((uint8_t *) terminal_string, strlen(terminal_string));
}

void stop() {
	char terminal_string[] = "\n\rStopping!\r\n";
    UARTSend((uint8_t *) terminal_string, strlen(terminal_string));
}

void left() {
	char terminal_string[] = "\n\rMoving Left!\r\n";
    UARTSend((uint8_t *) terminal_string, strlen(terminal_string));
}

void right() {
	char terminal_string[] = "\n\rMoving Right!\r\n";
    UARTSend((uint8_t *) terminal_string, strlen(terminal_string));
}

void backward() {
	char terminal_string[] = "\n\rMoving Backwards!\r\n";
    UARTSend((uint8_t *) terminal_string, strlen(terminal_string));
}

void clear(){
    char terminal_string[] = "\r\033[2J";
    UARTSend((uint8_t *) terminal_string, strlen(terminal_string));
}

void adc_func(){
	ADCProcessorTrigger(ADC0_BASE, 3); // trigger ADC conversion
//	ADCProcessorTrigger(ADC0_BASE, 2);
	SysCtlDelay(SysCtlClockGet() / 100); // small delay to prevent flooding
}

void Timer0A_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));

    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    uint32_t ui32Period = (SysCtlClockGet() / 1000) * TIMER_PERIOD_MS;
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);

    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0AIntHandler);
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

void start(){
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);
    Timer0A_init();
    TimerEnable(TIMER0_BASE, TIMER_A);
}

void execute_command(char *cmd){
	if(strcmp(cmd, prev_input_cmd) == 0){
		char terminal_string[] = "\n\rCommand cannot be used again!\r\n";
		UARTSend((uint8_t *) terminal_string, strlen(terminal_string));
		return;
	}

	int i; // like, whyy?
	for(i = 0; i < table_entries; i++){
        if (strcmp(cmd, lookup_table[i].cmd) == 0) {
            lookup_table[i].cmd_funct(); // run the matched function

            for(i = 0; i < 4; i++){
            	prev_input_cmd[i] = cmd[i];
            }
            return;
        }
	}
	char terminal_string[] = "\n\rCommand does not exist!\r\n";
	UARTSend((uint8_t *) terminal_string, strlen(terminal_string));

}

//*****************************************************************************
//
// PWM_init
//
//*****************************************************************************

void PWM_init(void){


	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

	// pins for wheels
	GPIOPinConfigure(GPIO_PB6_M0PWM0); // right wheel
	GPIOPinConfigure(GPIO_PB4_M0PWM2); // left wheel

	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);

	ui32PWMClock = SysCtlClockGet() / 64;
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

	// right wheel
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32Load);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * 10) / 100);

	// left wheel
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ui32Load);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * 10) / 100);

	// right wheel
	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);

	// left wheel
	PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_1);

}

//*****************************************************************************
//
// PWM Brightness Level Function
//
//*****************************************************************************

void BrightnessLevel(){
//
//	int brightness = 1;
//	dutyCycle = 50;
//
//	while(1){
//
//		dutyCycle += brightness;
//
//		if(dutyCycle >= 100){
//			dutyCycle = 90;
//			brightness = -1;
//		}
//		else if(dutyCycle <= 0){
//			dutyCycle = 10;
//			brightness = 1;
//		}
//
//		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * dutyCycle) / 100);
//		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * dutyCycle) / 100);
//		SysCtlDelay(SysCtlClockGet() / 200);
//	}
}


int main(void){
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPUEnable();
    ROM_FPULazyStackingEnable();
    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);
    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    //
    // Enable the GPIO pins for the LED.
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    // default green LED enable
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
    //
    // Enable UART0 and UART1
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();
    //
    // Set GPIO A0, A1, B0, B1
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    //
    // UART0, 115200, 8-N-1 and UART1, 9600, 8-N-1

    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 9600,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
    //
    // Enable the UART interrupt.
    //
    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    ROM_IntEnable(INT_UART1);
    ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

    // ADC Enable
    ADC_init();

    // PWM Enable
    PWM_init();

//    BrightnessLevel();

//    while(1){
//        // disable PWM outputs while sampling to reduce noise
//
       // ADCProcessorTrigger(ADC0_BASE, 3); // trigger ADC conversion
//        ADCProcessorTrigger(ADC0_BASE, 2);
//
//    }

    // Prompt for text to be entered.
    //
    char terminal_string[] = "\033[2JPlease enter characters in the remote Bluetooth Terminal and see the color change: ";
    UARTSend((uint8_t *) terminal_string, strlen(terminal_string));
    UARTSend1((uint8_t *) terminal_string, strlen(terminal_string));
    //
    // Loop forever echoing data through the UART.
    //
    while(1)
    {

    	if (command_received){
    		execute_command((char *)input_cmd);
    		command_received = false;
    		command_char_count = 0;
    	}
    }
}
