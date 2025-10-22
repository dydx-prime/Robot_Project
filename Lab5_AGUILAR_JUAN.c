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
uint32_t ADCAvVal;
float volts;
float distance;
volatile bool adc_active = false;

// pwm
#define PWM_FREQUENCY 5000
uint32_t ui32PWMClock;
uint32_t ui32Load;
volatile uint32_t dutyCycle = 50;


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
// ADC interrupt handler.
//
//*****************************************************************************

ADCSeq0IntHandler(void){

	// clear flag
	ADCIntClear(ADC0_BASE, 3);

	ADCSequenceDataGet(ADC0_BASE, 3, &ADCAvVal);

	volts = ADCAvVal * (3.3/4096.0);

	distance = 5.0685 * pow(volts, 2) - 23.329 * volts + 31.152;


	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

	if (distance < 8) {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
	}

	else if (distance < 10) {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3, GPIO_PIN_1 | GPIO_PIN_3);
	}

	else {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
	}

	// re-enable ADC interrupt for next conversion
	ADCIntEnable(ADC0_BASE, 3);

}

//*****************************************************************************
//
// ADC initialization function
//
//*****************************************************************************


void ADC_init(void){

	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)){};

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)){};

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)){};

	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

	// PE2 as an analog input pin
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
	ADCSequenceDisable(ADC0_BASE, 3);

	// ADC0 sequence 3 triggered by processor
	ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);

	ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH1);
	ADCSequenceEnable(ADC0_BASE, 3);
	ADCIntEnable(ADC0_BASE, 3);

	// register interrupt handler for ADC0 sequence 3
	ADCIntRegister(ADC0_BASE, 3, &ADCSeq0IntHandler);


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

typedef void (*Fn)(void);
typedef struct {
	char cmd[4]; // command + '/0'
	Fn cmd_funct; // pointer to command function
}lookup;

lookup lookup_table[] = {
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
	// toggle ADC mode on/off
	adc_active = !adc_active;
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

	GPIOPinConfigure(GPIO_PB6_M0PWM0);

	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);

	ui32PWMClock = SysCtlClockGet() / 64;
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32Load);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * dutyCycle) / 100);

	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);

}

void PWM_init2(void){


	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

	GPIOPinConfigure(GPIO_PB4_M0PWM2);

	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);

	ui32PWMClock = SysCtlClockGet() / 64;
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ui32Load);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * dutyCycle) / 100);

	PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_1);

}

//*****************************************************************************
//
// PWM Brightness Level Function
//
//*****************************************************************************

void BrightnessLevel(){

//	int brightness = 1;
	dutyCycle = 90;

//	while(1){
//
//		dutyCycle += brightness;
//
//		if(dutyCycle >= 100){
//			dutyCycle = 100;
//			brightness = -1;
//		}
//		else if(dutyCycle <= 0){
//			dutyCycle = 1;
//			brightness = 1;
//		}
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (ui32Load * dutyCycle) / 100);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, (ui32Load * dutyCycle) / 100);
		SysCtlDelay(SysCtlClockGet() / 200);
//	}
}


int
 main(void){
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
    //
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

    // PWM Enable
    PWM_init();
    PWM_init2();
    BrightnessLevel();

    // ADC Enable
    ADC_init();
    //

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
    	if (adc_active) {
    		ADCProcessorTrigger(ADC0_BASE, 3); // trigger ADC conversion
    		SysCtlDelay(SysCtlClockGet() / 100); // small delay to prevent flooding
    	}

    	if (command_received){
    		execute_command((char *)input_cmd);
    		command_received = false;
    		command_char_count = 0;
    	}
    }
}
