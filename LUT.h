/*
 * LUT.h
 *
 *  Created on: Oct 26, 2025
 *      Author: prime
 */

#ifndef LUT_H_
#define LUT_H_


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

// command variables
volatile bool command_received = false;
volatile int command_char_count = 0;
char input_cmd[4];
char prev_input_cmd[4] = "NA_";

// command functions
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

// other functions in main.c (gets rid of warnings)
void Timer0A_init(void);
void UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count);

// typedef struct
typedef void (*Fn)(void);
typedef struct {
	char cmd[4]; // command + '/0'
	Fn cmd_funct; // pointer to command function
}lookup;

// lookup table
lookup lookup_table[] = {
	{"STR", start},
    {"FWD", forward},
    {"SCN", scan},
    {"STP", stop},
    {"LFT", left},
    {"RGT", right},
    {"BAC", backward},
	{"CLC", clear},
    {"ADC", adc_func}
};

// number of entries in the table calculation
int table_entries = sizeof(lookup_table) / sizeof(lookup_table[0]);

// LUT command functions
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



#endif /* LUT_H_ */
