// SysTick.c
// Runs on TM4C123
// Provide functions that initialize the SysTick module
// Put your name here

// October 5, 2018

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2018
   Program 2.11, Section 2.6

 Copyright 2018 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
#include <stdint.h>

// Initialize SysTick with busy wait running at bus clock.
#define NVIC_ST_CTRL_R      (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R    (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R   (*((volatile unsigned long *)0xE000E018))
void SysTick_Init(void){
	// write this
	NVIC_ST_CTRL_R = 0;	// Disable
	NVIC_ST_RELOAD_R = 0xFFFFFF;	// Max reload value
	NVIC_ST_CURRENT_R = 0;	// Clear current value
	NVIC_ST_CTRL_R = 0x5;	// Enable and CLK_SRC on, inten off
}
// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(uint32_t delay){
	// write this
	NVIC_ST_RELOAD_R = delay - 1;	// Counts
	NVIC_ST_CURRENT_R = 0; // Clear current value
	while((NVIC_ST_CTRL_R & 0x10000) == 0); // NOP
}
// The delay parameter is in ms.
// 80000 bus cycles equals 1ms
void SysTick_Wait1ms(uint32_t delay){
	// write this
	uint32_t i = 0;
	for(i = 0; i < delay; i++){
		SysTick_Wait(80000);	// Wait 1 ms based on 12.5 ns bus cycle
	}
}

// 800000 bus cycles equals 10ms
void SysTick_Wait10ms(unsigned long delay){
	// write this
	unsigned long i = 0;
	for(i = 0; i < delay; i++){
		SysTick_Wait(800000);	// Wait 10 ms based on 12.5 ns bus cycle
	}
}

