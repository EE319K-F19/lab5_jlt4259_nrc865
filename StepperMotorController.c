// StepperMotorController.c starter file EE319K Lab 5
// Runs on TM4C123
// Finite state machine to operate a stepper motor.  
// Jonathan Valvano
// September 2, 2019

// Hardware connections (External: two input buttons and four outputs to stepper motor)
//  PA5 is Wash input  (1 means pressed, 0 means not pressed)
//  PA4 is Wiper input  (1 means pressed, 0 means not pressed)
//  PE5 is Water pump output (toggle means washing)
//  PE4-0 are stepper motor outputs 
//  PF1 PF2 or PF3 control the LED on Launchpad used as a heartbeat
//  PB6 is LED output (1 activates external LED on protoboard)

#include "SysTick.h"
#include "TExaS.h"
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"

void EnableInterrupts(void);
// edit the following only if you need to move pins from PA4, PE3-0      
// logic analyzer on the real board
#define PA4       (*((volatile unsigned long *)0x40004040))
#define PE50      (*((volatile unsigned long *)0x400240FC))
	
struct State{
	uint32_t Out;
	uint32_t Time;
	const struct State *Next[4];
};
typedef const struct State STyp;

// Naming convention: "Wipe" states mean the LED is off. "Spray" means the LED is on 
// (note that while spraying the LED still should alternate between on and off)
// The number after wipe/spray indicates the output for the motor.
// The 'C' is explained below.

#define Wipe1C1		&FSM[0]	// There are 4 cycles (C) the wiper goes through
#define Spray1C1	&FSM[1] // Cycle 1: 1,2,4,8,16
#define Wipe2C1		&FSM[2]
#define Spray2C1	&FSM[3]
#define Wipe4C1		&FSM[4]
#define Spray4C1	&FSM[5]
#define Wipe8C1		&FSM[6]
#define Spray8C1	&FSM[7]
#define Wipe16C1	&FSM[8]
#define Spray16C1	&FSM[9]

#define Wipe1C2		&FSM[10] // Cycle 2: 1,2,4,8,16 again
#define Spray1C2	&FSM[11]
#define Wipe2C2		&FSM[12]
#define Spray2C2	&FSM[13]
#define Wipe4C2		&FSM[14]
#define Spray4C2	&FSM[15]
#define Wipe8C2		&FSM[16]
#define Spray8C2	&FSM[17]
#define Wipe16C2	&FSM[18]
#define Spray16C2	&FSM[19]

#define Wipe8C3   &FSM[20] // Cycle 3: 8,4,2,1
#define Spray8C3	&FSM[21]
#define Wipe4C3		&FSM[22]
#define Spray4C3	&FSM[23]
#define Wipe2C3		&FSM[24]
#define Spray2C3	&FSM[25]
#define Wipe1C3		&FSM[26]
#define Spray1C3	&FSM[27]

#define Wipe16C4	&FSM[28] // Cycle 4: 16,8,4,2,1
#define Spray16C4	&FSM[29]
#define Wipe8C4		&FSM[30]
#define Spray8C4	&FSM[31]
#define Wipe4C4		&FSM[32]
#define Spray4C4	&FSM[33]
#define Wipe2C4		&FSM[34]
#define Spray2C4	&FSM[35]
#define Wipe1C4		&FSM[36]
#define Spray1C4	&FSM[37]

STyp FSM[38] = {
	{1, 5, {Wipe1C1, Wipe2C1, Spray2C1, Spray2C1}},
	{1 + 32, 5, {Wipe2C1, Wipe2C1, Wipe2C1, Spray2C1}},
	
	{2, 5, {Wipe4C1, Wipe4C1, Spray4C1, Spray4C1}},
	{2 + 32, 5, {Wipe4C1, Wipe4C1, Wipe4C1, Spray4C1}},
	
	{4, 5, {Wipe8C1, Wipe8C1, Spray8C1, Spray8C1}},
	{4 + 32, 5, {Wipe8C1, Wipe8C1, Wipe8C1, Spray8C1}},
	
	{8, 5, {Wipe16C1, Wipe16C1, Spray16C1, Spray16C1}},
	{8 + 32, 5, {Wipe16C1, Wipe16C1, Wipe16C1, Spray16C1}},
	
	{16, 5, {Wipe1C2, Wipe1C2, Spray1C2, Spray1C2}},
	{16 + 32, 5, {Wipe1C2, Wipe1C2, Wipe1C2, Spray1C2}},
	
	{1, 5, {Wipe2C2, Wipe2C2, Spray2C2, Spray2C2}},
	{1 + 32, 5, {Wipe2C2, Wipe2C2, Wipe2C2, Spray2C2}},
	
	{2, 5, {Wipe4C2, Wipe4C2, Spray4C2, Spray4C2}},
	{2 + 32, 5, {Wipe4C2, Wipe4C2, Wipe4C2, Spray4C2}},
	
	{4, 5, {Wipe8C2, Wipe8C2, Spray8C2, Spray8C2}},
	{4 + 32, 5, {Wipe8C2, Wipe8C2, Wipe8C2, Spray8C2}},
	
	{8, 5, {Wipe16C2, Wipe16C2, Spray16C2, Spray16C2}},
	{8 + 32, 5, {Wipe16C2, Wipe16C2, Wipe16C2, Spray16C2}},
	
	{16, 5, {Wipe8C3, Wipe8C3, Spray8C3, Spray8C3}},
	{16 + 32, 5, {Wipe8C3, Wipe8C3, Wipe8C3, Spray8C3}},
	
	{8, 5, {Wipe4C3, Wipe4C3, Spray4C3, Spray4C3}},
	{8 + 32, 5, {Wipe4C3, Wipe4C3, Wipe4C3, Spray4C3}},
	
	{4, 5, {Wipe2C3, Wipe2C3, Spray2C3, Spray2C3}},
	{4 + 32, 5, {Wipe2C3, Wipe2C3, Wipe2C3, Spray2C3}},
	
	{2, 5, {Wipe1C3, Wipe1C3, Spray1C3, Spray1C3}},
	{2 + 32, 5, {Wipe1C3, Wipe1C3, Wipe1C3, Spray1C3}},
	
	{1, 5, {Wipe16C4, Wipe16C4, Spray16C4, Spray16C4}},
	{1 + 32, 5, {Wipe16C4, Wipe16C4, Wipe16C4, Spray16C4}},
	
	{16, 5, {Wipe8C4, Wipe8C4, Spray8C4, Spray8C4}},
	{16 + 32, 5, {Wipe8C4, Wipe8C4, Wipe8C4, Spray8C4}},
	
	{8, 5, {Wipe4C4, Wipe4C4, Spray4C4, Spray4C4}},
	{8 + 32, 5, {Wipe4C4, Wipe4C4, Wipe4C4, Spray4C4}},
	
	{4, 5, {Wipe2C4, Wipe2C4, Spray2C4, Spray2C4}},
	{4 + 32, 5, {Wipe2C4, Wipe2C4, Wipe2C4, Spray2C4}},
	
	{2, 5, {Wipe1C4, Wipe1C4, Spray1C4, Spray1C4}},
	{2 + 32, 5, {Wipe1C4, Wipe1C4, Wipe1C4, Spray1C4}},
	
	{1, 5, {Wipe1C1, Wipe1C1, Spray1C1, Spray1C1}},
	{1 + 32, 5, {Wipe1C1, Wipe1C1, Wipe1C1, Spray1C1}}
};


const struct State *Pt;
uint32_t input;

void SendDataToLogicAnalyzer(void){
  UART0_DR_R = 0x80|(PA4<<2)|PE50;
}

int main(void){ 
  TExaS_Init(&SendDataToLogicAnalyzer);    // activate logic analyzer and set system clock to 80 MHz
  SysTick_Init();   
// you initialize your system here
//  PA5 is Wash input  (1 means pressed, 0 means not pressed)
//  PA4 is Wiper input  (1 means pressed, 0 means not pressed)
//  PE5 is Water pump output (toggle means washing)
//  PE4-0 are stepper motor outputs 
//  PF1 PF2 or PF3 control the LED on Launchpad used as a heartbeat
//  PB6 is LED output (1 activates external LED on protoboard)
	
	SYSCTL_RCGCGPIO_R |= 0x31;
	// in line assembly - NOP twice
	__nop();
	__nop();
	
	// set PA4,PA5 as inputs
	// PA4: Wiper
	// PA5: Spray
	GPIO_PORTA_DIR_R &= ~0x30;
	GPIO_PORTA_DEN_R |= 0x30;
	
	// set PE5 as water pump LED output
	// set PE4-0 as outputs (stepper motor)
	GPIO_PORTE_DIR_R |= 0x3F;
	GPIO_PORTE_DEN_R |= 0x3F;

  EnableInterrupts();

	// set the state pointer to the starting state
	Pt = Wipe1C1;
	
  while(1){
// output
		PE50 = Pt->Out;
// wait
		SysTick_Wait10ms(Pt->Time);
// input
		input = (GPIO_PORTA_DATA_R >> 4);
// next		
		Pt = Pt->Next[input];
  }
}
