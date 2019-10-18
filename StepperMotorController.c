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

// Wipe for the wipe alone
// Wash will wipe and flash the LED 

#define Wipe1C1		&FSM[0]	
#define Wash1C1		&FSM[1]   // stepper motor outputs 1,2,4,8,16
#define Wipe2C1		&FSM[2]
#define Wash2C1		&FSM[3]
#define Wipe4C1		&FSM[4]
#define Wash4C1		&FSM[5]
#define Wipe8C1		&FSM[6]
#define Wash8C1		&FSM[7]
#define Wipe16C1	&FSM[8]
#define Wash16C1	&FSM[9]
#define Wipe1C2		&FSM[10] // stepper motor outputs 1,2,4,8,16 
#define Wash1C2		&FSM[11]
#define Wipe2C2		&FSM[12]
#define Wash2C2		&FSM[13]
#define Wipe4C2		&FSM[14]
#define Wash4C2		&FSM[15]
#define Wipe8C2		&FSM[16]
#define Wash8C2		&FSM[17]
#define Wipe16C2	&FSM[18]
#define Wash16C2	&FSM[19]
#define Wipe8C3   &FSM[20] // stepper motor outputs 8,4,2,1
#define Wash8C3		&FSM[21]
#define Wipe4C3		&FSM[22]
#define Wash4C3		&FSM[23]
#define Wipe2C3		&FSM[24]
#define Wash2C3		&FSM[25]
#define Wipe1C3		&FSM[26]
#define Wash1C3		&FSM[27]

#define Wipe16C4	&FSM[28] // stepper motor outputs 16,8,4,2,1
#define Wash16C4	&FSM[29]
#define Wipe8C4		&FSM[30]
#define Wash8C4		&FSM[31]
#define Wipe4C4		&FSM[32]
#define Wash4C4		&FSM[33]
#define Wipe2C4		&FSM[34]
#define Wash2C4		&FSM[35]
#define Wipe1C4		&FSM[36]
#define Wash1C4		&FSM[37]

// added 32 to the Wash states in order to toggle the water LED, it hits PE5

STyp FSM[38] = {
	{1, 5, {Wipe1C1, Wipe2C1, Wash2C1, Wash2C1}}, //cycle 1 for wash and wipe
	{1 + 32, 5, {Wipe2C1, Wipe2C1, Wipe2C1, Wash2C1}},
	{2, 5, {Wipe4C1, Wipe4C1, Wash4C1, Wash4C1}},
	{2 + 32, 5, {Wipe4C1, Wipe4C1, Wipe4C1, Wash4C1}},
	{4, 5, {Wipe8C1, Wipe8C1, Wash8C1, Wash8C1}},
	{4 + 32, 5, {Wipe8C1, Wipe8C1, Wipe8C1, Wash8C1}},
	{8, 5, {Wipe16C1, Wipe16C1, Wash16C1, Wash16C1}},
	{8 + 32, 5, {Wipe16C1, Wipe16C1, Wipe16C1, Wash16C1}},
	{16, 5, {Wipe1C2, Wipe1C2, Wash1C2, Wash1C2}},
	{16 + 32, 5, {Wipe1C2, Wipe1C2, Wipe1C2, Wash1C2}},
	{1, 5, {Wipe2C2, Wipe2C2, Wash2C2, Wash2C2}},  //cycle 2 for wash and wipe 
	{1 + 32, 5, {Wipe2C2, Wipe2C2, Wipe2C2, Wash2C2}},
	{2, 5, {Wipe4C2, Wipe4C2, Wash4C2, Wash4C2}},
	{2 + 32, 5, {Wipe4C2, Wipe4C2, Wipe4C2, Wash4C2}},
	{4, 5, {Wipe8C2, Wipe8C2, Wash8C2, Wash8C2}},
	{4 + 32, 5, {Wipe8C2, Wipe8C2, Wipe8C2, Wash8C2}},
	{8, 5, {Wipe16C2, Wipe16C2, Wash16C2, Wash16C2}},
	{8 + 32, 5, {Wipe16C2, Wipe16C2, Wipe16C2, Wash16C2}},
	{16, 5, {Wipe8C3, Wipe8C3, Wash8C3, Wash8C3}},
	{16 + 32, 5, {Wipe8C3, Wipe8C3, Wipe8C3, Wash8C3}},
	{8, 5, {Wipe4C3, Wipe4C3, Wash4C3, Wash4C3}}, //cycle 3 for wash and wipe
	{8 + 32, 5, {Wipe4C3, Wipe4C3, Wipe4C3, Wash4C3}},
	{4, 5, {Wipe2C3, Wipe2C3, Wash2C3, Wash2C3}},
	{4 + 32, 5, {Wipe2C3, Wipe2C3, Wipe2C3, Wash2C3}},
	{2, 5, {Wipe1C3, Wipe1C3, Wash1C3, Wash1C3}},
	{2 + 32, 5, {Wipe1C3, Wipe1C3, Wipe1C3, Wash1C3}},
	{1, 5, {Wipe16C4, Wipe16C4, Wash16C4, Wash16C4}},
	{1 + 32, 5, {Wipe16C4, Wipe16C4, Wipe16C4, Wash16C4}},
	{16, 5, {Wipe8C4, Wipe8C4, Wash8C4, Wash8C4}},
	{16 + 32, 5, {Wipe8C4, Wipe8C4, Wipe8C4, Wash8C4}},
	{8, 5, {Wipe4C4, Wipe4C4, Wash4C4, Wash4C4}}, //cycle 4 for wash and wipe
	{8 + 32, 5, {Wipe4C4, Wipe4C4, Wipe4C4, Wash4C4}},
	{4, 5, {Wipe2C4, Wipe2C4, Wash2C4, Wash2C4}},
	{4 + 32, 5, {Wipe2C4, Wipe2C4, Wipe2C4, Wash2C4}},
	{2, 5, {Wipe1C4, Wipe1C4, Wash1C4, Wash1C4}},
	{2 + 32, 5, {Wipe1C4, Wipe1C4, Wipe1C4, Wash1C4}},
	{1, 5, {Wipe1C1, Wipe1C1, Wash1C1, Wash1C1}},
	{1 + 32, 5, {Wipe1C1, Wipe1C1, Wipe1C1, Wash1C1}}
};


const struct State *Pointer;
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
	
	SYSCTL_RCGCGPIO_R |= 0x11; // clocks for ports A and E
	__nop(); // wait for clock to set up 
	__nop();
	GPIO_PORTA_DIR_R &= ~0x30; // PA4 as wipe, PA5 as wash
	GPIO_PORTA_DEN_R |= 0x30;
	GPIO_PORTE_DIR_R |= 0x3F;	// set PE5 as LED output, PE4-0 is the stepper motor
	GPIO_PORTE_DEN_R |= 0x3F;
  EnableInterrupts();
	Pointer = Wipe1C1; // pointer to initial state
	
  while(1){
		PE50 = Pointer->Out; // output
		SysTick_Wait10ms(Pointer->Time); //wait for 50 ms
		input = (GPIO_PORTA_DATA_R >> 4); //shift PA5 and PA4 into bits 1 and 0, respectively 		
		Pointer = Pointer->Next[input]; // next
  }
}
