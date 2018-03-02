// TableTrafficLight.c
// Runs on LM4F120 or TM4C123
// Index implementation of a Moore finite state machine to operate
// a traffic light.
// Daniel Valvano, Jonathan Valvano
// July 20, 2013

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers",
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2013
   Volume 1 Program 6.8, Example 6.4
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
   Volume 2 Program 3.1, Example 3.1

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
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

// east facing red light connected to PB5
// east facing yellow light connected to PB4
// east facing green light connected to PB3
// north facing red light connected to PB2
// north facing yellow light connected to PB1
// north facing green light connected to PB0
// north facing car detector connected to PE1 (1=car present)
// east facing car detector connected to PE0 (1=car present)


#include "SysTick.h"

#define LIGHT                   (*((volatile unsigned long *)0x400053FC))
#define GPIO_PORTB_OUT          (*((volatile unsigned long *)0x400050C0))
#define GPIO_PORTB_CR_R         (*((volatile unsigned long *)0x40005524))	// bits 5-0
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
#define GPIO_PORTE_IN           (*((volatile unsigned long *)0x40024008)) // bits 1-0
#define SENSOR                  (*((volatile unsigned long *)0x4002401C))

#define P_LIGHT 								(*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_CR_R         (*((volatile unsigned long *)0x40024524))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOE      0x00000010  // port E Clock Gating Control
#define SYSCTL_RCGC2_GPIOB      0x00000002  // port B Clock Gating Control

#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
	
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_GPIOF       0x00000020

void Init_PortB(void);
void Init_PortE(void);
void Init_PortF(void);

// Linked data structure
struct State {
  unsigned long Out;//trafficlight
	unsigned long P_out;
  unsigned long Time;  
  unsigned long Next[9];}; //not 4 states anymore
typedef const struct State STyp;
#define goN   0
#define waitN 1
#define goE   2
#define waitE 3
#define goP		4
#define Pon1	5
#define Poff1	6
#define Pon2	7
#define Poff2	8
	
STyp FSM[9]=
{
	//9 rows
 {0x21,0x02,600,{goN,waitN, waitN, waitN, goN, waitN, waitN, waitN}}, 
 {0x22,0x02,200,{goE, goP, goE, goP, goE, goP, goE, goP}},
 {0x0C,0x02,600,{goE, waitE, goE, waitE, waitE, waitE, waitE, waitE}},
 {0x14,0x02,200,{goN, goP, goE, goP, goN, goP, goN, goN}},
 {0x24,0x08,600,{goP, goP, Pon1, Pon1, Pon1, Pon1, Pon1, Pon1}},
 {0x24, 0x02, 50, {Poff1, Poff1, Poff1, Poff1, Poff1, Poff1, Poff1, Poff1}},
 {0x24, 0x00, 50, {Pon2, Pon2, Pon2, Pon2, Pon2, Pon2, Pon2, Pon2}},
 {0x24, 0x02, 50, {Poff2, Poff2, Poff2, Poff2, Poff2, Poff2, Poff2, Poff2}},
 {0x24, 0x00, 50, {goN, goE, goE, goE, goN, goN, goE, goN}}
};
unsigned long S;  // index to the current state 
unsigned long Input; 
int main(void){ volatile unsigned long delay;
	Init_PortB();
	Init_PortE(); 
	Init_PortF();// 80 MHz, Program 10.1
  SysTick_Init();   // Program 10.2
	
	 while(1){
    LIGHT = FSM[S].Out;  // set lights
		P_LIGHT = FSM[S].P_out;
    SysTick_Wait10ms(FSM[S].Time);
    Input = SENSOR;     // read sensors
    S = FSM[S].Next[Input];
}
	 }
  void Init_PortB(void)
{
  volatile	unsigned int delay;
	SYSCTL_RCGC2_R |= 0x00000002;     // 1) B clock
  delay = SYSCTL_RCGC2_R; 
	GPIO_PORTB_CR_R |= 0x3F;// delay      
  GPIO_PORTB_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTB_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTB_DIR_R = 0x3F;          // 5) PF4,PF0 input, PB3,PB2,PB1 output   
  GPIO_PORTB_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTB_DEN_R = 0x3F;          // 7) enable digital pins PF4-PF0        
}


void Init_PortE(void)
{
	volatile unsigned int delay;
	SYSCTL_RCGC2_R |= 0x00000010;     // 1) B clock
  delay = SYSCTL_RCGC2_R; 
	GPIO_PORTE_CR_R |= 0x07;	// delay         
  GPIO_PORTE_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTE_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTE_DIR_R = 0x00;          // 5) PB4,PB0 input, PB3,PB2,PB1 output   
  GPIO_PORTE_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTE_DEN_R = 0x07;          // 7) enable digital pins PB4-PB0
}	

void Init_PortF(void)
{	
	volatile unsigned int delay;
	SYSCTL_RCGC2_R |= 0x00000020;     // 1) B clock
  delay = SYSCTL_RCGC2_R; 
	GPIO_PORTF_AMSEL_R &= ~0x0A; // 3) disable analog function on PB5-0
  GPIO_PORTF_PCTL_R &= ~0x000000FF; // 4) enable regular GPIO
  GPIO_PORTF_DIR_R |= 0x0A;    // 5) outputs on PB5-0
  GPIO_PORTF_AFSEL_R &= ~0x0A; // 6) regular function on PB5-0
  GPIO_PORTF_DEN_R |= 0x0A;    // 7) enable digital on PB5-0
  S = goN; 
}
//engine(plz no change)	
 


