//filename: Switch2.c
//Bryce Cotner, Gorkem Caylak
//February 11, 2018
//Implementaions for Swich debouncing and One shot timing
//Lab 3
//Saadallah Kassir
//February 6, 2018
//Hardware
//PF0,PF2,PF4,PF4 GPIO In

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
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

// PF4 connected to a negative logic switch using internal pull-up (trigger on both edges)
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "SysTick.h"
#define PF0       (*((volatile uint32_t *)0x40025004))
#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
#define PF4       (*((volatile uint32_t *)0x40025040))
#define PB6       (*((volatile uint32_t *)0x40005100))	
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

volatile static unsigned long pf0_touched;  
volatile static unsigned long pf2_touched;
volatile static unsigned long pf3_touched;// true on touch
volatile static unsigned long pf4_touched;   // true on release
volatile static unsigned long Last0;      // previous
volatile static unsigned long Last2;      // previous
volatile static unsigned long Last3;      // previous
volatile static unsigned long Last4;      // previous

void (*PF0Task)(void);    // user function to be executed on touch
void (*PF2Task)(void);  // user function to be executed on release
void (*PF3Task)(void);  // user function to be executed on release
void (*PF4Task)(void);  // user function to be executed on release

static void Timer1Arm(void){
  TIMER1_CTL_R = 0x00000000;    // 1) disable TIMER0A during setup
  TIMER1_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER1_TAMR_R = 0x0000001;    // 3) 1-SHOT mode
  TIMER1_TAILR_R = 160000;      // 4) 10ms reload value
  TIMER1_TAPR_R = 0;            // 5) bus clock resolution
  TIMER1_ICR_R = 0x00000001;    // 6) clear TIMER0A timeout flag
  TIMER1_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFF00FF)|0x00008000; // 8) priority 4
// interrupts enabled in the main program after all devices initialized
// vector number 35, interrupt number 19
  NVIC_EN0_R = 1<<21;           // 9) enable IRQ 19 in NVIC
  TIMER1_CTL_R = 0x00000001;    // 10) enable TIMER0A
}
static void GPIOArm(void){
  GPIO_PORTF_ICR_R = 0x1D;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x1D;      // (f) arm interrupt on PF4 *** No IME bit as mentioned in Book ***
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC  
	
}
// Initialize switch interface on PF4 
// Inputs:  pointer to a function to call on touch (falling edge),
//          pointer to a function to call on release (rising edge)
// Outputs: none 
void Switch_Init(void(*pf0task)(void), void(*pf2task)(void),void(*pf3task)(void),void(*pf4task)(void)){
  // **** general initialization ****
  SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
  while((SYSCTL_PRGPIO_R & 0x00000020) == 0){};
	GPIO_PORTF_LOCK_R =0x4c4f434b;
	GPIO_PORTF_CR_R = 0x1F;
		
  GPIO_PORTF_DIR_R = 0x02;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x1D;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x1F;     //     enable digital I/O on PF4   
  GPIO_PORTF_PCTL_R &= ~0x000FFF0F; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF change for speaker?
  GPIO_PORTF_PUR_R |= 0x1D;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x1D;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x1D;	//     PF4 is both edges
	GPIO_PORTF_IEV_R |= 0x1D; 
	GPIOArm();

  SYSCTL_RCGCTIMER_R |= 0x02;   // 0) activate TIMER0
  PF0Task = pf0task;
	PF2Task = pf2task;
	PF3Task = pf3task;
	PF4Task = pf4task;	     // user function 
  pf0_touched = 0;                       // allow time to finish activating
  pf2_touched = 0;
	pf3_touched = 0;
	pf4_touched = 0;
  Last0 = PF0;  
	Last2 = PF2;   
	Last3 = PF3;   
	Last4 = PF4;   // initial switch state
 }
// Interrupt on rising or falling edge of PF4 (CCP0)
void GPIOPortF_Handler(void){
   GPIO_PORTF_IM_R &= ~0x1D;    // disarm interrupt on  
  if(GPIO_PORTF_RIS_R&0X01){   
		GPIO_PORTF_IM_R &= ~0x01;
		if(Last0){
		pf0_touched=1;
		(*PF0Task)();
	 }
	 
 }
  else if(GPIO_PORTF_RIS_R&0X04){    // 0x10 means it was previously released
   GPIO_PORTF_IM_R &= ~0x04;
		if(Last2){
		pf2_touched=1;
		(*PF2Task)();
	 }
		
		
  }
	else if(GPIO_PORTF_RIS_R&0X08){    // 0x10 means it was previously released
   GPIO_PORTF_IM_R &= ~0x08;
		if(Last3){
		pf3_touched=1;
		(*PF3Task)();
		
  }
}
	
	else if(GPIO_PORTF_RIS_R&0X10){    // 0x10 means it was previously released
   GPIO_PORTF_IM_R &= ~0x10;
		if(Last4){
		pf4_touched=1;
		(*PF4Task)();
		
		}
  }
  //SysTick_Wait10ms(3); // start one shot

	Timer1Arm();
  //GPIOArm();  
}
	

// Wait for switch to be pressed 
// There will be minimum time delay from touch to when this function returns
// Inputs:  none
// ts: none 
void Timer1A_Handler(void){
 TIMER1_IMR_R = 0x00000000;
	 Last0=PF0; // switch state
	Last2=PF2;
	Last3=PF3;
	Last4=PF4;
	GPIOArm();  
}

void Switch_WaitPress(void){
  while(pf4_touched==0 ||  pf3_touched==0 ||pf2_touched==0 ||pf0_touched==0){}; // wait for press
		pf4_touched = 0;  // set up for next time
		pf3_touched = 0;
		pf2_touched = 0;
		pf0_touched = 0; 
}

// Wait for switch to be released 
// There will be minimum time delay from release to when this function returns
// Inputs:  none
// Outputs: none 


// Return current value of the switch 
// Repeated calls to this function may bounce during touch and release events
// If you need to wait for the switch, use WaitPress or WaitRelease
// Inputs:  none
// Outputs: false if switch currently pressed, true if released 
/*
unsigned long Switch_Input(void){
  return PF4;
}
*/
