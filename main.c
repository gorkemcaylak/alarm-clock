//filename: main.c
//Bryce Cotner, Gorkem Caylak
//February 11, 2018
//Draws most recent set time to LCD and triggers an alarm when the clock reaches
//a specific set time
//Lab 3
//February 6, 2018
//Hardware Configuration
// Backlight (pin 10) connected to +3.3 V
// MISO (pin 9) unconnected 
// SCK (pin 8) connected to PA2 (SSI0Clk)
// MOSI (pin 7) connected to PA5 (SSI0Tx)
// TFT_CS (pin 6) connected to PA3 (SSI0Fss)
// CARD_CS (pin 5) unconnected
// Data/Command (pin 4) connected to PA6 (GPIO)
// RESET (pin 3) connected to PA7 (GPIO)
// VCC (pin 2) connected to +3.3 V
// Gnd (pin 1) connected to ground
//PF0,PF2,PF4,PF4 GPIO In
//PB7 PWM out
//PE4 ADC in
//PF1 Heartbeat


// center of X-ohm potentiometer connected to PE3/AIN0
// bottom of X-ohm potentiometer connected to ground
// top of X-ohm potentiometer connected to +3.3V 

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "Timer1.h"
#include "ST7735.h"
#include "Timer2.h"
#include "ClockDraw.h"
#include "SysTick.h"
#include "ADCSWTrigger.h"
#include "Switch2.h"
#include "PWM.h"

#define PF0       (*((volatile uint32_t *)0x40025004))
#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
#define PF4       (*((volatile uint32_t *)0x40025040)))

#define SIZE 1000
#define Y_LENGTH 160
#define X_LENGTH 128

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void updateTime(void); //adds one second
void addTime(uint32_t timeToAdd); //set time helper
void ChangeMenuMode(uint8_t mode);
void ChangeClockMode(uint8_t mode);

static uint32_t Time = (9<<24);
static uint32_t TimeSave = 1;
static uint32_t AlarmTime = 0;
static uint32_t ClockMode=0;//0: Analog  1: Digital 12  2: Digital 24
static uint32_t MenuMode=0; //0:Main Menu 1:Set Time  2:Set Alarm  3:Customize
static uint32_t Color = ST7735_BLUE;
static bool Alarm_On=false;
static bool Alarm_Armed = false;
static uint32_t inc1,inc2,inc3,inc4; //debug
static bool firstValue = true;


void PF0_button(void) // Button call functions
{
	if(MenuMode==0) //which menumode was being displayed on LCD?
	{
		 ChangeMenuMode(1);
	}
	else if(MenuMode==1 || MenuMode==2)
	{
		addTime(0x00001000);
	}
	else if(MenuMode==3)
	{
		ClockDraw_ChangeColor();
	}
	inc1++;
}
void PF2_button(void)
{
	if(MenuMode==0)
	{
		ChangeMenuMode(2);
	}
	else if(MenuMode==1 || MenuMode==2)
	{
		addTime(0x0000A000);
	}
	inc2++;
}
void PF3_button(void)
{
	if(MenuMode==0)
	{
		if(Alarm_On && Alarm_Armed)
		{
			Alarm_On = false;
			TIMER0_CTL_R &= ~TIMER_CTL_TAEN;
		}
		Alarm_Armed = ((Alarm_Armed&0x1) ^0x1);
		ClockDraw_ToggleAlarm();
	}
	else if(MenuMode==1 || MenuMode==2)
	{
		addTime(0x01000000);
	}
	else if(MenuMode==3)
	{
		ChangeClockMode((ClockMode+1)%3);
	}
	inc3++;
}
void PF4_button(void)
{
	if(MenuMode==0)
	{
		ChangeMenuMode(3);
	}
	else if(MenuMode==1)
	{
		Time = TimeSave&0xFFFFF000; //Set Time to the updated Time
		ChangeMenuMode(0);
	}
	else if(MenuMode==2)
	{
		AlarmTime = TimeSave&0xFFFFF000; //Set Time to the updated Time
		ChangeMenuMode(0);
	}
	else if(MenuMode==3)
	{
		ChangeMenuMode(0);
	}
	inc4++;
}


volatile uint32_t ADCvalue;


void Timer0A_Init100HzInt(void){ //ADC Samples in
  volatile uint32_t delay;
  DisableInterrupts();
  // **** general initialization ****
  SYSCTL_RCGCTIMER_R |= 0x01;      // activate timer0
  delay = SYSCTL_RCGCTIMER_R;      // allow time to finish activating
  TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // disable timer0A during setup
  TIMER0_CFG_R = 0;                // configure for 32-bit timer mode
  // **** timer0A initialization ****
                                   // configure for periodic mode
  TIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
  TIMER0_TAILR_R = 799999;         // start value for 100 Hz interrupts
  TIMER0_IMR_R |= TIMER_IMR_TATOIM;// enable timeout (rollover) interrupt
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;// clear timer0A timeout flag
	
  // **** interrupt initialization ****
                                   // Timer0A=priority 2
  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x40000000; // top 3 bits
  NVIC_EN0_R = 1<<19;              // enable interrupt 19 in NVIC
}
void Timer0A_Handler(void)
{
	TIMER0_ICR_R = TIMER_ICR_TATOCINT;
	if(firstValue)
	{
		ADCvalue =  ADC0_InSeq3();
		firstValue = false;
	}
	else
	{
		uint32_t result  = ADC0_InSeq3();
		//checks if IR sensor is receiving less light than when alarm activated.
		if((result - ADCvalue)>300 && (((result - ADCvalue)&0x80000000)!=0x80000000))
		{
			TIMER0_CTL_R &= ~TIMER_CTL_TAEN;
			Alarm_On = false;
			firstValue = true;
		}
	}
}

int main(void){
  	PLL_Init(Bus80MHz);         
	inc1=0;
	inc2=0;
	inc3=0;
	inc4=0;
	PWM0A_Init(50000,25000);	// 80 MHz
  	SYSCTL_RCGCGPIO_R |= 0x20;            // activate port F
	while((SYSCTL_PRGPIO_R & 0x00000020) == 0){};
	ADC0_InitSWTriggerSeq3_Ch9();
	Timer0A_Init100HzInt();
	Timer2_Init(updateTime,0);
	SysTick_Init();
  	GPIO_PORTF_AFSEL_R &= ~0x06;          // disable alt funct on PF2, PF1                          // configure PF2 as GPIO
  	GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF00F)+0x00000000;
  	GPIO_PORTF_AMSEL_R = 0;               // disable analog functionality on PF
  	PF2 = 0;                      // turn off LED
	ST7735_InitR(INITR_REDTAB);
	Switch_Init(&PF0_button,&PF2_button,&PF3_button,&PF4_button);
	ClockDraw_Init();
	Time = (23<<24) + (50<<12);
	ChangeMenuMode(0);
	ChangeClockMode(1);
	EnableInterrupts();

	while(1)
	{
		if(MenuMode ==1) // setting time dont show current time
		{
			ClockDraw_Time(TimeSave);
		}
		else if(MenuMode == 2) //setting alarm dont show current time
		{
			ClockDraw_Time(TimeSave);
		}
		else
		{
			ClockDraw_Time(Time); //display current time
		}
		if((Time) == AlarmTime && Alarm_Armed)
		{
			Alarm_On = true;
		}
		
		while(Alarm_On==true && Alarm_Armed )
		{
			TIMER0_CTL_R |= TIMER_CTL_TAEN;// enable timer0A 32-b, periodic, interrupts
			PWM0_ENABLE_R ^= 0x01; 
			ClockDraw_Time(Time); // dont pause clock
			SysTick_Wait10ms(40); //Add space between tones
		}
		PWM0_ENABLE_R &= ~0x01; 
	}
}

void updateTime(void) //roll over clock logic
{
	if((Time&0x00000FFF)==0x03B)
	{ //59 sec check
		if((Time&0x00FFF000)==0x03B000)
		{ //59 min check
			if((Time&0xFF000000)==0x17000000)
			{ //23 hours check
				Time=0x00000000;
			}
			else
			{
				Time = Time & 0xFF000000;
				Time = Time + 0x01000000;
			}
		}
		else
		{
			Time=Time&0xFFFFF000;
			Time=Time + 0x00001000;
		}
	}
	else
	{
		Time+=0x0000001;
	}
}
void addTime(uint32_t timeToAdd) //used during time set and alarm set
{
	if(((TimeSave&0x00FFF000) + (timeToAdd&0x00FFF000))>0x003b000)//roll over minutes
	{
		uint32_t rollOver = 60 - ((timeToAdd>>12) + ((TimeSave&0x00FFF000)>>12));
		TimeSave = TimeSave&0xFF000FFF; //dosent reset seconds
		TimeSave += ((-1) * rollOver)<<12;
	}
	else if((TimeSave&0xFF000000)==0x17000000 && timeToAdd==0x01000000)//roll over hours
	{
		TimeSave = TimeSave&0x00FFFFFF; //dosent reset seconds
	}
	else
	{
		TimeSave = TimeSave		+ timeToAdd;
	}
}


void ChangeClockMode(uint8_t mode) //update clock display
{
	ClockDraw_ChangeClockMode(mode);
	ClockMode = mode;
}
	
void ChangeMenuMode(uint8_t mode) //update menu display
{
	if(mode==1)
	{
		TimeSave = Time ;
		TimeSave |= 1;
	}
	if(mode==2)
	{
		TimeSave = AlarmTime+1;
	}
	ClockDraw_ChangeMenuMode(mode);
	MenuMode = mode;
}



