//filename: ClockDraw.h
//Bryce Cotner, Gorkem Caylak
//February 11, 2018
//Header File for the Clock LCD Interface
//Lab 3
//Saadallah Kassir
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


#include <stdint.h>


/* 	ClockDraw_Line
// 	Initalized screen and draws the analog clock bitmap
//	Inputs: None
//	Outputs: None
*/
void ClockDraw_Init(void);


//************* ClockDraw_Line********************************************
//  Draws one//  Inputs: (x1,y1) is the start point
// line on the ST7735 color LCD
//          (x2,y2) is the end point
// x1,x2 are horizontal positions, columns from the left edge
//               must be less than 128
//               0 is on the left, 126 is near the right
// y1,y2 are vertical positions, rows from the top edge
//               must be less than 160
//               159 is near the wires, 0 is the side opposite the wires
//        color 16-bit color, which can be produced by ST7735_Color565() 
// Output: none


void ClockDraw_Line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, 
                 uint16_t color);


/*
//  Draw Analog or Digital time to ST7735
//  Inputs: time is the current system time in hours minutes and seconds 
// Output: none
*/
void ClockDraw_Time(uint32_t time);


/*	Changed Current Display Mode to enumarted value
//	Inputs: mode the display mode to show
*/
void ClockDraw_ChangeClockMode(uint8_t mode);

/*	Changed Current Menu Mode to enumarted value
//	Inputs: mode the display mode to show
*/
void ClockDraw_ChangeMenuMode(uint8_t mode);

/*	Toggles Clock Color
//	Inputs: none
*/
void ClockDraw_ChangeColor(void);

/* Toggles Alarm dsplays armed or not
// Inputs: none
*/
void ClockDraw_ToggleAlarm(void);

