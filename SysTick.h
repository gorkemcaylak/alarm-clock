
/*filename: Systick.h
;Bryce Cotner, Gorkem Caylak
;February 11, 2018
; Provide functions that initialize the SysTick module, wait at least a
; designated number of clock cycles, and wait approximately a multiple
; of 10 milliseconds using busy wait.  After a power-on-reset, the
; LM4F120 gets its clock from the 16 MHz precision internal oscillator,
; which can vary by +/- 1% at room temperature and +/- 3% across all
; temperature ranges.  If you are using this module, you may need more
; precise timing, so it is assumed that you are using the PLL to set
; the system clock to 50 MHz.  This matters for the function
; SysTick_Wait10ms(), which will wait longer than 10 ms if the clock is
; slower.
;Lab 3
;Saadallah Kassir
;February 6, 2018*/

#include <stdint.h>


/*;------------SysTick_Init------------
; Initialize SysTick with busy wait running at bus clock.
; Input: none
; Output: none
; Modifies: R0, R1*/
void SysTick_Init(void);

/*;------------SysTick_Wait------------
; Time delay using busy wait.
; Input: R0  delay parameter in units of the core clock (units of 12.5 nsec for 80 MHz clock)
; Output: none
; Modifies: R0, R1, R3*/
void SysTick_Wait(uint32_t delay);

/*------------SysTick_Wait10ms------------
; Time delay using busy wait.  This assumes 50 MHz clock
; Input: R0  number of times to wait 10 ms before returning
; Output: none
; Modifies: R0*/
void SysTick_Wait10ms(uint32_t delay);



	