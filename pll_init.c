#include <stdint.h>

#define TIMER_RCC (*((unsigned int *)0x400FE060)) // Run-Mode Clock Configuration
#define TIMER_RCC2 (*((unsigned int *)0x400FE070)) // Run-Mode Clock Configuration 2
#define RIS (*((unsigned int *)0x400FE050))// Raw Interrupt Status

void PLL_Init(void) {
  TIMER_RCC2 |= (1u<<31); // override RCC register field
  TIMER_RCC2 |= (0x1<<11); // bypasss PLL while initializing
  TIMER_RCC = (TIMER_RCC & ~0x7C0) + 0x540; // select crystal value - 16MHz
  TIMER_RCC2 &= ~(0x7<<4); // set oscillator source to main oscillator, 000 OSCSRC2
  TIMER_RCC2 &= ~(0x1<<13); // activate PLL by clearing PWRDN
  TIMER_RCC2 |= (0x1<<30); // create a 7 bit divisor using the 400 MHz PLL output, DIV400
  TIMER_RCC2 = (TIMER_RCC2&~ 0x1FC00000) + (0x4<<22); // set desired system divider to /5, SYSDIV
//  TIMER_RCC2 = (TIMER_RCC2&~ 0x1FC00000) + (0x63<<22); // set desired system divider to /99, SYSDIV

  while ((RIS & (0x1<<6)) == 0) {}; // wait for the PLL to lock
  TIMER_RCC2 &= ~(0x1<<11); // wait for the PLL to lock
}
