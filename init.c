#include "ee474.h"

void ADC_Init(void){
  //enable GPIO pin
  RCGCGPIO |= 0x10; //enable PE4
//  delay = RCGCGPIO; 
  GPIO_DIR_PORTE &= ~0x04;
  GPIO_REG_PORTE |= 0x04; //analog function
  GPIO_DEN_PORTE &= ~0x04; //enable analog
  GPIO_AMSEL_PORTE |= 0x04; //disable isolation

  //enable ADC0
  //ADC_CLK_EN |= 0x1; //enable ADC0
  ADC_RCGC0 |= 0x10000;//activate ADC
//  delay = ADC_RCGC0; //wait for Clock
  ADC_RCGC0 |= 0x300; //set max freq

  //enable sequencer 3
  ADC0_SSPRI = 0x0123; //set priority for ss3
  ADC0_ACTSS &= ~0x08; //disable ss3
  ADC0_EMUX |= 0x5000; //set input to ss3
  ADC0_SSMUX3 |= 0x000F; //clear field
  ADC0_SSMUX3 += 9; // input Ain9
  ADC0_SSCTL3 = 0x000A; //temperature sensor
  ADC0_ACTSS |= 0x08; //enable ss3
}

void PLL_Init(void) {
  CLK_RCC2 |= (1u<<31); // override RCC register field
  CLK_RCC2 |= (0x1<<11); // bypasss PLL while initializing
  CLK_RCC = (CLK_RCC & ~0x7C0) + 0x540; // select crystal value - 16MHz
  CLK_RCC2 &= ~(0x7<<4); // set oscillator source to main oscillator, 000 OSCSRC2
  CLK_RCC2 &= ~(0x1<<13); // activate PLL by clearing PWRDN
  CLK_RCC2 |= (0x1<<30); // create a 7 bit divisor using the 400 MHz PLL output, DIV400
  CLK_RCC2 = (CLK_RCC2&~ 0x1FC00000) + (0x4<<22); // set desired system divider to /5, SYSDIV
//  TIMER_RCC2 = (TIMER_RCC2&~ 0x1FC00000) + (0x63<<22); // set desired system divider to /99, SYSDIV

  while ((RIS & (0x1<<6)) == 0) {}; // wait for the PLL to lock
  CLK_RCC2 &= ~(0x1<<11); // wait for the PLL to lock
}

void PortF_Init(void)
{
  RCGCGPIO = 0xff;//enables GPIO clock on port

  GPIO_LOCK_PORTF = 0x4C4F434B;//unlocks pin pf0
  GPIO_CR_PORTF = 0xff;  //commits pin pf0
  GPIO_PUR_PORTF = 0x11;  //turns on pulldown resistors

  GPIO_DIR_PORTF = 0xee;  //set port F as output except for PF0 and PF4
  GPIO_DEN_PORTF = 0xff;  //enables digital PORT F
}

//timer initialization
void Timer0_Init(void){
  TIMER_EN |= 0x01; //enable Timer0
  TIMER_DIS0 &= ~0x01;//Ensure that timer is disbled
  TIMER_CON0 = 0x00000000;  //select 32-bit timer
  TIMER_MODE0 |= 0x12;  //configure TAMR field in GPTMTAMR (set to periodic and count down)
  TIMER_VAL0 = 0xF42400;  //set timer start to 16000000
  TIMER_INT0 |= 0x1F;  //enable Interrupts
  TIMER_DIS0 |= 0x1;  //enable the timer
}
