#include "init.c"
#include "ee474.h"

#define STATE_RED 4
#define STATE_BLUE 5
#define FLAG_NONE 0
#define FLAG_ONE 3

#define RCGCTIMER (*((volatile unsigned long *)0x400FE604))
#define GPTMCTL (*((volatile unsigned long*)0x4003000C))
#define GPTMCFG (*((volatile unsigned long*)0x40030000))
#define GPTMTAMR (*((volatile unsigned long*)0x40030004))
#define GPTMTAILR (*((volatile unsigned long*)0x40030028))
#define GPTMRIS (*((volatile unsigned long*)0x4003001C))
#define GPTMICR (*((volatile unsigned long*)0x40030024))
#define GPTMIMR (*((volatile unsigned long*)0x40030018))

unsigned long STATE = 0;
unsigned long FLAG = FLAG_NONE;

void LED_ON(unsigned int color);
void LED_OFF(void);
void Switching(void);

// initialize timer that waits for 2 second
void Timer_Init(void) {
  RCGCTIMER |= 0x1;
  GPTMCTL &= ~0x1;  // disables timer
  GPTMCFG = 0x0;
  GPTMTAMR |= (0x2<<0);  // set TAMR field in GPTMTAMR register to periodic mode
  GPTMTAMR &= (0x0<<4);  // enables GPTM Timer to count down
  GPTMTAILR = 0x00F42400; // sets interval to one second
}

// enables timer 0
void Timer_En(void) {
  GPTMIMR |= 0x1;  // enables time out interrupt mask
  EN0 |= (0x1<<19);
  GPTMCTL |= 0x1;  // enables timer 0, start counting
  GPTMICR |= (0x1<<0); // reset timer 0
  PRI4 = (3<<29);
//  NVIC_PRI4_R = (NVIC_PRI4_R&0xFF00FFFF)|0xA0000000; 
}

// disables timer 0
void Timer_Dis(void) {
  GPTMIMR &= ~0x1;  // disables time out interrupt mask
  DIS0 &= ~(0x0<<19);
  GPTMCTL &= ~0x1;  // disables timer
  GPTMICR |= (0x1<<0); // reset timer 0
}

void Timer0A_Handler(void) {
  FLAG = FLAG_ONE;
  GPTMICR |= (0x1<<0);
}


void PortF_Handler(void) {
//  if (GPIO_PORTF_DATA == 0x01 || GPIO_PORTF_DATA == 0x05) {
//
//  }
//  if (GPIO_PORTF_DATA == 0x10 || GPIO_PORTF_DATA == 0x12) {
//
//  }
//  GPIO_PORTF_ICR |= 0x11;          // clear the interrupt flag before return
}

int main()
{
  
  PortF_Init();
  ADC_Init();
  Timer0_Init();
  PLL_Init();
  Switching();
  
  return 0;
}

//turn on given color
void LED_ON(unsigned int color){
  GPIO_WRITE_PORTF |= 0x00;
  GPIO_WRITE_PORTF |= color;
}

//make PA2 low
void LED_OFF(void) {
  GPIO_WRITE_PORTF = 0x00;
}

void Switching(void) {
  int flash = 0;
  while (1) {
    switch (GPIO_WRITE_PORTF & 0x11) {
    case 0x01: // switch 1
      STATE = STATE_RED;
      Timer_Dis();
      break;
    case 0x10: // switch 2
      STATE = STATE_BLUE;
      Timer_En();
      break;
    default:
      break;
    }
    switch (STATE) {
    case STATE_RED:
      GPIO_WRITE_PORTF = 0x00;
      GPIO_WRITE_PORTF |= 0x02;
      break;
    case STATE_BLUE:
      if (FLAG == FLAG_ONE) {
        FLAG = FLAG_NONE;
        if (flash) {
          flash = 0;
          GPIO_WRITE_PORTF = 0x00;
          GPIO_WRITE_PORTF |= 0x04;
        } else {
          flash = 1;
          GPIO_WRITE_PORTF = 0x00;
        }
      }
      break;
    }

  }
}
