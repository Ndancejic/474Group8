#include "init.c"
#include "ee474.h"

#define STATE_SW0 4
#define STATE_SW4 5
#define FLAG_NONE 0
#define FLAG_ONE 3

#define RED 0x02
#define BLUE 0x04
#define VIOLET 0x06
#define GREEN 0x08
#define YELLOW 0x0A
#define LIGHT_BLUE 0x0C
#define WHITE 0x0E

unsigned long STATE = 0;
unsigned long FLAG = FLAG_NONE;
unsigned long result = 0;

void LED_ON(unsigned int color);
void LED_OFF(void);
void Switching(void);

void Timer0A_Handler(void) {
  FLAG = FLAG_ONE;
  ADC0_PSSI = 0x008;
  TIMER_RESET0 |= (0x1<<0);
}


void PortF_Handler(void) {
  if (GPIO_WRITE_PORTF == 0x01) {
    STATE = STATE_SW0;
  }
  if (GPIO_WRITE_PORTF == 0x10) {
    STATE = STATE_SW4;
  }
  GPIO_ICR_PORTF |= 0x11; // clear the interrupt flag before return
}

void ADC0_Handler(void) {
  while((ADC0_RIS&0x08)==0){};   // wait for conversion done
  result = ADC0_SSFIFO3&0xFFF;   // read result
  result = 147.5-((74*(3.3)*result)/4096);
  GPIO_DATA_PORTA = result;
  ADC0_ISC = 0x0008;             // acknowledge completion
}

int main()
{

  PortF_Init();
  PortA_Init();
  ADC_Init();
  Timer0_Init();
  PLL_Init(16);
  Interrupt_Init();
  LED_OFF();
  Switching();

  return 0;
}

//turn on given color
void LED_ON(unsigned int color){
  GPIO_WRITE_PORTF = 0x00;
  GPIO_WRITE_PORTF |= color;
}

//make PA2 low
void LED_OFF(void) {
  GPIO_WRITE_PORTF = 0x00;
}

void Switching(void) {
  while (1) {
    switch (GPIO_WRITE_PORTF & 0x11) {
    case 0x01: // switch 1
      STATE = STATE_SW0;
      break;
    case 0x10: // switch 2
      STATE = STATE_SW4;
      break;
    default:
      break;
    }
    switch (STATE) {
    case STATE_SW0:
      PLL_Init(4);
      break;
    case STATE_SW4:
      PLL_Init(80);
      break;
    }

    if (result > 0 && result <= 17) {
      LED_ON(RED);
    } else if (result > 17 && result <= 19) {
      LED_ON(BLUE);
    } else if (result > 19 && result <= 21) {
      LED_ON(VIOLET);
    } else if (result > 21 && result <= 23) {
      LED_ON(GREEN);
    } else if (result > 23 && result <= 25) {
      LED_ON(YELLOW);
    } else if (result > 25 && result <= 27) {
      LED_ON(LIGHT_BLUE);
    } else if (result > 27 && result <= 40) {
      LED_ON(WHITE);
    } else {
      LED_OFF();
    }
  }
}
