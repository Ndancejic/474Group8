#include "init.c"
#include "ee474.h"

void Timer0A_Handler(void) {

  GPTMICR |= (0x1<<0);
}


void PortF_Handler(void) {
  if (GPIO_PORTF_DATA == 0x01 || GPIO_PORTF_DATA == 0x05) {

  }
  if (GPIO_PORTF_DATA == 0x10 || GPIO_PORTF_DATA == 0x12) {

  }
  GPIO_PORTF_ICR |= 0x11;          // clear the interrupt flag before return
}

int main()
{

  PortF_Init();
  ADC_Init();
  Timer0_Init();
  PLL_Init();

}
