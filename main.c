#include "init.c"
#include "ee474.h"

int main()
{
  
  PortF_Init();
  ADC_Init();
  Timer0_Init();
  PLL_Init();
}
