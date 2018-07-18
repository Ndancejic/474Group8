#include "ee474.h"
#include "intrinsics.h"

void ADC_Init(void){
  //enable GPIO pin
  RCGCGPIO |= 0x10; //enable PE4
  GPIO_DIR_PORTE &= ~0x04;
  GPIO_REG_PORTE |= 0x04; //analog function
  GPIO_DEN_PORTE |= 0x04; //enable analog
  GPIO_AMSEL_PORTE &= ~0x04; //disable isolation
  
  //enable ADC0
  //ADC_CLK_EN |= 0x1; //enable ADC0
  ADC_RCGC0 |= 0x10000;//activate ADC
  ADC_RCGC0 &= ~0x300; //set max freq
  
  //enable sequencer 3
  ADC0_SSPRI = 0x0123; //set priority for ss3
  ADC0_ACTSS &= ~0x08; //disable ss3
  ADC0_EMUX &= ~0xF000; //set input to ss3
  ADC0_SSMUX3 &= ~0x000F; //clear field
  ADC0_SSMUX3 += 9; // input Ain9
  ADC0_SSCTL3 = 0x000E; //temperature sensor
  ADC0_ACTSS |= 0x08; //enable ss3
  ADC0_IM |= 0x8;
}

void PLL_Init(int mhz) {
  CLK_RCC2 |= (1u<<31); // override RCC register field
  CLK_RCC2 |= (0x1<<11); // bypasss PLL while initializing
  CLK_RCC = (CLK_RCC & ~0x7C0) + 0x540; // select crystal value - 16MHz
  CLK_RCC2 &= ~(0x7<<4); // set oscillator source to main oscillator, 000 OSCSRC2
  CLK_RCC2 &= ~(0x1<<13); // activate PLL by clearing PWRDN
  CLK_RCC2 |= (0x1<<30); // create a 7 bit divisor using the 400 MHz PLL output, DIV400
  if (mhz == 80){
    CLK_RCC2 = (CLK_RCC2&~ 0x1FC00000) + (0x4<<22); // set desired system divider to /5, SYSDIV
  }
  if (mhz == 4){
    CLK_RCC2 = (CLK_RCC2&~ 0x1FC00000) + (0x63<<22); // set desired system divider to /100, SYSDIV
  }
  if (mhz == 16){
    CLK_RCC2 = (CLK_RCC2&~ 0x1FC00000) + (0x18<<22); // set desired system divider to /25, SYSDIV
  }
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

void PortA_Init(void)
{
  RCGCGPIO = 0xff;//enables GPIO clock on port
  GPIO_DEN_PORTA &= ~0x02;  //disables digital PORT A
  GPIO_AFSEL_PORTA &= ~0x02; //regular port function
  GPIO_PCTL_PORTA |= 0x1; //PCTL
  GPIO_DIR_PORTA &= ~0x02;  //set port A as output except for PF0 and PF4
  GPIO_DEN_PORTA |= 0x02;  //enables digital PORT A
}

//timer initialization
void Timer0_Init(unsigned long mhz){
  TIMER_EN |= 0x01; //enable Timer0
  TIMER_DIS0 &= ~0x01;//Ensure that timer is disbled
  TIMER_CON0 = 0x00000000;  //select 32-bit timer
  TIMER_MODE0 |= 0x12;  //configure TAMR field in GPTMTAMR (set to periodic and count down)
  TIMER_VAL0 = mhz;  //set timer start to 16000000
  TIMER_INT0 |= 0x1F;  //enable Interrupts
  TIMER_DIS0 |= 0x11;  //enable the timer and output interrupt
}

void Interrupt_Init(void)
{
  GPIO_SENSE_F &= ~0x11;  //interrupt on edge
  GPIO_IBE_F &= ~0x11;  //interrupt on one edge
  GPIO_IEV_F |= 0x11;  //interrupt on rising edge
  GPIO_CLEAR_F = 0x11;  //ICR
  GPIO_IM_F |= 0x11;  //interupt mask register
  EN0 |= (1<<17); //enable #17
  EN0 |= 0x40080000;  //enable #19
  PRI4 |= (1<<13);  //Interrupt priority
  PRI4 |= 0x20000000;  //Interrupt priority
  PRI7 |= 0x00200000;  //Interrupt priority
  __enable_interrupt();  //enable global interrupts
}

void UART_Init(int mhz)
{
  //enabling clocks
  RCGCUART |= 0x01; //enables clock on UART0
  for(int i = 0; i < 100; i ++);
  RCGCGPIO |= 0x1; //enables GPIO clock on portA
  for(int i = 0; i < 100; i ++);
  
  UART0_CTL &= ~0x00000001; //disable UART0
  if (mhz == 80) {
    UART0_IBRD = 520;
    UART0_FBRD = 54;
  }
  if (mhz == 16) {
    UART0_IBRD = 104;
    UART0_FBRD = 11;
  }
  if (mhz == 4) {
    UART0_IBRD = 26;
    UART0_FBRD = 3;
  }
  UART0_CC = 0;
  UART0_LCRH = 0x00000060;
  UART0_CTL |= 0x00000300; //enable transmit and recieve UART0
  UART0_CTL |= 0x00000001; //enable UART0
  
  //setting up GPIO
  GPIO_ADIS_PORTA &= ~0x03; // disable analog
  GPIO_AFSEL_PORTA |= 0x03; //alternate hardware functions
  GPIO_DEN_PORTA |= 0x03;
  GPIO_DIR_PORTA = 0x2;

  GPIO_DR2R_PORTA &= ~0x03;
  GPIO_DR4R_PORTA &= ~0x03;
  GPIO_DR8R_PORTA |= 0x03;
  GPIO_SLR_PORTA |= 0x03;
  GPIO_PCTL_PORTA |= (GPIO_PCTL_PORTA&0xFFFFFF00) + 0x11; // configure PA0 and PA1 pins for UART functions
  while(GPIO_DATA_PORTA&0x2 != 0x2);
  GPIO_DATA_PORTA = 0;
}
