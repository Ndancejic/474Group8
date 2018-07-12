//initializes adc
void Init_ADC(void){
  ADC_CLK_EN |= 0x1; //enable ADC0
  GPIO_EN |= 0x10; //enable PE3
  GPIO_REG_PORTE |= 0x08; //analog function
  GPIO_DEN_PORTE &= ~0x08; //enable analog
  GPIO_AMSEL_PORTE |= 0x08; //disable isolation
}
