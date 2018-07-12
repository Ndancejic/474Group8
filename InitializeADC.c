//initializes adc and gpio
void Init_ADC(void){
  ADC_CLK_EN |= 0x1; //enable ADC0
  GPIO_EN |= 0x10; //enable PE3
  GPIO_REG_PORTE |= 0x08; //analog function
  GPIO_DEN_PORTE &= ~0x08; //enable analog
  GPIO_AMSEL_PORTE |= 0x08; //disable isolation


  /*
  enable module
  diable ss3
  trigger sequencer 3
  tale one sample at time set flag at 1st sample
  enable adc0 interrupt Mask

  Temo = (147.5 - (247.5 * adc0_ssfifo3_r)/4096.0); read result of ADC_CLK_EN
  handler measure temperature and determines whcih LED to turn once
  and determines which led to turn on
  clear flag, clear interrupt
  ADC0_ISC_R
  */
}
