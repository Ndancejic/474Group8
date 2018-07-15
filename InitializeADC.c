//initializes adc and gpio
void Init_ADC(void){
  //enable GPIO pin
  GPIO_EN |= 0x10; //enable PE4
  delay = GPIO_EN; delay
  GPIO_DIR_PORTE &= ~0x04;
  GPIO_REG_PORTE |= 0x04; //analog function
  GPIO_DEN_PORTE &= ~0x04; //enable analog
  GPIO_AMSEL_PORTE |= 0x04; //disable isolation

  //enable ADC0
  //ADC_CLK_EN |= 0x1; //enable ADC0
  ADC_RCGC0 |= 0x10000;//activate ADC
  delay = ADC_RCGC0; //wait for Clock
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
