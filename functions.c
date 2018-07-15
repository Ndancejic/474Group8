#include "ee474.h"

//turn on given color
void LED_ON(unsigned int color){
  GPIO_DATA_PORTA |= color;
}

//make PA2 low
void LED_OFF(unsigned int color) {
  GPIO_DATA_PORT &= ~color;
}
