#include <stdint.h>
#include <math.h>
#include "init.c"
#include "ee474.h"
#include "SSD2119.h"

#define BMP_WIDTH_OFFSET        0x0012
#define BMP_HEIGHT_OFFSET       0x0016

#define FLAG_NONE 0
#define FLAG_ONE 3
#define FLAG_ADC 5

#define RED 0x02
#define BLUE 0x04
#define VIOLET 0x06
#define GREEN 0x08
#define YELLOW 0x0A
#define LIGHT_BLUE 0x0C
#define WHITE 0x0E

#define MHZ16 (unsigned long)0xF42400
#define MHZ80 (unsigned long)0x4C4B400
#define MHZ4 (unsigned long)0x3D0900

typedef enum {
  STOP,
  GO,
  PASS
} state;

state STATE;
unsigned long FLAG = FLAG_NONE;
unsigned long result = 0;
unsigned char output[3];

typedef struct Node {
  short x, y, z;
} Node;

typedef struct Edge {
  short n1, n2;
} Edge;

typedef struct Nodes {
  Node n[8];
} Nodes;

typedef struct Edges {
  Edge e[1];
} Edges;

Node nodes[8] = {{-45, -45, -45}, {-45, -45, 45}, {-45, 45, -45}, {-45, 45, 45}, {45, -45, -45}, {45, -45, 45}, {45, 45, -45}, {45, 45, 45}};
Edge edges[12] = {{0, 1}, {1, 3}, {3, 2}, {2, 0}, {4, 5}, {5, 7}, {7, 6}, {6, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};


//draw
int centerx = 160;
int centery = 120;
int radius = 30;
int width = 60;
int height = 6*20 + 20;
int starty = 20;


void LED_ON(unsigned int color);
void LED_OFF(void);
void Switching(void);
void Rotating_Cube(void);
void Rotate_X(short theta);
void LCD_Switch(void);

void transmit(char data){
  while(UART0_FR&0x0020 != 0);
  UART0_DR = data;
}

int recieve(){
  while(UART0_FR&0x0010 != 0);
  return ((int) (UART0_DR&0xFF));
}

void Timer0A_Handler(void) {
  FLAG = FLAG_ONE;
  if(!LCD_TOUCH){
    ADC0_PSSI = 0x008;
  }
  TIMER_RESET0 |= (0x1<<0);
}


void PortF_Handler(void) {
  if (GPIO_WRITE_PORTF == 0x01) {
    ADC0_IM &= ~0x8;
    PLL_Init(4);
    UART_Init(4);
    TIMER_VAL0 = MHZ4;  //set timer start to 4000000
    ADC0_IM |= 0x8;
    LED_ON(RED);
  }
  if (GPIO_WRITE_PORTF == 0x10) {
    ADC0_IM &= ~0x8;
    PLL_Init(80);
    UART_Init(80);
    TIMER_VAL0 = MHZ80;  //set timer start to 4000000
    ADC0_IM |= 0x8;
  }
  GPIO_ICR_PORTF |= 0x11; // clear the interrupt flag before return
}

void ADC0_Handler(void) {
  FLAG = FLAG_ADC;
  while((ADC0_RIS&0x08)==0){};   // wait for conversion done
  unsigned long ADCvalue = ADC0_SSFIFO3&0xFFF;   // read result
  result = (long) (147.5-((74*(3.3)*ADCvalue)/4096));
  ADC0_ISC = 0x0008;             // acknowledge completion
  ADC0_DCISC |=0x8;
}

void drawStoplight(void){
  radius = 20;
  switch(STATE){
    case STOP:
      LCD_DrawFilledRect(centerx-width/2, starty, width, height, ((0xAA>>3)<<11) | ((0x55>>2)<<5) | (0x00>>3));
      LCD_DrawRect(centerx-width/2, starty, width, height, 0);
      LCD_DrawFilledCircle(centerx, starty + radius + 5, radius, ((0x00>>3)<<11) | ((0xAA>>2)<<5) | (0x00>>3));
      LCD_DrawCircle(centerx, starty + radius + 5, radius, 0);
      LCD_DrawFilledCircle(centerx, starty + 3*radius + 10, radius, ((0xAA>>3)<<11) | ((0xAA>>2)<<5) | (0xAA>>3));
      LCD_DrawCircle(centerx, starty + 3*radius + 10, radius, 0);
      LCD_DrawFilledCircle(centerx, starty + 5*radius + 15, radius, ((0xFF>>3)<<11) | ((0x55>>2)<<5) | (0x55>>3));
      LCD_DrawCircle(centerx, starty + 5*radius + 15, radius, 0);
      break;
    case GO:
      LCD_DrawFilledRect(centerx-width/2, starty, width, height, ((0xAA>>3)<<11) | ((0x55>>2)<<5) | (0x00>>3));
      LCD_DrawRect(centerx-width/2, starty, width, height, 0);
      LCD_DrawFilledCircle(centerx, starty + radius + 5, radius, ((0x55>>3)<<11) | ((0xFF>>2)<<5) | (0x55>>3));
      LCD_DrawCircle(centerx, starty + radius + 5, radius, 0);
      LCD_DrawFilledCircle(centerx, starty + 3*radius + 10, radius, ((0xAA>>3)<<11) | ((0xAA>>2)<<5) | (0xAA>>3));
      LCD_DrawCircle(centerx, starty + 3*radius + 10, radius, 0);
      LCD_DrawFilledCircle(centerx, starty + 5*radius + 15, radius, ((0xAA>>3)<<11) | ((0x00>>2)<<5) | (0x00>>3));
      LCD_DrawCircle(centerx, starty + 5*radius + 15, radius, 0);
      break;
    case PASS:
      LCD_DrawFilledRect(centerx-width/2, starty, width, height, ((0xAA>>3)<<11) | ((0x55>>2)<<5) | (0x00>>3));
      LCD_DrawRect(centerx-width/2, starty, width, height, 0);
      LCD_DrawFilledCircle(centerx, starty + radius + 5, radius, ((0x00>>3)<<11) | ((0xAA>>2)<<5) | (0x00>>3));
      LCD_DrawCircle(centerx, starty + radius + 5, radius, 0);
      LCD_DrawFilledCircle(centerx, starty + 3*radius + 10, radius, ((0xFF>>3)<<11) | ((0xFF>>2)<<5) | (0x55>>3));
      LCD_DrawCircle(centerx, starty + 3*radius + 10, radius, 0);
      LCD_DrawFilledCircle(centerx, starty + 5*radius + 15, radius, ((0xAA>>3)<<11) | ((0x00>>2)<<5) | (0x00>>3));
      LCD_DrawCircle(centerx, starty + 5*radius + 15, radius, 0);
      break;
  }
}

void Rotate_X(short theta) {
  double sinTheta = sin(theta);
  double cosTheta = cos(theta);
  for (int i = 0; i < 8; i++) {
    short y = nodes[i].y;
    short z = nodes[i].z;
    nodes[i].y = (short)(y * cosTheta - z * sinTheta);
    nodes[i].z = (short)(z * cosTheta + y * sinTheta);
  }
}

void Rotate_Y(short theta) {
  double sinTheta = sin(theta);
  double cosTheta = cos(theta);
  for (int i = 0; i < 8; i++) {
    short x = nodes[i].x;
    short z = nodes[i].z;
    nodes[i].x = (short)(x * cosTheta - z * sinTheta);
    nodes[i].z = (short)(z * cosTheta + x * sinTheta);
  }
}

void Rotate_Z(short theta) {
  double sinTheta = sin(theta);
  double cosTheta = cos(theta);
  for (int i = 0; i < 8; i++) {
    short y = nodes[i].y;
    short x = nodes[i].x;
    nodes[i].y = (short)(y * cosTheta - x * sinTheta);
    nodes[i].x = (short)(x * cosTheta + y * sinTheta);
  }
}

void drawCube(){
  for (int i = 0; i < 12; i++){
    Node n1 = nodes[edges[i].n1];
    Node n2 = nodes[edges[i].n2];
    LCD_DrawLine(centerx + n1.x, centery + n1.y, centerx + n2.x, centery + n2.y, Color4[0]);
  }
}

void eraseCube(){
  for (int i = 0; i < 12; i++){
    Node n1 = nodes[edges[i].n1];
    Node n2 = nodes[edges[i].n2];
    LCD_DrawLine(centerx + n1.x, centery + n1.y, centerx + n2.x, centery + n2.y, Color4[15]);
  }
}

int main()
{
  PLL_Init(16);
  Interrupt_Init();
  if(!LCD_TOUCH){
    ADC_Init();
    Timer0_Init(MHZ16);
  }
  if(LCD_USED){
    LCD_Init();
    Touch_Init();

    STATE = STOP;

    //set up background
    LCD_ColorFill(((0xFF>>3)<<11) | ((0xFF>>2)<<5) | (0xFF>>3));

    if(TRAFFIC){
      //draw Buttons
      LCD_DrawFilledCircle(centerx - 90, centery + 40, radius, ((0xFF>>3)<<11) | ((0x55>>2)<<5) | (0x55>>3));
      LCD_DrawCircle(centerx - 90, centery + 40, radius, 0);
      LCD_DrawFilledCircle(centerx + 90, centery + 40, radius, ((0x55>>3)<<11) | ((0x55>>2)<<5) | (0xFF>>3));
      LCD_DrawCircle(centerx + 90, centery + 40, radius, 0);

      //draw stoplight
      drawStoplight();
    }

    if(CUBE){
      drawCube();
    }
    
    LCD_SetTextColor(0,0,0);
    LCD_Switch();
  } else {
    PortF_Init();
    UART_Init(16);
    LED_OFF();
    Switching();
  }
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

//switching if LCD not initialized
void Switching(void) {
  while (1) {
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

    if(FLAG == FLAG_ADC){
      unsigned short tempResult = (unsigned short)result;
      output[0] = tempResult/10 + 0x30;  // tens digit
      tempResult = tempResult%10;
      output[1] = tempResult + 0x30;     // ones digit
      output[2] = 0;            // null termination
      transmit(output[0]);
      transmit(output[1]);
      FLAG = FLAG_NONE;
    }
  }
}

//switching temperature if LCD on
void LCD_Switch(void){
  while (1) {
    if(LCD_TOUCH){
      LCD_DrawFilledRect(0, 0, 22, 18, ((0xFF>>3)<<11) | ((0xFF>>2)<<5) | (0xFF>>3));
      long cal[7] = {
        280448,	//   86784,          // M0
        -3200,	// -1536,          // M1
        -220093760,	//-17357952,      // M2
        -3096,		//-144,           // M3
        -275592,		//-78576,         // M4
        866602824,	// 69995856,       // M5
        2287498		//201804,         // M6
      };
      long xPos, yPos, temp;
      xPos = Touch_ReadX();
      yPos = Touch_ReadY();

      temp = (((xPos * cal[0]) + (yPos * cal[1]) + cal[2]) / cal[6]);
      yPos =(((xPos * cal[3]) + (yPos * cal[4]) + cal[5]) / cal[6]);
      xPos = temp;

      xPos = xPos - 100;
      if(xPos < 0) xPos = 0;
      yPos = yPos - 80;
      if(yPos < 0) yPos = 0;

      LCD_SetCursor(0,0);
      LCD_PrintInteger(xPos);
      LCD_SetCursor(0,8);
      LCD_PrintInteger(yPos);
      if(TRAFFIC){
        if(yPos > 20 && yPos < 60 && xPos > 100 && xPos < 140){
          STATE = PASS;
          drawStoplight();
        }
        if(yPos > 60 && yPos < 100 && xPos > 10 && xPos < 50){
          STATE = GO;
          drawStoplight();
        }
      }
      if(CUBE){
        eraseCube();
        Rotate_X(10);
        Rotate_Y(10);
        drawCube();
      }
    } else {
      if(FLAG == FLAG_ADC){
        LCD_ColorFill(((0xFF>>3)<<11) | ((0xFF>>2)<<5) | (0xFF>>3));
        LCD_SetCursor(0,0);
        LCD_PrintInteger(result);
        FLAG = FLAG_NONE;
      }
    }
  }
}
