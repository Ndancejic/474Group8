#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include "init.c"
#include "ee474.h"
#include "SSD2119.h"
#include "draw.c"

#define BMP_WIDTH_OFFSET        0x0012
#define BMP_HEIGHT_OFFSET       0x0016

#define FLAG_NONE 0
#define FLAG_ONE 3
#define FLAG_ADC 5
#define FLAG_DRAW 2

// on board led
#define RED 0x02
#define BLUE 0x04
#define VIOLET 0x06
#define GREEN 0x08
#define YELLOW 0x0A
#define LIGHT_BLUE 0x0C
#define WHITE 0x0E

// traffic light led
#define PE1_GREEN 0x02
#define PE2_YELLOW 0x04
#define PE3_RED 0x08

// switch flag
#define SW1 1
#define SW2 2

// mHz in hex
#define MHZ16 (unsigned long)0xF42400
#define MHZ80 (unsigned long)0x4C4B400
#define MHZ100 (unsigned long)0x5F5E100
#define MHZ4 (unsigned long)0x3D0900
#define MHZ5 (unsigned long)0x4C4B40
#define MHZ40 (unsigned long)0x2625A00

// center of the screen (pixels)
#define centerx 160
#define centery 120

// ajust these when coordinates are changed
#define start_coorx 900
#define start_coory 900
#define pass_coorx 200
#define pass_coory 700
#define untouch_coorx 1200
#define untouch_coory 700

long yTouch = 615;
long xTouch = 2050;
long xPos, yPos;

typedef enum {
  STOP,
  GO,
  PASS,
  INIT
} state;

state STATE;
unsigned long FLAG = FLAG_NONE;
unsigned long result = 0;
unsigned char output[3];
unsigned long SW = 0;
bool TOUCH_ENABLE = false;
int TIMER_2 = 0;
int TIMER_5 = 0;
long TOUCHED = 0;

typedef struct Node {
  double x, y, z;
} Node;

typedef struct Edge {
  short n1, n2;
} Edge;

typedef struct Plane {
  short n1, n2, n3, n4;
} Plane;

typedef struct Nodes {
  Node n[8];
} Nodes;

typedef struct Edges {
  Edge e[1];
} Edges;

typedef struct Planes {
  Plane p[6];
} Planes;

Node nodes[8] = {{-45, -45, -45}, {-45, -45, 45}, {-45, 45, -45}, {-45, 45, 45}, {45, -45, -45}, {45, -45, 45}, {45, 45, -45}, {45, 45, 45}};
Edge edges[12] = {{0, 1}, {1, 3}, {3, 2}, {2, 0}, {4, 5}, {5, 7}, {7, 6}, {6, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
Plane planes[6] = {{0, 1, 2, 3}, {4, 5, 0, 1}, {0, 4, 2, 6}, {4, 5, 6, 7}, {5, 1, 7, 3}, {6, 7, 2, 3}};


void Switching(void);
void Rotating_Cube(void);
void Rotate_X(short theta);
void LCD_Switch(void);
void drawStoplight(int state, int radius, int width, int height, int starty);
void Draw_Cube(void);
void Erase_Cube(void);
void LCD(void);
void LCD_Print_Temp(void);
void LCD_Cube(void);
void LCD_Cube_Colored(void);
void LCD_Traffic(void);
void Rotate_X(short theta);
void Rotate_Y(short theta);
void Rotate_Z(short theta);
double findSlope(double x0, double y0, double x1, double y1);
double findY(double slope, double x0, double y0, double xPos);
void Draw_Filled_Cube(unsigned short color);
void LCD_DrawFilledPara(Plane p, unsigned short color);
int Get_NodeZ();


void transmit(char data){
  while(UART0_FR&0x0020 != 0);
  UART0_DR = data;
}

int recieve(){
  while(UART0_FR&0x0010 != 0);
  return ((int) (UART0_DR&0xFF));
}

//turn on given color Port F
void LED_ON(unsigned int color){
  GPIO_WRITE_PORTF = 0x00;
  GPIO_WRITE_PORTF |= color;
}

void LED_OFF(void) {
  GPIO_WRITE_PORTF = 0x00;
}

void LED_On_PortE(unsigned int color) {
  GPIO_DATA_PORTE = 0;
  GPIO_DATA_PORTE |= color;
}

void LED_Off_PortE(void) {
  GPIO_DATA_PORTE = 0;
}

void Timer0A_Handler(void) {
  FLAG = FLAG_ONE;
  if(!TOUCH_ENABLE)  
    ADC0_PSSI = 0x008;
  TIMER_2++;
  TIMER_5++;
  FLAG = FLAG_DRAW;
  TIMER_RESET0 |= (0x1<<0);
}

void PortF_Handler(void) {
  if (GPIO_READ_PF0_PF4 == 0x01)
    SW = SW1;
  if (GPIO_READ_PF0_PF4 == 0x10)
    SW = SW2;
  
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

int main()
{  
  
//  Switching();
  //  LCD_Print_Temp();
  //    LCD_Cube();
    LCD_Cube_Colored();
  //  LCD_Traffic();
  
  return 0;
}

void LCD_Cube_Colored(void) {
  //  Timer0_Init(MHZ16);
  PLL_Init(80);
  LCD_Init();
  Touch_Init();
  TOUCH_ENABLE = true;
  Touch_BeginWaitForTouch();
//  Timer_Interrupt();
  unsigned short color = 1;
  
  LCD_ColorFill(Color4[15]);
  LCD_SetTextColor(0,0,0);
  Draw_Filled_Cube(Color4[color%16]);
  
  while (1) {
    LCD_DrawFilledRect(0, 0, 25, 18, ((0xFF>>3)<<11) | ((0xFF>>2)<<5) | (0xFF>>3));
    LCD_SetCursor(0,0);
    LCD_PrintInteger(xPos);
    LCD_SetCursor(0,8);
    LCD_PrintInteger(yPos);
    
    xPos = Touch_ReadX();
    yPos = Touch_ReadY();
    
    xPos = xPos - 1080;
    if(xPos < 0) xPos = 0;
    yPos = yPos - 1360;
    if(yPos < 0) yPos = 0;
    
    // screen's been touched
    if(abs(xPos - untouch_coorx) >= 200 && abs(yPos - untouch_coory) >= 200){
      color ++;
    } 
    //    if (FLAG == FLAG_DRAW) {
    FLAG = FLAG_NONE;
    LCD_ColorFill(Color4[15]);
    //      Draw_Filled_Cube(Color4[15]);
    //      Erase_Cube();
    Rotate_X(30);
    Rotate_Y(30);
    Draw_Filled_Cube(Color4[color % 16]);
    for (int i = 0; i < 100000; i++);
    //    }
  }
}

void LCD_Cube(void) {
  Timer0_Init(MHZ5);
  PLL_Init(5);
  LCD_Init();
  Touch_Init();
  TOUCH_ENABLE = true;
  Touch_BeginWaitForTouch();
  
  LCD_ColorFill(Color4[15]);
  LCD_SetTextColor(0,0,0);
  Draw_Cube();
  bool rotate = false;
  
  while (1) {
    LCD_DrawFilledRect(0, 0, 25, 18, ((0xFF>>3)<<11) | ((0xFF>>2)<<5) | (0xFF>>3));
    LCD_SetCursor(0,0);
    LCD_PrintInteger(xPos);
    LCD_SetCursor(0,8);
    LCD_PrintInteger(yPos);
    
    xPos = Touch_ReadX();
    yPos = Touch_ReadY();
    
    xPos = xPos - 1080;
    if(xPos < 0) xPos = 0;
    yPos = yPos - 1360;
    if(yPos < 0) yPos = 0;
    
    // screen's been touched
    if(abs(xPos - untouch_coorx) >= 200 && abs(yPos - untouch_coory) >= 200){
      rotate = !rotate;
    } 
    if (rotate) {
      Erase_Cube();
      Rotate_X(30);
    }
    Draw_Cube();
  }
  
}

void LCD_Traffic(void) {
  int radius = 30;
  int starty = 20;
  
  LED_Init();
  Timer_Init(1);
  PLL_Init(16);
  LCD_Init();
  Touch_Init();
  TOUCH_ENABLE = true;
  Timer_Interrupt();
  //  Touch_BeginWaitForTouch();
  STATE = INIT;
  
  //set up background
  LCD_ColorFill(Color4[15]);
  LCD_SetTextColor(0,0,0);
  
  //draw Buttons
  LCD_DrawFilledCircle(centerx - 90, centery - 40, radius, ((0xFF>>3)<<11) | ((0x55>>2)<<5) | (0x55>>3));
  LCD_DrawCircle(centerx - 90, centery - 40, radius, 0);
  LCD_DrawFilledCircle(centerx + 90, centery - 40, radius, ((0x55>>3)<<11) | ((0x55>>2)<<5) | (0xFF>>3));
  LCD_DrawCircle(centerx + 90, centery - 40, radius, 0);
  
  //draw stoplight
  drawStoplight(STATE, radius, 60, 6*20+20, starty);
  
  while (1) {
    LCD_DrawFilledRect(0, 0, 25, 18, ((0xFF>>3)<<11) | ((0xFF>>2)<<5) | (0xFF>>3));
    LCD_SetCursor(0,0);
    LCD_PrintInteger(xPos);
    LCD_SetCursor(0,8);
    LCD_PrintInteger(yPos);
    
    xPos = Touch_ReadX();
    yPos = Touch_ReadY();
    
    xPos = xPos - 1080;
    if(xPos < 0) xPos = 0;
    yPos = yPos - 1360;
    if(yPos < 0) yPos = 0;
    
    // start/stop x: start_coorx +- 200, y: start_coory +- 200
    if(abs(xPos - start_coorx) <= 200 && abs(yPos - start_coory) <= 200 && STATE == INIT){
      if(TIMER_2 >= 2){
        STATE = STOP;
        TIMER_2 = 0;
      }
    } else if(abs(xPos - start_coorx) <= 200 && abs(yPos - start_coory) <= 200 && STATE != INIT){
      if(TIMER_2 >= 2){
        STATE = INIT;
        TIMER_2 = 0;
      }
      // passanger x: pass_coorx +- 200, y:  pass_coory +- 200
    } else if (abs(xPos - pass_coorx) <= 200 && abs(yPos - pass_coory) <= 200 && (STATE == GO || STATE == STOP)) {
      if (TIMER_2 >= 2) {
        TIMER_2 = 0;
        TIMER_5 = 0;
        STATE = PASS;
      }
    } else {
      TIMER_2 = 0;
    }
    
    if (STATE == INIT) {
      LED_Off_PortE();
      TIMER_5= 0;
      for(int i = 0; i < 10000; i++);
    } else if (STATE == GO) {
      drawStoplight(STATE, radius, 60, 6*20+20, starty);
      LED_On_PortE(PE1_GREEN);
      if (TIMER_5 >= 5) {
        STATE = STOP;
        TIMER_5 = 0;
      }
    } else if (STATE == STOP) { 
      drawStoplight(STATE, radius, 60, 6*20+20, starty);
      LED_On_PortE(PE3_RED);
      if (TIMER_5 >= 5) {
        STATE = GO;
        TIMER_5 = 0;
      }
    } else { // WARN STATE
      drawStoplight(STATE, radius, 60, 6*20+20, starty);
      LED_On_PortE(PE2_YELLOW);
      if (TIMER_5 >= 5) {
        STATE = STOP;
        TIMER_5 = 0;
      }
    }
  }
}


void LCD_Print_Temp(void) {
  PortF_Init();
  Timer0_Init(MHZ16);
  PLL_Init(16);
  UART_Init(16);
  ADC_Init();
  Interrupt_Init();
  
  LCD_Init(); // using port A and port B
  LCD_ColorFill(Color4[15]);
  LCD_SetTextColor(0, 0, 0);
  
  while (1) {
    if(FLAG == FLAG_ADC){
      LCD_ColorFill(Color4[15]);
      LCD_SetCursor(0,0);
      LCD_PrintInteger(result);
      FLAG = FLAG_NONE;
    }
    if (SW == SW1) {
      SW = 0;
      ADC0_IM &= ~0x8;
      PLL_Init(4);
      UART_Init(4);
      TIMER_VAL0 = MHZ4;  //set timer start to 4000000
      ADC0_IM |= 0x8;
    }
    if (SW == SW2) {
      SW = 0;
      ADC0_IM &= ~0x8;
      PLL_Init(80);
      UART_Init(80);
      TIMER_VAL0 = MHZ80;  //set timer start to 80000000
      ADC0_IM |= 0x8;
    }
  }
}


//switching if LCD not initialized
void Switching(void) {  
  PortF_Init();
  Timer0_Init(MHZ16);
  PLL_Init(16);
  UART_Init(16);
  ADC_Init();
  Interrupt_Init();
  LED_OFF();
  
  while (1) {
    //    if (result > 0 && result <= 17) {
    //      LED_ON(RED);
    //    } else if (result > 17 && result <= 19) {
    //      LED_ON(BLUE);
    //    } else if (result > 19 && result <= 21) {
    //      LED_ON(VIOLET);
    //    } else if (result > 21 && result <= 23) {
    //      LED_ON(GREEN);
    //    } else if (result > 23 && result <= 25) {
    //      LED_ON(YELLOW);
    //    } else if (result > 25 && result <= 27) {
    //      LED_ON(LIGHT_BLUE);
    //    } else if (result > 27 && result <= 40) {
    //      LED_ON(WHITE);
    //    } else {
    //      LED_OFF();
    //    }
    
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
    
    if (SW == SW1) {
      SW = 0;
      ADC0_IM &= ~0x8;
      PLL_Init(4);
      UART_Init(4);
      TIMER_VAL0 = MHZ4;  //set timer start to 4000000
      ADC0_IM |= 0x8;
      LED_ON(GREEN);
    }
    if (SW == SW2) {
      SW = 0;
      ADC0_IM &= ~0x8;
      PLL_Init(80);
      UART_Init(80);
      TIMER_VAL0 = MHZ80;  //set timer start to 80000000
      ADC0_IM |= 0x8;
      LED_ON(RED);
    }
  }
}

void Draw_Cube(void){
  for (int i = 0; i < 12; i++){
    Node n1 = nodes[edges[i].n1];
    Node n2 = nodes[edges[i].n2];
    LCD_DrawLine((short)(centerx + n1.x), (short)(centery + n1.y), (short)(centerx + n2.x), (short)(centery + n2.y), Color4[0]);
  }
}

void Draw_Filled_Cube(unsigned short color){
  for (int i = 0; i < 6; i++) {
    LCD_DrawFilledPara(planes[i], color);
  }
  for (int i = 0; i < 12; i++){
    Node n1 = nodes[edges[i].n1];
    Node n2 = nodes[edges[i].n2];
    LCD_DrawLine((short)(centerx + n1.x), (short)(centery + n1.y), (short)(centerx + n2.x), (short)(centery + n2.y), Color4[0]);
  }
}

void Erase_Cube(void){
  for (int i = 0; i < 12; i++){
    Node n1 = nodes[edges[i].n1];
    Node n2 = nodes[edges[i].n2];
    LCD_DrawLine((short)(centerx + n1.x), (short)(centery + n1.y), (short)(centerx + n2.x), (short)(centery + n2.y), Color4[15]);
  }
}

void Rotate_X(short theta) {
  double sinTheta = sin(theta);
  double cosTheta = cos(theta);
  for (int i = 0; i < 8; i++) {
    double y = nodes[i].y;
    double z = nodes[i].z;
    nodes[i].y = y * cosTheta - z * sinTheta;
    nodes[i].z = z * cosTheta + y * sinTheta;
  }
}

void Rotate_Y(short theta) {
  double sinTheta = sin(theta);
  double cosTheta = cos(theta);
  for (int i = 0; i < 8; i++) {
    double x = nodes[i].x;
    double z = nodes[i].z;
    nodes[i].x = x * cosTheta - z * sinTheta;
    nodes[i].z = z * cosTheta + x * sinTheta;
  }
}

void Rotate_Z(short theta) {
  double sinTheta = sin(theta);
  double cosTheta = cos(theta);
  for (int i = 0; i < 8; i++) {
    double y = nodes[i].y;
    double x = nodes[i].x;
    nodes[i].y = y * cosTheta - x * sinTheta;
    nodes[i].x = x * cosTheta + y * sinTheta;
  }
}

void drawStoplight(int state, int radius, int width, int height, int starty){
  radius = 20;
  switch(state){
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



// ************** LCD_DrawFilledRect **********************
// - Draws a filled rectangle, top left corner at (x,y)
// ********************************************************
void LCD_DrawFilledPara(Plane p, unsigned short color){
  //  nodes[p.n1].x;
  int index = Get_NodeZ();
  if (p.n1 != index && p.n2 != index && p.n3 != index && p.n4 != index) {
    double slope = findSlope(nodes[p.n1].x, nodes[p.n1].y, nodes[p.n2].x, nodes[p.n2].y);
    double x2 = nodes[p.n3].x;
    for (double x1 = nodes[p.n1].x; x1 < nodes[p.n2].x; x1++) {
      x2++;
      double y1 = findY(slope, nodes[p.n1].x, nodes[p.n1].y, x1);
      double y2 = findY(slope, nodes[p.n3].x, nodes[p.n3].y, x2);
      LCD_DrawLine(centerx + x1, centery + y1, centerx + x2, centery + y2, color);
    }
  }
}

int Get_NodeZ() {
  double z = 0;
  int result = 0;
  for (int i = 0; i < 12; i++) {
    if (z < nodes[i].z) {
      z = nodes[i].z;
      result = i;
    }
  }
  return result;
}


double findSlope(double x0, double y0, double x1, double y1){
  return (y1-y0)/(x1-x0);
}

double findY(double slope, double x0, double y0, double xPos){
  return slope*(xPos - x0) + y0;
}