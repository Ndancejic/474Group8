//GPIO READ/WRITE MEMORY LOCATIONS
#define GPIO_WRITE_PORTF (*((unsigned int *)0x400253fc)) //port F GPIODATA
#define GPIO_READ_PF0_PF4 (*((unsigned int *)0x40025044)) //read location for PF0 and PF4

//GPIO INITIALIZATIONS
//PortA:
#define GPIO_EN2 (*((unsigned int *)0x400FE108)) //enable gpio portA
#define GPIO_ADIS_PORTA (*((unsigned int *)0x40004528)) //Port A analog disable
#define GPIO_PCTL_PORTA (*((unsigned int *)0x4000452C)) //Port A PCTL
#define GPIO_DIR_PORTA (*((unsigned int *)0x40004400)) //Port A direction
#define GPIO_REG_PORTA (*((unsigned int *)0x40004420)) //PA5 and PA6 regular port function
#define GPIO_DEN_PORTA (*((unsigned int *)0x4000451C)) //Port A direction
#define GPIO_DATA_PORTA (*((unsigned int *)0x400043FC)) //read port A

//PortF:
#define GPIO_EN (*((unsigned int *)0x400FE608)) //location for enabling GPIO
#define GPIO_DIR_PORTF (*((unsigned int *)0x40025400)) //GPIO direction Port F
#define GPIO_DEN_PORTF (*((unsigned int *)0x4002551C)) //digital enable Port F
#define GPIO_LOCK_PORTF (*((unsigned int *)0x40025520)) //unlocking GPIO Port F
#define GPIO_CR_PORTF (*((unsigned int *)0x40025524)) //commiting pins Port F
#define GPIO_PUR_PORTF (*((unsigned int *)0x40025510)) //enabling pull down resistors Port F

//TIMER INITIALIZATIONS MEMORY LOCATIONS
#define TIMER_EN (*((unsigned int *)0x400FE604)) // enabling timers
#define TIMER_EN1 (*((unsigned int *)0x400FE65C)) // enabling wide timers
#define TIMER_DIS0 (*((unsigned int *)0x4003000C)) //disabling timer 0
#define TIMER_DIS1 (*((unsigned int *)0x4003100C)) //disabling timer 1
#define TIMER_CON (*((unsigned int *)0x40030000)) //set timer configuration (32 or 16)
#define TIMER_CON1 (*((unsigned int *)0x40031000)) //set timer configuration (32 or 16)
#define TIMER_MODE (*((unsigned int *)0x40030004)) //configure TnMR field 
#define TIMER_MODE1 (*((unsigned int *)0x40031004)) //configure TnMR field 
#define TIMER_VAL (*((unsigned int *)0x40030028)) //Load the start value into the GPTM Timer n Interval Load Register
#define TIMER_VAL1 (*((unsigned int *)0x40031028)) //Load the start value into the GPTM Timer n Interval Load Register
#define TIMER_INT (*((unsigned int *)0x40030018)) //GPTM Interrupt Mask Register
#define TIMER_INT1 (*((unsigned int *)0x40031018)) //GPTM Interrupt Mask Register
#define TIMER_POLL (*((unsigned int *)0x4003001C)) //Poll or wait for interrupt
#define TIMER_POLL1 (*((unsigned int *)0x4003101C)) //Poll or wait for interrupt
#define TIMER_MASK (*((unsigned int *)0x40030020)) //Poll or wait for interrupt
#define TIMER_MASK1 (*((unsigned int *)0x40031020)) //Poll or wait for interrupt
#define TIMER_RESET (*((unsigned int *)0x40030024)) //reset flags
#define TIMER_RESET1 (*((unsigned int *)0x40031024)) //reset flags

//INTERRUPTS
//port F
#define GPIO_SENSE (*((unsigned int *)0x40025404)) //interrupt sense register
#define GPIO_IBE (*((unsigned int *)0x40025408)) //interrupt both edges
#define GPIO_IEV (*((unsigned int *)0x4002540C)) //interrupt event (rising or falling)
#define GPIO_IM (*((unsigned int *)0x40025410)) //im
#define GPIO_CLEAR (*((unsigned int *)0x4002541C))
#define GPIO_STATUS (*((unsigned int *)0x40025414)) //clear interrupt flags

//port A
#define GPIO_SENSE_A (*((unsigned int *)0x40004404)) //interrupt sense register
#define GPIO_IBE_A (*((unsigned int *)0x40004408)) //interrupt both edges
#define GPIO_IEV_A (*((unsigned int *)0x4000440C)) //interrupt event (rising or falling)
#define GPIO_IM_A (*((unsigned int *)0x40004410)) //im
#define GPIO_CLEAR_A (*((unsigned int *)0x4000441C))
#define GPIO_STATUS_A (*((unsigned int *)0x40004414)) //clear interrupt flags

#define EN0 (*((unsigned int *)0xE000E100)) //choose Interrupts
#define DIS0 (*((unsigned int *)0xE000E180)) //disable interrupts
#define PRI0 (*((unsigned int *)0xE000E400))  //priority of interrupt 16-19
#define PRI4 (*((unsigned int *)0xE000E410))  //priority of interrupt 16-19
#define PRI5 (*((unsigned int *)0xE000E414))  //priority of interrupt 20-23
#define PRI7 (*((unsigned int *)0xE000E41C))  //priority of interrupt 28-30
