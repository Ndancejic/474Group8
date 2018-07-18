#ifndef EE474_H
#define EE474_H

//GPIO READ/WRITE MEMORY LOCATIONS
#define GPIO_WRITE_PORTF (*((unsigned int *)0x400253fc)) //port F GPIODATA
#define GPIO_READ_PF0_PF4 (*((unsigned int *)0x40025044)) //read location for PF0 and PF4

//GPIO INITIALIZATIONS
#define RCGCGPIO (*((unsigned int *)0x400FE608)) //location for enabling GPIO
//PortA:
#define GPIO_EN_A (*((unsigned int *)0x400FE108)) //enable gpio portA ?
#define GPIO_ADIS_PORTA (*((unsigned int *)0x40004528)) //Port A analog disable
#define GPIO_PCTL_PORTA (*((unsigned int *)0x4000452C)) //Port A PCTL
#define GPIO_DIR_PORTA (*((unsigned int *)0x40004400)) //Port A direction
#define GPIO_AFSEL_PORTA (*((unsigned int *)0x40004420)) //PA5 and PA6 regular port function
#define GPIO_DEN_PORTA (*((unsigned int *)0x4000451C)) //Port A direction
#define GPIO_DATA_PORTA (*((unsigned int *)0x400043FC)) //read port A
#define GPIO_DR2R_PORTA (*((unsigned int *)0x40004500)) //Port A drive control 2mA
#define GPIO_DR4R_PORTA (*((unsigned int *)0x40004504)) //Port A drive control 4mA
#define GPIO_DR8R_PORTA (*((unsigned int *)0x40004508)) //Port A drive control 8mA
#define GPIO_SLR_PORTA (*((unsigned int *)0x40004518)) //Port A slew rate

//PortE:
#define GPIO_DIR_PORTE (*((unsigned int *)0x40024400)) //GPIO direction Port F
#define GPIO_DEN_PORTE (*((unsigned int *)0x4002451C)) //digital enable Port F
#define GPIO_LOCK_PORTE (*((unsigned int *)0x40024520)) //unlocking GPIO Port F
#define GPIO_CR_PORTE (*((unsigned int *)0x40024524)) //commiting pins Port F
#define GPIO_PUR_PORTE (*((unsigned int *)0x40024510)) //enabling pull down resistors Port F
#define GPIO_REG_PORTE (*((unsigned int *)0x40024420)) //Alternate Function
#define GPIO_AMSEL_PORTE (*((unsigned int *)0x40024528)) //disable isolation

//PortF:
#define GPIO_DIR_PORTF (*((unsigned int *)0x40025400)) //GPIO direction Port F
#define GPIO_DEN_PORTF (*((unsigned int *)0x4002551C)) //digital enable Port F
#define GPIO_LOCK_PORTF (*((unsigned int *)0x40025520)) //unlocking GPIO Port F
#define GPIO_CR_PORTF (*((unsigned int *)0x40025524)) //commiting pins Port F
#define GPIO_PUR_PORTF (*((unsigned int *)0x40025510)) //enabling pull down resistors Port F
#define GPIO_ICR_PORTF (*((unsigned long *)0x4002541C)) //clear the interrupt flag Port F
//#define GPIO_REG_PORTA (*((unsigned int *)0x40025420)) //Alternate Function

//TIMER INITIALIZATIONS MEMORY LOCATIONS
//timer 0
#define TIMER_EN (*((unsigned int *)0x400FE604)) // enabling timers
#define TIMER_DIS0 (*((unsigned int *)0x4003000C)) //disabling timer 0
#define TIMER_CON0 (*((unsigned int *)0x40030000)) //set timer configuration (32 or 16)
#define TIMER_MODE0 (*((unsigned int *)0x40030004)) //configure TnMR field
#define TIMER_VAL0 (*((unsigned int *)0x40030028)) //Load the start value into the GPTM Timer n Interval Load Register
#define TIMER_INT0 (*((unsigned int *)0x40030018)) //GPTM Interrupt Mask Register
#define TIMER_POLL0 (*((unsigned int *)0x4003001C)) //Poll or wait for interrupt
#define TIMER_MASK0 (*((unsigned int *)0x40030020)) //Poll or wait for interrupt
#define TIMER_RESET0 (*((unsigned int *)0x40030024)) //reset flags

//timer 1
#define TIMER_EN1 (*((unsigned int *)0x400FE65C)) // enabling wide timers
#define TIMER_DIS1 (*((unsigned int *)0x4003100C)) //disabling timer 1
#define TIMER_CON1 (*((unsigned int *)0x40031000)) //set timer configuration (32 or 16)
#define TIMER_MODE1 (*((unsigned int *)0x40031004)) //configure TnMR field
#define TIMER_VAL1 (*((unsigned int *)0x40031028)) //Load the start value into the GPTM Timer n Interval Load Register
#define TIMER_INT1 (*((unsigned int *)0x40031018)) //GPTM Interrupt Mask Register
#define TIMER_POLL1 (*((unsigned int *)0x4003101C)) //Poll or wait for interrupt
#define TIMER_MASK1 (*((unsigned int *)0x40031020)) //Poll or wait for interrupt
#define TIMER_RESET1 (*((unsigned int *)0x40031024)) //reset flags

//INTERRUPTS
//port F
#define GPIO_SENSE_F (*((unsigned int *)0x40025404)) //interrupt sense register
#define GPIO_IBE_F (*((unsigned int *)0x40025408)) //interrupt both edges
#define GPIO_IEV_F (*((unsigned int *)0x4002540C)) //interrupt event (rising or falling)
#define GPIO_IM_F (*((unsigned int *)0x40025410)) //im
#define GPIO_CLEAR_F (*((unsigned int *)0x4002541C))
#define GPIO_STATUS_F (*((unsigned int *)0x40025414)) //clear interrupt flags

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

//ADC
#define ADC_CLK_EN (*((unsigned int *)0x400FE638)) //ADC Clock enable
#define ADC_RCGC0 (*((unsigned int *)0x400FE100)) //ADC Clock enable

//sequencer
#define ADC0_SSPRI (*((unsigned int *)0x400FE020)) //sequencer priority
#define ADC0_ACTSS (*((unsigned int *)0x40038000)) //enable sequencer
#define ADC0_EMUX (*((unsigned int *)0x40038014)) //enable sequencer
#define ADC0_SSMUX3 (*((unsigned int *)0x400380A0)) //enable sequencer
#define ADC0_SSCTL3 (*((unsigned int *)0x400380A4)) //enable sequencer
#define ADC0_IM (*((unsigned int *)0x40038008)) //enable INTERRUPTS
#define ADC0_ISC (*((unsigned int *)0x4003800C)) //interrupt status and clearing
#define ADC0_SSFIFO3 (*((unsigned int *)0x400380A8))
#define ADC0_RIS (*((unsigned int *)0x40038004))
#define ADC0_PSSI (*((unsigned int *)0x40038028))
#define ADC0_SSFIFO3 (*((unsigned int *)0x400380A8)) //Conversion Result Data, ADC code
#define ADC0_DCISC (*((unsigned int *)0x40038034)) //Conversion Result Data, ADC code

//board Clock
#define CLK_RCC (*((unsigned int *)0x400FE060)) // Run-Mode Clock Configuration
#define CLK_RCC2 (*((unsigned int *)0x400FE070)) // Run-Mode Clock Configuration 2
#define RIS (*((unsigned int *)0x400FE050))// Raw Interrupt Status

//UART
#define RCGCUART (*((unsigned int *)0x400FE618))// Raw Interrupt Status
#define UART0_CTL (*((unsigned int *)0x4000C030))// Raw Interrupt Status
#define UART0_IBRD (*((unsigned int *)0x4000C024))// Integer value of baud rate
#define UART0_FBRD (*((unsigned int *)0x4000C028))// decimal value of baud rate
#define UART0_LCRH (*((unsigned int *)0x4000C02C))// line control register
#define UART0_CC (*((unsigned int *)0x4000CFC8))// line control register
#define UART0_FR (*((unsigned int *)0x4000C018))//flag reister
#define UART0_DR (*((unsigned int *)0x4000C000))//data read

#endif
