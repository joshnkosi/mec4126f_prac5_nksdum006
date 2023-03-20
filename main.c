#define STM32F051
#include "stm32f0xx.h"
#include "lcd_stm32f0.h"
#include <stdio.h>
#include <stdint.h>

// global variables
uint8_t sw_count;

//function declarations
void display_on_LCD(uint8_t number);
void init_LEDs(void);
void display_on_LEDs(uint8_t n_leds);
void init_switches(void);
void init_external_interrupts(void);

void main(void)
{
  uint8_t count;
  count = 3;
  sw_count = 5;

  init_LCD();
  init_LEDs();
  init_switches();
  display_on_LCD(count);

  //Infinite loop
  while (1)
    {
	  display_on_LCD(count);
	  display_on_LEDs(count);
	  if (sw_count%2 == 1)
	  {
		  if (( GPIOA -> IDR & GPIO_IDR_1)==0)       //checks if sw1 is pressed then increments count by 1
		  {
			  if (count < 255)
				  count++;
		  }
		  else if ((GPIOA -> IDR & GPIO_IDR_2)==0)   //checks if sw2 is pressed then decrements count by 1
		  {
			  if (count > 0)
				  count--;
		  }
		  delay(100000);
	  }
	  else
	  {
		  display_on_LCD(0);
		  display_on_LEDs(0);
	  }
    }
}

//function definitions

void display_on_LCD(uint8_t number)
{
	char num_string[3];
	lcd_command(CLEAR);
	sprintf(num_string,"%d",number);  //converts number into a string
	lcd_putstring(num_string);     //displays string on the LCD
}

void init_LEDs(void)
{
	RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;    //enables clock
	//sets ports PB0 to PB7 to outputs
	GPIOB -> MODER |= GPIO_MODER_MODER0_0|
			          GPIO_MODER_MODER1_0|
			          GPIO_MODER_MODER2_0|
			          GPIO_MODER_MODER3_0|
			          GPIO_MODER_MODER4_0|
			          GPIO_MODER_MODER5_0|
			          GPIO_MODER_MODER6_0|
					  GPIO_MODER_MODER7_0;
}

void display_on_LEDs(uint8_t n_LEDs)
{
	GPIOB -> ODR &= ~n_LEDs;	 //clears output data register
	GPIOB -> ODR = n_LEDs;	 //displays n_LEDs on s as binary
}

void init_switches(void)
{
	RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
	GPIOA -> MODER &= ~(GPIO_MODER_MODER1|GPIO_MODER_MODER2|GPIO_MODER_MODER3);       //initializes PA0 and PA1 to input
	GPIOA -> PUPDR |= (GPIO_PUPDR_PUPDR1_0|
			           GPIO_PUPDR_PUPDR2_0|
					   GPIO_PUPDR_PUPDR3_0);	// enables pull up resistors to PA0 and PA1
}
void init_external_interrupts(void)
{
	RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI3_PA;
	EXTI -> IMR |= EXTI_IMR_MR3;
	EXTI -> FTSR|= EXTI_FTSR_TR3;

	NVIC_EnableIRQ(EXTI2_3_IRQn);
}

void EXTI2_3_IRQHandler(void)
{
	sw_count +=1;
	EXTI->PR |= EXTI_PR_PR3;
}

