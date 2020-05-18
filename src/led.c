 /// \file    led.c
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Built-in LEDs related functionalities
 ///
 /// Implements functions to initialise built-in LEDs, and switch them
  
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "led.h"

/* Global Variables ----------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
void ledsInit(void) {
	//Enable the clock for GPIO_E
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

	//Configure GPIO LED Pins in output mode
	u32 gpioe_moder = (	GPIO_MODER_MODER0_0 |
						GPIO_MODER_MODER1_0 |
						GPIO_MODER_MODER3_0 |
						GPIO_MODER_MODER4_0 );
	u32 gpio_moder_mask = (	GPIO_MODER_MODER0 |
							GPIO_MODER_MODER1 |
							GPIO_MODER_MODER3 |
							GPIO_MODER_MODER4 );
	GPIOE->MODER = (GPIOE->MODER & ~gpio_moder_mask) | gpioe_moder;
}

void ledOff(led_TypeDef led) {
	//Put corresponding pin in high state (negative logic)
	//								(bit BSy with y = led)
	//TODO: use atomic operations
	GPIOE->BSRRL |= ((u16)(0x1<<led));
}

void ledOn(led_TypeDef led) {
	//Put corresponding pin in low state (negative logic)
	//								(bit BRy with y = led)
	GPIOE->BSRRH |= ((u16)(0x1<<led));
}

void ledToggle(led_TypeDef led) {
	//Toggle corresponding pin (bit ODRy with y = led)
	GPIOE->ODR ^= ((u32)(0x1<<led));
}

void ledSetAll(uint8_t ledv1, uint8_t ledv2, uint8_t ledr1, uint8_t ledr2){
	if (ledv1)
		ledOn(LEDV1);
	else
		ledOff(LEDV1);
	if (ledv2)
		ledOn(LEDV2);
	else
		ledOff(LEDV2);
	if (ledr1)
		ledOn(LEDR1);
	else
		ledOff(LEDR1);
	if (ledr2)
		ledOn(LEDR2);
	else
		ledOff(LEDR2);
}

/**** END OF FILE ****/
