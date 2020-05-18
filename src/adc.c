 /// \file    adc.c
 /// \author  Alan ALLART
 /// \version 
 /// \brief   


/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* Global variables ----------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
void adcInit (void){
	adcInitADC();
	adcInitGPIO();
}

static void adcInitADC(void){
	//Enable Peripheral clock for the ADC3
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
    //Turn on the ADC3
    ADC3->CR2 |= ADC_CR2_ADON;
    //Change the sampling time to 480 clock cycles
    ADC3->SMPR2 |= ADC_SMPR2_SMP9;
}

static void adcInitGPIO(void){
		//Port F
	//Enable the clock for GPIO_F
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
	//Put PF pins in analog input mode
    GPIOF->MODER |= (GPIO_MODER_MODER3 | //PF3 <-> ADC3_IN9
					 GPIO_MODER_MODER4 | //PF4 <-> ADC3_IN14
					 GPIO_MODER_MODER5 | //PF5 <-> ADC3_IN15
					 GPIO_MODER_MODER6 | //PF6 <-> ADC3_IN4
					 GPIO_MODER_MODER7 ); //PF7 <-> ADC3_IN5
}

uint16_t adc3ReadChannel(uint8_t channel){
	ADC3->SQR3 = (ADC3->SQR3 & ~ADC_SQR3_SQ1) | channel;
	ADC3->CR2 |= ADC_CR2_SWSTART;
	while (!((ADC3->SR & ADC_SR_EOC) != 0));
	return ADC3->DR;
}

float adcConvertToTorqueDriving(uint16_t measure){
	return (measure-ADC_TORQUE_RESET_COUNT)*ADC_COUNT_TO_TORQUE_DRIVING;
}

float adcConvertToAngle(uint16_t measure){
	return (measure - ADC_POT_RESET_COUNT)*ADC_COUNT_TO_ANGLE_POT;
}

/**** END OF FILE ****/
