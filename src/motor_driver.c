 /// \file    motor_driver.c
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Motor control related functions
 ///
 /// Implements functions related to motor control such as initialisation
 /// and command application. Note that this file does not implements
 /// automatic control. \n \n


/* Includes ------------------------------------------------------------------*/
#include "motor_driver.h"

/* Global variables ----------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
static void motorPwmInit(void){
			//TIM4
		//Set the time base of TIM4
	//Enable APB1 clock for timer 4
	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	//Set the prescaler
	TIM4->PSC = (u16)(MOTOR_PWM_PSC - 1);
	//Set the Auto Reload Register
	TIM4->ARR = (u16)(MOTOR_PWM_ARR_1 - 1);
		//Set channels
	//Set Channel 1 in PWM 2 mode
	TIM4->CCMR1 = (TIM4->CCMR1 & ~TIM_CCMR1_OC1M) |
			(TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0);
	//Reset capture and compare register 1
	TIM4->CCR1 = 0;
	//Set Channel 2 in PWM 2 mode
	TIM4->CCMR1 = (TIM4->CCMR1 & ~TIM_CCMR1_OC2M) |
			(TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0);
	//Reset capture and compare register 2
	TIM4->CCR2 = 0;
	//Set Channel 3 in PWM 2 mode
	TIM4->CCMR2 = (TIM4->CCMR2 & ~TIM_CCMR2_OC3M) |
			(TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_0);
	//Reset capture and compare register 3
	TIM4->CCR3 = 0;
	//Set Channel 4 in PWM 2 mode
	TIM4->CCMR2 = (TIM4->CCMR2 & ~TIM_CCMR2_OC4M) |
			(TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_0);
	//Reset capture and compare register 4
	TIM4->CCR4 = 0;
		//Apply settings
	//Update settings registers
	TIM4->EGR |= TIM_EGR_UG;
	//Enable Output compare on channel 1
	TIM4->CCER |= TIM_CCER_CC1E;
	//Enable Output compare on channel 2
	TIM4->CCER |= TIM_CCER_CC2E;
	//Enable Output compare on channel 3
	TIM4->CCER |= TIM_CCER_CC3E;
	//Enable Output compare on channel 4
	TIM4->CCER |= TIM_CCER_CC4E;
	//Enable the counter
	TIM4->CR1 |= TIM_CR1_CEN;

			//TIM8
		//Time base of TIM8
	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
	//Set the prescaler
	TIM8->PSC = (u16)(MOTOR_PWM_PSC - 1);
	//Set the Auto Reload Register
	TIM8->ARR = (u16)(MOTOR_PWM_ARR_2 - 1);
		//Set channels
	//Set Channel 1 in PWM 2 mode
	TIM8->CCMR1 = (TIM8->CCMR1 & ~TIM_CCMR1_OC1M) |
			(TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0);
	//Reset capture and compare register 1
	TIM8->CCR1 = 0;
	//Set Channel 2 in PWM 2 mode
	TIM8->CCMR1 = (TIM8->CCMR1 & ~TIM_CCMR1_OC2M) |
			(TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0);
	//Reset capture and compare register 2
	TIM8->CCR2 = 0;
	//Set Channel 3 in PWM 2 mode
	TIM8->CCMR2 = (TIM8->CCMR2 & ~TIM_CCMR2_OC3M) |
			(TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_0);
	//Reset capture and compare register 3
	TIM8->CCR3 = 0;
	//Set Channel 4 in PWM 2 mode
	TIM8->CCMR2 = (TIM8->CCMR2 & ~TIM_CCMR2_OC4M) |
			(TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_0);
	//Reset capture and compare register 4
	TIM8->CCR4 = 0;
		//Apply settings
	//Update settings registers
	TIM8->EGR |= TIM_EGR_UG;
	//Enable Output compare on channel 1
	TIM8->CCER |= TIM_CCER_CC1E;
	//Enable Output compare on channel 2
	TIM8->CCER |= TIM_CCER_CC2E;
	//Enable Output compare on channel 3
	TIM8->CCER |= TIM_CCER_CC3E;
	//Enable Output compare on channel 4
	TIM8->CCER |= TIM_CCER_CC4E;
	//Enable counter main output
	TIM8->BDTR |= TIM_BDTR_MOE;
	//Enable the counter
	TIM8->CR1 |= TIM_CR1_CEN;


}

static void motorGpioInit(void){
		//Port D
	//Enable the clock for GPIO_D
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
		//Port D12 (TIM4_CH1)
	//Configure pin in AF mode (Alternate function)
	GPIOD->MODER = (GPIOD->MODER & ~GPIO_MODER_MODER12) | GPIO_MODER_MODER12_1;
	//Select AF2 as an AF
	GPIOD->AFR[1] = (u32)((GPIOD->AFR[1] & ~(0b1111<<16)) | (0x2<<16));
		//Port D13 (TIM4_CH2)
	//Configure pin in AF mode (Alternate function)
	GPIOD->MODER = (GPIOD->MODER & ~GPIO_MODER_MODER13) | GPIO_MODER_MODER13_1;
	//Select AF2 as an AF
	GPIOD->AFR[1] = (u32)((GPIOD->AFR[1] & ~(0b1111<<20)) | (0x2<<20));
		//Port D14 (TIM4_CH3)
	//Configure pin in AF mode (Alternate function)
	GPIOD->MODER = (GPIOD->MODER & ~GPIO_MODER_MODER14) | GPIO_MODER_MODER14_1;
	//Select AF2 as an AF
	GPIOD->AFR[1] = (u32)((GPIOD->AFR[1] & ~(0b1111<<24)) | (0x2<<24));
		//Port D15 (TIM4_CH4)
	//Configure pin in AF mode (Alternate function)
	GPIOD->MODER = (GPIOD->MODER & ~GPIO_MODER_MODER15) | GPIO_MODER_MODER15_1;
	//Select AF2 as an AF
	GPIOD->AFR[1] = (u32)((GPIOD->AFR[1] & ~(0b1111<<28)) | (0x2<<28));


		//Port I
	//Enable the clock for GPIO_I
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;
		//Port I5 (TIM8_CH1)
	//Configure pin in AF mode (Alternate Function)
	GPIOI->MODER = (GPIOI->MODER & ~GPIO_MODER_MODER5) | GPIO_MODER_MODER5_1;
	//Select AF2 as an AF
	GPIOI->AFR[0] = (u32)((GPIOI->AFR[0] & ~(0b1111<<20)) | (0b11<<20));
		//Port I6 (TIM8_CH2)
	//Configure pin in AF mode (Alternate Function)
	GPIOI->MODER = (GPIOI->MODER & ~GPIO_MODER_MODER6) | GPIO_MODER_MODER6_1;
	//Select AF2 as an AF
	GPIOI->AFR[0] = (u32)((GPIOI->AFR[0] & ~(0b1111<<24)) | (0b11<<24));
		//Port I7 (TIM8_CH3)
	//Configure pin in AF mode (Alternate Function)
	GPIOI->MODER = (GPIOI->MODER & ~GPIO_MODER_MODER7) | GPIO_MODER_MODER7_1;
	//Select AF2 as an AF
	GPIOI->AFR[0] = (u32)((GPIOI->AFR[0] & ~(0b1111<<28)) | (0b11<<28));
		//Port I2 (TIM8_CH4)
	//Configure pin in AF mode (Alternate Function)
	GPIOI->MODER = (GPIOI->MODER & ~GPIO_MODER_MODER2) | GPIO_MODER_MODER2_1;
	//Select AF2 as an AF
	GPIOI->AFR[0] = (u32)((GPIOI->AFR[0] & ~(0b1111<<8)) | (0b11<<8));
}

void motorInit(void){
	motorGpioInit();
	motorPwmInit();
}

void setMotorCommand(motor_Typedef motor, float command){
	//Front Wheel STeering MOTor
	if (motor == MOT_ST_FW){
		//CCW mode
		if (command < -EPS){
			//force TIM8_CH2 to high
			TIM8->CCMR1 = (TIM8->CCMR1 & ~TIM_CCMR1_OC2M) |
								(TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0);
			//set comparison reference for TIM8_CH1
			TIM8->CCR1 = (u16)(-command * MOTOR_PWM_ARR_2);
			//put TIM8_CH1 in PWM2 mode
			TIM8->CCMR1 = (TIM8->CCMR1 & ~TIM_CCMR1_OC1M) |
						(TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0);
		}
		//CW mode
		else if (command > EPS){
			//force TIM8_CH1 to high
			TIM8->CCMR1 = (TIM8->CCMR1 & ~TIM_CCMR1_OC1M) |
								(TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0);
			//set comparison reference for TIM8_CH2
			TIM8->CCR2 = (u16)(command * MOTOR_PWM_ARR_2);
			//put TIM8_CH2 in PWM2 mode
			TIM8->CCMR1 = (TIM8->CCMR1 & ~TIM_CCMR1_OC2M) |
						(TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0);
		}
		//break mode
		else{
			//force TIM8_CH1 to high
			TIM8->CCMR1 = (TIM8->CCMR1 & ~TIM_CCMR1_OC1M) |
								(TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0);
			//force TIM8_CH2 to high
			TIM8->CCMR1 = (TIM8->CCMR1 & ~TIM_CCMR1_OC2M) |
								(TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0);
		}
	}
	//Rear Wheel STeering MOTor
	else if (motor == MOT_ST_RW){
		//CCW mode
		if (command < -EPS){
			//force TIM8_CH4 to high
			TIM8->CCMR2 = (TIM8->CCMR2 & ~TIM_CCMR2_OC4M) |
								(TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_0);
			//set comparison reference for TIM8_CH3
			TIM8->CCR3 = (u16)(-command * MOTOR_PWM_ARR_2);
			//put TIM8_CH3 in PWM2 mode
			TIM8->CCMR2 = (TIM8->CCMR2 & ~TIM_CCMR2_OC3M) |
						(TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_0);
		}
		//CW mode
		else if (command > EPS){
			//force TIM8_CH3 to high
			TIM8->CCMR2 = (TIM8->CCMR2 & ~TIM_CCMR2_OC3M) |
								(TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_0);
			//set comparison reference for TIM8_CH4
			TIM8->CCR4 = (u16)(command * MOTOR_PWM_ARR_2);
			//put TIM8_CH4 in PWM2 mode
			TIM8->CCMR2 = (TIM8->CCMR2 & ~TIM_CCMR2_OC4M) |
						(TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_0);
		}
		//break mode
		else{
			//force TIM8_CH3 to high
			TIM8->CCMR2 = (TIM8->CCMR2 & ~TIM_CCMR2_OC3M) |
								(TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_0);
			//force TIM8_CH4 to high
			TIM8->CCMR2 = (TIM8->CCMR2 & ~TIM_CCMR2_OC4M) |
								(TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_0);
		}
	}
	//Front Wheel DRiving MOTor
	else if (motor == MOT_DR_FW){
		//CCW mode
		if (command < -EPS){
			//force TIM4_CH2 to high
			TIM4->CCMR1 = (TIM4->CCMR1 & ~TIM_CCMR1_OC2M) |
								(TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0);
			//set comparison reference for TIM4_CH1
			TIM4->CCR1 = (u16)(-command * MOTOR_PWM_ARR_1);
			//put TIM4_CH1 in PWM2 mode
			TIM4->CCMR1 = (TIM4->CCMR1 & ~TIM_CCMR1_OC1M) |
						(TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0);
		}
		//CW mode
		else if (command > EPS){
			//force TIM4_CH1 to high
			TIM4->CCMR1 = (TIM4->CCMR1 & ~TIM_CCMR1_OC1M) |
								(TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0);
			//set comparison reference for TIM4_CH2
			TIM4->CCR2 = (u16)(command * MOTOR_PWM_ARR_1);
			//put TIM4_CH2 in PWM2 mode
			TIM4->CCMR1 = (TIM4->CCMR1 & ~TIM_CCMR1_OC2M) |
						(TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0);
		}
		//break mode
		else{
			//force TIM4_CH1 to high
			TIM4->CCMR1 = (TIM4->CCMR1 & ~TIM_CCMR1_OC1M) |
								(TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0);
			//force TIM4_CH2 to high
			TIM4->CCMR1 = (TIM4->CCMR1 & ~TIM_CCMR1_OC2M) |
								(TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0);
		}
	}
	//Rear Wheel DRiving MOTor
	else if (motor == MOT_DR_RW){
		//CCW mode
		if (command < -EPS){
			//force TIM4_CH4 to high
			TIM4->CCMR2 = (TIM4->CCMR2 & ~TIM_CCMR2_OC4M) |
								(TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_0);
			//set comparison reference for TIM4_CH3
			TIM4->CCR3 = (u16)(-command * MOTOR_PWM_ARR_1);
			//put TIM4_CH3 in PWM2 mode
			TIM4->CCMR2 = (TIM4->CCMR2 & ~TIM_CCMR2_OC3M) |
						(TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_0);
		}
		//CW mode
		else if (command > EPS){
			//force TIM4_CH3 to high
			TIM4->CCMR2 = (TIM4->CCMR2 & ~TIM_CCMR2_OC3M) |
								(TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_0);
			//set comparison reference for TIM4_CH4
			TIM4->CCR4 = (u16)(command * MOTOR_PWM_ARR_1);
			//put TIM4_CH4 in PWM2 mode
			TIM4->CCMR2 = (TIM4->CCMR2 & ~TIM_CCMR2_OC4M) |
						(TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_0);
		}
		//break mode
		else{
			//force TIM4_CH3 to high
			TIM4->CCMR2 = (TIM4->CCMR2 & ~TIM_CCMR2_OC3M) |
								(TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_0);
			//force TIM4_CH4 to high
			TIM4->CCMR2 = (TIM4->CCMR2 & ~TIM_CCMR2_OC4M) |
								(TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_0);
		}
	}
}

/**** END OF FILE ****/
