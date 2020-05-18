 /// \file    encoder.c
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Implements encoder related functionalities such as position or
 /// speed calculation. \n \n


/* Includes ------------------------------------------------------------------*/
#include "encoder.h"

/* Global variables ----------------------------------------------------------*/
//!Total number of encoder count pre-overflow for rear steering wheel
volatile int8_t stRwOverflowCnt = 0;
//!Total number of encoder count pre-overflow for front steering wheel
volatile int8_t stFwOverflowCnt = 0;
//! The last motor speed computed by encoderCalculateSpeed()
static float motorsSpeed[2] = {0, 0};

/* Functions -----------------------------------------------------------------*/

static void encoderTimerInit(void){
		//TIM1
	//Enable APB1 clock for timer 1
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	//Set up TIM1 as slave mode for quadrature encoder interfacing
	TIM1->SMCR = (TIM1->SMCR & ~TIM_SMCR_SMS) |
			(TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
	//Activate interrupts handling by NVIC for TIM1_CC
	NVIC->ISER[0] |= 1<<27;
	//Activate CC3 interrupts
	TIM1->DIER |= TIM_DIER_CC3IE;
	//Enable CC4 interrupts
	TIM1->DIER |= TIM_DIER_CC4IE;
	//Set CC3 and CC4 thresholds
	TIM1->CCR3 = MIDDLE_16B + STEER_PREOF_CNT;
	TIM1->CCR4 = MIDDLE_16B - STEER_PREOF_CNT;
	//Enable TIM1 counter
	TIM1->CR1 |= TIM_CR1_CEN;

		//TIM2
	//Enable APB1 clock for timer 2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	//Set up TIM2 as slave mode for quadrature encoder interfacing
	TIM2->SMCR = (TIM2->SMCR & ~TIM_SMCR_SMS) |
			(TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
	//Enable TIM2 counter
	TIM2->CR1 |= TIM_CR1_CEN;

		//TIM3
	//Enable APB1 clock for timer 3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	//Set up TIM3 as slave mode for quadrature encoder interfacing
	TIM3->SMCR = (TIM3->SMCR & ~TIM_SMCR_SMS) |
			(TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
	//Activate interrupts handling by NVIC for TIM3
	NVIC->ISER[0] |= 1<<29;
	//Activate CC3 interrupts
	TIM3->DIER |= TIM_DIER_CC3IE;
	//Enable CC4 interrupts
	TIM3->DIER |= TIM_DIER_CC4IE;
	//Set CC3 and CC4 thresholds
	TIM3->CCR3 = MIDDLE_16B + STEER_PREOF_CNT;
	TIM3->CCR4 = MIDDLE_16B - STEER_PREOF_CNT;
	//Enable TIM3 counter
	TIM3->CR1 |= TIM_CR1_CEN;

		//TIM5
	//Enable APB1 clock for timer 5
	RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	//Set up TIM3 as slave mode for quadrature encoder interfacing
	TIM5->SMCR = (TIM5->SMCR & ~TIM_SMCR_SMS) |
			(TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1);
	//Enable TIM3 counter
	TIM5->CR1 |= TIM_CR1_CEN;
}

static void encoderGpioInit(void){

		//Port A
	//Enable clock for GPIO_A
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
		//PA15 (TIM2_CH1)
	//Choose TIM2 as an alternate function
	GPIOA->AFR[1] = (u32)((GPIOA->AFR[1] & ~(0b1111<<28)) | (0x1<<28));
	//Disable pull up and pull down
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR15_0 | GPIO_PUPDR_PUPDR15_1);

		//Port B
	//Enable clock fir GPIO_B
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
		//PB3 (TIM2_CH2)
	//Set pin as AF
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER3) | (GPIO_MODER_MODER3_1);
	//Choose TIM2 as an alternate function
	GPIOB->AFR[0] = (u32)((GPIOB->AFR[0] & ~(0b1111<<12)) | (0x1<<12));
	//Set GPIO speed as 2MHz
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR3);
		//PB4 (TIM3_CH1)
	//Set pin as AF
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER4) | (GPIO_MODER_MODER4_1);
	//Choose TIM3 as an alternate function
	GPIOB->AFR[0] = (u32)((GPIOB->AFR[0] & ~(0b1111<<16)) | (0x2<<16));
	//Set GPIO speed as 2MHz
	GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR4);

		//Port C
	//Enable clock for GPIO_C
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
		//PC7 (TIM3_CH2)
	//Set pin as AF
	GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER7) | (GPIO_MODER_MODER7_1);
	//Choose TIM3 as an alternate function
	GPIOC->AFR[0] = (u32)((GPIOC->AFR[0] & ~(0b1111<<28)) | (0x2<<28));

		//Port E
	//Enable clock for GPIO_E
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
		//PE9 (TIM1_CH1)
	//Set pin as AF
	GPIOE->MODER = (GPIOE->MODER & ~GPIO_MODER_MODER9) | (GPIO_MODER_MODER9_1);
	//Choose TIM1 as an alternate function
	GPIOE->AFR[1] = (u32)((GPIOE->AFR[1] & ~(0b1111<<4)) | (0x1<<4));
		//PE11 (TIM1_CH2)
	//Set pin as AF
	GPIOE->MODER = (GPIOE->MODER & ~GPIO_MODER_MODER11) | (GPIO_MODER_MODER11_1);
	//Choose TIM1 as an alternate function
	GPIOE->AFR[1] = (u32)((GPIOE->AFR[1] & ~(0b1111<<12)) | (0x1<<12));

		//Port H
	//Enable clock for GPIO_H
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
		//PH10 (TIM5_CH1)
	//Set pin as AF
	GPIOH->MODER = (GPIOH->MODER & ~GPIO_MODER_MODER10) | (GPIO_MODER_MODER10_1);
	//Choose TIM1 as an alternate function
	GPIOH->AFR[1] = (u32)((GPIOH->AFR[1] & ~(0b1111<<8)) | (0x2<<8));
		//PH11 (TIM5_CH2)
	//Set pin as AF
	GPIOH->MODER = (GPIOH->MODER & ~GPIO_MODER_MODER11) | (GPIO_MODER_MODER11_1);
	//Choose TIM1 as an alternate function
	GPIOH->AFR[1] = (u32)((GPIOH->AFR[1] & ~(0b1111<<12)) | (0x2<<12));
}

void encoderInit(void){
	encoderTimerInit();
	encoderGpioInit();
	encoderResetCount(ENC_ST_FW);
	encoderResetCount(ENC_DR_FW);
	encoderResetCount(ENC_ST_RW);
	encoderResetCount(ENC_DR_RW);
}

void encoderResetCount(encoder_Typedef enc){
	if (enc == ENC_ST_FW)
		TIM1->CNT = MIDDLE_16B;
	else if (enc == ENC_DR_FW)
		TIM2->CNT = MIDDLE_32B;
	else if (enc == ENC_ST_RW)
		TIM3->CNT = MIDDLE_16B;
	else if (enc == ENC_DR_RW)
		TIM5->CNT = MIDDLE_32B;
}

float encoderGetAngle(encoder_Typedef enc){
	int32_t diffCnt = 0;
	float angle = 0;

	if (enc == ENC_ST_FW){
		//Take into account the total number of counter pre-overflow
		diffCnt = TIM1->CNT - MIDDLE_16B;
		angle = (float)diffCnt * STEER_DEG_PER_CNT +
				stFwOverflowCnt * STEER_PREOF_ANGLE;
	}
	else if (enc == ENC_DR_FW){
		diffCnt = TIM2->CNT - MIDDLE_32B;
		angle = (float)diffCnt * DRIVING_DEG_PER_CNT;
	}
	else if (enc == ENC_ST_RW){

		//Deactivate CC3 and CC4 interrupts to avoid problems
		//with non atomic stOverflowCounter access
		TIM3->DIER &= ~TIM_DIER_CC3IE;
		TIM3->DIER &= ~TIM_DIER_CC4IE;
		//Take into account the total number of counter pre-overflow
		diffCnt = TIM3->CNT - MIDDLE_16B;
		angle = (float)diffCnt * STEER_DEG_PER_CNT +
				stRwOverflowCnt * STEER_PREOF_ANGLE;
		//Reactivate CC3/CC4 interrupts
		TIM3->DIER |= TIM_DIER_CC3IE;
		TIM3->DIER |= TIM_DIER_CC4IE;
	}
	else if (enc == ENC_DR_RW){
		diffCnt = TIM5->CNT - MIDDLE_32B;
		angle = (float)diffCnt * DRIVING_DEG_PER_CNT;
	}

	return angle;
}

void encoderCalculateSpeed(encoder_Typedef enc){
	//Exit if something else than a driving motor is given
	if (enc != ENC_DR_FW && enc != ENC_DR_RW)
		return;

	static uint32_t prevTime[2] = {0, 0};
	//Exit if 2 calls have been made too close from each others in time
	if (getTime_us()-prevTime[enc-2] < SPEEDCALC_MIN_TIME_BETWEEN_US)
		return;

	static float prevAngle[2] = {0, 0};
	uint32_t curTime = getTime_us();
	float curAngle = encoderGetAngle(enc);
		//degrees/second
	float speed = 1000000.0*(curAngle - prevAngle[enc-2])/
			(curTime - prevTime[enc-2]);
	prevTime[enc-2] = curTime;
	prevAngle[enc-2] = curAngle;

	motorsSpeed[enc-2] = speed;
}

float encoderGetSpeed(encoder_Typedef enc){

	if (enc != ENC_DR_FW && enc != ENC_DR_RW)
		return 0;

	encoderCalculateSpeed(enc);

	return motorsSpeed[enc-2];
}

void TIM3_IRQHandler(void){
	//Pre-overflow handling
	if ((TIM3->SR & TIM_SR_CC3IF) != 0){
		TIM3->SR &= ~TIM_SR_CC3IF;
		//Reset counter and update count of total overflows
		TIM3->CNT = MIDDLE_16B;
		stRwOverflowCnt++;
	}
	//Pre-underflow handling
	if ((TIM3->SR & TIM_SR_CC4IF)){
		TIM3->SR &= ~TIM_SR_CC4IF;
		//Reset counter and update count of total overflows
		TIM3->CNT = MIDDLE_16B;
		stRwOverflowCnt--;
	}
}

void TIM1_CC_IRQHandler(void){
	//Pre-overflow handling
	if ((TIM1->SR & TIM_SR_CC3IF) != 0){
		TIM1->SR &= ~TIM_SR_CC3IF;
		//Reset counter and update count of total overflows
		TIM1->CNT = MIDDLE_16B;
		stFwOverflowCnt++;
	}
	//Pre-underflow handling
	if ((TIM1->SR & TIM_SR_CC4IF)){
		TIM1->SR &= ~TIM_SR_CC4IF;
		//Reset counter and update count of total overflows
		TIM1->CNT = MIDDLE_16B;
		stFwOverflowCnt--;
	}
}

/**** END OF FILE ****/
