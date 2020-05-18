 /// \file    time.c
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Basic time functionalities
 ///
 /// Implements sysTick clock initialisation, delaying and time_getting
 /// functions


/* Includes ------------------------------------------------------------------*/
#include "time.h"

/* Global Variables ----------------------------------------------------------*/
volatile uint32_t localTime = 0; /* time since startup in us. Overflow each 1h10 */

/* Functions -----------------------------------------------------------------*/
void initSysTick(void){
	SysTick_Config(HCLK_FREQUENCY / TICK_FREQ);
	//SysTick calls sysTick_handler routine at tickFreq Hz
}

void timeAutoFunctionsInit(void){
		//Periodic Action 1 related settings
	//Enable clock for TIM6
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	//Activate interrupts for TIM6 and DAC (for which we don't care)
	NVIC->ISER[1] |= 1<<22;
	//Set prescaler and auto reload register
	TIM6->ARR = PERIODIC1_ARR-1;
	TIM6->PSC = PERIODIC1_PSC-1;
	//Enable the interrupt generation at overflow
	TIM6->DIER |= TIM_DIER_UIE;
	//Enable the TIMER
	TIM6->CR1 |= TIM_CR1_CEN;

		//Periodic action 2 related settings
	//Enable clock for TIM7
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
	//Activate interrupts for TIM7
	NVIC->ISER[1] |= 1<<23;
	//Set prescaler and auto reload register for TIM7
	TIM7->ARR = PERIODIC2_ARR-1;
	TIM7->PSC = PERIODIC2_PSC-1;
	//Enable the interrupt generation at overflows
	TIM7->DIER |= TIM_DIER_UIE;
	//Enable TIMER 7
	TIM7->CR1 |= TIM_CR1_CEN;
}

void delay_ms(uint32_t delay_ms) {
	//Capture the starting time
    uint32_t start = localTime;
    //Delay in us
    uint32_t delay = delay_ms*1000;
    //Wait until goal time is reached
    while(localTime - start <= delay);
}

void timeUpdate(void) {
    localTime += SYSTEMTICK_PERIOD_US;
}

uint32_t getTime_ms(void){
	return localTime/1000;
}

uint32_t getTime_us(void){
	return localTime;
}

void TIM6_DAC_IRQHandler(void){
	//If interrupt was raised for ITM6, reset flag and call ISR
	if (TIM6->SR & TIM_SR_UIF != 0){
		TIM6->SR &= ~TIM_SR_UIF;
		timePeriodicAction1();
	}
}

void TIM7_IRQHandler(void){
	//If interrupt was raised for TIM7, reset flag and call ISR
	if (TIM7->SR & TIM_SR_UIF != 0){
		TIM7->SR &= ~TIM_SR_UIF;
		timePeriodicAction2();
	}
}

static void timePeriodicAction1(void){
	ledToggle(LEDV2);

	uint8_t args[MAX_BYTE_ARGS_PACKET] = {0};
	float tempFloat;
	uint16_t tempUint16;

	//Sending front speed
	tempFloat = encoderGetSpeed(ENC_DR_FW);
	args[0] = *(((uint8_t*)(&tempFloat))+0);
	args[1] = *(((uint8_t*)(&tempFloat))+1);
	args[2] = *(((uint8_t*)(&tempFloat))+2);
	args[3] = *(((uint8_t*)(&tempFloat))+3);
	commandSendPacket(1, VAL_SPEED_FRONT, LEN_VAL_SPEEDFRONT, args);

	//Sending rear speed
	tempFloat = encoderGetSpeed(ENC_DR_RW);
	args[0] = *(((uint8_t*)(&tempFloat))+0);
	args[1] = *(((uint8_t*)(&tempFloat))+1);
	args[2] = *(((uint8_t*)(&tempFloat))+2);
	args[3] = *(((uint8_t*)(&tempFloat))+3);
	commandSendPacket(1, VAL_SPEED_REAR, LEN_VAL_SPEEDREAR, args);

	//Sending front angle
	tempFloat = encoderGetAngle(ENC_ST_FW);
	args[0] = *(((uint8_t*)(&tempFloat))+0);
	args[1] = *(((uint8_t*)(&tempFloat))+1);
	args[2] = *(((uint8_t*)(&tempFloat))+2);
	args[3] = *(((uint8_t*)(&tempFloat))+3);
	commandSendPacket(1, VAL_STEER_FRONT, LEN_VAL_STEERFRONT, args);

	//Sending rear angle
	tempFloat = encoderGetAngle(ENC_ST_RW);
	args[0] = *(((uint8_t*)(&tempFloat))+0);
	args[1] = *(((uint8_t*)(&tempFloat))+1);
	args[2] = *(((uint8_t*)(&tempFloat))+2);
	args[3] = *(((uint8_t*)(&tempFloat))+3);
	commandSendPacket(1, VAL_STEER_REAR, LEN_VAL_STEERREAR, args);

	//Sending rocker angle
	tempFloat = adcConvertToAngle(adc3ReadChannel(POS_RK));
	args[0] = *(((uint8_t*)(&tempFloat))+0);
	args[1] = *(((uint8_t*)(&tempFloat))+1);
	args[2] = *(((uint8_t*)(&tempFloat))+2);
	args[3] = *(((uint8_t*)(&tempFloat))+3);
	commandSendPacket(1, VAL_ANG_RCK, LEN_VAL_ANGRCK, args);

	//Sending front driving current
	tempUint16 = adc3ReadChannel(CUR_DR_FW);
	args[0] = (uint8_t)((tempUint16&0xFF00)>>8);
	args[1] = (uint8_t)((tempUint16&0x00FF)>>0);
	commandSendPacket(1, VAL_CUR_FRONT, LEN_VAL_CURFRONT, args);

	//Sending rear driving current
	tempUint16 = adc3ReadChannel(CUR_DR_RW);
	args[0] = (uint8_t)((tempUint16&0xFF00)>>8);
	args[1] = (uint8_t)((tempUint16&0x00FF)>>0);
	commandSendPacket(1, VAL_CUR_REAR, LEN_VAL_CURREAR, args);
}

static void timePeriodicAction2(void){
	ledToggle(LEDV1);
	setMotorCommand(MOT_DR_RW, controlCalcCommandSpeed(MOT_DR_RW));
	setMotorCommand(MOT_DR_FW, controlCalcCommandSpeed(MOT_DR_FW));
	setMotorCommand(MOT_ST_RW, controlCalcCommandAngle(MOT_ST_RW));
	setMotorCommand(MOT_ST_FW, controlCalcCommandAngle(MOT_ST_FW));
}

/**** END OF FILE ****/
