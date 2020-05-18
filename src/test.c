 /// \file    test.c
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Implements several test functions for the created libraries


/* Includes ------------------------------------------------------------------*/
#include "test.h"

/* Global variables ----------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

void testControlPosition(void){

    initSysTick();
    ledsInit();
    motorInit();
    encoderInit();

    //List of position in degrees to reach successively
    //To modify to test other configurations
    float positions[10] = {0, 10, -20, 30, -40, 50, -60, 70, -80, 90};

	float wantedPos = 0;
	float curPos = 0;
	float errPos = 0;
	uint32_t tempTime;
	uint8_t reached = 0;
	uint8_t reachedAndSteady = 0;

	//Remember to change the stop condition if the array of position
	//has been changed
	for(uint8_t i = 0; i < 10; i++){

		reachedAndSteady = 0;
		reached = 0;
		wantedPos = positions[i];
		curPos = encoderGetAngle(ENC_ST_RW);
		errPos = wantedPos - curPos; //error on position

		while (!reachedAndSteady){
			//Command setting
			delay_ms(10);
			controlSetGoal(MOT_ST_RW, wantedPos);
			setMotorCommand(MOT_ST_RW, controlCalcCommandAngle(MOT_ST_RW));
			curPos = encoderGetAngle(ENC_ST_RW);
			errPos = wantedPos - curPos;
			//Reached and steady validation
			if (-1 < errPos && errPos < 1){
				if (reached == 0){
					reached = 1;
					tempTime = getTime_ms();
				}
				else if (getTime_ms() - tempTime > 1000){
					reachedAndSteady = 1;
				}
			}
			else
				reached = 0;
		}
	}

}

void testEncoderReading(void){

	float angle = 0;

	angle = encoderGetAngle(ENC_DR_RW);
	if(-90 < angle && angle < 90){
		ledOff(LEDR1);
		ledOff(LEDR2);
		ledOn(LEDV1);
		ledOn(LEDV2);
	}
	else if (angle < -90){
		ledOn(LEDR1);
		ledOff(LEDR2);
		ledOff(LEDV1);
		ledOn(LEDV2);
	}
	else {
		ledOff(LEDR1);
		ledOn(LEDR2);
		ledOn(LEDV1);
		ledOff(LEDV2);
	}

	//Printing routine
//	delay_ms(500);
//	usartSendByte(*(((uint8_t*)(&angle))+0));
//	usartSendByte(*(((uint8_t*)(&angle))+1));
//	usartSendByte(*(((uint8_t*)(&angle))+2));
//	usartSendByte(*(((uint8_t*)(&angle))+3));
}

void testMotorSimpleCommand(void){
	delay_ms(500);
	setMotorCommand(MOT_DR_RW, -0.2);
	delay_ms(500);
	setMotorCommand(MOT_DR_RW, 0);
	delay_ms(500);
	setMotorCommand(MOT_DR_RW, 0.2);
	delay_ms(500);
	setMotorCommand(MOT_DR_RW, -0.7);
	delay_ms(500);
	setMotorCommand(MOT_DR_RW, 0);
	delay_ms(500);
	setMotorCommand(MOT_DR_RW, 0.7);
}

void testAdcReading(void){
	float angle = 0;
	delay_ms(500);
	angle = adcConvertToAngle(adc3ReadChannel(POS_RK));
	usartSendByte(*(((uint8_t*)(&angle))+0));
	usartSendByte(*(((uint8_t*)(&angle))+1));
	usartSendByte(*(((uint8_t*)(&angle))+2));
	usartSendByte(*(((uint8_t*)(&angle))+3));
}

void testCrc(void){
    uint8_t t[8] = {1, 162, 29, 95, 59};
    uint32_t crc = 1;

	crc = crcComputeArray(t, 5);
	usartSendByte(*(((uint8_t*)(&crc))+3));
	usartSendByte(*(((uint8_t*)(&crc))+2));
	usartSendByte(*(((uint8_t*)(&crc))+1));
	usartSendByte(*(((uint8_t*)(&crc))+0));
}
/**** END OF FILE ****/
