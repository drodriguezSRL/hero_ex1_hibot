 /// \file    control.c
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Implements automatic control related functions: command level
 /// computation mainly


/* Includes ------------------------------------------------------------------*/
#include "control.h"

/* Global variables ----------------------------------------------------------*/
//! The goal of the motors. Position goals for steering and speed goals for
//! driving
static float motorGoals[4] = {0, 0, 0, 0};

/* Functions -----------------------------------------------------------------*/

void controlSetGoal(motor_Typedef motor, float goal){
	if (motor == MOT_ST_FW || motor == MOT_ST_RW){
		if (-MAX_ABS_ANGLE_STEERING < goal && goal < MAX_ABS_ANGLE_STEERING)
			motorGoals[motor] = goal;
	}
	else{
		if (-MIN_ABS_ROTSPEED_DRIVING < goal && goal < MIN_ABS_ROTSPEED_DRIVING)
			motorGoals[motor] = 0;
		else
			motorGoals[motor] = goal;
	}
}

float controlCalcCommandAngle(motor_Typedef motor){
	//Exit if wrong parameters
	if (motor != MOT_ST_FW && motor != MOT_ST_RW)
		return 0;

	//Integral errors
	static float errSum[2] = {0, 0};
	//Previous call times
	static uint32_t prevTime[2] = {0, 0};
	uint32_t dt = getTime_ms() - prevTime[motor];
	//Update previous call time
	prevTime[motor] = getTime_ms();

	float posNow = encoderGetAngle(motor);
	//Position error
	float errPos = motorGoals[motor] - posNow;
	//Final command
	float command = 0;

	//Position good enough: turn errors to 0
	if (-STEER_POS_MARGIN < errPos && errPos < STEER_POS_MARGIN){
		errPos = 0;
		errSum[motor] = 0;
	}
	else {
		errSum[motor] += errPos*dt;
		//Makes the integral term saturate to avoid excessive overshoot
		//Due to a power cut off
		if (STEER_KI*errSum[motor] > MAX_ABS_COMMAND)
			errSum[motor] = MAX_ABS_COMMAND / STEER_KI;
		if (STEER_KI*errSum[motor] < -MAX_ABS_COMMAND)
			errSum[motor] = -MAX_ABS_COMMAND / STEER_KI;
	}
	//PI control equation
	command = STEER_KI*errSum[motor] + STEER_KP*errPos;
	//Truncate the command for it to be in the margins
	controlTruncCommand(&command);

	return command;
}

float controlCalcCommandSpeed(motor_Typedef motor){
	//Exit if wrong motor
	if (motor != MOT_DR_FW && motor != MOT_DR_RW)
		return 0;
	//Integral errors
	static float errSum[2] = {0, 0};
	//Last function call times
	static uint32_t prevTime[2] = {0, 0};
	uint32_t dt = getTime_us() - prevTime[motor-2];
	//Update previous call time
	prevTime[motor-2] = getTime_us();

	float spdNow = encoderGetSpeed(motor);
	//Speed error
	float errSpd = motorGoals[motor] - spdNow;
	float command = 0;

	errSum[motor-2] += errSpd*dt;

	//Makes the integral term saturate to avoid excessive overshoot
	//Due to a power cut off
	if (DRIVE_KI*errSum[motor-2] > MAX_ABS_COMMAND)
		errSum[motor-2] = MAX_ABS_COMMAND / DRIVE_KI;
	if (DRIVE_KI*errSum[motor-2] < -MAX_ABS_COMMAND)
		errSum[motor-2] = -MAX_ABS_COMMAND / DRIVE_KI;

	//PI control equation
	command = DRIVE_KI*errSum[motor-2] + DRIVE_KP*errSpd;
	//Truncate the command for it to be in the margins
	controlTruncCommand(&command);

	return command;
}

static void controlDebugLedsPos(float command){
	//No command
	if (-EPS < command && command < EPS)
	    ledSetAll(1, 1, 0, 0);

	//Full command
	else if (MAX_ABS_COMMAND < command)
		ledSetAll(0, 0, 0, 1);
	else if (command < -MAX_ABS_COMMAND)
		ledSetAll(0, 0, 1, 0);

	//Small command
	else if (EPS < command && command < MIN_ABS_COMMAND)
		ledSetAll(0, 1, 0, 1);
	else if (-MIN_ABS_COMMAND < command && command < -EPS)
		ledSetAll(1, 0, 1, 0);

	//In range command
	else if (MIN_ABS_COMMAND <= command && command <= MAX_ABS_COMMAND)
		ledSetAll(1, 0, 0, 0);
	else if (-MAX_ABS_COMMAND <= command && command <= -MIN_ABS_COMMAND)
		ledSetAll(0, 1, 0, 0);

	//Unknown
	else
		ledSetAll(1, 1, 1, 1);
}

static void controlDebugLedsSpd(float command, float errSpd){
	if (command < -EPS)
		command *= -1;
	if (errSpd < -EPS)
		errSpd *= -1;

	ledSetAll(0, 0, 0, 0);

	if (command < EPS)
		{}
	else if (command < MIN_ABS_COMMAND)
		ledOn(LEDV1);
	else if (command < MAX_ABS_COMMAND){
		ledOn(LEDV1);
		ledOn(LEDR1);
	}
	else
		ledOn(LEDR1);

	if (errSpd < EPS)
		ledOn(LEDV2);
	else if (errSpd < DRIVE_SPEED_ERR_DEBUG){
		ledOn(LEDV2);
		ledOn(LEDR2);
	}
	else
		ledOn(LEDR2);

}

static void controlTruncCommand(float* p_command){

	//Overshoot
	if (*p_command > MAX_ABS_COMMAND)
		*p_command = MAX_ABS_COMMAND;
	else if (*p_command < -MAX_ABS_COMMAND)
		*p_command = -MAX_ABS_COMMAND;

	//Undershoot
	else if (-MIN_ABS_COMMAND < *p_command && *p_command < MIN_ABS_COMMAND)
		*p_command = 0;
}

/**** END OF FILE ****/
