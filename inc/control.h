 /// \file    control.h
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Header file for control.c


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _INC_CONTROL_H_
#define _INC_CONTROL_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "common.h"
#include "led.h"
#include "time.h"
#include "encoder.h"
#include "motor_driver.h"
#include "usart.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
 /// \def MAX_ABS_COMMAND
 /// The maximum command that should be obtained as output of the control
 /// function
#define MAX_ABS_COMMAND 0.9
 /// \def MIN_ABS_COMMAND
 /// The minimum command that should be obtained as output of the control
 /// function
#define MIN_ABS_COMMAND 0.1
 /// \def STEER_POS_MARGIN
 /// The maximum accepted error on position for the steering angle
#define STEER_POS_MARGIN 0.5
 /// \def DRIVE_SPEED_MARGIN
 /// The maximum accepted error on speed for the driving motor
#define DRIVE_SPEED_MARGIN 1
 /// \def DRIVE_SPEED_ERR_DEBUG
 /// The speed error used in controlDebugLedsSpd() to switch between green
 /// and orange indication (in degrees/second)
#define DRIVE_SPEED_ERR_DEBUG 10

 /// \def STEER_KP
 /// The Proportional gain for the steering control algorithm (angle)
#define STEER_KP 0.10
 /// \def STEER_KI
 /// The Integral gain for the steering control algorithm (angle)
#define STEER_KI 0.012
 /// \def STEER_KD
 /// The Derivative gain for the steering control algorithm (angle)
#define STEER_KD 0.0001
 /// \def DRIVE_KP
 /// The Proportional gain for the driving control algorithm (speed)
#define DRIVE_KP 0.0005
 /// \def DRIVE_KI
 /// The Integral gain for the driving control algorithm (speed)
#define DRIVE_KI 0.00000005

 /// \def MIN_ABS_ROTSPEED_DRIVING
 /// The minimum absolute speed a driving motor should have for the driving motor
 /// driver to be fully operational. This security is implemented in
 /// controlSetGoal()
#define MIN_ABS_ROTSPEED_DRIVING 75
 /// \def MAX_ABS_ANGLE_STEERING
 /// The maximum steering angle the High Level side can ask. This is a security
 /// preventing cable damages. This security is implemented in controlSetGoal()
#define MAX_ABS_ANGLE_STEERING 80

/* Functions prototypes ------------------------------------------------------*/
 /// \brief sets the goal value for a given motor
 ///
 /// \param motor The motor whose goal is to be set
 /// \param goal The goal the motor should reach
 ///
 /// The goal of a driving motor is a rotation speed in deg/s. The goal of
 /// a steering motor is a position in deg. Only implemented for rear steering
 /// motor for now
void controlSetGoal(motor_Typedef motor, float goal);

 /// \brief gives the command needed to reach a certain angle for a
 ///  steering motor
 /// \param motor the motor whose command to apply is to be computed
 /// \return the value of the command computed by the algorithm.
 ///
 /// Implements a PI control on the angle whose gains are #STEER_KP, #STEER_KI
 /// and #STEER_KD. If the actual position is close to the goal position
 /// within +/- #STEER_POS_MARGIN, output
 ///
 /// The higher the call frequency the better the control algorithm.
 ///
 /// If a driving motor is given as parameter, instead of a steering, returns 0.
 /// If the actual position is close to the goal position within
 /// +/- #STEER_POS_MARGIN, outputs 0 command
float controlCalcCommandAngle(motor_Typedef motor);

 /// \brief gives the command needed to reach a certain speed
 /// \param motor The motor whose command to apply is to be computed
 ///
 /// The higher the call frequency the better the control algorithm
float controlCalcCommandSpeed(motor_Typedef motor);

 /// \brief Change the state of the LEDs depending on the command level
 /// \param command the command level
 ///
 /// - LEDV1 and LEDV2: position OK, command = 0
 /// - LEDVx and LEDRx: command absolute between 0 and #MIN_ABS_COMMAND
 /// - LEDVx: command normal (between #MIN_ABS_COMMAND and #MAX_ABS_COMMAND)
 /// - LEDRx: command absolute larger than #MAX_ABS_COMMAND
static void controlDebugLedsPos(float command);

 /// \brief Change the state of the LEDs depending on the command level and the
 /// error on the speed
 /// \param command the command level
 /// \param errSpd the actual error on the speed
 ///
 /// - Speed error indication
 /// 	- LEDR2 : no error
 /// 	- LEDR2 and LEDV2 : speed error below #DRIVE_SPEED_ERR_DEBUG
 /// degrees/second in absolute
 /// 	- LEDR2 : speed error above #DRIVE_SPEED_ERR_DEBUG
 /// degrees/second in absolute
 /// - Command level indication
 /// 	- Nothing : command at level 0
 /// 	- LEDV1 : command absolute between 0 and #MIN_ABS_COMMAND
 /// 	- LEDV1 and LEDR1 : command normal
 /// (between #MIN_ABS_COMMAND and #MAX_ABS_COMMAND)
 /// 	- LEDR1 : command absolute larger than #MAX_ABS_COMMAND
static void controlDebugLedsSpd(float command, float errSpd);

 /// \brief Truncates the command for it to be in the accepted range
 /// \param p_command a pointer toward the command
 ///
 /// The function does the following in absolute value:
 /// - Command smaller than #MIN_ABS_COMMAND: changes it to 0
 /// - Command in the range: does nothing
 /// - Command above #MAX_ABS_COMMAND: changes command for #MAX_ABS_COMMAND
static void controlTruncCommand(float* p_command);

#ifdef __cplusplus
}
#endif

#endif /* _INC_CONTROL_H_ */

 /**** END OF FILE ****/
