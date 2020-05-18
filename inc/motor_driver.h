 /// \file    motor_driver.h
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Header file for motor_driver.c

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MOTOR_DRIVER_H_
#define _MOTOR_DRIVER_H_

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "common.h"

/* Exported types ------------------------------------------------------------*/
 /// \enum motor_Typedef
 /// \brief list of the motors connected to the HiBot
typedef enum{
	MOT_ST_RW,//!< Rear Wheel STeering MOTor
	MOT_ST_FW,//!< Front Wheel STeering MOTor
	MOT_DR_RW,//!< Rear Wheel DRiving MOTor
 	MOT_DR_FW//!< Front Wheel DRiving MOTor
} motor_Typedef;

/* Exported constants --------------------------------------------------------*/
 /// \def MOTOR_PWM_FREQUENCY
 /// Frequency of the control PWM for the motors
#define MOTOR_PWM_FREQUENCY 32000
 /// \def MOTOR_PWM_PSC
 /// Prescaler of the control PWM for the motors
#define MOTOR_PWM_PSC 1
 /// \def MOTOR_PWM_ARR_1
 /// Auto Reload Register value (maximum of the PWM internal counter) for
 /// timers in APB1 clock domain
 ///
 /// Calculated using the relation (PulsesFrequency = InputFrequency / PSC / ARR)
#define MOTOR_PWM_ARR_1 (APB1_TIMER_FREQUENCY / MOTOR_PWM_PSC / \
						MOTOR_PWM_FREQUENCY)
/// \def MOTOR_PWM_ARR_2
/// Auto Reload Register value (maximum of the PWM internal counter) for
/// timers in APB1 clock domain
///
/// Calculated using the relation (PulsesFrequency = InputFrequency / PSC / ARR)
#define MOTOR_PWM_ARR_2 (APB2_TIMER_FREQUENCY / MOTOR_PWM_PSC / \
						MOTOR_PWM_FREQUENCY)

/* Functions prototypes ------------------------------------------------------*/

 /// \brief  Initialises PWM for all timers and channels
 ///
 /// Channels initialised:
 /// 	- TIM4: all channels
 /// 	- TIM8: all channels
 ///
 /// Parameters
 /// 	- ARR = #MOTOR_PWM_ARR_1 for TIM4 and #MOTOR_PWM_ARR_2 for TIM8
 /// 	- PSC = #MOTOR_PWM_PSC
 /// 	- CCR initialised to 0 for all channels
 /// 	- PWM mode 2 for all channels
static void motorPwmInit(void);

 /// \brief  Initialises GPIOs for PWM generation
 ///
 /// GPIO configured are the following:
 /// 	- PD12 (TIM4_CH1)
 /// 	- PD13 (TIM4_CH2)
 /// 	- PD14 (TIM4_CH3)
 /// 	- PD15 (TIM4_CH4)
 /// 	- PI5 (TIM8_CH1)
 /// 	- PI6 (TIM8_CH2)
 /// 	- PI7 (TIM8_CH3)
 /// 	- PI2  (TIM8_CH4)
 ///
 /// Configuration is the following:
 /// 	- Mode : AF
 /// 	- AF: AF2 (TIM3/4/5) or AF3 (TIM 8/9/10/11)
 /// 	- Default settings
static void motorGpioInit(void);

 /// \brief  Initialises the motor driver
 ///
 /// This is the function which should be used by the user to init motor
 /// functionalities. Calls motorPwmInit() and motorGpioInit()
void motorInit(void);

 /// \brief  Changes PWM applied to the motor driver
 /// \param motor The motor to be controlled
 /// \param  command the command ratio whose absolute value is either 0 or
 /// 		between #MIN_ABS_COMMAND and #MAX_ABS_COMMAND ;
 /// 		CCW mode with negative command value, CW mode with positive
 ///
 /// This function has no input correct system. Take care of being sure
 /// of the input correctness.
void setMotorCommand(motor_Typedef motor, float command);

#ifdef __cplusplus
}
#endif

#endif /* _LED_H_ */

 /**** END OF FILE ****/
