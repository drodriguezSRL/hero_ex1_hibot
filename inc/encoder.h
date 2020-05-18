 /// \file    encoder.h
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Header file for encoder.c


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _INC_ENCODER_H_
#define _INC_ENCODER_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "common.h"
#include "time.h"
#include "led.h"

/* Exported types ------------------------------------------------------------*/
 /// \enum encoder_Typedef
 /// \brief list of the encoders connected to the HiBot
typedef enum{
	ENC_ST_RW,//!< Rear Wheel STeering ENCoder
	ENC_ST_FW,//!< Front Wheel STeering ENCoder
	ENC_DR_RW,//!< Rear Wheel DRiving ENCoder
 	ENC_DR_FW//!< Front Wheel DRiving ENCoder
} encoder_Typedef;

/* Exported constants --------------------------------------------------------*/
 /// \def STEER_REDUCTION
 /// Reduction ratio of the steering motor
#define STEER_REDUCTION 540
 /// \def STEER_CNT_PER_REV
 /// The number of counts per revolution BEFORE the reductors for the steering
 /// motor
 /// (4 counts = 1 cycle for quadrature encoders)
#define STEER_CNT_PER_REV 2048
 /// \def STEER_DEG_PER_CNT
 /// The number of degrees per count for the output shaft (AFTER the reductors)
 /// of the steering motors
 /// (4 counts = 1 cycle for quadrature encoders)
#define STEER_DEG_PER_CNT (360.0 / (STEER_REDUCTION * STEER_CNT_PER_REV))
 /// \def STEER_PREOF_ANGLE
 /// The angle corresponding to #STEER_PREOF_CNT counts
#define STEER_PREOF_ANGLE 8
 /// \def STEER_PREOF_CNT
 /// An arbitrary number of counts slightly before overflow, in order to handle it
#define STEER_PREOF_CNT 24576

 /// \def DRIVING_REDUCTION
 /// Reduction ratio of the driving motor
#define DRIVING_REDUCTION 62
 /// \def DRIVING_CNT_PER_REV
 /// The number of counts per revolution BEFORE the reductors for the driving
 /// motor
 /// (4 counts = 1 cycle for quadrature encoders)
#define DRIVING_CNT_PER_REV 4096
 /// \def DRIVING_DEG_PER_CNT
 /// The number of degrees per count for the output shaft (AFTER the reductors)
 /// of the driving motors
 /// (4 counts = 1 cycle for quadrature encoders)
#define DRIVING_DEG_PER_CNT (360.0 / (DRIVING_REDUCTION * DRIVING_CNT_PER_REV))

 /// \def SPEEDCALC_MIN_TIME_BETWEEN_US
 /// The minimum time between two speed calculation process
 /// This is in order to avoid noise due to derivative on a too short period
#define SPEEDCALC_MIN_TIME_BETWEEN_US 500
/* Functions prototypes ------------------------------------------------------*/
 /// \brief Initialises TIM1/2/3/5 to count position increments using
 /// 	the built-in quadrature encoder interface
 ///
 /// Settings: for all timers: Slave mode encoder mode
static void encoderTimerInit(void);

 /// \brief Initialises GPIOs as input of the built-in incremental encoder
 /// 	interfaces
 ///
 /// GPIO configured are the following:
 /// 	- PE9  (TIM1_CH1)
 /// 	- PE11 (TIM1_CH2)
 /// 	- PB4  (TIM2_CH1)
 /// 	- PC7  (TIM2_CH2)
 /// 	- PA15 (TIM3_CH1)
 /// 	- PB3  (TIM3_CH2)
 /// 	- PH10 (TIM5_CH1)
 /// 	- PH11 (TIM5_CH2)
 ///
 /// Settings are the following:
 /// 	- AF mode
 /// 	- AF1 for TIM1/2 and AF2 for TIM3/5
 /// 	- default settings for the remaining parts
static void encoderGpioInit(void);

 /// \brief Initialises the encoder system
 ///
 /// This is the function which should be used by the user to initialise encoder
 /// related functionalities
void encoderInit(void);

 /// \brief Resets the position counter to its center
void encoderResetCount(encoder_Typedef enc);

 /// \brief Get the angle travelled by the motor since last counter reset (or
 /// 	since system startup)
 ///
 /// \return The angle in degrees
float encoderGetAngle(encoder_Typedef enc);

 /// \brief Calculate the average speed for a motor between now and last call
 /// \param enc the encoder of the motor whose speed is to be computed
 ///
 /// If two calls are made within #SPEEDCALC_MIN_TIME_BETWEEN_US microseconds,
 /// then doesn't compute a new speed
void encoderCalculateSpeed(encoder_Typedef enc);

 /// \brief Gives the mean speed calculated by encoderCalculateSpeed()
 /// \param enc The encoder of the driving motor whose speed is to be returned
 /// \return The mean speed
 ///
 /// If the parameter is not a driving motor, returns 0
float encoderGetSpeed(encoder_Typedef enc);

 /// \brief Interrupt handler for TIM3 general interrupts
void TIM3_IRQHandler(void);

 /// \brief Interrupt handler for TIM1 Capture and Compare interrupts
void TIM1_CC_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* _INC_ENCODER_H_ */

 /**** END OF FILE ****/
