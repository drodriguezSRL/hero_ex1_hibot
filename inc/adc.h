 /// \file    adc.h
 /// \author  Alan ALLART
 /// \version 
 /// \brief   Header file for adc.c


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _INC_ADC_H_
#define _INC_ADC_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "common.h"

/* Exported types ------------------------------------------------------------*/
 /// \enum adc3channel_Typedef
 /// \brief used channels of ADC3 for sensing
typedef enum{
	CUR_ST_FW = 9, //!< Channel for Front Wheel STeering motor CURrent sensing
	CUR_ST_RW = 14,//!< Channel for Rear Wheel STeering motor CURrent sensing
	CUR_DR_FW = 15,//!< Channel for Front Wheel DRiving motor CURrent sensing
	CUR_DR_RW = 4, //!< Channel for Rear Wheel DRiving motor CURrent sensing
	POS_RK    = 5  //!< Channel for POSition of RocKer sensing
} adc3channel_Typedef;

/* Exported constants --------------------------------------------------------*/
 /// \def  ADC_TOTAL_CODES
 /// The number of total available codes with a 12bits ADC (2^12)
#define ADC_TOTAL_CODES 4096
 /// \def ADC_COUNT_TO_TORQUE_DRIVING
 /// Ratio between one ADC LSB and the torque given by a driving motor in Nm/LSB
 ///
 /// Calculated as: V_lsb * BridgeDividerRatio * CurrentSensorSlope *
 /// MotorComboTorqueConstant
#define ADC_COUNT_TO_TORQUE_DRIVING 0.02
 /// \def ADC_TORQUE_RESET_COUNT
 /// The number of counts at 0 current
#define ADC_TORQUE_RESET_COUNT 2048
 /// \def ADC_COUNT_TO_ANGLE_POT
 /// The ratio between the ADC LSB and the angle on the potentiometer (deg/LSB)
 ///
 /// Calculated as V_lsb * AngleRes (AngleRes = TotalAngle / Vcc)
#define ADC_COUNT_TO_ANGLE_POT 0.022
 /// \def ADC_POT_RESET_COUNT
 /// The number of counts at 0 angle
#define ADC_POT_RESET_COUNT 2048
/* Functions prototypes ------------------------------------------------------*/

 /// \brief Initialises the ADC measuring system
 ///
 /// Settings for the ADC are:
 /// 	- ADC used: ADC3
 /// 	- ADC clock prescaler: 2
 /// 	- Sampling Time: 480 clock cycles
 /// 	- Measuring process: single
 ///
 /// GPIO pins in analog input mode:
 /// 	- PF3 (ADC3_IN9)
 /// 	- PF4 (ADC3_IN14)
 /// 	- PF5 (ADC3_IN15)
 /// 	- PF6 (ADC3_IN4)
 /// 	- PF7 (ADC3_IN5)
void adcInit (void);

 /// \brief Initialises the ADC part of the ADC measuring system
static void adcInitADC(void);

 /// \brief Initialises the GPIO used with the ADC
static void adcInitGPIO(void);

 /// \brief Reads from a channel of ADC3
 /// \param channel the channel to acquire the measure from
 /// \return the measured value in number of LSB
uint16_t adc3ReadChannel(adc3channel_Typedef channel);

 /// \brief Converts a measure in LSB in a measure in Torque for the driving
 /// motor
 /// \param measure The measure to convert in LSB
 /// \return the torque measured
 ///
 /// Make sure that the values of #ADC_COUNT_TO_TORQUE_DRIVING and
 /// #ADC_TORQUE_RESET_COUNT are correct
float adcConvertToTorqueDriving(uint16_t measure);

 /// \brief Converts a measure in LSB in a measure in degrees for the
 /// potentiometer (rocker angle measurement)
 /// \param measure The measure to convert in LSB
 /// \return the angle measured
 ///
 /// Make sure the values of #ADC_COUNT_TO_ANGLE_POT and #ADC_POT_RESET_COUNT
 /// are correct
float adcConvertToAngle(uint16_t measure);

#ifdef __cplusplus
}
#endif

#endif /* _INC_ADC_H_ */

 /**** END OF FILE ****/
