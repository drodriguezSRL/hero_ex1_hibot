 /// \file    test.h
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Header file for test.c


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _INC_TEST_H_
#define _INC_TEST_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "common.h"
#include "control.h"
#include "motor_driver.h"
#include "usart.h"
#include "adc.h"
#include "crc.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Functions prototypes ------------------------------------------------------*/
 /// \brief Test for the position control algorithm
 ///
 /// Try to reach different angles successively and remain steady during a
 /// given amount of time between the objectives \n
 /// Feel free to change the list of positions inside this function. Take care
 /// to also change the stop condition of the for loop if you change the
 /// number of items in the position list.
void testControlPosition(void);

 ///\brief Test for the reading of the encoder
 ///
 /// Frame 2 angles with LED codes, and can also send the angle trough UART \n
 /// If you change the angle for the LED framing, be sure that there is no
 /// timer overflow before this angle is reached
void testEncoderReading(void);

 /// \brief Gives a sequence of raw commands to the motor
void testMotorSimpleCommand(void);

 /// \brief Tests the ADC reading by reading a potentiometer and sending
 /// the actual angle through uart
void testAdcReading(void);

 /// \brief Prints the result of a CRC on the uart channel
void testCrc(void);

#ifdef __cplusplus
}
#endif

#endif /* _INC_TEST_H_ */

 /**** END OF FILE ****/
