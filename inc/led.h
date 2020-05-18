 /// \file    led.h
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Header file for led.c


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _LED_H_
#define _LED_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/* Exported types ------------------------------------------------------------*/
 /// \enum led_TypeDef
 /// \brief built-in LEDs definition to simplify library usage
typedef enum{
	LEDV1 = 1,//!< The first green LED
	LEDV2 = 4,//!< The second green LED
	LEDR1 = 0,//!< The first red LED
	LEDR2 = 3 //!< The second red LED
} led_TypeDef;


/* Exported constants --------------------------------------------------------*/

/* Functions prototypes ------------------------------------------------------*/

 /// \brief  Initiates the 4 embedded LEDs
void ledsInit(void);

 /// \brief  Puts the led in ON state
 /// \param  led the led to be switched ON
void ledOn(led_TypeDef led);

 /// \brief  Puts the led in OFF state
 /// \param  led the led to be switched OFF
void ledOff(led_TypeDef led);

 /// \brief  Toggles the state of the led
 /// \param  led the led to be toggled
 ///
 /// Please note that this operation is not perform in a atomic time
void ledToggle(led_TypeDef led);

 /// \brief Sets all the LEDs states in one command
 /// \param ledv1 0 to turn the LEDV1 off something else to turn it on
 /// \param ledv2 0 to turn the LEDV off something else to turn it on
 /// \param ledr1 0 to turn the LEDR1 off something else to turn it on
 /// \param ledr2 0 to turn the LEDR2 off something else to turn it on
void ledSetAll(uint8_t ledv1, uint8_t ledv2, uint8_t ledr1, uint8_t ledr2);

#ifdef __cplusplus
}
#endif

#endif /* _LED_H_ */

 /**** END OF FILE ****/
