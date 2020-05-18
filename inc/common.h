 /// \file    common.h
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Common macros and constants definitions
  

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _COMMON_H_
#define _COMMON_H_

#ifdef __cplusplus
 extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
 /// \def EPS
 /// A value used to compare floats
#define EPS 1E-16
 /// \def MIDDLE_32B
 /// The middle number for a 32bits unsigned integer
#define MIDDLE_32B 0x7FFFFFFF
 /// \def MIDDLE_16B
 /// The middle number for a 16bits unsigned integer
#define MIDDLE_16B 0x7FFF

 /// \def HCLK_FREQUENCY
 /// The frequency of the HCLK clock, defined by the configuration file
#define HCLK_FREQUENCY 168000000
 /// \def APB1_TIMER_FREQUENCY
 /// The frequency feeding the timers
#define APB1_TIMER_FREQUENCY (HCLK_FREQUENCY * 2 / 4)
 /// \def APB2_PERIPHERAL_FREQUENCY
 /// The frequency feeding the peripherals on APB2
#define APB2_PERIPHERAL_FREQUENCY (HCLK_FREQUENCY / 2)

#define APB2_TIMER_FREQUENCY (HCLK_FREQUENCY * 2 / 2)

/* Functions prototypes ------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* _LED_H_ */

 /**** END OF FILE ****/
