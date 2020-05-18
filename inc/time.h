 /// \file    time.h
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Header file for time.c

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _TIME_H_
#define _TIME_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "common.h"
#include "led.h"
#include "usart.h"
#include "control.h"
#include "motor_driver.h"
#include "commandInterface.h"
#include "encoder.h"
#include "adc.h"

 /* Exported constants --------------------------------------------------------*/
 /// \def TICK_FREQ
 /// Frequency of the sysTick interrupts
#define TICK_FREQ 1000000
 /// \def SYSTEMTICK_PERIOD_US
 /// Time increment between two sysTick interrupts in us
#define SYSTEMTICK_PERIOD_US 1

 /// \def PERIODIC1_FREQ
 /// The call frequency of the function timePeriodicAction1() in Hz. If you are to
 /// change this frequency, make sure that the #PERIODIC1_PSC value is still
 /// compatible with this new frequency
#define PERIODIC1_FREQ 5
 /// \def PERIODIC1_PSC
 /// The prescaler used in TIM6 to call back timePeriodicAction1(). This is a 16
 /// bits value maximum
#define PERIODIC1_PSC 700
 /// \def PERIODIC1_ARR
 /// The auto reload used in TIM6 to call back timePeriodicAction1(). This is a 16
 /// bits value maximum
#define PERIODIC1_ARR (APB1_TIMER_FREQUENCY / PERIODIC1_FREQ / PERIODIC1_PSC)

 /// \def PERIODIC2_FREQ
 /// The call frequency of the function timePeriodicAction2() in Hz. If you are to
 /// change this frequency, make sure that the #PERIODIC2_PSC value is still
 /// compatible with this new frequency
#define PERIODIC2_FREQ 100
 /// \def PERIODIC2_PSC
 /// The prescaler used in TIM7 to call back timePeriodicAction2(). This is a 16
 /// bits value maximum
#define PERIODIC2_PSC 700
 /// \def PERIODIC2_ARR
 /// The auto reload used in TIM7 to call back timePeriodicAction2(). This is a 16
 /// bits value maximum
#define PERIODIC2_ARR (APB1_TIMER_FREQUENCY / PERIODIC2_FREQ / PERIODIC2_PSC)

/* Functions prototypes ------------------------------------------------------*/
 /// \brief  Initiates sysTick timing system
void initSysTick(void);

 /// \brief Initialises the timers used for the auto called functions
 /// timePeriodicAction1() and timePeriodicAction2()
void timeAutoFunctionsInit(void);

 /// \brief  Busy-waits for a given amount of time
 /// \param  delay_ms the freeze duration in milliseconds
void delay_ms(uint32_t delay_ms);

 /// \brief  Updates the system local time
 ///
 /// This function is an interrupt routine for the system. It should not be used
 /// by the user for any purpose
void timeUpdate(void);

 /// \brief  Gives time since startup in ms
 /// \return time in milliseconds
 ///
 /// overflows each 1h10 approx
uint32_t getTime_ms(void);

 /// \brief  Gives time since startup in us
 /// \return time in microseconds
 ///
 /// overflows each 1h10 approx
uint32_t getTime_us(void);

 /// \brief IRQ Handler for the timer 6, used to call the timePeriodicAction1()
 /// ISR
void TIM6_DAC_IRQHandler(void);

 /// \brief IRQ Handler for the timer 7, used to call the timePeriodicAction2()
 /// ISR
void TIM7_IRQHandler(void);

 /// \brief Place here the actions to be automatically done at a rate given by
 /// #PERIODIC1_FREQ. Make sure the implied computations are not too big for
 /// this rate
 ///
 /// Now, implements automatic control
static void timePeriodicAction1(void);

 /// \brief Place here the actions to be automatically done at a rate given by
 /// #PERIODIC2_FREQ. Make sure the implied computations are not too big for
 /// this rate
 ///
 /// Now, implements auto data sending to master
static void timePeriodicAction2(void);

#ifdef __cplusplus
}
#endif

#endif /* _TIME_H_ */

 /**** END OF FILE ****/
