 /// \file    main.c
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Main setup and loop for the microcontroller

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Global variables ----------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

int main(void) {

    initSysTick();
    crcInit();
    ledsInit();
    usartInit();
    adcInit();
    motorInit();
    encoderInit();
    timeAutoFunctionsInit();

    ledOff(LEDR1);
    ledOff(LEDR2);
    ledOff(LEDV1);
    ledOff(LEDV2);

    //Main loop
    while(1) {
    	delay_ms(5);
    	commandExtractAndProcess();
    }
}

/**** END OF FILE ****/
