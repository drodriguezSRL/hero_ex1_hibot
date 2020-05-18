 /// \file    crc.c
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Built-in CRC32 related functions


/* Includes ------------------------------------------------------------------*/
#include "crc.h"

/* Global variables ----------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/
void crcInit(void){
	//Enable Peripheral clock for CRC module
    RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
}

uint32_t crcComputeArray(const uint8_t array[], uint8_t size){
	uint32_t cur = 0;
	uint32_t res = 0;
	uint8_t remainder = size%4;

	//Reset CRC initial value
	CRC->CR |= CRC_CR_RESET;

	//Compute CRC for all the 4 bytes blocks
	for (uint8_t i = 0; i < size-remainder; i+=4){
		cur = (array[i]<<24) +
				(array[i+1]<<16) +
				(array[i+2]<<8) +
				(array[i+3]);
		CRC->DR = (cur);
	}

	//Compute CRC for the remaining bytes, right filling with 0x00 blocks
	if (remainder > 0){
		cur = 0;
		for (uint8_t i = 0; i < remainder; i++){
			cur += array[size-remainder+i]<<(8*(3-i));
		}
		CRC->DR = (cur);
	}

	res = CRC->DR;

	return res;
}


/**** END OF FILE ****/
