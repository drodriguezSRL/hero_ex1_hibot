 /// \file    crc.h
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Header file for crc.c


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _INC_CRC_H_
#define _INC_CRC_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "common.h"
#include "usart.h"
#include "time.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Functions prototypes ------------------------------------------------------*/
/// \brief Initialises the CRC module
void crcInit(void);

/// \brief Computes the CRC on a given array of bytes
/// \param array[] an array of bytes
/// \param size the size of the array
/// \return the computed CRC
///
/// This CRC computed has the following properties:
/// 	- Denomination: CRC-32/MPEG-2
/// 	- POLY: 0x04C11DB7
/// 	- Init: 0xFFFFFFFF
/// 	- Initial bit reverse (RefIn): false
/// 	- Final bit reverse (RefOut) (made before XorOut): false
/// 	- XorOut: 0x00000000
///
/// If size is not a multiple of 4, then the last chunk is right filled with
/// the right amount of 0x00 blocks.
///
/// So CRC({0x00, 0x01, 0x02, 0x03, 0x04}, 5) automatically becomes
/// CRC({0x00, 0x01, 0x02, 0x03, 0x04, 0x00, 0x00, 0x00}, 8) and is computed as
/// CRC32(0x00010203) XOR CRC32(0x04000000).
uint32_t crcComputeArray(const uint8_t array[], uint8_t size);

#ifdef __cplusplus
}
#endif

#endif /* _INC_CRC_H_ */

 /**** END OF FILE ****/
