 /// \file    usart.h
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Header file for usart.c


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _INC_USART_H_
#define _INC_USART_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "common.h"
#include "led.h"

/* Exported constants --------------------------------------------------------*/
 /// \def FIFO_BUFFER_SIZE
 /// The size of the fifo_TypeDef fifo in bytes. Maximum value is
 /// 2^16 = 65536
#define FIFO_BUFFER_SIZE 1024

#define RX_BUFFER 0
#define TX_BUFFER 1

/* Exported types ------------------------------------------------------------*/
 /// \brief A ring fifo to be used as a buffer
 ///
 /// Used to implement #rxBuffer and #txBuffer communication buffers
typedef struct{
	uint8_t buffer[FIFO_BUFFER_SIZE]; //!< The fifo buffer itself
	uint16_t first; //!< The index of the first element (i.e. the next output)
	uint16_t last; //!< The index of the last element (i.e. the last output)
	uint16_t numBytes; //!< The total number of bytes in the fifo
	uint8_t empty; //!< Flag indicating that the buffer is empty
	uint8_t full; //!< Flag indicating that the buffer has come to an overflow
}fifo_TypeDef;

/* Functions prototypes ------------------------------------------------------*/
 /// \brief Initialises the USART6 interface parameters:
 /// Initialises baudrate, parity, number of bits, ...
static void usartInterfaceInit(void);

 /// \brief Initialises the COM port for the UART:
 /// configures pin PC6 as Tx and PG9 as Rx
static void usartGpioInit(void);

 /// \brief Initialises USART6 and COM port
 ///
 /// The following pins are used
 /// 	- Tx as PC6
 /// 	- Rx as PG9
 ///
 /// The following settings are done
 /// 	- 8 bits of total data (parity included, but parity deactivated now)
 /// 	- Parity: deactivated
 /// 	- 1 stop bit
 /// 	- 115200 bauds
void usartInit(void);

 /// \brief Handles the interrupts raised by the USART6 Transceiver
 ///
 /// Implements :
 /// 	- Action on RXNE interrupt: usartReceive_ISR()
 /// 	- Action on TXE interrupt: usartTransmit_ISR()
void USART6_IRQHandler(void);

 /// \brief ISR called on RXNE interrupt
 ///
 /// Handles the interrupt and place received byte in #rxBuffer fifo.
 /// If fifo is full, incoming data is discarded
void usartReceive_ISR(void);

 /// \brief ISR called on TXE interrupt
 ///
 /// Handles the interrupt and place the first byte of #txBuffer fifo
 /// in the Transmit Data Register
 /// Note that this routine also disables interrupts on TXE when the
 /// fifo gets empty.
void usartTransmit_ISR(void);

 /// \brief gives the first byte of #rxBuffer fifo
 /// \return the first byte in the fifo. Return 0 if fifo was empty
 ///
 /// To know if there is byte(s) available, check status flag
 /// fifo_TypeDef.empty
uint8_t usartGetByte(void);

 /// \brief places byte in the #txBuffer fifo
 /// \param c the byte to put at the end of the fifo
 ///
 /// If the fifo is already full, it discards the byte and does nothing
 /// To know if there is free space in the buffer, check status flag
 /// fifo_TypeDef.full
void usartSendByte(uint8_t c);

/// \brief empties #rxBuffer
void usartEmptyRxBuffer(void);

uint16_t usartBufferGetNumbytes(uint8_t buffer);
uint8_t usartBufferCheckValue(uint8_t buffer, uint16_t index);

#ifdef __cplusplus
}
#endif

#endif /* _INC_USART_H_ */

 /**** END OF FILE ****/
