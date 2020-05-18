 /// \file    usart.c
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Main related functions of USART communication. Mainly used to
 /// either debug or speak with OBC


/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* Global variables ----------------------------------------------------------*/
//! The buffer used for Rx data
static volatile fifo_TypeDef rxBuffer = {{0}, 0, 0, 0, 0, 0};
//! The buffer used for Tx data
static volatile fifo_TypeDef txBuffer = {{0}, 0, 0, 0, 0, 0};

/* Functions -----------------------------------------------------------------*/

static void usartInterfaceInit(void){
	//Enable the clock for USART6 peripheral
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    //Enable the USART6 interface
    USART6->CR1 |= USART_CR1_UE;
    //Set 115200 bauds
    USART6->BRR = (45<<4)+9;
    //Enable the USART transceiver
    USART6->CR1 |= USART_CR1_TE | USART_CR1_RE;
    //Enable interrupt on data reception
    USART6->CR1 |= USART_CR1_RXNEIE;
    //Enable the triggering of the USART interrupts in the NVIC system
    NVIC->ISER[2] |= 1<<7;
}

static void usartGpioInit(void){
	//Enable clock for the GPIO peripherals
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOGEN;
	//Put the GPIOs in AF mode
    GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER6) | (GPIO_MODER_MODER6_1);
    GPIOG->MODER = (GPIOG->MODER & ~GPIO_MODER_MODER9) | (GPIO_MODER_MODER9_1);
    //Add a pull up on Tx pin
    GPIOC->PUPDR = (GPIOC->PUPDR & ~GPIO_PUPDR_PUPDR6) | (GPIO_PUPDR_PUPDR6_0);
    //Map the AF of the PC6 and PG9 GPIOs (AFUSART6)
    GPIOC->AFR[0] = (GPIOC->AFR[0] & ~(0b1111<<24)) | (0b1000<<24);
    GPIOG->AFR[1] = (GPIOG->AFR[1] & ~(0b1111<<4)) | (0b1000<<4);
}

void usartInit(void){
	usartGpioInit();
	usartInterfaceInit();
}

void USART6_IRQHandler(void){
	//Received data register non empty
	if ((USART6->SR & USART_SR_RXNE) != 0){
		usartReceive_ISR();
	}
	//Transmit data register empty
	if ((USART6->SR & USART_SR_TXE) != 0){
		usartTransmit_ISR();
	}
}

void usartReceive_ISR(void){
	//FIFO already full
	if (rxBuffer.numBytes == FIFO_BUFFER_SIZE){
		//raise full flag
		rxBuffer.full = 1;
		//Discard incoming data
		USART6->DR;
		//Inform with LED
		ledOn(LEDR2);
	}
	//Free space in FIFO
	else{
		//TODO: Check and handle errors
		//Stores data in fifo
		rxBuffer.buffer[rxBuffer.last] = USART6->DR;
		//Updates info of fifo
		rxBuffer.last++;
		rxBuffer.numBytes++;
		//If became full, raise full flag
		if (rxBuffer.numBytes == FIFO_BUFFER_SIZE)
			rxBuffer.full = 1;
		//Roll over of the index if end of fifo reached
		if (rxBuffer.last == FIFO_BUFFER_SIZE)
			rxBuffer.last = 0;
	}
	rxBuffer.empty = 0;
}

void usartTransmit_ISR(void){
	//If fifo full, it will not be anymore
	if (txBuffer.numBytes == FIFO_BUFFER_SIZE)
		txBuffer.full = 0;
	//If non empty
	if (txBuffer.numBytes > 0){
		//Place data in the write register
		USART6->DR = txBuffer.buffer[txBuffer.first];
		//Updates the info of fifo
		txBuffer.first++;
		txBuffer.numBytes--;
		//Roll over the index if end is reached
		if (txBuffer.first == FIFO_BUFFER_SIZE){
			txBuffer.first = 0;
		}
		//Deactivate TXE interrupts if fifo gets empty
		if (txBuffer.numBytes == 0){
			USART6->CR1 &= ~USART_CR1_TXEIE;
			txBuffer.empty = 1;
		}
	}
}

uint8_t usartGetByte(void){
	//Disable Rx interrupts for avoiding pointers confusion
	USART6->CR1 &= ~USART_CR1_RXNEIE;

	//Received data
	uint8_t byte = 0;
	//If full before, we will create free space
	if (rxBuffer.numBytes == FIFO_BUFFER_SIZE){
		//Unraise overflow flag
		rxBuffer.full = 0;
	}
	//If non empty handle data in fifo
	if (rxBuffer.numBytes > 0){
		//Read first data in fifo
		byte = rxBuffer.buffer[rxBuffer.first];
		//Updates the info of fifo
		rxBuffer.first++;
		rxBuffer.numBytes--;
		//Roll over the index if end reached
		if(rxBuffer.first == FIFO_BUFFER_SIZE)
			rxBuffer.first = 0;
	}
	//If empty raise flag
	else
		rxBuffer.empty = 1;

	//Reactivate Rx interrupts
	USART6->CR1 |= USART_CR1_RXNEIE;

	return byte;
}

void usartSendByte(uint8_t c){
	//Disable Tx interrupts for avoiding pointers confusion
	USART6->CR1 &= ~USART_CR1_TXEIE;

	//If fifo full raise full flag
	if (txBuffer.numBytes == FIFO_BUFFER_SIZE)
		txBuffer.full = 1;
	//If non full handle data to send
	else{
		//Put data at the end of the fifo
		txBuffer.buffer[txBuffer.last] = c;
		//Updates fifo info
		txBuffer.last++;
		txBuffer.numBytes++;
		//If buffer got full, raise flag
		if (txBuffer.numBytes == FIFO_BUFFER_SIZE)
			txBuffer.full = 1;
		//Roll over the index if it reached end
		if (txBuffer.last == FIFO_BUFFER_SIZE){
			txBuffer.last = 0;
		}
	}
	txBuffer.empty = 0;

	//Reactivate TXE interrupt, as fifo is surely not empty
	USART6->CR1 |= USART_CR1_TXEIE;
}

uint16_t usartBufferGetNumbytes(uint8_t buffer){
	uint16_t numBytes = 0;
	if (buffer == RX_BUFFER){
		USART6->CR1 &= ~USART_CR1_RXNEIE;
		numBytes = rxBuffer.numBytes;
		USART6->CR1 |= USART_CR1_RXNEIE;
	}
	if (buffer == TX_BUFFER){
		USART6->CR1 &= ~USART_CR1_TXEIE;
		numBytes = txBuffer.numBytes;
		USART6->CR1 |= USART_CR1_TXEIE;
	}
	return numBytes;
}

uint8_t usartBufferCheckValue(uint8_t buffer, uint16_t index){
	uint8_t value = 0;
	uint16_t absoluteIndex = index;
	if (buffer == RX_BUFFER){
		USART6->CR1 &= ~USART_CR1_RXNEIE;
		if (index + rxBuffer.first >= FIFO_BUFFER_SIZE)
			absoluteIndex = absoluteIndex + 1 - rxBuffer.first;
		else
			absoluteIndex += rxBuffer.first;
		value = rxBuffer.buffer[absoluteIndex];
		USART6->CR1 |= USART_CR1_RXNEIE;
	}
	if (buffer == TX_BUFFER){
		USART6->CR1 &= ~USART_CR1_TXEIE;
		if (index + txBuffer.first >= FIFO_BUFFER_SIZE)
			absoluteIndex = absoluteIndex + 1 - txBuffer.first;
		else
			absoluteIndex += txBuffer.first;
		value = txBuffer.buffer[absoluteIndex];
		USART6->CR1 |= USART_CR1_TXEIE;
	}
	return value;
}

void usartEmptyRxBuffer(void){
	uint16_t bufferSize = rxBuffer.numBytes;
	for (uint16_t i = 0; i < bufferSize; i++){
		usartGetByte();
	}
}

/**** END OF FILE ****/
