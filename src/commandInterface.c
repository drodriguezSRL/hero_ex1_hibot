 /// \file    commandInterface.c
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Implements all the useful functions to decode byte streams
 /// coming from UART and interpret them as commands, after having done
 /// all the necessary checks.
 ///
 /// Those functions use COBS stuffing algorithm.
 /// See Stuart Cheshire paper (Consistent Overhead Byte Stuffing) from 1999
 /// for more explanations on the algorithm working.
 /// Also implements the packets coding and sending to the master


/* Includes ------------------------------------------------------------------*/
#include "commandInterface.h"

/* Global variables ----------------------------------------------------------*/
//! A buffer for storing an incoming stuffed packet without its delimiter
// The +1 for the sizes comes from the stuffing method
static uint8_t rxIncomingPacket[MAX_PACKET_SIZE+1] = {0};
//! A buffer for storing an unstuffed incoming packet
static uint8_t rxUnstuffedPacket[MAX_PACKET_SIZE] = {0};

/* Functions -----------------------------------------------------------------*/
//Stores the incoming packet in rxIncomingPacket
static uint8_t commandExtractPacketFromUsart(void){
	//Previous packet completely received (0) or not (1)
	static uint8_t packetIncoming = 0;
	//Current writing position in the buffer
	static uint8_t index = 0;
	uint8_t tempIndex = 0;
	uint8_t data;

	//Empty the rxBuffer until a COBS_DELIMITER shows up
	while (!packetIncoming){
		//If no data available quit
		if (usartBufferGetNumbytes(RX_BUFFER) == 0)
			return 0;
		//Else, if the data is a delimiter, start packet storage
		data = usartGetByte();
		if (data == COBS_DELIMITER){
			packetIncoming = 1;
		}
	}
	//Here, data == COBS_DELIMITER

	//If no other character available, quit
	if (usartBufferGetNumbytes(RX_BUFFER) == 0)
		return 0;

	//If two consecutive delimiters and buffer still empty, discard
	//the first, reset the incoming packet status and quit
	//Important to discard only the first and not both of them to allow
	//auto resynchronisation (that why we don't use usartGetByte() here)
	if (usartBufferCheckValue(RX_BUFFER, 0) == COBS_DELIMITER && index == 0){
		packetIncoming = 0;
		return 0;
	}

	//Otherwise, go through until the second delimiter and store what's
	//between the delimiters in the packet buffer
	data = usartGetByte();
	while (data != COBS_DELIMITER){
		//If packet too long, reset the status and quit
		if (index >= MAX_PACKET_SIZE+1){
			packetIncoming = 0;
			index = 0;
			return 0;
		}
		//Stores data in the buffer
		rxIncomingPacket[index] = data;
		index++;
		//Check for available data (otherwise quit) and take it
		if (usartBufferGetNumbytes(RX_BUFFER) == 0)
			return 0;
		data = usartGetByte();
	}

	//If program arrive here, means that 2 delimiters received within
	//the maximum size.
	//So reset the status and return the size of the packet
	tempIndex = index;
	packetIncoming = 0;
	index = 0;

	return tempIndex;
}

//Stores the unstuffed packet in rxUnstuffedPacket
static uint8_t commandUnstuffCOBS(uint8_t packetSize){

	//If no data quit
	if (packetSize == 0)
		return 0;

	uint8_t i = 0;
	//The size of the actual COBS block
	uint8_t nbChar;
	while (i < packetSize){
		nbChar = rxIncomingPacket[i];
		//Go through a COBS block and copy characters
		for (uint8_t j = 1; j < nbChar; j++){
			//Security against overflow
			if (i+j >= MAX_PACKET_SIZE+1)
				return 0;
			//Copy characters in the unstuffed buffer
			rxUnstuffedPacket[i+j-1] = rxIncomingPacket[i+j];
		}
		//Add the 0 for the block end
		rxUnstuffedPacket[i+nbChar-1] = 0;
		i += nbChar;
	}
	//-1 to "delete" the ending 0 which has been added at the encoding process
	return i-1;
}

static uint8_t commandCheckValidity(uint8_t size){
	uint8_t command = rxUnstuffedPacket[PACKET_CMD];
	uint8_t argSize = rxUnstuffedPacket[PACKET_LEN];
	uint32_t crc = (rxUnstuffedPacket[size-4]<<24) +
			(rxUnstuffedPacket[size-3]<<16) +
			(rxUnstuffedPacket[size-2]<<8) +
			(rxUnstuffedPacket[size-1]);

	//Check if command is in the command list
	if (!_IS_COMMAND(command)){
//		usartSendByte(1); //debug
		return 0;
	}

	//Check if the number of arguments match the command
	if (!_IS_LEN_CORRECT(command, argSize)){
//		usartSendByte(2); //debug
		return 0;
	}

	//Check if the number of arg bytes matches the overall length
	if (size != PACKET_LEN_WITHOUT_ARGS + argSize){
//		usartSendByte(3); //debug
		return 0;
	}

	//CRC check
	if (crcComputeArray(rxUnstuffedPacket, size-4) != crc){
//		usartSendByte(4); //debug
		return 0;
	}

	//If no fails in the above tests, the packet is considered correct
	return 1;
}

static void commandExecuteActions(uint8_t size){
	uint16_t id = (rxUnstuffedPacket[PACKET_ID_1]<<8) +
			(rxUnstuffedPacket[PACKET_ID_0]);
	uint8_t command = rxUnstuffedPacket[PACKET_CMD];
	uint8_t len = rxUnstuffedPacket[PACKET_LEN];
	uint32_t tempI;
	float tempF;

	switch (command){

		case CMD_LEDR1_ON:
			ledOn(LEDR1);
			break;

		case CMD_LEDR1_OFF:
			ledOff(LEDR1);
			break;

		case CMD_SETSPEED_FRONT:
			tempI = *(uint32_t*)(&((rxUnstuffedPacket[PACKET_LEN+1])));
			tempF = *((float *)(&(tempI)));
			controlSetGoal(MOT_DR_FW, tempF);
			break;

		case CMD_SETSPEED_REAR:
			tempI = *(uint32_t*)(&((rxUnstuffedPacket[PACKET_LEN+1])));
			tempF = *((float *)(&(tempI)));
			controlSetGoal(MOT_DR_RW, tempF);
			break;

		case CMD_SETSTEER_FRONT:
			tempI = *(uint32_t*)&((rxUnstuffedPacket[PACKET_LEN+1]));
			tempF = *((float *)(&(tempI)));
			controlSetGoal(MOT_ST_FW, tempF);
			break;

		case CMD_SETSTEER_REAR:
			tempI = *(uint32_t*)&((rxUnstuffedPacket[PACKET_LEN+1]));
			tempF = *((float *)(&(tempI)));
			controlSetGoal(MOT_ST_RW, tempF);
			break;


	}
}

void commandExtractAndProcess(void){
	uint8_t sizeIncoming = 0;
	uint8_t sizeUnstuffed = 0;
	uint32_t callTime = getTime_ms();
	uint32_t elapsedTime = 0;

	//Indicates message processing phase started
//	ledOn(LEDV2);

	//Continue until buffer empty
	while (usartBufferGetNumbytes(RX_BUFFER) != 0 &&
			elapsedTime < MAX_PROCESSING_TIME_MS){
		//Extract packet from ring buffer
		sizeIncoming = commandExtractPacketFromUsart();
		//If packet available, unstuff it with reverse COBS
		if (sizeIncoming > 0){
			sizeUnstuffed = commandUnstuffCOBS(sizeIncoming);
			//If message valid, execute the action
			if (commandCheckValidity(sizeUnstuffed)){
				commandExecuteActions(sizeUnstuffed);
			}
		}
		//Update elapsed time
		elapsedTime = getTime_ms() - callTime;
	}

	//Indicates messages processing phase ended
//	ledOff(LEDV2);
}

void commandSendPacket(uint16_t id, uint8_t cmd, uint8_t len, uint8_t args[]){
	uint32_t crc = 0;
	//The interpretable packet
	uint8_t unstuffedPacket[MAX_PACKET_SIZE];
	//The encoded packet, i.e. the one to send
	uint8_t encodedPacket[MAX_PACKET_SIZE+3];
	uint8_t sizeToSend = 0;

	//Fill the header of the packet
	unstuffedPacket[PACKET_ID_1] = (uint8_t)((id&0xFF00)>>8);
	unstuffedPacket[PACKET_ID_0] = (uint8_t)(id&0x00FF);
	unstuffedPacket[PACKET_CMD] = cmd;
	unstuffedPacket[PACKET_LEN] = len;

	//Fill the packet with arguments
	for (uint8_t i = 0; i < len; i++)
		unstuffedPacket[PACKET_LEN_BEFORE_ARGS+i] = args[i];

	//Fill the packet with the CRC code
	crc = crcComputeArray(unstuffedPacket, len+PACKET_LEN_BEFORE_ARGS);
	unstuffedPacket[PACKET_LEN_BEFORE_ARGS+len] =
				(uint8_t)((crc&0xFF000000)>>24);
	unstuffedPacket[PACKET_LEN_BEFORE_ARGS+len+1] =
				(uint8_t)((crc&0x00FF0000)>>16);
	unstuffedPacket[PACKET_LEN_BEFORE_ARGS+len+2] =
				(uint8_t)((crc&0x0000FF00)>>8);
	unstuffedPacket[PACKET_LEN_BEFORE_ARGS+len+3] =
				(uint8_t)((crc&0x000000FF)>>0);

	//Encode the packet with COBS algorithm
	commandStuffCOBS(unstuffedPacket, encodedPacket,
			len+PACKET_LEN_WITHOUT_ARGS);

	//Send the packet
	sizeToSend = len+PACKET_LEN_WITHOUT_ARGS+3;
	for (uint8_t i = 0; i < sizeToSend; i++)
		usartSendByte(encodedPacket[i]);

}

static void commandStuffCOBS(const uint8_t src[], uint8_t dst[], uint8_t size){
	uint8_t srcIndex = 0;
	uint8_t dstIndex = 1;
	//The size of the actual COBS block
	uint8_t code = COBS_DELIMITER+1;
	//The location where to place code when the end of the COBS block is reached
	uint8_t codeIndex = 1;

	//Put the starting 0
	dst[0] = COBS_DELIMITER;

	//Go through all the characters of src
	while (srcIndex < size){
		//If delimiter encountered, store code at correct location and reset
		//code
		if (src[srcIndex] == COBS_DELIMITER){
			dst[codeIndex] = code;
			dstIndex++;
			codeIndex = dstIndex;
			code = COBS_DELIMITER+1;
		}
		//Otherwise, copy src character and increment code
		else{
			dstIndex++;
			dst[dstIndex] = src[srcIndex];
			code++;
		}
		//Next src character
		srcIndex++;
	}
	//Place the code of the final COBS block and add the ending 0
	dst[codeIndex] = code;
	dst[size+2] = COBS_DELIMITER;
}

/**** END OF FILE ****/
