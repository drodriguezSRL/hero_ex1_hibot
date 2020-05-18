 /// \file    commandInterface.h
 /// \author  Alan ALLART
 /// \version V0:
 /// \brief   Header file for commandInterface.c
 ///
 /// When adding commands, please also modify #_IS_COMMAND(COMMAND) and
 /// _IS_LEN_CORRECT(CMD, LEN) consequently


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _INC_COMMANDINTERFACE_H_
#define _INC_COMMANDINTERFACE_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "common.h"
#include "usart.h"
#include "led.h"
#include "crc.h"
#include "motor_driver.h"
#include "control.h"

/* Exported constants --------------------------------------------------------*/
 /// \def MAX_PACKET_SIZE
 /// The maximum size of a packet (without delimiters)
#define MAX_PACKET_SIZE 24
 /// \def MAX_BYTE_ARGS_PACKET
 /// The max number of bytes you can have in a packet as command arguments
#define MAX_BYTE_ARGS_PACKET 16
 /// \def PACKET_LEN_WITHOUT_ARGS
 /// The total packet size if no arguments are passed via the packet
#define PACKET_LEN_WITHOUT_ARGS 8
 /// \def PACKET_LEN_BEFORE_ARGS
 /// The number of bytes before starting to have arguments
#define PACKET_LEN_BEFORE_ARGS 4
 /// \def COBS_DELIMITER
 /// The delimiter used to frame the packets
#define COBS_DELIMITER 0x00

 /// \def MAX_PROCESSING_TIME_MS
 /// The maximum time granted to the process function commandExtractAndProcess()
 /// to extract and execute the actions in the buffer
#define MAX_PROCESSING_TIME_MS 2

 /// \def PACKET_ID_1
 /// The index of the ID_1 byte in the packet
#define PACKET_ID_1 0
 /// \def PACKET_ID_0
 /// The index of the ID_0 byte in the packet
#define PACKET_ID_0 1
 /// \def PACKET_CMD
 /// The index of the CMD byte in the packet
#define PACKET_CMD 2
 /// \def PACKET_LEN
 /// The index of the LEN byte in the packet
#define PACKET_LEN 3

 /// \def CMD_LEDR1_ON
 /// The command to turn on the 1st red LED
#define CMD_LEDR1_ON 0x01
 /// \def CMD_LEDR1_OFF
 /// The command to turn off the 1st red LED
#define CMD_LEDR1_OFF 0x02
 /// \def CMD_SETSPEED_FRONT
 /// The command to set the front wheel driving speed
#define CMD_SETSPEED_FRONT 0x03
 /// \def CMD_SETSPEED_REAR
 /// The command to set the rear wheel driving speed
#define CMD_SETSPEED_REAR 0x04
 /// \def CMD_SETSTEER_FRONT
 /// The command to set the front wheel steering angle
#define CMD_SETSTEER_FRONT 0x05
 /// \def CMD_SETSTEER_REAR
 /// The command to set the rear wheel steering angle
#define CMD_SETSTEER_REAR 0x06

#define VAL_SPEED_FRONT 0x07

#define VAL_SPEED_REAR 0x08

#define VAL_STEER_FRONT 0x09

#define VAL_STEER_REAR 0x0A

#define VAL_CUR_FRONT 0x0B

#define VAL_CUR_REAR 0x0C

#define VAL_ANG_RCK 0x0D

 /// \def LEN_LEDR1_ON
 /// The number of byte arguments associated with the #CMD_LEDR1_ON command
#define LEN_LEDR1_ON 0
 /// \def LEN_LEDR1_OFF
 /// The number of byte arguments associated with the #CMD_LEDR1_OFF command
#define LEN_LEDR1_OFF 0
 /// \def LEN_SETSPEED_FRONT
 /// The number of byte arguments associated with the #CMD_SETSPEED_FRONT command
#define LEN_SETSPEED_FRONT 4
 /// \def LEN_SETSPEED_REAR
 /// The number of byte arguments associated with the #CMD_SETSPEED_REAR command
#define LEN_SETSPEED_REAR 4
 /// \def LEN_SETSTEER_FRONT
 /// The number of byte arguments associated with the #CMD_SETSTEER_FRONT command
#define LEN_SETSTEER_FRONT 4
 /// \def LEN_SETSTEER_REAR
 /// The number of byte arguments associated with the #CMD_SETSTEER_REAR command
#define LEN_SETSTEER_REAR 4

#define LEN_VAL_SPEEDFRONT 4

#define LEN_VAL_SPEEDREAR 4

#define LEN_VAL_STEERFRONT 4

#define LEN_VAL_STEERREAR 4

#define LEN_VAL_CURFRONT 2

#define LEN_VAL_CURREAR 2

#define LEN_VAL_ANGRCK 4

 /// \def _IS_COMMAND(COMMAND)
 /// Return 1 if COMMAND is in the command list, else 0
#define _IS_COMMAND(COMMAND)    (((COMMAND) == CMD_LEDR1_ON) || \
								((COMMAND) == CMD_LEDR1_OFF) || \
								((COMMAND) == CMD_SETSPEED_FRONT) || \
								((COMMAND) == CMD_SETSPEED_REAR) || \
								((COMMAND) == CMD_SETSTEER_FRONT) || \
								((COMMAND) == CMD_SETSTEER_REAR)      )
 /// \def _IS_LEN_CORRECT(CMD, LEN)
 /// Return 1 if LEN matches the byte arguments number needed with CMD
#define _IS_LEN_CORRECT(CMD, LEN) \
			((((CMD) == CMD_LEDR1_ON) && ((LEN) == LEN_LEDR1_ON)) || \
 			 (((CMD) == CMD_LEDR1_OFF) && ((LEN) == LEN_LEDR1_OFF)) || \
			 (((CMD) == CMD_SETSTEER_FRONT) && ((LEN) == LEN_SETSTEER_FRONT))	|| \
			 (((CMD) == CMD_SETSTEER_REAR) && ((LEN) == LEN_SETSTEER_REAR))	|| \
			 (((CMD) == CMD_SETSPEED_FRONT) && ((LEN) == LEN_SETSPEED_FRONT)) || \
			 (((CMD) == CMD_SETSPEED_REAR) && ((LEN) == LEN_SETSPEED_REAR)) )

/* Exported types ------------------------------------------------------------*/

/* Functions prototypes ------------------------------------------------------*/
 /// \brief Decode a COBS encoded packet
 /// \return the size of the decoded packet. 0 if size error
 /// \param packetSize the size of the current extracted packet
 ///
 /// Requires prior packet extraction using commandExtractPacketFromUsart().
 /// Read in the #rxIncomingPacket buffer and write the decoded packet in
 /// the #rxUnstuffedPacket buffer.
 ///
 /// Size error means overflow of the buffer when going through the COBS blocks.
 /// Probably means corrupted data
static uint8_t commandUnstuffCOBS(uint8_t packetSize);

 /// \brief Handle #rxBuffer to detect incoming packets and stores them
 /// \return the length of the packet stored. 0 if no packet available yet
 /// or if packet discarded
 ///
 /// Stores the incoming packet without #COBS_DELIMITER in #rxIncomingPacket
 /// and rewrite in this buffer if recalled (so data is lost if not already
 /// processed)
 ///
 /// Packets can be discarded for several reasons:
 /// 	- Packet too long ( > #MAX_PACKET_SIZE+1 )
 /// 	- 2 consecutive #COBS_DELIMITER
 ///
 /// This function automatically empties the buffer if usart started during a
 /// reception and auto re-synchronise if lost packets.
static uint8_t commandExtractPacketFromUsart(void);

 /// \brief check if a packet respect certain validity criteria
 /// \param size the length of stored packet
 /// \return 0 if the packet contains an error, 1 if it may be correct
 ///
 /// The criteria are the following:
 /// 	- CMD byte in the commands list
 /// 	- LEN byte matches with the overall packet size
 /// 	- LEN byte matches with the number of byte arguments needed for CMD
 /// 	- CRC32 validation
 ///
 /// This function verify the validity of the packet present in
 /// #rxUnstuffedPacket
static uint8_t commandCheckValidity(uint8_t size);

 /// \brief Given a correct packet, executes actions asked by this packet
 /// \param size the size of the current correct packet to interpret
 ///
 /// This function take the unstuffed packet in #rxUnstuffedPacket. Take care
 /// to correctly check that the packet respect all the security criteria with
 /// commandCheckValidity before executing the command stored in the buffer
static void commandExecuteActions(uint8_t size);

 /// \brief empties the #rxBuffer and process incoming packets
 ///
 /// This function takes all in hands to process and interpret incoming
 /// packets. It calls forth all the needed functions and does all the security
 /// checks, and then execute the action. It does that until the #rxBuffer is
 /// empty, or until the maximum processing time defined by
 /// #MAX_PROCESSING_TIME_MS has elapsed
void commandExtractAndProcess(void);

 /// \brief Build and send a packet to the master through UART
 /// \param id the ID of the packet to build
 /// \param cmd the code of the corresponding command
 /// \param len the number of bytes in the arguments (i.e. the size of args[])
 /// \param args[] an array filled with all the byte arguments
 ///
 /// This function calculate CRC herself and add herself the needed ending and
 /// starting #COBS_DELIMITER.
void commandSendPacket(uint16_t id, uint8_t cmd, uint8_t len, uint8_t args[]);

 /// \brief COBS encodes a byte array
 /// \param src[] the source array (to be encoded)
 /// \param dst[] the location where to write the encoded result
 /// \param size the size of src[]
 ///
 /// Please ensure dst[] is large enough to avoid overflow and data corruption
static void commandStuffCOBS(const uint8_t src[], uint8_t dst[], uint8_t size);

#ifdef __cplusplus
}
#endif

#endif /* _INC_COMMANDINTERFACE_H_ */

 /**** END OF FILE ****/
