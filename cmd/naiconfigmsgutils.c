/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include "cmd_naiconfigmsgutils.h"
#include "cmd_naimsgutils.h"

#ifdef __BAREMETAL
#include "alt_clock_manager.h"
#include "designware_i2c.h"
#endif

#if defined(__BAREMETAL) || defined(__LINUX) || defined (__CYGWIN)
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#elif __UBOOT
#include <common.h>
#include <malloc.h>
#include <crc.h>
#include <i2c.h>
#endif

#ifdef __LINUX
#include <unistd.h>

	//////////////////////
	#include "socal/socal.h"
	#include "socal/hps.h"
#endif

/* #define _DF3_VERBOSE 1 */

#define MAX_EEPROM_BYTES 0x100 /* 256 Bytes */
#define MAX_FLASH_PAGE_BYTES 0x10000 /*65536 Bytes */

#ifdef _MICROCONTROLLER
	/* Isolated DT Globals */
	#define DT_COMPLETION_TIMEOUT 1000  /*In milliseconds*/

	#define NUM_DT_CHANNELS 16
	#define MAX_ISOLATED_DT_WRITE_BUFFER_IN_BYTES 128 /*Make sure write buffer is as large as largest capable write in bytes of all microcontrollers*/
	#define MAX_ISOLATED_DT_ST_WRITE_IN_BYTES 128   /*ST Microcontroller can send 128 bytes at a clip*/
	#define MAX_ISOLATED_DT_KL17_WRITE_IN_BYTES 32  /*Each KL17 Data Packet can hold a max of 32 bytes of data*/
	#define MAX_ISOLATED_DT_READ_IN_BYTES  255
	#define MAX_ISOLATED_DT_TX_RAM_IN_BYTES 255

	const int32_t CHANNEL_SELECT_REG = 0x00000000;
	const int32_t SERIAL_CONTROL_REG = 0x00000004;
	const int32_t SERIAL_STATUS_REG = 0x00000008;
	const int32_t CHANNEL_PS_ENABLE = 0x00000010;
	const int32_t SERIAL_BAUDRATE_REG = 0x00000014;
	const int32_t TX_DATA_COUNT = 0x0000000C;
	const int32_t TX_RAM_START = 0x00001000;
	const int32_t RX_RAM_START = 0x00002000;
	const int32_t MODULE_COMMON_RESET_UART_REG = 0x00001024;
	const uint8_t ALL_CHANNELS = 0xFF;

	#define NO_MICROCONTROLLER_TYPE   0
	#define KL17_MICROCONTROLLER_TYPE 1
	#define ST_MICROCONTROLLER_TYPE   2

	/* STM Commands */
	const uint8_t STM_GET_CMD 		= 0x00;
	const uint8_t STM_READ_MEM_CMD 	= 0x11;
	const uint8_t STM_GO_CMD 		= 0x21;
	const uint8_t STM_WRITE_MEM_CMD = 0x31;
	const uint8_t STM_ERASE_MEM_CMD = 0x43;

	const uint8_t DT2_FORCE_BOOTLOADER = 0x7F; /* DT2 command to force ST into bootloader mode */
	const uint8_t STM_ACK  			= 0x79; /* ST Microcontroller ACK code */
	const uint8_t STM_NACK 			= 0x1F; /* ST Microcontroller NACK code */
	const uint8_t STM_SYNCH 		= 0x7F; /* ST Microcontroller SYNCH code */
	const int32_t STM_FLASH_OFFSET  = 0x8000;	

	/* STM Specific Functionality */
	static int32_t nai_st_put_in_config_mode(uint8_t ucChanIndex);

	/* KL17 COMMANDS */
	const uint8_t KL17_START_BYTE	= 0x5A;
	const int32_t KL17_FLASH_OFFSET = 0x00;

	/* KL17 PACKET TYPES */
	const uint8_t KL17_PACKET_TYPE_ACK				= 0xA1;
	const uint8_t KL17_PACKET_TYPE_NAK				= 0xA2;
	const uint8_t KL17_PACKET_TYPE_ACK_ABORT		= 0xA3;
	const uint8_t KL17_PACKET_TYPE_COMMAND			= 0xA4;
	const uint8_t KL17_PACKET_TYPE_DATA				= 0xA5;
	const uint8_t KL17_PACKET_TYPE_PING				= 0xA6;
	const uint8_t KL17_PACKET_TYPE_PING_RESPONSE 	= 0xA7;

	/* KL17 Commands */
	const uint8_t KL17_FLASH_ERASE_ALL_CMD			= 0x01;
	const uint8_t KL17_FLASH_ERASE_REGION_CMD		= 0x02;
	const uint8_t KL17_READ_MEMORY_CMD				= 0x03;
	const uint8_t KL17_WRITE_MEMORY_CMD				= 0x04;
	const uint8_t KL17_RESERVED1_CMD				= 0x05;
	const uint8_t KL17_FLASH_SECURITY_DISABLE_CMD	= 0x06;
	const uint8_t KL17_GET_PROPERTY_CMD				= 0x07;
	const uint8_t KL17_RESERVED2_CMD				= 0x08;
	const uint8_t KL17_EXECUTE_CMD					= 0x09;
	const uint8_t KL17_RESERVED3_CMD				= 0x0A;
	const uint8_t KL17_RESET_CMD					= 0x0B;
	const uint8_t KL17_SET_PROPERTY_CMD				= 0x0C;
	const uint8_t KL17_FLASH_ERASE_ALL_UNSECURE_CMD = 0x0D;
	const uint8_t KL17_RESERVED4_CMD				= 0x0E;
	const uint8_t KL17_RESERVED5_CMD				= 0x0F;
	const uint8_t KL17_RESERVED6_CMD				= 0x10;
	const uint8_t KL17_RESERVED7_CMD				= 0x11;

	/* KL17 Responses */
	const uint8_t KL17_GENERIC_RESPONSE				= 0xA0;
	const uint8_t KL17_GET_PROPERTY_RESPONSE		= 0xA7;
	const uint8_t KL17_READ_MEMORY_RESPONSE			= 0xA3;

	/* KL17 Specific Functionality */
	static uint16_t nai_kl17_crc16_update(const uint8_t * src, uint32_t lengthInBytes);
	static int32_t nai_kl17_put_in_config_mode(uint8_t ucChanIndex);
	static int32_t nai_kl17_send_ping(uint8_t ucChanIndex);
	static int32_t nai_kl17_write_memory_cmd(uint8_t ucChanIndex, uint32_t unStartAddress, uint32_t unByteCount);
	static int32_t nai_kl17_write_memory_data(uint8_t ucChanIndex, uint8_t *pucWriteBuf, uint8_t ucByteCount);
	static int32_t nai_kl17_write_memory_data_get_final_response(uint8_t ucChanIndex);
	static int32_t nai_kl17_perform_erase(uint8_t ucChanIndex);
	static int32_t nai_kl17_wait_for_ack(void);
	static int32_t nai_kl17_send_ack(uint8_t ucChanIndex);
	static int32_t nai_kl17_get_version_and_supported_functions(uint8_t ucChanIndex, uint8_t *pucDataBuf, uint8_t *pucByteCount);
	static int32_t nai_kl17_get_generic_response(uint8_t ucCmd, uint8_t ucCRC16Low, uint8_t ucCRC16High, uint32_t *punStatusCode);
	static int32_t nai_kl17_get_property_response(uint32_t *punStatusCode, uint8_t ucNumProperties, uint8_t *pucPropertyValues);
	
	uint8_t g_MicrocontrollerType = KL17_MICROCONTROLLER_TYPE;

	const uint32_t DT2_12M_BAUD_RATE = 0x189374BC;
	const uint32_t DT2_OPERATIONAL_1M_BAUD_RATE = 0x020C49BA;
	const uint32_t DT2_CONFIGURATION_115_2K_BAUD_RATE = 0x003C65E1;
	const uint32_t DT2_ST_DEFAULT_57_6K_BAUD_RATE = 0x001E32F0;
	const uint32_t DT2_ST_DEFAULT_9600_BAUD_RATE = 0x0005087D;

	const uint8_t DT2_CTRL_UART = 0x00;
	const uint8_t DT2_CTRL_TX_GO = 0x02;
	const uint8_t DT2_CTRL_RESET_RX_RAM = 0x04;
	const uint8_t DT2_CTRL_RESET_TX_RAM = 0x08;
	const uint8_t DT2_STAT_RX_RAM_OK_TO_READ = 0x01;
	const uint8_t DT2_STAT_TX_RAM_TRANSMITTED = 0x02;

	/* Microcontroller Common Functions */
	static uint8_t nai_get_micro_controller_type(void);
	static uint16_t nai_get_max_packet_write_in_bytes(uint8_t ucMicrocontrollerType);
	static int32_t nai_put_isolated_dt_in_config_mode(uint8_t ucChanIndex);
	static int32_t nai_send_data_isolated_dt(uint8_t ucChanIndex, uint8_t *pucDataBuf, uint8_t ucByteCount);
	static int32_t nai_write_memory_isolated_dt(uint8_t ucChanIndex, uint32_t unStartAddress, uint8_t *pucWriteBuf, uint8_t ucByteCount);
	static int32_t nai_perform_erase_isolated_dt(uint8_t ucChanIndex);
	static int32_t nai_config_isolated_dt(MsgPacketList *ptMsgPacketList);
	static int32_t nai_get_isolated_dt(MsgPacketList *ptMsgPacketList);
	static int32_t nai_erase_isolated_dt(MsgPacketList *ptMsgPacketList);
	static int32_t nai_get_version_and_supported_functions_isolated_dt(uint8_t ucChanIndex, uint8_t *pucDataBuf, uint8_t *pucByteCount);

	/* ST Microcontroller Specific Functions */
	static int32_t nai_st_send_cmd(uint8_t ucChanIndex, uint8_t ucCmd);
	static int32_t nai_st_write_memory_data(uint8_t ucChanIndex, uint32_t unStartAddress, uint8_t *pucWriteBuf, uint8_t ucByteCount);
	static int32_t nai_st_perform_erase(uint8_t ucChanIndex);
	static int32_t nai_st_receive_data(uint8_t ucChanIndex, uint8_t *pucDataBuf, uint8_t *pucByteCount);
	static int32_t nai_st_wait_for_ack(void);
	static int32_t nai_st_get_version_and_supported_functions(uint8_t ucChanIndex, uint8_t *pucDataBuf, uint8_t *pucByteCount);

	static BOOL g_bAlreadyInBootloader[NUM_DT_CHANNELS] = {FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE};
#endif /* _MICROCONTROLLER */

static uint32_t nai_get_completion_timeout(MsgPacket *ptMsgPacket);
	
static uint16_t g_usTranID = 1;
static BOOL g_bLock = FALSE;

extern uint16_t g_usMaxSlotCount;
extern uint8_t g_ucSlotID;
BOOL g_bExitConfigMode; /* Also used in cmd_naireceive.c - governs when to exit Config Mode */

/* We provide higher level calls so no need to expose the inner workings of actual read or write requests */
static int32_t make_read_request(uint16_t usCommandType, uint8_t ucRequesterID, uint8_t ucCompleterID, uint16_t usChipID, uint unOffset, uint8_t *pucBuf, int32_t nLen);
static int32_t make_write_request(uint16_t usCommandType, uint8_t ucRequesterID, uint8_t ucCompleterID, uint16_t usChipID, uint unEepromOffset, uint8_t *pucBuf, int32_t nLen);
static int32_t make_generic_zero_payload_cmd_request(uint16_t usCommandType, uint8_t ucRequesterID, uint8_t ucCompleterID);

#ifdef __UBOOT
static int32_t nai_debug_test(void);
#endif

static uint8_t pucTempBuf[MAX_FLASH_PAGE_BYTES];

/**************************************************************************************************************/
/* NAI Message Utility Command Routines                                                                       */
/** \defgroup MessageUtils Message Utility Functions                                                          */
/**************************************************************************************************************/

/**************************************************************************************************************/
/**
\ingroup MessageUtils
<summary>
aligned_malloc is responsible for allocating memory suitable for the given alignment request. (Typically this
is interchangeable with malloc under x86 systems with 4 byte alignment...we call this to ensure we are indeed
aligned to the requested boundary.) 
</summary>
<param name="size"> : (Input) amount of memory in bytes to allocate</param>
<param name="alignment"> : (Input) defines the byte boundary upon which to allocate memory - must be a power of 2</param>
<returns>VOID</returns>
*/
/**************************************************************************************************************/
void* aligned_malloc(size_t size, size_t alignment) {

    uintptr_t r = (uintptr_t)malloc(size + --alignment + sizeof(uintptr_t));
    uintptr_t t = r + sizeof(uintptr_t);
    uintptr_t o =(t + alignment) & ~(uintptr_t)alignment;

    if (!r) 
		return NULL;

    ((uintptr_t*)o)[-1] = r;

    return (void*)o;
}

/**************************************************************************************************************/
/**
\ingroup MessageUtils
<summary>
aligned_free is responsible for freeing memory allocated with the call "aligned_malloc". 
</summary>
<param name="p"> : (Input/output) pointer to free</param>
<returns>VOID</returns>
*/
/**************************************************************************************************************/
void aligned_free(void* p) {

    if (!p) 
		return;

    free((void*)(((uintptr_t*)p)[-1]));
}

/**************************************************************************************************************/
/**
\ingroup MessageUtils
<summary>
get_next_tran_id is responsible for returning the next available transaction ID.
</summary>
<returns> uint16_t : Transaction ID</returns>
*/
/**************************************************************************************************************/
static uint16_t get_next_tran_id(void)
{
	static uint16_t usTranID = 0;
	if (!g_bLock)
	{
		g_bLock = TRUE;
		g_usTranID++;
		
		if (g_usTranID >= 32767)
			g_usTranID = 1;
		usTranID = g_usTranID;
		g_bLock = FALSE;
	}

	return usTranID;
}

/**************************************************************************************************************/
/* NAI Message Creation Routines                                                                              */
/** \defgroup MessageCreation Message Creation Functions                                                      */
/**************************************************************************************************************/
/**************************************************************************************************************/
/**
\ingroup MessageCreation
<summary>
create_nai_msg_packet is responsible for creating a new MsgPacket struct from the word array passed in. 
</summary>
<param name="pusData"> : (Input) Buffer of data representing an entire message (headers and payload)</param>
<returns> MsgPacket* : Pointer to the newly created MsgPacket struct</returns>
*/
/**************************************************************************************************************/
MsgPacket * create_nai_msg_packet(uint16_t* pusData)
{
//	MsgPacket *ptNewMsgPacket = malloc(sizeof(MsgPacket));
	MsgPacket *ptNewMsgPacket = aligned_malloc(sizeof(MsgPacket), 4);

#ifdef _DEBUG_X
	int32_t i=0;

	for (i=0; i < MAX_SERDES_MSG_IN_WORDS; i++)
		printf("usData[%d] = 0x%4.4x\r\n", i, pusData[i]);
#endif
	memcpy(ptNewMsgPacket->tNAIMsg.msg, pusData, (MAX_SERDES_MSG_IN_WORDS*2));
	ptNewMsgPacket->ptNext = NULL;

	return ptNewMsgPacket;
}

/**************************************************************************************************************/
/**
\ingroup MessageCreation
<summary>
clone_nai_msg_packet_sans_payload is responsible for making a copy of a MsgPacket struct without the actual 
payload.
</summary>
<param name="ptMsgPacket"> : (Input) Pointer to MsgPacket struct from which to make a clone.</param>
<returns> MsgPacket* : Pointer to the newly created MsgPacket struct</returns>
*/
/**************************************************************************************************************/
MsgPacket * clone_nai_msg_packet_sans_payload(MsgPacket *ptMsgPacket)
{
//	MsgPacket *ptNewMsgPacket = malloc(sizeof(MsgPacket));
	MsgPacket *ptNewMsgPacket = aligned_malloc(sizeof(MsgPacket), 4);

	/* Make a copy of the Msg Packet passed in */
	memcpy(ptNewMsgPacket->tNAIMsg.msg, ptMsgPacket->tNAIMsg.msg, (MAX_SERDES_MSG_IN_WORDS*2));
	ptNewMsgPacket->ptNext = NULL;

	/* Clear out payload */
	memset(&(ptNewMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandPayLd.usData[0]), 0, (CONFIG_MAX_PAYLOAD_IN_WORDS*2)); 

	return ptNewMsgPacket;
}

/**************************************************************************************************************/
/**
\ingroup MessageCreation
<summary>
create_nai_msg_packet_list is responsible for creating a new (empty) MsgPacketList.
</summary>
<returns> MsgPacketList* : Pointer to the newly created MsgPacketList struct</returns>
*/
/**************************************************************************************************************/
MsgPacketList * create_nai_msg_packet_list()
{
//	MsgPacketList *ptNewMsgPacketList = malloc(sizeof(MsgPacketList));
	MsgPacketList *ptNewMsgPacketList = aligned_malloc(sizeof(MsgPacketList), 4);
		
	ptNewMsgPacketList->ptStart = NULL;
	ptNewMsgPacketList->ptEnd = ptNewMsgPacketList->ptStart;
	ptNewMsgPacketList->ptNext = NULL;
	ptNewMsgPacketList->nCount = 0;

	return ptNewMsgPacketList;
}

/**************************************************************************************************************/
/**
\ingroup MessageCreation
<summary>
find_nai_msg_packet_list is responsible for finding a given message packet list (list of packets that make up a 
single specified message) based on a specified transaction ID. NOTE: All packets of a given msg packet list will 
have the same ID!
</summary>
<param name="ptMsgList"> : (Input) Pointer to MsgList struct (structure containing 1 or more messages).</param>
<param name="usTranID"> : (Input) Transaction identifier indicating a specific message.</param>
<returns> MsgPacketList* : Pointer to the MsgPacketList object containing the list of packets that make up the 
message for the specified transaction ID</returns>
*/
/**************************************************************************************************************/
MsgPacketList * find_nai_msg_packet_list(MsgList *ptMsgList, uint16_t usTranID)
{
	MsgPacketList *ptTraverse = ptMsgList->ptStart;

	while (ptTraverse != NULL)
	{					
		if (ptTraverse->ptStart->tNAIMsg.tSerdesPayLd.tTransportHdr.usID == usTranID)
		{
#ifdef _VERBOSE
			printf("Found Transport ID: 0x%4x\r\n", ptTraverse->ptStart->tNAIMsg.tSerdesPayLd.tTransportHdr.usID);
#endif
			break;
		}
		ptTraverse = ptTraverse->ptNext;
	}	

	return ptTraverse;
}

/**************************************************************************************************************/
/**
\ingroup MessageCreation
<summary>
init_nai_msgs is responsible for initializing all of the fields of a specific MsgList struct.
</summary>
<param name="ptMsgList"> : (Input) Pointer to MsgList struct (structure containing 1 or more messages).</param>
<returns>None</returns>
*/
/**************************************************************************************************************/
void init_nai_msgs(MsgList *ptMsgList)
{
	ptMsgList->ptStart = NULL;
	ptMsgList->ptEnd = ptMsgList->ptStart;
	ptMsgList->nCount = 0;	
}

/**************************************************************************************************************/
/**
\ingroup MessageCreation
<summary>
create_and_append_nai_msg_packet is responsible for creating a new MsgPacket for the specified message data and
appending it to the supplied MsgPacketList.
</summary>
<param name="ptMsgPacketList"> : (Input) Pointer to MsgPacketList struct (structure containing 1 or more message 
packets).</param>
<param name="pusData"> : (Input) Data buffer containing all of the data making up a single message packet.</param>
<returns>None</returns>
<seealso cref="create_nai_msg_packet">
*/
/**************************************************************************************************************/
void create_and_append_nai_msg_packet(MsgPacketList *ptMsgPacketList, uint16_t* pusData)
{	
	MsgPacket *ptNewMsgPacket = create_nai_msg_packet(pusData);

#ifdef _VERBOSE
	printf("Trans ID: %u Adding New Msg Packet - Sequence #: %u\r\n", 
				ptNewMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usID, 
				ptNewMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usSequenceNum);
#endif

	if (ptMsgPacketList->ptEnd == NULL)
	{
		ptMsgPacketList->ptStart = ptNewMsgPacket;
	  	ptMsgPacketList->ptEnd = ptMsgPacketList->ptStart;
	}
	else
	{
		ptMsgPacketList->ptEnd->ptNext = ptNewMsgPacket;
		ptMsgPacketList->ptEnd = ptNewMsgPacket;
	}

	ptMsgPacketList->nCount++;
}

/**************************************************************************************************************/
/**
\ingroup MessageCreation
<summary>
delete_nai_msg_packets is responsible for removing all of the MsgPacket structs from the specified MsgPacketList.
</summary>
<param name="ptMsgPacketList"> : (Input) Pointer to MsgPacketList struct (structure containing 1 or more message 
packets).</param>
<returns>None</returns>
*/
/**************************************************************************************************************/
void delete_nai_msg_packets(MsgPacketList *ptMsgPacketList)
{
	MsgPacket *ptTraverse = NULL;	
	
	while (ptMsgPacketList->ptStart != NULL)
	{
		ptTraverse = ptMsgPacketList->ptStart;
		ptMsgPacketList->ptStart = ptMsgPacketList->ptStart->ptNext;		
//		free(ptTraverse);
		aligned_free(ptTraverse);
		ptMsgPacketList->nCount--;			
	}
}

/**************************************************************************************************************/
/**
\ingroup MessageCreation
<summary>
add_nai_msg is responsible for adding the specified MsgPacketList (all packets for a given message) to the
specified MsgList (list of MsgPacketList).
</summary>
<param name="ptMsgList"> : (Input) Pointer to MsgList struct (structure containing 1 or more messages).</param>
<param name="ptMsgPacketList"> : (Input) Pointer to MsgPacketList struct (structure containing 1 or more message 
packets).</param>
<returns>None</returns>
*/
/**************************************************************************************************************/
void add_nai_msg(MsgList *ptMsgList, MsgPacketList *ptMsgPacketList)
{
#ifdef _VERBOSE
	printf("Adding New Msg - Num of Packets: %u\r\n", ptMsgPacketList->ptEnd->tNAIMsg.tSerdesPayLd.tTransportHdr.usSequenceNum);
#endif

	if (ptMsgList->ptEnd == NULL)
	{
		ptMsgList->ptStart = ptMsgPacketList;
	  	ptMsgList->ptEnd = ptMsgList->ptStart;
	}
	else
	{
		ptMsgList->ptEnd->ptNext = ptMsgPacketList;
		ptMsgList->ptEnd = ptMsgPacketList;
	}

	ptMsgList->nCount++;
}

/**************************************************************************************************************/
/**
\ingroup MessageCreation
<summary>
delete_nai_msgs is responsible for deleting all MsgPacketList references for the specified MsgList.
</summary>
<param name="ptMsgList"> : (Input) Pointer to MsgList struct (structure containing 1 or more messages).</param>
packets).</param>
<returns>None</returns>
*/
/**************************************************************************************************************/
void delete_nai_msgs(MsgList *ptMsgList)
{
	MsgPacketList *ptTraverse = NULL;
	
	while (ptMsgList->ptStart != NULL)
	{
		ptTraverse = ptMsgList->ptStart;
		ptMsgList->ptStart = ptMsgList->ptStart->ptNext;
		delete_nai_msg_packets(ptTraverse);
//		free(ptTraverse);
		aligned_free(ptTraverse);
		ptMsgList->nCount--;
	}
}

/**************************************************************************************************************/
/* NAI Print Message Utility Command Routines                                                                 */
/** \defgroup PrintMessage Print Message Utility Functions                                                    */
/**************************************************************************************************************/

/**************************************************************************************************************/
/**
\ingroup PrintMessage
<summary>
print_nai_msgs is responsible for printing out information for all messages currently stored in the specified
MsgList.
</summary>
<param name="ptMsgList"> : (Input) Pointer to MsgList struct (structure containing 1 or more messages).</param>
<param name="bPrintPayLd"> : (Input) Specified whether or not to print the payload for each message.</param>
<returns>None</returns>
<seealso cref="print_nai_msg">
*/
/**************************************************************************************************************/
void print_nai_msgs(MsgList *ptMsgList, BOOL bPrintPayLd)
{
	MsgPacketList *ptTraverse = ptMsgList->ptStart;

	printf("\r\n******************************************************************\r\n");
	printf("Printing NAI Messages\r\n");
	printf("Total Num of Messages In Memory: %d\r\n", (int)ptMsgList->nCount);

	while (ptTraverse != NULL)
	{
		print_nai_msg(ptTraverse, bPrintPayLd);
		ptTraverse = ptTraverse->ptNext;
	}
	printf("\r\n******************************************************************\r\n");
}

/**************************************************************************************************************/
/**
\ingroup PrintMessage
<summary>
print_nai_msg is responsible for printing out information for a given MsgPacketList (list of packets making up 
a single message).
</summary>
<param name="ptMsgPacketList"> : (Input) Pointer to MsgPacketList struct (structure containing 1 or more message 
packets).</param>
<param name="bPrintPayLd"> : (Input) Specified whether or not to print the payload for each message.</param>
<returns>None</returns>
<seealso cref="print_nai_msg_payload">
*/
/**************************************************************************************************************/
void print_nai_msg(MsgPacketList *ptMsgPacketList, BOOL bPrintPayLd)
{
	if (ptMsgPacketList == NULL)
		return;

	MsgPacket *ptTraverse = ptMsgPacketList->ptStart;	
	int32_t nMsgPacketCount = 0;
	
	printf("\r\n******************************************************************\r\n");
	printf("Printing Msg Contents - Total Num Packets in Msg = %d\r\n", (int)ptMsgPacketList->nCount);

	while (ptTraverse != NULL)
	{
		nMsgPacketCount++;
		printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\r\n");
		printf("Msg Packet #:     %d\r\n", (unsigned int)nMsgPacketCount);
		printf("Serdes CmdType:   0x%4.4x - (%u)\r\n", ptTraverse->tNAIMsg.tSerdesHdr.ucType, ptTraverse->tNAIMsg.tSerdesHdr.ucType);
		printf("Serdes Length:    0x%4.4x - (%u)\r\n", (ptTraverse->tNAIMsg.tSerdesHdr.ucPayloadLength), (ptTraverse->tNAIMsg.tSerdesHdr.ucPayloadLength));
		printf("Transport ID:     0x%4.4x - (%u)\r\n", ptTraverse->tNAIMsg.tSerdesPayLd.tTransportHdr.usID, ptTraverse->tNAIMsg.tSerdesPayLd.tTransportHdr.usID);
		printf("Transport Length: 0x%8.4x - (%u)\r\n", (unsigned int)ptTraverse->tNAIMsg.tSerdesPayLd.tTransportHdr.unMsgLength, (unsigned int)ptTraverse->tNAIMsg.tSerdesPayLd.tTransportHdr.unMsgLength);
		printf("Transport Seq #:  0x%4.4x - (%u)\r\n", ptTraverse->tNAIMsg.tSerdesPayLd.tTransportHdr.usSequenceNum, ptTraverse->tNAIMsg.tSerdesPayLd.tTransportHdr.usSequenceNum);
		printf("Transport Expected Seq Count:  0x%4.4x - (%u)\r\n", ptTraverse->tNAIMsg.tSerdesPayLd.tTransportHdr.usExpectedSequenceCount, ptTraverse->tNAIMsg.tSerdesPayLd.tTransportHdr.usExpectedSequenceCount);
		printf("Msg CRC: 0x%8.4x - (%u)\r\n", (unsigned int)ptTraverse->tNAIMsg.tSerdesPayLd.tTransportHdr.unCRC, (unsigned int)ptTraverse->tNAIMsg.tSerdesPayLd.tTransportHdr.unCRC);

		if (bPrintPayLd)
			print_nai_msg_payload(ptTraverse);

		printf("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\r\n");

		ptTraverse = ptTraverse->ptNext;
	}	
	printf("******************************************************************\r\n");
}

/**************************************************************************************************************/
/**
\ingroup PrintMessage
<summary>
print_nai_msg_payload is responsible for printing out the payload information for a given MsgPacket.
</summary>
<param name="ptMsgPacket"> : (Input) Pointer to MsgPacket struct (may be partial message if message size is 
greater than packet size.</param>
<returns>None</returns>
<seealso cref="convert_bytes_to_words">
*/
/**************************************************************************************************************/
void print_nai_msg_payload(MsgPacket *ptMsgPacket)
{
	int32_t i = 0;
	uint16_t usPayloadLengthInWords = 0;

	if (ptMsgPacket != NULL)
	{
		usPayloadLengthInWords = convert_bytes_to_words(ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength);
		printf("Payload Length (in WORDS): %u\r\n", usPayloadLengthInWords);
		
		for (i=0; i < usPayloadLengthInWords; i++)		
			printf("Msg Payload[%d] = %4.4x\r\n", (unsigned int)i, (unsigned short)ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandPayLd.usData[i]);	
	}
}


/**************************************************************************************************************/
/* NAI Message Validation Routines                                                                            */
/** \defgroup MessageValidation Message Validation Functions                                                  */
/**************************************************************************************************************/

/**************************************************************************************************************/
/**
\ingroup MessageValidation
<summary>
compute_nai_msg_crc is responsible for computing the CRC for an entire message (possibly over multiple packets).
</summary>
<param name="ptMsgPacketList"> : (Input) Pointer to MsgPacketList struct (structure containing 1 or more message 
packets).</param>
<returns>uint32_t : computed CRC</returns>
*/
/**************************************************************************************************************/
uint32_t compute_nai_msg_crc(MsgPacketList *ptMsgPacketList)
{
	uint32_t unCRC = 0;

#ifdef _COMPUTE_CRC
	MsgPacket *ptTraverse = ptMsgPacketList->ptStart;
	NAIMsg *ptMsgCopy = NULL;
//	ptMsgCopy = (NAIMsg *)malloc(sizeof(NAIMsg));
	ptMsgCopy = (NAIMsg *)aligned_malloc(sizeof(NAIMsg), 4);

	while (ptTraverse != NULL)
	{	
		/* Let's make sure our buffer is all zero to start */
		memset(ptMsgCopy->msg, 0, sizeof(NAIMsg));

		/* Make a copy of the message we want to perform a CRC on...we do this because we need to 
           strip some information that is ok to change during transit before we compute the CRC */
		memcpy(ptMsgCopy->msg, ptTraverse->tNAIMsg.msg, sizeof(NAIMsg));
		ptMsgCopy->tSerdesHdr.usSERDES0 &= 0x3F; /* This zeros out SeqNumRx (bit 7) and SeqNumTx (bit 6) */
		ptMsgCopy->tSerdesHdr.usSERDES4 &= 0x0F; /* This zeros out COMPLETER_ID (bits 4 - 7) */
		ptMsgCopy->tSerdesPayLd.tTransportHdr.unCRC = 0; /* Do not take CRC into account when computing CRC (since it is not present when we 1st calculate CRC */

		unCRC = crc32(unCRC, (void *)ptMsgCopy->msg, ptMsgCopy->tSerdesHdr.ucPayloadLength);
		ptTraverse = ptTraverse->ptNext;
	}
#else
#ifdef _VERBOSE
	printf("***COMPUTING OF CRC HAS BEEN TURNED OFF***\r\n");	
#endif
#endif
	return unCRC;
}

/**************************************************************************************************************/
/**
\ingroup MessageValidation
<summary>
compute_crc_and_update_packet_list is responsible for computing the CRC for an entire message (possibly over 
multiple packets) and then updating each packet of the message with the overall CRC.
</summary>
<param name="ptMsgPacketList"> : (Input) Pointer to MsgPacketList struct (structure containing 1 or more message 
packets).</param>
<returns>uint32_t : computed CRC</returns>
<seealso cref="compute_nai_msg_crc">
*/
/**************************************************************************************************************/
uint32_t compute_crc_and_update_packet_list(MsgPacketList *ptMsgPacketList)
{
	uint32_t unNewCRC = compute_nai_msg_crc(ptMsgPacketList);
	
	/* Update all packets to reflect the new CRC (each packet stores the CRC of the entire msg) */
	MsgPacket *ptTraverse = ptMsgPacketList->ptStart;
	while (ptTraverse != NULL)
	{	
		ptTraverse->tNAIMsg.tSerdesPayLd.tTransportHdr.unCRC = unNewCRC;
		ptTraverse = ptTraverse->ptNext;
	}

	return unNewCRC;
}

/**************************************************************************************************************/
/**
\ingroup MessageValidation
<summary>
validate_nai_msgs is responsible for validating each message currently stored in the specified MsgList.
</summary>
<param name="ptMsgList"> : (Input) Pointer to MsgList struct (structure containing 1 or more messages).</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non Zero : ERROR
</returns>
<seealso cref="validate_nai_msg">
*/
/**************************************************************************************************************/
int32_t validate_nai_msgs(MsgList *ptMsgList)
{
	int32_t nStatus = NAI_SUCCESS;
	int32_t nTempStatus = NAI_SUCCESS;
	MsgPacketList *ptTraverse = ptMsgList->ptStart;

	while ( (ptTraverse != NULL) && (nStatus == NAI_SUCCESS) )
	{
		nTempStatus = validate_nai_msg(ptTraverse);
		if (nTempStatus != NAI_SUCCESS)
			nStatus = nTempStatus;
		ptTraverse = ptTraverse->ptNext;
	}

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup MessageValidation
<summary>
validate_nai_msg is responsible for validating a specific Msg.
</summary>
<param name="ptMsgPacketList"> : (Input) Pointer to MsgPacketList struct (structure containing 1 or more message 
packets).</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non Zero : ERROR
</returns>
<seealso cref="compute_nai_msg_crc">
*/
/**************************************************************************************************************/
int32_t validate_nai_msg(MsgPacketList *ptMsgPacketList)
{
	int32_t nStatus = NAI_SUCCESS;
#ifdef _VALIDATE_CRC
	uint32_t unMsgCRC = 0;

	unMsgCRC = compute_nai_msg_crc(ptMsgPacketList);
	if (unMsgCRC > 0 && unMsgCRC == ptMsgPacketList->ptStart->tNAIMsg.tSerdesPayLd.tTransportHdr.unCRC)
	{
#ifdef _VERBOSE
		printf("Transport ID: 0x%4.4x PASSED CRC! - Calculated: 0x%8.4x  Expected: 0x%8.4x\r\n", ptMsgPacketList->ptStart->tNAIMsg.tSerdesPayLd.tTransportHdr.usID, unMsgCRC, ptMsgPacketList->ptStart->tNAIMsg.tSerdesPayLd.tTransportHdr.unCRC);
#endif
	}		
	else
	{
#ifdef _VERBOSE
		printf("**Transport ID: 0x%4.4x FAILED CRC!** - Calculated: %8.4x  Expected: %8.4x\r\n", ptMsgPacketList->ptStart->tNAIMsg.tSerdesPayLd.tTransportHdr.usID, unMsgCRC, ptMsgPacketList->ptStart->tNAIMsg.tSerdesPayLd.tTransportHdr.unCRC);
#endif
		nStatus = -1;
	}
#else
#ifdef _VERBOSE
		printf("**VALIDATION OF CRC IS TURNED OFF**\r\n");
#endif
#endif
	return nStatus;
}

/**************************************************************************************************************/
/* NAI Message Processing Routines                                                                            */
/** \defgroup MessageProcessing Message Processing Functions                                                  */
/**************************************************************************************************************/

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
deal_with_nai_msg is responsible interrogating the message for the command type and reacting accordingly.
</summary>
<param name="ptMsgPacketList"> : (Input) Pointer to MsgPacketList struct (structure containing 1 or more message 
packets).</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_read_from_eeprom">
<seealso cref="nai_write_to_eeprom">
<seealso cref="nai_read_from_flash">
<seealso cref="nai_write_to_flash">
<seealso cref="nai_send_msg">
*/
/**************************************************************************************************************/
int32_t deal_with_nai_msg(MsgPacketList *ptMsgPacketList)
{
	int32_t nStatus = NAI_SUCCESS;

#ifdef _VERBOSE
	printf("Begin deal_with_nai_msg\r\n");
#endif

	/* Determine the requested command */
	if (ptMsgPacketList != NULL && ptMsgPacketList->ptStart != NULL)
	{
#ifdef _VERBOSE
	printf("	MsgPacketList and MsgPacket are NOT NULL\r\n");
#endif
		switch (ptMsgPacketList->ptStart->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usCommandType)
		{	
#if defined(__UBOOT) || defined(__BAREMETAL)
			case COMMAND_TYPECODE_READEEPROM :
				nStatus = nai_read_from_eeprom(ptMsgPacketList);
				/* Send the response! */
				if (nStatus == NAI_SUCCESS)
					nai_send_msg(ptMsgPacketList);				
				break;

			case COMMAND_TYPECODE_WRITEEEPROM :
				nStatus = nai_write_to_eeprom(ptMsgPacketList);
				break;
#endif

#ifdef __UBOOT
			case COMMAND_TYPECODE_ERASEFLASH :
				nStatus = nai_erase_flash(ptMsgPacketList);
				break;
#endif

#ifdef __UBOOT
			case COMMAND_TYPECODE_READFLASH :
				nStatus = nai_read_from_flash(ptMsgPacketList);
				/* Send the response! */
				if (nStatus == NAI_SUCCESS)
					nStatus = nai_send_msg(ptMsgPacketList);
				break;
#endif

#ifdef __UBOOT
			case COMMAND_TYPECODE_WRITEFLASH :
#ifdef _VERBOSE
	printf("	About to call nai_write_to_flash\r\n");
#endif
				nStatus = nai_write_to_flash(ptMsgPacketList);
				break;
#endif
			case COMMAND_TYPECODE_ASSIGNSLOT :				
				/* We are interested in storing the "Completer ID" - bits 4 - 7 */
				g_ucSlotID = (uint8_t)ptMsgPacketList->ptStart->tNAIMsg.tSerdesHdr.ucCompleterID; 
#ifdef _VERBOSE
				printf("***** SLOT NUM ASSIGNED: %u\r\n", (unsigned)g_ucSlotID);
#endif
				nStatus = NAI_SUCCESS;
				break;

			case COMMAND_TYPECODE_RETRIEVESLOT :
				nStatus = nai_retrieve_slot(ptMsgPacketList);
				if (nStatus == NAI_SUCCESS)
					nStatus = nai_send_msg(ptMsgPacketList);
				break;

#ifdef __UBOOT
			case COMMAND_TYPECODE_EXIT_CONFIG_MODE :
				g_bExitConfigMode = TRUE;
				break;

			case COMMAND_TYPECODE_RESET_MODULE :
				nai_reset_module();
				break;

			case COMMAND_TYPECODE_DEBUG :
				nStatus = nai_debug_test();		
				break;
#endif

#ifdef __UBOOT
#ifdef _MICROCONTROLLER
			case COMMAND_TYPECODE_CONFIG_MICRO :
				g_MicrocontrollerType = nai_get_micro_controller_type();
				if (g_MicrocontrollerType != NO_MICROCONTROLLER_TYPE)
					nStatus = nai_config_isolated_dt(ptMsgPacketList);
				else
					nStatus = NAI_NOT_SUPPORTED;
				break;
				
		    case COMMAND_TYPECODE_GET_MICRO :    	
				g_MicrocontrollerType = nai_get_micro_controller_type();
				if (g_MicrocontrollerType != NO_MICROCONTROLLER_TYPE)
					nStatus = nai_get_isolated_dt(ptMsgPacketList);
				else
					nStatus = NAI_NOT_SUPPORTED;
				break;
				
			case COMMAND_TYPECODE_ERASE_MICRO :		
				g_MicrocontrollerType = nai_get_micro_controller_type();
				if (g_MicrocontrollerType != NO_MICROCONTROLLER_TYPE)				
					nStatus = nai_erase_isolated_dt(ptMsgPacketList);
				else
					nStatus = NAI_NOT_SUPPORTED;
				break;
#endif
#endif				
			default:
				nStatus = NAI_COMMAND_NOT_RECOGNIZED;
				break;
		};	
	}
	else
	{
#ifdef _VERBOSE
		printf("ptMsgPacketList or ptMsgPacketList->ptStart is NULL!!\n");
#endif
		nStatus = NAI_INVALID_PARAMETER_VALUE;
	}

#ifdef _VERBOSE
	if (nStatus != NAI_SUCCESS)
	{
		printf("*************************************\r\n");
		printf("ERROR: %d\r\n", nStatus);
		printf("*************************************\r\n");
	}
	printf("END deal_with_nai_msg\r\n");
#endif
	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_msg_requires_finished_response is responsible for determining if the given message requires a completion
message to be sent back!
</summary>
<param name="ptMsgPacketList"> : (Input) Pointer to MsgPacketList struct (structure containing 1 or more message 
packets).</param>
<returns>BOOL : Whether or not message requires completion message 
	- TRUE  : Requires a completion message to be sent
	- FALSE : Does not require a completion message to be sent
</returns>
*/
/**************************************************************************************************************/
BOOL nai_msg_requires_finished_response(MsgPacketList *ptMsgPacketList)
{
	BOOL bRequiresResponse = FALSE;		
	MsgPacket *ptMsgPacket = NULL;

	if (ptMsgPacketList != NULL)
	{
		ptMsgPacket = ptMsgPacketList->ptStart;
		if (ptMsgPacket != NULL)
		{
			switch (ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usCommandType)
			{
				case COMMAND_TYPECODE_WRITEEEPROM :
				case COMMAND_TYPECODE_ERASEFLASH  :
				case COMMAND_TYPECODE_WRITEFLASH  :
#ifdef _MICROCONTROLLER
				case COMMAND_TYPECODE_CONFIG_MICRO :
				case COMMAND_TYPECODE_ERASE_MICRO  :
#endif
					bRequiresResponse = TRUE;
					break;

				default:
					bRequiresResponse = FALSE;
					break;
			}
		}
	}

	return bRequiresResponse;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_send_msg_finished_response is responsible for sending a msg finished response back to the original 
caller (RequesterID) to signal the module has finished processing the request. The payload will just be the 
status code!
</summary>
<param name="ptMsgPacketList"> : (Input) Pointer to MsgPacketList struct (structure containing 1 or more message 
packets).</param>
<param name="nExecutionStatus"> : (Input) Execution status reflecting the module's success or failure in executing
the request at hand.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_send_msg">
*/
/**************************************************************************************************************/
int32_t nai_send_msg_finished_response(MsgPacketList *ptMsgPacketList, int32_t nExecutionStatus)
{
	int32_t nStatus = NAI_SUCCESS;		
    uint32_t unPayLdWordCount = 2;	
	uint16_t usExpectedSequenceCount = 0;
	uint8_t  ucNewCompleterID = 0;
	uint8_t  ucNewRequesterID = 0;
	MsgPacket *ptMsgPacket = NULL;
	MsgPacketList *ptResponsePacketList = NULL;
	FIFOValue tFIFOVal;

	NAIMsg tNAIMsg;		
	memset(&tNAIMsg, 0, sizeof(NAIMsg));

#ifdef _VERBOSE
	printf("Begin nai_send_msg_finished_response\r\n");
#endif

	/* We need a valid packet in order to know who to send status back to */
	if (ptMsgPacketList != NULL && ptMsgPacketList->ptStart != NULL)
	{
#ifdef _VERBOSE
		printf("    MsgPacketList and MsgPacket are NOT NULL\r\n");
#endif

		ptMsgPacket = ptMsgPacketList->ptStart;

		/* Create Retrieve Request */
		ptResponsePacketList = create_nai_msg_packet_list();
		
		/* Determine expected sequence count...NOTE this takes into account all header info */
		usExpectedSequenceCount = 1; /*Only requesting slot number .. so 1 packet-sequence is all that is needed */

		/* SERDES HEADER */
		/* NOTE: Passed in packet receiver is now the completer and pass in packet completer is now the requester since we are responding! */
		ucNewCompleterID = (uint8_t)(ptMsgPacket->tNAIMsg.tSerdesHdr.ucRequesterID);
		ucNewRequesterID = (uint8_t)(ptMsgPacket->tNAIMsg.tSerdesHdr.ucCompleterID);

#ifdef _VERBOSE
		printf("\r\nRetrieveSlot Response Completer Slot: %u\r\n",(unsigned)ucNewCompleterID);
		printf("RetrieveSlot Response Requester Slot: %u\r\n",(unsigned)ucNewRequesterID);
#endif

		tNAIMsg.tSerdesHdr.ucType = 3;
		tNAIMsg.tSerdesHdr.ucToHPS = 1; /*NOT SURE IF THIS SHOULD BE SET FOR READ!*/
		tNAIMsg.tSerdesHdr.ucPayloadLength = 0; /* PAYLOAD LENGTH - TO BE FILLED IN BELOW! */
		tNAIMsg.tSerdesHdr.usSERDES2 = 0;
		tNAIMsg.tSerdesHdr.usSERDES3 = 0; 
		tNAIMsg.tSerdesHdr.ucRequesterID = ucNewRequesterID;
		tNAIMsg.tSerdesHdr.ucCompleterID = ucNewCompleterID;
		tNAIMsg.tSerdesHdr.usSERDES5 = 0;

		/* TRANSPORT HEADER */
		/* KEEP TRANSACTION ID SAME AS ORIGINAL REQUEST IN CASE WE NEED TO MATCH UP A RESPONSE WITH THE ORIGINAL REQUEST ON THE CALLER SIDE */
		tNAIMsg.tSerdesPayLd.tTransportHdr.usID = ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usID; 		
		tNAIMsg.tSerdesPayLd.tTransportHdr.unMsgLength = (unPayLdWordCount + (usExpectedSequenceCount * CONFIG_TOTAL_PKT_HDR_IN_WORDS)); /* Length in Words of entire */
		tNAIMsg.tSerdesPayLd.tTransportHdr.usSequenceNum = 1; 
		tNAIMsg.tSerdesPayLd.tTransportHdr.usExpectedSequenceCount = usExpectedSequenceCount; /*Expected Sequence Count */

		/* COMMAND HEADER */
		tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usCommandType = COMMAND_TYPECODE_REQUEST_FINISHED;

		/* KEEP THESE FIELDS THE SAME! */	
		tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unOffset = 0;
    	tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unPayLdRequestLength = 0;

		/* COMMAND PAYLOAD */
		/* The only payload is the 32 bit status value! */		
		tFIFOVal.unValue = (uint32_t)nExecutionStatus;
		tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandPayLd.usData[0] = tFIFOVal.usLoWord;
		tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandPayLd.usData[1] = tFIFOVal.usHiWord;

		tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength = 2;
		tNAIMsg.tSerdesHdr.ucPayloadLength = (uint8_t)((unPayLdWordCount+CONFIG_TOTAL_PKT_HDR_IN_WORDS) - TOTAL_SERDES_READWRITEREQ_HDR_IN_WORDS);

		/* Serdes Payload length must be a multiple of 2 */
		if ((tNAIMsg.tSerdesHdr.ucPayloadLength % 2) != 0)
			tNAIMsg.tSerdesHdr.ucPayloadLength++;
	
		/* Let's build a linked list of message packets that will represent a full message */
		create_and_append_nai_msg_packet(ptResponsePacketList, tNAIMsg.msg);

#ifdef _VERBOSE
		print_nai_msg(ptResponsePacketList, TRUE);
#endif
		/* Send Msg */
		nStatus = nai_send_msg(ptResponsePacketList);
		delete_nai_msg_packets(ptResponsePacketList);
		aligned_free(ptResponsePacketList);
		ptResponsePacketList = NULL;
	}
#ifdef _VERBOSE
	printf("END nai_send_msg_finished_response\r\n");
#endif

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_send_msgs is responsible for sending each Msg found in the specified MsgList over SERDES.
</summary>
<param name="ptMsgList"> : (Input) Pointer to MsgList struct (structure containing 1 or more messages).</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_send_msg">
*/
/**************************************************************************************************************/
int32_t nai_send_msgs(MsgList *ptMsgList)
{	
	int32_t nStatus = NAI_SUCCESS;
	MsgPacketList *ptMsgPackets = NULL;
	
	if (ptMsgList == NULL)
		return NAI_INVALID_PARAMETER_VALUE;

	ptMsgPackets = ptMsgList->ptStart;

	while (ptMsgPackets != NULL)
	{	
		nStatus = nai_send_msg(ptMsgPackets);
		if (nStatus != NAI_SUCCESS)
			break;

		ptMsgPackets = ptMsgPackets->ptNext;
	}

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_send_msg is responsible for sending each MsgPacket found in the specified MsgPacketList over SERDES.
</summary>
<param name="ptMsgPackets"> : (Input) Pointer to MsgPacketList struct (structure containing 1 or more message 
packets).</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="compute_nai_msg_crc">
*/
/**************************************************************************************************************/
int32_t nai_send_msg(MsgPacketList *ptMsgPackets)
{
	int32_t nStatus = NAI_SUCCESS;
	BOOL bWaitForStatusReply = FALSE;
	int32_t i = 0;
	volatile uint32_t unAddr = 0;
	volatile uint32_t unBeginTxAddr = 0; 
	int32_t nNumLoops = 0; 
	uint32_t unMsgCRC = 0;
	uint8_t ucCompleterID = 0;
	uint8_t ucRequesterID = 0;
	uint32_t ulTimer = 0;
	MsgList tMsgList;
	uint32_t unCompletionTimeout = 0;
	
#ifdef _DEBUG_X
	uint32_t unTemp = 0;
#endif

#ifdef _VERBOSE
	int32_t nLoopCount = 0;
	printf("**************nai_send_msg**************\r\n");
#endif

	bWaitForStatusReply = nai_msg_requires_finished_response(ptMsgPackets);

	unMsgCRC = compute_nai_msg_crc(ptMsgPackets);

	MsgPacket *ptTraverse = ptMsgPackets->ptStart;
	while (ptTraverse != NULL)
	{
		ucRequesterID = nai_get_serdes_requester_id(ptTraverse);
		ucCompleterID = nai_get_serdes_completer_id(ptTraverse);

#ifdef _VERBOSE
		if (nLoopCount == 0)
		{
			printf("RequesterID = 0x%1x\r\n", ucRequesterID);			
			printf("CompleterID = 0x%1x\r\n", ucCompleterID);
		}
		nLoopCount++;	
#endif		
		ulTimer = nai_get_timer(0);
		/* We have a message to send...but the FIFO is not ready...need to wait!*/	
		while (!nai_tx_fifo_empty(ucRequesterID, ucCompleterID))
		{
			/* If FIFO did not get serviced within a reasonable amount of time..get out */
			if (nai_get_timer(ulTimer) > (COMPLETION_TIMEOUT))
			{
				nStatus = NAI_TX_FIFO_NOT_EMPTY_TIMEOUT;
				break;
			}	
		}

		if (nStatus != NAI_SUCCESS)
			break;
		
		ptTraverse->tNAIMsg.tSerdesPayLd.tTransportHdr.unCRC = unMsgCRC; /* Store the calculated MSG CRC in each packet that makes up the entire MSG */			
		unAddr = nai_get_tx_fifo_address(ucRequesterID, ucCompleterID);
		unBeginTxAddr = nai_get_tx_fifo_pkt_ready_address(ucRequesterID, ucCompleterID);

		nNumLoops = ((TOTAL_SERDES_READWRITEREQ_HDR_IN_WORDS + (ptTraverse->tNAIMsg.tSerdesHdr.ucPayloadLength))); /* TOTAL_SERDES_READWRITEREQ_HDR_IN_WORDS = 6 Words + Num Words in Serdes PayLoad */
#ifdef _VERBOSE
		printf("Sending Msg Sequence #: %u\r\n", ptTraverse->tNAIMsg.tSerdesPayLd.tTransportHdr.usSequenceNum);
		printf("Num Words for FIFO    : %d\r\n", nNumLoops); 
#endif
		for (i=0; i <nNumLoops; i++)
		{
			FIFOValue tFIFOVal;
			tFIFOVal.usLoWord = ptTraverse->tNAIMsg.msg[i++];
			tFIFOVal.usHiWord = ptTraverse->tNAIMsg.msg[i];

#ifdef _DEBUG_X
			printf("FIFO VAL = 0x%8x\r\n", tFIFOVal.unValue);
#endif
			nai_write32(unAddr, tFIFOVal.unValue);
		}

		/* Now force transfer of data now that the FIFO is filled with current message*/
		nai_write32(unBeginTxAddr, (uint32_t)1);
		ptTraverse = ptTraverse->ptNext;				
	}

	/* OK - if we got here and no errors...then we need to wait to get completion status on the msg that was sent */
	if (bWaitForStatusReply && (nStatus == NAI_SUCCESS))
	{		
		/*NOTE: Each command may have different requirements regarding how long they will take to complete.
		 *      Hence, we now attempt to specify a reasonable wait time based upon our knowledge of how long
		 *      a given command typically takes. */
		unCompletionTimeout = nai_get_completion_timeout(ptMsgPackets->ptStart);
		
		/* Wait for packet! */
		ulTimer = nai_get_timer(0);
		while (!nai_rx_fifo_pkt_ready(ucRequesterID, ucCompleterID))
		{
			/* If FIFO did not get serviced within a reasonable amount of time..get out */
			/* NOTE: We make this timeout longer than majority as we don't know how long module "should" take to fulfill request */
			if (nai_get_timer(ulTimer) > unCompletionTimeout)
			{
				nStatus = NAI_RX_FIFO_PKT_NOT_READY_TIMEOUT;
				break;
			}			
		}	

		if (nStatus == NAI_SUCCESS)	
		{				
			/* Prepare for response! */
			init_nai_msgs(&tMsgList);
			nStatus = nai_receive_msg_packet(ucRequesterID, ucCompleterID, &tMsgList);

			if (nStatus == NAI_SUCCESS)
			{
#ifdef _VERBOSE
				print_nai_msgs(&tMsgList, TRUE);
#endif
				if (validate_nai_msgs(&tMsgList) == 0)
				{
					/* NOTE: Shound only have 1 message (1 packet) being returned */
					MsgPacketList *ptFirstMsg = tMsgList.ptStart;
					if (ptFirstMsg != NULL)
					{
						/* Extract the "Execution Status" out of the payload */
						MsgPacket *ptTraverse = ptFirstMsg->ptStart;
						if (ptTraverse != NULL && (ptTraverse->tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength >= 2))
						{			
							FIFOValue tFIFOValue;
							tFIFOValue.usLoWord = ptTraverse->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandPayLd.usData[0];
							tFIFOValue.usHiWord = ptTraverse->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandPayLd.usData[1];
							nStatus = (int32_t)tFIFOValue.unValue; 
						}
					}				
				}
			}

			delete_nai_msgs(&tMsgList);
		}
	}

#ifdef _VERBOSE
	printf("**************END nai_send_msg**************\r\n");
#endif

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_receive_msg_packet is responsible for receiving 1 entire packet from the SERDES bus and adds it to either
an existing message packet list (if this packet is a continuation of a message that was already started being
received) or create a new message packet.
</summary>
<param name="ucRequesterID"> : (Input) Slot ID of where receive request originated.</param>
<param name="ucCompleterID"> : (Input) Slot ID of where receive request is destined for.</param>
<param name="ptMsgList"> : (Output) Pointer to MsgList struct (structure containing 1 or more messages).</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="find_nai_msg_packet_list">
<seealso cref="create_nai_msg_packet_list">
<seealso cref="create_and_append_nai_msg_packet">
<seealso cref="add_nai_msg">
*/
/**************************************************************************************************************/
int32_t nai_receive_msg_packet(uint8_t ucRequesterID, uint8_t ucCompleterID, MsgList *ptMsgList)
{	
	int32_t nStatus = NAI_SUCCESS;
	volatile uint32_t unAddr = 0;	
	NAIMsg tReceiveMsg;
	FIFOValue tFIFOVal;
	uint32_t unWordsRead = 0;	
	uint16_t  usSerdesHdrWordsRead = 0;
	int32_t i = 0;
	MsgPacketList *ptMsgPacketList = NULL;
	uint8_t bAddMsgPacketList = 0;
	uint16_t usPacketLength = 0;

#ifdef _VERBOSE
printf("**************nai_receive_msg_packet**************\r\n");
#endif

	if (ptMsgList == NULL)
		return NAI_INVALID_PARAMETER_VALUE;

	nai_rx_fifo_clear_pkt_ready(ucRequesterID, ucCompleterID);
	memset(tReceiveMsg.msg, 0x0000, sizeof(tReceiveMsg.msg));

	unAddr = nai_get_rx_fifo_address(ucCompleterID);

	/* First we read just the SERDES Header info as we should be guaranteed this is present */
    /* From there, we can determine how many words are part of this SERDES packet */
	for (i=0; i < TOTAL_SERDES_READWRITEREQ_HDR_IN_WORDS; i++)
	{	
		tFIFOVal.unValue = nai_read32(unAddr);	

#ifdef _DEBUG_X
//#ifdef _VERBOSE
		printf("Read SERDES HDR FIFO Value: 0x%8x\r\n", tFIFOVal.unValue);
#endif
		tReceiveMsg.msg[i++] = 	tFIFOVal.usLoWord;
		tReceiveMsg.msg[i] = tFIFOVal.usHiWord;
		usSerdesHdrWordsRead += 2;
	}
		
	/* Now we read enough to know how many words are part of this SERDES packet...so let's read the desired amount off of the FIFO */
	usPacketLength = (tReceiveMsg.tSerdesHdr.ucPayloadLength); /* Payload Length for SERDES are bits 0 - 7 */
	while (unWordsRead < usPacketLength)
	{
		tFIFOVal.unValue = nai_read32(unAddr);

#ifdef _DEBUG_X
//#ifdef _VERBOSE
		printf("Read FIFO Value: 0x%8x\r\n", tFIFOVal.unValue);		
#endif

		tReceiveMsg.msg[i++] = 	tFIFOVal.usLoWord;
		unWordsRead++;
		if (unWordsRead < usPacketLength)
		{
			tReceiveMsg.msg[i++] = tFIFOVal.usHiWord;
			unWordsRead++;
		}
	}
	unWordsRead += usSerdesHdrWordsRead; /* Add back in number of SERDES Header words read */

#ifdef _VERBOSE
	/* Create a new msg packet list and fill it with FIFO data... */
	printf("Transport ID: 0x%4x\r\n", tReceiveMsg.tSerdesPayLd.tTransportHdr.usID);
#endif

	ptMsgPacketList = find_nai_msg_packet_list(ptMsgList, tReceiveMsg.tSerdesPayLd.tTransportHdr.usID);
	if (ptMsgPacketList == NULL)
	{
		ptMsgPacketList = create_nai_msg_packet_list();
		ptMsgPacketList->unWordsLeftToRead = tReceiveMsg.tSerdesPayLd.tTransportHdr.unMsgLength;
		bAddMsgPacketList = 1;
	}
    
	if (unWordsRead > ptMsgPacketList->unWordsLeftToRead)
		ptMsgPacketList->unWordsLeftToRead = 0;
	else
		ptMsgPacketList->unWordsLeftToRead -= unWordsRead;

	create_and_append_nai_msg_packet(ptMsgPacketList, &(tReceiveMsg.msg[0]));

	if (bAddMsgPacketList)
		add_nai_msg(ptMsgList, ptMsgPacketList);
#ifdef _VERBOSE
	printf("Transport ID: 0x%4x Remaining Words 0x%8.4x\r\n", tReceiveMsg.tSerdesPayLd.tTransportHdr.usID, ptMsgPacketList->unWordsLeftToRead);
	printf("**************END nai_receive_msg_packet**************\r\n");
#endif

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_get_serdes_completer_id is responsible for returning the completer ID from the MsgPacket.
</summary>
<param name="ptMsgPacket"> : (Input) Pointer to MsgPacket struct (may be partial message if message size is 
greater than packet size.</param>
<returns>uint8_t :  Completer ID</returns>
<seealso cref="compute_nai_msg_crc">
*/
/**************************************************************************************************************/
uint8_t nai_get_serdes_completer_id(MsgPacket *ptMsgPacket)
{
	/* Bits 4 - 7 of Serdes Header 4 is the completer ID */
	return  (uint8_t)ptMsgPacket->tNAIMsg.tSerdesHdr.ucCompleterID;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_get_serdes_requester_id is responsible for returning the requester ID from the MsgPacket.
</summary>
<param name="ptMsgPacket"> : (Input) Pointer to MsgPacket struct (may be partial message if message size is 
greater than packet size.</param>
<returns>uint8_t :  Requester ID</returns>
<seealso cref="compute_nai_msg_crc">
*/
/**************************************************************************************************************/
uint8_t nai_get_serdes_requester_id(MsgPacket *ptMsgPacket)
{
	/* Bits 0 - 3 of Serdes Header 4 is the Requester ID */
	return (uint8_t)ptMsgPacket->tNAIMsg.tSerdesHdr.ucRequesterID;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
calculate_nai_expected_sequence_count is responsible for determining how many packets will be required to send
the entire message.
</summary>
<param name="unPayloadWordLength"> : (Input) Stores the number of words found in the payload.</param>
<returns>uint16_t : calculated sequence count</returns>
*/
/**************************************************************************************************************/
uint16_t calculate_nai_expected_sequence_count(uint32_t unPayloadWordLength)
{
	uint16_t usExpectedSequenceCount = 0;
	uint16_t usTempCount = 0;
	uint32_t unPayloadWithHdr = 0;

#ifdef _DANT
	if (unPayloadWordLength != 0)
	{
#endif
		usTempCount = (uint16_t)(unPayloadWordLength / MAX_SERDES_MSG_IN_WORDS);
		if ((unPayloadWordLength % MAX_SERDES_MSG_IN_WORDS) > 0)
			usTempCount++;	

		unPayloadWithHdr = ((usTempCount * CONFIG_TOTAL_PKT_HDR_IN_WORDS) + unPayloadWordLength);
		usExpectedSequenceCount = (uint16_t)(unPayloadWithHdr / MAX_SERDES_MSG_IN_WORDS);
		if ((unPayloadWithHdr % MAX_SERDES_MSG_IN_WORDS) > 0)
			usExpectedSequenceCount++;
#ifdef _DANT
	}
	else /* Even with a zero payload...if we are looking to send a command in our CmdHeader...we need to have a sequence count of 1 */
		usExpectedSequenceCount = 1;
#endif		
	return usExpectedSequenceCount;
}


/**************************************************************************************************************/
/* NAI Flash Utility Routines                                                                                 */
/** \defgroup FlashUtils Flash Utility Functions                                                              */
/**************************************************************************************************************/

#ifdef __UBOOT

/**************************************************************************************************************/
/**
\ingroup FlashUtils
<summary>
nai_erase_flash is responsible for erasing specified portion of FLASH.
</summary>
<param name="ptMsgPacketList"> : (Input) Pointer to MsgPacketList struct.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="">
*/
/**************************************************************************************************************/
int32_t nai_erase_flash(MsgPacketList *ptMsgPacketList)
{
	int32_t nStatus = NAI_SUCCESS;
	uint32_t unOffset = 0;
	uint32_t unEraseOffset = 0;
	uint8_t ucNumPages = 0;	

	if (ptMsgPacketList == NULL)
		return NAI_INVALID_PARAMETER_VALUE;	
/********************************************************/
//print_nai_msg(ptMsgPacketList, TRUE /*print payload*/);
/********************************************************/
	MsgPacket *ptMsgPacket = ptMsgPacketList->ptStart;
	if (ptMsgPacket == NULL)
		return NAI_INVALID_PARAMETER_VALUE;

	unOffset = ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unOffset;
	ucNumPages = (uint8_t)ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unPayLdRequestLength; /*NOTE: we used unPayLdRequestLength to transfer the NumPages since we did not have a specific header item for NumPages and we do not want to make the header bigger than we need ... so we cludge a little here */

	/* We must erase full sectors in qspi that start on sector boundaries! */
	/* If offset is not on a page boundary..we just divide by the page size and truncate (by casting) */
	if ((unOffset % MAX_FLASH_PAGE_BYTES) != 0)
		unEraseOffset = ((uint32_t)(unOffset / MAX_FLASH_PAGE_BYTES) * MAX_FLASH_PAGE_BYTES);
	else /* Erase offset is the same as the write request offset */
		unEraseOffset = unOffset;

#ifdef _VERBOSE
	printf("Erase qspi - unEraseOffset = 0x%4.4x  Amount to Erase in Bytes = 0x%4.4x\r\n", unEraseOffset, (ucNumPages * MAX_FLASH_PAGE_BYTES));
#endif

#ifdef _QSPI_SUPPORT
	probe_qspi();
	nStatus = erase_qspi(unEraseOffset, (ucNumPages * MAX_FLASH_PAGE_BYTES));
#endif

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup FlashUtils
<summary>
nai_write_to_flash is responsible for writing a specific message to Flash (Message may consist of multiple packets).
</summary>
<param name="ptMsgPacketList"> : (Input) Pointer to MsgPacketList struct (structure containing 1 or more message 
packets).</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="compute_nai_msg_crc">
*/
/**************************************************************************************************************/
int32_t nai_write_to_flash(MsgPacketList *ptMsgPacketList)
{
	int32_t nStatus = NAI_SUCCESS;
#ifdef _QSPI_SUPPORT
	volatile uint32_t unAddress;	
#endif	
	uint32_t unOffset = 0x0;	
	uint16_t usPacketPayLdLength = 0;
	uint16_t usPacketPayLdLengthInWords = 0;
	uint32_t i = 0;
	uint32_t k = 0;
	uint32_t unMsgPayLdLength = 0;
	uint32_t unWriteBufLengthInWords = (MAX_FLASH_PAGE_BYTES >> 1);
	uint32_t unTotalWordsToWrite = 0;
#ifdef _PERFORM_QSPI_ERASE_ON_FLASH_WRITE
	uint16_t usNumSectorsToErase = 0;
	uint32_t unEraseOffset = 0x0;
#endif
	WORDValue tWORDValue;
#ifdef _QSPI_SUPPORT
	memset(pucTempBuf, 0, MAX_FLASH_PAGE_BYTES);
#endif

#ifdef _VERBOSE
	printf("BEGIN nai_write_to_flash\r\n");
#endif

	if (ptMsgPacketList == NULL)
		return NAI_INVALID_PARAMETER_VALUE;	
/********************************************************/
//print_nai_msg(ptMsgPacketList, TRUE /*print payload*/);
/********************************************************/
	MsgPacket *ptMsgPacket = ptMsgPacketList->ptStart;
	if (ptMsgPacket == NULL)
		return NAI_INVALID_PARAMETER_VALUE;

	/* Determine actual payload (minus header info) for entire message (all packets)!*/
	unTotalWordsToWrite = (ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.unMsgLength - (ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usExpectedSequenceCount * CONFIG_TOTAL_PKT_HDR_IN_WORDS));
	if (unTotalWordsToWrite < unWriteBufLengthInWords)
		unWriteBufLengthInWords = unTotalWordsToWrite;

	/* Determine how many sectors need erasing! */
#ifdef _PERFORM_QSPI_ERASE_ON_FLASH_WRITE
	usNumSectorsToErase = (uint16_t)((unWriteBufLengthInWords << 1) / MAX_FLASH_PAGE_BYTES);
	if (((unWriteBufLengthInWords << 1) % MAX_FLASH_PAGE_BYTES) != 0)
		usNumSectorsToErase++;
#endif
	unOffset = ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unOffset;		
//printf("offset = 0x%4.4x\r\n", unOffset);

	/* We must erase full sectors in qspi that start on sector boundaries! */
	/* If offset is not on a page boundary..we just divide by the page size and truncate (by casting) */
#ifdef _PERFORM_QSPI_ERASE_ON_FLASH_WRITE
	if ((unOffset % MAX_FLASH_PAGE_BYTES) != 0)
		unEraseOffset = ((uint32_t)(unOffset / MAX_FLASH_PAGE_BYTES) * MAX_FLASH_PAGE_BYTES);
	else /* Erase offset is the same as the write request offset */
		unEraseOffset = unOffset;
#endif

	/* Now we can erase what we need to use */
#ifdef _VERBOSE
#ifdef _PERFORM_QSPI_ERASE_ON_FLASH_WRITE
	printf("Erase qspi - unEraseOffset = 0x%4.4x  Amount to Erase in Bytes = 0x%4.4x\r\n", unEraseOffset, (usNumSectorsToErase * MAX_FLASH_PAGE_BYTES));
#endif	
#endif

#ifdef _QSPI_SUPPORT
	probe_qspi();
#ifdef _PERFORM_QSPI_ERASE_ON_FLASH_WRITE
	nStatus = erase_qspi(unEraseOffset, (usNumSectorsToErase * MAX_FLASH_PAGE_BYTES));
	if (nStatus != NAI_SUCCESS)
		return nStatus;
#endif
#endif

	while (ptMsgPacket != NULL)
	{	
		usPacketPayLdLength = ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength;

		/* Let's make sure we don't have a packet payload length greater than what we can currently handle */
		if (usPacketPayLdLength > (CONFIG_MAX_PAYLOAD_IN_WORDS*2))
			usPacketPayLdLength = (CONFIG_MAX_PAYLOAD_IN_WORDS*2);
	
		if ((usPacketPayLdLength % 2) != 0)
			usPacketPayLdLength++;

		/* Let's minimize the number of calls to write to qspi. - Build up a full msg buffer prior to calling the write */
		if (nStatus == NAI_SUCCESS)
		{	
			usPacketPayLdLengthInWords = convert_bytes_to_words(usPacketPayLdLength);
			for (k = 0; k < usPacketPayLdLengthInWords; k++)
			{
				tWORDValue.usValue = ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandPayLd.usData[k];
				pucTempBuf[i++] = tWORDValue.ucLoByte;
				pucTempBuf[i++] = tWORDValue.ucHiByte;
				unMsgPayLdLength += 2;


				if (i == MAX_FLASH_PAGE_BYTES)
				{
#ifdef _QSPI_SUPPORT
					unAddress = (volatile uint32_t)&pucTempBuf[0];

#ifdef _VERBOSE
					printf("Address = 0x%8.4x  Offset = 0x%4.4x  Size of msg buffer:%u\r\n", unAddress, unOffset, MAX_FLASH_PAGE_BYTES);
#endif
					nStatus = write_to_qspi(unAddress, unOffset, MAX_FLASH_PAGE_BYTES);
//printf("After write_to_qspi\r\n");
#endif
					unOffset += unMsgPayLdLength;
					unMsgPayLdLength = 0;
					i = 0;
				}
			}
		}

		if (nStatus != NAI_SUCCESS)
		{
#ifdef _VERBOSE
			printf("ERROR - write_to_qspi - status = %d\r\n", nStatus);
#endif
			return nStatus;
		}

		ptMsgPacket = ptMsgPacket->ptNext;
	}

#ifdef _QSPI_SUPPORT
	/* NOTE: It is possible that we had an even multiple of the MAX_FLASH_PAGE .. so there is nothing left to write here! */
	if (nStatus == NAI_SUCCESS && unMsgPayLdLength > 0)
	{
		/* Interact with QSPI */
		unAddress = (volatile uint32_t)&pucTempBuf[0];
#ifdef _VERBOSE
		printf("Address = 0x%8.4x  Offset = 0x%4.4x  Size of msg buffer:%u\r\n", unAddress, unOffset, unMsgPayLdLength);
#endif

		nStatus = write_to_qspi(unAddress, unOffset, unMsgPayLdLength);
	}
#endif

#ifdef _VERBOSE
	printf("END nai_write_to_flash\r\n");
#endif

	return nStatus;
}
#endif

#ifdef __UBOOT
/**************************************************************************************************************/
/**
\ingroup FlashUtils
<summary>
nai_read_from_flash is responsible for reading a specific amount of bytes from Flash.
</summary>
<param name="ptMsgPacketList"> : (Input) Pointer to MsgPacketList struct (structure containing 1 or more message 
packets).</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="calculate_nai_expected_sequence_count">
<seealso cref="probe_qspi">
<seealso cref="read_from_qspi">
<seealso cref="compute_crc_and_update_packet_list">
*/
/**************************************************************************************************************/
int32_t nai_read_from_flash(MsgPacketList *ptMsgPacketList)
{
	int32_t nStatus = NAI_SUCCESS;
	volatile uint32_t unAddress;	
	uint32_t unOffset = 0;
    uint32_t unRequestPayLdByteCount = 0;
	uint32_t unFlashPacketByteCountToRead = 0;
	uint32_t unMaxPayloadInBytes = (CONFIG_MAX_PAYLOAD_IN_WORDS*2);
	uint16_t usExpectedSequenceCount = 0;
	uint16_t usSeqNum = 0;
	uint8_t ucNewCompleterID = 0;
	uint8_t ucNewRequesterID = 0;

#ifndef _QSPI_SUPPORT
	uint32_t i = 0;
	uint32_t k = 0;
	uint32_t unLoopCnt = 0;
	WORDValue tWORDValue;
#endif

	MsgPacket *ptMsgPacket = ptMsgPacketList->ptStart;

	if (ptMsgPacket == NULL)
		return -1;

	unOffset = ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unOffset;
	unRequestPayLdByteCount = ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unPayLdRequestLength;

	/* Determine expected sequence count...NOTE this takes into account all header info */
	usExpectedSequenceCount = calculate_nai_expected_sequence_count(convert_bytes_to_words(unRequestPayLdByteCount));

	/* SERDES HEADER */
	/* NOTE: Passed in packet receiver is now the completer and pass in packet completer is now the requester since we are responding! */
	ucNewCompleterID = (uint8_t)(ptMsgPacket->tNAIMsg.tSerdesHdr.ucRequesterID);
	ucNewRequesterID = (uint8_t)(ptMsgPacket->tNAIMsg.tSerdesHdr.ucCompleterID);

	ptMsgPacket->tNAIMsg.tSerdesHdr.ucType = 3;
	ptMsgPacket->tNAIMsg.tSerdesHdr.ucToHPS = 1; /*NOT SURE IF THIS SHOULD BE SET FOR READ!*/
	ptMsgPacket->tNAIMsg.tSerdesHdr.ucPayloadLength = 0; /* PAYLOAD LENGTH - TO BE FILLED IN BELOW! */
	ptMsgPacket->tNAIMsg.tSerdesHdr.usSERDES2 = 0;
	ptMsgPacket->tNAIMsg.tSerdesHdr.usSERDES3 = 0; 
	ptMsgPacket->tNAIMsg.tSerdesHdr.ucRequesterID = ucNewRequesterID;
	ptMsgPacket->tNAIMsg.tSerdesHdr.ucCompleterID = ucNewCompleterID;
	ptMsgPacket->tNAIMsg.tSerdesHdr.usSERDES5 = 0;

	/* TRANSPORT HEADER */
	ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usID = get_next_tran_id();
	ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.unMsgLength = (convert_bytes_to_words(unRequestPayLdByteCount) + (usExpectedSequenceCount * CONFIG_TOTAL_PKT_HDR_IN_WORDS)); /* Length in Words of entire */
	ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usSequenceNum = 0;  /* Seq Number - TO BE FILLED IN BELOW! */
	ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usExpectedSequenceCount = usExpectedSequenceCount; /*Expected Sequence Count */

	/* COMMAND HEADER */
	ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usCommandType = COMMAND_TYPECODE_READFLASH;

	/* KEEP THESE FIELDS THE SAME! */	
	/* ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unOffset */
    /* ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unPayLdRequestLength */

	/* COMMAND PAYLOAD */
	/* ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength = 0; */ /*Packet payload length stored in bytes */

	/* Interact with QSPI */
#ifdef _QSPI_SUPPORT
	probe_qspi();
#endif

	while (unRequestPayLdByteCount != 0)
	{
		usSeqNum++;
		ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usSequenceNum = usSeqNum;

		unAddress = (volatile uint32_t)(&(ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandPayLd.usData[0]));

	#ifdef _VERBOSE
		printf("Read Back Address = %8.4x\r\n",unAddress);
	#endif

		unFlashPacketByteCountToRead = MIN(unRequestPayLdByteCount, unMaxPayloadInBytes);	

#ifdef _QSPI_SUPPORT
		#ifdef _VERBOSE
		printf("About to call read_from_qspi\r\n");
		printf("unAddress = %8.4x\r\n", unAddress);
		printf("unOffset = %8.4x (%u)\r\n",unOffset, unOffset);
		printf("unFlashPacketByteCountToRead = %8.4x (%u)\r\n",unFlashPacketByteCountToRead, unFlashPacketByteCountToRead);		
		
		#endif			

		nStatus = read_from_qspi(unAddress, unOffset, unFlashPacketByteCountToRead); /*NOTE: Can only stuff a max of CONFIG_MAX_PAYLOAD_IN_WORDS into a single packet */
		if (nStatus != 0)
		{
			printf("ERROR - read_from_qspi - status = %d\r\n", nStatus);
			#ifdef _VERBOSE
			printf("Byte Count To Read = %8.4x (%u)\r\n",unFlashPacketByteCountToRead, unFlashPacketByteCountToRead);
			printf("Offset = %8.4x (%u)\r\n",unOffset, unOffset);
			print_nai_msg(ptMsgPacketList, TRUE /*print payload*/);
			#endif	
			break;
		}
#else		
		unLoopCnt = convert_bytes_to_words(unFlashPacketByteCountToRead);
		for (i=0; i < unLoopCnt; i++)
		{
			tWORDValue.ucLoByte = pucTempBuf[k++];
			tWORDValue.ucHiByte = pucTempBuf[k++];
			ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandPayLd.usData[i] = tWORDValue.usValue;
		}
#endif
		unOffset += unFlashPacketByteCountToRead; /* Take into account # of bytes just read from flash..next packet of same msg needs to start reading where last packet left off! */

		/* Store how many bytes are in this packet's payload */
		ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength = (uint16_t)(unFlashPacketByteCountToRead);
		ptMsgPacket->tNAIMsg.tSerdesHdr.ucPayloadLength = (uint8_t)(MIN((convert_bytes_to_words(unFlashPacketByteCountToRead)+CONFIG_TOTAL_PKT_HDR_IN_WORDS), MAX_SERDES_MSG_IN_WORDS) - TOTAL_SERDES_READWRITEREQ_HDR_IN_WORDS);
		
		/* Serdes Payload length must be a multiple of 2 */
		if ((ptMsgPacket->tNAIMsg.tSerdesHdr.ucPayloadLength % 2) != 0)
			ptMsgPacket->tNAIMsg.tSerdesHdr.ucPayloadLength++;


		if (unRequestPayLdByteCount <= unMaxPayloadInBytes)
			unRequestPayLdByteCount = 0;
		else			
		{
		    unRequestPayLdByteCount -= unMaxPayloadInBytes;
		
			/*Add a new packet! */				
			ptMsgPacket->ptNext = clone_nai_msg_packet_sans_payload(ptMsgPacket);
			ptMsgPacket = ptMsgPacket->ptNext;
		}
	}

	if (nStatus == NAI_SUCCESS)
	{
		/* Compute new Msg CRC and update each packet with new value */
		compute_crc_and_update_packet_list(ptMsgPacketList);

		#ifdef _VERBOSE
		print_nai_msg(ptMsgPacketList, TRUE /*print payload*/);
		#endif
	}

	return nStatus;
}
#endif




/**************************************************************************************************************/
/* NAI EEPROM Utility Routines                                                                                */
/** \defgroup EEPROMUtils EEPROM Utility Functions                                                            */
/**************************************************************************************************************/
#if defined(__UBOOT) || defined(__BAREMETAL)

/**************************************************************************************************************/
/**
\ingroup EEPROMUtils
<summary>
nai_copy_eeprom_to_common is responsible for copying the entire contents of EEPROM to the common (shared memory)
area of the module. 
</summary>
<param name="usChipID"> : (Input) ID of EEPROM chip to copy</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
int32_t nai_copy_eeprom_to_common(uint16_t usChipID)
{
	int32_t nStatus = NAI_SUCCESS;
	
	/* Cut the number of 32 Bit values in 1/2 since Xilinx driver can only read up to 255 (not 256) at a time */
	uint16_t usNumEEPROM32BitValues = ((MAX_EEPROM_BYTES/4)/2);
	uint32_t unEEPROMData[usNumEEPROM32BitValues]; 
	uint16_t i = 0;	
	uint32_t unAddr = INTERFACE_MODULE_COMMON_EEPROM_START;
	uint32_t unOffset = 0;
	uint8_t  ucReadPass = 0; /* Indicates number of times i2c_read has been called */
	
	/* First - check to see if we can detect the desired device */
	if (i2c_probe(usChipID) != 0)
		return NAI_I2C_DEVICE_NOT_FOUND;

	/* If we are dealing with the Functional (top) Module...adjust the register address of where we will be writing */
	if (usChipID == I2C_FUNCTIONAL_MODULE_EEPROM)
		unAddr = FUNCTIONAL_MODULE_COMMON_EEPROM_START;
			
	do
	{			
#ifdef _DEBUG_X
		printf("Reading EEPROM - Current Pass = %u\r\n", i, (ucReadPass+1));
#endif		
		/* Second - read 1/2 EEPROM into buffer */
		nStatus = i2c_read(usChipID, unOffset /*Offset*/, 1, (uint8_t*)(&(unEEPROMData[0])), (MAX_EEPROM_BYTES/2));
		if (nStatus == NAI_SUCCESS)
		{
			for (i=0; i < usNumEEPROM32BitValues; i++)
			{
#ifdef _DEBUG_X
				printf("unEEPROMData[%u] = 0x%8.4x\r\n", i, unEEPROMData[i]);
#endif
				nai_common_write32(unAddr, unEEPROMData[i]);
				unAddr += 4; /* increment the address to write to by the size of the data being written */
			}
		}
		unOffset += (MAX_EEPROM_BYTES/2);
		ucReadPass++;
	}while (ucReadPass < 2);
	
	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup EEPROMUtils
<summary>
nai_write_to_eeprom is responsible for writing a specific amount of bytes to EEPROM. Bytes to be written are
considered to be the payload of the 1st MsgPacket of the specified MsgPacketList.
</summary>
<param name="ptMsgPacketList"> : (Input) Pointer to MsgPacketList struct (EEPROM is smaller than our packet 
payload so requests should always be just a single packet).</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
int32_t nai_write_to_eeprom(MsgPacketList *ptMsgPacketList)
{
	int32_t nStatus = NAI_SUCCESS;
	MsgPacket *ptMsgPacket = ptMsgPacketList->ptStart;

	if (ptMsgPacket == NULL)
		return -1;

	nStatus = write_at24c(ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usChipID, (uint)ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unOffset, (uint8_t *)&(ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandPayLd.usData[0]), (int32_t)(ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength));

	return nStatus;	
}

/**************************************************************************************************************/
/**
\ingroup EEPROMUtils
<summary>
nai_read_from_eeprom is responsible for reading a specific amount of bytes from EEPROM.
</summary>
<param name="ptMsgPacketList"> : (Input) Pointer to MsgPacketList struct (EEPROM is smaller than our packet 
payload so fulfilling a request should always be just a single packet).</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="compute_crc_and_update_packet_list">
*/
/**************************************************************************************************************/
int32_t nai_read_from_eeprom(MsgPacketList *ptMsgPacketList)
{
	int32_t nStatus = NAI_SUCCESS;
	uint32_t unOffset = 0;
    uint32_t unRequestPayLdByteCount = 0;
	uint32_t unFlashPacketByteCountToRead = 0;
	uint16_t usExpectedSequenceCount = 0;
	uint16_t usSeqNum = 0;
	uint8_t  ucNewCompleterID = 0;
	uint8_t  ucNewRequesterID = 0;
	uint8_t  ucExtraReadRequired = 0;
	
	MsgPacket *ptMsgPacket = ptMsgPacketList->ptStart;

	if (ptMsgPacket == NULL)
		return -1;

	unOffset = ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unOffset;
	unRequestPayLdByteCount = ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unPayLdRequestLength;

	/* Determine expected sequence count...NOTE this takes into account all header info */
	usExpectedSequenceCount = calculate_nai_expected_sequence_count(convert_bytes_to_words(unRequestPayLdByteCount));
	
	/* We expect since we are reading from EEPROM that max Bytes to read is 256 which fits well within our 240 word (or so) packet payload */
	if (usExpectedSequenceCount > 1)
		return -2;

	/* Now we re-use the request packet as our return packet ... NOTE: we could create a brand-new packet if that makes more sense....time will tell*/
	/* SERDES HEADER */
	ucNewCompleterID = (ptMsgPacket->tNAIMsg.tSerdesHdr.ucRequesterID);
	ucNewRequesterID = (ptMsgPacket->tNAIMsg.tSerdesHdr.ucCompleterID);

	ptMsgPacket->tNAIMsg.tSerdesHdr.ucType = 3;
	ptMsgPacket->tNAIMsg.tSerdesHdr.ucToHPS = 1; /*NOT SURE IF THIS SHOULD BE SET FOR READ!*/
	ptMsgPacket->tNAIMsg.tSerdesHdr.ucPayloadLength = 0; /* PAYLOAD LENGTH - TO BE FILLED IN BELOW! */
	ptMsgPacket->tNAIMsg.tSerdesHdr.usSERDES2 = 0;
	ptMsgPacket->tNAIMsg.tSerdesHdr.usSERDES3 = 0; 
	ptMsgPacket->tNAIMsg.tSerdesHdr.ucRequesterID = ucNewRequesterID;
	ptMsgPacket->tNAIMsg.tSerdesHdr.ucCompleterID = ucNewCompleterID;
	ptMsgPacket->tNAIMsg.tSerdesHdr.usSERDES5 = 0;

	/* TRANSPORT HEADER */
	ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usID = get_next_tran_id();
	ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.unMsgLength = (convert_bytes_to_words(unRequestPayLdByteCount) + (usExpectedSequenceCount * CONFIG_TOTAL_PKT_HDR_IN_WORDS)); /* Length in Words of entire */
	ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usSequenceNum = 0;  /* Seq Number - TO BE FILLED IN BELOW! */
	ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usExpectedSequenceCount = usExpectedSequenceCount; /*Expected Sequence Count */

	/* COMMAND HEADER */
	ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usCommandType = COMMAND_TYPECODE_READEEPROM;

	/* KEEP THESE FIELDS THE SAME! */	
	/* ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unOffset */
    /* ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unPayLdRequestLength */

	/* COMMAND PAYLOAD */
	/* ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength = 0; */ /*Packet payload length stored in bytes */

	if (unRequestPayLdByteCount > 0)
	{
		usSeqNum++;
		ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usSequenceNum = usSeqNum;
		
		/* Xilinx driver does not allow a read of 256 bytes (max is 255) so if caller is requesting full 256, we need a special case */
		if (unRequestPayLdByteCount >= MAX_EEPROM_BYTES)
		{
			unFlashPacketByteCountToRead = (MAX_EEPROM_BYTES - 1);
			ucExtraReadRequired = 1;
		}
		else
			unFlashPacketByteCountToRead = unRequestPayLdByteCount;
				
#ifdef _VERBOSE		
		printf("chip ID = %u\r\n", ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usChipID);
		printf("Offset = %u\r\n", unOffset);		
		printf("unFlashPacketByteCountToRead %8.4x\r\n",unFlashPacketByteCountToRead);
#endif

		if (i2c_probe(ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usChipID) != 0)
			nStatus = NAI_I2C_DEVICE_NOT_FOUND;

		if (nStatus == NAI_SUCCESS)
		{
			nStatus = i2c_read(ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usChipID, (uint)unOffset, 1, (uint8_t*)(&(ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandPayLd.usData[0])), (int32_t)unFlashPacketByteCountToRead);
			if (nStatus != 0)
				printf("ERROR - read_from_eeprom - status = %d\r\n", (unsigned int)nStatus);

			if (ucExtraReadRequired == 1)
			{
				nStatus = i2c_read(ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usChipID, (uint)(MAX_EEPROM_BYTES - 1), 1, (uint8_t*)(&(ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandPayLd.usData[((MAX_EEPROM_BYTES / 2) - 1)])), (int32_t)1);
				if (nStatus != 0)
					printf("ERROR - read_from_eeprom - status = %d\r\n", (unsigned int)nStatus);
				unFlashPacketByteCountToRead++; /* Since we read an extra byte...increase the byte count to read by one for proper bookkeeping */
			}

			/* Store how many bytes are in this packet's payload */
			ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength = (uint16_t)(unFlashPacketByteCountToRead);
			ptMsgPacket->tNAIMsg.tSerdesHdr.ucPayloadLength = (uint8_t)(MIN((convert_bytes_to_words(unFlashPacketByteCountToRead)+CONFIG_TOTAL_PKT_HDR_IN_WORDS), MAX_SERDES_MSG_IN_WORDS) - TOTAL_SERDES_READWRITEREQ_HDR_IN_WORDS);

			/* Serdes Payload length must be a multiple of 2 */
			if ((ptMsgPacket->tNAIMsg.tSerdesHdr.ucPayloadLength % 2) != 0)
				ptMsgPacket->tNAIMsg.tSerdesHdr.ucPayloadLength++;

			/* Compute new Msg CRC and update each packet with new value */
			compute_crc_and_update_packet_list(ptMsgPacketList);

#ifdef _VERBOSE
			print_nai_msg(ptMsgPacketList, TRUE /*print payload*/);
#endif
		}
	}
#ifdef _VERBOSE
	else
		printf("nai_read_from_eeprom - Requested Payload Count of 0 detected\r\n");		
#endif

	return nStatus;
}

#endif

/**************************************************************************************************************/
/* NAI High Level API Routines                                                                                */
/** \defgroup HighLevelAPI High Level API Functions                                                            */
/**************************************************************************************************************/

/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
make_read_request is responsible for doing all of the dirty work associated with reading from Flash or Eeprom.
Based upon the information provided via parameters, a valid request packet will be constructed, and sent and 
then return data will be taken off of the SERDES Fifo and the provided buffer will be filled with the payload(s)
of the packet(s) that are returned.
</summary>
<param name="usCommandType"> : (Input) Defines the type of Read command. (COMMAND_TYPECODE_READEEPROM or 
COMMAND_TYPECODE_READFLASH)</param>
<param name="ucRequesterID"> : (Input) Defines the Slot Number of the requesting module or mother board.</param>
<param name="ucCompleterID"> : (Input) Defines the Slot Number of the target module or mother board the 
request is being to.</param>
<param name="usChipID"> : (Input) ID of chip to read from (may have mutliple eeproms or flash devices</param>
<param name="unOffset"> : (Input) Offset (in Bytes) into the hardware the read request is being made to. (eeprom or flash)</param>
<param name="pucBuf"> : (Output) Buffer to hold payload data returned from the read request.</param>
<param name="nLen"> : (Input) Length (in Bytes) of how much data to read.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="create_nai_msg_packet_list">
<seealso cref="create_and_append_nai_msg_packet">
<seealso cref="nai_send_msg">
<seealso cref="delete_nai_msg_packets">
<seealso cref="init_nai_msgs">
*/
/**************************************************************************************************************/
static int32_t make_read_request(uint16_t usCommandType, uint8_t ucRequesterID, uint8_t ucCompleterID, uint16_t usChipID, uint unOffset, uint8_t *pucBuf, int32_t nLen)
{
	int32_t nStatus = NAI_SUCCESS;
	int32_t nMsgPacketCount = 0;	
	MsgList tMsgList;
	MsgPacketList *ptMsgPacketList = NULL;
	uint16_t usID = get_next_tran_id();
	uint16_t usExpectedSequenceCount = 0;	
	uint16_t usPacketPayLdLength = 0;
	uint32_t ulTimer;

	init_nai_msgs(&tMsgList);

	/* Create Retrieve Request */
	ptMsgPacketList = create_nai_msg_packet_list();
	nMsgPacketCount = 0;		
		
	/* Determine expected sequence count...NOTE this takes into account all header info */
	usExpectedSequenceCount = 1;

	nMsgPacketCount++;
	NAIMsg tNAIMsg;		
	memset(&tNAIMsg, 0, sizeof(NAIMsg));

	/* SERDES HEADER */
#ifdef _VERBOSE
	printf("IN make_read_request\r\n");
	printf("RequesterID = 0x%2.2x(%u)\r\n", ucRequesterID, ucRequesterID);
	printf("CompleterID = 0x%2.2x(%u)\r\n", ucCompleterID, ucCompleterID);
#endif
	tNAIMsg.tSerdesHdr.ucType = 3;
	tNAIMsg.tSerdesHdr.ucToHPS = 1;
	tNAIMsg.tSerdesHdr.ucPayloadLength = (uint8_t)(CONFIG_TOTAL_PKT_HDR_IN_WORDS - TOTAL_SERDES_READWRITEREQ_HDR_IN_WORDS);
	tNAIMsg.tSerdesHdr.usSERDES2 = 0;
	tNAIMsg.tSerdesHdr.usSERDES3 = 0;
	tNAIMsg.tSerdesHdr.ucRequesterID = ucRequesterID;
	tNAIMsg.tSerdesHdr.ucCompleterID = ucCompleterID;
	tNAIMsg.tSerdesHdr.usSERDES5 = 0;

	/* Serdes Payload length must be a multiple of 2 */
	if ((tNAIMsg.tSerdesHdr.ucPayloadLength % 2) != 0)
		tNAIMsg.tSerdesHdr.ucPayloadLength++;

	/* TRANSPORT HEADER */
	tNAIMsg.tSerdesPayLd.tTransportHdr.usID = usID;
	tNAIMsg.tSerdesPayLd.tTransportHdr.unMsgLength = CONFIG_TOTAL_PKT_HDR_IN_WORDS;    /* Length in Words - Single Packet Request with no payload!*/
	tNAIMsg.tSerdesPayLd.tTransportHdr.usSequenceNum = nMsgPacketCount;
	tNAIMsg.tSerdesPayLd.tTransportHdr.usExpectedSequenceCount = usExpectedSequenceCount;

	/* COMMAND HEADER */
	tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usChipID = usChipID;
	tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usCommandType = usCommandType;
	tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unOffset = (uint32_t)unOffset;
    tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unPayLdRequestLength = (uint32_t)nLen;

	/* COMMAND PAYLOAD */
	tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength = 0; /* Packet payload length stored in bytes - No Payload since just a request */

	/* Let's build a linked list of message packets that will represent a full message */
	create_and_append_nai_msg_packet(ptMsgPacketList, tNAIMsg.msg);

	/* Send request for data */
	nStatus = nai_send_msg(ptMsgPacketList); /*Don't need to wait for read request..as we will be waiting below!*/
	delete_nai_msg_packets(ptMsgPacketList);
	aligned_free(ptMsgPacketList);
	ptMsgPacketList = NULL;

	if (nStatus == NAI_SUCCESS)
	{
		/* Prepare for response! */
		init_nai_msgs(&tMsgList);

		/* Wait for packet! */		
		do 
		{
			ulTimer = nai_get_timer(0);
			while (!nai_rx_fifo_pkt_ready(ucRequesterID, ucCompleterID))
			{
				/* If FIFO did not get serviced within a reasonable amount of time..get out */
				if (nai_get_timer(ulTimer) > COMPLETION_TIMEOUT)
				{
					nStatus = NAI_RX_FIFO_PKT_NOT_READY_TIMEOUT;
					break;
				}			
			}

			/* Now fetch the next message packet and append it to the MsgList */
			if (nStatus == NAI_SUCCESS)
				nStatus = nai_receive_msg_packet(ucRequesterID, ucCompleterID, &tMsgList);

		} while	(nStatus == NAI_SUCCESS && tMsgList.ptEnd->unWordsLeftToRead > 0);

		if (nStatus == NAI_SUCCESS)
		{
#ifdef _VERBOSE
			print_nai_msgs(&tMsgList, TRUE);
#endif
			if (validate_nai_msgs(&tMsgList) == 0)
			{
				/* NOTE: Shound only have 1 message (1 + more packets) being returned */
				MsgPacketList *ptFirstMsg = tMsgList.ptStart;
				if (ptFirstMsg != NULL)
				{
					MsgPacket *ptTraverse = ptFirstMsg->ptStart;
					uint16_t usBufOffset = 0;
					while (ptTraverse != NULL)
					{			
						usPacketPayLdLength = MIN(ptTraverse->tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength, nLen);
						
						memcpy((pucBuf + usBufOffset), &(ptTraverse->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandPayLd.usData[0]), usPacketPayLdLength); /* Copy specified number of bytes from response message into output param */		
						usBufOffset += usPacketPayLdLength; /* Increment offset so we don't overwrite what we already have written */
						ptTraverse = ptTraverse->ptNext;
					}
				}				
			}
		}

		delete_nai_msgs(&tMsgList);
	}

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
make_write_request is responsible for doing all of the dirty work associated with writing data to Flash or Eeprom.
Based upon the information provided via parameters, a valid write request packet will be constructed and sent.
</summary>
<param name="usCommandType"> : (Input) Defines the type of Write command. (COMMAND_TYPECODE_WRITEEEPROM or 
COMMAND_TYPECODE_WRITEFLASH)</param>
<param name="ucRequesterID"> : (Input) Defines the Slot Number of the requesting module or mother board.</param>
<param name="ucCompleterID"> : (Input) Defines the Slot Number of the target module or mother board the 
request is being to.</param>
<param name="usChipID"> : (Input) ID of chip to write to (may have mutliple eeproms or flash devices</param>
<param name="unOffset"> : (Input) Offset (in Bytes) into the hardware the write request is being made to. (eeprom or flash)</param>
<param name="pucBuf"> : (Input) Buffer to hold payload of data to write.</param>
<param name="nLen"> : (Input) Length (in Bytes) of how much data to write.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="create_and_append_nai_msg_packet">
<seealso cref="nai_send_msg">
<seealso cref="delete_nai_msg_packets">
*/
/**************************************************************************************************************/
static int32_t make_write_request(uint16_t usCommandType, uint8_t ucRequesterID, uint8_t ucCompleterID, uint16_t usChipID, uint unOffset, uint8_t *pucBuf, int32_t nLen)
{
	int32_t nStatus = NAI_SUCCESS;
	uint32_t unMsgPayloadWordLength = convert_bytes_to_words(nLen); /* # of words to write */
	int32_t i = 0, k = 0;	
	int32_t nMsgPacketCount = 0;
	int32_t nPayloadWordsLeftToRead = 0;
	int32_t nPayloadWordCount = 0;
	MsgPacketList *ptMsgPacketList = NULL;
	uint16_t usID = get_next_tran_id();
	uint16_t usExpectedSequenceCount = 0;
	WORDValue tWordValue;

	ptMsgPacketList = create_nai_msg_packet_list();

	nMsgPacketCount = 0;		
	nPayloadWordsLeftToRead = (int32_t)unMsgPayloadWordLength;		

	/* Determine expected sequence count...NOTE this takes into account all header info */
	usExpectedSequenceCount = calculate_nai_expected_sequence_count(unMsgPayloadWordLength);

	do
	{		
		nMsgPacketCount++;
		NAIMsg tNAIMsg;		
		memset(&tNAIMsg, 0, sizeof(NAIMsg));

		/* SERDES HEADER */
#ifdef _VERBOSE
	printf("IN make_write_request\r\n");
	printf("RequesterID = 0x%2.2x(%u)\r\n", ucRequesterID, ucRequesterID);
	printf("CompleterID = 0x%2.2x(%u)\r\n", ucCompleterID, ucCompleterID);
#endif
		tNAIMsg.tSerdesHdr.ucType = 3;
		tNAIMsg.tSerdesHdr.ucToHPS = 1;
		tNAIMsg.tSerdesHdr.ucPayloadLength = (uint8_t)(MIN((nPayloadWordsLeftToRead+CONFIG_TOTAL_PKT_HDR_IN_WORDS), MAX_SERDES_MSG_IN_WORDS) - TOTAL_SERDES_READWRITEREQ_HDR_IN_WORDS);
		tNAIMsg.tSerdesHdr.usSERDES2 = 0;
		tNAIMsg.tSerdesHdr.usSERDES3 = 0;
		tNAIMsg.tSerdesHdr.ucRequesterID = ucRequesterID;
		tNAIMsg.tSerdesHdr.ucCompleterID = ucCompleterID;
		tNAIMsg.tSerdesHdr.usSERDES5 = 0;

		/* Serdes Payload length must be a multiple of 2 */
		if ((tNAIMsg.tSerdesHdr.ucPayloadLength % 2) != 0)	
			tNAIMsg.tSerdesHdr.ucPayloadLength++;

		/* TRANSPORT HEADER */
		tNAIMsg.tSerdesPayLd.tTransportHdr.usID = usID;
		tNAIMsg.tSerdesPayLd.tTransportHdr.unMsgLength = (unMsgPayloadWordLength + (usExpectedSequenceCount * CONFIG_TOTAL_PKT_HDR_IN_WORDS));    /* Length in Words */
		tNAIMsg.tSerdesPayLd.tTransportHdr.usSequenceNum = nMsgPacketCount;
		tNAIMsg.tSerdesPayLd.tTransportHdr.usExpectedSequenceCount = usExpectedSequenceCount;

		/* COMMAND HEADER */
		tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usChipID = usChipID;
		tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usCommandType = usCommandType;
		tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unOffset = (uint32_t)unOffset;
	    tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unPayLdRequestLength = 0x0;

		/* COMMAND PAYLOAD */
#ifdef _VERBOSE
		printf("Num Msg Words To Read: %ld\r\n", nPayloadWordsLeftToRead);
#endif
		nPayloadWordCount = MIN(nPayloadWordsLeftToRead, CONFIG_MAX_PAYLOAD_IN_WORDS);
		
		for (i=0; i < nPayloadWordCount; i++)
		{
			tWordValue.ucLoByte = pucBuf[k++];
			/* It is possible a buffer of "odd" size was passed in...let's make sure we don't try to access more than what we were given */
			if (k < nLen)
				tWordValue.ucHiByte = pucBuf[k++];
			else
				tWordValue.ucHiByte = 0x00;
		   	tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandPayLd.usData[i] = tWordValue.usValue;
		}

		tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength = (nPayloadWordCount * 2); /*Packet payload length stored in bytes */

		/* Let's build a linked list of message packets that will represent a full message */
		create_and_append_nai_msg_packet(ptMsgPacketList, tNAIMsg.msg);
		nPayloadWordsLeftToRead -= nPayloadWordCount;
#ifdef _VERBOSE
		printf("Num Msg Words Left To Read: %ld\r\n", nPayloadWordsLeftToRead);
#endif

	} while (nPayloadWordsLeftToRead > 0);

	/* Send request to write data */
//printf("make_write_request ... about to send message\r\n");
	nStatus = nai_send_msg(ptMsgPacketList); 
//printf("make_write_request ... after send message\r\n");
	delete_nai_msg_packets(ptMsgPacketList);
	aligned_free(ptMsgPacketList);

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
make_generic_zero_payload_cmd_request is responsible for doing all of the dirty work associated with making a simple command
(with no payload) to a given module. This function sets up a properly formatted MsgPacketList that encapsulates the command 
request and sends it to the appropriate module.
</summary>
<param name="usCommandType"> : (Input) Defines the type of command. </param>
<param name="ucRequesterID"> : (Input) Defines the Slot Number of the requesting module or mother board.</param>
<param name="ucCompleterID"> : (Input) Defines the Slot Number of the target module or mother board the 
request is being to.</param>

<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="create_and_append_nai_msg_packet">
<seealso cref="nai_send_msg">
<seealso cref="delete_nai_msg_packets">
*/
/**************************************************************************************************************/
static int32_t make_generic_zero_payload_cmd_request(uint16_t usCommandType, uint8_t ucRequesterID, uint8_t ucCompleterID)
{
	int32_t nStatus = NAI_SUCCESS;
	MsgPacketList *ptMsgPacketList = NULL;
	uint16_t usID = get_next_tran_id();

	ptMsgPacketList = create_nai_msg_packet_list();

	NAIMsg tNAIMsg;		
	memset(&tNAIMsg, 0, sizeof(NAIMsg));

	/* SERDES HEADER */
	tNAIMsg.tSerdesHdr.ucType = 3;
	tNAIMsg.tSerdesHdr.ucToHPS = 1;
	tNAIMsg.tSerdesHdr.ucPayloadLength = (uint8_t)(CONFIG_TOTAL_PKT_HDR_IN_WORDS - TOTAL_SERDES_READWRITEREQ_HDR_IN_WORDS);
	tNAIMsg.tSerdesHdr.usSERDES2 = 0;
	tNAIMsg.tSerdesHdr.usSERDES3 = 0;
	tNAIMsg.tSerdesHdr.ucRequesterID = ucRequesterID;
	tNAIMsg.tSerdesHdr.ucCompleterID = ucCompleterID;
	tNAIMsg.tSerdesHdr.usSERDES5 = 0;

	/* TRANSPORT HEADER */
	tNAIMsg.tSerdesPayLd.tTransportHdr.usID = usID;
	tNAIMsg.tSerdesPayLd.tTransportHdr.unMsgLength = CONFIG_TOTAL_PKT_HDR_IN_WORDS;    /* Length in Words */
	tNAIMsg.tSerdesPayLd.tTransportHdr.usSequenceNum = 1;
	tNAIMsg.tSerdesPayLd.tTransportHdr.usExpectedSequenceCount = 1;

	/* COMMAND HEADER */
	tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usChipID = 0;
	tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usCommandType = usCommandType;
	tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unOffset = 0;
    tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unPayLdRequestLength = 0;

	/* COMMAND PAYLOAD */
	tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength = 0; /*Packet payload length stored in bytes */

	/* Let's build a linked list of message packets that will represent a full message */
	create_and_append_nai_msg_packet(ptMsgPacketList, tNAIMsg.msg);

	/* Send request to write data */
	nStatus = nai_send_msg(ptMsgPacketList); 
	delete_nai_msg_packets(ptMsgPacketList);
	aligned_free(ptMsgPacketList);

	return nStatus;
}



#if defined(__UBOOT) || defined(__LINUX)
/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
nai_read_module_eeprom_request is responsibe for making a read request to EEPROM for the desired number of bytes of data.
</summary>
<param name="usChipID"> : (Input) Specifies the ID of the device to read data from.</param>
<param name="ucRequesterID"> : (Input) Defines the Slot Number of the requesting module or mother board.</param>
<param name="ucCompleterID"> : (Input) Defines the Slot Number of the target module or mother board the 
request is being to.</param>
<param name="unEepromOffset"> : (Input) Offset (in Bytes) into the EEPROM hardware the read request is being made to.</param>
<param name="pucBuf"> : (Output) Buffer to hold payload data returned from the read request.</param>
<param name="nLen"> : (Input) Length (in Bytes) of how much data to read.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="make_read_request">
*/
/**************************************************************************************************************/
int32_t nai_read_module_eeprom_request(uint16_t usChipID, uint8_t ucRequesterID, uint8_t ucCompleterID, uint32_t unEepromOffset, uint8_t *pucBuf, int32_t nLen)
{
	int32_t nStatus = NAI_SUCCESS;
	nStatus = make_read_request(COMMAND_TYPECODE_READEEPROM, ucRequesterID, ucCompleterID, usChipID, unEepromOffset, pucBuf, nLen);
	return nStatus;
}
#endif

#if defined(__UBOOT) || defined(__LINUX)
/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
nai_write_module_eeprom_request is responsible for making a request to have the desired number of bytes written to EEPROM.
</summary>
<param name="usChipID"> : (Input) Specifies the ID of the device to write data to.</param>
<param name="ucRequesterID"> : (Input) Defines the Slot Number of the requesting module or mother board.</param>
<param name="ucCompleterID"> : (Input) Defines the Slot Number of the target module or mother board the 
request is being to.</param>
<param name="unEepromOffset"> : (Input) Offset (in Bytes) into the EEPROM hardware the write request is being made to.</param>
<param name="pucBuf"> : (Input) Buffer to hold payload of data to write.</param>
<param name="nLen"> : (Input) Length (in Bytes) of how much data to write.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="make_write_request">
*/
/**************************************************************************************************************/
int32_t nai_write_module_eeprom_request(uint16_t usChipID, uint8_t ucRequesterID, uint8_t ucCompleterID, uint32_t unEepromOffset, uint8_t *pucBuf, int32_t nLen)
{
	int32_t nStatus = NAI_SUCCESS;
	nStatus = make_write_request(COMMAND_TYPECODE_WRITEEEPROM, ucRequesterID, ucCompleterID, usChipID, unEepromOffset, pucBuf, nLen);
	return nStatus;
}
#endif

#if defined(__UBOOT) || defined(__LINUX)

/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
nai_erase_flash_request is responsible for making a request to erase the desired number of FLASH pages (each page
is 65536 bytes).
</summary>
<param name="ucRequesterID"> : (Input) Defines the Slot Number of the requesting module or mother board.</param>
<param name="ucCompleterID"> : (Input) Defines the Slot Number of the target module or mother board the 
request is being to.</param>
<param name="unFlashOffset"> : (Input) Offset (in Bytes) into the FLASH hardware the read request is being made to.</param>
<param name="ucNumPages"> : (Input) Number of (65536 byte) pages to erase</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="create_nai_msg_packet_list">
<seealso cref="create_and_append_nai_msg_packet">
<seealso cref="nai_send_msg">
<seealso cref="delete_nai_msg_packets">
<seealso cref="aligned_free">
*/
/**************************************************************************************************************/
int32_t nai_erase_flash_request(uint8_t ucRequesterID, uint8_t ucCompleterID, uint32_t unFlashOffset, uint8_t ucNumPages)
{
	int32_t nStatus = NAI_SUCCESS;
	MsgPacketList *ptMsgPacketList = NULL;
	uint16_t usID = get_next_tran_id();

	ptMsgPacketList = create_nai_msg_packet_list();

	NAIMsg tNAIMsg;		
	memset(&tNAIMsg, 0, sizeof(NAIMsg));

	/* SERDES HEADER */
	tNAIMsg.tSerdesHdr.ucType = 3;
	tNAIMsg.tSerdesHdr.ucToHPS = 1;
	tNAIMsg.tSerdesHdr.ucPayloadLength = (uint8_t)(CONFIG_TOTAL_PKT_HDR_IN_WORDS - TOTAL_SERDES_READWRITEREQ_HDR_IN_WORDS);
	tNAIMsg.tSerdesHdr.usSERDES2 = 0;
	tNAIMsg.tSerdesHdr.usSERDES3 = 0;
	tNAIMsg.tSerdesHdr.ucRequesterID = ucRequesterID;
	tNAIMsg.tSerdesHdr.ucCompleterID = ucCompleterID;
	tNAIMsg.tSerdesHdr.usSERDES5 = 0;

	/* TRANSPORT HEADER */
	tNAIMsg.tSerdesPayLd.tTransportHdr.usID = usID;
	tNAIMsg.tSerdesPayLd.tTransportHdr.unMsgLength = CONFIG_TOTAL_PKT_HDR_IN_WORDS;    /* Length in Words */
	tNAIMsg.tSerdesPayLd.tTransportHdr.usSequenceNum = 1;
	tNAIMsg.tSerdesPayLd.tTransportHdr.usExpectedSequenceCount = 1;

	/* COMMAND HEADER */
	tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usChipID = 0;
	tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usCommandType = COMMAND_TYPECODE_ERASEFLASH;
	tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unOffset = unFlashOffset;
    tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unPayLdRequestLength = (uint32_t)ucNumPages;  /* Stuff num pages into PayLdRequestLength just to get the data across */

	/* COMMAND PAYLOAD */
	tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength = 0; /*Packet payload length stored in bytes */

	/* Let's build a linked list of message packets that will represent a full message */
	create_and_append_nai_msg_packet(ptMsgPacketList, tNAIMsg.msg);

	/* Send request to write data */
	nStatus = nai_send_msg(ptMsgPacketList); 
	delete_nai_msg_packets(ptMsgPacketList);
	aligned_free(ptMsgPacketList);

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
nai_read_module_flash_request is responsible for making a read request to FLASH for the desired number of bytes of data.
</summary>
<param name="ucRequesterID"> : (Input) Defines the Slot Number of the requesting module or mother board.</param>
<param name="ucCompleterID"> : (Input) Defines the Slot Number of the target module or mother board the 
request is being to.</param>
<param name="unFlashOffset"> : (Input) Offset (in Bytes) into the FLASH hardware the read request is being made to.</param>
<param name="pucBuf"> : (Output) Buffer to hold payload data returned from the read request.</param>
<param name="nLen"> : (Input) Length (in Bytes) of how much data to read.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="make_read_request">
*/
/**************************************************************************************************************/
int32_t nai_read_module_flash_request(uint8_t ucRequesterID, uint8_t ucCompleterID, uint32_t unFlashOffset, uint8_t *pucBuf, int32_t nLen)
{
	int32_t nStatus = NAI_SUCCESS;
	nStatus = make_read_request(COMMAND_TYPECODE_READFLASH, ucRequesterID, ucCompleterID, 0, unFlashOffset, pucBuf, nLen);
	return nStatus;
}
#endif

#if defined(__UBOOT) || defined(__LINUX)
/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
nai_write_module_flash_request is responsible for making a request to have the desired number of bytes written to FLASH.
</summary>
<param name="ucRequesterID"> : (Input) Defines the Slot Number of the requesting module or mother board.</param>
<param name="ucCompleterID"> : (Input) Defines the Slot Number of the target module or mother board the 
request is being to.</param>
<param name="unFlashOffset"> : (Input) Offset (in Bytes) into the FLASH hardware the write request is being made to.</param>
<param name="pucBuf"> : (Input) Buffer to hold payload of data to write.</param>
<param name="nLen"> : (Input) Length (in Bytes) of how much data to write.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="make_write_request">
*/
/**************************************************************************************************************/
int32_t nai_write_module_flash_request(uint8_t ucRequesterID, uint8_t ucCompleterID, uint32_t unFlashOffset, uint8_t *pucBuf, int32_t nLen)
{
	int32_t nStatus = NAI_SUCCESS;
	nStatus = make_write_request(COMMAND_TYPECODE_WRITEFLASH, ucRequesterID, ucCompleterID, 0, unFlashOffset, pucBuf, nLen);
	return nStatus;
}
#endif

#if defined(__UBOOT) || defined(__LINUX)
/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
nai_write_micro_request is responsible for making a request to have the desired number of bytes written to the
microcontroller FLASH.
</summary>
<param name="ucRequesterID"> : (Input) Defines the Slot Number of the requesting module or mother board.</param>
<param name="ucCompleterID"> : (Input) Defines the Slot Number of the target module or mother board the 
request is being to.</param>
<param name="ucChannel"> : (Input) Channel where microcontroller resides to write to.</param>
<param name="unOffset"> : (Input) Offset (in Bytes) into the FLASH hardware the write request is being made to.</param>
<param name="pucBuf"> : (Input) Buffer to hold payload of data to write.</param>
<param name="nLen"> : (Input) Length (in Bytes) of how much data to write.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="make_write_request">
*/
/**************************************************************************************************************/
int32_t nai_write_micro_request(uint8_t ucRequesterID, uint8_t ucCompleterID, uint8_t ucChannel, uint32_t unOffset, uint8_t *pucBuf, int32_t nLen)
{
	int32_t nStatus = NAI_SUCCESS;
	nStatus = make_write_request(COMMAND_TYPECODE_CONFIG_MICRO, ucRequesterID, ucCompleterID, (uint16_t)ucChannel, unOffset, pucBuf, nLen);
	return nStatus;
}
#endif

#if defined(__UBOOT) || defined(__LINUX)
/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
nai_get_micro_request is responsible for making a request to receive info from a microcontroller found at the
specified channel. Information includes microcontroller version number along with supported functions.
</summary>
<param name="ucRequesterID"> : (Input) Defines the Slot Number of the requesting module or mother board.</param>
<param name="ucCompleterID"> : (Input) Defines the Slot Number of the target module or mother board the 
request is being to.</param>
<param name="ucChannel"> : (Input) Channel where microcontroller resides to read from.</param>
<param name="pucBuf"> : (Input) Buffer to hold payload of data being read.</param>
<param name="nLen"> : (Input) Length (in Bytes) of how much data the buffer can hold.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="make_read_request">
*/
/**************************************************************************************************************/
int32_t nai_get_micro_request(uint8_t ucRequesterID, uint8_t ucCompleterID, uint8_t ucChannel, uint8_t *pucBuf, int32_t nLen)
{
	int32_t nStatus = NAI_SUCCESS;
	/*NOTE: We pass ucChannel in as chip ID - this will allow us to determine on the module side which channel user is requesting version info from */
	nStatus = make_read_request(COMMAND_TYPECODE_GET_MICRO, ucRequesterID, ucCompleterID, (uint16_t)ucChannel, 0, pucBuf, nLen);
	return nStatus;
}
#endif

#if defined(__UBOOT) || defined(__LINUX)
/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
nai_erase_micro_request is responsible for making a request to erase EEPROM of microcontroller found at the
specified channel. 
</summary>
<param name="ucRequesterID"> : (Input) Defines the Slot Number of the requesting module or mother board.</param>
<param name="ucCompleterID"> : (Input) Defines the Slot Number of the target module or mother board the 
request is being to.</param>
<param name="ucChannel"> : (Input) Channel where microcontroller resides to read from.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="make_write_request">
*/
/**************************************************************************************************************/
int32_t nai_erase_micro_request(uint8_t ucRequesterID, uint8_t ucCompleterID, uint8_t ucChannel)
{
	int32_t nStatus = NAI_SUCCESS;
	nStatus = make_write_request(COMMAND_TYPECODE_ERASE_MICRO, ucRequesterID, ucCompleterID, (uint16_t)ucChannel, 0, NULL, 0);
	return nStatus;
}
#endif

#if defined(__UBOOT) || defined(__LINUX)
/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
nai_exit_module_config_mode_request is responsible for forcing the module to exit configuration mode.
</summary>
<param name="ucRequesterID"> : (Input) Defines the Slot Number of the requesting module or mother board.</param>
<param name="ucCompleterID"> : (Input) Defines the Slot Number of the target module or mother board the 
request is being to.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="make_generic_zero_payload_cmd_request">
*/
/**************************************************************************************************************/
int32_t nai_exit_module_config_mode_request(uint8_t ucSlotID)
{
	int32_t nStatus = NAI_SUCCESS;
	int32_t nTempStatus = NAI_SUCCESS;
	uint16_t usDetectedStatus = 0;
	int32_t i = 0;

	if (ucSlotID != INVALID_SLOT_ID)	
		nStatus = make_generic_zero_payload_cmd_request(COMMAND_TYPECODE_EXIT_CONFIG_MODE, MB_SLOT, ucSlotID);
	else /* Passing in of INVALID_SLOT_ID indicates a request to exit config mode for all modules! */
	{
		nai_retreive_module_slots_status_request(&usDetectedStatus);

		for (i=1; i <= g_usMaxSlotCount; i++)
		{
			/* Only make a request on the FIFO if the module is present */
			if (((usDetectedStatus >> (i-1)) & 0x01) == 0x01)
			{
				nTempStatus = make_generic_zero_payload_cmd_request(COMMAND_TYPECODE_EXIT_CONFIG_MODE, MB_SLOT, i);
				if (nStatus == NAI_SUCCESS)
					nStatus = nTempStatus;
			}
		}
	}
	return nStatus;
}
#endif

#if defined(__UBOOT) || defined(__LINUX)
/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
nai_reset_module_request is responsible for forcing the module to perform a CPU reset.
</summary>
<param name="ucRequesterID"> : (Input) Defines the Slot Number of the requesting module or mother board.</param>
<param name="ucCompleterID"> : (Input) Defines the Slot Number of the target module or mother board the 
request is being to.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="make_generic_zero_payload_cmd_request">
*/
/**************************************************************************************************************/
int32_t nai_reset_module_request(uint8_t ucSlotID)
{
	int32_t nStatus = NAI_SUCCESS;
	int32_t nTempStatus = NAI_SUCCESS;
	uint16_t usDetectedStatus = 0;
	int32_t i = 0;

	if (ucSlotID != INVALID_SLOT_ID)	
		nStatus = make_generic_zero_payload_cmd_request(COMMAND_TYPECODE_RESET_MODULE, MB_SLOT, ucSlotID);
	else /* Passing in of INVALID_SLOT_ID indicates a request to reset all modules! */
	{
		nai_retreive_module_slots_status_request(&usDetectedStatus);
		
		for (i=1; i <= g_usMaxSlotCount; i++)
		{
			/* Only make a request on the FIFO if the module is present */
			if (((usDetectedStatus >> (i-1)) & 0x01) == 0x01)
			{
				nTempStatus = make_generic_zero_payload_cmd_request(COMMAND_TYPECODE_RESET_MODULE, MB_SLOT, i);
				if (nStatus == NAI_SUCCESS)
					nStatus = nTempStatus;
			}
		}		
	}
	return nStatus;
}
#endif

/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
nai_init_all_module_slots is responsible for determining which modules are phsically present and
telling each module what slot they are.
</summary>
<param name="usMaxSlotCount"> : (Input) Max number of module slots that should attempt to be initialized.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
int32_t nai_init_all_module_slots(uint16_t usMaxSlotCount)
{
	const int32_t MAX_ATTEMPTS = 200;
	int32_t i = 1;
	int32_t nStatus = NAI_SUCCESS;
	int32_t nTempStatus = NAI_SUCCESS;
	int32_t nAttemptCountdown = MAX_ATTEMPTS;
	uint16_t usDetectedStatus = 0;
	volatile uint32_t unModuleLinkInitDone   = 0x00004104;	
	FIFOValue tFIFOVal;

	/* Store the Max Slot Count */
	g_usMaxSlotCount = usMaxSlotCount;

	nai_retreive_module_slots_status_request(&usDetectedStatus);

	/* Now let's determine which modules are present! */		
	for (i=1; i <= g_usMaxSlotCount; i++)
	{
		if (((usDetectedStatus >> (i-1)) & 0x01) == 0x01)
		{
#ifdef _VERBOSE
			printf("Detected Module: %ld\r\n", i);
#endif	
			nAttemptCountdown = MAX_ATTEMPTS;
			/* We know the module is present..but let's make sure we wait until the LINK is up for this slot before we attempt to talk to it */
			while (nAttemptCountdown > 0)
			{	
				tFIFOVal.unValue = nai_read32(unModuleLinkInitDone);								
			
				if (((tFIFOVal.unValue >> (i-1)) & 0x0001) == 0x0001)
					break;

				nAttemptCountdown--;
			}				

			if (nAttemptCountdown > 0)
			{						
				nTempStatus = nai_assign_module_slot_number_request(i);
				if (nTempStatus != NAI_SUCCESS)
					nStatus = nTempStatus;
			}
			else
				nStatus = NAI_MODULE_NOT_READY;
		}	
#ifdef _VERBOSE
		else
			printf("Not Found Module: %d\r\n", i);
#endif					
	}

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
nai_assign_module_slot_number_request is responsible for making a assigning the module with a slot number.
</summary>
<param name="ucSlotID"> : (Input) Defines the Slot Number of this module.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="make_generic_zero_payload_cmd_request">
<seealso cref="make_read_request">
*/
/**************************************************************************************************************/
int32_t nai_assign_module_slot_number_request(uint8_t ucSlotID)
{
	int32_t nStatus = NAI_SUCCESS;
	uint8_t ucBuf[16];
	
	nStatus = make_generic_zero_payload_cmd_request(COMMAND_TYPECODE_ASSIGNSLOT, MB_SLOT, ucSlotID);
	if (nStatus == NAI_SUCCESS)
	{
		nStatus = make_read_request(COMMAND_TYPECODE_RETRIEVESLOT, MB_SLOT, ucSlotID, 0, 0, &ucBuf[0], 1);
		if (nStatus == NAI_SUCCESS)
		{
#ifdef _VERBOSE
			printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\r\n");
			printf("Expected Slot #: %u\r\n", (unsigned)ucSlotID);
			printf("Retrieved Slot #: %u\r\n", (unsigned)ucBuf[0]);
			printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\r\n");
#endif
			/* If Slot number did not verify...return error status */
			if (ucBuf[0] != ucSlotID)
				nStatus = NAI_ERROR_WRONG_SLOT_NUM;
		}
	}

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_retrieve_slot is responsible for retrieving the slot number assignment.
</summary>
<param name="ptMsgPacketList"> : (Input) Pointer to MsgPacketList struct (structure containing 1 or more message 
packets).</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="compute_crc_and_update_packet_list">
*/
/**************************************************************************************************************/
int32_t nai_retrieve_slot(MsgPacketList *ptMsgPacketList)
{
	int32_t nStatus = NAI_SUCCESS;		
    uint32_t unRequestPayLdByteCount = 0;	
	uint16_t usExpectedSequenceCount = 0;
	uint8_t  ucNewCompleterID = 0;
	uint8_t  ucNewRequesterID = 0;
	MsgPacket *ptMsgPacket = ptMsgPacketList->ptStart;

	if (ptMsgPacket == NULL)
		return -1;

	unRequestPayLdByteCount = ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unPayLdRequestLength;

	/* Determine expected sequence count...NOTE this takes into account all header info */
	usExpectedSequenceCount = 1; /*Only requesting slot number .. so 1 packet-sequence is all that is needed */

	/* SERDES HEADER */
	/* NOTE: Passed in packet receiver is now the completer and pass in packet completer is now the requester since we are responding! */
	ucNewCompleterID = (uint8_t)(ptMsgPacket->tNAIMsg.tSerdesHdr.ucRequesterID);
	ucNewRequesterID = (uint8_t)(ptMsgPacket->tNAIMsg.tSerdesHdr.ucCompleterID);

	#ifdef _VERBOSE
		printf("\r\nRetrieveSlot Response Completer Slot: %u\r\n",(unsigned)ucNewCompleterID);
		printf("RetrieveSlot Response Requester Slot: %u\r\n",(unsigned)ucNewRequesterID);
	#endif

	ptMsgPacket->tNAIMsg.tSerdesHdr.ucType = 3;
	ptMsgPacket->tNAIMsg.tSerdesHdr.ucToHPS = 1; /*NOT SURE IF THIS SHOULD BE SET FOR READ!*/
	ptMsgPacket->tNAIMsg.tSerdesHdr.ucPayloadLength = 0; /* PAYLOAD LENGTH - TO BE FILLED IN BELOW! */
	ptMsgPacket->tNAIMsg.tSerdesHdr.usSERDES2 = 0;
	ptMsgPacket->tNAIMsg.tSerdesHdr.usSERDES3 = 0; 
	ptMsgPacket->tNAIMsg.tSerdesHdr.ucRequesterID = ucNewRequesterID;
	ptMsgPacket->tNAIMsg.tSerdesHdr.ucCompleterID = ucNewCompleterID;
	ptMsgPacket->tNAIMsg.tSerdesHdr.usSERDES5 = 0;

	/* TRANSPORT HEADER */
	ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usID = get_next_tran_id(); 
	ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.unMsgLength = (convert_bytes_to_words(unRequestPayLdByteCount) + (usExpectedSequenceCount * CONFIG_TOTAL_PKT_HDR_IN_WORDS)); /* Length in Words of entire */
	ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usSequenceNum = 1; 
	ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usExpectedSequenceCount = usExpectedSequenceCount; /*Expected Sequence Count */

	/* COMMAND HEADER */
	ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usCommandType = 0;

	/* KEEP THESE FIELDS THE SAME! */	
	/* ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unOffset */
    /* ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unPayLdRequestLength */

	/* COMMAND PAYLOAD */
	/* ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength = 0; */ /*Packet payload length stored in bytes */

	/* The only payload is the slot number! */
	sprintf((char *)ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandPayLd.usData, "%c", g_ucSlotID);
	ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength = 1;
	ptMsgPacket->tNAIMsg.tSerdesHdr.ucPayloadLength = (uint8_t)((convert_bytes_to_words(unRequestPayLdByteCount)+CONFIG_TOTAL_PKT_HDR_IN_WORDS) - TOTAL_SERDES_READWRITEREQ_HDR_IN_WORDS);

	/* Serdes Payload length must be a multiple of 2 */
	if ((ptMsgPacket->tNAIMsg.tSerdesHdr.ucPayloadLength % 2) != 0)
		ptMsgPacket->tNAIMsg.tSerdesHdr.ucPayloadLength++;

	
	/* Compute new Msg CRC and update each packet with new value */
	compute_crc_and_update_packet_list(ptMsgPacketList);

#ifdef _VERBOSE
	print_nai_msg(ptMsgPacketList, TRUE /*print payload*/);
#endif

	return nStatus;	
}

/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
nai_retrieve_module_slot_number_request is responsible for retrieving the slot number a module.
</summary>
<param name="pucSlotID"> : (Input/Output) Input:Slot being queried. Output: Slot reported by the module.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="make_read_request">
*/
/**************************************************************************************************************/
int32_t nai_retrieve_module_slot_number_request(uint8_t *pucSlotID)
{
	int32_t nStatus = NAI_SUCCESS;
	uint8_t ucBuf[16];

	nStatus = make_read_request(COMMAND_TYPECODE_RETRIEVESLOT, MB_SLOT, *pucSlotID, 0, 0, ucBuf, sizeof(ucBuf));
	return nStatus;
}


#ifdef __UBOOT
int32_t nai_debug_test()
{
	int32_t nStatus = NAI_SUCCESS;
	uint8_t pTemp[65536]; 
	uint8_t pTempRead[65536];
	int i = 0;
	int k = 0;
	int nLen = 0x10000; /*65536*/
	uint32_t nLoopCnt = 256;
//	uint32_t nLoopCnt = 5120;
	uint32_t unOffset = 0;
	uint32_t unFailures = 0;
	BOOL bMisMatch = FALSE;
	volatile uint32_t unAddr = 0;

	probe_qspi();

	for (i=0; i < nLoopCnt; i++)
	{
		memset(pTemp, 0, nLen);
		for (k=0; k < nLen; k++)
			pTemp[k] = k;

		unOffset = ((i%256) * nLen);

		nStatus = erase_qspi(unOffset, 0x10000);
		if (nStatus != NAI_SUCCESS)
		{
			printf("*****************************************************\r\n");
			printf("erase_qspi ERROR: %d\r\n", nStatus);
			printf("*****************************************************\r\n");

			return nStatus;
		}


		unAddr = (volatile uint32_t)&pTemp[0];
		printf("write_to_qspi  [%d  of  %d]\r\n", i+1, nLoopCnt);
		nStatus = write_to_qspi(unAddr, unOffset, nLen);
		
		if (nStatus != NAI_SUCCESS)
		{
			printf("*****************************************************\r\n");
			printf("write_to_qspi ERROR: %d\r\n", nStatus);
			printf("*****************************************************\r\n");
			break;
		}

		unAddr = (volatile uint32_t)&pTempRead[0];
		nStatus = read_from_qspi(unAddr, unOffset, nLen);

		if (nStatus != NAI_SUCCESS)
		{
			printf("*****************************************************\r\n");
			printf("read_from_qspi ERROR: %d\r\n", nStatus);
			printf("*****************************************************\r\n");
			break;
		}

		bMisMatch = FALSE;
		for (k=0; k < nLen; k++)
		{		
//			printf("pTemp[%d] = %4.4x  pTempRead[%d] = %4.4x\r\n", k, pTemp[k], k, pTempRead[k]);	
			if (pTemp[k] != pTempRead[k])
			{
				unFailures++;
				printf("pTemp[%d] = %4.4x  pTempRead[%d] = %4.4x\r\n", k, pTemp[k], k, pTempRead[k]);
				bMisMatch = TRUE;
				break;
			}
		}

		if (bMisMatch)
			printf("Data Validation for [%d of %d] = FAILED\r\n", i+1, nLoopCnt);
		else
			printf("Data Validation for [%d of %d] = PASSED\r\n", i+1, nLoopCnt);

		printf("Total Number of Failures: %u\r\n", unFailures);
		printf("-----------------------------------------------------\r\n");
	}	
		
	return nStatus;
}
#endif

#ifdef __UBOOT
#ifdef _MICROCONTROLLER

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_get_micro_controller_type is responsible for returning which microcontroller is relevant to this module.
</summary>
<returns>int32_t : uint8_t : Microcontroller Type (0 = NA (Not Applicable), 1 = KL17, 2 = ST)
</returns>
*/
/**************************************************************************************************************/
static uint8_t nai_get_micro_controller_type(void)
{
	uint32_t nStatus = NAI_SUCCESS;
	uint8_t ucMicrocontrollerType = KL17_MICROCONTROLLER_TYPE;
	uint32_t unEEPROMData[50]; 
	uint32_t unOffset = 0;
	uint16_t usPartNumEEPROMValues = 6;
	uint16_t usVersionNumEEPROMValues = 1;
	uint8_t  ucContinue = 1;	
#ifdef _DEBUG_X		
	int32_t  i = 0;
#endif
	
	/* Read the Functional (top board) EEPROM and determine which Microcontroller Type is relevant! */
		
	/* First - check to see if we can detect the desired device */
	if (i2c_probe(I2C_FUNCTIONAL_MODULE_EEPROM) != 0)
		return NO_MICROCONTROLLER_TYPE;

	/* We want to fetch 2 pieces of information from EEPROM:  Part Number and Revision */
	unOffset = 0xD0;

	/* Read Part Number from EEPROM */
	nStatus = i2c_read(I2C_FUNCTIONAL_MODULE_EEPROM, unOffset /*Offset*/, 1, (uint8_t*)(&(unEEPROMData[0])), (usPartNumEEPROMValues*4)); /*24 Bytes*/
	if (nStatus == NAI_SUCCESS)
	{				
#ifdef _DEBUG_X		
		for (i=0; i < usPartNumEEPROMValues; i++)		
			printf("Part Number - unEEPROMData[%u] = 0x%8.4x\r\n", i, unEEPROMData[i]);
#endif	
		if (unEEPROMData[0] == 0xFFFFFFFF || unEEPROMData[0] == 0x00000000)
		{
			ucMicrocontrollerType = ST_MICROCONTROLLER_TYPE;
			ucContinue = 0;
		}
		else /* Let's make sure this is a module part that we know should have a Microcontroller (DT) */ 
		{
			/* Check for 105-*/
			if (unEEPROMData[0] != 0x2D353031) /*All DTs start with 105- so if we don't have that...we are out of luck! */
			{
				ucMicrocontrollerType = NO_MICROCONTROLLER_TYPE;
				ucContinue = 0;
			}
			/* Now check for 1050 = DT2 and DT5  or for 1002 = DT1 and DT4 */
			else if ((unEEPROMData[1] != 0x30353031) &&
					 (unEEPROMData[1] != 0x32303031))
			{
				ucMicrocontrollerType = NO_MICROCONTROLLER_TYPE;
				ucContinue = 0;			    			    
			}
		}
	}

	if (ucContinue)
	{
		/* Now retrieve the Version */
		unOffset = 0xE8;
		memset(&unEEPROMData[0], 0, usPartNumEEPROMValues);
		nStatus = i2c_read(I2C_FUNCTIONAL_MODULE_EEPROM, unOffset /*Offset*/, 1, (uint8_t*)(&(unEEPROMData[0])), (usVersionNumEEPROMValues*4)); /*4 Bytes*/
		if (nStatus == NAI_SUCCESS)
		{
#ifdef _DEBUG_X		
			for (i=0; i < usVersionNumEEPROMValues; i++)
				printf("Version Number - unEEPROMData[%u] = 0x%8.4x\r\n", i, unEEPROMData[i]);
#endif		
			/* If by chance we detect a rev B (which I don't think we ever will)...this version had ST microcontrollers */
			if (unEEPROMData[0] == 0x00000042)
				ucMicrocontrollerType = ST_MICROCONTROLLER_TYPE;
			else
				ucMicrocontrollerType = KL17_MICROCONTROLLER_TYPE;
		}	
	}
			
	return ucMicrocontrollerType;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_get_max_packet_write_in_bytes is responsible for returning how many bytes the specified microcontroller is
capable of writing on each "pass".
</summary>
<param name="ucMicrocontrollerType"> : Input: Specifies which Microcontroller we are targetting</param>
<returns>int32_t : uint16_t : Number of byes the specified microcontroller can write in one pass.
</returns>
*/
/**************************************************************************************************************/
static uint16_t nai_get_max_packet_write_in_bytes(uint8_t ucMicrocontrollerType)
{
	uint16_t usMaxPacketWriteInBytes =0;
	
	switch (ucMicrocontrollerType)
	{
		case KL17_MICROCONTROLLER_TYPE :
			usMaxPacketWriteInBytes = MAX_ISOLATED_DT_KL17_WRITE_IN_BYTES;
			break;
		case ST_MICROCONTROLLER_TYPE :
			usMaxPacketWriteInBytes = MAX_ISOLATED_DT_ST_WRITE_IN_BYTES;
			break;
		default:
			break;
	}	
	
	return usMaxPacketWriteInBytes;
}
/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_put_isolated_dt_in_config_mode is responsible for performing the necessary steps to get the Microcontroller
found at the specified channel to go into configuration mode.
</summary>
<param name="ucChanIndex"> : Input:Zero based index of channel </param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_put_isolated_dt_in_config_mode(uint8_t ucChanIndex)
{	
	int32_t nStatus = NAI_SUCCESS;
	int32_t ulTimer = 0;
	int32_t psEnable = 0;
	
	/* Hold Off Reseting of UART?? DANT - CHECK WITH CARMINE IF RESET HAPPENS ON HIGH OR LOW */
	nai_common_write32(MODULE_COMMON_RESET_UART_REG, 0x00000001);
	
	psEnable = nai_dt2_read32(CHANNEL_PS_ENABLE);
	psEnable |= (0x01 << ucChanIndex);
	
	/* Turn on the Power Supply of the Microcontroller found at the specified channel */
	nai_dt2_write32(CHANNEL_PS_ENABLE, psEnable);
	
	/* Give controller enough time to power up */
	ulTimer = nai_get_timer(0);
	while (1)
	{			
		if (nai_get_timer(ulTimer) > 100) /* 100 milliseconds */
			break;
	}			
#ifdef _DF3_VERBOSE				
	printf("Channel Index = %d\n", ucChanIndex);
#endif	
	/* Channel Selector */
	nai_dt2_write32(CHANNEL_SELECT_REG, (0x01 << ucChanIndex));
	
	/* Set Serial Control */
	nai_dt2_write32(SERIAL_CONTROL_REG, DT2_CTRL_UART); 

	switch (g_MicrocontrollerType)
	{
	case ST_MICROCONTROLLER_TYPE :
		nStatus = nai_st_put_in_config_mode(ucChanIndex);
		break;

	case KL17_MICROCONTROLLER_TYPE :
		nStatus = nai_kl17_put_in_config_mode(ucChanIndex);
		break;

	default :
		nStatus = NAI_COMMAND_NOT_RECOGNIZED;
		break;
	}

#ifdef _DF3_VERBOSE
	printf("After Set Serial to UART and Reset TX RAM\n");
#endif

	if (nStatus == NAI_SUCCESS)
		g_bAlreadyInBootloader[ucChanIndex] = TRUE;

	return nStatus;
}

static int32_t nai_st_put_in_config_mode(uint8_t ucChanIndex)
{
	int32_t nStatus = NAI_SUCCESS;
	int32_t ulTimer = 0;
	uint8_t ucCmdBuf[1];

#ifdef _DF3_VERBOSE
	printf("In nai_st_put_in_config_mode!\n");
#endif

	/* Let's assume the Microcontroller at the given channel is a "virgin" Microcontroller. (i.e. never been programmed).
	 * If we are correct, for ST Microcontrollers we will get an ACK the 1st time we send a 7F. If we are not correct,
	 * we will fail and we will need to 1st send a 7F in the operational baud rate to force the ST into configuration
	 * bootloader mode and then we need to change back to the configuration baud rate and send another 7F to Synch to the new baud rate */
	nai_dt2_write32(SERIAL_BAUDRATE_REG, DT2_CONFIGURATION_115_2K_BAUD_RATE);
	
	/* Now Reset UART */
	nai_common_write32(MODULE_COMMON_RESET_UART_REG, 0x00000000);
		
	/* Make sure both the TX and RX RAM are cleared */	
	nai_dt2_write32(SERIAL_CONTROL_REG, (DT2_CTRL_UART | DT2_CTRL_RESET_TX_RAM | DT2_CTRL_RESET_RX_RAM)); 
		
	/* Write 0x7F to TxRAM block and then signal a "GO" */
	ucCmdBuf[0] = STM_SYNCH;  /* Command ID */		
	nStatus = nai_send_data_isolated_dt(ucChanIndex, &ucCmdBuf[0], (sizeof(ucCmdBuf) / sizeof(uint8_t)));
	if (nStatus == NAI_SUCCESS)
		nStatus = nai_st_wait_for_ack();

	if (nStatus != NAI_SUCCESS)
	{
		nai_dt2_write32(SERIAL_BAUDRATE_REG, DT2_OPERATIONAL_1M_BAUD_RATE);
	
		/* We changed the baud rate...so let's make sure we clear both the TX and RX Ram locations in case there was junk received */
		nai_dt2_write32(SERIAL_CONTROL_REG, (DT2_CTRL_UART | DT2_CTRL_RESET_TX_RAM | DT2_CTRL_RESET_RX_RAM)); 
		
		ucCmdBuf[0] = DT2_FORCE_BOOTLOADER; /* Command to force microcontroller found at specified channel into bootloader mode (if in  operational mode) */
		nai_send_data_isolated_dt(ucChanIndex, &ucCmdBuf[0], (sizeof(ucCmdBuf) / sizeof(uint8_t)));
		
		/* Give ST Microcontroller just a little bit of time to get into bootloader mode */
		ulTimer = nai_get_timer(0);
		while (1)
		{			
			if (nai_get_timer(ulTimer) > 100) /* 100 milliseconds */
				break;
		}

		/* Force baud rate back to configuration baud of 115.2K */
		nai_dt2_write32(SERIAL_BAUDRATE_REG, DT2_CONFIGURATION_115_2K_BAUD_RATE);

		/* We changed the baud rate...so let's make sure we clear both the TX and RX Ram locations in case there was junk received */
		nai_dt2_write32(SERIAL_CONTROL_REG, (DT2_CTRL_UART |  DT2_CTRL_RESET_TX_RAM | DT2_CTRL_RESET_RX_RAM)); 
		
		/* Write 0x7F to TxRAM block and then signal a "GO" */
		ucCmdBuf[0] = STM_SYNCH;  /* Command ID */		
		nStatus = nai_send_data_isolated_dt(ucChanIndex, &ucCmdBuf[0], (sizeof(ucCmdBuf) / sizeof(uint8_t)));
		if (nStatus == NAI_SUCCESS)
			nStatus = nai_st_wait_for_ack();
	}
	
	return nStatus;
}

static int32_t nai_kl17_put_in_config_mode(uint8_t ucChanIndex)
{
	int32_t nStatus = NAI_SUCCESS;
	int32_t ulTimer = 0;
	uint8_t ucCmdBuf[1];

#ifdef _DF3_VERBOSE
	printf("In nai_kl17_put_in_config_mode!\n");
#endif

	/* Let's assume the Microcontroller at the given channel is a "virgin" Microcontroller. (i.e. never been programmed).
	 * If we are correct, for KL17 Microcontrollers we will get an ACK the 1st time we send a 7F. If we are not correct,
	 * we will fail and we will need to 1st send a 7F in the operational baud rate to force the KL17 into configuration
	 * bootloader mode and then we need to change back to the configuration baud rate and send another 7F to Synch to the new baud rate */
	nai_dt2_write32(SERIAL_BAUDRATE_REG, DT2_CONFIGURATION_115_2K_BAUD_RATE);
//	nai_dt2_write32(SERIAL_BAUDRATE_REG, DT2_ST_DEFAULT_57_6K_BAUD_RATE);
//	nai_dt2_write32(SERIAL_BAUDRATE_REG, DT2_ST_DEFAULT_9600_BAUD_RATE);
	
	/* Now Reset UART */
	nai_common_write32(MODULE_COMMON_RESET_UART_REG, 0x00000000);
	
//DANT -ASK CARMINE ABOUT THIS!!!
	nai_dt2_cal_write32(0x00000000, 4000); /* Voltage CAL gain factor */
//END DANT	
		
	/* Make sure both the TX and RX RAM are cleared */	
	nai_dt2_write32(SERIAL_CONTROL_REG, (DT2_CTRL_UART | DT2_CTRL_RESET_TX_RAM | DT2_CTRL_RESET_RX_RAM)); 
						
	if (nStatus == NAI_SUCCESS)
		nStatus = nai_kl17_send_ping(ucChanIndex);

	/* If we did not succeed in sending and receiving PING data, let's bump baud rate to operational speeds and try again! */
	if (nStatus != NAI_SUCCESS)
	{
		nai_common_write32(MODULE_COMMON_RESET_UART_REG, 0x00000001);
		nai_dt2_write32(SERIAL_BAUDRATE_REG, DT2_12M_BAUD_RATE);
		nai_common_write32(MODULE_COMMON_RESET_UART_REG, 0x00000000);

		/* We changed the baud rate...so let's make sure we clear both the TX and RX Ram locations in case there was junk received */
		nai_dt2_write32(SERIAL_CONTROL_REG, (DT2_CTRL_UART | DT2_CTRL_RESET_TX_RAM | DT2_CTRL_RESET_RX_RAM));

#ifdef _DF3_VERBOSE
		printf("About to send CMD 7F to place KL17 into Bootloader mode\n");
#endif

		ucCmdBuf[0] = DT2_FORCE_BOOTLOADER; /* Command to force microcontroller found at specified channel into bootloader mode (if in  operational mode) */
		nStatus = nai_send_data_isolated_dt(ucChanIndex, &ucCmdBuf[0], 1);

		if (nStatus == NAI_SUCCESS)
		{
			/* Give ST Microcontroller just a little bit of time to get into bootloader mode */
			ulTimer = nai_get_timer(0);
			while (1)
			{
				if (nai_get_timer(ulTimer) > 500) /* 100 milliseconds */
					break;
			}
			
			/* Force baud rate back to configuration baud of 115.2K */
			nai_common_write32(MODULE_COMMON_RESET_UART_REG, 0x00000001);
			nai_dt2_write32(SERIAL_BAUDRATE_REG, DT2_CONFIGURATION_115_2K_BAUD_RATE);
//			nai_dt2_write32(SERIAL_BAUDRATE_REG, DT2_ST_DEFAULT_57_6K_BAUD_RATE);
//			nai_dt2_write32(SERIAL_BAUDRATE_REG, DT2_ST_DEFAULT_9600_BAUD_RATE);			
			nai_common_write32(MODULE_COMMON_RESET_UART_REG, 0x00000000);

			/* We changed the baud rate...so let's make sure we clear both the TX and RX Ram locations in case there was junk received */
			nai_dt2_write32(SERIAL_CONTROL_REG, (DT2_CTRL_UART |  DT2_CTRL_RESET_TX_RAM | DT2_CTRL_RESET_RX_RAM)); 
			
			/* Send PING packet to have bootloader auto-detect BAUD rate we are talking at */
			nStatus = nai_kl17_send_ping(ucChanIndex);
		}
		else
			printf("Failed trying to command KL17 microcontroller into boot loader mode!\n");
	}	
	
	return nStatus;
}

static int32_t nai_kl17_send_ping(uint8_t ucChanIndex)
{
	int32_t nStatus = NAI_SUCCESS;
	int32_t ulTimer = 0;
	uint8_t ucCmdBuf[2];	
	int32_t nRXAddress = RX_RAM_START;
	int32_t nBytesToRead = 10;
	uint8_t ucNumRXLocationsToCheck = 20;
	uint8_t i = 0;
	uint8_t ucPingResponseDetected = 0;
		
#ifdef _DF3_VERBOSE
	printf("In nai_kl17_send_ping\n");
#endif
	/* For KL17 Microcontroller, we need to send a PING packet */
	ucCmdBuf[0] = KL17_START_BYTE;
	ucCmdBuf[1] = KL17_PACKET_TYPE_PING;	

	/* Send a ping packet in the hope of auto-negotiation succcess! */
	nStatus = nai_send_data_isolated_dt(ucChanIndex, &ucCmdBuf[0], (sizeof(ucCmdBuf) / sizeof(uint8_t)));

	/* Need to wait for PING Response! (10 Bytes) */
	if (nStatus == NAI_SUCCESS)
	{
		ulTimer = nai_get_timer(0);
		while (!ucPingResponseDetected)
		{			
			if (nai_get_timer(ulTimer) > (DT_COMPLETION_TIMEOUT))
			{
				nStatus = NAI_STM_RX_TIMEOUT;
				printf("RX Timed Out - RX RAM Data[0] = %x\n", (unsigned int)nai_dt2_read32(RX_RAM_START));
				break;
			}			
			
			nRXAddress = RX_RAM_START;
			
			/* NOTE: We may not see the Start Byte in the 1st location..so we look into the RX buffer a specified number of locations before we give up and try again */
			for (i = 0; i < ucNumRXLocationsToCheck; i++)
			{
				/* There is no way for us to know when the response is ready...so let's poll the initial RX location for the Start Bit! */
				/* We are expecting the 1st byte to be the START_BYTE (0x5A) followed by a 1 byte PING RESPONSE CODE (0xA7) then up to 8 bytes of protocol, options and CRC data */
				if (((nai_dt2_read32(nRXAddress) & 0x00FF) == KL17_START_BYTE) &&
					((nai_dt2_read32(nRXAddress+4) & 0x00FF) == KL17_PACKET_TYPE_PING_RESPONSE))
				{
#ifdef _DF3_VERBOSE
					printf("Detected PING RESPONSE at address: 0x%x\n", nRXAddress);
#endif					
					ucPingResponseDetected = 1;
					nBytesToRead -= 2; /* 8 Bytes left to read */
					nRXAddress += 8; /* Advance the pointer pointing to the RAM block of receive data */

					/*NOTE: FIFOs are 32bits but data being placed on FIFO is only BYTE wide */
					while (nBytesToRead > 0)
					{
						nai_dt2_read32(nRXAddress); /* Read data off of the FIFO...but we are really not interested in this data */
						nRXAddress +=4;
						nBytesToRead--;
					}
					break;
				}
				
				/* Move to next location of RX buffer */
				nRXAddress += 4;
			}
		}
	}
	
	return nStatus;
}


/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_st_send_cmd is responsible for sending a command to the ST microcontroller at the specified
channel.
</summary>
<param name="ucChanIndex"> : Input:Zero based index of channel </param>
<param name="ucCmd"> : Input:Hex command to be sent to isolated DT </param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_st_send_cmd(uint8_t ucChanIndex, uint8_t ucCmd)
{
	int32_t nStatus = NAI_SUCCESS;

	uint8_t ucCmdBuf[2];
	ucCmdBuf[0] = ucCmd;  /* Command ID */	
	ucCmdBuf[1] = ~ucCmd; /* Complement of Command ID */

#ifdef _DF3_VERBOSE
	printf("In nai_st_send_cmd\n");
#endif
	
	nStatus = nai_send_data_isolated_dt(ucChanIndex, &ucCmdBuf[0], (sizeof(ucCmdBuf) / sizeof(uint8_t)));
	if (nStatus == NAI_SUCCESS)
		nStatus = nai_st_wait_for_ack();

	return nStatus;
}


/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_send_data_isolated_dt is responsible for sending the desired data to the ST microcontroller found at the
specified channel.
</summary>
<param name="ucChanIndex"> : Input:Zero based index of channel </param>
<param name="pucDataBuf"> : Input:Data Buffer to be written </param>
<param name="ucByteCount"> : Input: Size in bytes of pucDataBuf </param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_send_data_isolated_dt(uint8_t ucChanIndex, uint8_t *pucDataBuf, uint8_t ucByteCount)
{	
	int32_t nStatus = NAI_SUCCESS;
	int32_t nTXAddress = TX_RAM_START;
	int32_t ulTimer = 0;
	uint8_t unDataIndex = 0;
	uint32_t unControlValue = 0;
	
#ifdef _DF3_VERBOSE	
	printf("In nai_send_data_isolated_dt\n");
#endif
	
	/* To send data means we are either writing to memory or simply setting up a command to execute */
	if (ucByteCount == 0 || ucByteCount > MAX_ISOLATED_DT_TX_RAM_IN_BYTES)
		return NAI_INVALID_PARAMETER_VALUE;
	
	/* Clear TX */
	unControlValue = nai_dt2_read32(SERIAL_CONTROL_REG);
	nai_dt2_write32(SERIAL_CONTROL_REG, (unControlValue | DT2_CTRL_RESET_TX_RAM));
	
#ifdef _DF3_VERBOSE	
	printf("After Reset TX RAM\n");
#endif
	
	/* NAI_SUCCESS indicates OK to start writing to TX RAM block */
	if (nStatus == NAI_SUCCESS)
	{		
		/* Write to RAM block */		
		for (unDataIndex = 0; unDataIndex < ucByteCount; unDataIndex++)
		{
			nai_dt2_write32(nTXAddress, (uint32_t)pucDataBuf[unDataIndex]);
			nTXAddress += 4;
		}
	
		nai_dt2_write32(TX_DATA_COUNT, ucByteCount); /* Specify how many bytes will be on the TX line */
		
#ifdef _DF3_VERBOSE		
		printf("After Writing Data to TX RAM but before GO\n");
#endif
		
		/* Now signal we are done writing to TX and data should go out the UART */
		/* We do this by setting Bit 1 to a 1 of the SERIAL_CONTROL_REG */
		unControlValue = nai_dt2_read32(SERIAL_CONTROL_REG);
		nai_dt2_write32(SERIAL_CONTROL_REG, (unControlValue | DT2_CTRL_TX_GO));
		
#ifdef _DF3_VERBOSE		
		printf("After GO TX RAM\n");
#endif		
		/* Now wait for signal that data was sent */
		ulTimer = nai_get_timer(0);
		while (1)
		{			
			if (nai_get_timer(ulTimer) > DT_COMPLETION_TIMEOUT)
			{
				nStatus = NAI_STM_TX_TIMEOUT;
				break;
			}	
				
			/* Read Serial Status to see if data was sent */
			if ((nai_dt2_read32(SERIAL_STATUS_REG) & DT2_STAT_TX_RAM_TRANSMITTED) == DT2_STAT_TX_RAM_TRANSMITTED)
				break;
		}
	}	
#ifdef _DF3_VERBOSE	
	printf("End of nai_send_data_isolated_dt - Status = %d\n", nStatus);
#endif	
	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_st_wait_for_ack is responsible for waiting for an "ACK" from the ST controller. Typically this is called
after each command request.
</summary>
<returns>int32_t : Status
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_st_wait_for_ack()
{
	int32_t nStatus = NAI_SUCCESS;
	int32_t ulTimer = 0;

	ulTimer = nai_get_timer(0);
	while (1)
	{
		if (nai_get_timer(ulTimer) > DT_COMPLETION_TIMEOUT)
		{
			nStatus = NAI_STM_RX_TIMEOUT;
			printf("RX Timed Out - RX RAM Data[0] = %x Data[1] = %x\n", (unsigned int)nai_dt2_read32(RX_RAM_START), (unsigned int)nai_dt2_read32(RX_RAM_START+4));
			break;
		}

#ifdef _DF3_VERBOSE
		printf("Attempting to read SERIAL STATUS to see if RX RAM OK\n");
#endif
		/* Read Serial Status to see if data has been received */
		if ((nai_dt2_read32(SERIAL_STATUS_REG) & DT2_STAT_RX_RAM_OK_TO_READ) == DT2_STAT_RX_RAM_OK_TO_READ)
		{
#ifdef _DF3_VERBOSE
			printf("After read SERIAL STATUS RX RAM OK to read!\n");
#endif
			/* Now that we know RX data is available...read it from the RAM block
			 * We are expecting either an ACK (0x79) or a NACK (0x1F) so read the 1st Byte from the RAM block and see if that is what we have */
			if (nai_dt2_read32(RX_RAM_START) != STM_ACK)
				nStatus = NAI_ACK_NOT_RECEIVED;
			break;
		}
	}

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_kl17_wait_for_ack is responsible for waiting for an "ACK" from the KL17 controller. Typically this is called
after each command request.
</summary>
<returns>int32_t : Status
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_kl17_wait_for_ack()
{
	int32_t nStatus = NAI_SUCCESS;
	int32_t ulTimer = 0;
	int32_t nRXAddress = RX_RAM_START;
	uint8_t ucNumRXLocationsToCheck = 20;
	uint8_t i = 0;
	uint8_t ucAckDetected = 0;
	
	ulTimer = nai_get_timer(0);
	while (!ucAckDetected)
	{
		if (nai_get_timer(ulTimer) > DT_COMPLETION_TIMEOUT)
		{
			nStatus = NAI_STM_RX_TIMEOUT;
			printf("RX Timed Out - RX RAM Data[0] = %x\n", (unsigned int)nai_dt2_read32(RX_RAM_START));
			break;
		}
		
		nRXAddress = RX_RAM_START;
		
		/* NOTE: We may not see the Start Byte in the 1st location..so we look into the RX buffer a specified number of locations before we give up and try again */
		for (i = 0; i < ucNumRXLocationsToCheck; i++)
		{
			/* We are expecting the 1st byte to be the START_BYTE (0x5A) followed by a 1 byte Packet Type ACK (0xA1)*/
			if (((nai_dt2_read32(nRXAddress) & 0x00FF) == KL17_START_BYTE) &&
				((nai_dt2_read32(nRXAddress+4) & 0x00FF) == KL17_PACKET_TYPE_ACK))
			{
	#ifdef _DF3_VERBOSE
				printf("Detected ACK RESPONSE at address: 0x%x\n", nRXAddress);
	#endif			
				ucAckDetected = 1;
				break;
			}
			
			/* Move to next location of RX buffer */
			nRXAddress += 4;
		}
	}

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_kl17_send_ack is responsible for sending an "ACK" to the KL17 controller. Typically this is called to
acknowledge a response packet was received.
</summary>
<param name="ucChanIndex"> : Input:Zero based index of channel </param>
<returns>int32_t : Status
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_kl17_send_ack(uint8_t ucChanIndex)
{
	int32_t nStatus = NAI_SUCCESS;
	uint8_t ucCmdBuf[2];

	/* We need to send an ACK to the KL17 Microcontroller */
	ucCmdBuf[0] = KL17_START_BYTE;
	ucCmdBuf[1] = KL17_PACKET_TYPE_ACK;

	nStatus = nai_send_data_isolated_dt(ucChanIndex, &ucCmdBuf[0], (sizeof(ucCmdBuf) / sizeof(uint8_t)));

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_st_receive_data is responsible for receiving data from the ST microcontroller found at the
specified channel.
</summary>
<param name="ucChanIndex"> : Input:Zero based index of channel </param>
<param name="pucDataBuf"> : Input:Data Buffer to receive data </param>
<param name="ucByteCount"> : Input/Output: Size in bytes of pucDataBuf - Output it is actual number of
							 bytes written to pucDatabuf</param>
<returns>int32_t : Status
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_st_receive_data(uint8_t ucChanIndex, uint8_t *pucDataBuf, uint8_t* pucByteCount)
{
	int32_t nStatus = NAI_SUCCESS;
	int32_t nRXAddress = (RX_RAM_START + 4); /*NOTE - we bump one 32 bit register up since we don't want to consider the ACK which we already read */
	uint32_t unControlValue = 0;
	uint8_t ucNumBytesToRead = 0;
	int32_t ulTimer = 0;
	uint8_t i = 0;

#ifdef _DF3_VERBOSE
	printf("In nai_st_receive_data\n");
#endif

	if (pucDataBuf == NULL)
		return NAI_INVALID_PARAMETER_VALUE;
	
	/* Make sure data is ready to be read */
	ulTimer = nai_get_timer(0);
	while (1)
	{			
		if (nai_get_timer(ulTimer) > DT_COMPLETION_TIMEOUT)
		{
			nStatus = NAI_STM_RX_TIMEOUT;
			break;
		}			
		
		/* Read Serial Status to see if data has been received */
		if ((nai_dt2_read32(SERIAL_STATUS_REG) & DT2_STAT_RX_RAM_OK_TO_READ) == DT2_STAT_RX_RAM_OK_TO_READ)
			break;
	}					
	
	if (nStatus == NAI_SUCCESS)
	{
		do
		{
			/* Now that we know RX data is available...read it from the RAM block */
			/* Read number of bytes to be taken off of the RX RAM block */
			ucNumBytesToRead = (uint8_t)(nai_dt2_read32(nRXAddress) & 0x000000FF);
			nRXAddress += 4;
		} while (ucNumBytesToRead == STM_ACK); /* NOTE: For some reason we seem to see an extra ACK in the beginning that documentation does not call out. Let's skip all consecutive ACKs */
		
		ucNumBytesToRead++; /*NOTE: + 1 since documentation states value read is 1 less than actual bytes */
		
#ifdef _DF3_VERBOSE			
		printf("Num Bytes to Read: %d\n", ucNumBytesToRead);
#endif
		if (ucNumBytesToRead <= *pucByteCount) 
		{	
			*pucByteCount = ucNumBytesToRead;
													
			/* Now we know how much of the RAM block should be "valid" for this request */
			while (ucNumBytesToRead != 0)
			{
				pucDataBuf[i++] = (uint8_t)(nai_dt2_read32(nRXAddress) & 0x000000FF);
				nRXAddress += 4;
				ucNumBytesToRead--;
			}

			/* We should also see an ACK to end the receive */
			if(nai_dt2_read32(nRXAddress) != STM_ACK)
				nStatus = NAI_ACK_NOT_RECEIVED;
		}
		else /* Buffer is not large enough! */
		{
			printf("Potential Buffer Overrun: Bytes to Read = %d Size of Buffer = %d\n", ucNumBytesToRead, *pucByteCount);
			nStatus = NAI_POTENTIAL_BUFFER_OVERRUN;
		}
	}
	
	/* Reset RX RAM */
	unControlValue = nai_dt2_read32(SERIAL_CONTROL_REG);
	nai_dt2_write32(SERIAL_CONTROL_REG, (unControlValue | DT2_CTRL_RESET_RX_RAM));
	
	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_write_memory_isolated_dt is responsible for writing the desired data to the microcontroller found at the
specified channel.
</summary>
<param name="ucChanIndex"> : Input:Zero based index of channel </param>
<param name="pucDataBuf"> : Input:Data Buffer to be written </param>
<param name="ucByteCount"> : Input: Size in bytes of pucDataBuf </param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_write_memory_isolated_dt(uint8_t ucChanIndex, uint32_t unStartAddress, uint8_t *pucWriteBuf, uint8_t ucByteCount)
{
	int32_t nStatus = NAI_SUCCESS;

#ifdef _DF3_VERBOSE
	printf("In nai_write_memory_isolated_dt\n");
#endif

	switch (g_MicrocontrollerType)
	{
	case KL17_MICROCONTROLLER_TYPE :
		nStatus = nai_kl17_write_memory_data(ucChanIndex, pucWriteBuf, ucByteCount);
		break;

	case ST_MICROCONTROLLER_TYPE :
		nStatus = nai_st_write_memory_data(ucChanIndex, unStartAddress, pucWriteBuf, ucByteCount);
		break;

	default :
		nStatus = NAI_COMMAND_NOT_RECOGNIZED;
		break;
	}
	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_st_write_memory_data is responsible for writing the desired data to the ST microcontroller found at the
specified channel.
</summary>
<param name="ucChanIndex"> : Input:Zero based index of channel </param>
<param name="pucDataBuf"> : Input:Data Buffer to be written </param>
<param name="ucByteCount"> : Input: Size in bytes of pucDataBuf </param>
<returns>int32_t : Status
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_st_write_memory_data(uint8_t ucChanIndex, uint32_t unStartAddress, uint8_t *pucWriteBuf, uint8_t ucByteCount)
{
	int32_t nStatus = NAI_SUCCESS;
	uint8_t ucDataBuf[MAX_ISOLATED_DT_TX_RAM_IN_BYTES];
	uint8_t ucHeaderBuf[5];	/* 4 Bytes of StartAddress and 1 Byte of Checksum */
	uint8_t ucCheckSum = 0;
	uint8_t ucDataIndex = 0;
	uint8_t i = 0;
	
#ifdef _DF3_VERBOSE
	printf("In nai_st_write_memory\n");
#endif
	
	if (pucWriteBuf == NULL)
		return NAI_INVALID_PARAMETER_VALUE;
		
	/* Initialize Data Buf to all zeros */
	memset(ucDataBuf, 0, sizeof(ucDataBuf));

	nStatus = nai_st_send_cmd(ucChanIndex, 0x31);
	
	if (nStatus == NAI_SUCCESS)
	{			
		/* Here we tell the ST MCU what the starting address we will be writing to. We also
		 * need to pass down a checksum that we compute by XORing the bytes making up the start address */
		/* MSB of Start Address is Byte 0 for Address!*/
		ucHeaderBuf[0] = (unStartAddress >> 24) & 0x000000FF;
		ucCheckSum = ucCheckSum ^ ucHeaderBuf[0];
		
		ucHeaderBuf[1] = (unStartAddress >> 16) & 0x000000FF;
		ucCheckSum = ucCheckSum ^ ucHeaderBuf[1];
		
		ucHeaderBuf[2] = (unStartAddress >> 8) & 0x000000FF;
		ucCheckSum = ucCheckSum ^ ucHeaderBuf[2];

		ucHeaderBuf[3] = (unStartAddress) & 0x000000FF;
		ucCheckSum = ucCheckSum ^ ucHeaderBuf[3];
		
		ucHeaderBuf[4] = ucCheckSum; /*Checksum - XOR(byte 1, byte 2, byte 3, byte 4) of StartAddress */
		
		nStatus = nai_send_data_isolated_dt(ucChanIndex, &ucHeaderBuf[0], (sizeof(ucHeaderBuf) / sizeof(uint8_t)));

		if (nStatus == NAI_SUCCESS)
			nStatus = nai_st_wait_for_ack();
		
		/* If NAI_SUCCESS, we are ready to send the data we want written to memory! */
		if (nStatus == NAI_SUCCESS)
		{
			ucDataBuf[ucDataIndex] = (ucByteCount - 1); /* First specify how much data needs to be written - 1 (based upon ST MCU documentation) */
			ucCheckSum = ucDataBuf[ucDataIndex]; /* Initialize check sum since we need to compute another checksum of the actual data to send */
			ucDataIndex++;
			
			for (i = 0; i < ucByteCount; i++)
			{
				ucDataBuf[ucDataIndex++] = pucWriteBuf[i];
				ucCheckSum = (ucCheckSum ^ pucWriteBuf[i]);
			}
			
			ucDataBuf[ucDataIndex++] = ucCheckSum;
			
			nStatus = nai_send_data_isolated_dt(ucChanIndex, &ucDataBuf[0], ucDataIndex);
			if (nStatus == NAI_SUCCESS)
				nStatus = nai_st_wait_for_ack();
		}
	}

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_kl17_write_memory_data is responsible for writing the desired data to the KL17 microcontroller found at the
specified channel.
</summary>
<param name="ucChanIndex"> : Input:Zero based index of channel </param>
<param name="pucDataBuf"> : Input:Data Buffer to be written </param>
<param name="ucByteCount"> : Input: Size in bytes of pucDataBuf </param>
<returns>int32_t : Status
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_kl17_write_memory_data(uint8_t ucChanIndex, uint8_t *pucWriteBuf, uint8_t ucByteCount)
{
	int32_t nStatus = NAI_SUCCESS;
	uint32_t unControlValue = 0;
	uint8_t ucDataBuf[38];
	uint16_t usCRC = 0;
	uint8_t i = 0;
	uint8_t ucPacketHeaderByteLength = 6;
	
#ifdef _DF3_VERBOSE
	printf("In nai_kl17_write_memory_data\n");
#endif

	/* Clear TX and RX RAM */
	unControlValue = nai_dt2_read32(SERIAL_CONTROL_REG);
	nai_dt2_write32(SERIAL_CONTROL_REG, (unControlValue | DT2_CTRL_RESET_TX_RAM | DT2_CTRL_RESET_RX_RAM));

	/* Framing Packet */
	/*Byte 0: Start Byte (0x5A)*/
	/*Byte 1: Packet Type (Data Pkt - 0xA5)*/
	/*Byte 2: Length Low (0x20)*/
	/*Byte 3: Length High (0x00)*/
	/*Byte 4: CRC16 Low (0x00)*/
	/*Byte 5: CRC16 High (0x00)*/
	/*Byte 6...N: Data Packet Payload = 32 Bytes*/

	/* Signify Data Packet */
	ucDataBuf[0] = 0x5A; /*Start Bit*/
	ucDataBuf[1] = 0xA5; /*Data Packet*/
	ucDataBuf[2] = (ucByteCount & 0xFF); /*Byte Count (low)*/ 
	ucDataBuf[3] = 0; /*Byte Count (High) - Should never be used??*/

	/* Fill in Data */
	for (i=0; i < ucByteCount; i++)
		ucDataBuf[ucPacketHeaderByteLength+i] = pucWriteBuf[i];

	/* Calculate the CRC */
	usCRC = nai_kl17_crc16_update(&ucDataBuf[0], (ucByteCount + ucPacketHeaderByteLength));

	/* Now fill in the CRC value */
	ucDataBuf[4] = (usCRC & 0x00FF);
	ucDataBuf[5] = ((usCRC >> 8) & 0x00FF);

	nStatus = nai_send_data_isolated_dt(ucChanIndex, &ucDataBuf[0], (ucByteCount + ucPacketHeaderByteLength));

	/* Now we need to wait for the initial ACK in response to us sending the WriteMemory command */
	if (nStatus == NAI_SUCCESS)
		nStatus = nai_kl17_wait_for_ack();

	/* Clear TX RAM */
	unControlValue = nai_dt2_read32(SERIAL_CONTROL_REG);
	nai_dt2_write32(SERIAL_CONTROL_REG, (unControlValue | DT2_CTRL_RESET_TX_RAM));

	return nStatus;
}


static int32_t nai_kl17_write_memory_data_get_final_response(uint8_t ucChanIndex)
{
	int32_t nStatus = NAI_SUCCESS;
	uint32_t unStatusCode = 0;
	
	nStatus = nai_kl17_get_generic_response(KL17_WRITE_MEMORY_CMD, 0, 0, &unStatusCode);

	/* If we did indeed receive a generic response for the Write Memory command, we need to send an ACK back */
	if (nStatus == NAI_SUCCESS)
		nStatus = nai_kl17_send_ack(ucChanIndex);
	else /* Error - we can interpret unStatusCode to understand error that took place */
	{
		printf("Call to nai_Kl17_get_generic_response returned error code: 0x%x\n", (unsigned int)nStatus);
		/*TODO - create function to look up text description of returned status code (unStatusCode)*/
	}
	
	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_kl17_write_memory_cmd is responsible for sending the write memory command to the microcontroller which 
tells the microcontroller that data is about to be sent, how much data and what address to start writting to.
</summary>
<param name="ucChanIndex"> : Input:Zero based index of channel </param>
<param name="unStartAddress"> : Start address of where to start flashing </param>
<param name="ucByteCount"> : Input: Size in bytes of pucDataBuf </param>
<returns>int32_t : Status
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_kl17_write_memory_cmd(uint8_t ucChanIndex, uint32_t unStartAddress, uint32_t unByteCount)
{
	int32_t nStatus = NAI_SUCCESS;
	uint32_t unControlValue = 0;
	uint8_t ucCmdBuf[18] = {0x5A,0xA4,0x0C,0x00,0x00,0x00,0x04,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
	uint32_t unStatusCode = 0;
	uint16_t usCRC = 0;	
	
#ifdef _DF3_VERBOSE
	printf("In nai_kl17_write_memory_cmd\n");
#endif

	/* Clear TX and RX RAM */
	unControlValue = nai_dt2_read32(SERIAL_CONTROL_REG);
	nai_dt2_write32(SERIAL_CONTROL_REG, (unControlValue | DT2_CTRL_RESET_TX_RAM | DT2_CTRL_RESET_RX_RAM));

	/* Framing Packet */
	/*Byte 0: Start Byte (0x5A)*/
	/*Byte 1: Packet Type (Command Pkt - 0xA4)*/
	/*Byte 2: Length Low (0x0C)*/
	/*Byte 3: Length High (0x00)*/
	/*Byte 4: CRC16 Low (0x06)*/
	/*Byte 5: CRC16 High (0x5A)*/
	/*Byte 6...N: Command Packet - Command Header = 4 Bytes*/
		/*Byte 6: Tag - (0x04 = WriteMemory)*/
		/*Byte 7: Flags - (0x00)*/
		/*Byte 8: Rsvd - (0x00)*/
		/*Byte 9: ParamCount - (0x02)*/
		/*Byte 10: Param 1 -(unStartAddress & 0xFF) */
		/*Byte 11: Param 1 -((unStartAddress >> 8) & 0xFF) */
		/*Byte 12: Param 1 -((unStartAddress >> 16) & 0xFF) */
		/*Byte 13: Param 1 -((unStartAddress >> 24) & 0xFF) */
		/*Byte 14: Param 2 - (ucByteCount & 0xFF) */
		/*Byte 15: Param 2 - () */ 
		/*Byte 16: Param 2 - () */ 
		/*Byte 17: Param 2 - () */ 

	/* Update the Start Address of where to write to */
	ucCmdBuf[10] = (unStartAddress & 0xFF);
	ucCmdBuf[11] = ((unStartAddress >> 8) & 0xFF);
	ucCmdBuf[12] = ((unStartAddress >> 16) & 0xFF);
	ucCmdBuf[13] = ((unStartAddress >> 24) & 0xFF);

	/* Update the Byte Count to write */
	ucCmdBuf[14] = (unByteCount & 0xFF);
	ucCmdBuf[15] = ((unByteCount >> 8) & 0xFF);
	ucCmdBuf[16] = ((unByteCount >> 16) & 0xFF);
	ucCmdBuf[17] = ((unByteCount >> 24) & 0xFF);
	
	/* Calculate the CRC */
	usCRC = nai_kl17_crc16_update(&ucCmdBuf[0], (sizeof(ucCmdBuf) / sizeof(uint8_t)));

	/* Now fill in the CRC value */
	ucCmdBuf[4] = (usCRC & 0x00FF);
	ucCmdBuf[5] = ((usCRC >> 8) & 0x00FF);

	nStatus = nai_send_data_isolated_dt(ucChanIndex, &ucCmdBuf[0], (sizeof(ucCmdBuf) / sizeof(uint8_t)));

	/* Now we need to wait for the initial ACK in response to us sending the WriteMemory command */
	if (nStatus == NAI_SUCCESS)
	{
		nStatus = nai_kl17_wait_for_ack();

		/* If we received the initial ACK from sending the FlashEraseAll command, we need to wait for a "GENERIC RESPONSE" */
		if (nStatus == NAI_SUCCESS)
		{
			nStatus = nai_kl17_get_generic_response(KL17_WRITE_MEMORY_CMD, 0, 0, &unStatusCode);

			/* If we did indeed receive a generic response for the Write Memory command, we need to send an ACK back */
			if (nStatus == NAI_SUCCESS)
				nStatus = nai_kl17_send_ack(ucChanIndex);
			else /* Error - we can interpret unStatusCode to understand error that took place */
			{
				printf("Call to nai_Kl17_get_generic_response returned error code: 0x%x\n", (unsigned int)nStatus);
				/*TODO - create function to look up text description of returned status code (unStatusCode)*/
			}
		}
	}

	/* Clear TX and RX RAM */	
	unControlValue = nai_dt2_read32(SERIAL_CONTROL_REG);
	nai_dt2_write32(SERIAL_CONTROL_REG, (unControlValue | DT2_CTRL_RESET_TX_RAM | DT2_CTRL_RESET_RX_RAM));

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_perform_erase_isolated_dt is responsible for erasing the entire microcontroller flash found at the
specified channel.
</summary>
<param name="ucChanIndex"> : Input:Zero based index of channel </param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_perform_erase_isolated_dt(uint8_t ucChanIndex)
{
	int32_t nStatus = NAI_SUCCESS;

#ifdef _DF3_VERBOSE
	printf("In nai_perform_erase_isolated_dt\n");
#endif

	switch (g_MicrocontrollerType)
	{
	case KL17_MICROCONTROLLER_TYPE :
		nStatus = nai_kl17_perform_erase(ucChanIndex);
		break;

	case ST_MICROCONTROLLER_TYPE :
		nStatus = nai_st_perform_erase(ucChanIndex);
		break;

	default :
		nStatus = NAI_COMMAND_NOT_RECOGNIZED;
		break;
	}

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_st_perform_erase is responsible for erasing the entire ST microcontroller flash found at the
specified channel.
</summary>
<param name="ucChanIndex"> : Input:Zero based index of channel </param>
<returns>int32_t : Status
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_st_perform_erase(uint8_t ucChanIndex)
{
	int32_t nStatus = NAI_SUCCESS;
	uint32_t unControlValue = 0;
	uint8_t ucCmdBuf[2];
	int32_t ulTimer = 0;

#ifdef _DF3_VERBOSE
	printf("In nai_st_perform_erase\n");
#endif

	/* Clear TX and RX RAM */
	unControlValue = nai_dt2_read32(SERIAL_CONTROL_REG);
	nai_dt2_write32(SERIAL_CONTROL_REG, (unControlValue | DT2_CTRL_RESET_TX_RAM | DT2_CTRL_RESET_RX_RAM));
	
	/* Write 0x43 to TxRAM block and then signal a "GO" */
	nStatus = nai_st_send_cmd(ucChanIndex, STM_ERASE_MEM_CMD);
	
	if (nStatus == NAI_SUCCESS)
	{							
		ucCmdBuf[0] = 0xFF;  /* Total Erase Command Combination (0xFF followed by 0x00) */
		ucCmdBuf[1] = 0x00;
		nStatus = nai_send_data_isolated_dt(ucChanIndex, &ucCmdBuf[0], (sizeof(ucCmdBuf) / sizeof(uint8_t)));
		if (nStatus == NAI_SUCCESS)
			nStatus = nai_st_wait_for_ack();
		
		/* Force a delay between erasing the ST and allowing a write */
		ulTimer = nai_get_timer(0);
		while (1)
		{			
			if (nai_get_timer(ulTimer) > 1500)
				break;
		}
	}			

	/* Clear TX and RX RAM */
	unControlValue = nai_dt2_read32(SERIAL_CONTROL_REG);
	nai_dt2_write32(SERIAL_CONTROL_REG, (unControlValue | DT2_CTRL_RESET_TX_RAM | DT2_CTRL_RESET_RX_RAM));
	
	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_kl17_perform_erase is responsible for erasing the entire KL17 microcontroller flash found at the
specified channel.
</summary>
<param name="ucChanIndex"> : Input:Zero based index of channel </param>
<returns>int32_t : Status
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_kl17_perform_erase(uint8_t ucChanIndex)
{
	int32_t nStatus = NAI_SUCCESS;
	uint32_t unControlValue = 0;
	uint8_t ucCmdBuf[10] = {0x5A,0xA4,0x04,0x00,0xF6,0x61,0x0D,0x00,0x00,0x00};
	uint32_t unStatusCode = 0;
	int32_t ulTimer = 0;

#ifdef _DF3_VERBOSE
	printf("In nai_kl17_perform_erase\n");
#endif

	/* Clear TX and RX RAM */
	unControlValue = nai_dt2_read32(SERIAL_CONTROL_REG);
	nai_dt2_write32(SERIAL_CONTROL_REG, (unControlValue | DT2_CTRL_RESET_TX_RAM | DT2_CTRL_RESET_RX_RAM));

	/* Framing Packet */
	/*Byte 0: Start Byte (0x5A)*/
	/*Byte 1: Packet Type (Command Pkt - 0xA4)*/
	/*Byte 2: Length Low (0x04)*/
	/*Byte 3: Length High (0x00)*/
	/*Byte 4: CRC16 Low (0xF6)*/
	/*Byte 5: CRC16 High (0x61)*/
	/*Byte 6...N: Command Packet - Command Header = 4 Bytes*/
		/*Byte 6: Tag - (0x0D = FlashEraseAllUnsecure)*/
		/*Byte 7: Flags - (0x00)*/
		/*Byte 8: Rsvd - (0x00)*/
		/*Byte 9: ParamCount - (0x00)*/
	nStatus = nai_send_data_isolated_dt(ucChanIndex, &ucCmdBuf[0], (sizeof(ucCmdBuf) / sizeof(uint8_t)));

	/* Now we need to wait for the initial ACK in response to us sending the FlashEraseAll command */
	if (nStatus == NAI_SUCCESS)
	{
		nStatus = nai_kl17_wait_for_ack();

		/* If we received the initial ACK from sending the FlashEraseAll command, we need to wait for a "GENERIC RESPONSE" */
		if (nStatus == NAI_SUCCESS)
		{
			nStatus = nai_kl17_get_generic_response(KL17_FLASH_ERASE_ALL_UNSECURE_CMD, 0x54, 0x81, &unStatusCode);

			/* If we did indeed receive a generic response for the FlashEraseAll command, we need to send an ACK back */
			if (nStatus == NAI_SUCCESS)
				nai_kl17_send_ack(ucChanIndex);
			else /* Error - we can interpret unStatusCode to understand error that took place */
			{
				printf("Call to nai_Kl17_get_generic_response returned error code: 0x%x\n", (unsigned int)nStatus);
				/*TODO - create function to look up text description of returned status code (unStatusCode)*/
			}
		
			/* Force a delay between erasing the KL17 and allowing a write */
			ulTimer = nai_get_timer(0);
			while (1)
			{			
				if (nai_get_timer(ulTimer) > 1500)
					break;
			}						
		}
	}
	
	/* Clear TX and RX RAM */
	unControlValue = nai_dt2_read32(SERIAL_CONTROL_REG);
	nai_dt2_write32(SERIAL_CONTROL_REG, (unControlValue | DT2_CTRL_RESET_TX_RAM | DT2_CTRL_RESET_RX_RAM));
	
	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_config_isolated_dt is responsible for writing the desired configuration data to the microcontroller(s).
This command works on a single channel or on all channels if value 255 is detected as the "ChipID". 
</summary>
<param name="ptMsgPacketList"> : Input: Serdes request to configure microcontrollers - data to configure
the chips with will be found in this linked list of packets </param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_config_isolated_dt(MsgPacketList *ptMsgPacketList)
{	
	int32_t nOverallStatus = NAI_SUCCESS;			
	int32_t nStatus = NAI_SUCCESS;
	uint8_t ucNumChannels = NUM_DT_CHANNELS;
	uint8_t ucChanRequest = 0;
	uint8_t ucChanIndex = 0; 
	MsgPacket *ptMsgPacket = NULL;
	WORDValue tWordValue;
	uint8_t ucWriteBuf[MAX_ISOLATED_DT_WRITE_BUFFER_IN_BYTES];   /* Can only write max of 128 bytes of config file at a single clip (ST = 128, KL17 = 32)! */
	uint8_t ucWriteIndex = 0;
	uint16_t usPayloadIndex = 0;
	uint32_t unStartAddress = 0;
	uint16_t usPayloadLengthInWords = 0;	
	uint32_t unPayloadByteCountRunningTally = 0;
	int32_t ulTimer = 0;
	uint32_t unTotalWordsToWrite = 0;
	
#ifdef _DF3_VERBOSE							
	printf("In nai_config_isolated_dt\n");
#endif		
		
	uint16_t usMaxPacketWrite = nai_get_max_packet_write_in_bytes(g_MicrocontrollerType);
	
	if (ptMsgPacketList != NULL && ptMsgPacketList->ptStart != NULL)
	{
		ptMsgPacket = ptMsgPacketList->ptStart;
		
		if (ptMsgPacket != NULL)
		{
			/* Fetch Channel */
			ucChanRequest = ((uint8_t)ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usChipID);
			if (ucChanRequest > NUM_DT_CHANNELS && ucChanRequest != ALL_CHANNELS)
			{
				nStatus = NAI_INVALID_PARAMETER_VALUE;
				printf("Invalid Parameter Detected - ucChanRequest = %u\n", ucChanRequest);
			}
							
			if (nStatus == NAI_SUCCESS)
			{				
				if (ucChanRequest != ALL_CHANNELS)
				{
					ucChanIndex = ucChanRequest - 1;
					ucNumChannels = (ucChanIndex + 1);
				}
				
				for (; ucChanIndex < ucNumChannels; ucChanIndex++)
				{	
					nStatus = NAI_SUCCESS;
					
					/* Each channel will be flashed the same information..so we need to reset our pointer back to the start of the message data */
					ptMsgPacket = ptMsgPacketList->ptStart;
					ucWriteIndex = 0;
						
					printf("\n*************************************************************************************\n");
					if (g_MicrocontrollerType == KL17_MICROCONTROLLER_TYPE)
						printf("Configuring KL17 Microcontroller found at channel %u...\n", ucChanIndex+1);
					else
						printf("Configuring ST Microcontroller found at channel %u...\n", ucChanIndex+1);

					/* Let's start with a cleared write buffer */	
					memset(ucWriteBuf, 0, sizeof(ucWriteBuf));				
					
					/* Force current channel to go into bootloader config mode */
					if (!g_bAlreadyInBootloader[ucChanIndex])
						nStatus = nai_put_isolated_dt_in_config_mode(ucChanIndex);
		
					if (nStatus == NAI_SUCCESS)
					{							
						printf("Erasing...\n");
						nStatus = nai_perform_erase_isolated_dt(ucChanIndex);
																		
						if (nStatus == NAI_SUCCESS)
						{
							printf("Configuring...\n");
												
							/* For KL17 Microcontroller - we must first command the microcontroller telling it that we will be sending data..and then send the 
							   data packets! */
							if (g_MicrocontrollerType == KL17_MICROCONTROLLER_TYPE)
							{
								unStartAddress = KL17_FLASH_OFFSET;
								unTotalWordsToWrite = (ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.unMsgLength - (ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usExpectedSequenceCount * CONFIG_TOTAL_PKT_HDR_IN_WORDS));
								
								nStatus = nai_kl17_write_memory_cmd(ucChanIndex, unStartAddress, (unTotalWordsToWrite*2)); /*We need to tell KL17 how many "BYTES" we will be flashing*/
								if (nStatus != NAI_SUCCESS)
								{
									/* Store 1st error so we can return error code even if all channels pass except 1 */
									if (nOverallStatus == NAI_SUCCESS)
										nOverallStatus = nStatus;
									break;
								}
							}
							else
								unStartAddress = STM_FLASH_OFFSET;
							
							while ( (nStatus == NAI_SUCCESS) && (ptMsgPacket != NULL) )
							{
								usPayloadLengthInWords = convert_bytes_to_words(ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength);
								
								while( (nStatus == NAI_SUCCESS) && (usPayloadIndex < usPayloadLengthInWords) )
								{													
									/* Fill as much of ucWriteBuf as we can with data in this message..if we reach the max ucWriteBuf can handle, send data to isolated DT and
									 * then reset ucWriteBuf and continue from where we left off */
									tWordValue.usValue = ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandPayLd.usData[usPayloadIndex++];
									ucWriteBuf[ucWriteIndex++] = tWordValue.ucLoByte;
									unPayloadByteCountRunningTally++;
									
									/* Only write the Hi Byte if caller sent enough data - this addresses last byte of data falling on an odd 
									   boundary - With SERDES we send 16 bit WORDS so we have to pad the last byte if on an odd boundary with a hi byte of a zero. */
	#ifdef _DEBUG_X
	if ((unPayloadByteCountRunningTally + 5) >= ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength)
		printf("unPayloadByteCountRunningTally = %u  - PacketPayloadLength = 0x%x (%u)\n", unPayloadByteCountRunningTally, ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength, ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength);
	#endif
									if (unPayloadByteCountRunningTally < ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength)
									{
										ucWriteBuf[ucWriteIndex++] = tWordValue.ucHiByte;
										unPayloadByteCountRunningTally++;				
	#ifdef _DEBUG_X
	printf("ucWriteBuf[%u] = 0x%2x\n", (ucWriteIndex-1), ucWriteBuf[(ucWriteIndex-1)]);
	#endif
									}	
						
									if (ucWriteIndex >= usMaxPacketWrite)
									{
	#ifdef _DEBUG_X
	printf("Reached usMaxPacketWrite - 0x%x\n", ucWriteIndex);
	printf("unStartAddress = 0x%x\n", unStartAddress);
	#endif
										/* We have maxed out the WRITE buffer so now we must pass it off to the specified channel's isolated DT to flash */
										nStatus = nai_write_memory_isolated_dt(ucChanIndex, unStartAddress, &ucWriteBuf[0], ucWriteIndex);
										unStartAddress += (ucWriteIndex); /* We just wrote ucWriteIndex bytes so we need to increment the start address for the next write block */
										
										ucWriteIndex = 0; /* Reset the write index and reinitialize the write buffer to all zeros */
										memset(ucWriteBuf, 0, sizeof(ucWriteBuf));
									}					
								}		
								
								usPayloadIndex = 0; /* Reset the Payload Index since we will be moving onto a new message packet */
								unPayloadByteCountRunningTally = 0; /* Reset the Payload Byte Count Running Tally since we are moving to a new message packet */
								ptMsgPacket = ptMsgPacket->ptNext;								
							}									
						}
						else
							printf("Failed to erase Microcontroller! Status = %d\n", (int)nStatus);
			
						/* Is there information left in the Write Buffer that we need to write out to the isolated DT at the specified channel? */
						if ( (nStatus == NAI_SUCCESS) && (ucWriteIndex > 0) )
						{
#ifdef _DF3_VERBOSE							
							printf("Additional Info after loop - amt in buffer: 0x%x (%u)\n", ucWriteIndex, ucWriteIndex);
#endif							
							nStatus = nai_write_memory_isolated_dt(ucChanIndex, unStartAddress, &ucWriteBuf[0], ucWriteIndex);
						}
					}
					
					/* Here we need to get final response from KL17 Microcontroller to finish out the Write Memory command */
					if (nStatus == NAI_SUCCESS && g_MicrocontrollerType == KL17_MICROCONTROLLER_TYPE)
						nStatus = nai_kl17_write_memory_data_get_final_response(ucChanIndex);
					
					/* Force a delay before we return - it has been witnessed that even though the ST Microcontroller gives us status of success, 
					 * immediately shutting off the module after returning from this function causes data to not get written (almost like it was buffered and than lost! */
					ulTimer = nai_get_timer(0);
					while (1)
					{			
						if (nai_get_timer(ulTimer) > 2000)
							break;
					}	
										
					if (nStatus == NAI_SUCCESS)
					{
						if (g_MicrocontrollerType == KL17_MICROCONTROLLER_TYPE)
							printf("Successfully Configured KL17 Microcontroller!\n");
						else
							printf("Successfully Configured ST Microcontroller!\n");
					}
					else /* An error was detected at some point programming this channel microcontroller so we perform an erase in attempt to keep things in a good state */
					{
						printf("FAILED to Configure - Microcontroller! Status = %d\n", (int)nStatus);
						
						/* Only attempt erase of channel if we were able to get into the boot loader...otherwise, there is no point */
						if (g_bAlreadyInBootloader[ucChanIndex])
						{
							printf("Erasing entire Microcontroller since error detected during configuration attempt.\n");
							nai_perform_erase_isolated_dt(ucChanIndex);	
						}
						
						/* Store 1st error so we can return error code even if all channels pass except 1 */
						if (nOverallStatus == NAI_SUCCESS)
							nOverallStatus = nStatus;						
					}
					printf("*************************************************************************************\n");																							
				}					
			}
		}
	}

//	print_nai_msg(ptMsgPacketList, TRUE);

	return nOverallStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_get_version_and_supported_functions_isolated_dt is responsible for getting version and supported 
function information. 
</summary>
<param name="ucChanIndex"> : Input:Zero based index of channel </param>
<param name="pucDataBuf"> : Output: Data Buffer of requested information </param>
<param name="pucByteCount"> : Output: How many bytes can be found in the Data Buf</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_get_version_and_supported_functions_isolated_dt(uint8_t ucChanIndex, uint8_t *pucDataBuf, uint8_t *pucByteCount)
{
	int32_t nStatus = NAI_SUCCESS;

	switch (g_MicrocontrollerType)
	{
	case KL17_MICROCONTROLLER_TYPE :
		nStatus = nai_kl17_get_version_and_supported_functions(ucChanIndex, pucDataBuf, pucByteCount);
		break;
	case ST_MICROCONTROLLER_TYPE :
		nStatus = nai_st_get_version_and_supported_functions(ucChanIndex, pucDataBuf, pucByteCount);
		break;
	default :
		nStatus = NAI_COMMAND_NOT_RECOGNIZED;
		break;
	}

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_st_get_version_and_supported_functions is responsible for getting version and supported 
function information from the ST Microcontroller. 
</summary>
<param name="ucChanIndex"> : Input:Zero based index of channel </param>
<param name="pucDataBuf"> : Output: Data Buffer of requested information </param>
<param name="pucByteCount"> : Output: How many bytes can be found in the Data Buf</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_st_get_version_and_supported_functions(uint8_t ucChanIndex, uint8_t *pucDataBuf, uint8_t *pucByteCount)
{
	int32_t nStatus = NAI_SUCCESS;

#ifdef _DF3_VERBOSE							
	printf("In nai_st_get_version_and_supported_functions\n");
#endif		

	/* Write 0x00 to TxRAM block and then signal a "GO" */
	nStatus = nai_st_send_cmd(ucChanIndex, STM_GET_CMD);

	if (nStatus == NAI_SUCCESS)
	{
#ifdef _DF3_VERBOSE
		printf("STM_GET_CMD sent OK\n");
#endif
		nStatus = nai_st_receive_data(ucChanIndex, pucDataBuf, pucByteCount);

		if (nStatus != NAI_SUCCESS)
			printf("Failed nai_st_receive_data - Status = %d\n", (int)nStatus);

	}

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_kl17_get_version_and_supported_functions is responsible for getting version and supported 
function information from the KL17 Microcontroller. 
NOTE: It appears the KL17 GetProperty Command Tag (0x07) does not return multiple property values as was 
      alluded to in the documentation. So the caller can expect only the response to the 1st parameter requested
      which is currently the version of the microcontroller (0x01). Since we are not using this function for
      any production purpose, we are not spending the time to circumvent this issue / misunderstanding.
</summary>
<param name="ucChanIndex"> : Input:Zero based index of channel </param>
<param name="pucDataBuf"> : Output: Data Buffer of requested information </param>
<param name="pucByteCount"> : Output: How many bytes can be found in the Data Buf</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_kl17_get_version_and_supported_functions(uint8_t ucChanIndex, uint8_t *pucDataBuf, uint8_t *pucByteCount)
{
	int32_t nStatus = NAI_SUCCESS;
	uint32_t unControlValue = 0;
	uint8_t ucCmdBuf[18] = {0x5A,0xA4,0x0C,0x00,0x00,0x00,0x07,0x00,0x00,0x02,0x01,0x00,0x00,0x00,0x07,0x00,0x00,0x00}; /*Property 01 is Current Version, Property 07 is Available Commands*/
	uint32_t unStatusCode = 0;
	uint16_t usCRC = 0;

#ifdef _DF3_VERBOSE							
	printf("In nai_kl17_get_version_and_supported_functions\n");
#endif		

	printf("NOTE: It appears the KL17 GetProperty Command Tag (0x07) does not return multiple property values as was alluded to in the documentation.\n"); 
	printf("So the caller can expect only the response to the 1st parameter requested which is currently the version of the microcontroller (0x01).\n");
	printf("Since we are not using this function for any production purpose, we are not spending the time to circumvent this issue\n");

	/* Clear TX and RX RAM */
	unControlValue = nai_dt2_read32(SERIAL_CONTROL_REG);
	nai_dt2_write32(SERIAL_CONTROL_REG, (unControlValue | DT2_CTRL_RESET_TX_RAM | DT2_CTRL_RESET_RX_RAM));

	/* Calculate the CRC */
	usCRC = nai_kl17_crc16_update(&ucCmdBuf[0], (sizeof(ucCmdBuf) / sizeof(uint8_t)));

	/* Now fill in the CRC value */
	ucCmdBuf[4] = (usCRC & 0x00FF);
	ucCmdBuf[5] = ((usCRC >> 8) & 0x00FF);

	/* Framing Packet */
	/*Byte 0: Start Byte (0x5A)*/
	/*Byte 1: Packet Type (Command Pkt - 0xA4)*/
	/*Byte 2: Length Low (0x0C)*/
	/*Byte 3: Length High (0x00)*/
	/*Byte 4: CRC16 Low (0x00)*/
	/*Byte 5: CRC16 High (0x00)*/
	/*Byte 6...N: Command Packet - Command Header = 9 Bytes*/
		/*Byte 6: Tag - (0x02 = GetProperty)*/
		/*Byte 7: Flags - (0x00)*/
		/*Byte 8: Rsvd - (0x00)*/
		/*Byte 9: ParamCount - (0x00)*/
	nStatus = nai_send_data_isolated_dt(ucChanIndex, &ucCmdBuf[0], (sizeof(ucCmdBuf) / sizeof(uint8_t)));

	/* Now we need to wait for the initial ACK in response to us sending the FlashEraseAll command */
	if (nStatus == NAI_SUCCESS)
	{
		nStatus = nai_kl17_wait_for_ack();

		/* If we received the initial ACK from sending the FlashEraseAll command, we need to wait for a "GET PROPERTY RESPONSE" */
		if (nStatus == NAI_SUCCESS)
		{			
			nStatus = nai_kl17_get_property_response(&unStatusCode, 3 /*3 properties requested (status, current id and available functions*/, pucDataBuf);
						
			/* If we did indeed receive a property response for the command, we need to send an ACK back */
			if (nStatus == NAI_SUCCESS)
			{
				nai_kl17_send_ack(ucChanIndex);			
			}
			else /* Error - we can interpret unStatusCode to understand error that took place */
			{
				printf("Call to nai_kl17_get_property_response returned error code: 0x%x\n", (unsigned int)nStatus);
				/*TODO - create function to look up text description of returned status code (unStatusCode)*/
			}
		}
	}

	/* Clear TX and RX RAM */
	unControlValue = nai_dt2_read32(SERIAL_CONTROL_REG);
	nai_dt2_write32(SERIAL_CONTROL_REG, (unControlValue | DT2_CTRL_RESET_TX_RAM | DT2_CTRL_RESET_RX_RAM));

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_kl17_crc16_update is responsible for updating the CRC16 for the KL17 microcontroller based upon the 
buffer passed in.
</summary>
<param name="src"> : Input: Input Buffer to get CRC16 for </param>
<param name="lengthInBytes"> : Input: Length in bytes of the src buffer passed in </param>
<returns>int32_t : CRC16 value 
</returns>
*/
/**************************************************************************************************************/
static uint16_t nai_kl17_crc16_update(const uint8_t * src, uint32_t lengthInBytes)
{
	uint32_t crc = 0;
	uint32_t j;

#ifdef _DF3_VERBOSE							
	printf("In nai_kl17_crc16_update\n");
#endif		

	for (j=0; j < lengthInBytes; ++j)
	{
		/*SKIP THE CRC BYTES*/
		if ((j == 4) || (j == 5))
			continue;
			
		uint32_t i;
		uint32_t byte = src[j];
		crc ^= byte << 8;

		for (i=0; i < 8; ++i)
		{
			uint32_t temp = crc << 1;
			if (crc & 0x8000)
				temp ^= 0x1021;
			crc = temp;
		}
	}

	return crc;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_kl17_get_generic_response is responsible for returning a generic response from a KL17 command.
</summary>
<param name="ucCmd"> : Input: Cmd tag that the response should be in response of. </param>
<param name="ucCRC16Low"> : Input: Lower 8 bytes of the CRC16 value </param>
<param name="ucCRC16High"> : Input: Upper 8 bytes of the CRC16 value </param>
<param name="ucCRC16High"> : Output: punStatusCode - status of generic response packet (0 = Success) </param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_kl17_get_generic_response(uint8_t ucCmd, uint8_t ucCRC16Low, uint8_t ucCRC16High, uint32_t *punStatusCode)
{
	int32_t nStatus = NAI_SUCCESS;
	int32_t ulTimer = 0;
	int32_t nRXAddress = RX_RAM_START;
	int32_t nCmdPayloadLength = 0;
	uint8_t ucParamCount = 0;
	uint8_t ucResponseCmd = 0;
	uint8_t ucResponseReceived = 0;
	uint8_t ucNumRXLocationsToCheck = 20;
	uint8_t i = 0;
	
#ifdef _DF3_VERBOSE							
	printf("In nai_kl17_get_generic_response\n");
#endif		

	ulTimer = nai_get_timer(0);
	while (!ucResponseReceived)
	{
		if (nai_get_timer(ulTimer) > DT_COMPLETION_TIMEOUT)
		{
			nStatus = NAI_STM_RX_TIMEOUT;
			printf("RX Timed Out - RX RAM Data at address 0x%x = 0x%x\n", RX_RAM_START, (unsigned int)nai_dt2_read32(RX_RAM_START));
			break;
		}
		
		nRXAddress = RX_RAM_START;
		
		/* NOTE: We may not see the Start Byte in the 1st location..so we look into the RX buffer a specified number of locations before we give up and try again */
		for (i = 0; i < ucNumRXLocationsToCheck; i++)
		{
			/* Now that we know RX data is available...read it from the RAM block */
			/* We are expecting the 1st byte to be the START_BYTE (0x5A) followed by a 1 byte Packet Type Command (0xA4)*/
			if (((nai_dt2_read32(nRXAddress) & 0x00FF) == KL17_START_BYTE) &&
				((nai_dt2_read32(nRXAddress+4) & 0x00FF) == KL17_PACKET_TYPE_COMMAND))
			{
#ifdef _DF3_VERBOSE
				printf("Detected ACK at address: 0x%x\n", nRXAddress);
#endif
				nRXAddress += 8;
				nCmdPayloadLength = ((nai_dt2_read32(nRXAddress) & 0x00FF) | ((nai_dt2_read32(nRXAddress+4) & 0x00FF) << 8));
				
#ifdef _DF3_VERBOSE
				printf("Payload Length: 0x%x\n", nCmdPayloadLength);
#endif
				nRXAddress += 8;
#ifdef _DF3_VERBOSE
				printf("Detected Checksum low: 0x%x\n", (nai_dt2_read32(nRXAddress) & 0x00FF));
				printf("Detected Checksum high: 0x%x\n", (nai_dt2_read32(nRXAddress+4) & 0x00FF));
#endif			
				/* Only check CRC if passed in values are both not zero */
				if (ucCRC16Low != 0 && ucCRC16High != 0)
				{
					/* Make sure the Checksum values are what they are supposed to be! */
					if (((nai_dt2_read32(nRXAddress) & 0x00FF) != ucCRC16Low) || ((nai_dt2_read32(nRXAddress+4) & 0x00FF) != ucCRC16High))
					{
						nStatus = NAI_COMMAND_FAILED;
						break;
					}
					else /* If CRC is correct...we know we got the response we are looking for! */
					{
						ucResponseReceived = 1;
						*punStatusCode = 0; /*Success Code*/
#ifdef _DF3_VERBOSE
						printf("Expected checksum values were received!\n");
#endif											
						break;
					}					
				}	
#ifdef _DF3_VERBOSE
				printf("No Checksum Values were passed in to check..must interrogate packet!\n");
#endif					
										
				/* Now we are into the command payload */
				nRXAddress += 8;

				/* Make sure this is a Generic Response packet */
				if ((nai_dt2_read32(nRXAddress) & 0x00FF) == KL17_GENERIC_RESPONSE)
				{
#ifdef _DF3_VERBOSE
					printf("Correct response type of GENERIC was detected!\n");
#endif											
					nCmdPayloadLength--;
					if (nCmdPayloadLength > 0)
					{
						/* Read the Flags */
						nRXAddress += 4;
						nai_dt2_read32(nRXAddress); /*Don't care about flags*/
						nCmdPayloadLength--;

						/* Read the Reserved */
						if (nCmdPayloadLength > 0)
						{
							nRXAddress += 4;
							nai_dt2_read32(nRXAddress); /*Don't care about reserved*/
							nCmdPayloadLength--;

							/* Read the Param Count */
							if (nCmdPayloadLength > 0)
							{
								nRXAddress += 4;
								ucParamCount = (uint8_t)(nai_dt2_read32(nRXAddress) & 0x00FF);
								nCmdPayloadLength--;

								/* Each parameter is 32 bits (4 bytes) in length - we should have 2 parameters to read! */
								if (ucParamCount == 2 && nCmdPayloadLength == (ucParamCount * 4))
								{
#ifdef _DF3_VERBOSE
									printf("1st Param: Reading status code!\n");
#endif																					
									/* 1st parameter is status code */
									nRXAddress += 4;
									*punStatusCode = (uint8_t)((nai_dt2_read32(nRXAddress) & 0x00FF) | ((nai_dt2_read32(nRXAddress+4) & 0x00FF) << 8) | ((nai_dt2_read32(nRXAddress+8) & 0x00FF) << 16) | ((nai_dt2_read32(nRXAddress+12) & 0x00FF) << 24));

#ifdef _DF3_VERBOSE
									printf("2nd Param: Reading Command Tag!\n");
#endif																
									nRXAddress += 16;
									/* 2nd parameter is Command Tag...which should equal the passed in command code */
									ucResponseCmd = (uint8_t)(nai_dt2_read32(nRXAddress) & 0x00FF);

									/* NOTE: We don't care about last 3 bytes of response command as response command should only be 1 byte! */
									if (ucResponseCmd != ucCmd)
									{
#ifdef _DF3_VERBOSE
										printf("Command Tag did NOT match expected Command!\n");
#endif																											
										nStatus = NAI_RESPONSE_COMMAND_MISMATCH;
										break;
									}
									ucResponseReceived = 1;
								}
								else /* Unexpected param count or paylaod length */
								{
#ifdef _DF3_VERBOSE
									printf("Invalid Payload detected - 1!\n");
#endif																																	
									nStatus = NAI_INVALID_PAYLOAD_LENGTH;
								}
							}
							else /* Invalid payload */
							{
#ifdef _DF3_VERBOSE
								printf("Invalid Payload detected - 2!\n");
#endif																												
								nStatus = NAI_INVALID_PAYLOAD_LENGTH;
							}
						}
						else /* Invalid payload */
						{
#ifdef _DF3_VERBOSE
							printf("Invalid Payload detected - 3!\n");
#endif																											
							nStatus = NAI_INVALID_PAYLOAD_LENGTH;
						}
					}
					else /* Invalid payload */
					{
#ifdef _DF3_VERBOSE
						printf("Invalid Payload detected - 4!\n");
#endif																										
						nStatus = NAI_INVALID_PAYLOAD_LENGTH;
					}
				} /* Packet type is not "RESPONSE PACKET" */
				else
				{
#ifdef _DF3_VERBOSE
					printf("Invalid Packet Type detected!\n");
#endif																									
					nStatus = NAI_INVALID_PACKET_TYPE;
				}
				
				break;				
			}			
			/* Increment to the next address of the RX buffer */
			nRXAddress += 4;				
		}		
	}

	if (nStatus == NAI_SUCCESS)
		if (*punStatusCode != 0)
			nStatus = NAI_COMMAND_FAILED;

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_kl17_get_property_response is responsible for returning a property response from a KL17 command.
</summary>
<param name="punStatusCode"> : Output: Status state of get property command made to the KL17 microcontroller</param>
<param name="ucNumProperties"> : Input:  Number of properties expected to get results for</param>
<param name="pucPropertyValues"> : Output: Actual property values </param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_kl17_get_property_response(uint32_t *punStatusCode, uint8_t ucNumProperties, uint8_t *pucPropertyValues)
{
	int32_t nStatus = NAI_SUCCESS;
	int32_t ulTimer = 0;
	int32_t nRXAddress = RX_RAM_START;
	int32_t nCmdPayloadLength = 0;
	uint8_t ucParamCount = 0;
	uint8_t i = 0;
	uint8_t index = 0;
	uint8_t ucKeepTrying = 1;

#ifdef _DF3_VERBOSE							
	printf("In nai_kl17_get_property_response\n");
#endif		

	ulTimer = nai_get_timer(0);
	while (ucKeepTrying)
	{
		if (nai_get_timer(ulTimer) > DT_COMPLETION_TIMEOUT)
		{
			nStatus = NAI_STM_RX_TIMEOUT;
			printf("RX Timed Out - RX RAM Data[0] = %x\n", (unsigned int)nai_dt2_read32(RX_RAM_START));
			break;
		}

		/* We are expecting the 1st byte to be the START_BYTE (0x5A) followed by a 1 byte Packet Type Cmd (0xA4)*/
		if (((nai_dt2_read32(nRXAddress) & 0x00FF) == KL17_START_BYTE) &&
			((nai_dt2_read32(nRXAddress+4) & 0x00FF) == KL17_PACKET_TYPE_COMMAND))
		{
			ucKeepTrying = 0;
			nRXAddress += 8;
			nCmdPayloadLength = ((nai_dt2_read32(nRXAddress) & 0x00FF) | ((nai_dt2_read32(nRXAddress+4) & 0x00FF) << 8));
			nRXAddress += 8;

			/* Let's skip past CRC - not really interested in checking CRC at this point */
			nRXAddress +=8;

			/* Make sure this is a Get Property Response packet */
			if ((nai_dt2_read32(nRXAddress) & 0x00FF) == KL17_GET_PROPERTY_RESPONSE)
			{
				nCmdPayloadLength--;
				if (nCmdPayloadLength > 0)
				{
					/* Read the Flags */
					nRXAddress += 4;
					nai_dt2_read32(nRXAddress); /*Don't care about flags*/
					nCmdPayloadLength--;

					/* Read the Reserved */
					if (nCmdPayloadLength > 0)
					{	
						nRXAddress += 4;
						nai_dt2_read32(nRXAddress); /*Don't care about reserved*/
						nCmdPayloadLength--;

						/* Read the Param Count */
						if (nCmdPayloadLength > 0)
						{
							nRXAddress += 4;
							ucParamCount = (uint8_t)(nai_dt2_read32(nRXAddress) & 0x00FF);
#ifdef _DF3_VERBOSE								
							printf("Param Count %d\n", (int32_t)ucParamCount);
#endif							
							nCmdPayloadLength--;

							/* Each parameter is 32 bits (4 bytes) in length - we should have 3 parameters to read! */
							if (ucParamCount >= 2) /* && nCmdPayloadLength == (ucParamCount * 4))*/
							{
								/* 1st parameter/property is status code */
								nRXAddress += 4;
								*punStatusCode = (uint8_t)((nai_dt2_read32(nRXAddress) & 0x00FF) | ((nai_dt2_read32(nRXAddress+4) & 0x00FF) << 8) | ((nai_dt2_read32(nRXAddress+8) & 0x00FF) << 16) | ((nai_dt2_read32(nRXAddress+12) & 0x00FF) << 24));
#ifdef _DF3_VERBOSE									
								printf("Status = %d\n", (int32_t)*punStatusCode);
#endif								
								nRXAddress += 16;

								/* Rest of properties will be 32 bit values */
								for (i=0; i < ucNumProperties; i++)
								{   
									index = i * 4;
									pucPropertyValues[index] = (nai_dt2_read32(nRXAddress) & 0x00FF);
									pucPropertyValues[index+1] = (nai_dt2_read32(nRXAddress+4) & 0x00FF);
									pucPropertyValues[index+2] = (nai_dt2_read32(nRXAddress+8) & 0x00FF);
									pucPropertyValues[index+3] = (nai_dt2_read32(nRXAddress+12) & 0x00FF);
									nRXAddress += 16;
								}
							}
							else /* Unexpected param count or paylaod length */
							{
								nStatus = NAI_INVALID_PAYLOAD_LENGTH;
							}
						}
						else /* Invalid payload */
						{
							nStatus = NAI_INVALID_PAYLOAD_LENGTH;
						}
					}
					else /* Invalid payload */
					{
						nStatus = NAI_INVALID_PAYLOAD_LENGTH;
					}
				}
				else /* Invalid payload */
				{
					nStatus = NAI_INVALID_PAYLOAD_LENGTH;
				}
			} /* Packet type is not "RESPONSE PACKET" */
			else
			{
				nStatus = NAI_INVALID_PACKET_TYPE;
			}
			break;
		}
		
		nRXAddress += 4;
	}


	if (nStatus == NAI_SUCCESS)
		if (*punStatusCode != 0)
			nStatus = NAI_COMMAND_FAILED;

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_get_isolated_dt is responsible for retrieving version number of ST microcontroller bootloader as 
well as supported functions. This command only works on a single channel. 
</summary>
<param name="ptMsgPacketList"> : Input/Output:Serdes request as well as packet list of data to be returned
containing requested information </param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_get_isolated_dt(MsgPacketList *ptMsgPacketList)
{
	int32_t nStatus = NAI_SUCCESS;
	uint32_t unControlValue = 0;
	uint8_t ucChanRequest = 0;
	uint8_t ucChanIndex = 0;	
	uint8_t ucNewCompleterID = 0;
	uint8_t ucNewRequesterID = 0;	
	MsgPacket *ptMsgPacket = NULL;
	WORDValue tWordValue;
	uint8_t k = 0;
	uint32_t unLoopCnt = 0;
	
	uint8_t i = 0;
	uint8_t ucData[MAX_ISOLATED_DT_READ_IN_BYTES];		
	uint8_t ucDataLength = MAX_ISOLATED_DT_READ_IN_BYTES;
	memset(&ucData[0], 0, ucDataLength);

#ifdef _DF3_VERBOSE	
	printf("In nai_get_isolated_dt\n");
#endif
		
	if (ptMsgPacketList != NULL && ptMsgPacketList->ptStart != NULL)
	{
		ptMsgPacket = ptMsgPacketList->ptStart;
		
		if (ptMsgPacket != NULL)
		{
			/* Fetch Channel */
			ucChanRequest = ((uint8_t)ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usChipID);
			if (ucChanRequest > NUM_DT_CHANNELS)
				nStatus = NAI_INVALID_PARAMETER_VALUE;					
				
			if (nStatus == NAI_SUCCESS)
			{
				k = 0;
				unLoopCnt = 0;
				ucChanIndex = (ucChanRequest - 1);		
							
				if (!g_bAlreadyInBootloader[ucChanIndex])		
					nStatus = nai_put_isolated_dt_in_config_mode(ucChanIndex);
				
				if (nStatus == NAI_SUCCESS)
				{
					/* Clear TX and RX RAM */
					unControlValue = nai_dt2_read32(SERIAL_CONTROL_REG);
					nai_dt2_write32(SERIAL_CONTROL_REG, (unControlValue | DT2_CTRL_RESET_TX_RAM | DT2_CTRL_RESET_RX_RAM));

					nStatus = nai_get_version_and_supported_functions_isolated_dt(ucChanIndex, &ucData[0], &ucDataLength);

					if (nStatus == NAI_SUCCESS)
					{
						/* Now we re-use the request packet as our return packet ... NOTE: we could create a brand-new packet if that makes more sense....time will tell*/
						/* SERDES HEADER */
						ucNewCompleterID = (ptMsgPacket->tNAIMsg.tSerdesHdr.ucRequesterID);
						ucNewRequesterID = (ptMsgPacket->tNAIMsg.tSerdesHdr.ucCompleterID);

						ptMsgPacket->tNAIMsg.tSerdesHdr.ucType = 3;
						ptMsgPacket->tNAIMsg.tSerdesHdr.ucToHPS = 1; /*NOT SURE IF THIS SHOULD BE SET FOR READ!*/
						ptMsgPacket->tNAIMsg.tSerdesHdr.ucPayloadLength = 0; /* PAYLOAD LENGTH - TO BE FILLED IN BELOW! */
						ptMsgPacket->tNAIMsg.tSerdesHdr.usSERDES2 = 0;
						ptMsgPacket->tNAIMsg.tSerdesHdr.usSERDES3 = 0;
						ptMsgPacket->tNAIMsg.tSerdesHdr.ucRequesterID = ucNewRequesterID;
						ptMsgPacket->tNAIMsg.tSerdesHdr.ucCompleterID = ucNewCompleterID;
						ptMsgPacket->tNAIMsg.tSerdesHdr.usSERDES5 = 0;

						/* TRANSPORT HEADER */
						ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usID = get_next_tran_id();
						ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.unMsgLength = (convert_bytes_to_words(ucDataLength) + (1 * CONFIG_TOTAL_PKT_HDR_IN_WORDS)); /* Length in Words of entire */
						ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usSequenceNum = 0;  /* Seq Number - TO BE FILLED IN BELOW! */
						ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usExpectedSequenceCount = 1; /*Expected Sequence Count */

						/* COMMAND HEADER */
						ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usCommandType = COMMAND_TYPECODE_GET_MICRO;

						/* KEEP THESE FIELDS THE SAME! */
						/* ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unOffset */
						/* ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.unPayLdRequestLength */

						/* COMMAND PAYLOAD */
						/* ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength = 0; */ /*Packet payload length stored in bytes */

						unLoopCnt = convert_bytes_to_words((uint32_t)ucDataLength);
						for (i=0; i < unLoopCnt; i++)
						{
							tWordValue.ucLoByte = ucData[k++];
							tWordValue.ucHiByte = ucData[k++];
							ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandPayLd.usData[i] = tWordValue.usValue;
						}

						/* Store how many bytes are in this packet's payload */
						ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportHdr.usPacketPayLdLength = (uint16_t)(ucDataLength);
						ptMsgPacket->tNAIMsg.tSerdesHdr.ucPayloadLength = (uint8_t)((unLoopCnt+CONFIG_TOTAL_PKT_HDR_IN_WORDS) - TOTAL_SERDES_READWRITEREQ_HDR_IN_WORDS);

						/* Serdes Payload length must be a multiple of 2 */
						if ((ptMsgPacket->tNAIMsg.tSerdesHdr.ucPayloadLength % 2) != 0)
							ptMsgPacket->tNAIMsg.tSerdesHdr.ucPayloadLength++;

						/* Now send the message */
#ifdef _DF3_VERBOSE
						printf("Sending GET information back to MB\n");
#endif
						nStatus = nai_send_msg(ptMsgPacketList);
#ifdef _DF3_VERBOSE
						printf("Finished Sending GET information back to MB = %d\n", nStatus);
#endif						
#ifdef _DEBUG_X		
						printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
						for (i=0; i < sizeof(ucData); i++)
						{
							printf("ucData[%d] = 0x%2x\n", i, ucData[i]);
						}
						printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
#endif
					}
				}
				else
					printf("Failed putting isolated DT into config mode - Status = %d\n", (int)nStatus);
			}
		}
	}
	
	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup IsoloatedDT
<summary>
nai_erase_isolated_dt is responsible for erasing the entire microcontroller bootloader
This command works on a single channel or on all channels if value 255 is detected as the "ChipID". 
</summary>
<param name="ptMsgPacketList"> : Input: Serdes request to erase all</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t nai_erase_isolated_dt(MsgPacketList *ptMsgPacketList)
{
	int32_t nStatus = NAI_SUCCESS;
	uint8_t ucNumChannels = NUM_DT_CHANNELS;
	uint8_t ucChanRequest = 0;	
	uint8_t ucChanIndex = 0;	
	MsgPacket *ptMsgPacket = NULL;	
		
#ifdef _DF3_VERBOSE							
	printf("In nai_erase_isolated_dt\n");
#endif		

	if (ptMsgPacketList != NULL && ptMsgPacketList->ptStart != NULL)
	{
		ptMsgPacket = ptMsgPacketList->ptStart;
		
		if (ptMsgPacket != NULL)
		{
			/* Fetch Channel */
			ucChanRequest = ((uint8_t)ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usChipID);
			if (ucChanRequest > NUM_DT_CHANNELS && ucChanRequest != ALL_CHANNELS)
				nStatus = NAI_INVALID_PARAMETER_VALUE;					
				
			if (nStatus == NAI_SUCCESS)
			{
				if (ucChanRequest != ALL_CHANNELS)
				{
					ucChanIndex = (ucChanRequest - 1);
					ucNumChannels = (ucChanIndex + 1);
				}
				
				for (; ucChanIndex < ucNumChannels; ucChanIndex++)
				{
					printf("\n*************************************************************************************\n");
					printf("Erasing Microcontroller found at channel %u...\n", ucChanIndex+1);
					if (!g_bAlreadyInBootloader[ucChanIndex])
						nStatus = nai_put_isolated_dt_in_config_mode(ucChanIndex);
					
					if (nStatus == NAI_SUCCESS)
						nStatus = nai_perform_erase_isolated_dt(ucChanIndex);						
					
					if (nStatus == NAI_SUCCESS)
						printf("Successfully erased Microcontroller!\n");
					else
						printf("Failed to erase Microcontroller! Status = %d\n", (int)nStatus);
					printf("*************************************************************************************\n");
				}
			}
		}
	}
	return nStatus;
}
#endif /* _MICROCONTROLLER */

static uint32_t nai_get_completion_timeout(MsgPacket *ptMsgPacket)
{
	/* Default timeout */
	uint32_t unCompletionTimeout = (COMPLETION_TIMEOUT * 4);

#ifdef _MICROCONTROLLER
	uint8_t ucNumChannels = 1;
#endif
	
	if (ptMsgPacket != NULL)
	{	    
		switch (ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usCommandType)
		{
#ifdef _MICROCONTROLLER
			case COMMAND_TYPECODE_CONFIG_MICRO :
			case COMMAND_TYPECODE_ERASE_MICRO :
			if (ptMsgPacket->tNAIMsg.tSerdesPayLd.tTransportPayLd.tCommandHdr.usChipID == (uint32_t)ALL_CHANNELS)
				ucNumChannels = NUM_DT_CHANNELS;			
			else
				ucNumChannels = 1;
			unCompletionTimeout *= (ucNumChannels + 2);  /* Multiply the standard timeout by the number of channels we will be processing + 2 (buffer) */
			break;
#endif
			default :			
			break;
		}
	}
	
	return unCompletionTimeout;
}
#endif /* __UBOOT */
