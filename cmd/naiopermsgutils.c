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

#include "cmd_naiopermsgutils.h"
#include "cmd_naimsgutils.h"

#if defined(__BAREMETAL) || defined(__LINUX) || defined (__CYGWIN)
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#endif

#ifdef __UBOOT
#include <common.h>
#endif

#ifdef __LINUX
#include <unistd.h>
#include <pthread.h>
#include "socal/socal.h"
#include "socal/hps.h"

	#ifdef _USE_MUTEX
	#define MAX_AVAIL_SLOTS 6

	/* Keep an array of mutexes - one for each slot..this way we only prevent access when different threads are trying to access same module */
	static pthread_mutex_t	g_SERDES_Mutex[MAX_AVAIL_SLOTS+1]; //Plus 1 for MB
	#endif
#endif

/* Global Slot ID...used to store Requester ID - defined in cmd_naimsgutils.h */
extern uint8_t g_ucSlotID;
extern uint8_t g_ucMsgUtilsInitialized;

static int32_t nai_send_serdes_oper_msg(NAIOperMsg *ptNAIOperMsg);
static int32_t nai_receive_serdes_oper_msg(NAIOperMsg *ptNAIOperMsg);

#ifdef _USE_MUTEX
static int32_t initSERDES_Mutex(uint8_t ucSlotID);
static int32_t lockSERDES_Mutex(uint8_t ucSlotID);
static int32_t unlockSERDES_Mutex(uint8_t ucSlotID);
#endif


/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_init_msg_utils is responsible for performing any necessary steps to make sure communication can take place
between mb and slots.
</summary>
<param name="ucID"> : (Input)ID of caller (used to differentiate whether caller is PPC, ARM, INTEL etc..).</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_init_as_slot">
*/
/**************************************************************************************************************/
int32_t nai_init_msg_utils(uint8_t ucID)
{
	int32_t nStatus = nai_init_as_slot(ucID);

	if (nStatus == NAI_SUCCESS)
	{
#ifdef _USE_MUTEX
		uint8_t i = 0;

		/* Initialize all of the mutexes */
		for (i=0; i <= MAX_AVAIL_SLOTS; i++)
			initSERDES_Mutex(i);
#endif

		nStatus = nai_perform_init_slot_addressing();
	}

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_init_slot_addressing is responsible for initializing all module slot addresses.
</summary>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
int32_t nai_init_slot_addressing()
{
	int32_t nStatus = nai_perform_init_slot_addressing();
	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_read_reg16_request is responsible for making the SERDES request to retrieve a single 16 bit register value.
</summary>
<param name="unAddress"> : (Input) Register address to read.</param>
<param name="pusValue"> : (Output) Register 16 Bit value to be returned to caller.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_get_module_id_and_address">
<seealso cref="nai_read_reg16_by_slot_request">
*/
/**************************************************************************************************************/
int32_t nai_read_reg16_request(uint32_t unAddress, uint16_t *pusValue)
{
	int32_t nStatus = NAI_SUCCESS;
	uint8_t ucCompleterID = 0;	
	uint32_t unModuleOffset = 0;
	
#ifdef _VERBOSE
	printf("**************nai_read_reg16_request**************\r\n");
#endif

	if (!g_ucMsgUtilsInitialized)
		return NAI_SYSTEM_NOT_READY;

	nStatus = nai_get_module_id_and_address(unAddress, &ucCompleterID, &unModuleOffset);

	if (nStatus == NAI_SUCCESS)
		nStatus = nai_read_reg16_by_slot_request(ucCompleterID, unModuleOffset, pusValue);

#ifdef _VERBOSE
	printf("**************END nai_read_reg16_request**************\r\n");
#endif

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_write_reg16_request is responsible for making the SERDES request to write a specified 16 bit value to the 
specified register address.
</summary>
<param name="unAddress"> : (Input) Register address to write to.</param>
<param name="usValue"> : (Output) Register 16 Bit value to be assigned.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_get_module_id_and_address">
<seealso cref="nai_write_reg16_by_slot_request">
*/
/**************************************************************************************************************/
int32_t nai_write_reg16_request(uint32_t unAddress, uint16_t usValue)
{
	int32_t nStatus = NAI_SUCCESS;
	uint8_t ucCompleterID = 0;	
	uint32_t unModuleOffset = 0;

#ifdef _VERBOSE
	printf("**************nai_write_reg16_request**************\r\n");
#endif

	if (!g_ucMsgUtilsInitialized)
		return NAI_SYSTEM_NOT_READY;

	nStatus = nai_get_module_id_and_address(unAddress, &ucCompleterID, &unModuleOffset);

	if (nStatus == NAI_SUCCESS)
		nStatus = nai_write_reg16_by_slot_request(ucCompleterID, unModuleOffset, usValue);		
	
	#ifdef _VERBOSE
		printf("**************END nai_write_reg16_request**************\r\n");
	#endif

	return nStatus;
}


/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_read_reg32_request is responsible for making the SERDES request to retrieve a single 32 bit register value.
</summary>
<param name="unAddress"> : (Input) Register address to read.</param>
<param name="punValue"> : (Output) Register 32 Bit value to be returned to caller.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_get_module_id_and_address">
<seealso cref="nai_read_reg32_by_slot_request">
*/
/**************************************************************************************************************/
int32_t nai_read_reg32_request(uint32_t unAddress, uint32_t *punValue)
{
	int32_t nStatus = NAI_SUCCESS;
	uint8_t ucCompleterID = 0;	
	uint32_t unModuleOffset = 0;
	
#ifdef _VERBOSE
	printf("**************nai_read_reg32_request**************\r\n");
#endif

	if (!g_ucMsgUtilsInitialized)
		return NAI_SYSTEM_NOT_READY;

    NAIOperMsg tNAIOperMsg;
	memset(&tNAIOperMsg.msg, 0, sizeof(tNAIOperMsg.msg));

	nStatus = nai_get_module_id_and_address(unAddress, &ucCompleterID, &unModuleOffset);

	if (nStatus == NAI_SUCCESS)
		nStatus = nai_read_reg32_by_slot_request(ucCompleterID, unModuleOffset, punValue);

#ifdef _VERBOSE
	printf("**************END nai_read_reg32_request**************\r\n");
#endif

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_write_reg32_request is responsible for making the SERDES request to write a specified 32 bit value to the 
specified register address.
</summary>
<param name="unAddress"> : (Input) Register address to write to.</param>
<param name="unValue"> : (Output) Register 32 Bit value to be assigned.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_get_module_id_and_address">
<seealso cref="nai_write_reg32_by_slot_request">
*/
/**************************************************************************************************************/
int32_t nai_write_reg32_request(uint32_t unAddress, uint32_t unValue)
{
	int32_t nStatus = NAI_SUCCESS;
	uint8_t ucCompleterID = 0;	
	uint32_t unModuleOffset = 0;

#ifdef _VERBOSE
	printf("**************nai_write_reg32_request**************\r\n");
#endif

	if (!g_ucMsgUtilsInitialized)
		return NAI_SYSTEM_NOT_READY;

	nStatus = nai_get_module_id_and_address(unAddress, &ucCompleterID, &unModuleOffset);

	if (nStatus == NAI_SUCCESS)
		nStatus = nai_write_reg32_by_slot_request(ucCompleterID, unModuleOffset, unValue);

	#ifdef _VERBOSE
		printf("**************END nai_write_reg32_request**************\r\n");
	#endif

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_read_block16_request is responsible for reading a block of registers - not necessarily contiguous.
</summary>
<param name="unStartAddress"> : (Input) Register address at which to start writing to.</param>
<param name="usCount"> : (Input) Number of registers (and values) to write.</param>
<param name="usStride"> : (Input)Governs how many bytes of address should be skipped between memory access.</param>
<param name="pusDataBuf"> : (Output) 16 Bit Data values read.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_get_module_id_and_address">
<seealso cref="nai_read_block16_by_slot_request">
*/
/**************************************************************************************************************/
int32_t nai_read_block16_request(uint32_t unStartAddress, uint32_t usCount, uint8_t ucStride, uint16_t *pusDataBuf)
{
	int32_t nStatus = NAI_SUCCESS;
	uint8_t ucCompleterID = 0;	
	uint32_t unModuleOffset = 0;

	#ifdef _VERBOSE
		printf("**************nai_read_block16_request**************\r\n");
	#endif
 
	if (!g_ucMsgUtilsInitialized)
		return NAI_SYSTEM_NOT_READY;

	nStatus = nai_get_module_id_and_address(unStartAddress, &ucCompleterID, &unModuleOffset);

	if (nStatus == NAI_SUCCESS)
		nStatus = nai_read_block16_by_slot_request(ucCompleterID, unModuleOffset, usCount, ucStride, pusDataBuf);

	#ifdef _VERBOSE
		printf("**************END nai_read_block16_request**************\r\n");
	#endif

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_write_block16_request is responsible for writing to a block of registers - not necessarily contiguous.
</summary>
<param name="unStartAddress"> : (Input) Register address at which to start writing to.</param>
<param name="usCount"> : (Input) Number of registers (and values) to write.</param>
<param name="usStride"> : (Input)Governs how many bytes of address should be skipped between memory writes.</param>
<param name="pusDataBuf"> : (Input) 16 Bit Data values to be written.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_get_module_id_and_address">
<seealso cref="nai_write_block16_by_slot_request">
*/
/**************************************************************************************************************/
int32_t nai_write_block16_request(uint32_t unStartAddress, uint32_t usCount, uint8_t ucStride, uint16_t *pusDataBuf)
{
	int32_t nStatus = NAI_SUCCESS;	
	uint8_t ucCompleterID = 0;	
	uint32_t unModuleOffset = 0;

#ifdef _VERBOSE
	printf("**************nai_write_block16_request**************\r\n");
#endif
	
	if (!g_ucMsgUtilsInitialized)
		return NAI_SYSTEM_NOT_READY;

	nStatus = nai_get_module_id_and_address(unStartAddress, &ucCompleterID, &unModuleOffset);
	
	if (nStatus == NAI_SUCCESS)
		nStatus = nai_write_block16_by_slot_request(ucCompleterID, unModuleOffset, usCount, ucStride, pusDataBuf);
	
#ifdef _VERBOSE
	printf("**************END nai_write_block16_request**************\r\n");
#endif
	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_read_block32_request is responsible for reading a block of registers - not necessarily contiguous.
</summary>
<param name="unStartAddress"> : (Input) Register address at which to start writing to.</param>
<param name="usCount"> : (Input) Number of registers (and values) to write.</param>
<param name="usStride"> : (Input)Governs how many bytes of address should be skipped between memory access.</param>
<param name="punDataBuf"> : (Output) Data values read.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_get_module_id_and_address">
<seealso cref="nai_read_block32_by_slot_request">
*/
/**************************************************************************************************************/
int32_t nai_read_block32_request(uint32_t unStartAddress, uint32_t usCount, uint8_t ucStride, uint32_t *punDataBuf)
{
	int32_t nStatus = NAI_SUCCESS;
	uint8_t ucCompleterID = 0;	
	uint32_t unModuleOffset = 0;

	#ifdef _VERBOSE
		printf("**************nai_read_block32_request**************\r\n");
	#endif
 
	if (!g_ucMsgUtilsInitialized)
		return NAI_SYSTEM_NOT_READY;

	nStatus = nai_get_module_id_and_address(unStartAddress, &ucCompleterID, &unModuleOffset);

	if (nStatus == NAI_SUCCESS)
		nStatus = nai_read_block32_by_slot_request(ucCompleterID, unModuleOffset, usCount, ucStride, punDataBuf);

	#ifdef _VERBOSE
		printf("**************END nai_read_block32_request**************\r\n");
	#endif

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_write_block32_request is responsible for writing to a block of registers - not necessarily contiguous.
</summary>
<param name="unStartAddress"> : (Input) Register address at which to start writing to.</param>
<param name="usCount"> : (Input) Number of registers (and values) to write.</param>
<param name="usStride"> : (Input)Governs how many bytes of address should be skipped between memory access.</param>
<param name="punDataBuf"> : (Input) Data values to be written.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_get_module_id_and_address">
<seealso cref="nai_write_block32_by_slot_request">
*/
/**************************************************************************************************************/
int32_t nai_write_block32_request(uint32_t unStartAddress, uint32_t usCount, uint8_t ucStride, uint32_t *punDataBuf)
{
	int32_t nStatus = NAI_SUCCESS;	
	uint8_t ucCompleterID = 0;	
	uint32_t unModuleOffset = 0;

#ifdef _VERBOSE
	printf("**************nai_write_block32_request**************\r\n");
#endif
	
	if (!g_ucMsgUtilsInitialized)
		return NAI_SYSTEM_NOT_READY;

	nStatus = nai_get_module_id_and_address(unStartAddress, &ucCompleterID, &unModuleOffset);

	if (nStatus == NAI_SUCCESS)
		nStatus = nai_write_block32_by_slot_request(ucCompleterID, unModuleOffset, usCount, ucStride, punDataBuf);

#ifdef _VERBOSE
	printf("**************END nai_write_block32_request**************\r\n");
#endif
	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_send_serdes_oper_msg is responsible for sending a single operational message via the SERDES bus.
</summary>
<param name="ptNAIOperMsg"> : (Input) Operational Message to be placed on the SERDES bus.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_tx_fifo_empty">
<seealso cref="nai_get_tx_fifo_address">
<seealso cref="nai_get_tx_fifo_pkt_ready_address">
*/
/**************************************************************************************************************/
static int32_t nai_send_serdes_oper_msg(NAIOperMsg *ptNAIOperMsg)
{
	int32_t nStatus = NAI_SUCCESS;	
	int32_t i = 0;
	volatile uint32_t unAddr = 0;
	volatile uint32_t unBeginTxAddr = 0; 
	int32_t nNumLoops = 0; 
	FIFOValue tFIFOVal;
	uint32_t ulTimer;

#ifdef _DEBUG_X
	uint32_t unTemp = 0;
#endif

#ifdef _VERBOSE
	printf("**************nai_send_serdes_oper_msg**************\r\n");
#endif

	if (ptNAIOperMsg == NULL)
		return NAI_INVALID_PARAMETER_VALUE;

	/* We have a message to send...but the FIFO is not ready...need to wait!*/	
	ulTimer = nai_get_timer(0);
	while (!nai_tx_fifo_empty(ptNAIOperMsg->tSerdesHdr.ucRequesterID, ptNAIOperMsg->tSerdesHdr.ucCompleterID))
	{
		if (nai_get_timer(ulTimer) > COMPLETION_TIMEOUT)
		{
			nStatus = NAI_TX_FIFO_NOT_EMPTY_TIMEOUT;
			break;
		}
	}
	
	if (nStatus == NAI_SUCCESS)
	{
		unAddr = nai_get_tx_fifo_address(ptNAIOperMsg->tSerdesHdr.ucRequesterID, ptNAIOperMsg->tSerdesHdr.ucCompleterID);
		unBeginTxAddr = nai_get_tx_fifo_pkt_ready_address(ptNAIOperMsg->tSerdesHdr.ucRequesterID, ptNAIOperMsg->tSerdesHdr.ucCompleterID);

		/*NOTE: Receive packet requests indicate they have payload, but they really do not...the payload length indicates how much we expect to get back! */
		/*      Keith has it such that the LSB of ucType indicates whether "REAL" payload is present.  1=Payload, 0=No Payload */
		if ((ptNAIOperMsg->tSerdesHdr.ucType & 0x01) == 0x01)
			nNumLoops = ((TOTAL_SERDES_READWRITEREQ_HDR_IN_WORDS + (ptNAIOperMsg->tSerdesHdr.ucPayloadLength))); /* TOTAL_SERDES_READWRITEREQ_HDR_IN_WORDS = 6 Words + Payload Length) */
		else
			nNumLoops = TOTAL_SERDES_READWRITEREQ_HDR_IN_WORDS;

	#ifdef _VERBOSE
		printf("Num Words for FIFO    : %ld\r\n", nNumLoops);
	#endif

		for (i=0; i <nNumLoops; i++)
		{		
			tFIFOVal.usLoWord = ptNAIOperMsg->msg[i++];
			tFIFOVal.usHiWord = ptNAIOperMsg->msg[i];

	#ifdef _DEBUG_X
			printf("FIFO VAL = 0x%8x\r\n", tFIFOVal.unValue);
	#endif
			nai_write32(unAddr, tFIFOVal.unValue);
		}

		/* Now force transfer of data now that the FIFO is filled with current message*/
		nai_write32(unBeginTxAddr, (uint32_t)1);
	}

#ifdef _VERBOSE
	printf("**************END nai_send_serdes_oper_msg**************\r\n");
#endif
	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_receive_serdes_oper_msg is responsible for receiving a single operational message from the SERDES bus.
</summary>
<param name="ptNAIOperMsg"> : (Input) Operational Message read from the FIFO.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_rx_fifo_clear_pkt_ready">
<seealso cref="nai_get_rx_fifo_address">
*/
/**************************************************************************************************************/
static int32_t nai_receive_serdes_oper_msg(NAIOperMsg *ptNAIOperMsg)
{
	int32_t nStatus = NAI_SUCCESS;
	
	volatile uint32_t unAddr = 0;
	FIFOValue tFIFOVal;
	uint32_t unWordsRead = 0;	
	uint16_t  usSerdesHdrWordsRead = 0;
	int32_t i = 0;
	uint16_t usPacketLength = 0;

#ifdef _VERBOSE
	printf("**************nai_receive_serdes_oper_msg**************\r\n");
#endif

	if (ptNAIOperMsg == NULL)
		return NAI_INVALID_PARAMETER_VALUE;

	nai_rx_fifo_clear_pkt_ready(ptNAIOperMsg->tSerdesHdr.ucRequesterID, ptNAIOperMsg->tSerdesHdr.ucCompleterID);

	unAddr = nai_get_rx_fifo_address(ptNAIOperMsg->tSerdesHdr.ucCompleterID);

	/* First we read just the SERDES Header info as we should be guaranteed this is present */
    /* From there, we can determine how many words are part of this SERDES packet */
	for (i=0; i < TOTAL_SERDES_READWRITEREQ_HDR_IN_WORDS; i++)
	{		
		tFIFOVal.unValue = nai_read32(unAddr);

#ifdef _DEBUG_X
		printf("Read SERDES HDR FIFO Value: 0x%8x\r\n", tFIFOVal.unValue);
#endif
		ptNAIOperMsg->msg[i++] 	= tFIFOVal.usLoWord;
		ptNAIOperMsg->msg[i] 	= tFIFOVal.usHiWord;
		usSerdesHdrWordsRead += 2;
	}
		
	/* Now we read enough to know how many words are part of this SERDES packet...so let's read the desired amount off of the FIFO */
	usPacketLength = (ptNAIOperMsg->tSerdesHdr.ucPayloadLength); /* Payload Length for SERDES are bits 0 - 7 */
	while (unWordsRead < usPacketLength)
	{
		tFIFOVal.unValue = nai_read32(unAddr);

#ifdef _DEBUG_X
		printf("Read FIFO Value: 0x%8x\r\n", tFIFOVal.unValue);		
#endif

		ptNAIOperMsg->msg[i++] = tFIFOVal.usLoWord;
		unWordsRead++;
		if (unWordsRead < usPacketLength)
		{
			ptNAIOperMsg->msg[i++] = tFIFOVal.usHiWord;
			unWordsRead++;
		}
	}

#ifdef _VERBOSE
	printf("**************END nai_receive_serdes_oper_msg**************\r\n");
#endif
	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_read_reg16_by_slot_request is responsible for making the SERDES request to retrieve a single 16 bit register value.
</summary>
<param name="ucSlotID"> : (Input) Slot ID of module to read from.</param>
<param name="unModuleOffset"> : (Input) Offset into module address space of where to read.</param>
<param name="pusValue"> : (Output) Register 16 Bit value to be returned to caller.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_send_serdes_oper_msg">
<seealso cref="nai_receive_serdes_oper_msg">
*/
/**************************************************************************************************************/
int32_t nai_read_reg16_by_slot_request(uint8_t ucSlotID, uint32_t unModuleOffset, uint16_t *pusValue)
{
#ifdef _USE_MUTEX
	if (lockSERDES_Mutex(ucSlotID) != NAI_SUCCESS)
		return NAI_UNABLE_TO_LOCK_MUTEX;
#endif

	int32_t nStatus = NAI_SUCCESS;
	FIFOValue tFIFOVal;
	uint8_t ucCompleterID = ucSlotID;	
	uint8_t ucRequesterID = g_ucSlotID;
	uint32_t ulTimer = 0;

#ifdef _VERBOSE
	printf("**************nai_read_reg16_by_slot_request**************\r\n");
#endif
/*DANT - We do not need to do this check for "by_slot_request" since no address info is required
	if (!g_ucMsgUtilsInitialized)
		return NAI_SYSTEM_NOT_READY;
*/
    NAIOperMsg tNAIOperMsg;
	memset(&tNAIOperMsg.msg, 0, sizeof(tNAIOperMsg.msg));

	/* SERDES HEADER */
#ifdef _VERBOSE
	printf("RequesterID = 0x%2.2x(%u)\r\n", ucRequesterID, ucRequesterID);
	printf("CompleterID = 0x%2.2x(%u)\r\n", ucCompleterID, ucCompleterID);
#endif
				
	tNAIOperMsg.tSerdesHdr.ucType = SERDES_READREG;
	tNAIOperMsg.tSerdesHdr.ucToHPS = 0;

	if ((unModuleOffset & 0x0003) == 0)
		tNAIOperMsg.tSerdesHdr.ucByteEnable = 0x3;
	else if ((unModuleOffset & 0x0003) == 2)
		tNAIOperMsg.tSerdesHdr.ucByteEnable = 0xC;
	else
		nStatus = NAI_MIS_ALIGNED_BYTE_ENABLE;

	if (nStatus == NAI_SUCCESS)
	{
		tNAIOperMsg.tSerdesHdr.ucPayloadLength = (uint8_t)(2); /* No Payload for making the request...but we must tell Keith how much data we are expecting! */
		tNAIOperMsg.tSerdesHdr.ucBlockAddrIncrVal = 0; /* Make sure we have a zero "stride" for a single register read */

		tFIFOVal.unValue = unModuleOffset;
		tNAIOperMsg.tSerdesHdr.usAddressLo = tFIFOVal.usLoWord;
		tNAIOperMsg.tSerdesHdr.usAddressHi = tFIFOVal.usHiWord;	

		tNAIOperMsg.tSerdesHdr.ucRequesterID = ucRequesterID;
		tNAIOperMsg.tSerdesHdr.ucCompleterID = ucCompleterID;		

		/* Send request for data */
		nStatus = nai_send_serdes_oper_msg(&tNAIOperMsg);

		if (nStatus == NAI_SUCCESS)
		{
			/* ** NOW WAIT FOR RESPONSE ** */

			/* Prepare for response! */
			memset(&tNAIOperMsg.msg, 0, sizeof(tNAIOperMsg.msg));

			/* Wait for packet! */
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
		
			if (nStatus == NAI_SUCCESS)
			{
				tNAIOperMsg.tSerdesHdr.ucRequesterID = ucRequesterID;
				tNAIOperMsg.tSerdesHdr.ucCompleterID = ucCompleterID;

				/* Now receive the packet */
				nStatus = nai_receive_serdes_oper_msg(&tNAIOperMsg);

				if (tNAIOperMsg.tSerdesHdr.ucPayloadLength >= 2) /* One 32 bit value = 2 Words (4 Bytes) but we are only interested in the lo word */
				{
					/* Depending on ByteEnable...we need to either read from lower order or upper order bits */
					if (tNAIOperMsg.tSerdesHdr.ucByteEnable == 0x3)					
						*pusValue = tNAIOperMsg.tSerdesPayLd.usData[0];
					else
						*pusValue = tNAIOperMsg.tSerdesPayLd.usData[1];
				}
				else
					nStatus = NAI_SERDES_UNEXPECTED_PAYLOAD_COUNT;
			}
		}
	}

#ifdef _VERBOSE
	printf("**************END nai_read_reg16_by_slot_request**************\r\n");
#endif

#ifdef _USE_MUTEX
	if (unlockSERDES_Mutex(ucSlotID) != NAI_SUCCESS)
	{
		if (nStatus == NAI_SUCCESS)
			nStatus = NAI_UNABLE_TO_UNLOCK_MUTEX;
	}
#endif

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_write_reg16_by_slot_request is responsible for making the SERDES request to write a specified 16 bit value to the 
specified register address.
</summary>
<param name="ucSlotID"> : (Input) Slot ID of module to write to.</param>
<param name="unModuleOffset"> : (Input) Offset into module address space of where to write.</param>
<param name="usValue"> : (Output) Register 16 Bit value to be assigned.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_send_serdes_oper_msg">
*/
/**************************************************************************************************************/
int32_t nai_write_reg16_by_slot_request(uint8_t ucSlotID, uint32_t unModuleOffset, uint16_t usValue)
{
#ifdef _USE_MUTEX
	if (lockSERDES_Mutex(ucSlotID) != NAI_SUCCESS)
		return NAI_UNABLE_TO_LOCK_MUTEX;
#endif

	int32_t nStatus = NAI_SUCCESS;
	FIFOValue tFIFOVal;
	uint8_t ucCompleterID = ucSlotID;	
	uint8_t ucRequesterID = g_ucSlotID;

#ifdef _VERBOSE
	printf("**************nai_write_reg16_by_slot_request**************\r\n");
#endif

/*DANT - We do not need to do this check for "by_slot_request" since no address info is required
	if (!g_ucMsgUtilsInitialized)
		return NAI_SYSTEM_NOT_READY;
*/
    NAIOperMsg tNAIOperMsg;
	memset(&tNAIOperMsg.msg, 0, sizeof(tNAIOperMsg.msg));

	/* NOTE: We initialized the entire tNAIOperMsg to zero above..so here we just flesh out those items that need to be set for this write command */
	tNAIOperMsg.tSerdesHdr.ucType = SERDES_WRITEREG;
    tNAIOperMsg.tSerdesHdr.ucToHPS = 0;

	if ((unModuleOffset & 0x0003) == 0)
		tNAIOperMsg.tSerdesHdr.ucByteEnable = 0x3;
	else if ((unModuleOffset & 0x0003) == 2)
		tNAIOperMsg.tSerdesHdr.ucByteEnable = 0xC;
	else 
		nStatus = NAI_MIS_ALIGNED_BYTE_ENABLE;

	if (nStatus == NAI_SUCCESS)
	{
		tNAIOperMsg.tSerdesHdr.ucPayloadLength = 2; /* 2 Words (32 bit value) (Keith expects 32 bit value for now)*/
		tNAIOperMsg.tSerdesHdr.ucBlockAddrIncrVal = 0; /* Make sure we have a zero "stride" for a sing register write */

		tFIFOVal.unValue = unModuleOffset;
		tNAIOperMsg.tSerdesHdr.usAddressLo = tFIFOVal.usLoWord;
		tNAIOperMsg.tSerdesHdr.usAddressHi = tFIFOVal.usHiWord;	

		tNAIOperMsg.tSerdesHdr.ucRequesterID = ucRequesterID;
		tNAIOperMsg.tSerdesHdr.ucCompleterID = ucCompleterID;

		/* Assign Payload */
		/* Depending on ByteEnable...we need to either write lower order or upper order bits */
		if (tNAIOperMsg.tSerdesHdr.ucByteEnable == 0x3)
		{		
			tNAIOperMsg.tSerdesPayLd.usData[0] = usValue;
			tNAIOperMsg.tSerdesPayLd.usData[1] = 0;
		}
		else
		{
			tNAIOperMsg.tSerdesPayLd.usData[0] = 0;
			tNAIOperMsg.tSerdesPayLd.usData[1] = usValue;
		}

		nStatus = nai_send_serdes_oper_msg(&tNAIOperMsg);
	}
		
	#ifdef _VERBOSE
		printf("**************END nai_write_reg16_by_slot_request**************\r\n");
	#endif

#ifdef _USE_MUTEX
	if (unlockSERDES_Mutex(ucSlotID) != NAI_SUCCESS)
	{
		if (nStatus == NAI_SUCCESS)
			nStatus = NAI_UNABLE_TO_UNLOCK_MUTEX;
	}
#endif

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_read_reg32_by_slot_request is responsible for making the SERDES request to retrieve a single 32 bit register value.
</summary>
<param name="ucSlotID"> : (Input) Slot ID of module to read from.</param>
<param name="unModuleOffset"> : (Input) Offset into module address space of where to read.</param>
<param name="punValue"> : (Output) Register 32 Bit value to be returned to caller.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_send_serdes_oper_msg">
<seealso cref="nai_receive_serdes_oper_msg">
*/
/**************************************************************************************************************/
int32_t nai_read_reg32_by_slot_request(uint8_t ucSlotID, uint32_t unModuleOffset, uint32_t *punValue)
{
#ifdef _USE_MUTEX
	if (lockSERDES_Mutex(ucSlotID) != NAI_SUCCESS)
		return NAI_UNABLE_TO_LOCK_MUTEX;
#endif

	int32_t nStatus = NAI_SUCCESS;
	FIFOValue tFIFOVal;
	uint8_t ucCompleterID = ucSlotID;	
	uint8_t ucRequesterID = g_ucSlotID;
	uint32_t ulTimer = 0;

#ifdef _VERBOSE
	printf("**************nai_read_reg32_by_slot_request**************\r\n");
#endif

/*DANT - We do not need to do this check for "by_slot_request" since no address info is required
	if (!g_ucMsgUtilsInitialized)
		return NAI_SYSTEM_NOT_READY;
*/
    NAIOperMsg tNAIOperMsg;
	memset(&tNAIOperMsg.msg, 0, sizeof(tNAIOperMsg.msg));

	/* SERDES HEADER */
#ifdef _VERBOSE
	printf("RequesterID = 0x%2.2x(%u)\r\n", ucRequesterID, ucRequesterID);
	printf("CompleterID = 0x%2.2x(%u)\r\n", ucCompleterID, ucCompleterID);
#endif
	tNAIOperMsg.tSerdesHdr.ucType = SERDES_READREG;
	tNAIOperMsg.tSerdesHdr.ucToHPS = 0;
	tNAIOperMsg.tSerdesHdr.ucByteEnable = SERDES_32BITDATA;
	tNAIOperMsg.tSerdesHdr.ucPayloadLength = (uint8_t)(2); /* No Payload for making the request...but we must tell Keith how much data we are expecting! */
	tNAIOperMsg.tSerdesHdr.ucBlockAddrIncrVal = 0; /* Make sure we have a zero "stride" for a single register read */

	tFIFOVal.unValue = unModuleOffset;
	tNAIOperMsg.tSerdesHdr.usAddressLo = tFIFOVal.usLoWord;
	tNAIOperMsg.tSerdesHdr.usAddressHi = tFIFOVal.usHiWord;	

	tNAIOperMsg.tSerdesHdr.ucRequesterID = ucRequesterID;
	tNAIOperMsg.tSerdesHdr.ucCompleterID = ucCompleterID;		

	/* Send request for data */
	nStatus = nai_send_serdes_oper_msg(&tNAIOperMsg);

	if (nStatus == NAI_SUCCESS)
	{
		/* ** NOW WAIT FOR RESPONSE ** */

		/* Prepare for response! */
		memset(&tNAIOperMsg.msg, 0, sizeof(tNAIOperMsg.msg));

		/* Wait for packet! */
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

		if (nStatus == NAI_SUCCESS)
		{
			tNAIOperMsg.tSerdesHdr.ucRequesterID = ucRequesterID;
			tNAIOperMsg.tSerdesHdr.ucCompleterID = ucCompleterID;

			/* Now receive the packet */
			nStatus = nai_receive_serdes_oper_msg(&tNAIOperMsg);

			if (tNAIOperMsg.tSerdesHdr.ucPayloadLength >= 2) /* One 32 bit value = 2 Words (4 Bytes) */
			{
				tFIFOVal.usLoWord = tNAIOperMsg.tSerdesPayLd.usData[0];
				tFIFOVal.usHiWord = tNAIOperMsg.tSerdesPayLd.usData[1];
				*punValue = (uint32_t)tFIFOVal.unValue;		
			}
			else
				nStatus = NAI_SERDES_UNEXPECTED_PAYLOAD_COUNT;
		}
	}

#ifdef _VERBOSE
	printf("**************END nai_read_reg32_by_slot_request**************\r\n");
#endif

#ifdef _USE_MUTEX
	if (unlockSERDES_Mutex(ucSlotID) != NAI_SUCCESS)
	{
		if (nStatus == NAI_SUCCESS)
			nStatus = NAI_UNABLE_TO_UNLOCK_MUTEX;
	}
#endif

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_write_reg32_by_slot_request is responsible for making the SERDES request to write a specified 32 bit value to the 
specified register address.
</summary>
<param name="ucSlotID"> : (Input) Slot ID of module to write to.</param>
<param name="unModuleOffset"> : (Input) Offset into module address space of where to write.</param>
<param name="unValue"> : (Output) Register 32 Bit value to be assigned.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_send_serdes_oper_msg">
*/
/**************************************************************************************************************/
int32_t nai_write_reg32_by_slot_request(uint8_t ucSlotID, uint32_t unModuleOffset, uint32_t unValue)
{
#ifdef _USE_MUTEX
	if (lockSERDES_Mutex(ucSlotID) != NAI_SUCCESS)
		return NAI_UNABLE_TO_LOCK_MUTEX;
#endif

	int32_t nStatus = NAI_SUCCESS;
	FIFOValue tFIFOVal;
	uint8_t ucCompleterID = ucSlotID;	
	uint8_t ucRequesterID = g_ucSlotID;

#ifdef _VERBOSE
	printf("**************nai_write_reg32_by_slot_request**************\r\n");
#endif

/*DANT - We do not need to do this check for "by_slot_request" since no address info is required
	if (!g_ucMsgUtilsInitialized)
		return NAI_SYSTEM_NOT_READY;
*/

    NAIOperMsg tNAIOperMsg;
	memset(&tNAIOperMsg.msg, 0, sizeof(tNAIOperMsg.msg));

	/* NOTE: We initialized the entire tNAIOperMsg to zero above..so here we just flesh out those items that need to be set for this write command */
	tNAIOperMsg.tSerdesHdr.ucType = SERDES_WRITEREG;
    tNAIOperMsg.tSerdesHdr.ucToHPS = 0;
	tNAIOperMsg.tSerdesHdr.ucByteEnable = SERDES_32BITDATA;
	tNAIOperMsg.tSerdesHdr.ucPayloadLength = 2; /* 2 Words (32 bit value) */
	tNAIOperMsg.tSerdesHdr.ucBlockAddrIncrVal = 0; /* Make sure we have a zero "stride" for a sing register write */

	tFIFOVal.unValue = unModuleOffset;
	tNAIOperMsg.tSerdesHdr.usAddressLo = tFIFOVal.usLoWord;
	tNAIOperMsg.tSerdesHdr.usAddressHi = tFIFOVal.usHiWord;	

	tNAIOperMsg.tSerdesHdr.ucRequesterID = ucRequesterID;
	tNAIOperMsg.tSerdesHdr.ucCompleterID = ucCompleterID;

	/* Assign Payload */
	tFIFOVal.unValue = (uint32_t)unValue;
	tNAIOperMsg.tSerdesPayLd.usData[0] = tFIFOVal.usLoWord;
	tNAIOperMsg.tSerdesPayLd.usData[1] = tFIFOVal.usHiWord;

	nStatus = nai_send_serdes_oper_msg(&tNAIOperMsg);
	
	#ifdef _VERBOSE
		printf("**************END nai_write_reg32_by_slot_request**************\r\n");
	#endif

#ifdef _USE_MUTEX
	if (unlockSERDES_Mutex(ucSlotID) != NAI_SUCCESS)
	{
		if (nStatus == NAI_SUCCESS)
			nStatus = NAI_UNABLE_TO_UNLOCK_MUTEX;
	}
#endif

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_read_block16_by_slot_request is responsible for reading a block of registers.
NOTE: Current Functionality for Strides of 0, 2, 4, 8 and 12 only.
</summary>
<param name="ucSlotID"> : (Input) Slot ID of module to read from.</param>
<param name="unModuleOffsetStart"> : (Input) Offset into module address space of where to start reading.</param>
<param name="usCount"> : (Input) Number of registers (and values) to write.</param>
<param name="usStride"> : (Input)Governs how many bytes of address should be skipped between memory access.</param>
<param name="pusDataBuf"> : (Output) 16 Bit Data values read.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_send_serdes_oper_msg">
<seealso cref="nai_receive_serdes_oper_msg">
*/
/**************************************************************************************************************/
int32_t nai_read_block16_by_slot_request(uint8_t ucSlotID, uint32_t unModuleOffsetStart, uint32_t usCount, uint8_t ucStride, uint16_t *pusDataBuf)
{
#ifdef _USE_MUTEX
	if (lockSERDES_Mutex(ucSlotID) != NAI_SUCCESS)
		return NAI_UNABLE_TO_LOCK_MUTEX;
#endif
	
	int32_t nStatus = NAI_SUCCESS;
	int32_t nPayloadLeftToRead = (int32_t)(usCount); 
	uint16_t i;
	uint8_t ucCompleterID = ucSlotID;	
	uint8_t ucRequesterID = g_ucSlotID;
	uint32_t ulTimer = 0;	
	uint16_t usChunkIndex = 0;	
		
	FIFOValue tFIFOVal;
	uint16_t usDataIndex = 0;

	NAIOperMsg tNAIOperMsg;
	memset(&tNAIOperMsg.msg, 0, sizeof(tNAIOperMsg.msg));
			
	#ifdef _VERBOSE
		printf("**************nai_read_block16_by_slot_request**************\r\n");
	#endif
	
	/* Must be 32 bit aligned */
	if ((unModuleOffsetStart & 0x0003) != 0)
		return NAI_MIS_ALIGNED_BYTE_ENABLE;
				
/*DANT - We do not need to do this check for "by_slot_request" since no address info is required
	if (!g_ucMsgUtilsInitialized)
		return NAI_SYSTEM_NOT_READY;
*/

    /* Make sure Stide is a multiple of 4 for 32 bit alignment */
    if ((ucStride % 4) != 0)
		return NAI_STRIDE_CAUSES_MISALIGNMENT;

	/* SERDES HEADER */
#ifdef _VERBOSE
	printf("RequesterID = 0x%2.2x(%u)\r\n", ucRequesterID, ucRequesterID);
	printf("CompleterID = 0x%2.2x(%u)\r\n", ucCompleterID, ucCompleterID);
#endif
		
	while (nPayloadLeftToRead > 0 && nStatus == NAI_SUCCESS)
	{
		memset(&tNAIOperMsg.msg, 0, sizeof(tNAIOperMsg.msg));
		
		/* NOTE: We expect data to be "packed" */
		tNAIOperMsg.tSerdesHdr.ucDataMode = 1;
		tNAIOperMsg.tSerdesHdr.ucByteEnable = SERDES_32BITDATA;
						
		tNAIOperMsg.tSerdesHdr.ucType = SERDES_READREG;
		tNAIOperMsg.tSerdesHdr.ucToHPS = 0;
		tNAIOperMsg.tSerdesHdr.ucPayloadLength = (uint8_t)(MIN(nPayloadLeftToRead, OPER_MAX_PAYLOAD_IN_WORDS)); /* No Payload for making the request!...but we must tell Keith how much data we are expecting! */
		tNAIOperMsg.tSerdesHdr.ucBlockAddrIncrVal = (ucStride/4); 

		tNAIOperMsg.tSerdesHdr.ucRequesterID = ucRequesterID;
		tNAIOperMsg.tSerdesHdr.ucCompleterID = ucCompleterID;		

		if (ucStride == 0)
			tFIFOVal.unValue = unModuleOffsetStart;
		else
			tFIFOVal.unValue = (unModuleOffsetStart + (usChunkIndex * OPER_MAX_PAYLOAD_IN_WORDS * ucStride));

		tNAIOperMsg.tSerdesHdr.usAddressLo = tFIFOVal.usLoWord;
		tNAIOperMsg.tSerdesHdr.usAddressHi = tFIFOVal.usHiWord;	

		/* Send request for data */
		nStatus = nai_send_serdes_oper_msg(&tNAIOperMsg);

		if (nStatus == NAI_SUCCESS)
		{
			/* Prepare for response! */
			memset(&tNAIOperMsg.msg, 0, sizeof(tNAIOperMsg.msg));

			/* Wait for packet! */
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

			if (nStatus != NAI_SUCCESS)
				break;
				
			tNAIOperMsg.tSerdesHdr.ucPayloadLength = (uint8_t)(MIN(nPayloadLeftToRead, OPER_MAX_PAYLOAD_IN_WORDS)); /* We must tell Keith how much data we are expecting! */
			tNAIOperMsg.tSerdesHdr.ucRequesterID = ucRequesterID;
			tNAIOperMsg.tSerdesHdr.ucCompleterID = ucCompleterID;

			/* Now receive the packet */
			nStatus = nai_receive_serdes_oper_msg(&tNAIOperMsg);
		
			if (nStatus == NAI_SUCCESS)
			{
				for (i=0; i < tNAIOperMsg.tSerdesHdr.ucPayloadLength; i++)
				{
#ifdef _VERBOSE					
					printf("Received Data - usData[%d] = 0x%x\n", i, tNAIOperMsg.tSerdesPayLd.usData[i]);
#endif
					pusDataBuf[usDataIndex++] = tNAIOperMsg.tSerdesPayLd.usData[i];
				}
					
				nPayloadLeftToRead -= tNAIOperMsg.tSerdesHdr.ucPayloadLength;
			}
			
			usChunkIndex++;
		}
	}

#ifdef _VERBOSE
	printf("**************END nai_read_block16_by_slot_request**************\r\n");
#endif

#ifdef _USE_MUTEX
	if (unlockSERDES_Mutex(ucSlotID) != NAI_SUCCESS)
	{
		if (nStatus == NAI_SUCCESS)
			nStatus = NAI_UNABLE_TO_UNLOCK_MUTEX;
	}
#endif

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_write_block16_by_slot_request is responsible for writing to a block of registers - not necessarily contiguous.
NOTE: Currently hardware only supports a stride of 0 for a true "block" write.  Any other stride will regress to
      single writes to be made. (Eventually hardware will be modified to rectify this shortcomming.
</summary>
<param name="ucSlotID"> : (Input) Slot ID of module to write to.</param>
<param name="unModuleOffsetStart"> : (Input) Offset into module address space of where to start writing.</param>
<param name="usCount"> : (Input) Number of registers (and values) to write.</param>
<param name="usStride"> : (Input)Governs how many bytes of address should be skipped between memory writes.</param>
<param name="pusDataBuf"> : (Input) 16 Bit Data values to be written.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_send_serdes_oper_msg">
*/
/**************************************************************************************************************/
int32_t nai_write_block16_by_slot_request(uint8_t ucSlotID, uint32_t unModuleOffsetStart, uint32_t usCount, uint8_t ucStride, uint16_t *pusDataBuf)
{
#ifdef _USE_MUTEX
	if (lockSERDES_Mutex(ucSlotID) != NAI_SUCCESS)
		return NAI_UNABLE_TO_LOCK_MUTEX;
#endif

	int32_t nStatus = NAI_SUCCESS;	
	uint16_t i;
	FIFOValue tFIFOVal;
	int32_t nPayloadWordsLeftToWrite = (int32_t)usCount; 
	uint16_t usDataIndex = 0;
	uint8_t ucCompleterID = ucSlotID;	
	uint8_t ucRequesterID = g_ucSlotID;
	uint16_t usChunkIndex = 0;
	
#ifdef _VERBOSE
	printf("**************nai_write_block16_by_slot_request**************\r\n");
#endif
	
	/* Must be 32 bit aligned */
	if ((unModuleOffsetStart & 0x0003) != 0)
		return NAI_MIS_ALIGNED_BYTE_ENABLE;
			
/*DANT - We do not need to do this check for "by_slot_request" since no address info is required
	if (!g_ucMsgUtilsInitialized)
		return NAI_SYSTEM_NOT_READY;
*/

    /* Make sure Stide is a multiple of 4 - 32 bit addressing */
    if ((ucStride % 4) != 0)
		return NAI_STRIDE_CAUSES_MISALIGNMENT;
		
	while (nStatus == NAI_SUCCESS && nPayloadWordsLeftToWrite > 0)
	{
		NAIOperMsg tNAIOperMsg;
		memset(&tNAIOperMsg.msg, 0, sizeof(tNAIOperMsg.msg));

		tNAIOperMsg.tSerdesHdr.ucType = SERDES_WRITEREG;
		tNAIOperMsg.tSerdesHdr.ucToHPS = 0;
		tNAIOperMsg.tSerdesHdr.ucPayloadLength = (uint8_t)MIN(nPayloadWordsLeftToWrite, OPER_MAX_PAYLOAD_IN_WORDS);
		tNAIOperMsg.tSerdesHdr.ucBlockAddrIncrVal = (ucStride/4);
		
		if (ucStride == 0)
			tFIFOVal.unValue = unModuleOffsetStart;
		else
			tFIFOVal.unValue = (unModuleOffsetStart + (usChunkIndex * OPER_MAX_PAYLOAD_IN_WORDS * ucStride));
		
		tNAIOperMsg.tSerdesHdr.usAddressLo = tFIFOVal.usLoWord;
		tNAIOperMsg.tSerdesHdr.usAddressHi = tFIFOVal.usHiWord;	
	
		tNAIOperMsg.tSerdesHdr.ucRequesterID = ucRequesterID;
		tNAIOperMsg.tSerdesHdr.ucCompleterID = ucCompleterID;
		
		/* Use Packing! */
		tNAIOperMsg.tSerdesHdr.ucDataMode = 1;
		tNAIOperMsg.tSerdesHdr.ucByteEnable = SERDES_32BITDATA; 

		/* Assign Payload */
		for (i=0; i < tNAIOperMsg.tSerdesHdr.ucPayloadLength; i++)
		{
			printf("Assigned Payload: usDataBuf[%d] = 0x%x\n", i, pusDataBuf[usDataIndex]);
			tNAIOperMsg.tSerdesPayLd.usData[i] = pusDataBuf[usDataIndex++];
		}

		nStatus = nai_send_serdes_oper_msg(&tNAIOperMsg);

		nPayloadWordsLeftToWrite -= tNAIOperMsg.tSerdesHdr.ucPayloadLength;
		usChunkIndex++;		
	}
	
#ifdef _VERBOSE
	printf("**************END nai_write_block16_by_slot_request**************\r\n");
#endif

#ifdef _USE_MUTEX
	if (unlockSERDES_Mutex(ucSlotID) != NAI_SUCCESS)
	{
		if (nStatus == NAI_SUCCESS)
			nStatus = NAI_UNABLE_TO_UNLOCK_MUTEX;
	}
#endif

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_read_block32_by_slot_request is responsible for reading a block of registers - not necessarily contiguous.
</summary>
<param name="ucSlotID"> : (Input) Slot ID of module to read from.</param>
<param name="unModuleOffsetStart"> : (Input) Offset into module address space of where to start reading.</param>
<param name="usCount"> : (Input) Number of registers (and values) to write.</param>
<param name="usStride"> : (Input)Governs how many bytes of address should be skipped between memory access.</param>
<param name="punDataBuf"> : (Output) Data values read.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_send_serdes_oper_msg">
<seealso cref="nai_receive_serdes_oper_msg">
*/
/**************************************************************************************************************/
int32_t nai_read_block32_by_slot_request(uint8_t ucSlotID, uint32_t unModuleOffsetStart, uint32_t usCount, uint8_t ucStride, uint32_t *punDataBuf)
{
#ifdef _USE_MUTEX
	if (lockSERDES_Mutex(ucSlotID) != NAI_SUCCESS)
		return NAI_UNABLE_TO_LOCK_MUTEX;
#endif

	int32_t nStatus = NAI_SUCCESS;
	int32_t nPayloadLeftToRead = (int32_t)(usCount * 2); /*Number of Words (not 32 bit values)!*/
	uint16_t i;
	uint8_t ucCompleterID = ucSlotID;	
	uint8_t ucRequesterID = g_ucSlotID;
	uint32_t ulTimer = 0;
	uint16_t usChunkIndex = 0;

	#ifdef _VERBOSE
		printf("**************nai_read_block32_by_slot_request**************\r\n");
	#endif
 
    /* SERDES uses 32 bit addressing so make sure Stide is a multiple of 4 */
    if ((ucStride % 4) != 0)
		return NAI_STRIDE_CAUSES_MISALIGNMENT;
		 
/*DANT - We do not need to do this check for "by_slot_request" since no address info is required
	if (!g_ucMsgUtilsInitialized)
		return NAI_SYSTEM_NOT_READY;
*/
	FIFOValue tFIFOVal;
	uint16_t usDataIndex = 0;

	NAIOperMsg tNAIOperMsg;
	
	/* SERDES HEADER */
#ifdef _VERBOSE
	printf("RequesterID = 0x%2.2x(%u)\r\n", ucRequesterID, ucRequesterID);
	printf("CompleterID = 0x%2.2x(%u)\r\n", ucCompleterID, ucCompleterID);
#endif
	
	while (nPayloadLeftToRead > 0 && nStatus == NAI_SUCCESS)
	{
		memset(&tNAIOperMsg.msg, 0, sizeof(tNAIOperMsg.msg));
		tNAIOperMsg.tSerdesHdr.ucType = SERDES_READREG;
		tNAIOperMsg.tSerdesHdr.ucToHPS = 0;
		tNAIOperMsg.tSerdesHdr.ucByteEnable = SERDES_32BITDATA;
		tNAIOperMsg.tSerdesHdr.ucBlockAddrIncrVal = (ucStride/4); 

		tNAIOperMsg.tSerdesHdr.ucRequesterID = ucRequesterID;
		tNAIOperMsg.tSerdesHdr.ucCompleterID = ucCompleterID;			
		tNAIOperMsg.tSerdesHdr.ucPayloadLength = (uint8_t)(MIN(nPayloadLeftToRead, OPER_MAX_PAYLOAD_IN_WORDS)); /* No Payload for making the request!...but we must tell Keith how much data we are expecting! */

		if (ucStride == 0)
			tFIFOVal.unValue = unModuleOffsetStart;
		else
			tFIFOVal.unValue = (unModuleOffsetStart + (usChunkIndex * (OPER_MAX_PAYLOAD_IN_WORDS/2) * ucStride));
			
		tNAIOperMsg.tSerdesHdr.usAddressLo = tFIFOVal.usLoWord;
		tNAIOperMsg.tSerdesHdr.usAddressHi = tFIFOVal.usHiWord;	

		/* Send request for data */
		nStatus = nai_send_serdes_oper_msg(&tNAIOperMsg);		
			
		if (nStatus == NAI_SUCCESS)
		{			
			/* Prepare for response! */
			memset(&tNAIOperMsg.msg, 0, sizeof(tNAIOperMsg.msg));

			/* Wait for packet! */
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

			if (nStatus != NAI_SUCCESS)
				break;

			tNAIOperMsg.tSerdesHdr.ucPayloadLength = (uint8_t)(MIN(nPayloadLeftToRead, OPER_MAX_PAYLOAD_IN_WORDS)); /* We must tell Keith how much data we are expecting! */
			tNAIOperMsg.tSerdesHdr.ucRequesterID = ucRequesterID;
			tNAIOperMsg.tSerdesHdr.ucCompleterID = ucCompleterID;

			/* Now receive the packet */
			nStatus = nai_receive_serdes_oper_msg(&tNAIOperMsg);
			
			if (nStatus == NAI_SUCCESS)
			{
				for (i=0; i < tNAIOperMsg.tSerdesHdr.ucPayloadLength; i++)
				{
					tFIFOVal.usLoWord = tNAIOperMsg.tSerdesPayLd.usData[i++];
					tFIFOVal.usHiWord = tNAIOperMsg.tSerdesPayLd.usData[i];
					punDataBuf[usDataIndex++] = (uint32_t)tFIFOVal.unValue;
				}

				nPayloadLeftToRead -= tNAIOperMsg.tSerdesHdr.ucPayloadLength;
			}
			
			usChunkIndex++;
		}
	}

#ifdef _VERBOSE
	printf("**************END nai_read_block32_by_slot_request**************\r\n");
#endif

#ifdef _USE_MUTEX
	if (unlockSERDES_Mutex(ucSlotID) != NAI_SUCCESS)
	{
		if (nStatus == NAI_SUCCESS)
			nStatus = NAI_UNABLE_TO_UNLOCK_MUTEX;
	}
#endif

	return nStatus;
}


/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
nai_write_block32_by_slot_request is responsible for writing to a block of registers - not necessarily contiguous.
</summary>
<param name="ucSlotID"> : (Input) Slot ID of module to write to.</param>
<param name="unModuleOffsetStart"> : (Input) Offset into module address space of where to start writing.</param>
<param name="usCount"> : (Input) Number of registers (and values) to write.</param>
<param name="usStride"> : (Input)Governs how many bytes of address should be skipped between memory writes.</param>
<param name="punDataBuf"> : (Input) Data values to be written.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="nai_send_serdes_oper_msg">
*/
/**************************************************************************************************************/
int32_t nai_write_block32_by_slot_request(uint8_t ucSlotID, uint32_t unModuleOffsetStart, uint32_t usCount, uint8_t ucStride, uint32_t *punDataBuf)
{
#ifdef _USE_MUTEX
	if (lockSERDES_Mutex(ucSlotID) != NAI_SUCCESS)
		return NAI_UNABLE_TO_LOCK_MUTEX;
#endif

	int32_t nStatus = NAI_SUCCESS;	
	uint16_t i;
	FIFOValue tFIFOVal;
	int32_t nPayloadWordsLeftToWrite = (int32_t)(usCount * 2); /*Number of Words (not 32 bit values)!*/
	uint16_t usDataIndex = 0;
	uint8_t ucCompleterID = ucSlotID;	
	uint8_t ucRequesterID = g_ucSlotID;
	uint16_t usChunkIndex = 0;
	
#ifdef _VERBOSE
	printf("**************nai_write_block32_by_slot_request**************\r\n");
#endif
	
    /* SERDES uses 32 bit addressing so make sure Stide is a multiple of 4 */
    if ((ucStride % 4) != 0)
		return NAI_STRIDE_CAUSES_MISALIGNMENT;
			
/*DANT - We do not need to do this check for "by_slot_request" since no address info is required
	if (!g_ucMsgUtilsInitialized)
		return NAI_SYSTEM_NOT_READY;
*/

	while (nStatus == NAI_SUCCESS && nPayloadWordsLeftToWrite > 0)
	{
		NAIOperMsg tNAIOperMsg;
		memset(&tNAIOperMsg.msg, 0, sizeof(tNAIOperMsg.msg));

		/* NOTE: We initialized the entire tNAIOperMsg to zero above..so here we just flesh out those items that need to be set for this write command */
		tNAIOperMsg.tSerdesHdr.ucType = SERDES_WRITEREG;
        tNAIOperMsg.tSerdesHdr.ucToHPS = 0;
		tNAIOperMsg.tSerdesHdr.ucByteEnable = SERDES_32BITDATA;
		tNAIOperMsg.tSerdesHdr.ucPayloadLength = (uint8_t)MIN(nPayloadWordsLeftToWrite, OPER_MAX_PAYLOAD_IN_WORDS);
		tNAIOperMsg.tSerdesHdr.ucBlockAddrIncrVal = (ucStride/4); 

		if (ucStride == 0)
			tFIFOVal.unValue = unModuleOffsetStart;
		else
			tFIFOVal.unValue = (unModuleOffsetStart + (usChunkIndex * (OPER_MAX_PAYLOAD_IN_WORDS/2) * ucStride));
			
		tNAIOperMsg.tSerdesHdr.usAddressLo = tFIFOVal.usLoWord;
		tNAIOperMsg.tSerdesHdr.usAddressHi = tFIFOVal.usHiWord;	
	
		tNAIOperMsg.tSerdesHdr.ucRequesterID = ucRequesterID;
		tNAIOperMsg.tSerdesHdr.ucCompleterID = ucCompleterID;

		/* Assign Payload */
		for (i=0; i < tNAIOperMsg.tSerdesHdr.ucPayloadLength; i++)
		{
			tFIFOVal.unValue = (uint32_t)punDataBuf[usDataIndex++];
			tNAIOperMsg.tSerdesPayLd.usData[i++] = tFIFOVal.usLoWord;
			tNAIOperMsg.tSerdesPayLd.usData[i] = tFIFOVal.usHiWord;
		}

		nStatus = nai_send_serdes_oper_msg(&tNAIOperMsg);

		nPayloadWordsLeftToWrite -= tNAIOperMsg.tSerdesHdr.ucPayloadLength;
		usChunkIndex++;
	}

#ifdef _VERBOSE
	printf("**************END nai_write_block32_by_slot_request**************\r\n");
#endif

#ifdef _USE_MUTEX
	if (unlockSERDES_Mutex(ucSlotID) != NAI_SUCCESS)
	{
		if (nStatus == NAI_SUCCESS)
			nStatus = NAI_UNABLE_TO_UNLOCK_MUTEX;
	}
#endif

	return nStatus;
}


/*****************************************************************************/
#ifdef _USE_MUTEX
/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
initSERDES_Mutex is responsible for initializing all of the slot based Mutexes used to defend against multiple
threads tring to access the same module simulataneously.
</summary>
<param name="ucSlotID"> : (Input) Slot ID of module to create Mutex for.</param>
<returns>int32_t : Status
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t initSERDES_Mutex(uint8_t ucSlotID)
{
	int32_t nStatus = NAI_SUCCESS;

#ifdef __LINUX
	if (ucSlotID >= 0 && ucSlotID <= MAX_AVAIL_SLOTS)
	{
		nStatus = pthread_mutex_init(&g_SERDES_Mutex[(ucSlotID)], NULL);
		if (nStatus != 0)
			printf("\r\nERROR cmd_naiopermsgutils.c - Unable to init mutex!");
	}
#endif

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
lockSERDES_Mutex is responsible for locking the specified slot's mutex.  A Timeout mutex lock is used such that
if we are unable to acquire a lock within a given timeout period..we give up trying and return an error.
</summary>
<param name="ucSlotID"> : (Input) Slot ID of module to acquire a Mutex lock.</param>
<returns>int32_t : Status
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t lockSERDES_Mutex(uint8_t ucSlotID)
{
	int32_t nStatus = NAI_SUCCESS;
#ifdef __LINUX
	if (ucSlotID >= 0 && ucSlotID <= MAX_AVAIL_SLOTS)
	{
		struct timespec abs_time;
		clock_gettime(CLOCK_REALTIME , &abs_time);
		abs_time.tv_sec += 4; //4 second timeout
		nStatus = pthread_mutex_timedlock( &g_SERDES_Mutex[(ucSlotID)],  &abs_time);
		if (nStatus != 0)
		{
			printf("\r\nERROR cmd_naiopermsgutils.c - Unable to obtain mutex Lock!");
			fflush(stdout);
		}
	}
#endif

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup SerdesMessageProcessing
<summary>
unlockSERDES_Mutex is responsible for unlocking the specified slot's mutex.
</summary>
<param name="ucSlotID"> : (Input) Slot ID of module of which to unlock the Mutex.</param>
<returns>int32_t : Status
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static int32_t unlockSERDES_Mutex(uint8_t ucSlotID)
{
	int32_t nStatus = NAI_SUCCESS;
#ifdef __LINUX
	if (ucSlotID >= 0 && ucSlotID <= MAX_AVAIL_SLOTS)
	{
		nStatus = pthread_mutex_unlock( &g_SERDES_Mutex[(ucSlotID)] );
#ifdef _VERBOSE
		if (nStatus != 0)
		{
			printf("\r\nERROR cmd_naiopermsgutils.c - Unable to obtain Unlock mutex!");
			fflush(stdout);
		}
#endif
	}
	return nStatus;
#endif
}
#endif

