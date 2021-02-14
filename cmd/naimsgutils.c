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
#include "cmd_naimsgutils.h"
#include "NAISerdes.h"

#ifdef __BAREMETAL
#include "alt_clock_manager.h"
#include "designware_i2c.h"
#endif

#if defined(__BAREMETAL) || defined(__LINUX) || defined (__CYGWIN) || defined (__VXWORKS)
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#elif __UBOOT
#include <common.h>
#include <console.h>
#include <malloc.h>
#include <crc.h>
#include <i2c.h>
#include <nai_mb_fpga_address.h>
#endif

#ifdef __LINUX
	#include <unistd.h>
	#include <sys/time.h>
#include "../FPGA_LWHPS2FPGA.h"
#endif

#ifdef __UBOOT
static int32_t MAX_ARG_LEN = 33;
extern int do_spi_flash(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
extern int do_spi_flash_erase(int argc, char * const argv[]);
extern int do_spi_flash_read_write(int argc, char * const argv[]);
#endif

#ifdef __LINUX
extern void* g_HPS2FPGA_VirtualBase;
extern void* g_LWHPS2FPGA_VirtualBase;
#endif

#ifdef __VXWORKS
extern uint32_t getNaiAddr();
#endif

#define THEORETICAL_MAX_SLOTS 10

uint16_t g_usMaxSlotCount = 0x0A;
uint8_t g_ucMsgUtilsInitialized = 0;
uint8_t ucHardCodedModuleSlot = 0;
uint8_t g_ucSlotID = INVALID_SLOT_ID; /* Global Slot ID...used to store Requester ID */
uint32_t g_unBaseAddress = 0x7FFFC000; /* Default to address for module until told otherwise */
uint32_t g_unModuleStartAddresses[THEORETICAL_MAX_SLOTS]; /* For now we create array of set size allowing room for growth (i.e. up to 10 module slots) */

static BOOL nai_is_mb(uint8_t ucSlotID);

static BOOL nai_is_mb(uint8_t ucSlotID)
{
	if ((ucSlotID == MB_SLOT) ||
		(ucSlotID == PPC_MB_SLOT))
		return TRUE;

	return FALSE;
}

#if defined(__UBOOT) || defined(__BAREMETAL)
static uint32_t getNaiModuleCommonBaseAddr(void);
static uint32_t getNaiModuleDT2CtrlBaseAddr(void);
static uint32_t getNaiModuleDT2CalBaseAddr(void);
uint32_t getNaiAddr()
{
	uint32_t unAddress = PS2FPGA_MODULE_BASE_ADDRESS;
	
	/* If dealing with module...we still use heavy weight bus (For Now) */
	if (!nai_is_mb(g_ucSlotID))
		unAddress = 0x7FFFC000;

	return unAddress;
}

static uint32_t getNaiModuleCommonBaseAddr()
{
	return 0x40000000;
}

static uint32_t getNaiModuleDT2CtrlBaseAddr()
{
	return 0x60000000;
}

static uint32_t getNaiModuleDT2CalBaseAddr()
{
	return 0x68000000;
}
#endif


void nai_assign_hard_coded_module_slot(uint8_t ucSlotID)
{
	ucHardCodedModuleSlot = ucSlotID;	
}

#ifdef __UBOOT
/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
nai_reset_module is responsible for resetting the module CPU.
</summary>
<returns>VOID
</returns>
*/
/**************************************************************************************************************/
void nai_reset_module()
{
	do_reset(NULL, 0, 0, NULL);
}
#endif

/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
nai_perform_init_slot_addressing is responsible for initializing all module slot addresses.
</summary>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
int32_t nai_perform_init_slot_addressing()
{
	int32_t nStatus = NAI_SUCCESS;

	if (ucHardCodedModuleSlot == 0)
	{
		int32_t i = 0;
		int32_t nAddr = COMMON_MBINFO_MODULESTART_ADDR;

		if(g_ucMsgUtilsInitialized == 1)
			return nStatus;

		/* Check to see if system is ready */
		if ((uint32_t)nai_read32(COMMON_MBINFO_READY_ADDR) != 0xA5A5A5A5)
			return NAI_SYSTEM_NOT_READY;

		for (i=0; i < THEORETICAL_MAX_SLOTS; i++)
		{
			/* First initialize the entry to all F's */
			g_unModuleStartAddresses[i] = 0xFFFFFFFF;
					
			/* DANT - This will need to be revisited - Mark has 7 entries but I leave room for 10 */
			if (i <= 6)
			{
				g_unModuleStartAddresses[i] = (uint32_t)nai_read32(nAddr);
				nAddr+= 0x00000004; /*Increment to next 32 bit address location*/
			}
		}
	}

	if (nStatus == NAI_SUCCESS)
		g_ucMsgUtilsInitialized = 1;

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup MessageUtils
<summary>
nai_get_timer is responsible for returning the the elapsed number of milliseconds.
</summary>
<param name="unTimer"> : (Input) Chip to be written to.</param>
<returns>uint32_t - numberof milliseconds elapsed since the passed in parameter value</returns>
*/
/**************************************************************************************************************/
uint32_t nai_get_timer(uint32_t unTimer)
{
	uint32_t unElapsedTime = 0;

#if defined(__UBOOT) || defined(__BAREMETAL)
	unElapsedTime = get_timer(unTimer);	
#elif defined(__LINUX)
	struct timeval start;
	gettimeofday(&start, NULL);
	unElapsedTime = ((start.tv_sec * 1000) - unTimer);
#elif defined(__VXWORKS)
	unElapsedTime = (((tickGet()/sysClkRateGet()) * 1000) - unTimer);
#endif	

	return unElapsedTime;
}

/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
nai_retreive_module_slots_status_request is responsible for retrieving a bitmapped value indicating which module
slots have modules.
</summary>
<param name="pusSlotIDs"> : (Output) Output: Bitmapped where each bit 0 - 5 is set hi if a module was found to be 
present in the slot associated with the bitmap location. Bit 0 = Slot 1, Bit 1 = Slot 2 ... Bit 5 = Slot 6</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
<seealso cref="make_read_request">
*/
/**************************************************************************************************************/
int32_t nai_retreive_module_slots_status_request(uint16_t *pusSlotIDs)
{
	int32_t nStatus = NAI_SUCCESS;	
	volatile uint32_t unModuleDetectedStatus = 0x00000100;
	FIFOValue tFIFOVal;
	uint32_t ulTimer = 0;

	/* Wait until FPGA finishes the detection process on each module - Bits 8 - 13 reflect detection ready for modules 1 - 6*/
	ulTimer = nai_get_timer(0);
	while (1)
	{
		if (nai_get_timer(ulTimer) > COMPLETION_TIMEOUT)
		{
			nStatus = NAI_DETECT_MODULES_TIMEOUT;
			break;
		}

		tFIFOVal.unValue = nai_read32(unModuleDetectedStatus);
		if ((tFIFOVal.unValue & 0x3F00) == 0x3F00)
			break;
    }
	
	if (nStatus == NAI_SUCCESS)
	{
		/* Now that detection process is complete..we can get the module status */
		*pusSlotIDs = (uint16_t)(tFIFOVal.unValue & 0x003F);
	}
	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_get_module_id_and_address is responsible for retrieving the module ID and module offset address based on 
the given (passed in) global address.
</summary>
<param name="unAddress"> : (Input) 32 Bit global address (includes card offset).</param>
<param name="pucModuleID"> : (Output) Pointer to the Module ID that is determined based on the passed in 
global address.</param>
<param name="pucModuleAddress"> : (Output) Pointer to the Module Address (Card offset is stripped away)</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
int32_t nai_get_module_id_and_address(uint32_t unAddress, uint8_t *pucModuleID, uint32_t *punModuleAddress)
{
	int32_t nStatus = NAI_SUCCESS;	

	if (ucHardCodedModuleSlot == 0)
	{
		uint8_t i = 0;
		uint8_t k = 0;
		BOOL bFound = FALSE;

		for (i=0; i < THEORETICAL_MAX_SLOTS; i++)
		{            
			/* Skip over any unused module slots */
			if (g_unModuleStartAddresses[i] == 0xFFFFFFFF)
				continue;

			/* Does the given address match exactly with the module start address we are currently looking at? */
			if (unAddress == g_unModuleStartAddresses[i])
			{
				*pucModuleID = (i + 1);
				*punModuleAddress = (unAddress - g_unModuleStartAddresses[i]);
				bFound = TRUE;					
				break;		
			}
				
			/* Now find a valid slot to check against..once again skipping over all empty slots */
			k = (i + 1);
			while ((k < THEORETICAL_MAX_SLOTS) && g_unModuleStartAddresses[k] == 0xFFFFFFFF)
				k++;
			
			/* If no more slots to check...break */
			if (k == THEORETICAL_MAX_SLOTS)
			{
				/* It is possible that the given address is greater than the last physical slot starting address and that there are 
                   no more modules...this means the address correlates with the last occupied slot */
				if (unAddress > g_unModuleStartAddresses[i])
				{
					*pucModuleID = (i + 1);
					*punModuleAddress = (unAddress - g_unModuleStartAddresses[i]);
					bFound = TRUE;					
				}
				break;			
			}
					
			/* Now let's check to see if we found the slot the given address belongs to */
			if (unAddress > g_unModuleStartAddresses[i] && unAddress < g_unModuleStartAddresses[k])
			{
				*pucModuleID = (i + 1);
				*punModuleAddress = (unAddress - g_unModuleStartAddresses[i]);
				bFound = TRUE;
				break;
			}

			/* Increment i passed all the indexes we already tested...take into account that i will be incremented and we want to use the ending boundary now 
               as the start of our next check */
			i = (k-1);
		}

		if (!bFound)
			nStatus = NAI_MODULE_NOT_FOUND;
	}
	else
	{
		*pucModuleID = ucHardCodedModuleSlot;
		*punModuleAddress = unAddress;
	}

	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_write16 is responsible for writing a 16 bit value to the specified address.
</summary>
<param name="unAddress"> : (Input) 32 Bit address of location to write to.</param>
<param name="usValue"> : (Input) 16 Bit value to be written.</param>
<returns>VOID</returns>
*/
/**************************************************************************************************************/
void nai_write16(uint32_t unAddress, uint16_t usValue)
{
	#ifdef __LINUX		
		write_LWHPS2FPGA_Int16(LWHPS2FPGA_OPER_COMMONAREA_ADDR+unAddress,usValue);
	#elif defined(__UBOOT) || defined (__BAREMETAL)
		*((volatile uint32_t *)(g_unBaseAddress + unAddress)) = usValue;
	#elif __VXWORKS	
		vxbWrite16(48, g_unBaseAddress + unAddress, usValue);
	#endif
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_read16 is responsible for reading a 16 bit value from the specified address.
</summary>
<param name="unAddress"> : (Input) 32 Bit address of location of where to read from.</param>
<returns>uint16_t - 16 Bit value read from desired register</returns>
*/
/**************************************************************************************************************/
uint16_t nai_read16(uint32_t unAddress)
{
	uint16_t usValue = 0;

	#ifdef __LINUX
		read_LWHPS2FPGA_Int16(LWHPS2FPGA_OPER_COMMONAREA_ADDR+unAddress,&usValue);
	#elif defined(__UBOOT) || defined(__BAREMETAL)
		usValue =  (uint16_t)(*(volatile uint32_t *)(g_unBaseAddress + unAddress));
	#elif __VXWORKS	
		usValue = vxbRead16(48,(g_unBaseAddress + unAddress));
	#endif
	
	return usValue;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_write32 is responsible for writing a 32 bit value to the specified address.
</summary>
<param name="unAddress"> : (Input) 32 Bit address of location to write to.</param>
<param name="unValue"> : (Input) 32 Bit value to be written.</param>
<returns>VOID</returns>
*/
/**************************************************************************************************************/
void nai_write32(uint32_t unAddress, uint32_t unValue)
{
	#ifdef __LINUX
	write_LWHPS2FPGA_Int32(LWHPS2FPGA_OPER_COMMONAREA_ADDR+unAddress,unValue);
	#elif defined(__UBOOT) || defined(__BAREMETAL)
#ifdef _DEBUG_X
		printf("nai_write32 address 0x%x, value 0x%x\n", g_unBaseAddress + unAddress, unValue);
#endif
		*((volatile uint32_t *)(g_unBaseAddress + unAddress)) = unValue;
	#elif __VXWORKS	
#ifdef _DEBUG_X
		printf("nai_write32 address 0x%x, value 0x%x\n", g_unBaseAddress + unAddress, unValue);
#endif		
		vxbWrite32(48, g_unBaseAddress + unAddress, unValue);
	#endif
}


/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_read32 is responsible for reading a 32 bit value from the specified address.
</summary>
<param name="unAddress"> : (Input) 32 Bit address of location of where to read from.</param>
<returns>uint16_t - 32 Bit value read from desired register</returns>
*/
/**************************************************************************************************************/
uint32_t nai_read32(uint32_t unAddress)
{
	uint32_t unValue = 0;
		
	#ifdef __LINUX
	read_LWHPS2FPGA_Int32(LWHPS2FPGA_OPER_COMMONAREA_ADDR+unAddress,&unValue);
	#elif defined(__UBOOT) || defined(__BAREMETAL)
		unValue =  *(volatile uint32_t *)(g_unBaseAddress + unAddress);
	#elif __VXWORKS			
		unValue = vxbRead32(48,(g_unBaseAddress + unAddress));
	#endif
	
	return unValue;
}

#if defined(__UBOOT) || defined(__BAREMETAL)
/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_common_write32 is responsible for writing a 32 bit value to the module's common area specified address.
</summary>
<param name="unAddress"> : (Input) 32 Bit address of location to write to.</param>
<param name="unValue"> : (Input) 32 Bit value to be written.</param>
<returns>VOID</returns>
*/
/**************************************************************************************************************/
void nai_common_write32(uint32_t unAddress, uint32_t unValue)
{
#ifdef _DEBUG_X
	printf("nai_common_write32 address 0x%x, value 0x%x\n", getNaiModuleCommonBaseAddr() + unAddress, unValue);
#endif
	*((volatile uint32_t *)(getNaiModuleCommonBaseAddr() + unAddress)) = unValue;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_common_read32 is responsible for reading a 32 bit value from the specified address of the module's common area.
</summary>
<param name="unAddress"> : (Input) 32 Bit address of location of where to read from.</param>
<returns>uint16_t - 32 Bit value read from desired register</returns>
*/
/**************************************************************************************************************/
uint32_t nai_common_read32(uint32_t unAddress)
{
	uint32_t unValue = 0;		
	unValue =  *(volatile uint32_t *)(getNaiModuleCommonBaseAddr() + unAddress);	
	return unValue;
}
#endif

#if defined(__UBOOT) || defined(__BAREMETAL)
/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_dt2_write32 is responsible for writing a 32 bit value to a specified address of the Microcontroller on a DT2 module.
</summary>
<param name="unAddress"> : (Input) 32 Bit address of location to write to.</param>
<param name="unValue"> : (Input) 32 Bit value to be written.</param>
<returns>VOID</returns>
*/
/**************************************************************************************************************/
void nai_dt2_write32(uint32_t unAddress, uint32_t unValue)
{
	int32_t ulTimer;
#ifdef _VERBOSE
	printf("nai_dt_write32 address 0x%x, value 0x%x\n", getNaiModuleDT2CtrlBaseAddr() + unAddress, unValue);
#endif
	*((volatile uint32_t *)(getNaiModuleDT2CtrlBaseAddr() + unAddress)) = unValue;
	
	/* NOTE: we need to provide a slight delay between writes ... without delay we get inconsistent results */
	ulTimer = nai_get_timer(0);
	while (1)
	{			
		if (nai_get_timer(ulTimer) > 1)
			break;
	}
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_dt2_read32 is responsible for reading a 32 bit value from the specified address of the Microcontroller on a DT2 module.
</summary>
<param name="unAddress"> : (Input) 32 Bit address of location of where to read from.</param>
<returns>uint16_t - 32 Bit value read from desired register</returns>
*/
/**************************************************************************************************************/
uint32_t nai_dt2_read32(uint32_t unAddress)
{
	uint32_t unValue = 0;
	int32_t ulTimer;
	
	unValue =  *(volatile uint32_t *)(getNaiModuleDT2CtrlBaseAddr() + unAddress);
#ifdef _VERBOSE	
//	printf("nai_dt_read32 address 0x%x, value 0x%x\n", getNaiModuleDT2CtrlBaseAddr() + unAddress, unValue);
#endif	

	/* NOTE: we need to provide a slight delay between reads ... without delay we get inconsistent results */
	ulTimer = nai_get_timer(0);
	while (1)
	{			
		if (nai_get_timer(ulTimer) > 1)
			break;
	}
	
	return unValue;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_dt2_cal_write32 is responsible for writing a 32 bit value to a specified address of the Microcontroller on a DT2 module.
</summary>
<param name="unAddress"> : (Input) 32 Bit address of location to write to.</param>
<param name="unValue"> : (Input) 32 Bit value to be written.</param>
<returns>VOID</returns>
*/
/**************************************************************************************************************/
void nai_dt2_cal_write32(uint32_t unAddress, uint32_t unValue)
{
	int32_t ulTimer;
#ifdef _VERBOSE
	printf("nai_dt2_cal_write32 address 0x%x, value 0x%x\n", getNaiModuleDT2CalBaseAddr() + unAddress, unValue);
#endif
	*((volatile uint32_t *)(getNaiModuleDT2CalBaseAddr() + unAddress)) = unValue;
	
	/* NOTE: we need to provide a slight delay between writes ... without delay we get inconsistent results */
	ulTimer = nai_get_timer(0);
	while (1)
	{			
		if (nai_get_timer(ulTimer) > 1)
			break;
	}
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_dt2_cal_read32 is responsible for reading a 32 bit value from the specified address of the Microcontroller on a DT2 module.
</summary>
<param name="unAddress"> : (Input) 32 Bit address of location of where to read from.</param>
<returns>uint16_t - 32 Bit value read from desired register</returns>
*/
/**************************************************************************************************************/
uint32_t nai_dt2_cal_read32(uint32_t unAddress)
{
	uint32_t unValue = 0;
	int32_t ulTimer;
	
	unValue =  *(volatile uint32_t *)(getNaiModuleDT2CalBaseAddr() + unAddress);
#ifdef _VERBOSE	
	printf("nai_dt2_cal_read32 address 0x%x, value 0x%x\n", getNaiModuleDT2CalBaseAddr() + unAddress, unValue);
#endif	

	/* NOTE: we need to provide a slight delay between reads ... without delay we get inconsistent results */
	ulTimer = nai_get_timer(0);
	while (1)
	{			
		if (nai_get_timer(ulTimer) > 1)
			break;
	}
	
	return unValue;
}
#endif

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_get_tx_fifo_address is responsible for returning the TX FIFO address used to obtain TX Data.
</summary>
<param name="ucRequesterID"> : (Input) Slot ID of where receive request originated.</param>
<param name="ucCompleterID"> : (Input) Slot ID of where receive request is destined for.</param>
<returns>uint32_t : Address location of TX FIFO Data</returns>
*/
/**************************************************************************************************************/
uint32_t nai_get_tx_fifo_address(uint8_t ucRequesterID, uint8_t ucCompleterID)
{
	volatile uint32_t unTxAddr = 0x00000000; /* Default to Module TX FIFO address */

	if (nai_is_mb(ucRequesterID))
	{
		switch (ucCompleterID)
		{	
			case MODULE_1_SLOT :
				unTxAddr = 0x00000000;				
				break;

			case MODULE_2_SLOT :
				unTxAddr = 0x00000004;				
				break;

			case MODULE_3_SLOT :
				unTxAddr = 0x00000008;
				break;

			case MODULE_4_SLOT :
				unTxAddr = 0x0000000C;
				break;

			case MODULE_5_SLOT :
				unTxAddr = 0x00000010;
				break;

			case MODULE_6_SLOT :
				unTxAddr = 0x00000014;
				break;
		};
	}

	return unTxAddr;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_get_tx_fifo_pkt_ready_address is responsible for returning the TX FIFO address used to obtain Packet Ready
Status.
</summary>
<param name="ucRequesterID"> : (Input) Slot ID of where receive request originated.</param>
<param name="ucCompleterID"> : (Input) Slot ID of where receive request is destined for.</param>
<returns>uint32_t : Address location of TX FIFO Packet Ready</returns>
*/
/**************************************************************************************************************/
uint32_t nai_get_tx_fifo_pkt_ready_address(uint8_t ucRequesterID, uint8_t ucCompleterID)
{
	volatile uint32_t unTxPktReadyAddr = 0x00000020; /* Default to Module Packet Ready address */

	if (nai_is_mb(ucRequesterID))
	{
		switch (ucCompleterID)
		{	
			case MODULE_1_SLOT :
				unTxPktReadyAddr = 0x00000020;
				break;

			case MODULE_2_SLOT :
				unTxPktReadyAddr = 0x00000024;
				break;

			case MODULE_3_SLOT :
				unTxPktReadyAddr = 0x00000028;
				break;

			case MODULE_4_SLOT :			
				unTxPktReadyAddr = 0x0000002C;
				break;

			case MODULE_5_SLOT :				
				unTxPktReadyAddr = 0x00000030;
				break;

			case MODULE_6_SLOT :				
				unTxPktReadyAddr = 0x00000034;
				break;
		};
	}

	return unTxPktReadyAddr;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_get_rx_fifo_address is responsible for returning the FIFO address used to obtain the data currently in the 
specified FIFO.
</summary>
<param name="ucCompleterID"> : (Input) Slot ID of where data will be coming from.</param>
<returns>uint32_t : Address location of RX FIFO Data</returns>
*/
/**************************************************************************************************************/
uint32_t nai_get_rx_fifo_address(uint8_t ucCompleterID)
{
	volatile uint32_t unRxAddr = 0x00000080;	/* Default to Module Read Data address */

#ifdef _VERBOSE
	printf("nai_get_rx_fifo_address:  g_ucSlotID = %u\r\n", g_ucSlotID);	
#endif

	if (nai_is_mb(g_ucSlotID))
	{
		switch (ucCompleterID)
		{	
			case MODULE_1_SLOT :
				unRxAddr = 0x00000080;
				break;

			case MODULE_2_SLOT :
				unRxAddr = 0x00000084;
				break;

			case MODULE_3_SLOT :
				unRxAddr = 0x00000088;
				break;

			case MODULE_4_SLOT :			
				unRxAddr = 0x0000008C;
				break;

			case MODULE_5_SLOT :				
				unRxAddr = 0x00000090;
				break;

			case MODULE_6_SLOT :				
				unRxAddr = 0x00000094;
				break;

			default:
				break;
		};
	}

	return unRxAddr;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_get_rx_fifo_num_words_address is responsible for returning the FIFO address used to obtain the number of 
words currently in the specified FIFO.
</summary>
<param name="ucRequesterID"> : (Input) Slot ID of where receive request originated.</param>
<param name="ucCompleterID"> : (Input) Slot ID of where receive request is destined for.</param>
<returns>uint32_t : Address location of RX FIFO Number of Words</returns>
*/
/**************************************************************************************************************/
uint32_t nai_get_rx_fifo_num_words_address(uint8_t ucRequesterID, uint8_t ucCompleterID)
{
	volatile uint32_t unRxNumWordsAddr = 0x000000A0; /* Default to Module Num Words address */

	if (nai_is_mb(ucRequesterID))
	{
		switch (ucCompleterID)
		{	
			case MODULE_1_SLOT :
				unRxNumWordsAddr = 0x000000A0;
				break;

			case MODULE_2_SLOT :
				unRxNumWordsAddr = 0x000000A4;
				break;

			case MODULE_3_SLOT :
				unRxNumWordsAddr = 0x000000A8;
				break;

			case MODULE_4_SLOT :			
				unRxNumWordsAddr = 0x000000AC;
				break;

			case MODULE_5_SLOT :				
				unRxNumWordsAddr = 0x000000B0;
				break;

			case MODULE_6_SLOT :				
				unRxNumWordsAddr = 0x000000B4;
				break;
		};
	}

	return unRxNumWordsAddr;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_tx_fifo_empty is responsible for determining if the SERDES TX FIFO for the specified slot is empty.
</summary>
<param name="ucRequesterID"> : (Input) Slot identifier governing what slot the request is coming from.</param>
<param name="ucCompleterID"> : (Input) Slot identifier governing what slot the request is targeted for.</param>
<returns>BOOL : TRUE if FIFO is empty</returns>
*/
/**************************************************************************************************************/
BOOL nai_tx_fifo_empty(uint8_t ucRequesterID, uint8_t ucCompleterID)
{
	BOOL bFIFOEmpty = TRUE;
	volatile uint32_t unFIFOsEmptyStatusRegister = 0x00000040;  /* Assume module slot by default */

	if (ucRequesterID == ASSIGNED_SLOT_ID) 
			/*if (ucRequesterID == PPC_MB_SLOT)HNP*/
		ucRequesterID = g_ucSlotID;

/*DANT
	if (ucRequesterID > g_usMaxSlotCount)
		return FALSE;
*/

	if (nai_is_mb(ucRequesterID))
	{
		if (ucCompleterID > g_usMaxSlotCount)
			return FALSE;

		unFIFOsEmptyStatusRegister = 0x00000040;
	
		/* Bit shift by desired slot number (MB=0, Slot1=1, Slot2=2 ... Slot6=6) */				
		uint8_t ucTemp = (uint8_t)(nai_read16(unFIFOsEmptyStatusRegister));		
		bFIFOEmpty = (((ucTemp >> (ucCompleterID-1)) & 0x01) == 0x01);		
	}
	else /* Request is from module so no need for bit shifting */
	{
		uint8_t ucTemp = (uint8_t)(nai_read16(unFIFOsEmptyStatusRegister));		
		bFIFOEmpty = (ucTemp == 0x01);
	}
	return bFIFOEmpty;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_rx_fifo_empty is responsible for determining if the SERDES RX FIFO for the specified slot is empty.
</summary>
<param name="ucRequesterID"> : (Input) Slot identifier governing what slot the request is coming from.</param>
<param name="ucCompleterID"> : (Input) Slot identifier governing what slot the request is targeted for.</param>
<returns>BOOL : TRUE if FIFO is empty</returns>
*/
/**************************************************************************************************************/
BOOL nai_rx_fifo_empty(uint8_t ucRequesterID, uint8_t ucCompleterID)
{
	BOOL bFIFOEmpty = TRUE;
	volatile uint32_t unFIFOsEmptyStatusRegister = 0x000000C0;  /* Assume module slot by default */

	if (ucRequesterID == ASSIGNED_SLOT_ID)
		ucRequesterID = g_ucSlotID;

/*DANT
	if (ucRequesterID > g_usMaxSlotCount)
		return FALSE;
*/

	if (nai_is_mb(ucRequesterID))
	{
		if (ucCompleterID > g_usMaxSlotCount)
			return FALSE;

		unFIFOsEmptyStatusRegister = 0x000000C0;

		/* Bit shift by desired slot number (MB=0, Slot1=1, Slot2=2 ... Slot6=6) */
		uint8_t ucTemp = (uint8_t)(nai_read16(unFIFOsEmptyStatusRegister));
		bFIFOEmpty = (((ucTemp >> (ucCompleterID-1)) & 0x01) == 0x01);
	}
	else /* Request is from module so no need for bit shifting */				
	{
		uint8_t ucTemp = (uint8_t)(nai_read16(unFIFOsEmptyStatusRegister));
		bFIFOEmpty = (ucTemp == 0x01);
	}
	return bFIFOEmpty;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_rx_fifo_pkt_ready is responsible for determining if the SERDES RX FIFO has a funl packet ready for retrieval.
</summary>
<param name="ucRequesterID"> : (Input) Slot identifier governing what slot the request is coming from.</param>
<param name="ucCompleterID"> : (Input) Slot identifier governing what slot the request is targeted for.</param>
<returns>BOOL : TRUE if FIFO is empty</returns>
*/
/**************************************************************************************************************/
BOOL nai_rx_fifo_pkt_ready(uint8_t ucRequesterID, uint8_t ucCompleterID)
{
	BOOL bPktReady = FALSE;
	volatile uint32_t unFIFOsPktReadyStatusRegister = 0x000000E4;  /* Assume module slot by default */

	if (ucRequesterID == ASSIGNED_SLOT_ID)
		ucRequesterID = g_ucSlotID;

/*DANT
	if (ucRequesterID > g_usMaxSlotCount)
		return FALSE;
*/
	if (nai_is_mb(ucRequesterID))
	{
		if (ucCompleterID > g_usMaxSlotCount)
			return FALSE;
 
		unFIFOsPktReadyStatusRegister = 0x000000E4;

//#ifdef _VERBOSE
//	printf("nai_rx_fifo_pkt_ready: %u \r\n", (*(uint8_t *)unFIFOsPktReadyStatusRegister));
//#endif

		/* Read pkt Ready Status register */
		uint8_t ucTemp = (uint8_t)(nai_read16(unFIFOsPktReadyStatusRegister));
		bPktReady = (((ucTemp >> (ucCompleterID-1)) & 0x01) == 0x01);
	}
	else /* Request is from module so no need for bit shifting */
	{		
		uint8_t ucTemp = (uint8_t)(nai_read16(unFIFOsPktReadyStatusRegister));
		bPktReady = (ucTemp == 0x01);
	}
	return bPktReady;
}

/**************************************************************************************************************/
/**
\ingroup MessageProcessing
<summary>
nai_rx_fifo_clear_pkt_ready is responsible for clearing the SERDES RX FIFO full packet ready bit.
</summary>
<param name="ucRequesterID"> : (Input) Slot identifier governing what slot the request is coming from.</param>
<param name="ucCompleterID"> : (Input) Slot identifier governing what slot the request is targeted for.</param>
<returns>void</returns>
*/
/**************************************************************************************************************/
void nai_rx_fifo_clear_pkt_ready(uint8_t ucRequesterID, uint8_t ucCompleterID)
{	
	volatile uint32_t unFIFOsClearPktReadyStatusRegister = 0x000000E4;  /* Assume module slot by default */

	if (ucRequesterID == ASSIGNED_SLOT_ID)
		ucRequesterID = g_ucSlotID;

/*DANT
	if (ucRequesterID > g_usMaxSlotCount)
		return;
*/

	if (nai_is_mb(ucRequesterID))
	{
		if (ucCompleterID > g_usMaxSlotCount)
			return;
 
		unFIFOsClearPktReadyStatusRegister = 0x000000E4;

		/* Write a 1 to clear pkt Ready Status register */
		nai_write16(unFIFOsClearPktReadyStatusRegister,(uint16_t)(0x01 << (ucCompleterID-1)));
	}
	else /* Request is from module so no need for bit shifting */
	{
		nai_write16(unFIFOsClearPktReadyStatusRegister, (uint16_t)0x0001);
	}
}

/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
nai_init_as_slot is responsible for initializing the global Slot Number to reflect that this is the desired slot.
</summary>
<param name="ucSlotID"> : (Input) Input:Slot ID being assigned.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
int32_t nai_init_as_slot(uint8_t ucSlotID)
{
#ifdef _VERBOSE
	printf("nai_init_as_slot: 0x%1x\r\n", ucSlotID);
#endif

	if (ucSlotID > g_usMaxSlotCount)
		return NAI_INVALID_SLOT_ID;

	g_ucSlotID = ucSlotID;

#ifndef __LINUX
	g_unBaseAddress = getNaiAddr();		
#endif		

	return NAI_SUCCESS;
}

/**************************************************************************************************************/
/**
\ingroup HighLevelAPI
<summary>
nai_get_max_slot_count is responsible for returning the max number of module slots that can be used.
</summary>
<returns>uint16_t : Maximum number of slots
</returns>
*/
/**************************************************************************************************************/
uint16_t nai_get_max_slot_count(void)
{
	return g_usMaxSlotCount;
}


/**************************************************************************************************************/
/**
\ingroup MessageUtils
<summary>
convert_bytes_to_words is responsible for coverting the specified number of bytes into words.
</summary>
<param name="ulBytes"> : (Input) Numeric value (of bytes) to be converted to words.</param>
<returns>uint32_t : Number of words</returns>
*/
/**************************************************************************************************************/
uint32_t convert_bytes_to_words(uint32_t unBytes)
{
	uint32_t unWordCount = 0;
	
	//It is possible that we are dealing with odd number of words.
	if ((unBytes % 2) > 0)
		unWordCount = 1;

	unWordCount += (uint32_t)(unBytes / 2);

	return unWordCount;
}

/**************************************************************************************************************/
/**
\ingroup MessageUtils
<summary>
do_naisleep is responsible for forcing a stall in execution for the desired number of seconds.
</summary>
<param name="unSeconds"> : (Input) Number of seconds to sleep.</param>
<returns>None</returns>
*/
/**************************************************************************************************************/
int32_t do_naisleep(uint32_t unSeconds)
{
#ifdef __LINUX
	sleep(unSeconds);

#elif __BAREMETAL
	int32_t i = 0;

	for (i=0; i< 750; i++)
		printf("Sleep Loop Cnt: %ld\r\n", i);
#elif __UBOOT
	ulong start = get_timer(0);
	ulong delay;

	delay = (ulong)unSeconds * CONFIG_SYS_HZ;

	while (get_timer(start) < delay) {
		if (ctrlc ())
			return (-1);

		udelay (100);
	}
#endif
	return 0;
}

#ifdef __UBOOT
/**************************************************************************************************************/
/**
\ingroup FlashUtils
<summary>
probe_qspi is responsible for probing to see if flash is present.
</summary>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
static BOOL g_bQSPIProbed = FALSE;
int32_t probe_qspi()
{
	int32_t nStatus = NAI_SUCCESS;
	
	if (!g_bQSPIProbed)
	{
		int32_t argc = 2;
		char *argv[argc+1];

		char arg0[MAX_ARG_LEN];
		char arg1[MAX_ARG_LEN];
	
		memset(arg0, 0, MAX_ARG_LEN);
		snprintf(arg1, MAX_ARG_LEN, "probe");

		argv[0] = &arg0[0];
		argv[1] = &arg1[0];
		argv[2] = NULL;
	
		nStatus = do_spi_flash(NULL, 0, argc, argv);
		
		if (nStatus == NAI_SUCCESS)
			g_bQSPIProbed = TRUE;
	}

	return nStatus;	
}

/**************************************************************************************************************/
/**
\ingroup FlashUtils
<summary>
probe_qspi is responsible for erasing flash (or a portion thereof).
</summary>
<param name="unOffset"> : (Input) Offset into flash from which to start erasing.</param>
<param name="unLen"> : (Input) Length (in Bytes) of how much flash to erase.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
int32_t erase_qspi(uint32_t unOffset, uint32_t unLen)
{
	int32_t nStatus = NAI_SUCCESS;
	int32_t argc = 3;
	char *argv[argc+1];

	char arg0[MAX_ARG_LEN];
	char arg1[MAX_ARG_LEN];
	char arg2[MAX_ARG_LEN];

	snprintf(arg0, MAX_ARG_LEN, "erase");
	snprintf(arg1, MAX_ARG_LEN, "%x", unOffset);
	snprintf(arg2, MAX_ARG_LEN, "+%x", unLen);
	
	argv[0] = &arg0[0];
	argv[1] = &arg1[0];
	argv[2] = &arg2[0];
	argv[3] = NULL;

#ifdef _VERBOSE
	printf("Erase argv[0] = %s\r\n", argv[0]);	
	printf("Erase argv[1] = %s\r\n", argv[1]);
	printf("Erase argv[2] = %s\r\n", argv[2]);
#endif

	nStatus = do_spi_flash_erase(argc, argv);
	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup FlashUtils
<summary>
write_to_qspi is responsible for writing the specified amount of data to flash.
</summary>
<param name="unAddr"> : (Input) Address of RAM from which to take data to write.</param>
<param name="unOffset"> : (Input) Offset into flash of where to start writing.</param>
<param name="unLen"> : (Input) Length (in Bytes) of how much data to write.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
int32_t write_to_qspi( uint32_t unAddr, uint32_t unOffset, uint32_t unLen)
{
	int32_t nStatus = NAI_SUCCESS;	
	int32_t argc = 4;
	char *argv[argc+1];

	char arg0[MAX_ARG_LEN];
	char arg1[MAX_ARG_LEN];
	char arg2[MAX_ARG_LEN];
	char arg3[MAX_ARG_LEN];

	snprintf(arg0, MAX_ARG_LEN, "write");
	snprintf(arg1, MAX_ARG_LEN, "0x%x", unAddr);
	snprintf(arg2, MAX_ARG_LEN, "0x%x", unOffset);
	snprintf(arg3, MAX_ARG_LEN, "0x%x", unLen);
	
	argv[0] = &arg0[0];
	argv[1] = &arg1[0];
	argv[2] = &arg2[0];
	argv[3] = &arg3[0];
	argv[4] = NULL;

	nStatus = do_spi_flash_read_write(argc, argv);
	return nStatus;
}

/**************************************************************************************************************/
/**
\ingroup FlashUtils
<summary>
read_from_qspi is responsible for reading the specified amount of data from flash to a memory location in RAM.
</summary>
<param name="unAddr"> : (Input) Address of RAM where data will be stored when flash is read.</param>
<param name="unOffset"> : (Input) Offset into flash of where to start reading.</param>
<param name="unLen"> : (Input) Length (in Bytes) of how much data to read.</param>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
int32_t read_from_qspi( uint32_t unAddr, uint32_t unOffset, uint32_t unLen)
{
	int32_t nStatus = NAI_SUCCESS;
	int32_t argc = 4;
	char *argv[argc+1];

	char arg0[MAX_ARG_LEN];
	char arg1[MAX_ARG_LEN];
	char arg2[MAX_ARG_LEN];
	char arg3[MAX_ARG_LEN];

	snprintf(arg0, MAX_ARG_LEN, "read");
	snprintf(arg1, MAX_ARG_LEN, "0x%x", unAddr);
	snprintf(arg2, MAX_ARG_LEN, "0x%x", unOffset);
	snprintf(arg3, MAX_ARG_LEN, "0x%x", unLen);

	argv[0] = &arg0[0];
	argv[1] = &arg1[0];
	argv[2] = &arg2[0];
	argv[3] = &arg3[0];
	argv[4] = NULL;

#ifdef _VERBOSE
	printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\r\n");
	printf("arg0[0] = %s\r\n", arg0);
	printf("arg0[1] = %s\r\n", arg1);
	printf("arg0[2] = %s\r\n", arg2);
	printf("arg0[3] = %s\r\n", arg3);
	printf("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\r\n");
#endif

	nStatus = do_spi_flash_read_write(argc, argv);
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
write_at24c is responsible for writing the specified buffer to EEPROM.
</summary>
<param name="chip"> : (Input) Chip to be written to.</param>
<param name="addr"> : (Input) Address at which to start writing.</param>
<param name="buf"> : (Input) Buffer of data to be written.</param>
<param name="len"> : (Input) Length of the buffer.</param>
<returns>None</returns>
*/
/**************************************************************************************************************/
int32_t write_at24c(uint8_t chip, uint32_t addr, uint8_t *buf, int32_t len) 
{
    int32_t i;
    int32_t curr;
    int32_t rc;
    const int32_t page_size = 8;

#ifdef _VERBOSE
	for (i = 0; i < len; i++) 
		printf("write_at24c - buff[%ld] = %u\r\n", i, (unsigned)buf[i]);
#endif

    for (i = 0, curr = 0; i < len; i += curr) 
	{
		curr = len - i;
		curr = ((curr <= page_size) ? curr : page_size);
//printf("About to call i2c_write with params: chip = %x  addr = %x, 1, offset = %x, length = %x\r\n", chip, (addr + i), (buf + i), curr);
		rc = i2c_write(chip, addr + i, 1, buf + i, curr);			
		udelay(10000); /* Wait 10 milli-seconds .. if we don't wait there are is a timing issue */
		if (rc)
			break;
    }

    return rc;
}
#endif


#ifndef __UBOOT
/*-
 *  COPYRIGHT (C) 1986 Gary S. Brown.  You may use this program, or
 *  code or tables extracted from it, as desired without restriction.
 *
 *  First, the polynomial itself and its table of feedback terms.  The
 *  polynomial is
 *  X^32+X^26+X^23+X^22+X^16+X^12+X^11+X^10+X^8+X^7+X^5+X^4+X^2+X^1+X^0
 *
 *  Note that we take it "backwards" and put the highest-order term in
 *  the lowest-order bit.  The X^32 term is "implied"; the LSB is the
 *  X^31 term, etc.  The X^0 term (usually shown as "+1") results in
 *  the MSB being 1
 *
 *  Note that the usual hardware shift register implementation, which
 *  is what we're using (we're merely optimizing it by doing eight-bit
 *  chunks at a time) shifts bits into the lowest-order term.  In our
 *  implementation, that means shifting towards the right.  Why do we
 *  do it this way?  Because the calculated CRC must be transmitted in
 *  order from highest-order term to lowest-order term.  UARTs transmit
 *  characters in order from LSB to MSB.  By storing the CRC this way
 *  we hand it to the UART in the order low-byte to high-byte; the UART
 *  sends each low-bit to hight-bit; and the result is transmission bit
 *  by bit from highest- to lowest-order term without requiring any bit
 *  shuffling on our part.  Reception works similarly
 *
 *  The feedback terms table consists of 256, 32-bit entries.  Notes
 *
 *      The table can be generated at runtime if desired; code to do so
 *      is shown later.  It might not be obvious, but the feedback
 *      terms simply represent the results of eight shift/xor opera
 *      tions for all combinations of data and CRC register values
 *
 *      The values must be right-shifted by eight bits by the "updcrc
 *      logic; the shift must be unsigned (bring in zeroes).  On some
 *      hardware you could probably optimize the shift in assembler by
 *      using byte-swap instructions
 *      polynomial $edb88320
 *
 *
 * CRC32 code derived from work by Gary S. Brown.
 */
static uint32_t crc32_tab[] = {
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
	0xe963a535, 0x9e6495a3,	0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
	0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
	0xf3b97148, 0x84be41de,	0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec,	0x14015c4f, 0x63066cd9,
	0xfa0f3d63, 0x8d080df5,	0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
	0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,	0x35b5a8fa, 0x42b2986c,
	0xdbbbc9d6, 0xacbcf940,	0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
	0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
	0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,	0x76dc4190, 0x01db7106,
	0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
	0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
	0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
	0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
	0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
	0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
	0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
	0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
	0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
	0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
	0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
	0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
	0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
	0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
	0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
	0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
	0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
	0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
	0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
	0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
	0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
	0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
	0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
	0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

uint32_t crc32(uint32_t crc, const void *buf, size_t size)
{
	const uint8_t *p;

	p = buf;
	crc = crc ^ ~0U;

	while (size--)
		crc = crc32_tab[(crc ^ *p++) & 0xFF] ^ (crc >> 8);

	return crc ^ ~0U;
}
#endif
