/*
 * NAI Receive
 */
#include "NAIComms.h"
#include "cmd_naimsgutils.h"
#include "cmd_naiconfigmsgutils.h"

#ifndef __UBOOT
#include <stdio.h>
#include <string.h>
#else
#include <common.h>
#endif

extern BOOL g_bExitConfigMode;
BOOL nai_is_config_mode_requested(void);

/**************************************************************************************************************/
/**
\ingroup NAIReceive
<summary>
nai_is_config_mode_requested is responsible for checking GPIO 54 to see if the module is being instructed to
stay in configuration mode. If the GPIO is set high (1), naireceive will stay running and waiting for configuration
messages to be received. If the GPIO is set low (0), naireceive will exit allowing for the operational baremetal
application to start.
</summary>
<returns>BOOL - true if config mode is being requested; false otherwise
</returns>
*/
/**************************************************************************************************************/
BOOL nai_is_config_mode_requested()
{
	volatile uint32_t unValue = 0;
	uint32_t unGPIODirectionAddr = 0xE000A284;
	uint32_t unGPIOAddr = 0xE000A068;
	
	/* Ensure GPIO pin 55 used for Config Mode is configured for input */	
	unValue =  *(volatile uint32_t *)(unGPIODirectionAddr);
	unValue &= 0xFFFFFFFD;  /* Zero out 2nd bit to ensure it is configured as input */
	*((volatile uint32_t *)(unGPIODirectionAddr)) = unValue;

	/* Now check the actual value of the GPIO to see if we need to stay in config mode */
	unValue =  *(volatile uint32_t *)(unGPIOAddr);		
#ifdef _VERBOSE
	printf("GPIO value = %x\r\n", unValue);
	printf("Computation = %x\r\n", (unValue & 2));
#endif
	return ((unValue & 2) == 2); /* (GPIO 55) */
}

/**************************************************************************************************************/
/**
\ingroup NAIReceive
<summary>
do_naireceive is responsible for copying contents of EEPROMs found on the module to a common area as well as
parameter and calibration information. This function is also responsible for receiving and processing configuration 
messages if instructed to stay in config mode.
</summary>
<returns>int32_t : Status 
	- 0  : SUCCESS
	- Non-Zero : ERROR
</returns>
*/
/**************************************************************************************************************/
int do_naireceive(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[])
{	
	uint8_t ucModuleSlot = INVALID_SLOT_ID;
	uint8_t ucForceStayInReceive = 0;
	MsgList tMsgList;
	int32_t nStatus = 0;
	g_bExitConfigMode = FALSE;

	printf("Starting naireceive ... (press any key to exit)\r\n");
	if (argc == 2)
		ucModuleSlot = (unsigned char)simple_strtoul (argv[1], NULL, 16);
			
	if (argc == 3)
		ucForceStayInReceive = (unsigned char)simple_strtoul (argv[1], NULL, 16);
		
	/* MB - Logic to service all FIFOs */
	if (ucModuleSlot != INVALID_SLOT_ID)
		nai_init_as_slot(ucModuleSlot);

	/* Copy contents of EEPROM to Module Common area */
	nStatus = nai_copy_eeprom_to_common(I2C_INTERFACE_MODULE_EEPROM);
	if (nStatus != NAI_SUCCESS)
	{
		printf("FAILED to copy EEPROM data from chip %x to module common area\r\n", I2C_INTERFACE_MODULE_EEPROM);
		return nStatus;
	}

	nStatus = nai_copy_eeprom_to_common(I2C_FUNCTIONAL_MODULE_EEPROM);
	if (nStatus != NAI_SUCCESS)
	{
		printf("FAILED to copy EEPROM data from chip %x to module common area\r\n", I2C_FUNCTIONAL_MODULE_EEPROM);
#ifndef _IGNORE_MISSING_TOP_MODULE
		return nStatus
#else
		printf("Ignoring EEPROM copy failure from chip %x\r\n", I2C_FUNCTIONAL_MODULE_EEPROM);
		nStatus = NAI_SUCCESS;
#endif
	}

	/* Determine if we should be in operational or config mode.  Check GPIO line (54) - if set high
       we should stay in config mode.  If set low, we should exit config mode and allow for operational (baremetal app). */
	if ((ucForceStayInReceive == 1) || nai_is_config_mode_requested())
	{			
		printf("Ready and Waiting for configuration messages...\r\n");

		/* Use handshake register to notify MB that module is ready to accept messages 
          	 NOTE: Bit 0 is set to indicate acknowledgement of request to stay in config mode and bit 15 is set to say 
                 config mode is operation read and messages can begin to be sent */
		nai_common_write32((MODULE_COMMON_HANDSHAKE_ADDR), 0x8001);

		while (1)
		{		
			if (serial_tstc() || g_bExitConfigMode)
			{
				/* Set handshake register to notify MB that module is no longer ready to accept messages */
				nai_common_write32(MODULE_COMMON_HANDSHAKE_ADDR, 0x0);			
				break;
			}
			
			if (nai_rx_fifo_pkt_ready(ASSIGNED_SLOT_ID, INVALID_SLOT_ID))
			{
				init_nai_msgs(&tMsgList);

#ifdef _VERBOSE
				printf("FIFO NOT empty for Completer Slot ID: 0x%2x \r\n", ucModuleSlot);			
#endif
				do
				{				
					if (nai_rx_fifo_pkt_ready(ASSIGNED_SLOT_ID, INVALID_SLOT_ID))
						nai_receive_msg_packet(ASSIGNED_SLOT_ID, INVALID_SLOT_ID, &tMsgList);
				} while	(tMsgList.ptEnd->unWordsLeftToRead > 0);

#ifdef _VERBOSE
				print_nai_msgs(&tMsgList, FALSE);
#endif
				/* If message is valid (i.e. CRC checks out!), deal with the message. */
				if (validate_nai_msg(tMsgList.ptEnd) == 0)
					nStatus = deal_with_nai_msg(tMsgList.ptEnd);

				/* Send a response message to those messages requiring a completion response! */
				if (nai_msg_requires_finished_response(tMsgList.ptEnd))
					nai_send_msg_finished_response(tMsgList.ptEnd, nStatus);

				delete_nai_msgs(&tMsgList);
			}		
		}

		/* If user typed a char to exit receiver mode...eat the character so it does not get displayed on the console */
		if (serial_tstc())
			serial_getc();
	}
	else
	{
		/* Write a zero to handshake register to signal to MB that we are definitely not in config mode. */
		nai_common_write32(MODULE_COMMON_HANDSHAKE_ADDR, 0x0000);
	}
	printf("End of cmd_naireceive!\r\n");
	return nStatus;
}

/***************************************************/

#ifndef __BAREMETAL
U_BOOT_CMD(
	naireceive,	4,	1,	do_naireceive,
	"naireceive utility command",
	"[module slot number]"
	"[stay in receive: 1=STAY 0=NORMAL]"
);
#endif


