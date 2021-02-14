/*
 * (C) Copyright 2014 North Atlantic Industries, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef _NAI_MODULE_H
#define _NAI_MODULE_H

#include <asm/io.h>
#include <nai_common.h>

#define NAI_MODULE_ERROR -1
#define NAI_MODULE_SUCCESS 0

#ifdef CONFIG_NAI_MAX_MODULE_SLOT
#define MAX_MODULE_SLOT CONFIG_NAI_MAX_MODULE_SLOT
#else
#define MAX_MODULE_SLOT 6 //default to max 6 slots
#endif /*CONFIG_NAI_MAX_MODULE_SLOT*/

#define NAI_MODULE_SLOT_1	0x00
#define NAI_MODULE_SLOT_2	0x01
#define NAI_MODULE_SLOT_3	0x02
#define NAI_MODULE_SLOT_4	0x03
#define NAI_MODULE_SLOT_5	0x04
#define NAI_MODULE_SLOT_6	0x05
#define NAI_MODULE_ALL_SLOT	0xFF


enum Module_type
{
	INF_MOD = 0,
	FUNC_MOD
};

#define MODULE_ID_STRING_LEN 4

typedef struct Nai_mod_eeprom{
	char moduleId[MODULE_ID_STRING_LEN];
	u32 moduleSize;
	u8  gpioDirection;    /*Address: 0xA0*/
	u8  gpioData;		  /*Address: 0xA1*/
	u8  spare;			  /*Address: 0xA2*/
	u8  gpioConfig;		  /*Address: 0xA3*/
	u32 gpioSourceSelect; /*Address: 0xA4*/
} Nai_mod_eeprom;

typedef struct Nai_func_mod{
	Nai_mod_eeprom modEepromData;
	Nai_version version;
} Nai_func_mod; 

typedef struct Nai_inf_mod{
	Nai_mod_eeprom modEepromData;
	Nai_version version;
}Nai_inf_mod;


typedef struct Nai_mod{
	u32 modTypeId;
	u8 moduleSlot;
	bool moduleRdy;
	Nai_inf_mod infModule;
	Nai_func_mod funcModule;
}Nai_mod;

/* 
 * Init All Module
 */
void nai_init_module_board(void);
/* 
 * Enable Module(s)
 */
s32 nai_enable_module_board(u8 moduleSlot);
/* 
 * Disable Module(s)
 */
s32 nai_disable_module_board(u8 moduleSlot);
/* 
 * Module Reset
 */
s32 nai_module_reset(u8 moduleSlot);
/* 
 * Get Module Version and Other Info
 */
void nai_get_module_info(Nai_mod *mod);
/* 
 * Get Module eeprom data
 */
s32 nai_get_module_eeprom_info(Nai_mod *mod);

/* 
 * Check Module size is within range
 */
u32 nai_chk_module_size_range(u32 modSize);

/* 
 * Check Module ready state
 */
void nai_chk_mod_rdy_state(u8 slotId);

#endif /*_NAI_MODULE_H*/
