/*
 * (C) Copyright 2014 North Atlantic Industries, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef _NAI_ICB_H
#define _NAI_ICB_H

#include <asm/io.h>
#include <nai_common.h>

/* 
 * Generate & Print ICB CAL
 */
void nai_icb_gen_all_cal(u32,bool);
void nai_icb_gen_cal(u8,u8,u32,bool);
void nai_icb_print_cal(void);

/* 
 * Init ICB CAL
 */
void nai_icb_init_cal(void);

/* 
 * Init ICB ISerdes reset Rdy
 */
bool nai_is_icb_iserdes_in_reset(void);

#ifdef CONFIG_NAI_ICB_MST
/*
 * Disable ICB
 */
void nai_icb_disable(void);

/*
 * Enable ICB
 */
void nai_icb_enable(void);

/*
 * Get ICB Enable/Disable status
 */
bool nai_is_icb_enabled(void);

/*
 * Get Slave FW Ready status
 */
bool nai_is_slv_fw_rdy(void);

/*
 * Get Slave FPGA Ready status
 */
bool nai_is_slv_fpga_rdy(void);

/*
 * Reset Slave Zynq
 */
void nai_slv_reset(u8 mode);
#endif /*CONFIG_NAI_ICB_MST*/

#endif /*_NAI_ICB_H*/
