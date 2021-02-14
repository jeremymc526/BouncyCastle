/*
 * (C) Copyright 2014 North Atlantic Industries, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#ifndef _NAI_MISC_H
#define _NAI_MISC_H

/* 
 * output MB info to console
 */
#ifdef CONFIG_NAI_MISC_INFO
void nai_misc_info(void);
#endif

/* MB FPGA PCIE to DDR map setting*/
#ifdef CONFIG_NAI_PCIE_DDR_MAP
void nai_config_pcie_ddr_map(void);
#endif

/*MB SATA WP enable cntr*/
#ifdef CONFIG_NAI_WP_SATA_CHIP
void nai_sata_wp(bool);
#endif
/* 
 * reset MB sata disk
 */
#ifdef CONFIG_NAI_RESET_SATA_CHIP
void nai_sata_reset(void);
#endif

/* 
 * reset Ethnet Phy
 */
#ifdef CONFIG_NAI_RESET_ETH_PHY
void nai_eth_phy_reset(void);
#endif

/* 
 * Print INT2 PostCode
 */
#ifdef CONFIG_NAI_INT2_POSTCODE
void nai_print_int2_postcode(void);
#endif
	
#endif /*_NAI_MISC_H*/
