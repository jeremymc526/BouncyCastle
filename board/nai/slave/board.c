/*
 * (C) Copyright 2012 Michal Simek <monstr@monstr.eu>
 * (C) Copyright 2014 North Atlantic Industries, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <fpga.h>
#include <zynqpl.h>
#include <asm/arch/hardware.h>
#include <asm/arch/sys_proto.h>
#include <version.h>
#include <nai_icb.h>

DECLARE_GLOBAL_DATA_PTR;

#define SLAVE_MEM_BASE		0x40000000
#define FPGA_BUILD_TIME		(SLAVE_MEM_BASE + 0x1000)
#define FPGA_REVISION		(SLAVE_MEM_BASE + 0x1004)
#define SLAVE_READY_STATE	(SLAVE_MEM_BASE + 0x100C)
#define SLAVE_FW_READY_BIT_MASK	0x00000001
#define SLAVE_FPGA_READY_BIT_MASK	0x00000002

	
#if (defined(CONFIG_FPGA) && !defined(CONFIG_SPL_BUILD)) || \
    (defined(CONFIG_SPL_FPGA_SUPPORT) && defined(CONFIG_SPL_BUILD))
static xilinx_desc fpga = XILINX_XC7Z015_DESC(0x15);
#endif

#if !defined(CONFIG_SPL_BUILD)
const char __weak uboot_version_string[CONFIG_VERSION_STRING_LEN] = PLAIN_VERSION ".00." U_BOOT_DATE " at " U_BOOT_TIME "." NAI_UBOOT_VER;
#endif

int board_init(void)
{
#if !defined(CONFIG_SPL_BUILD)
	/* Copy version string to a known location in OCRAM */
	strncpy((char *)CONFIG_UBOOT_VERSION_STRING, uboot_version_string, CONFIG_VERSION_STRING_LEN);
#endif

#if (defined(CONFIG_FPGA) && !defined(CONFIG_SPL_BUILD)) || \
    (defined(CONFIG_SPL_FPGA_SUPPORT) && defined(CONFIG_SPL_BUILD))
	fpga_init();
	fpga_add(fpga_xilinx, &fpga);
#endif
	return 0;
}

int board_late_init(void)
{
	u32 boot_mode = zynq_slcr_get_boot_mode() & ZYNQ_BM_MASK;
	u32 f_rev = readl(FPGA_REVISION);
	u32 f_build_time = readl(FPGA_BUILD_TIME);
	u32 f_day, f_mon, f_yr, f_hr, f_min, f_sec;
	
	switch (boot_mode) {
	case ZYNQ_BM_QSPI:
		env_set"modeboot", "qspiboot");
		break;
	default:
		printf("Unsupported boot mode (%d)\n", boot_mode);
		env_set("modeboot", "");
		break;
	}

	//set slave fpga ready
	writel(SLAVE_FPGA_READY_BIT_MASK, SLAVE_READY_STATE);
		
	/*if ICB ISERDED is NOT in reset
	 * then init ICB cal value
	*/
	if(!nai_is_icb_iserdes_in_reset()){
		nai_icb_init_cal();
	}

	// Print FPGA revision and build time
	f_sec = f_build_time & 0x3f;			// Bits 0-5:   Seconds (0 - 59)
	f_min = (f_build_time >> 6) & 0x3f;		// Bits 6-11:  Minutes (0 - 59)
	f_hr  = (f_build_time >> 12) & 0x1f;	// Bits 12-16: Hours (0 - 23)
	f_yr  = (f_build_time >> 17) & 0x3f;	// Bits 17-22: Year (0 - 63)
	f_mon = (f_build_time >> 23) & 0xf;		// Bits 23-26: Month (1 - 12)
	f_day = f_build_time >> 27;				// Bits 27-31: Day (1 - 31)

	printf("FPGA Revision:   %X\n", f_rev);
	printf("FPGA Build Time: %02d/%02d/%04d at %02d:%02d:%02d\n",
		f_mon, f_day, 2000 + f_yr, f_hr, f_min, f_sec);

	return 0;
}

int dram_init(void)
{
	gd->ram_size = CONFIG_SYS_SDRAM_SIZE;

	return 0;
}
