/*
 * (C) Copyright 2012 Michal Simek <monstr@monstr.eu>
 * (C) Copyright 2014 North Atlantic Industries, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <fdtdec.h>
#include <fpga.h>
#include <mmc.h>
#include <zynqpl.h>
#include <asm/arch/hardware.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/ps7_init_gpl.h>
#include <version.h>
#if defined(CONFIG_NAI_MISC)
#include <nai_misc.h>
#endif
#if defined(CONFIG_NAI_MB)
#include <nai_mb.h>
#endif
#if defined(CONFIG_NAI_MODULE_SUPPORT)
#include <nai_module_ext.h>
#endif
#ifdef CONFIG_NAI_ICB
#include <nai_icb.h>
#endif
#if defined(CONFIG_NAI_CPCI)
#include <nai_pci.h>
#endif /* CONFIG_NAI_CPCI*/

#ifdef CONFIG_NAI_VME
#include <nai_vme.h>
#endif /* CONFIG_NAI_VME*/

#if defined(CONFIG_NAI_PCI_EP430)
#include <pci_ep430.h>
#include <pci.h>
#endif /* CONFIG_NAI_PCI_EP430*/

#if defined(CONFIG_PCI_XILINX)
#include <dm/uclass.h>
#endif

#if CONFIG_IS_ENABLED(AHCI)
#include <iprop_ahsata.h>
#include <ahci.h>
#include <dm/device.h>
#include <dm/device-internal.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

#if (defined(CONFIG_FPGA) && !defined(CONFIG_SPL_BUILD)) || \
    (defined(CONFIG_SPL_FPGA_SUPPORT) && defined(CONFIG_SPL_BUILD))
static xilinx_desc fpga;

/* It can be done differently */
xilinx_desc fpga010 = XILINX_XC7Z010_DESC(0x10);
xilinx_desc fpga015 = XILINX_XC7Z015_DESC(0x15);
xilinx_desc fpga020 = XILINX_XC7Z020_DESC(0x20);
xilinx_desc fpga030 = XILINX_XC7Z030_DESC(0x30);
xilinx_desc fpga045 = XILINX_XC7Z045_DESC(0x45);
xilinx_desc fpga100 = XILINX_XC7Z100_DESC(0x100);
#endif

#if defined(CONFIG_NAI_PCI_EP430)
static struct pci_controller hose;
#elif defined(CONFIG_PCI_XILINX)	 
static struct udevice *udev;
#endif

#if !defined(CONFIG_SPL_BUILD)
const char __weak uboot_version_string[CONFIG_VERSION_STRING_LEN] = PLAIN_VERSION ".00." U_BOOT_DATE " at " U_BOOT_TIME "." NAI_UBOOT_VER;
#endif

#if defined(CONFIG_NAI_PCI_EP430)
void pci_init_board(void)
{
#if defined(CONFIG_NAI_BOARD_INIT_PCI_ENUM_ENABLE)
	printf("PCI Enum \n");
	
	pci_ep430_init(&hose);
#endif /* CONFIG_NAI_BOARD_INIT_PCI_ENUM_ENABLE */
}
#endif /* CONFIG_NAI_PCI_EP430 */

#if defined(CONFIG_PCI_XILINX)
static void pci_xilinx_enum(void)
{
#if defined(CONFIG_NAI_BOARD_INIT_PCI_ENUM_ENABLE)
	int ret;
	
	printf("PCI Xilinx Enum \n");
	ret = uclass_get_device(UCLASS_PCI, 0, &udev);
	if (ret)
		printf("Failed to get Xilinx PCIe DM device, err=%d\n", ret);
#endif /* CONFIG_NAI_BOARD_INIT_PCI_ENUM_ENABLE */
}
#endif /* CONFIG_PCI_XILINX */

int board_init(void)
{
#if defined(CONFIG_ENV_IS_IN_EEPROM) && !defined(CONFIG_SPL_BUILD)
	unsigned char eepromsel = CONFIG_SYS_I2C_MUX_EEPROM_SEL;
#endif
#if (defined(CONFIG_FPGA) && !defined(CONFIG_SPL_BUILD)) || \
    (defined(CONFIG_SPL_FPGA_SUPPORT) && defined(CONFIG_SPL_BUILD))
	u32 idcode;

	idcode = zynq_slcr_get_idcode();

	switch (idcode) {
	case XILINX_ZYNQ_7010:
		fpga = fpga010;
		break;
	case XILINX_ZYNQ_7015:
		fpga = fpga015;
		break;
	case XILINX_ZYNQ_7020:
		fpga = fpga020;
		break;
	case XILINX_ZYNQ_7030:
		fpga = fpga030;
		break;
	case XILINX_ZYNQ_7045:
		fpga = fpga045;
		break;
	case XILINX_ZYNQ_7100:
		fpga = fpga100;
		break;
	}
#endif

#if !defined(CONFIG_SPL_BUILD)
	/* Copy version string to a known location in OCRAM */
	strncpy((char *)CONFIG_UBOOT_VERSION_STRING, uboot_version_string, CONFIG_VERSION_STRING_LEN);
#endif
	
	/*
	 * NAI: Changed the temporary workaround of clearing pending IRQs to UART0 (e0000000)
	 * Xilinx reference board is using UART1 (e0001000), NAI target board is using UART1
	 */
	 
	/* temporary hack to clear pending irqs before Linux as it
	 * will hang Linux
	 */
	writel(0x26d, 0xe0000014);

#if (defined(CONFIG_FPGA) && !defined(CONFIG_SPL_BUILD)) || \
    (defined(CONFIG_SPL_FPGA_SUPPORT) && defined(CONFIG_SPL_BUILD))
	fpga_init();
	fpga_add(fpga_xilinx, &fpga);
#endif
#if defined(CONFIG_ENV_IS_IN_EEPROM) && !defined(CONFIG_SPL_BUILD)
	if (eeprom_write(CONFIG_SYS_I2C_MUX_ADDR, 0, &eepromsel, 1))
		puts("I2C:EEPROM selection failed\n");
#endif
	return 0;
}

int board_late_init(void)
{
	switch ((zynq_slcr_get_boot_mode()) & ZYNQ_BM_MASK) {
	case ZYNQ_BM_QSPI:
		env_set("modeboot", "qspiboot");
		break;
	case ZYNQ_BM_NAND:
		env_set("modeboot", "nandboot");
		break;
	case ZYNQ_BM_NOR:
		env_set("modeboot", "norboot");
		break;
	case ZYNQ_BM_SD:
		env_set("modeboot", "sdboot");
		break;
	case ZYNQ_BM_JTAG:
		env_set("modeboot", "jtagboot");
		break;
	default:
		env_set("modeboot", "");
		break;
	}

#if defined(CONFIG_PCI_XILINX)
	pci_xilinx_enum();
#endif

#ifdef CONFIG_NAI_INT2_POSTCODE
	nai_print_int2_postcode();
#endif

/* 
 * Move slave reset to FSBL
 * to reduce power on delay
 */
//#ifdef CONFIG_NAI_ICB_MST
//  /*held slave zynq in reset*/
//  nai_slv_reset(0);
//  /*release slave zynq out of reset*/
//  nai_slv_reset(1);
//#endif

#ifdef CONFIG_NAI_MODULE_SUPPORT
	/* Enable All Available Module Boards */
	nai_init_module();

#ifdef CONFIG_NAI_VME
	/*TODO: VME Setup delay
	 * VME Setup needs to wait for 
	 * slave ICB calibration and FW ready
	 * NEED HW Team's help to solve this delay
	 * with setup 64ARM1 VME*/
	vme_init();
#endif

#endif /* CONFIG_NAI_MODULE_SUPPORT */

#ifdef CONFIG_NAI_MB
	/* Update MB versions to MB common memory */
	nai_update_mb_version();
#endif

#ifdef CONFIG_NAI_MISC
	#ifdef CONFIG_NAI_MISC_INFO
		//output MB info to console
		nai_misc_info();
	#endif
#endif /* CONFIG_NAI_MISC */


#ifdef CONFIG_NAI_MISC	
	#ifdef CONFIG_NAI_RESET_ETH_PHY
		//reset eth phy
		nai_eth_phy_reset();
	#endif

        #ifdef CONFIG_NAI_PCIE_DDR_MAP
		nai_config_pcie_ddr_map();
        #endif	
#endif /* CONFIG_NAI_MISC */

/* Read Etherent MAC from EEPROM and set the MAC addr in environment  */
#ifdef CONFIG_NAI_READ_EEPROM_MAC_ADDR
	mac_read_from_eeprom();
#endif
	
#ifdef CONFIG_NAI_CPCI
	/* Need to wait for all boards to report BAR information*/
	mdelay(1000);
#endif
	return 0;
}

#ifdef CONFIG_DISPLAY_BOARDINFO
int checkboard(void)
{
	u32 version = zynq_get_silicon_version();

	version <<= 1;
	if (version > (PCW_SILICON_VERSION_3 << 1))
		version += 1;

	puts("Board: Xilinx Zynq\n");
	printf("Silicon: v%d.%d\n", version >> 1, version & 1);

	return 0;
}
#endif

#if !defined(CONFIG_SYS_SDRAM_BASE) && !defined(CONFIG_SYS_SDRAM_SIZE)
int dram_init_banksize(void)
{
	return fdtdec_setup_memory_banksize();
}

int dram_init(void)
{
	if (fdtdec_setup_memory_size() != 0)
		return -EINVAL;

	zynq_ddrc_init();

	return 0;
}
#else
int dram_init(void)
{
	gd->ram_size = CONFIG_SYS_SDRAM_SIZE;

	zynq_ddrc_init();

	return 0;
}
#endif

#if CONFIG_IS_ENABLED(AHCI)
static int sata_mb_probe(struct udevice *dev)
{
	int err;

#ifdef CONFIG_NAI_RESET_SATA_CHIP	
	/* reset sata disk */
	nai_sata_reset();
#endif
#ifdef CONFIG_NAI_WP_SATA_CHIP
	nai_sata_wp(true);
#endif
	err = iprop_ahsata_probe(dev);
	if (err) {
		if (sata_dm_port_status(0, 0) != 0)
			/* There's a device, but link not established */
			device_remove(dev, DM_REMOVE_NORMAL);
	}

	return 0;
}

static int sata_mb_remove(struct udevice *dev)
{
	return 0;
}

struct ahci_ops sata_mb_ops = {
	.port_status = iprop_ahsata_port_status,
	.reset	= iprop_ahsata_bus_reset,
	.scan	= iprop_ahsata_scan,
};

static const struct udevice_id sata_mb_ids[] = {
	{ .compatible = "nai,naimb-ahci" },
	{ }
};

U_BOOT_DRIVER(sata_mb) = {
	.name		= "iprop_ahci",
	.id		= UCLASS_AHCI,
	.of_match	= sata_mb_ids,
	.ops		= &sata_mb_ops,
	.probe		= sata_mb_probe,
	.remove		= sata_mb_remove,  /* reset bus to stop it */
};
#endif /* AHCI */

