/*
 * This file is auto-generated by PetaLinux SDK 
 * DO NOT MODIFY this file, the modification will not persist
 */

#ifndef __PLNX_CONFIG_H
#define __PLNX_CONFIG_H

/* The following table includes the supported baudrates */


#define CONFIG_SYS_BAUDRATE_TABLE  { 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400 }



/* processor - ps7_cortexa9_0 */
#define CONFIG_CPU_FREQ_HZ	766666687
#define CONFIG_CLOCKS
#define CONFIG_ARM_DCC
#define CONFIG_SYS_LDSCRIPT	"arch/arm/mach-zynq/u-boot.lds"

/* main_memory - ps7_ddr_0 */

/* uart - ps7_uart_1 */
#define PSSERIAL0	"psserial0=setenv stdout ttyPS0;setenv stdin ttyPS0\0"
#define SERIAL_MULTI	"serial=setenv stdout serial;setenv stdin serial\0"
#define CONSOLE_ARG	"console=console=ttyPS0,115200\0"
#define SERIAL_MULTI  "serial=setenv stdout serial;setenv stdin serial\0"
#define CONFIG_BAUDRATE	115200

/* ethernet - ps7_ethernet_0 */
#define CONFIG_SYS_FAULT_ECHO_LINK_DOWN
#define CONFIG_MII
#define CONFIG_NET_MULTI
#define CONFIG_NETCONSOLE	1
#define CONFIG_SERVERIP	10.110.0.252
#define CONFIG_IPADDR

/* spi_flash - ps7_qspi_0 */
#define XILINX_PS7_QSPI_CLK_FREQ_HZ	200000000
#define CONFIG_SF_DEFAULT_SPEED	(XILINX_PS7_QSPI_CLK_FREQ_HZ / 4)

/* i2c - ps7_i2c_0 */
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_ZYNQ
#define CONFIG_ZYNQ_I2C0
#define CONFIG_ZYNQ_I2C1
#define CONFIG_SYS_I2C_ZYNQ_SLAVE	0
#define CONFIG_SYS_I2C_ZYNQ_SPEED 	400000

/* usb - ps7_usb_0 */
#define CONFIG_EHCI_IS_TDI
#define CONFIG_USB_CABLE_CHECK
#define CONFIG_USB_FUNCTION_THOR
#define CONFIG_THOR_RESET_OFF
#define CONFIG_SUPPORT_VFAT
#define CONFIG_SYS_DFU_DATA_BUF_SIZE 0x600000
#define DFU_DEFAULT_POLL_TIMEOUT 300

/* devcfg - ps7_dev_cfg_0 */
#define CONFIG_FPGA_ZYNQPL

/* ps7_scutimer_0 */
#define ZYNQ_SCUTIMER_BASEADDR	0xF8F00600
#define CONFIG_SYS_TIMER_COUNTS_DOWN
#define CONFIG_SYS_TIMERBASE	ZYNQ_SCUTIMER_BASEADDR
#define CONFIG_SYS_TIMER_COUNTER	(CONFIG_SYS_TIMERBASE + 0x4)

/* FPGA */

/* main_memory - ps7_ddr_0 */
#define CONFIG_SYS_SDRAM_BASE	0x0
#define CONFIG_SYS_SDRAM_SIZE	0x20000000

/* Memory testing handling */
#define CONFIG_SYS_MEMTEST_START	CONFIG_SYS_SDRAM_BASE
#define CONFIG_SYS_MEMTEST_END	(CONFIG_SYS_SDRAM_BASE + 0x1000)
#define CONFIG_SYS_LOAD_ADDR	CONFIG_SYS_SDRAM_BASE /* default load address */
#define CONFIG_NR_DRAM_BANKS	1

/* Size of malloc() pool */
#define SIZE	0xC00000
#undef CONFIG_SYS_MALLOC_LEN
#define CONFIG_SYS_MALLOC_LEN	SIZE

/* Physical Memory Map */
#define CONFIG_SYS_INIT_RAM_ADDR	0xFFFF0000
#define CONFIG_SYS_INIT_RAM_SIZE	0x2000
#define CONFIG_SYS_INIT_SP_ADDR	(CONFIG_SYS_INIT_RAM_ADDR + \
				 CONFIG_SYS_INIT_RAM_SIZE - \
				 GENERATED_GBL_DATA_SIZE)


/* BOOTP options */
#define CONFIG_BOOTP_SERVERIP
#define CONFIG_BOOTP_BOOTFILESIZE
#define CONFIG_BOOTP_BOOTPATH
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_HOSTNAME
#define CONFIG_BOOTP_MAY_FAIL
#define CONFIG_BOOTP_DNS
#define CONFIG_BOOTP_SUBNETMASK
#define CONFIG_BOOTP_PXE

/*Command line configuration.*/
#define CONFIG_CMDLINE_EDITING
#define CONFIG_AUTO_COMPLETE

/* Miscellaneous configurable options */
#define CONFIG_SYS_CBSIZE	2048/* Console I/O Buffer Size      */
#define CONFIG_SYS_PBSIZE	(CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

/* Use the HUSH parser */
#define CONFIG_SYS_PROMPT_HUSH_PS2 "> "

#define CONFIG_ENV_VARS_UBOOT_CONFIG
#define CONFIG_ENV_OVERWRITE	/* Allow to overwrite the u-boot environment variables */

#define CONFIG_LMB

/* architecture dependent code */
#define CONFIG_SYS_HZ   1000

/* Boot Argument Buffer Size */
#define CONFIG_SYS_MAXARGS      32      /* max number of command args */
#define CONFIG_SYS_LONGHELP


#undef CONFIG_BOOTM_NETBSD

/* Initial memory map for Linux */
#define CONFIG_SYS_BOOTMAPSZ 0x08000000

/* QSPI environment storage*/
#define CONFIG_ENV_OFFSET	0x520000
#define CONFIG_ENV_SIZE		0x20000
#define CONFIG_ENV_SECT_SIZE	0x20000

/* PREBOOT */
#define CONFIG_PREBOOT	"echo U-BOOT for ${hostname};setenv preboot;"

/* Board hostname */
#define CONFIG_HOSTNAME		nai-75g5-x2

/* Zynq 7000 cache line is 32 bytes */
#undef CONFIG_SYS_CACHELINE_SIZE
#define CONFIG_SYS_CACHELINE_SIZE	32

/* The highest 64k OCM address */
#define OCM_HIGH_ADDR	0xffff0000

/* Reset QSPI device on soft reboot */
#define CONFIG_NAI_32MB_QSPI_RESET_WR
#define CONFIG_NAI_32MB_QSPI_RESET_GPIO 7

/* Both MACs use a single MDIO bus */
#define CONFIG_NAI_MDIO_MUX_WR

/* NAI SERDES Functionality */
#define CONFIG_NAI_SERDES
#define CONFIG_NAI_SEND

/* EEPROM */
#define CONFIG_ZYNQ_EEPROM
#define CONFIG_ID_EEPROM
#define CONFIG_DISABLE_EEPROM_CRC
#define CONFIG_SYS_EEPROM_BUS_NUM 		0
/*#define CONFIG_NAI_READ_EEPROM_MAC_ADDR*/
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN 		1
#define CONFIG_SYS_I2C_EEPROM_ADDR 		0x50
#define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS 	4
#define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS	10
#define CONFIG_SYS_EEPROM_SIZE			2048 /* Bytes */

/* sata config */
#define CONFIG_SYS_SATA_MAX_DEVICE 1
#define CONFIG_LBA48

/* Environment settings */
#define CONFIG_CMD_ASKENV

/* Save a file over serial in s-record format. Why the hell? */
#define CONFIG_CMD_SAVES

/* Edit commandline */
#define CONFIG_CMDLINE_EDITING

/* Make the BOOTM LEN big enough for the compressed image */
#define CONFIG_SYS_BOOTM_LEN 0x1000000

/*Support for Zynq DMA Test*/
#define CONFIG_ZYNQ_DMA

/* Support legacy FIT image format */
#define CONFIG_IMAGE_FORMAT_LEGACY

/* Support legacy FIT image format */
#define CONFIG_IMAGE_FORMAT_LEGACY

/*NAI MB COMMON */
#define CONFIG_NAI_COMMON

/*NAI MB CONFIG*/
#define CONFIG_NAI_MB

/*NAI MB FSBL UBOOT VERSION CONFIG*/
#define CONFIG_VERSION_STRING_LEN		47
#define CONFIG_FSBL_VERSION_STRING		(OCM_HIGH_ADDR + 0xFD00)
#define CONFIG_UBOOT_VERSION_STRING		(CONFIG_FSBL_VERSION_STRING + CONFIG_VERSION_STRING_LEN)

/* NAI Module Feature */
#define CONFIG_NAI_MAX_MODULE_SLOT 3 //max slot: 1 ~ 6
#define CONFIG_NAI_MODULE_SUPPORT

/* Module size is power of 2 */
#define CONFIG_NAI_DEFAULT_MODULE_LO_SIZE_LIMIT		 0x4000    // 16KB
#define CONFIG_NAI_DEFAULT_MODULE_HI_SIZE_LIMIT		 0x200000  // 2MB
#define CONFIG_NAI_MAX_MODULE_MASK_SIZE_LIMIT		 0x2000000 // 32MB
/*#define CONFIG_NAI_MODULE_ID_SLV //ID module via SLAVE*/
#define CONFIG_NAI_MODULE_ID_IIC //ID module via IIC
#define CONFIG_NAI_MODULE_ARM //module arm config

/* FPGA base address */
#define PS2FPGA_BASE_ADDRESS			0x43C00000

/* NAI Module AXI I2C Feature */
#define CONFIG_SYS_I2C_ZYNQ_AXI

/* AXI i2c base address */
#define ZYNQ_AXI_I2C_BASEADDR0			0x41000000
#define ZYNQ_AXI_I2C_BASEADDR1 			0x41001000
#define ZYNQ_AXI_I2C_BASEADDR2 			0x41002000
#define ZYNQ_AXI_I2C_BASEADDR3 			0x41003000
#define ZYNQ_AXI_I2C_BASEADDR4 			0x41004000
#define ZYNQ_AXI_I2C_BASEADDR5 			0x41005000
		
#define CONFIG_NAI_MAX_AXI_I2C_PORT 		CONFIG_NAI_MAX_MODULE_SLOT
#define CONFIG_NAI_MODULE1_EEPROM_BUS_NUM 	2
#define CONFIG_NAI_MODULE2_EEPROM_BUS_NUM 	3
#define CONFIG_NAI_MODULE3_EEPROM_BUS_NUM 	4
#define CONFIG_NAI_MODULE4_EEPROM_BUS_NUM 	5
#define CONFIG_NAI_MODULE5_EEPROM_BUS_NUM 	6
#define CONFIG_NAI_MODULE6_EEPROM_BUS_NUM 	7


/* PCI Configuration */
#define CONFIG_NAI_CPCI

/* CONFIG_NAI_HPS_PAGING */
#undef CONFIG_NAI_HPS_PAGING

/* EP430 PCI IPCore */
#define CONFIG_NAI_PCI_EP430

/*#define CONFIG_NAI_BOARD_INIT_PCI_ENUM_ENABLE*/
#undef CONFIG_NAI_BOARD_INIT_PCI_ENUM_ENABLE

/* CONFIG_PCI */
#define CONFIG_PCI

#define CONFIG_PCI_PNP
#define CONFIG_CMD_PCI

/*Configure PCI Bar Size */
/*PCI bar size is power of 2 */
#define CONFIG_NAI_SETUP_CPCI_BAR0
#define CONFIG_NAI_SETUP_CPCI_BAR1
//#undef CONFIG_NAI_SETUP_CPCI_BAR1

#define CONFIG_NAI_DEFAULT_LO_CPCI_BAR0_SIZE	0x10000
#define CONFIG_NAI_DEFAULT_MAX_CPCI_BAR0_SIZE	0x4000000

#define CONFIG_NAI_DEFAULT_LO_CPCI_BAR1_SIZE	0x10000
#define CONFIG_NAI_DEFAULT_MAX_CPCI_BAR1_SIZE	0x4000000

#define CONFIG_NAI_MISC
#define CONFIG_NAI_MISC_INFO
#define CONFIG_NAI_PCIE_DDR_MAP
/*#define CONFIG_NAI_RESET_SATA_CHIP*/
#define CONFIG_NAI_RESET_ETH_PHY
#define CONFIG_NAI_FPGA_RESET_ETH_PHY

/* NAI Get MB/Module Info Command*/
#define CONFIG_CMD_NAIINFO

/* Memory test */
#define CONFIG_SYS_ALT_MEMTEST
#define CONFIG_SYS_MEMTEST_SCRATCH		OCM_HIGH_ADDR

/* SC1 module test command */
#define CONFIG_CMD_SC1

/* Temperature/voltage monitor command */
#define CONFIG_CMD_TVMON

/* QSPI Partition Layout */

/*
 * 1. Boot Header - Image Header and Partition table
 * 2. FSBL
 * 3. FPGA BIT
 * 4. U-BOOT
 * 5. U-BOOT ENV
 * 6. KERNEL
 * 7. DTB
 *8. ROOTFS
 */

#define NAI_BOOTHEADER_IMAGE_OFFSET 		"0x0"
#define NAI_BOOTHEADER_IMAGE_SIZE 		"0x20000"
 
#define NAI_FSBL_IMAGE_OFFSET 			"0x20000"
#define NAI_FSBL_IMAGE_SIZE 			"0x40000"

#define NAI_FPGA_BIT_IMAGE_OFFSET 		"0x60000"
#define NAI_FPGA_BIT_IMAGE_SIZE 		"0x400000"

#define NAI_UBOOT_IMAGE_OFFSET 			"0x460000"
#define NAI_UBOOT_IMAGE_SIZE 			"0xC0000"
/*We must change CONFIG_ENV_OFFSET and CONFIG_ENV_SIZE when 
 * we change the ENV OFFSET
 */
#define NAI_ENV_OFFSET 				"0x520000"
#define NAI_ENV_SIZE		 		"0x20000"

#define NAI_LINUX_KERNEL_IMAGE_OFFSET 		"0x540000"
#define NAI_LINUX_KERNEL_IMAGE_SIZE 		"0x500000"

#define NAI_DTB_IMAGE_OFFSET 			"0xA40000"
#define NAI_DTB_IMAGE_SIZE 			"0x20000"

#define NAI_ROOTFS_IMAGE_OFFSET 		"0xA60000"
#define NAI_ROOTFS_IMAGE_SIZE 			"0xEC0000"


/* Extra U-Boot Env settings */
#define CONFIG_EXTRA_ENV_SETTINGS \
	SERIAL_MULTI \
	CONSOLE_ARG \
	DFU_ALT_INFO_RAM \
	DFU_ALT_INFO_MMC \
	PSSERIAL0 \
	"nc=setenv stdout nc;setenv stdin nc;\0" \
	"autoload=no\0" \
	"sdbootdev=0\0" \
	"clobstart=0x10000000\0" \
	"netstart=0x10000000\0" \
	"dtbnetstart=0x02800000\0" \
	"loadaddr=0x10000000\0" \
	"cpcibar1size=0x100000\0" \
	"bootheadersize="NAI_BOOTHEADER_IMAGE_SIZE"\0" \
	"bootheaderstart="NAI_BOOTHEADER_IMAGE_OFFSET"\0" \
	"bootheader_img=boot-header.bin\0" \
	"load_bootheader=tftp ${clobstart} ${bootheader_img}\0" \
	"update_bootheader=setenv img bootheader; setenv psize ${bootheadersize}; " \
		"setenv installcmd \"install_bootheader\"; run load_bootheader test_img; " \
		"setenv img; setenv psize; setenv installcmd\0" \
	"install_bootheader=sf probe 0 && sf erase ${bootheaderstart} ${bootheadersize} && " \
		"sf write ${clobstart} ${bootheaderstart} ${filesize} \0" \
	"fsblsize="NAI_FSBL_IMAGE_SIZE"\0" \
	"fsblstart="NAI_FSBL_IMAGE_OFFSET"\0" \
	"fsbl_img=zynq_fsbl.bin\0" \
	"load_fsbl=tftp ${clobstart} ${fsbl_img}\0" \
	"update_fsbl=setenv img fsbl; setenv psize ${fsblsize}; setenv installcmd \"install_fsbl\"; " \
		"run load_fsbl test_img; setenv img; setenv psize; setenv installcmd\0" \
	"install_fsbl=sf probe 0 && sf erase ${fsblstart} ${fsblsize} && " \
		"sf write ${clobstart} ${fsblstart} ${filesize} \0" \
	"fpgasize="NAI_FPGA_BIT_IMAGE_SIZE"\0" \
	"fpgastart="NAI_FPGA_BIT_IMAGE_OFFSET"\0" \
	"fpga_img=mbfpga.bit.bin\0" \
	"load_fpga=tftp ${clobstart} ${fpga_img}\0" \
	"update_fpga=setenv img fpga; setenv psize ${fpgasize}; setenv installcmd \"install_fpga\"; " \
		"run load_fpga test_img; setenv img; setenv psize; setenv installcmd\0" \
	"install_fpga=sf probe 0 && sf erase ${fpgastart} ${fpgasize} && " \
		"sf write ${clobstart} ${fpgastart} ${filesize} \0 " \
	"ubootsize="NAI_UBOOT_IMAGE_SIZE"\0" \
	"ubootstart="NAI_UBOOT_IMAGE_OFFSET"\0" \
	"uboot_img=u-boot.bin\0" \
	"load_uboot=tftp ${clobstart} ${uboot_img}\0" \
	"update_uboot=setenv img uboot; setenv psize ${ubootsize}; setenv installcmd \"install_uboot\"; " \
		"run load_uboot test_img; setenv img; setenv psize; setenv installcmd\0" \
	"install_uboot=sf probe 0 && sf erase ${ubootstart} ${ubootsize} && " \
		"sf write ${clobstart} ${ubootstart} ${filesize} && " \
		"run eraseenv \0" \
	"bootenvsize="NAI_ENV_SIZE"\0" \
	"bootenvstart="NAI_ENV_OFFSET"\0" \
	"eraseenv=sf probe 0 && sf erase ${bootenvstart} ${bootenvsize}\0" \
	"kernelsize="NAI_LINUX_KERNEL_IMAGE_SIZE"\0" \
	"kernelstart="NAI_LINUX_KERNEL_IMAGE_OFFSET"\0" \
	"kernel_img=zImage\0" \
	"load_kernel=tftp ${clobstart} ${kernel_img}\0" \
	"update_kernel=setenv img kernel; setenv psize ${kernelsize}; setenv installcmd \"install_kernel\"; " \
		"run load_kernel test_img; setenv img; setenv psize; setenv installcmd\0" \
	"install_kernel=sf probe 0 && sf erase ${kernelstart} ${kernelsize} && " \
		"sf write ${clobstart} ${kernelstart} ${filesize}\0" \
	"dtbsize="NAI_DTB_IMAGE_SIZE"\0" \
	"dtbstart="NAI_DTB_IMAGE_OFFSET"\0" \
	"dtb_img=system.dtb\0" \
	"load_dtb=tftp ${clobstart} ${dtb_img}\0" \
	"update_dtb=setenv img dtb; setenv psize ${dtbsize}; setenv installcmd \"install_dtb\"; " \
		"run load_dtb test_img; setenv img; setenv psize; setenv installcmd\0" \
	"install_dtb=sf probe 0 && sf erase ${dtbstart} ${dtbsize} && " \
		"sf write ${clobstart} ${dtbstart} ${filesize}\0" \
	"rootfssize="NAI_ROOTFS_IMAGE_SIZE"\0" \
	"rootfsstart="NAI_ROOTFS_IMAGE_OFFSET"\0" \
	"rootfs_img=urootfs.cpio.gz\0" \
	"load_rootfs=tftp ${clobstart} ${rootfs_img}\0" \
	"update_rootfs=setenv img rootfs; setenv psize ${rootfssize}; setenv installcmd \"install_rootfs\"; " \
		"run load_rootfs test_crc; setenv img; setenv psize; setenv installcmd\0" \
	"install_rootfs=sf probe 0 && sf erase ${rootfsstart} ${rootfssize} && " \
		"sf write ${clobstart} ${rootfsstart} ${rootfssize}\0" \
	"update_all= run update_fsbl && " \
				"run update_fpga && " \
				"run update_uboot && " \
				"run update_kernel && " \
				"run update_dtb && " \
				"run update_rootfs \0" \
	"cp_kernel2ram=sf probe 0 && sf read ${netstart} ${kernelstart} ${kernelsize}\0" \
	"sdboot=echo boot Petalinux; mmcinfo && fatload mmc 0 ${netstart} ${kernel_img} && bootm \0" \
	"fault=echo ${img} image size is greater than allocated place - partition ${img} is NOT UPDATED\0" \
	"test_crc=if imi ${clobstart}; then run test_img; else echo ${img} Bad CRC - ${img} is NOT UPDATED; fi\0" \
	"test_img=setenv var \"if test ${filesize} -gt ${psize}\\; then run fault\\; " \
		"else run ${installcmd}\\; fi\"; run var; setenv var\0" \
	"kernelbootstart=0x1000000\0" \
	"dtbbootstart=0x1A00000\0" \
	"rootfsbootstart=0x2000000\0" \
	"sataroot=/dev/sda1 rw rootflags=data=ordered \0" \
	"ramdiskroot=/dev/ram\0" \
	"defaultbootargs=console=ttyPS0,115200\0" \
	"chk_mod=echo check module status; modrdy\0" \
	"sata_boot=run chk_mod && echo SATA BOOT && " \
		"sf probe 0 && sf read ${kernelbootstart} ${kernelstart} ${kernelsize} && " \
		"sf read ${dtbbootstart} ${dtbstart} ${dtbsize} && " \
		"setenv bootargs ${defaultbootargs} root=${sataroot}&& " \
		"bootz ${kernelbootstart} - ${dtbbootstart} \0" \
	"qspi_boot=run chk_mod  && echo QSPI BOOT && " \
		"sf probe 0 && sf read ${kernelbootstart} ${kernelstart} ${kernelsize} && " \
		"sf read ${dtbbootstart} ${dtbstart} ${dtbsize} && " \
		"sf read ${rootfsbootstart} ${rootfsstart} ${rootfssize} && " \
		"setenv bootargs ${defaultbootargs} root=${ramdiskroot} rw && " \
		"bootz ${kernelbootstart} ${rootfsbootstart} ${dtbbootstart} \0" \
	"netboot=tftp ${kernelbootstart} ${kernel_img} && " \
		"tftp ${dtbbootstart} ${dtb_img} && " \
		"tftp ${rootfsbootstart} ${rootfs_img} && " \
		"setenv bootargs ${defaultbootargs} root=${ramdiskroot} rw && " \
		"bootz ${kernelbootstart} ${rootfsbootstart} ${dtbbootstart} \0" \
	"default_bootcmd=run qspi_boot\0" \
	""

/* BOOTCOMMAND */
#undef CONFIG_BOOTCOMMAND
#define CONFIG_BOOTCOMMAND	"run default_bootcmd"

#endif /* __PLNX_CONFIG_H */
