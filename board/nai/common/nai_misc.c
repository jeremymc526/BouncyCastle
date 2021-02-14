/*
 * North Atlantic Industries
 * tyang@naii.com
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
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

#include <common.h>
#include <asm/io.h>
#include <nai_mb_fpga_address.h>
#include <nai_common.h>
#include <nai_mb.h>
#include <nai_misc.h>

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define DEBUGF(x...) printf(x)
#else
#define DEBUGF(x...)
#endif /* DEBUG */

#ifdef CONFIG_NAI_RESET_SATA_CHIP
#define RESET_DURATION_5MS	0x01
#define RESET_DURATION_10MS	0x02
#define RESET_DURATION_15MS	0x03
#define RESET_DURATION_20MS	0x04
#define SATA_INTERNAL_RESET_BIT 0x04  /*BIT 2*/
#endif /*CONFIG_NAI_RESET_SATA_CHIP*/

#ifdef CONFIG_NAI_FPGA_RESET_ETH_PHY
#define RESET_DURATION_5MS	0x01
#define RESET_DURATION_10MS	0x02
#define RESET_DURATION_15MS	0x03
#define RESET_DURATION_20MS	0x04
#define ETH_PHY_RESET_BIT 	0x10000 /*BIT 16*/
#endif /*CONFIG_NAI_FPGA_RESET_ETH_PHY*/

#ifdef CONFIG_NAI_INT2_POSTCODE
static bool _is_postcode_enable(void);
static int _get_postcode_poll_time(void);
static uchar _get_postcode(void);
static void _print_postcode(uchar);

struct postcode_params {
	const char *name;
	u8 code;
	const char *desc;
};

const struct postcode_params postcode_table[] = {
	/*SEC Phase*/
	{"SEC_SYSTEM_POWER_ON", 0x01, ""},
	{"SEC_BEFORE_MICROCODE_PATCH", 0x02, ""},
	{"SEC_AFTER_MICROCODE_PATCH", 0x03, ""},
	{"SEC_ACCESS_CSR", 0x04, ""},
	{"SEC_GENERIC_MSRINIT", 0x05, ""},
	{"SEC_CPU_SPEEDCFG", 0x06, ""},
	{"SEC_SETUP_CAR_OK", 0x07, ""},
	{"SEC_FORCE_MAX_RATIO", 0x08, ""},
	{"SEC_GO_TO_SECSTARTUP", 0x09, ""},
	{"SEC_GO_TO_PEICORE", 0x0A, ""},
	/*PEI Phase*/
	{"PEI_SIO_INIT",0x70,""},
	{"PEI_CPU_REG_INIT",0x71,""},
	{"PEI_CPU_AP_INIT",0x72,""},
	{"PEI_CPU_HT_RESET",0x73,""},
	{"PEI_PCIE_MMIO_INIT",0x74,""},
	{"PEI_NB_REG_INIT",0x75,""},
	{"PEI_SB_REG_INIT",0x76,""},
	{"PEI_PCIE_TRAINING",0x77,""},
	{"PEI_TPM_INIT",0x78,""},
	{"PEI_SMBUS_INIT",0x79,""},
	{"PEI_PROGRAM_CLOCK_GEN",0x7A,""},
	{"PEI_IGD_EARLY_INITIAL",0x7B,""},
	{"PEI_WATCHDOG_INIT",0x7D,""},
	{"PEI_MEMORY_INIT",0x7E,""}	,
	{"PEI_MEMORY_INIT_FOR_CRISIS",0x7F,""},
	{"PEI_MEMORY_INSTALL",0x80,""},
	{"PEI_TXTPEI",0x81,""},
	{"PEI_SWITCH_STACK",0x82,""},
	{"PEI_MEMORY_CALLBACK",0x83,""},
	{"PEI_ENTER_RECOVERY_MODE",0x84,""},
	{"PEI_RECOVERY_MEDIA_FOUND",0x85,""},
	{"PEI_RECOVERY_MEDIA_NOT_FOUND",0x86,""},
	{"PEI_RECOVERY_LOAD_FILE_DONE",0x87,""},
	{"PEI_RECOVERY_START_FLASH",0x88,""},
	{"PEI_FINDING_DXE_CORE",0x8A,""},
	{"PEI_GO_TO_DXE_CORE",0x8B,""},
	/*DXE Phase*/
	{"DXE_TCGDXE",0x40,""},
	{"DXE_SB_SPI_INIT",0x41,""},
	{"DXE_CF9_RESET",0x42,""},
	{"DXE_SB_SERIAL_GPIO_INIT",0x43,""},
	{"DXE_NB_INIT",0x45,""},
	{"DXE_SIO_INIT",0x46,""},
	{"DXE_LEGACY_REGION",0x47,""},
	{"DXE_SB_INIT",0x48,""},
	{"DXE_IDENTIFY_FLASH_DEVICE",0x49,""},
	{"DXE_FTW_INIT",0x4A,""},
	{"DXE_VARIABLE_INIT",0x4B,""},
	{"DXE_VARIABLE_INIT_FAIL",0x4C,""},
	{"DXE_MTC_INIT",0x4D,""},
	{"DXE_CPU_INIT",0x4E,""},
	{"DXE_MP_CPU_INIT",0x4F,""},
	{"DXE_SMART_TIMER_INIT",0x51,""},
	{"DXE_PCRTC_INIT",0x52,""},
	{"DXE_SATA_INIT",0x53,""},
	{"DXE_SMM_CONTROLER_INIT",0x54,""},
	{"DXE_LEGACY_INTERRUPT",0x55,""},
	{"DXE_RELOCATE_SMBASE",0x56,""},
	{"DXE_FIRST_SMI",0x57,""},
	{"DXE_VTD_INIT",0x58,""},
	{"DXE_BEFORE_CSM16_INIT",0x59,""},
	{"DXE_AFTER_CSM16_INIT",0x5A,""},
	{"DXE_SB_DISPATCH",0x5C,""},
	{"DXE_SB_IOTRAP_INIT",0x5D,""},
	{"DXE_SUBCLASS_DRIVER",0x5E,""},
	{"DXE_PPM_INIT",0x5F,""},
	{"DXE_HECIDRV_INIT",0x60,""},
	{"DXE_FLASH_PART_NONSUPPORT",0x62,""},
	/*BDS Phase*/
	{"BDS_ENTER_BDS",0x10,""},
	{"BDS_INSTALL_HOTKEY",0x11,""},
	{"BDS_ASF_INIT",0x12,""},
	{"BDS_PCI_ENUMERATION_START",0x13,""},
	{"BDS_BEFORE_PCIIO_INSTALL",0x14,""},
	{"BDS_PCI_ENUMERATION_END",0x15,""},
	{"BDS_CONNECT_CONSOLE_IN",0x16,""},
	{"BDS_CONNECT_STD_ERR",0x18,""},
	{"BDS_CONNECT_USB_HC",0x19,""},
	{"BDS_CONNECT_USB_BUS",0x1A,""},
	{"BDS_CONNECT_USB_DEVICE",0x1B,""},
	{"BDS_NO_CONSOLE_ACTION",0x1C,""},
	{"BDS_DISPLAY_LOGO_SYSTEM_INFO",0x1D,""},
	{"BDS_START_SATA_CONTROLLER",0x1F,""},
	{"BDS_START_ISA_ACPI_CONTROLLER",0x20,""},
	{"BDS_START_ISA_BUS",0x21,""},
	{"BDS_START_ISA_FDD",0x22,""},
	{"BDS_START_ISA_SEIRAL",0x23,""},
	{"BDS_START_IDE_BUS",0x24,""},
	{"BDS_START_AHCI_BUS",0x25,""},
	{"BDS_CONNECT_LEGACY_ROM",0x26,""},
	{"BDS_ENUMERATE_ALL_BOOT_OPTION",0x27,""},
	{"BDS_END_OF_BOOT_SELECTION",0x28,""},
	{"BDS_ENTER_SETUP",0x29,""},
	{"BDS_ENTER_BOOT_MANAGER",0x2A,""},
	{"BDS_BOOT_DEVICE_SELECT",0x2B,""},
	{"BDS_EFI64_SHADOW_ALL_LEGACY_ROM",0x2C,""},
	{"BDS_ACPI_S3SAVE",0x2D,""},
	{"BDS_GO_UEFI_BOOT",0x30,""},
	{"BDS_LEGACY16_PREPARE_TO_BOOT",0x31,""},
	{"BDS_EXIT_BOOT_SERVICES",0x32,""},
	{"BDS_RECOVERY_START_FLASH",0x35,""},
	{"BDS_START_SDHC_BUS",0x36,""},
	{"BDS_CONNECT_ATA_LEGACY",0x37,""},
	{"BDS_CONNECT_SD_LEGACY",0x38,""},
	/*PostBDS*/
	{"POST_BDS_NO_BOOT_DEVICE ",0xF9,""},
	{"POST_BDS_START_IMAGE ",0xFB,""},
	{"POST_BDS_ENTER_INT19 ",0xFD,""},
	{"POST_BDS_JUMP_BOOT_SECTOR ",0xFE,""},
	/*S3*/
	{"S3_RESTORE_MEMORY_CONTROLLER",0xC0,""},
	{"S3_INSTALL_S3_MEMORY",0xC1,""},
	{"S3_SWITCH_STACK",0xC2,""},
	{"S3_MEMORY_CALLBACK",0xC3,""},
	{"S3_ENTER_S3_RESUME_PEIM",0xC4,""},
	{"S3_BEFORE_ACPI_BOOT_SCRIPT",0xC5,""},
	{"S3_BEFORE_RUNTIME_BOOT_SCRIPT",0xC6,""},
	{"S3_BEFORE_RELOCATE_SMM_BASE",0xC7,""},
	{"S3_BEFORE_MP_INIT",0xC8,""},
	{"S3_BEFORE_RESTORE_ACPI_CALLBACK",0xC9,""},
	{"S3_AFTER_RESTORE_ACPI_CALLBACK",0xCA,""},
	{"S3_GO_TO_FACS_WAKING_VECTOR",0xCB,""},
	/*ASL*/
	{"ASL_ENTER_S1",0x51,""},
	{"ASL_ENTER_S3",0x53,""},
	{"ASL_ENTER_S4",0x54,""},
	{"ASL_ENTER_S5",0x55,""},
	{"ASL_WAKEUP_S1",0xE1,""},
	{"ASL_WAKEUP_S3",0xE3,""},
	{"ASL_WAKEUP_S4",0xE4,""},
	/*SMM*/
	{"SMM_IDENTIFY_FLASH_DEVICE",0xA0,""},
	{"SMM_SMM_PLATFORM_INIT",0xA2,""},
	{"SMM_ACPI_ENABLE_START",0xA6,""},
	{"SMM_ACPI_ENABLE_END",0xA7,""},
	{"SMM_S1_SLEEP_CALLBACK",0xA1,""},
	{"SMM_S3_SLEEP_CALLBACK",0xA3,""},
	{"SMM_S4_SLEEP_CALLBACK",0xA4,""},
	{"SMM_S5_SLEEP_CALLBACK",0xA5,""},
	{"SMM_ACPI_DISABLE_START",0xA8,""},
	{"SMM_ACPI_DISABLE_END",0xA9,""},
	/*InsydeH20DDT*/
	{"INSYDE_WAIT_DEVICE_CON",0x0D,""},
	{"INSYDE_WAIT_DEVICE_CON",0xD0,""},
	{"INSYDE_DDT_RDY",0xD1,""},
	{"INSYDE_EHCI_NOT_FOUND",0xD2,""},
	{"INSYDE_DBG_PORT_LOW_SPEED_DEV",0xD3,""},
	{"INSYDE_DDT_CABLE_LOW_SPEED_DEV",0xD4,""},
	{"INSYDE_DDT_TX_ERROR",0xD5,""},
	{"INSYDE_DDT_TX_ERROR",0xD6,""},
	{"INSYDE_DDT_TX_ERROR",0xD7,""},
};
#endif /*CONFIG_NAI_INT2_POSTCODE*/

#ifdef CONFIG_NAI_PCIE_DDR_MAP
void nai_config_pcie_ddr_map(){

   u32 msk = CONFIG_SYS_SDRAM_SIZE - 1;

   /*ddr start address offset*/
   writel(0, PS2FPGA_PCIE_DDR_MAP);
   /*ddr mask address offset*/
   writel(msk, PS2FPGA_PCIE_DDR_MASK);

}
#endif 

#ifdef CONFIG_NAI_RESET_SATA_CHIP
void nai_sata_reset(){
	
	DEBUGF("%s: \n", __func__);
	printf("Reset SATA Chip \n");
	u32 sataResetBit = SATA_INTERNAL_RESET_BIT;
	
	//configure reset duration to 5ms
	writel(RESET_DURATION_5MS, PS2FPGA_RESET_DURATION_OFFSET);
	writel(sataResetBit, PS2FPGA_RESET_OFFSET);
}
#endif

#ifdef CONFIG_NAI_WP_SATA_CHIP
void nai_sata_wp(bool enable){
	
	if(enable){
		printf("Enable SATA WP \n");
		writel(0x1, PS2FPGA_SATA_WP_OFFSET);
	}else{
		printf("Disable SATA WP \n");
		writel(0x0, PS2FPGA_SATA_WP_OFFSET);
	}
}
#endif

#ifdef CONFIG_NAI_RESET_ETH_PHY
void nai_eth_phy_reset(){
	
	DEBUGF("%s: \n", __func__);
	printf("Reset ETH Phy \n");
	u32 data = 0;
	
	//The Eth Phy reset line is control by GPIO_51 or MB FPGA Reset register
//ETH PHY FPGA Reset
#ifdef CONFIG_NAI_FPGA_RESET_ETH_PHY
	printf("FPGA Reset ETH Phy \n");
	data = RESET_DURATION_5MS;
	writel(data, PS2FPGA_RESET_DURATION_OFFSET);
	data = ETH_PHY_RESET_BIT;
	writel(data, PS2FPGA_RESET_OFFSET);

#else 
	#ifdef CONFIG_NAI_GPIO_RESET_ETH_PHY
		printf("GPIO Reset ETH Phy \n");
		//ETH PHY GPIO Reset	
		//Set gpio_51 to output
		data = readl(0xe000a244); 
		data |= (1 << 19);
		writel(data, 0xe000a244);
		
		//enable gpio_51 to output
		data = readl(0xe000a248); 
		data |= (1 << 19);
		writel(data, 0xe000a248);
		
		//set gpio output to low
		data = readl(0xe000a044); 
		data &= ~(1 << 19);
		writel(data, 0xe000a044);
		
		//wait 5 ms
		udelay(5000);
		
		//set gpio output to high
		data = readl(0xe000a044); 
		data |= (1 << 19);
		writel(data, 0xe000a044);
	#endif
#endif


}
#endif

#ifdef CONFIG_NAI_MISC_INFO
void nai_misc_info()
{
	DEBUGF("%s: \n", __func__);
	Nai_mb mb;
	
	//Display MB FPGA Revision
	nai_get_mb_info(&mb);
	printf("FPGA Rev: %05d.%05d \n",(mb.masterVer.fpgaRev >> 16 & 0xffff), (mb.masterVer.fpgaRev & 0xffff));
	printf("FPGA Build Date: %02d/%02d/%04d at %02d:%02d:%02d \n",
		mb.masterVer.fpgaDecodedBuildTime.month,
		mb.masterVer.fpgaDecodedBuildTime.day,
		mb.masterVer.fpgaDecodedBuildTime.year,
		mb.masterVer.fpgaDecodedBuildTime.hour,
		mb.masterVer.fpgaDecodedBuildTime.minute,
		mb.masterVer.fpgaDecodedBuildTime.second);
	
}
#endif /* CONFIG_NAI_MISC_INFO */

#ifdef CONFIG_NAI_INT2_POSTCODE
static bool _is_postcode_enable(){
	
	bool ret = false;
	
	if (env_get("enablepostcode") != NULL)
			ret = true;
	
	return ret;
}

static int _get_postcode_poll_time(){
	
	int ret = -1;
	
	if (env_get("enablepostcode") != NULL){
		/*default to 10 seconds*/
		ret = env_get_ulong("postcodedelay", 10, 10);
	}
	
	return ret;
}

static uchar _get_postcode(){
	uchar ret;
	/*SATA Rev Register in FPGA is map to POSTCODE for INT2 board*/
	ret = readl(PS2FPGA_TOP_SATA_REV_OFFSET);
	return ret;
}

static void _print_postcode(uchar code){
	
	const struct postcode_params *params;
	
	params = postcode_table;
	for (; params->name != NULL; params++) {
		if (params->code == code) {
			printf("%s Code:0x%02x\n",params->name,code);
		}
	}
}

void nai_print_int2_postcode(){
	
	int delay;
	uchar code = 0;
	uchar lastcode = 0;
	ulong time;
	
	if(_is_postcode_enable() == true){
		
		delay = _get_postcode_poll_time();
		
		printf("++INT2 PostCode++\n");
		printf("poll delay %d sec\n",delay);
		if(delay > 0){
				//check if slave zynq is ready
			time = get_timer(0);
			do{
		
				code = _get_postcode();
				
				if(lastcode != code)
					_print_postcode(code);
				
				lastcode = code;
							
			}while(get_timer(time) < (delay * 1000));
		}
		printf("--INT2 PostCode--\n");
	}
}
#endif
