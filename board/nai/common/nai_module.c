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
#include <malloc.h>
#include <i2c.h>
#include <nai_mb_fpga_address.h>
#include <nai_common.h>
#include <nai_mb.h>
#include <nai_module.h>
#include <nai_icb.h>
#include <nai_pci.h>
#include <master_slave.h>
#include <NAISerdes.h>
#include <cmd_naiopermsgutils.h>

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define DEBUGF(x...) printf(x)
#else
#define DEBUGF(x...)
#endif /* DEBUG */

#define BIT(x)	(1<<(x))
#define MODULE_ID_CTS		0x43545300
#define MODULE_ID_NONE		0x4E4F4E00
#define MODULE_ID_SPACE		0x00000020
#define MODULE_ADDR_RDY		0xA5A5A5A5

#define MODULE_EEPROM_50_ADDR	0x50
#define MODULE_EEPROM_51_ADDR	0x51
#define MODULE_EEPROM_MAX_COUNT  196

#define MODULE_COMMON_MEM_ADDRESS				0x0
#define MODULE_FPGA_BUILD_TIME	(MODULE_COMMON_MEM_ADDRESS + 0x0030)
#define MODULE_FPGA_REVISION	(MODULE_COMMON_MEM_ADDRESS + 0x003C)
#define MODULE_FW_REVISION		(MODULE_COMMON_MEM_ADDRESS + 0x0074)
#define MODULE_UBOOT_REVISION	(MODULE_COMMON_MEM_ADDRESS + 0x0078)
#define MODULE_FSBL_REVISION	(MODULE_COMMON_MEM_ADDRESS + 0x007C)
#define MODULE_FW_BUILD_TIME	(MODULE_COMMON_MEM_ADDRESS + 0x0080)
#define MODULE_UBOOT_BUILD_TIME	(MODULE_COMMON_MEM_ADDRESS + 0x0098)
#define MODULE_FSBL_BUILD_TIME	(MODULE_COMMON_MEM_ADDRESS + 0x00B0)

//Module common module mode ready state offset
#define MODULE_READY_STATE      (MODULE_COMMON_MEM_ADDRESS + 0x025C)
//Module Configuration Mode States
#define MODULE_CONFIGSTATE_REQUEST_ACK_BIT      		0x00000001
#define MODULE_CONFIGSTATE_READY_BIT             		0x00008000
//Module Operation Mode States
#define MODULE_OPERSTATE_FW_ENTERED_BIT             	(1UL << 16)
#define MODULE_OPERSTATE_FW_COMMONPOPULATED_BIT  		(1UL << 17)
#define MODULE_OPERSTATE_FW_PARAM_LOADED_BIT        	(1UL << 18)
#define MODULE_OPERSTATE_FW_CALIB_LOADED_BIT        	(1UL << 19)
#define MODULE_OPERSTATE_FW_CPLD1_PROG_BIT				(1UL << 20)
#define MODULE_OPERSTATE_FW_CPLD2_PROG_BIT				(1UL << 21)
#define MODULE_OPERSTATE_FW_READY_BIT               	(1UL << 31)

#define MODULE_FUNC_ID			(MODULE_COMMON_MEM_ADDRESS + 0x0E00)
#define MODULE_FUNC_MEM_SIZE	(MODULE_COMMON_MEM_ADDRESS + 0x0E04)

#define MODULE_INF_ID			(MODULE_COMMON_MEM_ADDRESS + 0x0F00)
#define MODULE_INF_MEM_SIZE		(MODULE_COMMON_MEM_ADDRESS + 0x0F04)

#define PS2FPGA_MODULE_DETECTED_DONE_SHIFT	0x08
#define PS2FPGA_MODULE_LINK_DONE_SHIFT	0x08

#define PS2FPGA_MODULE_LINK_STATUS_MB_COMMON_SHIFT	0x10 //shift 16bits
#define MODULE_25000_USDELAY					(25 * 1000)	//25ms
#define MODULE_50000_USDELAY					(50 * 1000)	//50ms
#define MAX_MODULE_BIT_MASK	0x3f	//6 slots
#define MODULE_DETECT_DONE_TIMEOUT		(3 * 1000) // 1 * 1000ms = 3 seconds
#define MODULE_LINK_DONE_TIMEOUT		(1 * 1000) // 1 * 1000ms = 1 seconds

#define MODULE_FW_READY_TIMEOUT		(5 * 1000) // 5 * 1000ms = 5 seconds

#define MODULE_DEFAULT_ADDR			0xFFF1
#define MODULE_DEFAULT_MASK			0x3FFE
#define MODULE_ADDR_START_SHIFT     18
#define MODULE_ADDR_END_SHIFT       2
#define MODULE_ADDR_OFFSET(m)       (PS2FPGA_MODULE1_ADDR_OFFSET + (m * 8))
#define MODULE_MASK_OFFSET(m)       (PS2FPGA_MODULE1_ADDR_MASK_OFFSET + (m * 8))

#ifdef CONFIG_NAI_DEFAULT_MODULE_LO_SIZE_LIMIT
#define DEFAULT_MODULE_LO_SIZE_LIMIT CONFIG_NAI_DEFAULT_MODULE_LO_SIZE_LIMIT
#else
#define DEFAULT_MODULE_LO_SIZE_LIMIT 0x4000 //16KB
#endif

#ifdef CONFIG_NAI_DEFAULT_MODULE_HI_SIZE_LIMIT
#define DEFAULT_MODULE_HI_SIZE_LIMIT CONFIG_NAI_DEFAULT_MODULE_HI_SIZE_LIMIT
#else
#define DEFAULT_MODULE_HI_SIZE_LIMIT 0x200000 //2MB
#endif

#ifdef CONFIG_NAI_MAX_MODULE_MASK_SIZE_LIMIT
#define DEFAULT_MODULE_MAX_MASK_SIZE_LIMIT CONFIG_NAI_MAX_MODULE_MASK_SIZE_LIMIT
#else
#define DEFAULT_MODULE_MAX_MASK_SIZE_LIMIT 0x800000 //8MB
#endif

#define MOD_RECOVERY_COUNT	5
/*global to store bitmask for module Slot Power En and modue HSS ready
 * Slot 1 : bit 0
 * Slot 2 : bit 1
 * Slot 3 : bit 2
 * Slot 4 : bit 3
 * Slot 5 : bit 4
 * Slot 6 : bit 5
*/
static u8 gModuleFound = 0;
static u8 gModuleDetectDone = 0;
static u8 gModuleHSSLinkReady = 0;
static u8 gModulePwrEn = 0;
static u8 gModuleRdy = 0;
static u8 gModuleSpecialGPIO = 0;


#if defined(CONFIG_NAI_MODULE1_EEPROM_BUS_NUM) || defined(CONFIG_NAI_MODULE2_EEPROM_BUS_NUM) \
|| defined(CONFIG_NAI_MODULE3_EEPROM_BUS_NUM) || defined(CONFIG_NAI_MODULE4_EEPROM_BUS_NUM) \
|| defined(CONFIG_NAI_MODULE5_EEPROM_BUS_NUM) || defined(CONFIG_NAI_MODULE6_EEPROM_BUS_NUM)
static unsigned int const i2c_bus_num[] = {
	(unsigned int)CONFIG_NAI_MODULE1_EEPROM_BUS_NUM,
#if (MAX_MODULE_SLOT > 1)
	(unsigned int)CONFIG_NAI_MODULE2_EEPROM_BUS_NUM,
#endif	
#if (MAX_MODULE_SLOT > 2)
	(unsigned int)CONFIG_NAI_MODULE3_EEPROM_BUS_NUM,
#endif		
#if (MAX_MODULE_SLOT > 3)	
	(unsigned int)CONFIG_NAI_MODULE4_EEPROM_BUS_NUM,
#endif		
#if (MAX_MODULE_SLOT > 4)	
	(unsigned int)CONFIG_NAI_MODULE5_EEPROM_BUS_NUM,
#endif		
#if (MAX_MODULE_SLOT > 5)	
	(unsigned int)CONFIG_NAI_MODULE6_EEPROM_BUS_NUM,
#endif		
};
#endif

/*function prototype*/
static bool _nai_is_module_fw_rdy(u8 slotId, u32 fwRdyBit);
static s32 _nai_disable_module(u8 moduleSlot);
static void _nai_enable_refClk(void);
static void _nai_disable_refClk(void);
static void _nai_module_hss_reset(u8 mode, u8 moduleSlot);
static s32 _nai_module_detect_done(u8 moduleSlot);
static s32 _nai_enable_module(u8 moduleSlot);
static void _nai_log_module_status(void);
static s32 _nai_module_link_init_done(u8 moduleSlot);
static s32 _nai_module_pwr_off_seq(u8 moduleSlot);
static s32 _nai_module_pwr_on_seq(u8 moduleSlot);
static s32 _nai_module_reset_seq(u8 moduleSlot);
static void _nai_find_module(u32 *foundedModule);
static void _nai_parse_module_eeprom(Nai_mod *mod, u8* buf);
static void _nai_set_module_gpios(Nai_mod *mod);
static s32 _nai_read_module_eeprom(Nai_mod *mod);
static u32 _nai_cal_module_mask(u32 modSize);
static u32 _nai_chk_module_size_range(u32 modSize);
static s32 _nai_en_module_ps(u8 moduleSlot);
static s32 _nai_dis_module_ps(u8 moduleSlot);
static void _nai_module_addr_mask(void);
static void _nai_module_enum(void);
static void _nai_module_enum_default(void);

#ifdef CONFIG_NAI_1U_2U_3U_MB
static void _nai_1u_2u_3u_mb_init_module(void);
static s32 _nai_axii2c_read_module_eeprom(u8 moduleSlot, u32 count, u8 *buf);
#endif

#ifdef CONFIG_NAI_6U_MB
static void _nai_6u_mb_init_module(void);
static s32 _nai_slv_read_module_eeprom(u8 moduleSlot, u32 count, u8 *buf);
#endif



static void _nai_enable_refClk(void){
	
	writel(1, PS2FPGA_MODULE_REF_CLK_OFFSET);
	//wait
	udelay(MODULE_25000_USDELAY);
}	

static void _nai_disable_refClk(void){
	
	writel(0, PS2FPGA_MODULE_REF_CLK_OFFSET);
	//wait
	udelay(MODULE_25000_USDELAY);
}

static bool _nai_is_module_fw_rdy(u8 slotId, u32 fwRdyBit){

	bool ret = false;
	s32 status = NAI_SUCCESS;
	u32 data = 0;
	int loopCount = 0;
	ulong time = 0;
	
	
	if( (slotId < NAI_MODULE_SLOT_1) || (slotId > MAX_MODULE_SLOT) ){
		printf("Invalid module slot %02d \n", slotId+1);
		return ret;
	}
	
	if( (fwRdyBit != MODULE_OPERSTATE_FW_ENTERED_BIT)
		&& (fwRdyBit != MODULE_OPERSTATE_FW_COMMONPOPULATED_BIT)
		&& (fwRdyBit != MODULE_OPERSTATE_FW_PARAM_LOADED_BIT)
		&& (fwRdyBit != MODULE_OPERSTATE_FW_CALIB_LOADED_BIT)
		&& (fwRdyBit != MODULE_OPERSTATE_FW_READY_BIT) ){
		printf("Invalid FW Rdy BIT 0x%08x \n", fwRdyBit);
		return ret;	
	}

	if( (gModuleFound & (1 << slotId)) &&
		(gModulePwrEn & (1 << slotId)) &&
		(gModuleHSSLinkReady & (1 << slotId)) ){
		
		nai_init_msg_utils(MB_SLOT);
		
		time = get_timer(0);
		
		do{
			loopCount++;
			status = nai_read_reg32_by_slot_request((u8)(slotId+1), (u32)MODULE_READY_STATE, &data);
			if (status != NAI_SUCCESS){
				break;
			}
			
			/*timeout*/
			if(get_timer(time) > MODULE_FW_READY_TIMEOUT)
				break;	
				
		}while(!(data & fwRdyBit));
		
		if(!(data & fwRdyBit) || (status != NAI_SUCCESS)){
			/*if fw not ready or hss com failed then module not rdy*/
			gModuleRdy &= ~(1 << slotId);
			if((status != NAI_SUCCESS)){
				printf("module %d hss com failed [hss=%d retry=%d]\n",slotId+1,status,loopCount);
			}
			
			printf("module %d NOT Ready[fw:0x%08x hss:%d retry=%d]\n",slotId+1,data,status,loopCount);
			
		}else{
			gModuleRdy |= (1 << slotId);
			//printf("Module %d FW Ready[fw:0x%08x retry=%d]\n",slotId+1,data,loopCount);
			ret = true;
		}
	}
	
	return ret;
}

static void _nai_module_hss_reset(u8 mode, u8 moduleSlot){
	
	u8 val = 0;
	
	if( (moduleSlot < NAI_MODULE_SLOT_1) || (moduleSlot > MAX_MODULE_SLOT) )
		return;
	
	val = readl(PS2FPGA_MODULE_HSS_OFFSET);
			
	if(mode){
		//Release module HSS from reset
		val &= ~(1 << moduleSlot);
		//printf("HSS out of reset modulet bitmask 0x%08x\n",val);
		
		//TODO: A bug in MB FPGA that we can't use bitmap to take each module out of HSS reset
		//Temp WR for the above bug: take all module out of HSS reset by clear all bits in the HSS register
		val = 0x0;
		
		writel(val, PS2FPGA_MODULE_HSS_OFFSET);
		
	}else{
		
		//Held module HSS in reset
		val |= (1 << moduleSlot);
		
		//TODO: A bug in MB FPGA that we can't use bitmap to held each module in HSS reset
		//Temp WR for the above bug: take all module out of HSS reset by clear all bits in the HSS register
		val = MAX_MODULE_BIT_MASK;
		
		//printf("HSS held reset modulet bitmask 0x%08x\n",val);
		writel(val, PS2FPGA_MODULE_HSS_OFFSET);
	}
	
	//wait
	udelay(MODULE_25000_USDELAY);
	
}

static s32 _nai_module_detect_done(u8 moduleSlot)
{
	u32 moduleDetectReg = 0;
	u8 moduleDoneByte = 0;
	u8 moduleReadyByte = 0;
	s32 ret = NAI_MODULE_SUCCESS;
	ulong time = 0;
		
	if( (moduleSlot < NAI_MODULE_SLOT_1) || (moduleSlot > MAX_MODULE_SLOT) ){
		ret = NAI_MODULE_ERROR;
		goto EXIT;
	}
	
	time = get_timer(0);
	do{
		
		moduleDetectReg = readl(PS2FPGA_MODULE_DETECT_STATUS_OFFSET);
		/*
		 * upper byte is module detect done
		 * lower byte is module ready
		*/
		moduleDoneByte = moduleDetectReg >> PS2FPGA_MODULE_DETECTED_DONE_SHIFT;
		moduleReadyByte = moduleDetectReg & 0xFF;
		
		//timeout
		if(get_timer(time) > MODULE_DETECT_DONE_TIMEOUT)
			break;			
			
	}while(!(moduleDoneByte & (1 << moduleSlot)));

	if( (moduleDoneByte & (1 << moduleSlot)) && (moduleReadyByte & (1 << moduleSlot)) ){
		printf("Module# %x detected [Status = 0x%08x]\n", moduleSlot+1, moduleDetectReg);
	}else{
		printf("Module# %x is NOT detected [Status = 0x%08x]\n", moduleSlot+1, moduleDetectReg);
		ret = NAI_MODULE_ERROR;
	}
	
EXIT:	
	return ret;
}

static s32 _nai_module_link_init_done(u8 moduleSlot)
{
	u32 modulelinkReg = 0;
	u32 modulelinkDone = 0;
	u32 modulelinkReady = 0;
	s32 ret = NAI_MODULE_SUCCESS;
	ulong time = 0;
	
	if( (moduleSlot < NAI_MODULE_SLOT_1) || (moduleSlot > MAX_MODULE_SLOT) ){
		ret = NAI_MODULE_ERROR;
		goto EXIT;
	}
		
	time = get_timer(0);
	do{
		
		modulelinkReg = readl(PS2FPGA_MODULE_LINK_STATUS_OFFSET);
		/*
		 * upper byte is module detect done
		 * lower byte is module ready
		*/

		modulelinkDone = (modulelinkReg >> PS2FPGA_MODULE_LINK_DONE_SHIFT);
		modulelinkReady = modulelinkReg & 0xFF;
		
		//timeout
		if(get_timer(time) > MODULE_LINK_DONE_TIMEOUT)
			break;			

	}while(!(modulelinkDone & (1 << moduleSlot)));



	if( (modulelinkDone & (1 << moduleSlot)) && (modulelinkReady & (1 << moduleSlot)) ){
		printf("Module# %x Link is Ready [Status = 0x%08x]\n", moduleSlot+1, modulelinkReg);
	}else{
		printf("Module# %x Link is NOT Ready [Status = 0x%08x]\n", moduleSlot+1, modulelinkReg);
		ret = NAI_MODULE_ERROR;
	}
	
EXIT:	
	return ret;
}

static s32 _nai_en_module_ps(u8 moduleSlot){
	u8 val = 0;
	s32 ret = NAI_MODULE_SUCCESS;
	
	if( (moduleSlot < NAI_MODULE_SLOT_1) || (moduleSlot > MAX_MODULE_SLOT) ){
		ret = NAI_MODULE_ERROR;
		goto EXIT;
	}
	
	//printf("en_ps module slot : %d \n", moduleSlot+1);
	//enable module pwr supply
	val = readl(PS2FPGA_MODULE_POWER_OFFSET);
	val |= (1 << moduleSlot);
	//printf("module pwr_on bit_mask val : 0x%08x\n", val);
	writel(val, PS2FPGA_MODULE_POWER_OFFSET);
	//wait 
	udelay(MODULE_50000_USDELAY);
		
EXIT:	
	return ret;
	
}

static s32 _nai_dis_module_ps(u8 moduleSlot){
	u8 val = 0;
	s32 ret = NAI_MODULE_SUCCESS;
	
	if( (moduleSlot < NAI_MODULE_SLOT_1) || (moduleSlot > MAX_MODULE_SLOT) ){
		ret = NAI_MODULE_ERROR;
		goto EXIT;
	}
	
	//printf("dis_ps module slot : %d\n", moduleSlot+1);
	//disable module pwr supply
	val = readl(PS2FPGA_MODULE_POWER_OFFSET);
	val &= ~(1 << moduleSlot);
	//printf("module pwr_on bit_mask val : 0x%08x\n", val);
	writel(val, PS2FPGA_MODULE_POWER_OFFSET);
	//wait 
	udelay(MODULE_50000_USDELAY);
		
EXIT:	
	return ret;
	
}

static s32 _nai_module_pwr_on_seq(u8 moduleSlot){
	
	u8 val = 0;
	s32 ret = NAI_MODULE_SUCCESS;
	
	if( (moduleSlot < NAI_MODULE_SLOT_1) || (moduleSlot > MAX_MODULE_SLOT) ){
		ret = NAI_MODULE_ERROR;
		goto EXIT;
	}
	
	//printf("Power ON module slot : %d \n", moduleSlot+1);

	//enable module pwr supply
	_nai_en_module_ps(moduleSlot);
	
	//enable module clk
	val = readl(PS2FPGA_MODULE_CLK_OFFSET);
	val |= (1 << moduleSlot);
	//printf("module clk_en bit_mask val : 0x%08x\n", val);
	writel(val, PS2FPGA_MODULE_CLK_OFFSET);
	//wait
	udelay(MODULE_25000_USDELAY);
	
	//enable ref clk
	_nai_enable_refClk();

	//take module out of reset
	val = readl(PS2FPGA_MODULE_RESET_OFFSET);
	val |= (1 << moduleSlot);
	//printf("module reset bit_mask val : 0x%08x\n", val);
	writel(val, PS2FPGA_MODULE_RESET_OFFSET);
	//wait
	udelay(MODULE_25000_USDELAY);
	
EXIT:	
	return ret;
}

static s32 _nai_module_pwr_off_seq(u8 moduleSlot){
	
	u8 val = 0;
	s32 ret = NAI_MODULE_SUCCESS;
	
	if( (moduleSlot < NAI_MODULE_SLOT_1) || (moduleSlot > MAX_MODULE_SLOT) ){
		ret = NAI_MODULE_ERROR;
		goto EXIT;
	}
			
	//printf("Power OFF module slot : %d \n", moduleSlot+1);			
	//held module in reset
	val = readl(PS2FPGA_MODULE_RESET_OFFSET);
	val &= ~(1 << moduleSlot);
	//printf("module reset bit_mask val : 0x%08x\n", val);
	writel(val, PS2FPGA_MODULE_RESET_OFFSET);
	//wait
	udelay(MODULE_25000_USDELAY);
			
	//disable module clk
	val = readl(PS2FPGA_MODULE_CLK_OFFSET);
	val &= ~(1 << moduleSlot);
	//printf("module clk_en bit_mask val : 0x%08x\n", val);
	writel(val, PS2FPGA_MODULE_CLK_OFFSET);
	//wait
	udelay(MODULE_25000_USDELAY);
	
	//disable module pwr supply
	_nai_dis_module_ps(moduleSlot);

	//wait 
	udelay(MODULE_25000_USDELAY);

EXIT:	
	return ret;
}

static s32 _nai_module_reset_seq(u8 moduleSlot){
	
	u8 val = 0;
	s32 ret = NAI_MODULE_SUCCESS;
	
	if( (moduleSlot < NAI_MODULE_SLOT_1) || (moduleSlot > MAX_MODULE_SLOT) ){
		ret = NAI_MODULE_ERROR;
		goto EXIT;
	}
	
	//Reset Module only if it's power ON
	if( (gModulePwrEn & (1 << moduleSlot)) ){			
		printf("Reset module slot : %d \n", moduleSlot+1);			
		/* Module Reset and HSS is a bitmap register
		 * Bit 0 = Module 1
		 * Bit 1 = Module 2
		 * Bit 2 = Module 3
		 * Bit 3 = Module 4
		 * Bit 4 = Module 5
		 * Bit 5 = Module 6
		 */
		
		/* SoftReset module operation
		 * 1. Held module HSS in reset
		 * 2. wait 1ms (based on KF's module reset spec)
		 * 3. Held module reset 
		 * 4. wait 1ms (based on KF's module reset spec)
		 * 5. Release module from reset
		 * 6. wait for module detect done and module ready (timeout in 1seconds)
		 * 7. take module hss out of reset 
		 * 8. wait for link init done (timeout in 1seconds)
		 */

		//1. Held module HSS in reset
		_nai_module_hss_reset(0, moduleSlot);

		// 2. Held module nReset
		val = readl(PS2FPGA_MODULE_RESET_OFFSET);
		val &= ~(1 << moduleSlot);
		writel(val, PS2FPGA_MODULE_RESET_OFFSET);
		// 3. wait 1ms (based on KF's module reset spec)
		udelay(MODULE_25000_USDELAY);
		// 4. Release module from nReset
		val = readl(PS2FPGA_MODULE_RESET_OFFSET);
		val |= (1 << moduleSlot);
		writel(val, PS2FPGA_MODULE_RESET_OFFSET);
		
		//5. wait for module detect done and module ready (timeout in 1seconds)	
		if( NAI_MODULE_SUCCESS == _nai_module_detect_done(moduleSlot)){
			gModuleDetectDone |=  (1 << moduleSlot);
		}else{
			gModuleDetectDone &=  ~(1 << moduleSlot);
		}

//TODO: Waiting for HSS Reset from HW. 
#if 0			
		//6. Release module HSS from reset
		if((gModuleDetectDone & (1 << moduleSlot))){
			_nai_module_hss_reset(1, moduleSlot);
			
			//7. wait for link init done (timeout in 1seconds)
			if( NAI_MODULE_SUCCESS == _nai_module_link_init_done(moduleSlot)){
				gModuleHSSLinkReady |=  (1 << moduleSlot);
			}else{
				gModuleHSSLinkReady &=  ~(1 << moduleSlot);
			}
		}
		
		DEBUGF("Module gModuleHSSLinkReady BitMask 0x%02x \n", gModuleHSSLinkReady);
#endif
			
		DEBUGF("Module gModuleDetectDone BitMask 0x%02x \n", gModuleDetectDone);

	}else{
	
		printf("Error: module slot : %d not enabled \n", moduleSlot+1);
		DEBUGF("Module slot %d is not enable gModulePwrEn bitMask: 0x%02x \n", moduleSlot+1, gModulePwrEn);	
		
	}//endo of if( (gModulePwrEn & (1 << moduleSlot)) )

EXIT:	
	return ret;
}

static s32 _nai_enable_module(u8 moduleSlot){
		
	s32 ret = NAI_MODULE_SUCCESS;

	if( (moduleSlot < NAI_MODULE_SLOT_1) || (moduleSlot > MAX_MODULE_SLOT) ){
		ret = NAI_MODULE_ERROR;
		goto EXIT;
	}
	
	if (gModuleSpecialGPIO & (1 << moduleSlot)) {
		printf("Module slot : %d will not be enabled due to custom GPIO config\n", moduleSlot+1);
		goto EXIT;
	}
		
	if( (gModuleFound & (1 << moduleSlot)) 
	&& !(gModulePwrEn & (1 << moduleSlot)) ){

		printf("Enable module slot : %d \n", moduleSlot+1);		
		
		//Held HSS in reset
		_nai_module_hss_reset(0, moduleSlot);

		//Power on module
		_nai_module_pwr_on_seq(moduleSlot);
		gModulePwrEn |= (1 << moduleSlot);
	
//TODO: Waiting for HSS Reset from HW. 
#if 0
		//Take HSS out of reset
		if((gModuleDetectDone & (1 << moduleSlot))){
			_nai_module_hss_reset(1, moduleSlot);
			
			if(NAI_MODULE_SUCCESS == _nai_module_link_init_done(moduleSlot)){
				gModuleHSSLinkReady |= (1 << moduleSlot);
			}else{
				gModuleHSSLinkReady &= ~(1 << moduleSlot);
			}
		}	
#endif		
	}else{
		if(gModulePwrEn & (1 << moduleSlot))
			printf("module slot : %d was enabled \n", moduleSlot+1);
		else
			printf("Error: module slot : %d not detected \n", moduleSlot+1);
	}//end if

EXIT:	

	return ret;
}

static s32 _nai_disable_module(u8 moduleSlot){
	
	s32 ret = NAI_MODULE_SUCCESS;
	u32 val = 0;
		
	if( (moduleSlot < NAI_MODULE_SLOT_1) || (moduleSlot > MAX_MODULE_SLOT) ){
		ret = NAI_MODULE_ERROR;
		goto EXIT;
	}
	
	
	if( (gModuleFound & (1 << moduleSlot)) 
		&& (gModulePwrEn & (1 << moduleSlot)) ){		

			printf("Disable module slot : %d \n", moduleSlot+1);
			//clear module config mode
			val = readl(PS2FPGA_MODULE_CONFIG_MODE_OFFSET);
			val &= ~(1 << moduleSlot);
			writel(val, PS2FPGA_MODULE_CONFIG_MODE_OFFSET);
//TODO: Waiting for HSS Reset from HW. 
#if 0			
			//held HSS in reset
			_nai_module_hss_reset(0, moduleSlot);
#endif			
			_nai_module_pwr_off_seq(moduleSlot);
			
			gModulePwrEn &= ~(1 << moduleSlot); 
			gModuleHSSLinkReady &= ~(1 << moduleSlot);
			gModuleDetectDone &= ~(1 << moduleSlot);
			gModuleRdy &= ~(1 << moduleSlot);
			
			//if no all module slot power are off
			//held HSS in reset and turn off ref clk
			//clear all global variable
			if(!gModulePwrEn){
				
				//TODO: Fix HSS WR: Remove this line when we have a fix from HW
				//held HSS in reset
				_nai_module_hss_reset(0, moduleSlot);

				//TODO: Fix REF CLK WR for 6U board: Ref CLK will disable ICB
				//disable ref clk
#ifdef CONFIG_NAI_1U_2U_3U_MB
				_nai_disable_refClk();
#endif
			
				gModulePwrEn = 0;
				gModuleHSSLinkReady = 0;
				gModuleDetectDone = 0;
				gModuleRdy = 0;
			
			}//end if
			
		}else{
			if(!(gModulePwrEn & (1 << moduleSlot)) )
				printf("module slot : %d was disabled \n", moduleSlot+1);
			else
				printf("Error: module slot : %d not detected \n", moduleSlot+1);
		}//end if

EXIT:	
	return ret;
}

#ifdef CONFIG_NAI_1U_2U_3U_MB
static s32 _nai_axii2c_read_module_eeprom(u8 moduleSlot, u32 count, u8 *buf){
	
	unsigned int bus;
	s32 ret = NAI_MODULE_SUCCESS;
	
	if((moduleSlot < NAI_MODULE_SLOT_1) || (moduleSlot > MAX_MODULE_SLOT) 
		|| (count < 0) || (count > 255) ){
		ret = NAI_MODULE_ERROR;
		goto EXIT;
	}
	
	//Save the current I2C bus	
	bus = i2c_get_bus_num();
			
	//Set I2C Bus per module slot
	i2c_set_bus_num(i2c_bus_num[moduleSlot]);
	DEBUGF( "%s:bus%d \n",__func__,i2c_get_bus_num());
	
	if(i2c_probe(MODULE_EEPROM_50_ADDR) == 0){
		i2c_read(MODULE_EEPROM_50_ADDR, 0, 1, buf, count);
	}else if (i2c_probe(MODULE_EEPROM_51_ADDR) == 0){
		i2c_read(MODULE_EEPROM_51_ADDR, 0, 1, buf, count);
	}else{
		//printf("No Module Found on Slot %d \n", moduleSlot+1);
		ret = NAI_MODULE_ERROR; 
	}
				
	//Restore I2C bus
	i2c_set_bus_num(bus);
	DEBUGF("%s:module %02x \n",__func__,moduleSlot);
	
EXIT:	
	return ret;
}
#endif

#ifdef CONFIG_NAI_6U_MB
static s32 _nai_slv_read_module_eeprom(u8 moduleSlot, u32 count, u8 *buf){
	
	s32 ret = NAI_MODULE_SUCCESS;
	u32 retVal = 0;
	
	if((moduleSlot < NAI_MODULE_SLOT_1) || (moduleSlot > MAX_MODULE_SLOT) 
		|| (count < 0) || (count > 255) ){
		ret = NAI_MODULE_ERROR;
		goto EXIT;
	}
		
	retVal = slave_read((SLAVE_MODULE1_INF_EEPROM_DATA - ( moduleSlot << 8 )), 1, count, (u32 *)buf, FALSE);
	
	if(retVal != count){
		ret = NAI_MODULE_ERROR;
		printf("%s: Failed to read eeprom data from slv ret=%x, count=%x\n",__func__, retVal, count);
	}
		
EXIT:	
	return ret;

}
#endif

static void _nai_parse_module_eeprom(Nai_mod *mod, u8* buf){
	
	u32 data;
	u16 i;
	u8  blankEEPROM = 1;
	
	if(	mod->modTypeId == INF_MOD){
		
		if(buf){
			/* parse module ID */
			strncpy(mod->infModule.modEepromData.moduleId, (char *)buf, MODULE_ID_STRING_LEN);
			
			/* parse module size */
			data = ( buf[4] | 
					(buf[5] << 8) | 
					(buf[6] << 16) | 
					(buf[7] << 24) );
			/*
			* The moduleSize is stored
			* as little endian in the module common memory area
			*/
			mod->infModule.modEepromData.moduleSize = cpu_to_be32(data);
			
			for (i=0; i < MODULE_EEPROM_MAX_COUNT; i++) {
				if (buf[i] != 0xFF) {
					blankEEPROM = 0;
					break;
				}
			}
			/* NOTE: if we are dealing with a "virgin" eeprom than we do not look at gpio info since
			 * EEPROM is set to all FFs which would give us a FALSE positive and module would not get powered */
			if (!blankEEPROM) {			
				/* Currently GPIO info is not stored in module common area...
				 * The belief is that we only need this info before module is
				 * powered so we know how to set the GPIOs for the module. If 
				 * we need to store info in module common, we may need to 
				 * convert to little indian like module size 
				 */
				/* parse gpio direction */			
				mod->infModule.modEepromData.gpioDirection = buf[160];

				/* parse gpio data */
				mod->infModule.modEepromData.gpioData = buf[161];
				
				/* parse gpio config */
				mod->infModule.modEepromData.gpioConfig = buf[163];
				
				/* parse gpio source select */
				data = (buf[164] |
					   (buf[165] << 8) |
					   (buf[166] << 16) |
					   (buf[167] << 24) );
				mod->infModule.modEepromData.gpioSourceSelect = cpu_to_be32(data);
			}
			else {
				mod->infModule.modEepromData.gpioDirection = 0;
				mod->infModule.modEepromData.gpioData = 0;			
				mod->infModule.modEepromData.gpioConfig = 0;				
				mod->infModule.modEepromData.gpioSourceSelect = 0;
				mod->infModule.modEepromData.spare = 0;
			}			
		}		
	}
}

static void _nai_set_module_gpios(Nai_mod *mod){
	u32 modGPIO_DataRegAddr = 0;
	u32 modGPIO_DirectionRegAddr = 0;
	u32 modGPIO_SourceSelRegAddr = 0;
	
	/*GPIOs only need to be "forced" if config is has a 1 in the 1st bit (bit 0) - 
	 * indicating this is a special module
	 */
	if ((mod->infModule.modEepromData.gpioConfig & 0x01) == 1)
	{
		printf("Module at slot %d found to have special GPIO settings\n", (mod->moduleSlot+1));
	
		gModuleSpecialGPIO |= (1 << mod->moduleSlot);
		
		switch (mod->moduleSlot){
		case NAI_MODULE_SLOT_1 :
			modGPIO_DataRegAddr			= 0x43C44140;
			modGPIO_DirectionRegAddr	= 0x43C44160;
			modGPIO_SourceSelRegAddr	= 0x43C44180;
			break;
				
		case NAI_MODULE_SLOT_2 :
			modGPIO_DataRegAddr			= 0x43C44144;
			modGPIO_DirectionRegAddr	= 0x43C44164;
			modGPIO_SourceSelRegAddr	= 0x43C44184;
			break;
				
		case NAI_MODULE_SLOT_3 :
			modGPIO_DataRegAddr			= 0x43C44148;
			modGPIO_DirectionRegAddr	= 0x43C44168;
			modGPIO_SourceSelRegAddr	= 0x43C44188;
			break;
				
		case NAI_MODULE_SLOT_4 :
			modGPIO_DataRegAddr			= 0x43C4414C;
			modGPIO_DirectionRegAddr	= 0x43C4416C;
			modGPIO_SourceSelRegAddr	= 0x43C4418C;
			break;
			
		case NAI_MODULE_SLOT_5 :
			modGPIO_DataRegAddr			= 0x43C44150;
			modGPIO_DirectionRegAddr	= 0x43C44170;
			modGPIO_SourceSelRegAddr	= 0x43C44190;
			break;
			
		case NAI_MODULE_SLOT_6 :
			modGPIO_DataRegAddr			= 0x43C44154;
			modGPIO_DirectionRegAddr	= 0x43C44174;
			modGPIO_SourceSelRegAddr	= 0x43C44194;		
			break;			
		
		default:
			break;
		};
		
		/* Tell FPGA about "special" gpio settings */
		printf("Module at slot %d has GPIO Data value of 0x%08x \n", (mod->moduleSlot+1), mod->infModule.modEepromData.gpioData);
		writel(mod->infModule.modEepromData.gpioData, modGPIO_DataRegAddr);
		printf("Module at slot %d has GPIO Direction value of 0x%08x \n", (mod->moduleSlot+1), mod->infModule.modEepromData.gpioDirection);
		writel(mod->infModule.modEepromData.gpioDirection, modGPIO_DirectionRegAddr);
		printf("Module at slot %d has GPIO Source Select value of 0x%08x \n", (mod->moduleSlot+1), mod->infModule.modEepromData.gpioSourceSelect);
		writel(mod->infModule.modEepromData.gpioSourceSelect, modGPIO_SourceSelRegAddr);
	}	
	else {
		gModuleSpecialGPIO &= ~(1 << mod->moduleSlot);	
	}	
}

static s32 _nai_read_module_eeprom(Nai_mod *mod){

	s32 ret = NAI_MODULE_SUCCESS;
#ifdef CONFIG_NAI_6U_MB
	bool slaveFWReady = false;
#endif	
	u8*	pBuf;
	/*hardcode module eeprom data count to 196 bytes for now*/
	u32 count = MODULE_EEPROM_MAX_COUNT; 
	u8 slotId = mod->moduleSlot;
	mod->moduleRdy = false;
	
	if( (slotId < NAI_MODULE_SLOT_1) || (slotId > MAX_MODULE_SLOT) ){
		printf("Invalid module slot %02d \n", slotId+1);
		goto EXIT;
	}
	
	pBuf = (u8 *)malloc((count * sizeof(* pBuf)));
    memset(pBuf, 0, count);
    
#if defined(CONFIG_NAI_1U_2U_3U_MB)
	/*
	 * check module is power off and module is connected 
	 * before read module eeprom
	 */
	if(!(gModulePwrEn & (1 << slotId)) &&
			(gModuleFound & (1 << slotId)) ){
		if(NAI_MODULE_SUCCESS == _nai_axii2c_read_module_eeprom(slotId, count, pBuf)){
			_nai_parse_module_eeprom(mod, pBuf);
			
			/* Only set module ready flag to true if not a special module! */
			if ((mod->infModule.modEepromData.gpioConfig & 0x01) == 0) {
				//set module info ready
				mod->moduleRdy = true;
			}
			ret = NAI_MODULE_SUCCESS;
		}
	}
	
#elif defined(CONFIG_NAI_6U_MB)
	/*
	 * check module is power off and module is connected 
	 * before read module eeprom
	 */
	if(!(gModulePwrEn & (1 << slotId)) &&
		( gModuleFound & (1 << slotId)) ){ 
	/*check slave zynq is ready before read module eeprom data*/
		slaveFWReady = nai_is_slv_fw_rdy();
		if( true == slaveFWReady ){
			if(NAI_MODULE_SUCCESS ==  _nai_slv_read_module_eeprom(slotId, count, pBuf)){
				_nai_parse_module_eeprom(mod, pBuf);
				
				/* Only set module ready flag to true if not a special module! */
				if ((mod->infModule.modEepromData.gpioConfig & 0x01) == 0) {
					//set module info ready
					mod->moduleRdy = true;
				}
				ret = NAI_MODULE_SUCCESS;
			}
		}
	}
#else
	ret = NAI_MODULE_ERROR;
#endif	
	
	free(pBuf);
	
EXIT:		
	return ret;
}


/* 
 * 1U and 3U Motherboard init sequence
 */
#ifdef CONFIG_NAI_1U_2U_3U_MB
static void _nai_1u_2u_3u_mb_init_module(void){
	
	u32 slotIndex = 0;
	u32	detectedModule = 0;
		
	gModulePwrEn = 0;
	gModuleDetectDone = 0;
	gModuleHSSLinkReady = 0;
	gModuleFound = 0;
	gModuleSpecialGPIO = 0;
	
	printf("1U/2U/3U MB Init All Module \n");			
	

	//disable ref clk
	_nai_disable_refClk();
	
	//Power off All Module
	for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
		//held HSS in reset
		_nai_module_hss_reset(0, slotIndex);
		_nai_module_pwr_off_seq(slotIndex);
	}

	//search for connected module by reading eeprom on module
	_nai_find_module(&detectedModule);
	gModuleFound = detectedModule;

	/*configure module mask*/
	_nai_module_addr_mask();	
	
	for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
		if( (gModuleFound & (1 << slotIndex)) ){
			printf("Found Module on Slot %d \n", slotIndex+1);
		}
	}

#ifdef CONFIG_NAI_CPCI
			/*Config PCI device size and mode*/
			nai_cpci_setup();
#endif /* CONFIG_NAI_CPCI*/

	/*module enum*/
	_nai_module_enum();
	
	DEBUGF("Module gModuleFound: 0x%02x \n", gModuleFound);
	
	if(gModuleFound){
		//Power ON each module 		
		for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
			/* NOTE: We do not perform this if module has special GPIO configuration */
			if( (gModuleFound & (1 << slotIndex)) && ((gModuleSpecialGPIO & (1 << slotIndex))==0)){ 
				_nai_enable_module(slotIndex);
			}
		}
		
		//Check Module Detect and Ready
		for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
			/* NOTE: We do not perform this if module has special GPIO configuration */
			if((gModuleFound & (1 << slotIndex)) && (gModulePwrEn & (1 << slotIndex)) && ((gModuleSpecialGPIO & (1 << slotIndex))==0)){
				if( NAI_MODULE_SUCCESS == _nai_module_detect_done(slotIndex)){
					gModuleDetectDone |= (1 << slotIndex);
				}else{
					gModuleDetectDone &= ~(1 << slotIndex);
				}
			}
		}
			
		//Start TODO: Fix HSS WR: Remove the WR
		for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
			/* NOTE: We do not perform this if module has special GPIO configuration */
			if((gModuleDetectDone & (1 << slotIndex)) && ((gModuleSpecialGPIO & (1 << slotIndex))==0)){
				//release hss from reset
				_nai_module_hss_reset(1, slotIndex);
				
				//Verify HSS Link Ready
				if(NAI_MODULE_SUCCESS == _nai_module_link_init_done(slotIndex)){
					gModuleHSSLinkReady |= (1 << slotIndex);
					/*check module FW is rdy*/
					_nai_is_module_fw_rdy(slotIndex, MODULE_OPERSTATE_FW_ENTERED_BIT);
				}else{
					gModuleHSSLinkReady &= ~(1 << slotIndex);
				}
			}
		}
		//END TODO: Fix HSS WR: 
		
	}//end of if(gModuleFound)
		
}
#endif

static void _nai_find_module(u32 *foundedModule){

#ifdef CONFIG_NAI_6U_MB		
	u32 ret = 0;
#endif
	
#ifdef CONFIG_NAI_1U_2U_3U_MB
	u8 buf;
	u32 slotIndex = 0;
#endif	
	*foundedModule = 0;
	
#if defined(CONFIG_NAI_6U_MB)		
	
	ret = slave_read(SLAVE_DETECTED_MODULE, 1, 1, foundedModule, FALSE);		
	if(!ret)	
		*foundedModule = 0x0;
#elif defined(CONFIG_NAI_1U_2U_3U_MB)
	//module detect by reading module eeprom
	for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
		if(NAI_MODULE_SUCCESS == _nai_axii2c_read_module_eeprom(slotIndex, 1, &buf)){
			*foundedModule |= (1 << slotIndex); 
		}
	}
#else
	*foundedModule = 0;
#endif			

}


#ifdef CONFIG_NAI_6U_MB
static void _nai_6u_mb_init_module(void){
	
	bool	slaveFWReady = false;
	u32		slotIndex = 0;
	u32		detectedModule = 0;
		
	gModulePwrEn = 0;
	gModuleDetectDone = 0;
	gModuleHSSLinkReady = 0;
	gModuleFound = 0;
	gModuleSpecialGPIO = 0;
	
	printf("6U MB Init All Module \n");
	
	//Disable ref clk
	_nai_disable_refClk();

	//Power OFF All Module
	for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
		//Held hss in reset
		_nai_module_hss_reset(0, slotIndex);
		_nai_module_pwr_off_seq(slotIndex);
	}
	
	/*TODO: Fix HW REF CLK WR for 6U board: 
	 * ICB is using module Ref CLK and as a workaround
	 * we need enable module refClk before enable ICB
	 */
	_nai_enable_refClk();
	//enable ICB
	nai_icb_enable();

	//check mb slave zynq ready state
	slaveFWReady = nai_is_slv_fw_rdy();
	
	if(slaveFWReady){
		/*1.Check detected module from slave*/
		/*1.1 init uart1 port*/
		uart1_init();
		
		/*1.2 read detected module register from slave*/
		_nai_find_module(&detectedModule);
		gModuleFound = detectedModule;
	
		for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
			if( (gModuleFound & (1 << slotIndex)) ){
				printf("Found Module on Slot %d \n", slotIndex+1);
			}
		}
		
		/*configure module mask*/
		_nai_module_addr_mask();
			
		/*module enum*/
		_nai_module_enum();
		
		/*1.3 output found module status to debug console*/
		if(gModuleFound){

			//1.4 Power ON each module 		
			for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
				if( (gModuleFound & (1 << slotIndex)) && ((gModuleSpecialGPIO & (1 << slotIndex))==0) ){ 
					_nai_enable_module(slotIndex);
				}
			}
			
			//Check Module Detect and Ready
			for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
				if((gModuleFound & (1 << slotIndex)) && (gModulePwrEn & (1 << slotIndex)) && ((gModuleSpecialGPIO & (1 << slotIndex))==0)){
					if( NAI_MODULE_SUCCESS == _nai_module_detect_done(slotIndex)){
						gModuleDetectDone |= (1 << slotIndex);
					}else{
						gModuleDetectDone &= ~(1 << slotIndex);
					}
				}
			}
		
			//Start TODO: Fix HSS WR: Remove the WR
			for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
			
				if((gModuleDetectDone & (1 << slotIndex)) && ((gModuleSpecialGPIO & (1 << slotIndex))==0) ){
					//release hss from reset
					_nai_module_hss_reset(1, slotIndex);
					
					//Verify HSS Link Ready
					if(NAI_MODULE_SUCCESS == _nai_module_link_init_done(slotIndex)){
						gModuleHSSLinkReady |= (1 << slotIndex);
						/*check module FW is rdy*/
						_nai_is_module_fw_rdy(slotIndex, MODULE_OPERSTATE_FW_ENTERED_BIT);
					}else{
						gModuleHSSLinkReady &= ~(1 << slotIndex);
					}
				}	
			}
			//END TODO: Fix HSS WR: 			
				
		}//end if
				
	}else{
		printf("Slave Zynq is not ready !!!\n");
	}

}
#endif

static void _nai_log_module_status(void){
	
	u32 moduleStatus = 0;
	
	//log found module slot and module serdes link ready to MB Common area
	moduleStatus = gModuleFound << 24 | gModulePwrEn << 16 | gModuleRdy << 8 | gModuleHSSLinkReady << 0;
	DEBUGF("Module Status [status = 0x%08x] \n", moduleStatus);	
	writel(moduleStatus, PS2FPGA_MB_COMMON_MODULE_STATUS_OFFSET);
		
}

static u32 _nai_chk_module_size_range(u32 modSize){
	
	u32 size = modSize;
	/*check module size is not out of range*/
	if(DEFAULT_MODULE_HI_SIZE_LIMIT < size){
		size = DEFAULT_MODULE_HI_SIZE_LIMIT;
		DEBUGF("%s:module is out of range and set size to: 0x%08x\n",__func__,size);
	}else if(DEFAULT_MODULE_LO_SIZE_LIMIT > size){
		size = DEFAULT_MODULE_LO_SIZE_LIMIT;
		DEBUGF("%s:module is out of range and set size to: 0x%08x\n",__func__,size);
	}else{
		size = modSize;	
	}
	
	return size;
}

static u32 _nai_cal_module_mask(u32 modSize){
	
	u32 data = (DEFAULT_MODULE_LO_SIZE_LIMIT - 1);
	u32 startSize = 0;
    u32 nextSize = 0;
		
	/*init start and next size */
	startSize = DEFAULT_MODULE_LO_SIZE_LIMIT;
	nextSize = startSize * 2;
	do{
		 if (startSize > modSize ){
			 data = ((modSize - 1) | (DEFAULT_MODULE_LO_SIZE_LIMIT - 1)); 
			 break;
		 }else{
			 if ((startSize < modSize) && (nextSize >= modSize)){
				data = ((modSize - 1) | (nextSize - 1));
				 break;
			 }else if (DEFAULT_MODULE_MAX_MASK_SIZE_LIMIT < modSize){ 		
				data = ((modSize - 1) | (DEFAULT_MODULE_MAX_MASK_SIZE_LIMIT - 1)); 
				break;
			 }else{
				 startSize = nextSize;
				 nextSize *= 2;
			 }
		 }	
	}while(DEFAULT_MODULE_MAX_MASK_SIZE_LIMIT > startSize);
	
	return data;
}
/*default all 8 module slots in mb common memory
NOTE: This is based on MarkL MBexec Logic */
static void _nai_module_enum_default(){
	u32	index = 0;
	u32	maxSlot = 8;
	DEBUGF("%s\n",__func__);
	for(index = 0; index < maxSlot; index++){
		 /*module address to 0xFFFFFFFF
		 * module size to 0x00000000 
		 * module id to 0x00000000 */
		writel(0xFFFFFFFF, (PS2FPGA_MB_COMMON_MOD_SLOT1_ADDR_OFFSET+(index*4)));
		writel(0x0, (PS2FPGA_MB_COMMON_MOD_SLOT1_SIZE_OFFSET+(index*4)));
		writel(0x0, (PS2FPGA_MB_COMMON_MOD_SLOT1_ID_OFFSET+(index*4)));
	}
}
/*populated MB common area with module id, address and size*/
static void _nai_module_enum(){
	
	u32	slotIndex = 0;
	u32	modSize = 0;
	u32	modId = 0;
	/*module start after mb 1K MB*/
	u32	modAddr = PS2FPGA_MB_COMMON_SIZE;
	u32	lastSize = 0;
	Nai_mod mod;
	
	_nai_module_enum_default();
	
	for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
		memset(&mod, 0, sizeof(mod));
		modId = 0;
		mod.modTypeId = INF_MOD;
		mod.moduleSlot = slotIndex;
		DEBUGF("%s: module slot: %d\n",__func__,slotIndex+1);
		_nai_read_module_eeprom(&mod);
		if( false == mod.moduleRdy){
			/*no module 
			 * module size to 0x00000000 
			 * module id to 0x00000000 */
			 writel(0xFFFFFFFF, (PS2FPGA_MB_COMMON_MOD_SLOT1_ADDR_OFFSET+(slotIndex*4)));
			 writel(0x0, (PS2FPGA_MB_COMMON_MOD_SLOT1_SIZE_OFFSET+(slotIndex*4)));
			
			/* If module is not a "special" module...then no module was really detected */
			if ((mod.infModule.modEepromData.gpioConfig & 0x01) == 0) {
				 DEBUGF("%s no module\n",__func__);
				 modId = MODULE_ID_NONE | MODULE_ID_SPACE;
				 writel(modId, (PS2FPGA_MB_COMMON_MOD_SLOT1_ID_OFFSET+(slotIndex*4)));
			}
			else{
				 /*Module found to be "special" module so store modId */
				 modId |= mod.infModule.modEepromData.moduleId[0] << 24;
				 modId |= mod.infModule.modEepromData.moduleId[1] << 16;
				 modId |= mod.infModule.modEepromData.moduleId[2] << 8;
				 modId |= mod.infModule.modEepromData.moduleId[3]<< 0;
				 DEBUGF("Special Module: %s: mod_id:0x%08x\n",__func__,modId);
				 modId |= MODULE_ID_SPACE;
				 writel(modId, (PS2FPGA_MB_COMMON_MOD_SLOT1_ID_OFFSET+(slotIndex*4)));
				 
				 /*set module GPIOs*/
				 _nai_set_module_gpios(&mod);	
				 
				 /* Turn on reference clock  NOTE: normal modules do this at power on but "special" modules need to make sure the reference 
				    clock was turned on in case it is the only module on the board */
				 _nai_enable_refClk();
			}
		}else{			
			/*get module size*/
			modSize = mod.infModule.modEepromData.moduleSize;	
			modId |= mod.infModule.modEepromData.moduleId[0] << 24;
			modId |= mod.infModule.modEepromData.moduleId[1] << 16;
			modId |= mod.infModule.modEepromData.moduleId[2] << 8;
			modId |= mod.infModule.modEepromData.moduleId[3]<< 0;
			DEBUGF("%s: mod_id:0x%08x mod_size:0x%08x\n",__func__,modId,modSize);
			/*
			 * 1. module id = CTS
			 * module address to 0xffffffff
			 * module size to 0x0 
			 * module id to 0x0
			 */
			 if(MODULE_ID_CTS == modId){
				writel(0xFFFFFFFF, (PS2FPGA_MB_COMMON_MOD_SLOT1_ADDR_OFFSET+(slotIndex*4)));
				writel(0x0, (PS2FPGA_MB_COMMON_MOD_SLOT1_SIZE_OFFSET+(slotIndex*4)));
				//add space char
				modId |= MODULE_ID_SPACE;
				writel(modId, (PS2FPGA_MB_COMMON_MOD_SLOT1_ID_OFFSET+(slotIndex*4)));
			 }else if((modSize == 0xFFFFFFFF || modSize == 0x0) 
						|| (modId == 0xFFFFFFFF || modId == 0x0)){
				/* 
				  * 2. module id and size  is 0xffffffff/0x0
				  * module address to 0xffffffff
				  * module size to 0x0 
				  * module id to 'NONE '  
				  */
				writel(0xFFFFFFFF, (PS2FPGA_MB_COMMON_MOD_SLOT1_ADDR_OFFSET+(slotIndex*4)));
				writel(0x0, (PS2FPGA_MB_COMMON_MOD_SLOT1_SIZE_OFFSET+(slotIndex*4)));
				modId = MODULE_ID_NONE | MODULE_ID_SPACE;
				writel(modId, (PS2FPGA_MB_COMMON_MOD_SLOT1_ID_OFFSET+(slotIndex*4)));
			 }else{
				/*
				* 3. 
				* module address to [last_module_address+last_module_size]
				* module size to [module_size] 
				* module id to ['mod_id ']  
				*/
				modAddr += lastSize;
				writel(modAddr, (PS2FPGA_MB_COMMON_MOD_SLOT1_ADDR_OFFSET+(slotIndex*4)));
				writel(modSize, (PS2FPGA_MB_COMMON_MOD_SLOT1_SIZE_OFFSET+(slotIndex*4)));
				modId |= MODULE_ID_SPACE;
				writel(modId, (PS2FPGA_MB_COMMON_MOD_SLOT1_ID_OFFSET+(slotIndex*4)));
				lastSize = modSize;
			 }//if
		}//if

	}//for 
	
	/*set module address rdy flag*/
	writel(MODULE_ADDR_RDY,PS2FPGA_MB_COMMON_MOD_SLOT_ADDR_RDY_OFFSET);
}

static void _nai_module_addr_mask(){
	
	u32	slotIndex = 0;
	u32	data = 0;
	u32	modSize = 0;
	u32	modStartAddr = 0;
	u32	modEndAddr = 0;
//include the first 16KB of MB common memory 
	u32	prevEndAddr = (PS2FPGA_MB_COMMON_SIZE - 1);
	u32	totalSize = PS2FPGA_MB_COMMON_SIZE;
	Nai_mod mod;
	
	//config all module slot
	for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
		memset(&mod, 0, sizeof(mod));
		mod.modTypeId = INF_MOD;
		mod.moduleSlot = slotIndex;
		DEBUGF("%s: module slot: %d\n",__func__,slotIndex+1);
		_nai_read_module_eeprom(&mod);
		if( false == mod.moduleRdy){
			/*set default module addr*/
			DEBUGF("%s: module id: %d NOT ready\n",__func__, slotIndex);		
			data = ((MODULE_DEFAULT_ADDR << MODULE_ADDR_START_SHIFT) |
				   (MODULE_DEFAULT_ADDR  << MODULE_ADDR_END_SHIFT) | 0x3);
				
			DEBUGF("%s: module addr: 0x%08x\n",__func__,data);
			writel(data, MODULE_ADDR_OFFSET(slotIndex));
			
			/*set default module mask*/
			data = MODULE_DEFAULT_MASK;
			DEBUGF("%s: module mask: 0x%08x\n",__func__,data);
			writel(data, MODULE_MASK_OFFSET(slotIndex));
		}else{	
			/*get module size*/
			modSize = mod.infModule.modEepromData.moduleSize;			
			
			DEBUGF("%s: module id: %s \n",__func__,mod.infModule.modEepromData.moduleId);				
			DEBUGF("%s: module size: 0x%08x\n",__func__,modSize);
			
			/*check module size*/
			modSize = _nai_chk_module_size_range(modSize);
			
			totalSize += modSize;
						
			/*check module size is not out of range*/
			if( DEFAULT_MODULE_MAX_MASK_SIZE_LIMIT < totalSize ){
				
				/*reach max module size limit*/
				DEBUGF("%s:Reach max module size limit Totalsize: 0x%08x\n",__func__,totalSize);
				
				/*set default module addr*/
				data = ((MODULE_DEFAULT_ADDR << MODULE_ADDR_START_SHIFT) |
				(MODULE_DEFAULT_ADDR  << MODULE_ADDR_END_SHIFT) | 0x3);
			   
				writel(data, MODULE_ADDR_OFFSET(slotIndex));
				/*set default module mask*/
				data = MODULE_DEFAULT_MASK;
				writel(data, MODULE_MASK_OFFSET(slotIndex));
				totalSize -= modSize;
				//continue with next module
				continue;
			}
			
			/*get module start address*/
				/*take previous module end address*/
			modStartAddr = (prevEndAddr + 1);
			DEBUGF("%s: module start addr: 0x%08x\n",__func__,modStartAddr);
			
			/*get module end address*/
				/*take current module start address + module size - 0x1*/
			modEndAddr = (modStartAddr + (modSize - 1));
			prevEndAddr = modEndAddr;
			DEBUGF("%s: module end addr: 0x%08x\n",__func__,modEndAddr);
			/*bit shift module start/end address*/
			/*take bit 23 ~ 14 from module start address*/
			modStartAddr = ((0xFFC000 & modStartAddr) >> 14);	
			/*take bit 23 ~ 14 from module end address*/
			modEndAddr = ((0xFFC000 & modEndAddr) >> 14);				
			
			/*set module start and end address*/
				/*start address bit 31 ~ 18 */
				/*end address bit 11 ~ 2 */
			//DEBUGF("%s: module start addr: 0x%08x\n",__func__,(modStartAddr << MODULE_ADDR_START_SHIFT));
			//DEBUGF("%s: module end addr: 0x%08x\n",__func__,(modEndAddr << MODULE_ADDR_END_SHIFT));
			data = ((modStartAddr << MODULE_ADDR_START_SHIFT) | 
					(modEndAddr << MODULE_ADDR_END_SHIFT));
			DEBUGF("%s: module addr: 0x%08x 0x%08x\n",__func__,data, MODULE_ADDR_OFFSET(slotIndex));
			writel(data, MODULE_ADDR_OFFSET(slotIndex));
			
			/*
			 * set module mask size
			 */
			data = _nai_cal_module_mask(modSize);
			DEBUGF("%s: module mask: 0x%08x 0x%08x\n",__func__,data, MODULE_MASK_OFFSET(slotIndex));
			writel(data, MODULE_MASK_OFFSET(slotIndex)); 

		}
	}//end for loop
	
	/*set all module mask size*/
	data = _nai_cal_module_mask(totalSize);
	DEBUGF("%s:all module mask: 0x%08x\n",__func__,data);
	writel(data, PS2FPGA_ALL_MODULE_ADDR_MASK_OFFSET); 
		
}

/* 
 * Init All Modules
 */
void nai_init_module_board(){

printf("-------------------------------------\n");
#if defined(CONFIG_NAI_1U_2U_3U_MB)
	_nai_1u_2u_3u_mb_init_module();
#elif defined(CONFIG_NAI_6U_MB)
	_nai_6u_mb_init_module();
#else
	printf("MB Init sequence not define \n");			
#endif
	
	//log found module slot and module serdes link ready to MB Common area
	_nai_log_module_status();
	
	DEBUGF("Found Module on Slot gModuleFound: 0x%02x \n", gModuleFound);
	DEBUGF("Module gModulePwrEn BitMask 0x%02x \n", gModulePwrEn);
	DEBUGF("Module gModuleDetectDone BitMask 0x%02x \n", gModuleDetectDone);
	DEBUGF("Module gModuleHSSLinkReady BitMask 0x%02x \n", gModuleHSSLinkReady);
	
printf("-------------------------------------\n");
	
}



/* 
 * Enable Module(s)
 */
s32 nai_enable_module_board(u8 moduleSlot){
	
	s32 ret = NAI_MODULE_SUCCESS;
	u32 slotIndex = 0;
	
	if( ((moduleSlot < NAI_MODULE_SLOT_1) || (moduleSlot > MAX_MODULE_SLOT)) 
			&&  (moduleSlot != NAI_MODULE_ALL_SLOT)
			&& !gModuleFound ) {
		
		if(!gModuleFound){
			printf("No Module has been initialized: gModuleFound=0x%02x \n", gModuleFound);
		}else{
			printf("NAI Invalid module slot %d \n", moduleSlot+1);
		}
		
		
		ret = NAI_MODULE_ERROR;
		goto EXIT;
	}
	
	printf("-------------------------------------\n");
	printf("Enable Module \n");
	if( NAI_MODULE_ALL_SLOT == moduleSlot ){
		//Enable all available modules slot
		for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
				_nai_enable_module(slotIndex);
		}
	
		//Check Module Detect and Ready
		for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
			if((gModuleFound & (1 << slotIndex)) && (gModulePwrEn & (1 << slotIndex))){
				if( NAI_MODULE_SUCCESS == _nai_module_detect_done(slotIndex)){
					gModuleDetectDone |= (1 << slotIndex);
				}else{
					gModuleDetectDone &= ~(1 << slotIndex);
				}
			}
		}
			
		//Start TODO: Fix HSS WR: Remove the WR
		for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
			
			if((gModuleDetectDone & (1 << slotIndex))){
				//release hss from reset
				_nai_module_hss_reset(1, slotIndex);
				
				//Verify HSS Link Ready
				if(NAI_MODULE_SUCCESS == _nai_module_link_init_done(slotIndex)){
					gModuleHSSLinkReady |= (1 << slotIndex);
					/*check module FW is rdy*/
					_nai_is_module_fw_rdy(slotIndex, MODULE_OPERSTATE_FW_ENTERED_BIT);
				}else{
					gModuleHSSLinkReady &= ~(1 << slotIndex);
				}
			}
		}
		//END TODO: Fix HSS WR:
		 					
	} else {
				
		_nai_enable_module(moduleSlot);
			
		//Check Module Detect and Ready
		if((gModuleFound & (1 << moduleSlot)) && (gModulePwrEn & (1 << moduleSlot))){
			if( NAI_MODULE_SUCCESS == _nai_module_detect_done(moduleSlot)){
				gModuleDetectDone |= (1 << moduleSlot);
			}else{
				gModuleDetectDone &= ~(1 << moduleSlot);
			}
		}
		
		//Start TODO: Fix HSS WR: Remove the WR			
		if((gModuleDetectDone & (1 << moduleSlot))){
			//release hss from reset
			_nai_module_hss_reset(1, moduleSlot);
						
			//Verify HSS Link Ready
			if(NAI_MODULE_SUCCESS == _nai_module_link_init_done(moduleSlot)){
				gModuleHSSLinkReady |= (1 << moduleSlot);
				/*check module FW is rdy*/
				_nai_is_module_fw_rdy(moduleSlot, MODULE_OPERSTATE_FW_ENTERED_BIT);
			}else{
				gModuleHSSLinkReady &= ~(1 << moduleSlot);
			}
		}
		//END TODO: Fix HSS WR:
	}
	
	//log found module slot and module serdes link ready to MB Common area
	_nai_log_module_status();
	
	DEBUGF("Found Module on Slot gModuleFound: 0x%02x \n", gModuleFound);
	DEBUGF("Module gModulePwrEn BitMask 0x%02x \n", gModulePwrEn);
	DEBUGF("Module gModuleDetectDone BitMask 0x%02x \n", gModuleDetectDone);
	DEBUGF("Module gModuleHSSLinkReady BitMask 0x%02x \n", gModuleHSSLinkReady);
	
	printf("-------------------------------------\n");

EXIT:	
	
	return ret;
}

/* 
 * Disable Module(s)
 */
s32 nai_disable_module_board(u8 moduleSlot){
	
	s32 ret = NAI_MODULE_SUCCESS;
	u32 slotIndex = 0;
	
	if( ((moduleSlot < NAI_MODULE_SLOT_1) || (moduleSlot > MAX_MODULE_SLOT)) 
			&&  (moduleSlot != NAI_MODULE_ALL_SLOT)
			&& !gModuleFound ) {
		
		if(!gModuleFound){
			printf("No Module has been initialized: gModuleFound=0x%02x \n", gModuleFound);
		}else{
			printf("NAI Invalid module slot %d \n", moduleSlot+1);
		}
		
		ret = NAI_MODULE_ERROR;
		goto EXIT;
	}
	
	printf("-------------------------------------\n");
	printf("Disable Module \n");
	//disable all slots
	if ( NAI_MODULE_ALL_SLOT == moduleSlot ){ 		
		//Power off all module slot
		for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
			_nai_disable_module(slotIndex);
		}	
	}else{	
		_nai_disable_module(moduleSlot);
	}
	
	//log found module slot and module serdes link ready to MB Common area
	_nai_log_module_status();
	
	DEBUGF("Found Module on Slot gModuleFound: 0x%02x \n", gModuleFound);
	DEBUGF("Module gModulePwrEn BitMask 0x%02x \n", gModulePwrEn);
	DEBUGF("Module gModuleDetectDone BitMask 0x%02x \n", gModuleDetectDone);
	DEBUGF("Module gModuleHSSLinkReady BitMask 0x%02x \n", gModuleHSSLinkReady);
	printf("-------------------------------------\n");

EXIT:	
	return ret;
}

/* 
 * Reset Module(s)
 */
s32 nai_module_reset(u8 moduleSlot) {

	s32 ret = NAI_MODULE_SUCCESS;
	u32 slotIndex = 0;

	if( ((moduleSlot < NAI_MODULE_SLOT_1) || (moduleSlot > MAX_MODULE_SLOT)) 
			&&  (moduleSlot != NAI_MODULE_ALL_SLOT)
			&& !gModuleFound ) {
		
		if(!gModuleFound){
			printf("No Module has been initialized: gModuleFound=0x%02x \n", gModuleFound);
		}else{
			printf("NAI Invalid module slot %d \n", moduleSlot+1);
		}
		
		ret = NAI_MODULE_ERROR;
		goto EXIT;
	}
	
	printf("-------------------------------------\n");
	printf("Reset Module \n");
	if( moduleSlot == NAI_MODULE_ALL_SLOT){
		for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
			_nai_module_reset_seq(slotIndex);
		}
		
		//Start TODO: Fix HSS WR: Remove the WR
		for(slotIndex = 0; slotIndex < MAX_MODULE_SLOT; slotIndex++){
			
			if((gModuleDetectDone & (1 << slotIndex))){
				//release hss from reset
				_nai_module_hss_reset(1, slotIndex);
				
				//Verify HSS Link Ready
				if(NAI_MODULE_SUCCESS == _nai_module_link_init_done(slotIndex)){
					gModuleHSSLinkReady |= (1 << slotIndex);
					/*check module FW is rdy*/
					_nai_is_module_fw_rdy(slotIndex, MODULE_OPERSTATE_FW_ENTERED_BIT);
				}else{
					gModuleHSSLinkReady &= ~(1 << slotIndex);
				}
			}
			
		}
		//END TODO: Fix HSS WR:
		
	}else{
		_nai_module_reset_seq(moduleSlot);
		
		//Start TODO: Fix HSS WR: Remove the WR			
		if((gModuleDetectDone & (1 << moduleSlot))){
			//release hss from reset
			_nai_module_hss_reset(1, moduleSlot);
			
			//Verify HSS Link Ready
			if(NAI_MODULE_SUCCESS == _nai_module_link_init_done(moduleSlot)){
				gModuleHSSLinkReady |= (1 << moduleSlot);
				/*check module FW is rdy*/
				_nai_is_module_fw_rdy(moduleSlot, MODULE_OPERSTATE_FW_ENTERED_BIT);
			}else{
				gModuleHSSLinkReady &= ~(1 << moduleSlot);
			}
		}
		//END TODO: Fix HSS WR:
	}
	
	//log found module slot and module serdes link ready to MB Common area
	_nai_log_module_status();
	
	
	DEBUGF("Found Module on Slot gModuleFound: 0x%02x \n", gModuleFound);
	DEBUGF("Module gModulePwrEn BitMask 0x%02x \n", gModulePwrEn);
	DEBUGF("Module gModuleDetectDone BitMask 0x%02x \n", gModuleDetectDone);
	DEBUGF("Module gModuleHSSLinkReady BitMask 0x%02x \n", gModuleHSSLinkReady);
	printf("-------------------------------------\n");
EXIT:
	return ret;
}

u32 nai_chk_module_size_range(u32 modSize){
	return _nai_chk_module_size_range(modSize);
}

s32 nai_get_module_eeprom_info(Nai_mod *mod)
{
	s32 ret = NAI_MODULE_SUCCESS;
	u8 slotId = mod->moduleSlot;
	
	if( (slotId < NAI_MODULE_SLOT_1) || (slotId > MAX_MODULE_SLOT) ){
		printf("Invalid module slot %02d \n", slotId+1);
		ret =  NAI_MODULE_ERROR;
		goto EXIT;
	}
	
	if(	mod->modTypeId == INF_MOD){
		if( NAI_MODULE_ERROR == _nai_read_module_eeprom(mod)){
			//printf("Failed to read module [%d] eeprom data\n", slotId+1);
			ret = NAI_MODULE_ERROR;
		}
	}else{
		printf("This module type [%d] is not supported yet\n", mod->modTypeId);
		ret = NAI_MODULE_ERROR;
	}

EXIT:	
	return ret;
}

void nai_get_module_info(Nai_mod *mod)
{
	u8 slotId = mod->moduleSlot;
	u32 data = 0;
	s32 status = NAI_SUCCESS;
	u8 buf[BUILD_TIME_STRING_LEN];
	u16 modulePhy = 0;
	u16 modulePowerOn = 0;
	u16 moduleHSSRdy = 0;
	mod->moduleRdy = false;
	u32 loopCount = 0;
	
	if( (slotId < NAI_MODULE_SLOT_1) || (slotId > MAX_MODULE_SLOT) ){
		printf("Invalid module slot %02d \n", slotId+1);
		return;
	}
	
	if(	mod->modTypeId == INF_MOD){
		//check there is a module on the slot
		data = readl(PS2FPGA_MB_COMMON_MODULE_STATUS_OFFSET);
		//TODO: I should check both module phy connection and HSS link ready
		//waiting KF to read HSS Link ready & done register to all MB FPGA
		//in the meantime check only the phy bitmask
		//31 ~ 26 bits - Bitmap for each module phyical connection status (we will use I2C to probe each module slot)
		//27 ~ 16 bits - Bitmap for each module Power status 
		//8 ~ 0 bits - Bitmap for each module serdes ready status
		modulePhy = (((data >> 24) >> slotId) & 0x1);
		modulePowerOn = (((data >> 16) >> slotId) & 0x1);
		moduleHSSRdy = ((data >> slotId) & 0x1);
		if(modulePhy && 
			modulePowerOn && 
				moduleHSSRdy){
			
			if(!_nai_is_module_fw_rdy(slotId,MODULE_OPERSTATE_FW_COMMONPOPULATED_BIT)){
				goto EXIT;
			}
			
			//module id
			status = nai_read_reg32_by_slot_request((u8)(slotId+1), (u32)MODULE_INF_ID, &data);
			if (status != NAI_SUCCESS){
				goto MODULE_ERROR;
			}
			
			/*
			* The moduleId is stored  
			* as little endian in the module common memory area
			*/
			//mod->infModule.moduleId = cpu_to_be32(data);
			data = cpu_to_be32(data);
		
			buf[0] = (data >> 24); 
			buf[1] = (data >> 16); 
			buf[2] = (data >> 8); 
			buf[3] = data; 
			strncpy(mod->infModule.modEepromData.moduleId, (char *)buf, MODULE_ID_STRING_LEN);
			
			//module size
			status = nai_read_reg32_by_slot_request((u8)(slotId+1),  (u32)MODULE_INF_MEM_SIZE, &data);
			if (status != NAI_SUCCESS){	
				goto MODULE_ERROR;
			}
			
			/*
			* The moduleSize is stored
			* as little endian in the module common memory area
			*/
			mod->infModule.modEepromData.moduleSize =  cpu_to_be32(data);
		
			//fsbl
			status = nai_read_reg32_by_slot_request((u8)(slotId+1),  (u32)MODULE_FSBL_REVISION, &data);
			if (status != NAI_SUCCESS){
				goto MODULE_ERROR;
			}
			mod->infModule.version.fsblRevHiLo = ((data >> 16) & 0xFFFF); 
			mod->infModule.version.fsblRevPatchLevel = ((data >> 8) & 0xFF) ;
			mod->infModule.version.fsblRevSubLevel = (data & 0xFF);
			
			status = nai_read_block32_by_slot_request((u8)(slotId+1), (u32)MODULE_FSBL_BUILD_TIME, (BUILD_TIME_STRING_LEN/4), 4, (u32 *)buf);
			if (status != NAI_SUCCESS){
				goto MODULE_ERROR;
			}
			strncpy(mod->infModule.version.fsblBuildTimeStr, (char *)buf, BUILD_TIME_STRING_LEN);
			
			//uboot		
			status = nai_read_reg32_by_slot_request((u8)(slotId+1),  (u32)MODULE_UBOOT_REVISION, &data);
			if (status != NAI_SUCCESS){
				goto MODULE_ERROR;
			}
			mod->infModule.version.ubootRevHiLo = ((data >> 16) & 0xFFFF); 
			mod->infModule.version.ubootRevPatchLevel = ((data >> 8) & 0xFF) ;
			mod->infModule.version.ubootRevSubLevel = (data & 0xFF);
			
			status = nai_read_block32_by_slot_request((u8)(slotId+1), (u32)MODULE_UBOOT_BUILD_TIME, (BUILD_TIME_STRING_LEN/4), 4, (u32 *)buf);
			if (status != NAI_SUCCESS){
				goto MODULE_ERROR;
			}
			strncpy(mod->infModule.version.ubootBuildTimeStr, (char *)buf, BUILD_TIME_STRING_LEN);
			
			//fw
			status =  nai_read_reg32_by_slot_request((u8)(slotId+1),  (u32)MODULE_FW_REVISION, &data);
			if (status != NAI_SUCCESS){
				goto MODULE_ERROR;
			}
			
			mod->infModule.version.fwRevMajor = ((data >> 8) & 0xFF);
			mod->infModule.version.fwRevMinor = (data & 0xFF);
			
			status = nai_read_block32_by_slot_request((u8)(slotId+1), (u32)MODULE_FW_BUILD_TIME, (BUILD_TIME_STRING_LEN/4), 4, (u32 *)buf);
			if (status != NAI_SUCCESS){
				goto MODULE_ERROR;
			}
			strncpy(mod->infModule.version.fwBuildTimeStr, (char *)buf, BUILD_TIME_STRING_LEN);
			
			//fpga
			status = nai_read_reg32_by_slot_request((u8)(slotId+1),  (u32)MODULE_FPGA_REVISION, &data);
			if (status != NAI_SUCCESS){
				goto MODULE_ERROR;
			}
			
			mod->infModule.version.fpgaRev = data;
			
			status = nai_read_reg32_by_slot_request((u8)(slotId+1), (u32)MODULE_FPGA_BUILD_TIME, &data);
			if (status != NAI_SUCCESS){
				goto MODULE_ERROR;
			}
			
			mod->infModule.version.fpgaBuildTime = data;
			nai_decode_fpga_compile_count(&mod->infModule.version.fpgaDecodedBuildTime, mod->infModule.version.fpgaBuildTime);
			
			/* Only set moduleRdy flag if module is not detected to be a "special" module */
			if ((mod->infModule.modEepromData.gpioConfig & 0x01) == 0) {
				//set module info ready
				mod->moduleRdy = true;
			}
		}//if	
	}else{
		printf("This module type [%d] is not supported yet\n", mod->modTypeId);
	}//if

MODULE_ERROR:
	if (status != NAI_SUCCESS){
		printf("Failed to communicated with module %d [Status=%d retry=%d]\n",slotId+1,status,loopCount);
	}
				
EXIT:
	return;
}

void nai_chk_mod_rdy_state(u8 slotId){
	
	int retry = 0;
	
	if( (slotId < NAI_MODULE_SLOT_1) || (slotId > MAX_MODULE_SLOT) ){
		printf("Invalid module slot %02d \n", slotId+1);
		return;
	}
	
	if (gModuleSpecialGPIO & (1 << slotId)){
		printf("Special GPIO configuration detected, module slot %02d will not be checked for ready state!\n", slotId+1);
		return;
	}
	
	/*TODO: How are we going to handle module without FW*/
	while(retry < MOD_RECOVERY_COUNT){
				
		if( (gModuleFound & (1 << slotId)) && !(gModuleRdy & (1 << slotId)) ){	
				printf("module %d not ready \n", slotId+1);
				printf("resetting module %d \n", slotId+1);
				nai_module_reset(slotId);
		}else{
			break;
		}		
		retry++; 
	};
	
	if(retry >= MOD_RECOVERY_COUNT)
		printf("failed to recovery module %d \n", slotId+1);
	
	_nai_log_module_status();	
}
