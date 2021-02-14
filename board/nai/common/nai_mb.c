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
#include <version.h>
#include <nai_mb_fpga_address.h>
#include <nai_common.h>
#include <nai_mb.h>
#include <nai_icb.h>
#include <master_slave.h>

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define DEBUGF(x...) printf(x)
#else
#define DEBUGF(x...)
#endif /* DEBUG */



enum revision_id
{
    TOP_MB_REV = 0,
    TOP_PS_REV,
    TOP_MODULE_REV,	
    TOP_SATA_REV,
    TOP_VME_REV,
    TOP_VME_IP_REV,
    TOP_PCIE_REV,
    TOP_CPCI_REV,
    TOP_MB_COMPILE_COUNT
};

enum mst_ver_type
{
    MST_TLVL = 0,
    MST_MBEX,
    MST_BH,
    MST_FSBL,
    MST_FPGA,
    MST_UBOT,
    MST_FW,
    MST_KRNK,
    MST_RTFS,
    MST_DTB,
    MST_BROM,
    MST_DEBT,
    MST_TYPE_MAX
};

enum slv_ver_type
{
    SLV_TLVL = 0,
    SLV_BH,
    SLV_FSBL,
    SLV_FPGA,
    SLV_UBOT,
    SLV_FW,
    SLV_TYPE_MAX
};

/*MB version 0x43c41600*/
typedef volatile struct {
    struct {
        char procid[4]; 
	char type[4];
        u16  major;
	u16  minor;
	char year[4];
	char month[2];
	char day[2];
	char hh[2];
	char mm[2];
	char ss[2];
	char tt[2];
	u32 _reserved[5];
    } mb_mst_version[MST_TYPE_MAX]; /*mb master arm0: 0x1600 0x183C*/
    u32 _reservered[240]; /*reserved: 0x1840 ~ 0x1BFC*/
    struct {
        char procid[4];
	char type[4];
        u16 major;
	u16 minor;
	char year[4];
	char month[2];
	char day[2];
	char hh[2];
	char mm[2];
	char ss[2];
	char tt[2];
	u32 _reserved[5];
    } mb_slv_version[6];

} MB_VERSION_REG;

/*ab month name*/
static char const ab_month_name[][4] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

/*global variable */
MB_VERSION_REG *pMBVersion = (MB_VERSION_REG *) PS2FPGA_MB_VERSION_BASE_ADDRESS;

static u32 _get_mb_fpga_revision(u32 revision_id);
static void _update_fpga_rev(void);
static void _update_fsbl_rev(void);
#ifdef CONFIG_NAI_ZYNQ_SLAVE
static void _update_fw_rev(void);
#endif
static void _update_uboot_rev(void);


void nai_update_mb_version(){
	/*FSBL Rev & Build Time*/	
	_update_fsbl_rev();
	/*UBOOT Rev & Build Time*/
	_update_uboot_rev();
	 
	//FPGA Rev & Build Time
	_update_fpga_rev();
	
	//SLAVE FW Rev & Build Time
#ifdef CONFIG_NAI_ZYNQ_SLAVE
	_update_fw_rev();
#endif
}

void nai_get_mb_info(Nai_mb* mb)
{
	u32	data;
	
	/*Get Motherboard master CPU*/
	/*fsbl*/
	data = readl(PS2FPGA_MB_COMMON_FSBL_REV_OFFSET);
	mb->masterVer.fsblRevHiLo = ((data >> 16) & 0xFFFF); 
	mb->masterVer.fsblRevPatchLevel = ((data >> 8) & 0xFF) ;
	mb->masterVer.fsblRevSubLevel = (data & 0xFF);
	mb->masterVer.fsblMajor = pMBVersion->mb_mst_version[MST_FSBL].major;
	mb->masterVer.fsblMinor = pMBVersion->mb_mst_version[MST_FSBL].minor;
	strncpy(mb->masterVer.fsblBuildTimeStr, (char *)PS2FPGA_MB_COMMON_FSBL_BUILD_TIME_OFFSET, BUILD_TIME_STRING_LEN);
	
	/*uboot*/
	data = readl(PS2FPGA_MB_COMMON_UBOOT_REV_OFFSET);
	mb->masterVer.ubootRevHiLo = ((data >> 16) & 0xFFFF); 
	mb->masterVer.ubootRevPatchLevel = ((data >> 8) & 0xFF) ;
	mb->masterVer.ubootRevSubLevel = (data & 0xFF);
	mb->masterVer.ubootMajor = pMBVersion->mb_mst_version[MST_UBOT].major;
	mb->masterVer.ubootMinor = pMBVersion->mb_mst_version[MST_UBOT].minor;
	strncpy(mb->masterVer.ubootBuildTimeStr, (char *)PS2FPGA_MB_COMMON_UBOOT_BUILD_TIME_OFFSET, BUILD_TIME_STRING_LEN);
	
	/*fpga*/
	mb->masterVer.fpgaRev = readl(PS2FPGA_MB_COMMON_FPGA_REV_OFFSET);
	mb->masterVer.fpgaBuildTime = readl(PS2FPGA_MB_COMMON_FPGA_BUILD_TIME_OFFSET);
	
	nai_decode_fpga_compile_count(&mb->masterVer.fpgaDecodedBuildTime, mb->masterVer.fpgaBuildTime);

#ifdef CONFIG_NAI_ZYNQ_SLAVE	
	/*Get Motherboard slave CPU*/
	/*fsbl*/
	data = readl(PS2FPGA_MB_COMMON_SLAVE_FSBL_REV_OFFSET);
	mb->slaveVer.fsblRevHiLo = ((data >> 16) & 0xFFFF); 
	mb->slaveVer.fsblRevPatchLevel = ((data >> 8) & 0xFF) ;
	mb->slaveVer.fsblRevSubLevel = (data & 0xFF);
	strncpy(mb->slaveVer.fsblBuildTimeStr, (char *)PS2FPGA_MB_COMMON_SLAVE_FSBL_BUILD_TIME_OFFSET, BUILD_TIME_STRING_LEN);
	mb->slaveVer.fsblMajor = pMBVersion->mb_slv_version[SLV_FSBL].major;
	mb->slaveVer.fsblMinor = pMBVersion->mb_slv_version[SLV_FSBL].minor;
	/*uboot*/
	data = readl(PS2FPGA_MB_COMMON_SLAVE_UBOOT_REV_OFFSET);
		
	mb->slaveVer.ubootRevHiLo = ((data >> 16) & 0xFFFF); 
	mb->slaveVer.ubootRevPatchLevel = ((data >> 8) & 0xFF) ;
	mb->slaveVer.ubootRevSubLevel = (data & 0xFF);
	mb->slaveVer.ubootMajor = pMBVersion->mb_slv_version[SLV_UBOT].major;
	mb->slaveVer.ubootMinor = pMBVersion->mb_slv_version[SLV_UBOT].minor;
	strncpy(mb->slaveVer.ubootBuildTimeStr, (char *)PS2FPGA_MB_COMMON_SLAVE_UBOOT_BUILD_TIME_OFFSET, BUILD_TIME_STRING_LEN);
	
	/*fpga*/
	mb->slaveVer.fpgaRev = readl(PS2FPGA_MB_COMMON_SLAVE_FPGA_REV_OFFSET);
	mb->slaveVer.fpgaBuildTime = readl(PS2FPGA_MB_COMMON_SLAVE_FPGA_BUILD_TIME_OFFSET);
	nai_decode_fpga_compile_count(&mb->slaveVer.fpgaDecodedBuildTime, mb->slaveVer.fpgaBuildTime);
	
	/*fw*/
	data = readl(PS2FPGA_MB_COMMON_SLAVE_FW_REV_OFFSET);
	mb->slaveVer.fwRevMajor = ((data >> 16) & 0xFFFF); 
	mb->slaveVer.fwRevMinor = (data & 0xFFFF) ;
	strncpy(mb->slaveVer.fwBuildTimeStr, (char *)PS2FPGA_MB_COMMON_SLAVE_FW_BUILD_TIME_OFFSET, BUILD_TIME_STRING_LEN);
#endif
	
}

static char* _get_ascii_mm(const char* month){
    
    int cnt;
    static char local[3];
    char *p = &local[0];
    
    for(cnt = 0; cnt < 12; cnt++){
      if(strncasecmp(month, ab_month_name[cnt], 3)  == 0){
          snprintf(p,sizeof(p),"%02d", (cnt + 1));
	  return p;	  
      }
    }

    return NULL;
}


static void _update_fsbl_rev(void){

#if defined(CONFIG_NAI_ZYNQ_SLAVE) && defined(CONFIG_NAI_ICB_MST)
	s32 ret = 0;
#endif	
	char revStr[CONFIG_VERSION_STRING_LEN];
	char tempStr[CONFIG_VERSION_STRING_LEN];
	u32 data = 0;

	/*UBOOT Rev & Build Time*/
	
	/*Get Master UBOOT rev from OCRAM */
	/* parse revison string,
	 * 2014.00.00. 
	 * 
	 */
	strncpy(revStr, (char *)CONFIG_FSBL_VERSION_STRING, CONFIG_VERSION_STRING_LEN);

	DEBUGF("FSBL Build %s \n", revStr);
	/*parse high_rev low_rev: 2014*/
	tempStr[0] = revStr[0];
	tempStr[1] = revStr[1];
	tempStr[2] = revStr[2];	
	tempStr[3] = revStr[3];
	tempStr[4] = '\0';
	
	data |= (simple_strtoul(tempStr, NULL, 10) << 16);
	
	/*parse patch level: 00 */
	tempStr[0] = revStr[5];
	tempStr[1] = revStr[6];
	tempStr[2] = '\0';
	data |= (simple_strtoul(tempStr, NULL, 10) << 8);
	
	/*parse sub-level: 00*/
	tempStr[0] = revStr[8];
	tempStr[1] = revStr[9];
	tempStr[2] = '\0';
	data |= (simple_strtoul(tempStr, NULL, 10));
	
	DEBUGF("FSBL Rev 0x%08x \n", data);
	/*Store FSBL Rev & Build Time into MB Common memory area */
	writel(data, PS2FPGA_MB_COMMON_FSBL_REV_OFFSET);
	/*store revision new memory location and new format*/		
	strncpy((char *)pMBVersion->mb_mst_version[MST_FSBL].procid, "ARM0", 4);
	strncpy((char *)pMBVersion->mb_mst_version[MST_FSBL].type, "FSBL", 4);
	/*major*/
	strncpy(tempStr, &revStr[35], 5);
	tempStr[5] = '\0';
	pMBVersion->mb_mst_version[MST_FSBL].major = simple_strtoul(tempStr,NULL,10);
	/*minor*/
	strncpy(tempStr, &revStr[41], 5);
	tempStr[5] = '\0';
	pMBVersion->mb_mst_version[MST_FSBL].minor = simple_strtoul(tempStr, NULL, 10);
        /*build time*/	
	strncpy(tempStr, &revStr[11], BUILD_TIME_STRING_LEN);
	tempStr[BUILD_TIME_STRING_LEN-1] = '\0';
	strncpy((char *)PS2FPGA_MB_COMMON_FSBL_BUILD_TIME_OFFSET, tempStr, BUILD_TIME_STRING_LEN);
	/*month MM*/
	strncpy((char *)pMBVersion->mb_mst_version[MST_FSBL].month, _get_ascii_mm(tempStr), 2);
	/*day DD*/
	strncpy((char *)pMBVersion->mb_mst_version[MST_FSBL].day, &tempStr[4], 2);
	/*year yyyy*/
	strncpy((char *)pMBVersion->mb_mst_version[MST_FSBL].year, &tempStr[7], 4);
	/*hour hh*/
	strncpy((char *)pMBVersion->mb_mst_version[MST_FSBL].hh, &tempStr[15], 2);
	/*minute mm*/
	strncpy((char *)pMBVersion->mb_mst_version[MST_FSBL].mm, &tempStr[18], 2);
	/*second ss*/
	strncpy((char *)pMBVersion->mb_mst_version[MST_FSBL].ss, &tempStr[21], 2);

#if defined(CONFIG_NAI_ZYNQ_SLAVE) && defined(CONFIG_NAI_ICB_MST)
	if(nai_is_slv_fw_rdy()){
	        /*store revision new memory location and new format*/		
        	strncpy((char *)pMBVersion->mb_slv_version[SLV_FSBL].procid, "ARM1", 4);
	        strncpy((char *)pMBVersion->mb_slv_version[SLV_FSBL].type, "FSBL", 4);
		/*Get Slave fsbl rev from Slave common memory area via UART*/
		ret = slave_read(SLAVE_FSBL_REVISION, 4, 1, &data, FALSE);
		/*Store Slave fsbl Rev into MB common memory area */
		if(ret > 0){
			DEBUGF("Slave FSBL Rev 0x%08x \n", data);
			writel(data, PS2FPGA_MB_COMMON_SLAVE_FSBL_REV_OFFSET);
		        /*store slave in new memory location*/
			pMBVersion->mb_slv_version[SLV_FSBL].major = ((data >> 16) & 0xFFFF);
			pMBVersion->mb_slv_version[SLV_FSBL].minor = (data & 0xFFFF);
		}
		//Get Slave UBOOT Build Time
		ret = slave_read(SLAVE_FSBL_BUILD_TIME, 1, BUILD_TIME_STRING_LEN, (u32 *)revStr, FALSE);
		/*Store FSBL Build time into MB common memory area */
		if(ret > 0){
			DEBUGF("Slave FSBL Build Time %s \n", revStr);
			strncpy((char *)PS2FPGA_MB_COMMON_SLAVE_FSBL_BUILD_TIME_OFFSET, revStr, BUILD_TIME_STRING_LEN);
	                /*month MM*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_FSBL].month, _get_ascii_mm(revStr), 2);
	                /*day DD*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_FSBL].day, &revStr[4], 2);
	                /*year yyyy*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_FSBL].year, &revStr[7], 4);
	                /*hour hh*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_FSBL].hh, &revStr[15], 2);
	                /*minute mm*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_FSBL].mm, &revStr[18], 2);
	                /*second ss*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_FSBL].ss, &revStr[21], 2);
		}
	}else{
		printf("%s: MB Slave zynq is not ready!! \n",__func__);
	}
#endif
}

static void _update_uboot_rev(void){

#if defined(CONFIG_NAI_ZYNQ_SLAVE) && defined(CONFIG_NAI_ICB_MST)
	s32 ret = 0;
#endif	
	char revStr[CONFIG_VERSION_STRING_LEN];
	char tempStr[CONFIG_VERSION_STRING_LEN];
	u32 data = 0;

/*UBOOT Rev & Build Time*/
	
	/*Get Master UBOOT rev from OCRAM */
	/* parse revison string,
	 * 2014.00.00. 
	 * 
	 */	 
	strncpy(revStr, (char *)CONFIG_UBOOT_VERSION_STRING, CONFIG_VERSION_STRING_LEN);
	
	DEBUGF("UBOOT Build %s \n", revStr);
	/*parse high_rev low_rev: 2014*/
	tempStr[0] = revStr[0];
	tempStr[1] = revStr[1];
	tempStr[2] = revStr[2];	
	tempStr[3] = revStr[3];
	tempStr[4] = '\0';
	
	data |= (simple_strtoul(tempStr, NULL, 10) << 16);
	
	/*parse patch level: 00 */
	tempStr[0] = revStr[5];
	tempStr[1] = revStr[6];
	tempStr[2] = '\0';
	data |= (simple_strtoul(tempStr, NULL, 10) << 8);
	
	/*parse sub-level: 00*/
	tempStr[0] = revStr[8];
	tempStr[1] = revStr[9];
	tempStr[2] = '\0';
	data |= (simple_strtoul(tempStr, NULL, 10));
	
	DEBUGF("UBOOT Rev 0x%08x \n", data);	
	/*Store UBOOT Rev & Build Time into MB Common memory area */
	writel(data, PS2FPGA_MB_COMMON_UBOOT_REV_OFFSET);
	/*store revsion to an new memory area with new format*/	
	strncpy((char *)pMBVersion->mb_mst_version[MST_UBOT].procid, "ARM0", 4);
	strncpy((char *)pMBVersion->mb_mst_version[MST_UBOT].type, "UBOT", 4);
	/*major*/
	strncpy(tempStr, &revStr[35],5);
	tempStr[5] = '\0';
	pMBVersion->mb_mst_version[MST_UBOT].major = simple_strtoul(tempStr,NULL,10);
	/*minor*/
	strncpy(tempStr, &revStr[41], 5);
	tempStr[5] = '\0';
	pMBVersion->mb_mst_version[MST_UBOT].minor = simple_strtoul(tempStr, NULL, 10);
	/*build time*/
	strncpy(tempStr, &revStr[11], BUILD_TIME_STRING_LEN);
	tempStr[BUILD_TIME_STRING_LEN-1] = '\0';
	strncpy((char *)PS2FPGA_MB_COMMON_UBOOT_BUILD_TIME_OFFSET, tempStr, BUILD_TIME_STRING_LEN);
	/*month MM*/
	strncpy((char *)pMBVersion->mb_mst_version[MST_UBOT].month, _get_ascii_mm(tempStr), 2);
	/*day DD*/
	strncpy((char *)pMBVersion->mb_mst_version[MST_UBOT].day, &tempStr[4], 2);
	/*year yyyy*/
	strncpy((char *)pMBVersion->mb_mst_version[MST_UBOT].year, &tempStr[7], 4);
	/*hour hh*/
	strncpy((char *)pMBVersion->mb_mst_version[MST_UBOT].hh, &tempStr[15], 2);
	/*minute mm*/
	strncpy((char *)pMBVersion->mb_mst_version[MST_UBOT].mm, &tempStr[18], 2);
	/*second ss*/
	strncpy((char *)pMBVersion->mb_mst_version[MST_UBOT].ss, &tempStr[21], 2);

#if defined(CONFIG_NAI_ZYNQ_SLAVE) && defined(CONFIG_NAI_ICB_MST)
	if(nai_is_slv_fw_rdy()){
	        /*store revision new memory location and new format*/		
        	strncpy((char *)pMBVersion->mb_slv_version[SLV_UBOT].procid, "ARM1", 4);
	        strncpy((char *)pMBVersion->mb_slv_version[SLV_UBOT].type, "UBOT", 4);
		/*Get Slave UBOOT rev from Slave common memory area via UART*/
		ret = slave_read(SLAVE_UBOOT_REVISION, 4, 1, &data, FALSE);
		/*Store Slave UBOOT Rev into MB common memory area */
		if(ret > 0){
			DEBUGF("Slave UBOOT Rev 0x%08x \n", data);
			writel(data, PS2FPGA_MB_COMMON_SLAVE_UBOOT_REV_OFFSET);
		        /*store slave in new memory location*/
			pMBVersion->mb_slv_version[SLV_UBOT].major = ((data >> 16) & 0xFFFF);
			pMBVersion->mb_slv_version[SLV_UBOT].minor = (data & 0xFFFF);
		}
		//Get Slave UBOOT Build Time
		ret = slave_read(SLAVE_UBOOT_BUILD_TIME, 1, BUILD_TIME_STRING_LEN, (u32 *)revStr, FALSE);
		/*Store FSBL Build time into MB common memory area */
		if(ret > 0){
			DEBUGF("Slave UBOOT Build Time %s \n", revStr);
			strncpy((char *)PS2FPGA_MB_COMMON_SLAVE_UBOOT_BUILD_TIME_OFFSET, revStr, BUILD_TIME_STRING_LEN);
	                /*month MM*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_UBOT].month, _get_ascii_mm(revStr), 2);
	                /*day DD*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_UBOT].day, &revStr[4], 2);
	                /*year yyyy*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_UBOT].year, &revStr[7], 4);
	                /*hour hh*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_UBOT].hh, &revStr[15], 2);
	                /*minute mm*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_UBOT].mm, &revStr[18], 2);
	                /*second ss*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_UBOT].ss, &revStr[21], 2);
		}
	}else{
		printf("%s: MB Slave zynq is not ready!! \n",__func__);
	}
#endif

}


static void _update_fpga_rev(void){

#if defined(CONFIG_NAI_ZYNQ_SLAVE) && defined(CONFIG_NAI_ICB_MST)
	s32 ret = 0;
#endif	
	u32 rev  = 0, time = 0, tmp = 0;
	static char str[5];

	/*Get Master FPGA rev */
	rev = _get_mb_fpga_revision(TOP_MB_REV);
	/*Store FPGA Rev & Build Time into MB Common memory area */
	DEBUGF("FPGA Rev %x \n",rev);		
	writel(rev, PS2FPGA_MB_COMMON_FPGA_REV_OFFSET);
	
	/*Get Master FPGA Compile time */
	time = _get_mb_fpga_revision(TOP_MB_COMPILE_COUNT);
	DEBUGF("FPGA Build Time 0x%08x \n",time);		
	/*FPGA Compile Time (Bits 31..27=Day, 26..23=Month, 22..17=Yr(2000), 16..12=Hr, 11..6=Min, 5..0=Sec)*/
	writel(time, PS2FPGA_MB_COMMON_FPGA_BUILD_TIME_OFFSET);

	/*store revsion in new memory location with new format*/
	strncpy((char *)pMBVersion->mb_mst_version[MST_FPGA].procid, "ARM0", 4);
	strncpy((char *)pMBVersion->mb_mst_version[MST_FPGA].type, "FPGA", 4);
	
	/*major*/
	pMBVersion->mb_mst_version[MST_FPGA].major = (rev >> 16); 
	/*minor*/
	pMBVersion->mb_mst_version[MST_FPGA].minor = rev;
	/*build time*/
	/*FPGA Compile Time (Bits 31..27=Day, 26..23=Month, 22..17=Yr(2000), 16..12=Hr, 11..6=Min, 5..0=Sec)*/
	/*month DD*/
	//5 bits ddddd 31days
	tmp = ((time & 0xf8000000) >> 27); 
	snprintf(str, sizeof(str),"%02d", tmp);
	strncpy((char *)pMBVersion->mb_mst_version[MST_FPGA].day, str, 2);
	//4 bits mmmm 12 months
	tmp = ((time & 0x07800000) >> 23); 
	snprintf(str, sizeof(str),"%02d", tmp);
	strncpy((char *)pMBVersion->mb_mst_version[MST_FPGA].month, str, 2);
	//6 bits yyyyyy 0 t0 63 years 2000 ~ 2063
	tmp = ((time & 0x007e0000) >> 17);
	tmp += 2000;
	snprintf(str, sizeof(str),"%04d", tmp);
	/*year yyyy*/
	strncpy((char *)pMBVersion->mb_mst_version[MST_FPGA].year, str, 4);
	//5 bits hhhhhh 00 ~ 23 hours
	tmp = ((time & 0x0001f000) >> 12);
	snprintf(str, sizeof(str),"%02d", tmp);
	/*hour hh*/
	strncpy((char *)pMBVersion->mb_mst_version[MST_FPGA].hh, str, 2);
	//6 bits mmmmmm 59 minutes
	tmp = ((time & 0x00000fc0) >> 6);
	snprintf(str, sizeof(str),"%02d", tmp);
	/*minute mm*/
	strncpy((char *)pMBVersion->mb_mst_version[MST_FPGA].mm, str, 2);
	//6 bits ssssss 59 secnods
	tmp = ((time & 0x0000003f) >> 0);
	snprintf(str, sizeof(str),"%02d", tmp);
	/*second ss*/
	strncpy((char *)pMBVersion->mb_mst_version[MST_FPGA].ss, str, 2);

	/*Get Slave FPGA rev from Slave common memory area via UART*/
#if defined(CONFIG_NAI_ZYNQ_SLAVE) && defined(CONFIG_NAI_ICB_MST)
	//check if mb slave zynq is ready
	if(nai_is_slv_fw_rdy()){
	        /*store revision new memory location and new format*/		
        	strncpy((char *)pMBVersion->mb_slv_version[SLV_FPGA].procid, "ARM1", 4);
	        strncpy((char *)pMBVersion->mb_slv_version[SLV_FPGA].type, "FPGA", 4);
		/*Get Slave FPGA rev from Slave common memory area via UART*/
		ret = slave_read(SLAVE_FPGA_REVISION, 4, 1, &tmp, FALSE);
		/*Store Slave FPGA Rev into MB common memory area */
		if(ret > 0){
			DEBUGF("Slave FPGA Rev %d \n", tmp);
			writel(tmp, PS2FPGA_MB_COMMON_SLAVE_FPGA_REV_OFFSET);
		        /*store slave in new memory location*/
			pMBVersion->mb_slv_version[SLV_FPGA].major = ((tmp >> 16) & 0xFFFF);
			pMBVersion->mb_slv_version[SLV_FPGA].minor = (tmp & 0xFFFF);
		}
		//Get Slave FPGA Build Time
		ret = slave_read(SLAVE_FPGA_BUILD_TIME, 4, 1, &time, FALSE);
		/*Store FPGA Build time into MB common memory area */
		if(ret > 0){
			DEBUGF("Slave FPGA Build Time 0x%08x \n", time);
			writel(time, PS2FPGA_MB_COMMON_SLAVE_FPGA_BUILD_TIME_OFFSET);
	                /*build time*/
	                /*FPGA Compile Time (Bits 31..27=Day, 26..23=Month, 22..17=Yr(2000), 16..12=Hr, 11..6=Min, 5..0=Sec)*/
	                /*month DD*/
	                //5 bits ddddd 31days
	                tmp = ((time & 0xf8000000) >> 27); 
            	        snprintf(str, sizeof(str),"%02d", tmp);
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_FPGA].day, str, 2);
	                //4 bits mmmm 12 months
	                tmp = ((time & 0x07800000) >> 23); 
	                snprintf(str, sizeof(str),"%02d", tmp);
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_FPGA].month, str, 2);
	                //6 bits yyyyyy 0 t0 63 years 2000 ~ 2063
	                tmp = ((time & 0x007e0000) >> 17);
	                tmp += 2000;
	                snprintf(str, sizeof(str),"%04d", tmp);
	                /*year yyyy*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_FPGA].year, str, 4);
	                //5 bits hhhhhh 00 ~ 23 hours
	                tmp = ((time & 0x0001f000) >> 12);
	                snprintf(str, sizeof(str),"%02d", tmp);
	                /*hour hh*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_FPGA].hh, str, 2);
	                //6 bits mmmmmm 59 minutes
	                tmp = ((time & 0x00000fc0) >> 6);
	                snprintf(str, sizeof(str),"%02d", tmp);
	                /*minute mm*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_FPGA].mm, str, 2);
	                //6 bits ssssss 59 secnods
	                tmp = ((time & 0x0000003f) >> 0);
	                snprintf(str, sizeof(str),"%02d", tmp);
	                /*second ss*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_FPGA].ss, str, 2);
		}
	}else{
		printf("%s: MB Slave zynq is not ready!! \n",__func__);
	}
#endif	

}

#ifdef CONFIG_NAI_ZYNQ_SLAVE
static void _update_fw_rev(void){
	
	s32 ret = 0;
	u32 data  = 0;
	char revStr[CONFIG_VERSION_STRING_LEN];
	
	if(nai_is_slv_fw_rdy()){	
	        /*store revision new memory location and new format*/		
        	strncpy((char *)pMBVersion->mb_slv_version[SLV_FW].procid, "ARM1", 4);
	        strncpy((char *)pMBVersion->mb_slv_version[SLV_FW].type, "FW", 2);
		/*Get Slave FW rev from Slave common memory area via UART*/
		ret = slave_read(SLAVE_FW_REVISION, 4, 1, &data, FALSE);
		/*Store Slave FW Rev into MB common memory area */
		if(ret > 0){
			DEBUGF("Slave FW Rev 0x%08x \n", data);
			writel(data, PS2FPGA_MB_COMMON_SLAVE_FW_REV_OFFSET);
		        /*store slave in new memory location*/
			pMBVersion->mb_slv_version[SLV_FW].major = ((data >> 16) & 0xFFFF);
			pMBVersion->mb_slv_version[SLV_FW].minor = (data & 0xFFFF);
		}
		//Get Slave FW Build Time
		ret = slave_read(SLAVE_FW_BUILD_TIME, 1, BUILD_TIME_STRING_LEN, (u32 *)revStr, FALSE);
		/*Store FW Build time into MB common memory area */
		if(ret > 0){
			DEBUGF("Slave FW Build Time %s \n", revStr);
			strncpy((char *)PS2FPGA_MB_COMMON_SLAVE_FW_BUILD_TIME_OFFSET, revStr, BUILD_TIME_STRING_LEN);
	                /*month MM*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_FW].month, _get_ascii_mm(revStr), 2);
	                /*day DD*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_FW].day, &revStr[4], 2);
	                /*year yyyy*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_FW].year, &revStr[7], 4);
	                /*hour hh*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_FW].hh, &revStr[15], 2);
	                /*minute mm*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_FW].mm, &revStr[18], 2);
	                /*second ss*/
	                strncpy((char *)pMBVersion->mb_slv_version[SLV_FW].ss, &revStr[21], 2);
		}
	}else{
		printf("%s: MB Slave zynq is not ready!! \n",__func__);
	}
}
#endif /*CONFIG_NAI_ZYNQ_SLAVE*/

static u32 _get_mb_fpga_revision(u32 revision_id)
{
	u32 revision = -1;
	DEBUGF("%s: \n", __func__);
	switch (revision_id){
		case TOP_MB_REV:
			revision = readl(PS2FPGA_TOP_MB_REV_OFFSET);
			break;
		case TOP_PS_REV:	  
			revision = readl(PS2FPGA_TOP_PS_REV_OFFSET);
			break;
		case TOP_MODULE_REV:
			revision = readl(PS2FPGA_TOP_MODULE_REV_OFFSET);
			break;
		case TOP_SATA_REV:
			revision = readl(PS2FPGA_TOP_SATA_REV_OFFSET);
			break;
		case TOP_VME_REV:
			revision = readl(PS2FPGA_TOP_VME_REV_OFFSET);
			break;
		case TOP_VME_IP_REV:
			revision = readl(PS2FPGA_TOP_VME_IP_REV_OFFSET);
			break;
		case TOP_PCIE_REV: 	
			revision = readl(PS2FPGA_TOP_PCIE_REV_OFFSET);
			break;
		case TOP_CPCI_REV:	
			revision = readl(PS2FPGA_TOP_CPIC_REV_OFFSET);
			break;
		case TOP_MB_COMPILE_COUNT:
			revision = readl(PS2FPGA_MB_COMPILE_COUNT_OFFSET);
			break;
		default:
			break;
	}
	
	DEBUGF("%s: [FPGA_Revsion_ID.Version]: [%x.%x]\n", __func__, revision_id, revision);
	return revision;
}
