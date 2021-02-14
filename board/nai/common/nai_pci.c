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
#include <nai_module_ext.h>
#include <nai_mb.h>
#include <nai_pci.h>


#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define DEBUGF(x...) printf(x)
#else
#define DEBUGF(x...)
#endif /* DEBUG */
 
#define CPCI_REV_0 (0x0)
#define CPCI_REV_1 (0x1)
#define CPCI_REV_2 (0x2)

/*BAR_MEMORY_SPACE 1 = MEMORY Space 0 = I/O Space */
#define BAR_MEMORY_SPACE    BIT(0)
/*BAR_ENABLE 1 = enable 0 = disable*/
#define BAR_ENABLE             BIT(1)
/*BAR_PREFETCHABLE_MODE 1 = enable 0 = disable*/
#define BAR_PREFETCHABLE_MODE BIT(2)
#define BAR_SIZE_BIT_MASK        (0xFFFF0000)

#define BAR_SIZE_64K    0x10000
#define BAR_SIZE_128K    0x20000
#define BAR_SIZE_256K    0x40000
#define BAR_SIZE_512K    0x80000
#define BAR_SIZE_1M        0x100000
#define BAR_SIZE_2M        0x200000
#define BAR_SIZE_4M        0x400000
#define BAR_SIZE_8M        0x800000

#ifdef CONFIG_NAI_DEFAULT_LO_CPCI_BAR0_SIZE
#define DEFAULT_BAR0_LO_SIZE     CONFIG_NAI_DEFAULT_LO_CPCI_BAR0_SIZE
#else
#define DEFAULT_BAR0_LO_SIZE     BAR_SIZE_64K
#endif

#ifdef CONFIG_NAI_DEFAULT_LO_CPCI_BAR1_SIZE
#define DEFAULT_BAR1_LO_SIZE     CONFIG_NAI_DEFAULT_LO_CPCI_BAR1_SIZE
#else
#define DEFAULT_BAR1_LO_SIZE     BAR_SIZE_64K
#endif

#ifdef CONFIG_NAI_DEFAULT_MAX_CPCI_BAR0_SIZE
#define DEFAULT_BAR0_MAX_SIZE     CONFIG_NAI_DEFAULT_MAX_CPCI_BAR0_SIZE
#else
#define DEFAULT_BAR0_MAX_SIZE     BAR_SIZE_64K
#endif

#ifdef CONFIG_NAI_DEFAULT_MAX_CPCI_BAR1_SIZE
#define DEFAULT_BAR1_MAX_SIZE     CONFIG_NAI_DEFAULT_MAX_CPCI_BAR1_SIZE
#else
#define DEFAULT_BAR1_MAX_SIZE     BAR_SIZE_64K
#endif

        
static void _cpci_setup(void);
static void _setup_cpci_bar(u32 bar_setup_addr, u8 bar_mode, u32 bar_size);
static u32 _align_bar_size(const u32 low, const u32 high, const u32 size);

#ifdef CONFIG_NAI_SETUP_CPCI_BAR1
static u32 _get_env_cpci_bar1_size(void);
#endif

#ifdef CONFIG_NAI_SETUP_CPCI_BAR1
static u32 _get_env_cpci_bar1_size(){
    
    u32 ret = DEFAULT_BAR1_LO_SIZE;
    
    DEBUGF("%s ret1 0x%08x\n", __func__, ret);
    if (env_get("cpcibar1size") != NULL){
        /*default to 64K*/
        ret = env_get_ulong("cpcibar1size", 16, DEFAULT_BAR1_LO_SIZE);
        
        DEBUGF("%s ret2 0x%08x\n", __func__, ret);
        if(ret < DEFAULT_BAR1_LO_SIZE)
            ret = DEFAULT_BAR1_LO_SIZE;
        else if (ret > DEFAULT_BAR1_MAX_SIZE)
            ret = DEFAULT_BAR1_MAX_SIZE;    
    }
    
    return ret;
}
#endif

#ifdef CONFIG_NAI_SETUP_CPCI_BAR0
/*we are not using u-boot env to set cPCI bar0 size*/
#if 0
static u32 _get_env_cpci_bar0_size(){
    
    u32 ret = DEFAULT_BAR0_LO_SIZE;
    
    DEBUGF("%s ret1 0x%08x\n", __func__, ret);
    if (env_get("cpcibar0size") != NULL){
        /*default to 64K*/
        ret = env_get_ulong("cpcibar0size", 16, DEFAULT_BAR0_LO_SIZE);
        
        DEBUGF("%s ret2 0x%08x\n", __func__, ret);
        if(ret < DEFAULT_BAR0_LO_SIZE)
            ret = DEFAULT_BAR0_LO_SIZE;
        else if (ret > DEFAULT_BAR0_MAX_SIZE)
            ret = DEFAULT_BAR0_MAX_SIZE;    
    }
    
    return ret;
}
#endif
#endif

static u32 _align_bar_size(const u32 low, const u32 high, const u32 size){
    
    u32 start = 0;
    u32 nextAlign = 0;
    u32 barSize = 0;
    
    barSize = size;
    
    DEBUGF("%s:low 0x%08x high 0x%08x size 0x%08x\n", __func__, low,high,barSize);
    
    if( low > barSize ){
        DEBUGF("%s return low 0x%08x\n", __func__, low);
        return low;
    }else if( high < barSize ){
        DEBUGF("%s return low 0x%08x\n", __func__, high);
        return high;
    }    
    
    start = low;
    nextAlign = start * 2;
    
    do{
        
         if ((start < barSize) && (barSize < nextAlign)){
             barSize = nextAlign;
             DEBUGF("%s:nextAlign barSize 0x%08x\n", __func__, barSize);
             break;
         }
         
         start = nextAlign;
         nextAlign *= 2;         
         
    }while(high > start);
        
    return barSize;
}

static void _setup_cpci_bar(u32 bar_setup_addr, u8 bar_mode, u32 bar_size)
{

    u32 bar_config_data = 0;
    u32    mask_bar_size = 0;
    
    DEBUGF("%s: bar_addr %x bar_mode %x bar_size 0x%08x \n", __func__, bar_setup_addr, bar_mode, bar_size);
    
    //encode bar size
    mask_bar_size = ((bar_size - (64 * 1024)) & BAR_SIZE_BIT_MASK);
    DEBUGF("%s: mask_bar_size 0x%08x\n", __func__, mask_bar_size);
    
    bar_config_data = (mask_bar_size | bar_mode);
    DEBUGF("%s: bar_config_data 0x%08x\n", __func__, bar_config_data);
    
    writel(bar_config_data, bar_setup_addr);
}

static void _cpci_setup(void)
{
    u32 cpci_mode = 0;
    u32 slot = 0;
    u32 barSize = 0;
    u32 detected = 0;

#ifdef CONFIG_NAI_SETUP_CPCI_BAR0
        
/* read each module size from each available module on
 * MB and caculated the total requried size
 */
    /*always include MB common memroy area size*/
    barSize = PS2FPGA_MB_COMMON_SIZE;
    
    /*get size of module*/
    for(slot = 0; slot < NUM_MOD_SLOT; slot++)
    {
        detected = pCommonModule->mod[slot].status.bits.detected;
        if(detected)
        {
            barSize += pCommonModule->mod_size[slot];
            DEBUGF("%s:bar0 size 0x%08x\n", __func__, barSize);
        }
    }
    
    barSize = _align_bar_size(DEFAULT_BAR0_LO_SIZE, DEFAULT_BAR0_MAX_SIZE, barSize);
    //Enable BAR0
    //Setup BAR0 size
    //Set the BAR0 type to Memory Space
    DEBUGF("%s:bar0 size 0x%08x\n", __func__, barSize);
    cpci_mode = (BAR_ENABLE | BAR_MEMORY_SPACE);
    _setup_cpci_bar(PS2FPGA_CPCI_BAR0_SETUP_REG_OFFSET, 
                        cpci_mode,
                        barSize);
#endif

#ifdef CONFIG_NAI_SETUP_CPCI_BAR1
        //Enable BAR1
        //Setup BAR1 size
        //Set the BAR1 type to Memory Space
        barSize = _get_env_cpci_bar1_size();
        DEBUGF("%s:bar1 size 0x%08x\n", __func__, barSize);
        barSize = _align_bar_size(DEFAULT_BAR1_LO_SIZE, DEFAULT_BAR1_MAX_SIZE, barSize);
        DEBUGF("%s:bar1 size 0x%08x\n", __func__, barSize);
        cpci_mode = (BAR_ENABLE | BAR_MEMORY_SPACE);
        _setup_cpci_bar(PS2FPGA_CPCI_BAR1_SETUP_REG_OFFSET, 
                            cpci_mode,
                            barSize);
#endif    

}

void nai_cpci_setup(void){
    _cpci_setup();
}


