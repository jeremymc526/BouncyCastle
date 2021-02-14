/*
 * North Atlantic Industries 2017
 *
 */
#include <common.h>
#include <asm/io.h>
#include <malloc.h>
#include <i2c.h>
#include <nai_mb_fpga_address.h>
#include <nai_icb.h>
#include <nai_module_ext.h>
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

#define EEPROM_50_ADDR     0x50
#define EEPROM_51_ADDR     0x51
#define EEPROM_SIZE        256

/*special module defined */
#define MODULE_ID_CTS      0x43545300UL 
#define MODULE_ID_ES1      0x45533100UL
#define MODULE_ID_ES2      0x45533200UL
#define MODULE_ID_EM1      0x454D3100UL
#define MODULE_ID_FM2      0x464D3200UL
#define MODULE_ID_FM5      0x464D3500UL
#define MODULE_ID_FM7      0x464D3700UL
#define MODULE_ID_FW1      0x46573100UL
#define MODULE_ID_FW2      0x46573200UL
#define MODULE_ID_TE1      0x54453100UL
#define MODULE_ID_NON      0x4E4F4E00UL
#define MODULE_ID_SPACE    0x00000020UL

#define MODULE_ADDR_RDY    0xA5A5A5A5UL

#define MODULE_DONE_TIMEOUT        (10 * 1000)     //3 * 1000ms = 3 seconds
#define MODULE_100000_USDELAY      (100 * 1000)    //100ms 
#define MODULE_25000_USDELAY       (25 * 1000)     //25ms
/*AXI I2C Port defined*/
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

#define HSS_MOD_COMMON_MEM_ADDRESS      0x0
#define HSS_MOD_VER_ADDR               (HSS_MOD_COMMON_MEM_ADDRESS + 0x0030)

//Module common module mode ready state offset
#define HSS_MOD_READY_STATE      (HSS_MOD_COMMON_MEM_ADDRESS + 0x025C)
//Module Operation Mode States
#define HSS_MOD_OPERSTATE_FW_ENTERED_BIT                 (1UL << 16)
#define HSS_MOD_FW_READY_TIMEOUT                         (5 * 1000) // 5 * 1000ms = 5 seconds
#define HSS_MOD_RECOVERY_COUNT    5

#ifdef CONFIG_NAI_DEFAULT_MODULE_LO_SIZE_LIMIT
#define MODULE_LO_SIZE CONFIG_NAI_DEFAULT_MODULE_LO_SIZE_LIMIT
#else
#define MODULE_LO_SIZE 0x4000 //16KB
#endif

#ifdef CONFIG_NAI_DEFAULT_MODULE_HI_SIZE_LIMIT
#define MODULE_HI_SIZE CONFIG_NAI_DEFAULT_MODULE_HI_SIZE_LIMIT
#else
#define MODULE_HI_SIZE 0x200000 //2MB
#endif

#ifdef CONFIG_NAI_MAX_MODULE_MASK_SIZE_LIMIT
#define MODULE_MAX_SIZE CONFIG_NAI_MAX_MODULE_MASK_SIZE_LIMIT
#else
#define MODULE_MAX_SIZE 0x800000 //8MB
#endif


/*MB Reset Register Map 0x43C1 0008*/
typedef volatile struct {
    
    union {                               /*0x008 Reset duration*/
        u32 reg;
        struct {
                u32 reset_dur       : 8;  /*bit 0 - 7 reset duration*/
                u32 _reservered1    :24;  /*bit 8 - 31 reserved*/
        } bits;
    } reset_timer;
    
    union {                               /*0x00C Reset Control Reg*/
        u32 reg;
        struct {
            u32 _reservered1    : 5;  /*bit 0 - 4 reserved*/
            u32 pcie            : 1;  /*bit 5 PCIe reset*/
            u32 _reservered2    :26;  /*bit 6 - 31 reserved*/
        } bits;
    } reset_ctr;
        
    u32 all_mod_mask;  /*0x068 all module address decode mask*/
     
} MB_RESET_CONFIG_REG;

/*MB FPGA Module Address Config Status Register Map 0x43C0 0018*/
typedef volatile struct {
    union {
        u32 reg;
        struct {
                u32 _reservered1   :20;  /*bit 0 - 19  reservered*/
                u32 config_done    : 1;  /*bit 20 FPGA module address config done*/
                u32 _reservered2   :11;  /*bit 21 - 31 reservered*/
        } bits;
    } status;
} MOD_ADDR_CONFIG_STATUS;

/*Module ID Status Register Map 0x43C0 0028 - 0x43C0 0060*/
typedef volatile struct {
    u32 mod_addr_offset;                  /*0x028 HSS module memory start offset*/
    union {                               /*0x02C module status from FPGA*/
        u32 reg;
        struct {
                u32 detected       : 8;  /*bit 0 - 7  module detected by FPGA bit (ReadOnly)*/
                u32 _reservered1   : 8;  /*bit 8 - 15 reservered*/
                u32 powered        : 8;  /*bit 16 - 23 module powered on by FPGA (ReadOnly)*/
                u32 _reservered2   : 8;  /*bit 24 - 31 reservered*/
        } bits;
    } status;
    struct {                             /*0x030 - 0x05C module id and module size that FPGA read from module eeprom*/
        u32 id;
        u32 size;
    } mod[6];
} MOD_INFO_REG;

/*Module address Configuration Register Map 0x43C1 0030*/
typedef volatile struct {
                      /*0x000 - 0x064 module 1 - 7 address and enable*/
    struct {
        union {
            u32 reg;
            struct {
                u32 _reserverd1   : 2;  /*bit 0 - 1 reserved*/
                u32 end_adr       :10;  /*bit 2 - 11 end of module address*/
                u32 _reservered2  : 6;  /*bit 12 - 17 start of module address*/
                u32 start_adr     :10;  /*bit 18 - 27 start of module address*/
                u32 _reservered3  : 4;  /*bit 28 - 31 reserverd*/
            } bits;
        } adr;
        union {
            u32 reg;
            struct {
                u32 enabled       : 1;  /*bit 0 HSS access enable bit*/
                u32 _reserverd1   :31;  /*bit 1 - 31 reserved*/
            } bits;
        } status;
    } mod_adr_cfg[7];
    
    u32 all_mod_mask;  /*0x068 all module address decode mask*/
     
} MOD_ADDR_CONFIG;

/*Module HSS Configuration Register Map 0x43C4 4100*/
typedef volatile struct {
    union {
        u32 reg;                     /*0x000 - module hss detected status */
        struct {
            u32 detected        : 8; /*bit 0 - 7 hss detected bitmap per module slot*/
            u32 detection_done  : 8; /*bit 8 - 15 hss detection Done bitmap per module slot*/
            u32 _reserved       :16; /*bit 16 - 31 reserved*/
        } bits;
    } det_status;
    
    union {
        u32 reg;                    /*0x004 - module hss  link status */
        struct {
            u32 linked        : 8; /*bit 0 - 7 hss linked bitmap per module slot*/
            u32 link_done     : 8; /*bit 8 - 15 hss link done bitmap per module slot*/
            u32 _reserved     :16; /*bit 16 - 31 reserved*/
        } bits;
    } link_status;
    
    u32 resetn;                    /*0x008 - module hss reset bitmap bit 0 ~ 7*/
    u32 pwr_supply_en;             /*0x00C - module hss power supply enable bitmap bit 0 ~ 7*/
    u32 link_fail_stat;            /*0x010 - module link failed status bitmap bit 0 ~ 7*/
    u32 clk_en;                    /*0x014 - module clock enable bitmap bit 0 ~ 7*/
    u32 _reserved1;                /*0x018 - reserved*/
    u32 hss_if_reset;              /*0x01C - module hss IF reset */
    u32 dll_reset;                 /*0x020 - module dll reset */
    u32 config_mod;                /*0x024 - module configuration bitmap bit 0 - 7 */
    u32 ref_clk;                   /*0x028 - module reference clock */
    u32 _reserved2[5];             /*0x02C - 0x03C reserved*/
    u32 mod_gpio_data[6];          /*0x040 - 0x54 - module 1 - 7 gpiodata bitmap bit 0 ~ 7 [1=hi|0=low] */
    u32 _reserved3[2];             /*0x058 - 0x05C reserved*/
    u32 mod_gpio_dir[6];           /*0x060 - 0x074 module 1 - 6 gpio direction bitmap bit 0 ~ 7 [1=ouput|0=input] */
    u32 _reserved4[2];             /*0x078 - 0x07C reserved*/
                                   /*0x080 - 0x094 module io source selection per module slot
                                    *  Bit 0 - 16 
                                    *    0x00        = HSS configuration source
                                    *    0x01 - 0x0E = reserverd
                                    *    0x0F        = GPIO configuration source
                                   */ 
    u32 mod_src_sel[6];           /*0x080 - 0x94 module 1 - 6 source selection */
    
} MOD_CONFIG_REG;

#pragma pack(1)
typedef struct {
    u32 fpga_compile_time;
    u32 reserved[2];
    u32 fpga_rev;
    u32 reserved1[13];
    u8  fw_minor;
    u8  fw_major;
    u16 reserved2;
    u8  uboot_rev_sub;
    u8  uboot_rev_patch;
    u8  uboot_rev_lo;
    u8  uboot_rev_hi;
    u8  fsbl_rev_sub;
    u8  fsbl_rev_patch;
    u8  fsbl_rev_lo;
    u8  fsbl_rev_hi;
    char fw_build_str[24];
    char uboot_build_str[24];
    char fsbl_build_str[24];
} MOD_DATA;
#pragma pack()

/*global variable*/
MB_COMMON_MODULE *pCommonModule = (MB_COMMON_MODULE *) PS2FPGA_MB_COMMON_MODULE_STATUS_BASE_ADDR;
MB_RESET_CONFIG_REG *pMbResetReg = (MB_RESET_CONFIG_REG *) PS2FPGA_RESET_DURATION_OFFSET;
MOD_INFO_REG *pModInfo = (MOD_INFO_REG *) PS2FPGA_MOD_INFO_OFFSET;
MOD_ADDR_CONFIG *pModAddrConf = (MOD_ADDR_CONFIG *) PS2FPGA_MODULE1_ADDR_OFFSET;
MOD_ADDR_CONFIG_STATUS *pModAddrConfSts = (MOD_ADDR_CONFIG_STATUS *) PS2FPGA_TOP_PCIE_REV_OFFSET;
MOD_CONFIG_REG *pModConfReg = (MOD_CONFIG_REG *) PS2FPGA_MODULE_CONFIG_BASE_ADDR;

static u8 eeprom_data[EEPROM_SIZE];

/*static function prototype*/
static u32 _get_mb_fpga_rev(void);
static void _print_mod_status(void);
static void _find_module(void);
static void _init_common(void);
static void _init_module(void);
static void _deinit_module(void);
static void _reset_module(void);
static void _init_addr_msk(void);

static void _hss_module_init(u8 slot);
static void _hss_module_post_init(u8 slot);
static void _hss_module_deinit(u8 slot);
static void _hss_module_reset(u8 slot);
static void _hss_wait_link(u8 slot);
static void _hss_wait_detection(u8 slot);
static void _hss_wait_fw_rdy(u8 slot);

static void _em1_module_init(u8 slot);
static void _es1_module_init(u8 slot);
static void _es1_module_deinit(u8 slot);
#if defined(CONFIG_NAI_MODULE_PROC) 
static void _fm5_module_init(u8 slot);
#endif
static void _fw1_module_init(u8 slot);
static void _te1_module_init(u8 slot);
static void _te1_module_deinit(u8 slot);

#if defined(CONFIG_NAI_MODULE_ID_IIC)
static void _find_module_iic(void);
static s32 _read_eeprom_iic(u8 slot, u32 addr, u32 count, u8 *buf);
static s32 _write_eeprom_iic(u8 slot, u32 addr, u32 count, u8 *buf);
static void _wait_fpga_config_done(void);
static void _find_module_fpga(void);
#endif

#if defined(CONFIG_NAI_MODULE_ID_SLV)
static void _find_module_slv(void);
static s32 _read_eeprom_slv(u8 slot, u32 addr, u32 count, u8 *buf);
/*TODO: This a workaround for 6U MB
*because HSS IF reset bitmap is not
*supported in 6U MB FPGA. If a hss
* module is ready then we have to
* reset all hss module on the MB
*/
#endif

#ifdef CONFIG_NAI_MODULE_HSS_WR
static void _hss_mod_6u_rdy_state(void);
#endif


#ifdef CONFIG_NAI_MODULE_ID_IIC
static s32 _read_eeprom_iic(u8 slot, u32 addr, u32 count, u8 *buf)
{
    unsigned int bus;
    s32 ret = NAI_MODULE_SUCCESS;
    
    //Save the current I2C bus
    bus = i2c_get_bus_num();
            
    //Set I2C Bus per module slot
    i2c_set_bus_num(i2c_bus_num[slot]);
    
    DEBUGF( "%s:bus%d \n",__func__,i2c_get_bus_num());
    
    /* we have a special CTS module, and the eeprom address of
     * that module is 0x51,and all other modules eeprom 
     * address are 0x50*/
    if(i2c_probe(EEPROM_50_ADDR) == 0){
        i2c_read(EEPROM_50_ADDR, addr, 1, buf, count);
    }else if (i2c_probe(EEPROM_51_ADDR) == 0){
        i2c_read(EEPROM_51_ADDR, addr, 1, buf, count);
    }else{
        ret = NAI_MODULE_ERROR; 
    }
    
    //Restore I2C bus
    i2c_set_bus_num(bus);
    
    DEBUGF("%s:module %02x \n",__func__,slot);
    return ret;
}

static s32 _write_eeprom_iic(u8 slot, u32 addr, u32 count, u8 *buf)
{
    unsigned int bus;
    s32 ret = NAI_MODULE_SUCCESS;
    
    //Save the current I2C bus
    bus = i2c_get_bus_num();
            
    //Set I2C Bus per module slot
    i2c_set_bus_num(i2c_bus_num[slot]);
    
    DEBUGF( "%s:bus%d \n",__func__,i2c_get_bus_num());
    
    /* we have a special CTS module, and the eeprom address of
     * that module is 0x51,and all other modules eeprom 
     * address are 0x50*/
    if(i2c_probe(EEPROM_50_ADDR) == 0){
        i2c_write(EEPROM_50_ADDR, addr, 1, buf, count);
    }else if (i2c_probe(EEPROM_51_ADDR) == 0){
        i2c_write(EEPROM_51_ADDR, addr, 1, buf, count);
    }else{
        ret = NAI_MODULE_ERROR; 
    }
    
    //Restore I2C bus
    i2c_set_bus_num(bus);
    
    DEBUGF("%s:module %02x \n",__func__,slot);
    return ret;
}

static void _find_module_iic()
{
    u8 slot = 0, phy_slot = 0;
    /*set module id and size to MB module common area*/
    for(slot = 0; slot < MAX_MODULE_SLOT; slot++)
    {
        /*look for module and read mod id and size for eeprom*/
        if(_read_eeprom_iic(slot, 0, 12, eeprom_data) == NAI_MODULE_SUCCESS)
        {
            /*set module detected bitmap*/
            pCommonModule->mod[slot].status.bits.detected = 1;
            
            /*
             * Mark's MBexec wants big endian when storing
             * module ID and module size data to MB common
             * memory area
             */
            /*set module id*/
            pCommonModule->mod_id[slot] = (eeprom_data[0] << 24) | 
                                          (eeprom_data[1] << 16) | 
                                          (eeprom_data[2] << 8)  | 
                                          (eeprom_data[3] );
            
            /*set module size*/
            pCommonModule->mod_size[slot] = (eeprom_data[4] << 24) | 
                                            (eeprom_data[5] << 16) | 
                                            (eeprom_data[6] << 8)  | 
                                            (eeprom_data[7] );
            /*update module slot position*/
	    phy_slot = slot + 1;
	    if(eeprom_data[8] != phy_slot)
	    {
                /*update module eeprom with current slot phyical position*/
                _write_eeprom_iic(slot, 8, 1, &phy_slot);
	    }
		DEBUGF("%s:mod status 0x%01x \n",__func__,pCommonModule->mod[slot].status.bits.detected);
		DEBUGF("%s:mod id   0x%08x  \n",__func__,pCommonModule->mod_id[slot]);
		DEBUGF("%s:mod size 0x%08x \n",__func__,pCommonModule->mod_size[slot]);
        }
    }
}

static void _wait_fpga_config_done(void)
{
    ulong time = 0;
    u8 config_done = 0;
    
    time = get_timer(0);
    
    do{
        config_done =  pModAddrConfSts->status.bits.config_done;
        //timeout
        if(get_timer(time) > MODULE_DONE_TIMEOUT)
            break;
    }while(!config_done);
    
    DEBUGF("Module# %x fpga config done [fpga_config_done = 0x%02x]\n",slot,config_done);
}

static void _find_module_fpga(void)
{
    u8 slot = 0, powered = 0, detected = 0;
    
    
    /*set module id and size to MB module common area*/
    for(slot = 0; slot < MAX_MODULE_SLOT; slot++)
    {
        /*clear MB module powered and detected status*/
        pCommonModule->mod[slot].status.bits.detected = 0;
        pCommonModule->mod[slot].status.bits.power = 0;
        
        detected = (pModInfo->status.bits.detected & (1 << slot));
        /*look for module and read mod id and size from fpga*/
        if(detected)
        {
            /*set module detected bitmap in MB common*/
            pCommonModule->mod[slot].status.bits.detected = 1;
            
            powered = (pModInfo->status.bits.powered & (1 << slot));
            /*set module powered bitmap in MB common*/
            if(powered)
              pCommonModule->mod[slot].status.bits.power = 1;
            
            /*
             * Mark's MBexec wants big endian when storing
             * module ID and module size data to MB common
             * memory area
             */
            /*set module id*/
            pCommonModule->mod_id[slot] = pModInfo->mod[slot].id;
            /*set module size*/
            pCommonModule->mod_size[slot] = pModInfo->mod[slot].size;

            DEBUGF("%s:mod status 0x%01x \n",__func__,pCommonModule->mod[slot].status.bits.detected);
            DEBUGF("%s:mod id   0x%08x  \n",__func__,pCommonModule->mod_id[slot]);
            DEBUGF("%s:mod size 0x%08x \n",__func__,pCommonModule->mod_size[slot]);
        }
    }
}
#endif /*CONFIG_NAI_MODULE_ID_IIC*/

#if defined(CONFIG_NAI_MODULE_ID_SLV)
static s32 _read_eeprom_slv(u8 slot, u32 addr, u32 count, u8 *buf)
{
    
    s32 ret = NAI_MODULE_SUCCESS;
    u32 nb = 0, eepromAddr = 0;
    
    eepromAddr = (SLAVE_MODULE1_INF_EEPROM_DATA - (slot << 8)) + addr;
    nb = slave_read(eepromAddr, 1, count, (u32 *)buf, FALSE);
    
    if(nb != count)
    {
        ret = NAI_MODULE_ERROR;
        printf("%s: Failed to read eeprom data from slv ret=%x, count=%x\n",__func__, nb, count);
    }
    
    return ret;
}

static void _find_module_slv()
{
    u32 slot = 0;
    u32 detected = 0;
    
    /*look for module*/
    slave_read(SLAVE_DETECTED_MODULE, 1, 1, &detected, FALSE);
    
    /*set module id and size to MB module common area*/
    for(slot = 0; slot < MAX_MODULE_SLOT; slot++)
    {
        if(detected & (1 << slot))
        {
            /*set module detected*/
            pCommonModule->mod[slot].status.bits.detected = 1;
            _read_eeprom_slv(slot,0,8,eeprom_data);
            
            /*
             * Mark's MBexec wants big endian when storing
             * module ID and module size data to MB common
             * memory area
             */
            pCommonModule->mod_id[slot] = (eeprom_data[0] << 24) |
                                          (eeprom_data[1] << 16) |
                                          (eeprom_data[2] << 8)  |
                                          (eeprom_data[3] );
            
            /*set module size*/
            pCommonModule->mod_size[slot] = (eeprom_data[4] << 24) |
                                            (eeprom_data[5] << 16) |
                                            (eeprom_data[6] << 8)  |
                                            (eeprom_data[7] );
            
            
            DEBUGF("%s:mod status 0x%01x \n",__func__,pCommonModule->mod[slot].status.bits.detected);
            DEBUGF("%s:mod id   0x%08x  \n",__func__,pCommonModule->mod_id[slot]);
            DEBUGF("%s:mod size 0x%08x \n",__func__,pCommonModule->mod_size[slot]);
        }
    }
}
#endif /*CONFIG_NAI_MODULE_ID_SLV*/

static void _find_module(void)
{

#if defined(CONFIG_NAI_MODULE_ID_SLV)
    _find_module_slv();
#elif defined(CONFIG_NAI_MODULE_ID_IIC)
    /*fpga major revision v2.0 above*/
    if((_get_mb_fpga_rev() >> 16) >= 2)
    {
        _wait_fpga_config_done();
        
        if(pModAddrConfSts->status.bits.config_done)
            _find_module_fpga();
    }
    else
    {
        _find_module_iic();
    }
#endif
}

/*init module address decoder mask*/
static void _init_addr_msk(void)
{
    u32 slot  = 0;
    u32 addr  = 0;
    u32 size  = 0;
    u32 start = 0;
    u32 next  = 0;
    /*include module address offset in the total size*/
    u32 total_size = 0;
    u32 detected = 0;

    if((_get_mb_fpga_rev() >> 16) >= 2)
    {
       /* Include module address offset in the total size */
       total_size = pModInfo->mod_addr_offset;
    }
    else
    {
       /* Include 16KiB MB common area in the total size */
       total_size = PS2FPGA_MB_COMMON_SIZE;
    }
    
    /*set address mask for each module*/
    for(slot = 0; slot < MAX_MODULE_SLOT; slot++)
    {
        /*get module detected status*/
        detected = pCommonModule->mod[slot].status.bits.detected;
        
        if(detected)
        {
            size = pCommonModule->mod_size[slot];
            addr = pCommonModule->mod_addr[slot];

            if(size >= MODULE_LO_SIZE && size <= MODULE_HI_SIZE)
            {
                /*take bit 23 ~ 14 of a start of address and set to mod address mask*/
                pModAddrConf->mod_adr_cfg[slot].adr.bits.start_adr = ((0xFFC000 & addr) >> 14); 
                /*take bit 23 ~ 14 of a end of address and set to mod address mask*/
                pModAddrConf->mod_adr_cfg[slot].adr.bits.end_adr = ((0xFFC000 & ((addr + size) -1)) >> 14);
                /*enable module access*/
                pModAddrConf->mod_adr_cfg[slot].status.bits.enabled = 1;
                /*increment total size*/
                total_size += size;
            }
            else
            {
                /*disable module access when size is out of range*/
                pModAddrConf->mod_adr_cfg[slot].status.bits.enabled = 0;
            }
            continue;
        }
        
        /*disable module access when there is no module a slot*/
        pModAddrConf->mod_adr_cfg[slot].status.bits.enabled = 0;
    }
    
    /*find all module decode address mask*/
    start = MODULE_LO_SIZE;
    next = start * 2;

    do
    {
        if((total_size > start) && (total_size < next))
        {
            total_size = next;
            break;
        }
        start = next;
        next *= 2;
    }
    while(start < MODULE_MAX_SIZE);

    
    if(total_size > MODULE_MAX_SIZE)
    {
        total_size = MODULE_MAX_SIZE;
    }

    /*set all module address decode mask*/
    pModAddrConf->all_mod_mask = (total_size - 1);
    DEBUGF("%s:all mask 0x%08x \n",__func__,pModAddrConf->all_mod_mask);
}

/*init MB common area with module id/size/address*/
static void _init_common(void)
{
    /*module address start*/
    u32 addr = 0;
    u32 detected = 0;
    u32 size = 0;
    u32 id   = 0;
    u8 slot  = 0;

    /* Set Mod start based on FPGA revision check 2.0+ */
    if((_get_mb_fpga_rev() >> 16) >= 2)
    {
       /* Module addr start defined by hardware register */
       addr = pModInfo->mod_addr_offset;
       writel(addr, PS2FPGA_MB_COMMON_MODULE_BASE_ADDR);
    }
    else
    {
       /* Module addr start after 16Kib MB Common area */
       addr = PS2FPGA_MB_COMMON_SIZE;
    }
    
    /* 
     * Updated MB common area's module ID/size/addr
     * based on Mark's MBExec requirement
    */
    for(slot = 0; slot < MAX_MODULE_SLOT; slot++)
    {
        /*get module detected status*/
        detected = pCommonModule->mod[slot].status.bits.detected;
        
        if(detected)
        {
            /*
             * Read back module size and module id 
             * that was read from eeprom
             */
            size = pCommonModule->mod_size[slot];
            id = pCommonModule->mod_id[slot];
            
            if(id == 0xFFFFFFFF)
            {
                /*incorrect module id*/
                /*write 0xFFFFFFFF to module address*/
                pCommonModule->mod_addr[slot] = 0xFFFFFFFF;
                /*reset module size to 0*/
                pCommonModule->mod_size[slot] = 0;
                /*reset module id to NON*/
                pCommonModule->mod_id[slot] = MODULE_ID_NON|MODULE_ID_SPACE;
            }
            else if(size < MODULE_LO_SIZE || size > MODULE_HI_SIZE)
            { 
                /*incorrect module size*/
                /*write 0xFFFFFFFF to module address*/
                pCommonModule->mod_addr[slot] = 0xFFFFFFFF;
                /*reset module size to 0*/
                pCommonModule->mod_size[slot] = 0;
                /*update module ID*/
                pCommonModule->mod_id[slot] = id|MODULE_ID_SPACE;
            }
            else
            {
                /*update module module address*/
                pCommonModule->mod_addr[slot] = addr;
                /*update module ID*/
                pCommonModule->mod_id[slot] = id|MODULE_ID_SPACE;
                /*increment module address for next module slot*/
                addr += size;
            }
        }
        else
        {
            /*module was not detected on this slot*/
            /*Fill MB common area with default module value*/
            /* write 0xFFFFFFFF module address*/
            pCommonModule->mod_addr[slot] = 0xFFFFFFFF;
            /* write 0 to  module size*/
            pCommonModule->mod_size[slot] = 0;
            /* write NON module id*/
            pCommonModule->mod_id[slot] = MODULE_ID_NON|MODULE_ID_SPACE;
        }
        
        DEBUGF("%s:mod addr 0x%01x \n",__func__,pCommonModule->mod_addr[slot]);
        DEBUGF("%s:mod id   0x%08x  \n",__func__,pCommonModule->mod_id[slot]);
        DEBUGF("%s:mod size 0x%08x \n",__func__,pCommonModule->mod_size[slot]);
    }
    
    /*Set module address read*/
    pCommonModule->module_addr_rdy = 0xA5A5A5A5;

}

#if defined(CONFIG_NAI_MODULE_PROC) 
static void _fm5_module_init(u8 slot)
{
    /*change mod mio[7:0] config src to GPIO*/
    pModConfReg->mod_src_sel[slot] = 0xFFFFFFFF;
    
    /*gpio 0, 3, 4, 5, 6, 7 not connected and set them to input*/
    pModConfReg->mod_gpio_dir[slot] &= ~(1 << 0);
    pModConfReg->mod_gpio_dir[slot] &= ~(1 << 3);
    pModConfReg->mod_gpio_dir[slot] &= ~(1 << 4);
    pModConfReg->mod_gpio_dir[slot] &= ~(1 << 5);
    pModConfReg->mod_gpio_dir[slot] &= ~(1 << 6);
    pModConfReg->mod_gpio_dir[slot] &= ~(1 << 7);
    
    /*set gpio 1 PCIe-PD output low*/
    pModConfReg->mod_gpio_dir[slot] |= (1 << 1);
    pModConfReg->mod_gpio_data[slot] &= ~(1 << 1);
    
#if defined(CONFIG_NAI_MODULE_PROC)
    /*set gpio 2 (PCIe-SEL) output high for proc (intel/ppc based mb)*/
    pModConfReg->mod_gpio_dir[slot] |= (1 << 2);
    pModConfReg->mod_gpio_data[slot] |= (1 << 2);
#elif defined(CONFIG_NAI_MODULE_ARM)
    /*set gpio 2 (PCIe-SEL) output low for arm (zynq arm based mb)*/  
    pModConfReg->mod_gpio_dir[slot] |= (1 << 2);
    pModConfReg->mod_gpio_data[slot] &= ~(1 << 2);
#endif
}
#endif

static void _fw1_module_init(u8 slot)
{
#if defined(CONFIG_NAI_MODULE_PROC)
    /*change mod mio[7:0] config src to GPIO*/
    pModConfReg->mod_src_sel[slot] = 0xFFFFFFFF;
    
    /*gpio 0, 3, 4, 5 not connected and set them to input*/
    pModConfReg->mod_gpio_dir[slot] &= ~(1 << 0);
    pModConfReg->mod_gpio_dir[slot] &= ~(1 << 3);
    pModConfReg->mod_gpio_dir[slot] &= ~(1 << 4);
    pModConfReg->mod_gpio_dir[slot] &= ~(1 << 5);
    
    /*set gpio 1 PCIe-PD output low*/
    pModConfReg->mod_gpio_dir[slot] |= (1 << 1);
    pModConfReg->mod_gpio_data[slot] &= ~(1 << 1);
    
    /*set gpio 2 (PCIe-SEL) output high for proc (intel/ppc 68ARM1 mb)*/
    pModConfReg->mod_gpio_dir[slot] |= (1 << 2);
    pModConfReg->mod_gpio_data[slot] |= (1 << 2);
    
    /*set gpio 7 high to enable power supply*/
    pModConfReg->mod_gpio_dir[slot] |= (1 << 7);
    pModConfReg->mod_gpio_data[slot] |= (1 << 7);
    
    /*wait 100ms based on TI XIO02213A Firewire datasheet*/
    udelay(MODULE_100000_USDELAY);
    
    /*set gpio 6 high to release reset*/
    pModConfReg->mod_gpio_dir[slot] |= (1 << 6);
    pModConfReg->mod_gpio_data[slot] |= (1 << 6);
    
#elif defined(CONFIG_NAI_MODULE_ARM_HSM3)
    u32 tmp;
    /*saved current reset duration */
    tmp = pMbResetReg->reset_timer.bits.reset_dur;
    /*wait 100ms based on TI XIO02213A Firewire datasheet*/
    /*set PCIe reset duration to 100ms and each increment is 0.625ms 
     * 160 * 0.625ms = 100ms
     */
    pMbResetReg->reset_timer.bits.reset_dur = 160;
    /*Take PCIe out of reset */
    pMbResetReg->reset_ctr.bits.pcie = 1;
    /*restore reset duration register*/
    pMbResetReg->reset_timer.bits.reset_dur = tmp;
#else
    printf("FW module is not supported on this MB!!!\n");
#endif
}

static void _em1_module_init(u8 slot)
{
    /*change mod mio[7:0] config src to gpio*/
    pModConfReg->mod_src_sel[slot] = 0xFFFFFFFF;
    
    /*gpio 0, 4, 5, 7 not connected and set them to input*/
    pModConfReg->mod_gpio_dir[slot] &= ~(1 << 0);
    pModConfReg->mod_gpio_dir[slot] &= ~(1 << 4);
    pModConfReg->mod_gpio_dir[slot] &= ~(1 << 5);
    pModConfReg->mod_gpio_dir[slot] &= ~(1 << 7);
    
    /*set gpio 1 PCIe-PD output low*/ 
    pModConfReg->mod_gpio_dir[slot] |= (1 << 1);
    pModConfReg->mod_gpio_data[slot] &= ~(1 << 1);

#if defined(CONFIG_NAI_MODULE_PROC)    
    /*set gpio 2 (PCIe-SEL) output high for proc (intel/ppc based mb)*/ 
    pModConfReg->mod_gpio_dir[slot] |= (1 << 2);
    pModConfReg->mod_gpio_data[slot] |= (1 << 2);
#elif defined(CONFIG_NAI_MODULE_ARM)  
    /*set gpio 2 (PCIe-SEL) output low for proc (zynq arm based mb)*/ 
    pModConfReg->mod_gpio_dir[slot] |= (1 << 2);
    pModConfReg->mod_gpio_data[slot] &= ~(1 << 2);
#endif    
    
    /*set gpio 3 (PCIe-WAKE) input */
    pModConfReg->mod_gpio_dir[slot] &= ~(1 << 3);
        
    /*set gpio 6 PCIe-RSTn high*/
    pModConfReg->mod_gpio_dir[slot] |= (1 << 6);
    pModConfReg->mod_gpio_data[slot] |= (1 << 6);
    
}

static void _es1_module_init(u8 slot)
{
    /*change mod mio[7:0] config src to HSS*/
    pModConfReg->mod_src_sel[slot] = 0;
    
    /*gpio 0, 1, 2, 3, 4, 5 not connected*/
    
    /*enable module power supply*/
    pModConfReg->pwr_supply_en |= (1 << slot);
    udelay(MODULE_100000_USDELAY);
    
    /*take module out of reset*/
    pModConfReg->resetn |= (1 << slot);
    
    /*set module power status*/
    pCommonModule->mod[slot].status.bits.power = 1;
}

static void _es1_module_deinit(u8 slot)
{
    /*change mod mio[7:0] config src to HSS*/
    pModConfReg->mod_src_sel[slot] = 0;
    
    /*held module in of reset*/
    pModConfReg->resetn &= ~(1 << slot);
    udelay(MODULE_25000_USDELAY);
        
    /*disable module power supply*/
    pModConfReg->pwr_supply_en &= ~(1 << slot);
    
    /*clear module power status*/
    pCommonModule->mod[slot].status.bits.power = 0;
    
}


static void _te1_module_init(u8 slot)
{
    /*change mod mio[7:0] config src to HSS*/
    pModConfReg->mod_src_sel[slot] = 0;
    
    /*gpio 0, 1, 2, 3 not connected*/
    /*enable module power supply*/
    
    pModConfReg->pwr_supply_en |= (1 << slot);
    udelay(MODULE_25000_USDELAY);
    
    /*enable module clock*/
    pModConfReg->clk_en |= (1 << slot);
    udelay(MODULE_25000_USDELAY);
    
    /*take module out of reset*/
    pModConfReg->resetn |= (1 << slot);
    udelay(MODULE_25000_USDELAY);
    
    /*set module power status*/
    pCommonModule->mod[slot].status.bits.power = 1;
}

static void _te1_module_deinit(u8 slot)
{
    /*change mod mio[7:0] config src to HSS*/
    pModConfReg->mod_src_sel[slot] = 0;
    
    /*held module in of reset*/
    pModConfReg->resetn &= ~(1 << slot);
    udelay(MODULE_25000_USDELAY);
    
    /*disable module clock*/
    pModConfReg->clk_en &= ~(1 << slot);
    udelay(MODULE_25000_USDELAY);
    
    /*disable module power supply*/
    pModConfReg->pwr_supply_en &= ~(1 << slot);
    udelay(MODULE_25000_USDELAY);
    
    /*clear module power status*/
    pCommonModule->mod[slot].status.bits.power = 0;
    
}

static void _hss_wait_detection(u8 slot)
{
    ulong time = 0;
    u8 detected = 0;
    u8 detection_done = 0;
        
    //clear hss_det_done status
    pCommonModule->mod[slot].status.bits.hss_det_done = 0;
    
    time = get_timer(0);
    
    do{
        detection_done = pModConfReg->det_status.bits.detection_done;
        detected = pModConfReg->det_status.bits.detected;
        //timeout
        if(get_timer(time) > MODULE_DONE_TIMEOUT)
            break;
        
    }while(!(detection_done & (1 << slot)) || !(detected & (1 << slot)));
    
    
    if((detection_done & (1 << slot)) && (detected & (1 << slot)))
    {
         //set hss_det_done status
        pCommonModule->mod[slot].status.bits.hss_det_done = 1;
    }
    
    DEBUGF("Module# %x HSS Detected [detected = 0x%02x] [detection_done = 0x%02x]\n",slot,detected,detection_done);
}

static void _hss_wait_link(u8 slot)
{
    ulong time = 0;
    u8 linked = 0;
    u8 link_done = 0;
    
    //clear hss link done status
    pCommonModule->mod[slot].status.bits.hss_link_done = 0;
    
    time = get_timer(0);
    
    do{
        link_done = pModConfReg->link_status.bits.link_done;
        linked = pModConfReg->link_status.bits.linked;
        //timeout
        if(get_timer(time) > MODULE_DONE_TIMEOUT)
            break;
    }while(!(link_done & (1 << slot)) || !(linked & (1 << slot)));
    
    
    if((link_done & (1 << slot)) && (linked & (1 << slot))){
        //set hss_link_done status
        pCommonModule->mod[slot].status.bits.hss_link_done = 1;
    }
    
    DEBUGF("Module# %x HSS Link [linked = 0x%02x] [link_done = 0x%02x]\n",slot,linked,link_done);
    
}

static void _hss_module_reset(u8 slot)
{
    u32 detected = 0;
    DEBUGF("%s:hss interface module slot %d \n",__func__,slot);
    /*change mod mio[7:0] config src to HSS*/
    pModConfReg->mod_src_sel[slot] = 0;
    
    /*held hss interface in reset*/
    #if defined(CONFIG_NAI_MODULE_HSS_WR)
    /*TODO: This a workaround for 6U MB
     *because HSS IF reset bitmap is not
     *supported in 6U MB FPGA.  We can't support
     *special module on 6U MB till HSS IF reset
     *bitmap is supported.
     */
        pModConfReg->hss_if_reset = 0x3f;
    #else
        pModConfReg->hss_if_reset |= (1 << slot);
    #endif
    
    /*held module in of reset*/
    pModConfReg->resetn &= ~(1 << slot);
    udelay(MODULE_25000_USDELAY);
    
    /*take module out of reset*/
    pModConfReg->resetn |= (1 << slot);
    udelay(MODULE_25000_USDELAY);
    
    /*wait for hss detetion*/
    _hss_wait_detection(slot);

    detected = pCommonModule->mod[slot].status.bits.hss_det_done;
    if(detected)
    {
        /*release module hss interface from reset*/
        #if defined(CONFIG_NAI_MODULE_HSS_WR)
         /*TODO: This a workaround for 6U MB
         *because HSS IF reset bitmap is not
         *supported in 6U MB FPGA.  We can't support
         *special module on 6U MB till HSS IF reset
         *bitmap is supported.
         */
            pModConfReg->hss_if_reset = 0;
        #else
            pModConfReg->hss_if_reset &= ~(1 << slot);
        #endif
        udelay(MODULE_25000_USDELAY);
        
        /*wait for hss link detetion*/ 
        _hss_wait_link(slot);
        
        /*wait for hss module fw ready*/
        if(pCommonModule->mod[slot].status.bits.hss_link_done)
            _hss_wait_fw_rdy(slot);
    }
    else
    {
        DEBUGF("%s: not hss_det_done 0x%08x \n",__func__,detected);
    }

}

static void _hss_module_init(u8 slot)
{
    DEBUGF("%s:hss interface module slot %d \n",__func__,slot);
    /*change mod mio[7:0] config src to HSS*/
    pModConfReg->mod_src_sel[slot] = 0;
    
    /*held hss interface in reset*/
    #if defined(CONFIG_NAI_MODULE_HSS_WR)
    /*TODO: This a workaround for 6U MB
     *because HSS IF reset bitmap is not
     *supported in 6U MB FPGA.  We can't support
     *special module on 6U MB till HSS IF reset
     *bitmap is supported.
     */
      pModConfReg->hss_if_reset = 0x3f;
    #else
      pModConfReg->hss_if_reset |= (1 << slot);
    #endif
        
    /*enable module power supply*/
    pModConfReg->pwr_supply_en |= (1 << slot);
    udelay(MODULE_25000_USDELAY);
    
    /*enable module clock*/
    pModConfReg->clk_en |= (1 << slot);
    udelay(MODULE_25000_USDELAY);

    /*take module out of reset*/
    pModConfReg->resetn |= (1 << slot);
    udelay(MODULE_25000_USDELAY);
    
    /*set module power status*/
    pCommonModule->mod[slot].status.bits.power = 1;
}

static void _hss_module_post_init(u8 slot)
{
    u32 detected = 0, hssInReset = 0;
    
    /*change mod mio[7:0] config src to HSS*/
    pModConfReg->mod_src_sel[slot] = 0;
    
    /*wait for hss detetion*/
    _hss_wait_detection(slot);
    
    detected = pCommonModule->mod[slot].status.bits.hss_det_done;
    DEBUGF("%s:hss_det_done 0x%08x \n",__func__,detected);
    if(detected)
    {
        hssInReset = (pModConfReg->hss_if_reset & (1 << slot));
        if(hssInReset)
        {
            /*release module hss interface from reset*/
            #if defined(CONFIG_NAI_MODULE_HSS_WR)
            /*TODO: This a workaround for 6U MB
            *because HSS IF reset bitmap is not
            *supported in 6U MB FPGA.  We can't support
            *special module on 6U MB till HSS IF reset
            *bitmap is supported.
            */
              pModConfReg->hss_if_reset = 0;
            #else
              pModConfReg->hss_if_reset &= ~(1 << slot);
            #endif
            //udelay(MODULE_25000_USDELAY);
        }
        
        /*wait for hss link detetion*/ 
        _hss_wait_link(slot);
        
        /*wait for hss module fw ready*/
        if(pCommonModule->mod[slot].status.bits.hss_link_done)
            _hss_wait_fw_rdy(slot);
    }
    
}

static void _hss_module_deinit(u8 slot)
{
    /*change mod mio[7:0] config src to HSS*/
    pModConfReg->mod_src_sel[slot] = 0;
    
    /*held hss interface in reset*/
    #if defined(CONFIG_NAI_MODULE_HSS_WR)
    /*TODO: This a workaround for 6U MB
     *because HSS IF reset bitmap is not
     *supported in 6U MB FPGA.  We can't support
     *special module on 6U MB till HSS IF reset
     *bitmap is supported.
     */
        pModConfReg->hss_if_reset = 0x3f;
    #else
        pModConfReg->hss_if_reset |= (1 << slot);
    #endif
    
    udelay(MODULE_25000_USDELAY);
    
    /*held module in of reset*/
    pModConfReg->resetn &= ~(1 << slot);
    udelay(MODULE_25000_USDELAY);
    
    /*disable module clock*/
    pModConfReg->clk_en &= ~(1 << slot);
    udelay(MODULE_25000_USDELAY);
    
    /*disable module power supply*/
    pModConfReg->pwr_supply_en &= ~(1 << slot);
    udelay(MODULE_25000_USDELAY);
    
    /*clear hss det_done, link_done, power status*/
    pCommonModule->mod[slot].status.bits.hss_det_done = 0;
    pCommonModule->mod[slot].status.bits.hss_link_done = 0;
    pCommonModule->mod[slot].status.bits.power = 0;
}

static void _hss_wait_fw_rdy(u8 slot)
{
    s32 status = NAI_SUCCESS;
    u32 data = 0;
    ulong time = 0;
    
    //clear hss fw ready bit
    pCommonModule->mod[slot].status.bits.hss_fw_ready = 0;
    
    nai_init_msg_utils(MB_SLOT);
        
    time = get_timer(0);
        
    do{
        status = nai_read_reg32_by_slot_request((u8)(slot+1), (u32)HSS_MOD_READY_STATE, &data);
        
        if (status != NAI_SUCCESS){
            break;
        }
        
        /*timeout*/
        if(get_timer(time) > HSS_MOD_FW_READY_TIMEOUT)
            break;    
            
    }while(!(data & HSS_MOD_OPERSTATE_FW_ENTERED_BIT));
    
    if((data & HSS_MOD_OPERSTATE_FW_ENTERED_BIT) && (status == NAI_SUCCESS))
    {
        //set hss fw ready bit
        pCommonModule->mod[slot].status.bits.hss_fw_ready = 1;
    }
    
    DEBUGF("Module# %x HSS FW 0x%08x\n", slot,data);

}

static void _init_module(void)
{
    u8 slot  = 0;
    u32 id = 0;
    u32 detected = 0;
    u32 power = 0;
    
    for(slot = 0; slot < MAX_MODULE_SLOT; slot++)
    {
        /*get module detected status*/
        detected = pCommonModule->mod[slot].status.bits.detected;
        
        if(detected)
        {
            /*let's enable reference clk*/
            pModConfReg->ref_clk = 1;
            
            /*read mod id from mb common area*/
            id = pCommonModule->mod_id[slot];
            /*get module detected status*/
            power = pCommonModule->mod[slot].status.bits.power;
            switch (id)
            {
                case MODULE_ID_CTS|MODULE_ID_SPACE :
                  DEBUGF("%s:CTS 0x%08x \n",__func__,id);
                  /*do nothing*/
                  break;
                case MODULE_ID_ES1|MODULE_ID_SPACE :
                  DEBUGF("%s:ES1 0x%08x \n",__func__,id);
                  if(!(power))
                      _es1_module_init(slot);
                  break;
                case MODULE_ID_ES2|MODULE_ID_SPACE :
                  DEBUGF("%s:ES2 0x%08x \n",__func__,id);
                  break;
                case MODULE_ID_EM1|MODULE_ID_SPACE :
                  DEBUGF("%s:EM1 0x%08x \n",__func__,id);
                  _em1_module_init(slot);
                  break;
                case MODULE_ID_FM2|MODULE_ID_SPACE :
                case MODULE_ID_FM5|MODULE_ID_SPACE :
                case MODULE_ID_FM7|MODULE_ID_SPACE :
                  DEBUGF("%s:FMx 0x%08x \n",__func__,id);
#if defined(CONFIG_NAI_MODULE_PROC)    
                  _fm5_module_init(slot);
#endif
                  break;
                case MODULE_ID_FW1|MODULE_ID_SPACE :
                case MODULE_ID_FW2|MODULE_ID_SPACE :
                  DEBUGF("%s:FW 0x%08x \n",__func__,id);
                  _fw1_module_init(slot);
                  break;
                case MODULE_ID_TE1|MODULE_ID_SPACE :
                  DEBUGF("%s:TE1 0x%08x \n",__func__,id);
                  if(!(power))
                      _te1_module_init(slot);
                  break;
                case MODULE_ID_NON|MODULE_ID_SPACE :
                  DEBUGF("%s:NON 0x%08x \n",__func__,id);
                  /*TODO: This is a hack!!!
                   * Currently, MBexec is using HSS interface to
                   * program a blank EEPROM on a interface module.
                   * Once AXI_IIC EEPROM programming is implemented in
                   * the MBExec then we should remove this hack.
                   * As of now, a module with blank EEPROM, we'll
                   * assume it's a hss interface module.
                   */ 
                  if(!(power))
                    _hss_module_init(slot);
                  break;
                default :                  
                  DEBUGF("%s:HSS module 0x%08x \n",__func__,id);
                  if(!(power))
                    _hss_module_init(slot);
                  break;
            }
        }
    }
    
    /*hss module post init*/
    for(slot = 0; slot < MAX_MODULE_SLOT; slot++)
    {
        /*get module detected status*/
        detected = pCommonModule->mod[slot].status.bits.detected;
        
        if(detected)       
        {
            /*read mod id from mb common area*/
            id = pCommonModule->mod_id[slot];
            /*TODO: 
            * Once AXI_IIC EEPROM programming is implemented in
            * the MBExec then we don't have to run hss post init
            * for MODULE_ID_NON|MODULE_ID_SPACE
            */ 
            if( (id != (MODULE_ID_CTS|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_ES1|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_ES2|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_EM1|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FW1|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FW2|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FM2|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FM5|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FM7|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_TE1|MODULE_ID_SPACE)))
            {
                _hss_module_post_init(slot);
            }
        }
    }

}

static void _deinit_module(void)
{
    u8 slot  = 0;
    u32 id = 0;
    u32 detected = 0;
    u32 power = 0;
     
    DEBUGF("%s:detected 0x%02x power 0x%02x\n",__func__,detected,power);
    for(slot = 0; slot < MAX_MODULE_SLOT; slot++)
    {
        /*get module detected status*/
        detected = pCommonModule->mod[slot].status.bits.detected;
        
        if(detected)
        {
            /*get module power status*/
            power = pCommonModule->mod[slot].status.bits.power;
            /*read mod id from mb common area*/
            id = pCommonModule->mod_id[slot];
            switch (id)
            {
                case MODULE_ID_CTS|MODULE_ID_SPACE :
                  DEBUGF("%s:CTS 0x%08x \n",__func__,id);
                  /*do nothing*/
                  break;
                case MODULE_ID_ES1|MODULE_ID_SPACE :
                  DEBUGF("%s:ES1 0x%08x \n",__func__,id);
                  if(power)
                      _es1_module_deinit(slot);
                  break;
                case MODULE_ID_ES2|MODULE_ID_SPACE :
                  DEBUGF("%s:ES2 0x%08x \n",__func__,id);
                  break;
                case MODULE_ID_EM1|MODULE_ID_SPACE :
                  DEBUGF("%s:EM1 0x%08x \n",__func__,id);
                  /*do nothing*/
                  break;
                case MODULE_ID_FW1|MODULE_ID_SPACE :
                case MODULE_ID_FW2|MODULE_ID_SPACE :
                case MODULE_ID_FM2|MODULE_ID_SPACE :
                case MODULE_ID_FM5|MODULE_ID_SPACE :
                case MODULE_ID_FM7|MODULE_ID_SPACE :
                  DEBUGF("%s:FM5 0x%08x \n",__func__,id);
                  /*do nothing*/
                  break;
                case MODULE_ID_TE1|MODULE_ID_SPACE :
                  DEBUGF("%s:TE1 0x%08x \n",__func__,id);
                  if(power)
                      _te1_module_deinit(slot);
                   break;
                case MODULE_ID_NON|MODULE_ID_SPACE :
                  DEBUGF("%s:NON 0x%08x \n",__func__,id);
                  /*TODO: This is a hack!!!
                   * Currently, MBexec is using HSS interface to
                   * program a blank EEPROM on a interface module.
                   * Once AXI_IIC EEPROM programming is implemented in
                   * the MBExec then we should remove this hack.
                   * As of now, a module with blank EEPROM, we'll
                   * assume it's a hss interface module.
                   */ 
                  if(power)
                    _hss_module_deinit(slot);
                  break;
                default :                  
                  /*hss interface module*/
                  DEBUGF("%s:HSS module 0x%08x \n",__func__,id);
                  if(power)
                      _hss_module_deinit(slot);
                  break;
            }
            
            /*let's disable reference clk*/
            pModConfReg->ref_clk = 0;
        }
    }
}

static void _reset_module(void)
{
    u8 slot  = 0;
    u32 id = 0;
    u32 detected = 0;
    u32 power = 0;
    
    for(slot = 0; slot < MAX_MODULE_SLOT; slot++)
    {
        /*get module detected status*/
        detected = pCommonModule->mod[slot].status.bits.detected;
        
        if(detected)
        {
            /*get module power status*/
            power = pCommonModule->mod[slot].status.bits.power;
            /*read mod id from mb common area*/
            id = pCommonModule->mod_id[slot];
            
            switch (id)
            {
                case MODULE_ID_CTS|MODULE_ID_SPACE :
                  DEBUGF("%s:CTS 0x%08x \n",__func__,id);
                  /*do nothing*/
                  break;
                case MODULE_ID_ES1|MODULE_ID_SPACE :
                  DEBUGF("%s:ES1 0x%08x \n",__func__,id);
                  /*do nothing*/
                  break;
                case MODULE_ID_ES2|MODULE_ID_SPACE :
                  DEBUGF("%s:ES2 0x%08x \n",__func__,id);
                  /*do nothing*/
                  break;
                case MODULE_ID_EM1|MODULE_ID_SPACE :
                  DEBUGF("%s:EM1 0x%08x \n",__func__,id);
                  /*do nothing*/
                  break;
                case MODULE_ID_FW1|MODULE_ID_SPACE :
                case MODULE_ID_FW2|MODULE_ID_SPACE :
                case MODULE_ID_FM2|MODULE_ID_SPACE :
                case MODULE_ID_FM5|MODULE_ID_SPACE :
                case MODULE_ID_FM7|MODULE_ID_SPACE :
                  DEBUGF("%s:FM5 0x%08x \n",__func__,id);
                  /*do nothing*/;
                  break;
                case MODULE_ID_TE1|MODULE_ID_SPACE :
                  DEBUGF("%s:TE1 0x%08x \n",__func__,id);
                  /*do nothing*/
                  break;
                case MODULE_ID_NON|MODULE_ID_SPACE :
                  DEBUGF("%s:NON 0x%08x \n",__func__,id);
                  /*do nothing*/
                  break;
                default :                  
                  DEBUGF("%s:HSS module 0x%08x \n",__func__,id);
                  if(power)
                      _hss_module_reset(slot);
                  break;
            }
        }
    }
}

void static _print_mod_status(void)
{
    char id_str[4];
    u32 id;
    u8 slot  = 0;
    u32 addr = 0;
    u32 size = 0;
    u32 detected = 0;
    
    for(slot = 0; slot < MAX_MODULE_SLOT; slot++)
    {
        printf("slot %02d\n",slot);
        detected = pCommonModule->mod[slot].status.bits.detected;
        if(detected)
        {
            /*mod id sizse*/
            id = pCommonModule->mod_id[slot];
            id_str[0] = pCommonModule->mod_id[slot] >> 24;
            id_str[1] = pCommonModule->mod_id[slot] >> 16;
            id_str[2] = pCommonModule->mod_id[slot] >> 8;
            id_str[3] = 0;
            
            addr = pCommonModule->mod_addr[slot];
            size = pCommonModule->mod_size[slot];
            
            printf("\tmodule id: %s \n",id_str);
            printf("\taddr: 0x%08x size: 0x%08x\n",addr,size);
            
            if( (id != (MODULE_ID_CTS|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_ES1|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_ES2|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_EM1|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FW1|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FW2|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FM2|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FM5|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FM7|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_TE1|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_NON|MODULE_ID_SPACE)))
            {
                printf("\thss module detected[0x%02x] linked[0x%02x] fw[0x%02x]\n"
                ,pCommonModule->mod[slot].status.bits.hss_det_done
                ,pCommonModule->mod[slot].status.bits.hss_link_done
                ,pCommonModule->mod[slot].status.bits.hss_fw_ready);
            }
            
            continue;
        }
        printf("\tempty\n");
    }
}

static u32 _get_mb_fpga_rev(void)
{
    u32 rev = 0;
    
    /*get fpga revision*/
    rev = readl(PS2FPGA_TOP_MB_REV_OFFSET);
    
    return rev;
}
/*entry point*/
void nai_init_module(void)
{
printf("-------------------------------------\n");
printf("Initializing Module\n");
#if defined(CONFIG_NAI_ZYNQ_SLAVE) && defined(CONFIG_NAI_ICB_MST)
    //let's kick off fancy ICB 
    nai_icb_enable();
    
    if(nai_is_slv_fw_rdy())
    {
        uart1_init();
    }
    else
    {
        printf("Slave Zynq is not ready !!!\n");
        return;
    }
#endif    
    /*searching for module*/
    _find_module();
    /*init mb common area*/
    _init_common();
    /*init module address mask*/
    _init_addr_msk();
#ifdef CONFIG_NAI_CPCI
    
    /*fpga v2.0 will handle the cpci bar configuration*/
    /*fpga major revision is less than v2.0*/
    if((_get_mb_fpga_rev() >> 16) < 2)
    {
        printf("configure cpci\n");
        /*setup cPCI bar*/
        nai_cpci_setup();
    }
#endif
    /*let's party*/
    _init_module();
    /*print module information*/
    _print_mod_status();
    
    printf("-------------------------------------\n");
}

void nai_disable_module(void)
{
    /*deinit all availabe module*/
    _deinit_module();
}

void nai_enable_module(void)
{
    /*init all availabe module*/
    _init_module();
}

void nai_module_reset()
{
    _reset_module();
}

void nai_print_module_info(void)
{
    char id[4];
    u8 slot  = 0;
    u32 addr = 0;
    u32 size = 0;
    u32 detected = 0;
    u32 hss_fw_ready = 0;
    s32 status = NAI_SUCCESS;
    Nai_fpga_build_time fpga_build_time;
    MOD_DATA *data;
   
    /* Allocate memory for mod data buffer */
    data = (MOD_DATA *)malloc(sizeof(MOD_DATA));
    
    if (!data)
    {
        printf("%s: no memory!\n", __func__);
        return;
    }
    
    for(slot = 0; slot < MAX_MODULE_SLOT; slot++)
    {
        detected = pCommonModule->mod[slot].status.bits.detected;
        hss_fw_ready = pCommonModule->mod[slot].status.bits.hss_fw_ready;
        
        if(detected)
        {
            printf("-------- -------- Slot %02d Info -------- --------\n", slot);
            
            /*mod id sizse*/
            id[0] = pCommonModule->mod_id[slot] >> 24;
            id[1] = pCommonModule->mod_id[slot] >> 16;
            id[2] = pCommonModule->mod_id[slot] >> 8;
            id[3] = 0;
            
            addr = pCommonModule->mod_addr[slot];
            size = pCommonModule->mod_size[slot];
            
            printf("\tModule ID: %s \n",id);
            printf("\tAddress: 0x%08x Size: 0x%08x\n",addr,size);
            
            if(hss_fw_ready)
            {
                /*hss only mod FSBL, U_BOOT, FW FPGA */
                nai_init_msg_utils(MB_SLOT);
                
                status = nai_read_block32_by_slot_request((slot+1), HSS_MOD_VER_ADDR,(sizeof(MOD_DATA)/4),4,(u32 *)data);
                
                if (status != NAI_SUCCESS)
                {
                    printf("\tFailed to get slot %02d module version info\n",slot);
                    continue;
                }
                
                printf("\tFSBL Rev: %04d.%02d.%02d\n", 
                    ((data->fsbl_rev_hi << 8)|(data->fsbl_rev_lo << 0)),
                    data->fsbl_rev_patch,
                    data->fsbl_rev_sub);
                printf("\tFSBL Build Time: %s\n", data->fsbl_build_str);
                
                printf("\tUBOOT Rev: %04d.%02d.%02d\n", 
                    ((data->uboot_rev_hi << 8)|(data->uboot_rev_lo << 0)),
                    data->uboot_rev_patch,
                    data->uboot_rev_sub);
                printf("\tUBOOT Build Time: %s\n", data->uboot_build_str);
                
                printf("\tFW Rev: %02d.%02d\n", 
                    data->fw_major,
                    data->fw_minor);
                printf("\tFW Build Time: %s\n",data->fw_build_str);
                
                printf("\tFPGA Rev: 0x%08X \n", data->fpga_rev);
                nai_decode_fpga_compile_count(&fpga_build_time, data->fpga_compile_time);
                
                printf("\tFPGA Build Time: %02d/%02d/%04d at %02d:%02d:%02d\n", 
                    fpga_build_time.month,
                    fpga_build_time.day,
                    fpga_build_time.year,
                    fpga_build_time.hour,
                    fpga_build_time.minute,
                    fpga_build_time.second);
            }
            continue;
        }
        printf("-------- -------- Slot %02d is Empty -------- --------\n", slot);
    }
}

/*This is hss module recovery check function.
 *In this function if a hss module is not ready 
 * then it will perform a reset on that module.
 * This is only a temporary workaround, this will
 * add addition delay to system bootup time
 */
void nai_chk_hss_mod_rdy_state(void)
{
    int retry = 0;
    u8 slot  = 0;
    u32 id = 0;
    u32 detected = 0;
    u32 fw_rdy = 0;
    
    for(slot = 0; slot < MAX_MODULE_SLOT; slot++)
    {
        /*get module detected status*/
        detected = pCommonModule->mod[slot].status.bits.detected;
        
        if(detected)
        {
            /*read mod id from mb common area*/
            id = pCommonModule->mod_id[slot];
            
            if( (id != (MODULE_ID_CTS|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_ES1|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_ES2|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_EM1|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FW1|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FW2|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FM2|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FM5|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FM7|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_TE1|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_NON|MODULE_ID_SPACE)))
            {
                do{
                     /*verify module fw ready*/
                    _hss_wait_fw_rdy(slot);
                    
                    fw_rdy = pCommonModule->mod[slot].status.bits.hss_fw_ready;
                    
                    if(fw_rdy)
                      break;
                    
                    printf("recovering slot %d hss module\n",slot);
                    #if defined(CONFIG_NAI_MODULE_HSS_WR)
                      /*TODO: This a workaround for 6U MB
                       *because HSS IF reset bitmap is not
                       *supported in 6U MB FPGA. If a hss
                       * module is not ready then we have to
                       * reset all hss module on the MB
                      */
                      _hss_mod_6u_rdy_state();
                    #else
                      _hss_module_reset(slot);
                    #endif
                    retry++;
                }while(retry < HSS_MOD_RECOVERY_COUNT);
                
                if((retry >= HSS_MOD_RECOVERY_COUNT) && (fw_rdy == 0))
                {
                    printf("failed to recover slot %d hss module\n",slot);
                }
            }
        }
    }
}

#if defined(CONFIG_NAI_MODULE_HSS_WR)
/*TODO: This a workaround for 6U MB
*because HSS IF reset bitmap is not
*supported in 6U MB FPGA. If a hss
* module is not ready then we have to
* reset all hss module on the MB
*/
static void _hss_mod_6u_rdy_state(void)
{
    u8 slot  = 0;
    u32 id = 0;
    u32 detected = 0;
    
    for(slot = 0; slot < MAX_MODULE_SLOT; slot++)
    {
        /*get module detected status*/
        detected = pCommonModule->mod[slot].status.bits.detected;
        
        if(detected)
        {
            /*read mod id from mb common area*/
            id = pCommonModule->mod_id[slot];
            
            if( (id != (MODULE_ID_CTS|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_ES1|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_ES2|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FW1|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FW2|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FM2|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FM5|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_FM7|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_TE1|MODULE_ID_SPACE)) &&
                (id != (MODULE_ID_NON|MODULE_ID_SPACE)))
            {
                _hss_module_reset(slot);
            }
        }
    }
}
#endif
