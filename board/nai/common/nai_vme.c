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
#include <nai_mb.h>
#include <nai_module_ext.h>
#include <nai_icb.h>
#include <master_slave.h>
#include <nai_vme.h>


//#define DEBUG
#undef DEBUG
#ifdef DEBUG
#define DEBUGF(x...) printf(x)
#else
#define DEBUGF(x...)
#endif /* DEBUG */
 
#define BIT(x)	(1<<(x))

#define SLAVE_PS_GPIO_INPUT_DATA_BASEADDR_0 0xE000A060
#define SLAVE_PS_GPIO_INPUT_DATA_BASEADDR_1 0xE000A064

#ifdef CONFIG_NAI_VME_LO_SLV_WINDOW_SIZE_LIMIT
#define SLV_WIN_LO_SIZE    CONFIG_NAI_VME_LO_SLV_WINDOW_SIZE_LIMIT
#else
#define SLV_WIN_LO_SIZE 	0x100 /*256B*/
#endif

#ifdef CONFIG_NAI_VME_HI_SLV_WINDOW_SIZE_LIMIT
#define SLV_WIN_HI_SIZE    CONFIG_NAI_VME_HI_SLV_WINDOW_SIZE_LIMIT
#else
#define SLV_WIN_HI_SIZE 	0x800000 /*8MB*/
#endif

#define VME_AM_A32		0x00
#define VME_AM_A24		0x30
#define VME_AM_USR		0x08
#define VME_AM_SUP		0x0C
#define VME_AM_BLT		0x03
#define VME_AM_PROG		0x02
#define VME_AM_DATA		0x01
#define VME_AM_MBLT		0x00

typedef enum
{
    SLV_WIN_1 = 0,
    SLV_WIN_2,
    SLV_WIN_3,
    SLV_WIN_4,
    SLV_WIN_5,
    SLV_WIN_6,
    SLV_WIN_7,
    SLV_WIN_8,
    NUM_SLV_WIN
} SLV_WIN;

/*vme local user csr register 0x80000000*/
typedef volatile struct {
    u32 int_ebl;            /* 0x000 */
    u32 int_status;         /* 0x004 */
                            
    union {
        u32 reg;            /* 0x008 vme interrupt enable register*/
        struct {
            u32 vie_swirq  : 1; /*bit 0 vme software interrupt reuqest enable*/
            u32 vie_uirq   : 1; /*bit 1 vme user interrupt reuqest enable*/
            u32 _reserved  :30; /*bit 2 - 31 reserved*/
        } bits;
    } vint_ebl;
    
    u32 vint_status;        /* 0x00C */
    u32 vme_irqh_cmd;       /* 0x010 */
    u32 vme_irq_stat[7];    /* 0x014 - 0x02C IRQ7 ~ IRQ1*/
    u32 vme_int;            /* 0x030 */
    u32 vme_int_stat;       /* 0x034 */
                            
    union {
        u32 reg;            /* 0x038 vme interrupter map register*/
        struct {
            u32 vint_swirq  : 3;  /*bit 0 - 2 software interrupt map*/
            u32 _reserved   : 1;  /*bit 3 reserved*/
            u32 vint_uirq   : 3;  /*bit 4 - 6 user interrupt map*/
            u32 _reserved1  : 1;  /*bit 7 reserved*/
            u32 vint_type   : 2;  /*bit 8 - 9 interrupt type*/
            u32 _reserved2  :22;  /*bit 10 - 31 reserved*/
        } bits;
    } vme_int_map;
    
    u32 vme_int_stat_sw;    /* 0x03C */
    u32 vme_semaphore;      /* 0x040 */
    u32 _reserved1[3];      /* 0x044 - 0x04C */
    u32 mailbox[4];         /* 0x050 - 0x05C */
    u32 dma_vaddr;          /* 0x060 */
    u32 dma_laddr;          /* 0x064 */
    u32 dma_cmd;            /* 0x068 */
    u32 dma_stat;           /* 0x06C */
    u32 slv_acc_msk4;       /* 0x070 */
    u32 slv_acc_cmp4;       /* 0x074 */
    u32 slv_acc_dec4;       /* 0x078 */
    u32 _reserved2;         /* 0x07C */
    u32 slv_acc_msk3;       /* 0x080 */
    u32 slv_acc_cmp3;       /* 0x084 */
    u32 slv_acc_dec3;       /* 0x088 */
    u32 _reserved3;         /* 0x08C */
    u32 slv_acc_msk2;       /* 0x090 */
    u32 slv_acc_cmp2;       /* 0x094 */
    u32 slv_acc_dec2;       /* 0x098 */
    u32 _reserved4;         /* 0x09C */
                            
                            /* 0x0A0 - 0x0A8 slave access window 1 register*/  
    struct {
        union {
            u32 reg;        /*0x0A0 decode mask register*/
            struct {
                u32 _reserved       : 8;  /*bits 0 - 7 */
                u32 slvw_adem       :24;  /*bits 8 - 31 */
            } bits;
        } msk;
        
        union {
            u32 reg;        /*0x0A4 decode Compare register*/
            struct {
                u32 _reserved       : 8;  /*bits 0 - 7 */
                u32 slvw_ader       :24;  /*bits 8 - 31 */
            } bits;
            
        } cmp;
        
        union {
            u32 reg;        /*0x0A8 decode register*/
            struct {
                u32 slvw_faf        : 1;  /*bits 0 fixed address function*/
                u32 slvw_dfs        : 1;  /*bits 1 dynamic function sizing*/
                u32 _reserved       : 2;  /*bits 2 - 3 reserved*/
                u32 slvw_sel        : 3;  /*bits 4 - 6 slave window selector*/
                u32 _reserved1      : 1;  /*bits 7 reserved*/
                u32 slvw_offset     :16;  /*bits 8 - 23 data access*/
                u32 data_access     : 1;  /*bits 24 data access*/
                u32 prog_access     : 1;  /*bits 25 program access*/
                u32 nonpriv_access  : 1;  /*bits 26 not-privileged access*/
                u32 sup_access      : 1;  /*bits 27 supervisory access*/
                u32 blt_access      : 1;  /*bits 28 */
                u32 mblt_access     : 1;  /*bits 29 */
                u32 slvw_adtack     : 1;  /*bits 30 */
                u32 slvw_ebl        : 1;  /*bits 31 slave window enable */
            } bits;
        } dec;
    } slv_acc1;
    
    u32 _reserved5;         /* 0x0AC */
    u32 vme_mstr;           /* 0x0B0 */
    u32 _reserved6[7];      /* 0x0B4 - 0x0CC */
    
    union {
        u32 reg;            /* 0x0D0 system controller register*/
        struct {
            u32 ga              : 5;  /*bits 0 - 4 Geographic Address*/
            u32 gap             : 1;  /*bits 5 Geographic Adress Parity*/
            u32 sysctrl_status  : 1;  /*bits 6 system controller status*/
            u32 _reserved       : 1;  /*bits 7 reserved*/
            u32 sysctrl_set     : 1;  /*bits 8 activate system controller*/
            u32 bus_arb         : 1;  /*bits 9 bus arbiter*/
            u32 sreset          : 1;  /*bits 10 system reset*/
            u32 lreset          : 1;  /*bits 11 local reset*/
            u32 acfail_ebl      : 1;  /*bits 12 AC Fail detect enable*/
            u32 _reserved1      : 3;  /*bits 13 - 15 reserved*/
            u32 berrtimer       : 8;  /*bits 16 - 23 bus err timer*/
            u32 _reserved2      : 8;  /*bits 24 - 31 reserved*/
        } bits;
    } sys_ctrl;
    
    u32 _reserved7[7];      /* 0x0D4 - 0x0EC */
    
    union {
        u32 reg;            /* 0x0F0 Device control register*/
        struct {
            u32 lendian         : 1;  /*bits 0 local bus endian selection*/
            u32 _reserved       :31;  /*bits 1 - 31 reserved*/
        } bits;
    } dev_ctrl;
    
    u32 _reserved8[2];      /* 0x0F4 - 0x0F8*/
                            
    union {
        u32 reg;            /* 0x0FC device core version*/
        struct {
                u32 ver_n        : 4;  /*bits 0 - 3  core release candidate*/
                u32 ver_z        : 4;  /*bits 4 - 7 core patch version*/
                u32 ver_y        : 4;  /*bits 8 - 11 core minor version*/
                u32 ver_x        : 4;  /*bits 12 - 15 core major version*/
                u32 ver_usr      :16;  /*bits 16 - 31 user version*/
        } bits;
    } dev_ver;
    
    u32 _reserved9[12];     /* 0x100 - 0x138 Slave Access Window 8 ~ window 5 (Not used)*/
} VME_USER_CSR_REG;

/*vme local csr register 0x8000 0700*/
typedef volatile struct {
    u32 _reserved1[24];      /* 0x000 - 0x05C*/
    
    struct {
        union {
            u32 reg;         /*0x060 csr_ader[31:24]*/
            struct {
                u32 _reserved :24;  /*bits 0 - 23 reserved*/
                u32 bus_comp : 8;   /*bits 24 -31 address bus compare bits*/
            } bits;
        } ader_31_24;
        union {
            u32 reg;         /*0x064 csr_ader[23:16]*/
            struct {
                u32 _reserved :24;  /*bits 0 - 23 reserved*/
                u32 bus_comp  : 8;  /*bits 24 -31 address bus compare bits*/
            } bits;
        } ader_23_16;
        union {
            u32 reg;         /*0x068 csr_ader[15:8]*/
            struct {
                u32 _reserved :24;  /*bits 0 - 23 reserved*/
                u32 bus_comp  : 8;  /*bits 24 -31 address bus compare bits*/
            } bits;
        } ader_15_8;
        union {
            u32 reg;         /*0x06C csr_ader[7:0]*/
            struct {
                u32 _reserved :24;  /*bits 0 - 23 reserved*/
                u32 xam       : 1;  /*bits 24 xam mode*/
                u32 dfsr      : 1;  /*bits 25 dynanic function size read*/
                u32 am        : 6;  /*bits 26 -31 address modifer*/
            } bits;
        } ader_7_0;
        
    } csr_ader[NUM_SLV_WIN];
    
    u32 _reserved2[2];       /* 0x0E0 - 0x0E4 reserved*/
    u32 ubit_clr;            /* 0x0E8 */
    u32 ubit_set;            /* 0x0EC */
    u32 cram_owner;          /* 0x0F0 */
    
    union {
        u32 reg;             /* 0x0F4 */
        struct {
            u32 _reserved1 : 26;   /*bit 0-25: reserverd*/
            u32 cramox     : 1;    /*bit 26: reserverd*/
            u32 berrsx     : 1;    /*bit 27: bus error status*/
            u32 modeblc    : 1;    /*bit 28: slv access module enable*/
            u32 _reserved2 : 1;    /*bit 29: slv access module enable*/
            u32 sdec       : 1;    /*bit 30: sysfail driver enable*/
            u32 lrstc      : 1;    /*bit 31: local reset*/
        } bits;
    } bit_clr;
    
    union {
        u32 reg;             /* 0x0F8 */
        struct {
            u32 _reserved1 : 26;   /*bit 0-25: reserverd*/
            u32 cramos     : 1;    /*bit 26: reserverd*/
            u32 berrss     : 1;    /*bit 27: bus error status*/
            u32 modebls    : 1;    /*bit 28: slv access module enable*/
            u32 _reserved2 : 1;    /*bit 29: slv access module enable*/
            u32 sdes       : 1;    /*bit 30: sysfail driver enable*/
            u32 lrsts      : 1;    /*bit 31: local reset*/
        } bits;
    } bit_set;
    
    union {
        u32 reg;             /* 0x0FC csr base address register*/
        struct {
            u32 _reserved    :27; /*bits 0 - 26 reserved*/
            u32 addr         : 5; /*bits 27 - 31 cr/csr base address*/
        } bits;
    } crbar;
} VME_CSR_REG;

/*TODO: We should create common struct to hold all
 * MB FPGA memory map
 */
 /*FPGA vme geographic address register 0x43C1 0000*/
typedef volatile struct {
    union {
        u32 reg;        /*0x000 - vme geographic address */
        struct {
            u32 ga        : 5; /*bit 0 - 4 Geo grpahic address bits*/
            u32 gap       : 1; /*bit 5 Geo grpahic parity bit*/
            u32 _reserved :26; /*bit 6 - 31 reserved*/
        } bits;
    } geoaddr;
} VME_FPGA_GA_REG;

/*TODO: We should create a common struct for
 * MB common area memory map
 */
/*MB common area vme brd config register  0x43C4 0164*/
typedef volatile struct {
    union {
        u32 reg;        /*0x000 - vme brd config */
        struct {
            u32 amAddrCyle  : 1; /*bit 0: A24=1 A32=0 */
            u32 amPriv      : 1; /*bit 1: Supervisory=1 Non-Privileged=0*/
            u32 sysCtlMode  : 1; /*bit 2: SystemController=1 Slave=0 */
            u32 geoMode     : 1; /*bit 3: GeoGgraphic mode mode=1 None Geo graphic mode=0*/
            u32 irq_lv      : 3; /*bit 4-6: interrupt level 1 ~ 7*/
            u32 _reserved   : 1; /*bit 7: reserved*/
            u32 ga          : 5; /*bit 8-12: GA0-GA4*/
            u32 gap         : 1; /*bit 13: GAP*/
            u32 _reserved1  : 2; /*bit 14-15: reserved*/
            u32 vmeAddrLSB  : 8; /*bit 16-23: VME address LSB*/
            u32 vmeAddrMSB  : 8; /*bit 24-31: VME address MSB*/
        } bits;
    } config;
} VME_BRD_CONF_REG;

/*vme ip core register 0x43C8 0000*/
typedef volatile struct {
    union {
        u32 reg;            /*0x000: VME Core Irq Register*/
        struct {
            u32 usr_int_status :  18; /*bit 0-17*/
            u32 usr_intn       :  1;  /*bit 18*/
            u32 usr_resetn     :  1;  /*bit 19*/
            u32 usr_sysfailn   :  1;  /*bit 20*/
            u32 _reserved      :  2;  /*bit 21 - 22*/
            u32 usr_vint_req   :  1;  /*bit 23*/
            u32 _reserved1     :  8;  /*bit 24 - 31*/
        } bits;
    } irq;
                                     /*0x004: VME Core Slave Register*/
    union {
        u32 reg;
        struct {
            u32 _reserved      :  8; /*bit 0 - 7*/
            u32 lock           :  1;  /*bit 8*/
            u32 csr            :  1;  /*bit 9*/
            u32 ext_csr        :  1;  /*bit 10*/
            u32 _reserved1     :  9;  /*bit 11 - 19*/
            u32 dsize          :  2;  /*bit 20 - 21 0/3=D32 1=D8 2=D16*/
            u32 _reserved2     :  7;  /*bit 22 - 28*/
            u32 upper_vme_addr :  3;  /*bit 29 - 31*/
        } bits;
    } slave;
    
    u32 _reserved1;          /*0x008: VME Core Master Register (Not used)*/
    u32 _reserved2;          /*0x00C: VME Last Transaction Info Register (Not used)*/
    u32 _reserved3[60];      /*0x010 - 0x0FC: Reserved*/
    u32 _reserved4[21];      /*0x100 - 0x150: VME Master Window Start Address Register*/
    u32 _reserved5[21];      /*0x154 - 0x1A4: VME Master Window Stop Address Register*/
    u32 _reserved6[406];     /*0x1A8 - 0x7FC: Reserved*/
    u32 _reserved7;          /*0x800: VME64X CR Region Register (Not used) */
} VME_CORE_REG;

/*global variable*/
VME_CSR_REG *pVmeIpCsrReg = (VME_CSR_REG *) VME_LOCAL_CSR_BASEADDR;
VME_USER_CSR_REG *pVmeIpUsrCsrReg = (VME_USER_CSR_REG *) VME_LOCAL_USR_CSR_BASEADDR;
VME_CORE_REG *pVmeCoreReg = (VME_CORE_REG *) PS2FPGA_VME_CONFIG_BASE_ADDRESS;
VME_FPGA_GA_REG *pFpgaGaeReg = (VME_FPGA_GA_REG *) PS2FPGA_VME_GEO_ADDR_BASE_ADDR;
VME_BRD_CONF_REG *pBrdConfigReg = (VME_BRD_CONF_REG *) PS2FPGA_MB_COMMON_VME_CONFIG_BASE_ADDR;

/*static function*/
static void _en_local_csr_acc(void);
static void _en_lendian(void);
static void _dis_lendian(void);
static void _en_slv_window1_acc(void);
static void _setup_irq_lvl(void);
static void _setup_slv_window1_adr(void);
static void _int_vme_brd_config(void);
static void _int_vme_ip_core(void);
static u32 _get_align_slv_adem(void);
static u32 _get_slv_am(void);
void _print_vme_config(void);

#ifdef DEBUG
void _debug_print(void);
#endif

/*int_brd_config*/
static void _int_vme_brd_config()
{
    u32 data = 0, ret = 0;
    
    /*Is my slave fw up running*/
    if(!nai_is_slv_fw_rdy())
    {
        return;
    }
    
    /*init uart1 between mst and slave fw*/
    uart1_init();
    
    /*Ask slave for geo address*/
    ret = slave_read(SLAVE_VME_BACKPLANE_GEOADDR, 4, 1, &data, FALSE);
    if(ret)
    {
        /*set geo address*/
        pBrdConfigReg->config.bits.ga = (data & 0x1F);
        /*set geo address parity bit*/
        pBrdConfigReg->config.bits.gap = ((data & 0x20) >> 5);
    }
    
    /*Ask slave for jumper setting*/
    ret = slave_read(SLAVE_PS_GPIO_INPUT_DATA_BASEADDR_0, 4, 1, &data, FALSE);
    if(ret)
    {
        /*gpio 17 address modifer a24/a32*/
        pBrdConfigReg->config.bits.amAddrCyle = ((data & 0x20000) >> 17);
        /*gpio 18 address modifer privilege*/
        pBrdConfigReg->config.bits.amPriv = ((data & 0x40000) >> 18);
        /*gpio 19 sysctl mode*/
        pBrdConfigReg->config.bits.sysCtlMode = ((data & 0x80000) >> 19);
        /*gpio 20 geographic mode*/
        pBrdConfigReg->config.bits.geoMode = ((data & 0x100000) >> 20);
        /*gpio 23~25 irq level*/
        pBrdConfigReg->config.bits.irq_lv = ((data & 0x3800000) >> 23);
    }
    
    /*Ask slave for dip switch setting*/
    ret = slave_read(SLAVE_PS_GPIO_INPUT_DATA_BASEADDR_1, 4, 1, &data, FALSE);
    if(ret)
    {
        /*gpio 34 - 41 vme address msb*/
        pBrdConfigReg->config.bits.vmeAddrLSB = ((data & 0x3FC) >> 2);
        /*gpio 42 - 49 vme address lsb*/
        pBrdConfigReg->config.bits.vmeAddrMSB = ((data & 0x3FC00) >> 10);
    }
}

static u32 _get_slv_am()
{
    u32 am = 0;
    
    if(pBrdConfigReg->config.bits.amAddrCyle == 0)
        am = VME_AM_A24;
    else
        am = VME_AM_A32;
        
    if(pBrdConfigReg->config.bits.amPriv == 0)
        am |= VME_AM_SUP;
    else
        am |= VME_AM_USR;
    
    am |= VME_AM_DATA;
    
    return am;
}

static u32 _get_align_slv_adem()
{
    int i = 0;
    u32 size = 0, start = 0, next = 0, adem = 0;
    
    /*default slave window size to 16KB*/
    size = PS2FPGA_MB_COMMON_SIZE;
    
    /*get size of module*/
    for(i = 0; i < NUM_MOD_SLOT; i++)
    {
        size += pCommonModule->mod_size[i];
    }
    
    /*window size alawys a muliple of 256 byte*/
    start = SLV_WIN_LO_SIZE;
    next = start * 2;

    do
    {
        if((size > start) && (size < next))
        {
            size = next;
            break;
        }
        start = next;
        next *= 2;
    }
    while(start < SLV_WIN_HI_SIZE);

    
    if(size > SLV_WIN_HI_SIZE)
    {
        size = SLV_WIN_HI_SIZE;
    }

    /*get slave window addr decode mask*/
    adem = (~(size-1) >> 8);
    
    return adem;
}

static void _en_lendian()
{
    /*enable little endian mode*/
    pVmeIpUsrCsrReg->dev_ctrl.bits.lendian = 1;
}

static void _dis_lendian()
{
    /*disable little endian mode*/
    pVmeIpUsrCsrReg->dev_ctrl.reg = 0;
}

static void _en_local_csr_acc()
{
    /*set local csr access*/
    pVmeCoreReg->slave.bits.csr = 1;
    /*set local csr data size to 32bit*/
    pVmeCoreReg->slave.bits.dsize = 3;
}

static void _setup_slv_window1_adr()
{
    u32 tmp = 0;
    
    /*enable slave window module*/
    pVmeIpCsrReg->bit_set.bits.modebls = 1;
        
    /*set slv window_1 addr compare register */
    tmp = pBrdConfigReg->config.bits.vmeAddrMSB;
    pVmeIpCsrReg->csr_ader[SLV_WIN_1].ader_31_24.bits.bus_comp = tmp;
    
    tmp = pBrdConfigReg->config.bits.vmeAddrLSB;
    pVmeIpCsrReg->csr_ader[SLV_WIN_1].ader_23_16.bits.bus_comp = tmp;
        
    /*set slv window_1 addr modifer*/
    tmp = _get_slv_am();
    pVmeIpCsrReg->csr_ader[SLV_WIN_1].ader_7_0.bits.am = tmp;
}

static void _en_slv_window1_acc()
{
    u32 tmp = 0;
    
    /*set slv window_1 addr decode mask register*/
    tmp = _get_align_slv_adem();
    pVmeIpUsrCsrReg->slv_acc1.msk.bits.slvw_adem = tmp;
    
    /*enable slv window_1 access*/
    pVmeIpUsrCsrReg->slv_acc1.dec.bits.slvw_ebl = 1;
}

static void _setup_irq_lvl()
{
    u32 tmp = 0;
    
    /*enable user interrupt request*/
    pVmeIpUsrCsrReg->vint_ebl.bits.vie_uirq = 1;
    /*read brd irq level config*/
    tmp = pBrdConfigReg->config.bits.irq_lv;
    /*set user interrupter level*/
    pVmeIpUsrCsrReg->vme_int_map.bits.vint_uirq = tmp;
    /*set D08 interrupte type*/
    pVmeIpUsrCsrReg->vme_int_map.bits.vint_type = 0;
}

/*init vme ip core*/
static void _int_vme_ip_core(void)
{
    u32 tmp = 0;

    /*ask vme core to give me access ip's csr*/
    _en_local_csr_acc();
    
    /*disable little endian mode*/
    _dis_lendian();
    
    /*give the geo address to vme core*/
    pFpgaGaeReg->geoaddr.bits.ga =  pBrdConfigReg->config.bits.ga;
    pFpgaGaeReg->geoaddr.bits.gap =  pBrdConfigReg->config.bits.gap;

    /*set crbar register with csr base address*/
    tmp = pBrdConfigReg->config.bits.ga;
    tmp = tmp ^ 0x1F;
    pVmeIpCsrReg->crbar.bits.addr = tmp;
    
    /*are we in geographic mode*/
    if(pBrdConfigReg->config.bits.geoMode == 1)
    {
        /*let's enable slave window1 access and address decode mask*/
        _en_slv_window1_acc();
    }
    else
    {
        /*let's enable slave winow1 access and address decode mask*/
        _en_slv_window1_acc();
        
        /*let's setup slave window1 address and AM*/
        _setup_slv_window1_adr();

        /*let's setup interrupter*/
        _setup_irq_lvl();
    }
    
    /*enable little endian mode*/
    _en_lendian();
}

void _print_vme_config()
{
    u32 tmp = 0;
    /*ask vme core to give me access ip's csr*/
    _en_local_csr_acc();
    
    /*disable little endian mode*/
    _dis_lendian();
    
    printf("+++VME Config+++\n");
    tmp = pVmeIpUsrCsrReg->sys_ctrl.bits.sysctrl_status;
    printf("SysCon               [%s]\n",tmp ? "Y" : "N");
    tmp = pBrdConfigReg->config.bits.geoMode;
    printf("GeoMode              [%s]\n",tmp ? "Y" : "N");
    tmp = (pVmeIpCsrReg->crbar.bits.addr << 19);
    printf("CSRBAR               [0x%08x]\n",tmp);
    tmp  = (pVmeIpCsrReg->csr_ader[SLV_WIN_1].ader_31_24.bits.bus_comp << 24);
    tmp |= (pVmeIpCsrReg->csr_ader[SLV_WIN_1].ader_23_16.bits.bus_comp << 16);
    tmp |= (pVmeIpCsrReg->csr_ader[SLV_WIN_1].ader_15_8.bits.bus_comp << 8);
    printf("Slave Start Addr     [0x%08x]\n",tmp);
    tmp |= ~(pVmeIpUsrCsrReg->slv_acc1.msk.bits.slvw_adem << 8);
    printf("Slave End Addr       [0x%08x]\n",tmp);
    tmp = pVmeIpCsrReg->csr_ader[SLV_WIN_1].ader_7_0.bits.am;
    printf("Slave AM             [0x%02x]\n",tmp);
    tmp = pVmeIpUsrCsrReg->vme_int_map.bits.vint_uirq;
    printf("IRQ LV               [0x%02x]\n",tmp);
    printf("++++++++++++++++\n");
    
    /*enable little endian mode*/
    _en_lendian();    
}

#ifdef DEBUG
void _debug_print()
{
    /*ask vme core to give me access ip's csr*/
    _en_local_csr_acc();
    
    /*disable little endian mode*/
    _dis_lendian();
    
    DEBUGF("%s:**** Backplane Geo Address ***\n",__func__);
    DEBUGF("%s:ga 0x%02X \n",__func__,pBrdConfigReg->config.bits.ga);
    DEBUGF("%s:gap 0x%02X \n",__func__,pBrdConfigReg->config.bits.gap);
    DEBUGF("%s:**** Brd J3 ***\n",__func__);
    DEBUGF("%s:amAddrCyle 0x%02X \n",__func__,pBrdConfigReg->config.bits.amAddrCyle);
    DEBUGF("%s:amPriv 0x%02X \n",__func__,pBrdConfigReg->config.bits.amPriv);
    DEBUGF("%s:sysCtlMode 0x%02X \n",__func__,pBrdConfigReg->config.bits.sysCtlMode);
    DEBUGF("%s:geoMode 0x%02X \n",__func__,pBrdConfigReg->config.bits.geoMode);
    DEBUGF("%s:**** Brd J4 ***\n",__func__);
    DEBUGF("%s:irqlv 0x%02X \n",__func__,pBrdConfigReg->config.bits.irq_lv);
    DEBUGF("%s:**** Brd Dip SW2 ***\n",__func__);
    DEBUGF("%s:vmeAddrMSB 0x%02X \n",__func__,pBrdConfigReg->config.bits.vmeAddrMSB);
    DEBUGF("%s:**** Brd Dip SW3 ***\n",__func__);
    DEBUGF("%s:vmeAddrLSB 0x%02X \n",__func__,pBrdConfigReg->config.bits.vmeAddrLSB);
    DEBUGF("%s:**** VME CSR Reg ***\n",__func__);
    DEBUGF("%s:crbar.bits.addr: 0x%08X \n",__func__,pVmeIpCsrReg->crbar.bits.addr);
    DEBUGF("%s:sysctrl status: 0x%08X \n",__func__,pVmeIpUsrCsrReg->sys_ctrl.bits.sysctrl_status);
    DEBUGF("%s:**** VME CSR SLV Reg ***\n",__func__);
    DEBUGF("%s:slv module access: 0x%08X \n",__func__,pVmeIpCsrReg->bit_set.bits.modebls);
    DEBUGF("%s:slv bus comp[31:24]: 0x%08X \n",__func__,pVmeIpCsrReg->csr_ader[SLV_WIN_1].ader_31_24.bits.bus_comp);
    DEBUGF("%s:slv bus comp[23:16] 0x%08X \n",__func__,pVmeIpCsrReg->csr_ader[SLV_WIN_1].ader_23_16.bits.bus_comp);
    DEBUGF("%s:slv am[] 0x%08X \n",__func__,pVmeIpCsrReg->csr_ader[SLV_WIN_1].ader_7_0.bits.am);
    DEBUGF("%s:**** VME USR CSR SLV Reg ***\n",__func__);
    DEBUGF("%s:slv dec.slv_ebl: 0x%08X \n",__func__,pVmeIpUsrCsrReg->slv_acc1.dec.bits.slvw_ebl);
    DEBUGF("%s:slv msl.slvw_adem: 0x%08X \n",__func__,pVmeIpUsrCsrReg->slv_acc1.msk.bits.slvw_adem);
    DEBUGF("%s:slv irq level: 0x%08X \n",__func__,pVmeIpUsrCsrReg->vme_int_map.bits.vint_uirq);
    DEBUGF("%s:slv irq type: 0x%08X \n",__func__,pVmeIpUsrCsrReg->vme_int_map.bits.vint_type);
    
    /*enable little endian mode*/
    _en_lendian();
}
#endif
void vme_get_config()
{
    /*_print_vme_config*/
    _print_vme_config();
}

void vme_init()
{
    /*init vme brd config*/
    _int_vme_brd_config();
    
    /*int vme ip core*/
    _int_vme_ip_core();
        
    /*print vme config*/
    _print_vme_config();

#ifdef DEBUG    
    _debug_print();
#endif
}
    

