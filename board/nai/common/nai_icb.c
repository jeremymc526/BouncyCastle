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
#include <malloc.h>
#include <nai_icb.h>

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define DEBUGF(x...) printf(x)
#else
#define DEBUGF(x...)
#endif /* DEBUG */

#define ICB_NUM_CHAN             2 //
#define ICB_NUM_LANE             9 //9 data lane per channel on IOSERDES
#define ICB_ERROR               -1 // error code

#define ICB_MAX_BITSLIP                4 //Due to 4:1 ratio
#define ICB_TRAINING_PATTERN_HI        0x0000000F // B"1000" pattern for each SERDES, due to interleaving
#define ICB_TRAINING_PATTERN_LO        0xF8000000 // B"1000" pattern for each SERDES, due to interleaving
#define ICB_TRAINING_PATTERN_LANE      0x8 // B"1000" pattern for each SERDES, due to interleaving

#define ICB_NUM_TAPS             32 //32 taps per lane
#define ICB_SAMPLE_COUNT         0x00FFFFFF
#define ICB_MAX_EYE              2
#define ICB_MIN_EYE_WIDTH        4
#define ICB_MAX_EYE_WIDTH        13

#define ICB_1000US_DELAY         (1 * 1000) //25 * 1000us = 1ms
#define ICB_25000US_DELAY        (25 * 1000) //25 * 1000us = 25ms
#define ICB_1000MS_TIMEOUT       (1 * 1000) //1 * 1000ms = 1 second
#define ICB_5000MS_TIMEOUT       (5 * 1000) //5 * 1000ms = 5 seconds
#define ICB_10000MS_TIMEOUT      (10 * 1000) //10 * 1000ms = 10 seconds
#define ICB_20000MS_TIMEOUT      (20 * 1000) //20 * 1000ms = 20 seconds

/*global variable*/
/*ICB control/Status register 0x43C1 0200*/
typedef volatile struct {
                                 /*0x200: icb config/status register*/
    union {
        u32 reg;
        struct {
            u32 slave_fpga_ready   : 1;  /* Bit  0 */
            u32 _reserved1       : 3;  /* Bit  1 ~ 3 */
            u32 mst_pll_lock     : 1;  /* Bit  4 */
            u32 slv_pll_lock     : 1;  /* Bit  5 */
            u32 mst_cal_done     : 1;  /* Bit  6 */
            u32 slv_cal_done     : 1;  /* Bit  7 */
            u32 slv_fw_ready     : 1;  /* Bit  8 */
            u32 _reserved2       : 7;  /* Bit  9 ~ 15 */
            u32 mst_ready        : 1;  /* Bit  16 */
            u32 slv_resetn       : 1;  /* Bit  17 */
            u32 _reserved3       : 2;  /* Bit  18 ~ 19 */
            u32 mst_o_reset      : 1;  /* Bit  20 */
            u32 mst_i_reset      : 1;  /* Bit  21 */
            u32 slv_o_reset      : 1;  /* Bit  22 */
            u32 slv_i_reset      : 1;  /* Bit  23 */
            u32 _reserved4       : 8;  /* Bit  24 ~ 31 */
        } bits;
    } ctrl;
    
    u32 idealycntval;              /*0x204: icb idelay center value register*/
    
} ICB_CONFIG_STATUS_REGS;

/*ICB Calibration  control/status regiseter 0x43C4 8000*/
typedef volatile struct {
    u32 idcv_out_ch[2];           /*0x000 - 0x004: Channel 1&2 output delay count value register*/    
    u32 idcv_in_ch[2];            /*0x008 - 0x00C: Channel 1&2 input delay count value register*/
    u32 sample_num_ch[2];         /*0x010 - 0x014: Channel 1&2 number of sample register*/
    u32 sample_cnt_ch[2];         /*0x018 - 0x01C: Channel 1&2 sample count complete register*/
    u32 err_cnt_ch[2];            /*0x020 - 0x024: Channel 1&2 err count register*/
    u32 iserd_data_hi_ch[2];      /*0x028 - 0x02C: Channel 1&2 iserdes high data register*/
    u32 iserd_data_lo_ch[2];      /*0x030 - 0x034: Channel 1&2 iserdes low data register*/
    
                                  /*0x038 - 0x03C: Channel 1&2 load delay count register*/
    union {
        u32     reg;
        struct {
            u32     load      :  1;   /* Bit  0 */
            u32     _reserved  : 31;  /* Bits 1-31 */
        } bits;
    } idcv_ld_ch[2];
    
                                  /*0x040 - 0x044: Channel 1&2 bitslip register*/
    union {
        u32     reg;
        struct {
            u32     slip      :  1;   /* Bit  0 */
            u32     _reserved  : 31;   /* Bits 1-31 */
        } bits;
    } bitslip_ch[2];
    
                                  /*0x048 - 0x04C: Channel 1&2 error counter reset register*/
    union {
        u32     reg;
        struct {
            u32     reset      :  1;   /* Bit  0 */
            u32     _reserved  : 31;   /* Bits 1-31 */
        } bits;
    } err_cnt_rst[2];
    
                                  /*0x050 - 0x054: Channel 1&2 calibration done register*/
    union {
        u32     reg;
        struct {
            u32     ready      :  1;   /* Bit  0 */
            u32     _reserved  : 31;   /* Bits 1-31 */
        } bits;
    } cal_done[2];
    
                                  /*0x058 iserde input reset register*/
    union {
        u32     reg;
        struct {
            u32     reset      :  1;   /* Bit  0 */
            u32     _reserved  : 31;   /* Bits 1-31 */
        } bits;
    } iserd_reset_in;
    
    u32 _reserved1;               /*0x05C: Reserved*/
    
                                  /*0x060 - 0x064: Channel 1&2 data match select register*/
    union {
        u32     reg;
        struct {
            u32     select     :  4;   /* Bit 0 3*/
            u32     _reserved  : 28;   /* Bits 4-31 */
        } bits;
    } datatmatch_sel[2];
    
    u32 _reserved2[6];            /*0x068 - 0x07C: Reserved*/
    
                                  /*0x080 - 0x0A0: Channel 1 iserdes data lane 1 ~ 9 register*/
                                  /*0x0A4 - 0x0BC: Reserved*/
                                  /*0x0C0 - 0x0E0: Channel 2 iserdes data lane 1 ~ 9 register*/
                                  /*0x0E0 - 0x0FC: Reserved*/
    struct {
        u32 reg[9];
        u32 _reserved[7];
    }iserd_data[2];
    
                                  /*0x100 - 0x120: Channel 1 output delay count of lane 1 ~ 9 register*/
                                  /*0x124 - 0x13C: Reserved*/
                                  /*0x140 - 0x160: Channel 2 output delay count of lane 1 ~ 9 register*/
                                  /*0x164 - 0x17C: Reserved*/
    struct {
        u32 reg[9];
        u32 _reserved[7];
    }idcv_out[2];
    
                                  /*0x180 - 0x1A0: Channel 1 input delay count of lane 1 ~ 9 register*/
                                  /*0x1A4 - 0x1BC: Reserved*/
                                  /*0x1C0 - 0x1E0: Channel 2 input delay count of lane 1 ~ 9 register*/
                                  /*0x1E4 - 0x1FC: Reserved*/
    struct {
        u32 reg[9];
        u32 _reserved[7];
    }idcv_in[2];
    
                                  /*0x200 - 0x220: Channel 1 load delay count of lane 1 ~ 9 register*/
                                  /*0x224 - 0x23C: Reserved*/
                                  /*0x240 - 0x260: Channel 2 load delay count of lane 1 ~ 9 register*/
                                  /*0x264 - 0x27C: Reserved*/
    struct {
       u32 reg[9];
       u32 _reserved[7];
    }idcv_ld[2];
    
                                  /*0x280 - 0x2A0: Channel 1 bitslip of lane 1 ~ 9 register*/
                                  /*0x2A4 - 0x2BC: Reserved*/
                                  /*0x2C0 - 0x2E0: Channel 2 bitslip of lane 1 ~ 9 register*/
                                  /*0x2E4 - 0x2FC: Reserved*/
    struct {
        u32 reg[9];
        u32 _reserved[7];
    }bitslip[2];
    
} ICB_CAL_REGS;


ICB_CONFIG_STATUS_REGS *icbCtrlRegs = (ICB_CONFIG_STATUS_REGS *)PS2FPGA_MB_SLV_ICB_CONFIG_OFFSET;
ICB_CAL_REGS *icbCalRegs = (ICB_CAL_REGS *)ICB_CAL_BASE_ADDRESS;

#ifdef CONFIG_NAI_ICB_MST
static bool gICBEnabled     = false;
static bool gSlaveFwReady   = true;
static bool gSlaveFpgaReady = true;
#endif

#ifdef CONFIG_NAI_ICB_MST_CAL
    const u8 gMstCalData[ICB_NUM_CHAN][ICB_NUM_LANE] = CONFIG_NAI_ICB_MST_CAL;
#endif

#ifdef CONFIG_NAI_ICB_SLV_64_CAL
    const u8 gSlv64CalData[ICB_NUM_CHAN][ICB_NUM_LANE] = CONFIG_NAI_ICB_SLV_64_CAL;
#endif

#ifdef CONFIG_NAI_ICB_SLV_68_CAL
    const u8 gSlv68CalData[ICB_NUM_CHAN][ICB_NUM_LANE] = CONFIG_NAI_ICB_SLV_68_CAL;
#endif


/*function prototype*/
static void _icb_init_cal(void);
static void _icb_load_cal(u8 chan, u8 lane, u8 calData);
static s32 _icb_wa_bitslip(u8 chan, u8 lane);
static s32 _icb_wa_lane(u8 chan);
static s32 _icb_wa_all(void);
static s32 _icb_get_tap_error(u8 chan, u8 lane, u8 tap, u32 sample);
static s32 _icb_find_cal(u8 chan, u8 lane, u32 sample, bool verbose);
static u8 _icb_read_cal(u8 chan, u8 lane);
static bool _is_icb_iserdes_in_reset(void);

#ifdef CONFIG_NAI_ICB_MST
static void _icb_disable(void);
static void _icb_enable(void);
static bool _is_icb_calDone(void);
static bool _is_icb_pll_locked(void);
static bool _is_icb_enabled(void);
static bool _is_slv_fw_rdy(void);
static bool _is_slv_fpga_rdy(void);
static void _nai_slv_reset(u8 mode);
#endif /*CONFIG_NAI_ICB_MST*/

static void _icb_load_cal(u8 chan, u8 lane, u8 calData)
{
    u32 val = 0;
    DEBUGF("%s: calData: %x \n",__func__,calData);
    /*write cal data to output delay counter register*/
    icbCalRegs->idcv_out[chan].reg[lane] = calData;

    /*load cal data*/
    icbCalRegs->idcv_ld[chan].reg[lane] = 1;

    /*read back cal data from input delay counter register*/
    val = icbCalRegs->idcv_in[chan].reg[lane];

    if(val != calData)
    {
        printf("Error: loading calData 0x%08x != 0x%08x to chan %2d lane %2d\n",calData, val, chan, lane);
    }
}

static u8 _icb_read_cal(u8 chan, u8 lane)
{
    u32 ret = 0;
    
    /*read back cal data from input delay counter register*/
    ret = icbCalRegs->idcv_in[chan].reg[lane];
    
    return ret;
}

static void _icb_init_cal(void)
{
    int chan = 0;
    int lane = 0;
    u8 calData = 0;
#ifdef CONFIG_NAI_ICB_SLV
    u32 type = 0;
#endif
    DEBUGF("%s\n",__func__);
    
#ifdef CONFIG_NAI_ICB_SLV
    /*Asking SLV FPGA for Board Type*/
    type = readl(SLAVE_FPGA_BOARD_TYPE);
#endif    
    
    for(chan = 0; chan < ICB_NUM_CHAN; chan++)
    {
        for(lane = 0; lane < ICB_NUM_LANE; lane++)
        {
#ifdef CONFIG_NAI_ICB_SLV
     	   /* set icb cal based on board type
	    * 64ARM/G5 = 0x07
	    * 68ARM1 = 0x06
	    */
	   if(type == 0x06){
         calData = gSlv68CalData[chan][lane];
	   }else{
         calData = gSlv64CalData[chan][lane];
	   }
#else
	   calData = gMstCalData[chan][lane];
#endif
           /*load cal data from global static array*/
           _icb_load_cal(chan, lane, calData);

        }
    }
    
}

s32 _icb_wa_bitslip(u8 chan, u8 lane)
{
    int bitSlipCnt = 0;
    s32 ret = ICB_ERROR;
    u32 val = 0;
    
    for(bitSlipCnt = 0; bitSlipCnt <= ICB_MAX_BITSLIP; bitSlipCnt++)
    {
        /*Read data (4-bit) from individual lane*/
        val = icbCalRegs->iserd_data[chan].reg[lane];
        DEBUGF("iserd data=0x%01x\n", val);
        if(val == ICB_TRAINING_PATTERN_LANE)
        {
            DEBUGF("ICB WA Done%02d\n",lane);
            ret= 0;
            break;
        }
        else
        {
            /* set bitslip register
            * 4bit lane data is not matching with
            * training pattern
            */
            icbCalRegs->bitslip[chan].reg[lane] = 1;
            /*must wait > 3 CLK_DIV*/
            mdelay(1); 
        }
    }
    
    return ret;
}

s32 _icb_wa_lane(u8 chan)
{
    u32 loVal = 0;
    u32 hiVal = 0;
    s32 ret = ICB_ERROR;
    
    /*Read (Total/All-lanes 36-bit) value
    * from iserdes data register*/
    hiVal = icbCalRegs->iserd_data_hi_ch[chan];
    loVal = icbCalRegs->iserd_data_lo_ch[chan];
    DEBUGF("chan=%02d,IS_D_O after=0x%01x%08x \n", chan,hiVal,loVal);
    if((hiVal == ICB_TRAINING_PATTERN_HI) && (loVal == ICB_TRAINING_PATTERN_LO))
    {
        DEBUGF("WA PASSED\n");
        icbCalRegs->cal_done[chan].bits.ready = 1;
        ret = 0;
    }
    
    return ret;
}

s32 _icb_wa_all(void)
{
    int chan = 0;
    int lane = 0;
    s32 ret = 0;
    
    for(chan = 0; chan < ICB_NUM_CHAN; chan++)
    {
        for (lane = 0; lane < ICB_NUM_LANE; lane++)
        {
            ret = _icb_wa_bitslip(chan, lane);
            if(ret == ICB_ERROR)
            {
                DEBUGF("ICB WA BITSLIP Failed \n");
                goto error;
            }
        }
        ret = _icb_wa_lane(chan);
        if(ret == ICB_ERROR)
        {
            DEBUGF("ICB WA LANE Failed \n");
            goto error;
        }
    }
    
error:    
    return ret;
}

static bool _is_icb_iserdes_in_reset(void)
{
    
    bool ret = true;
    u32 data = 0;
    ulong time = 0;
    
    /*TODO: There is an issue in the MB/SLAVE FPGA
     * , when we read the iserdes register right after 
     * SW set slave read, the return data is 0 which it's not 
     * the HW default value
     * WR: wait 1ms before read iserdes register
     */ 
    udelay(ICB_1000US_DELAY);
    /*wait for master/slave iserdes out of reset*/
    time = get_timer(0);
    do
    {
        data = icbCalRegs->iserd_reset_in.bits.reset;
        data &= 0x00000001; //bitmask bit:1
        /*timeout*/
        if(get_timer(time) > ICB_10000MS_TIMEOUT)
            break;
    }
    while(data);
    
    if(data)
    {
        printf("ICB ISERDES RST TO 0x%08x \n", data);
        ret = true;
    }
    else
    {
        ret = false;
    }
    
    return ret;
}

#ifdef CONFIG_NAI_ICB_MST
/*
 * Enable ICB 
 * release Master & Slave I/O serdes from reset 
 */
static void _icb_enable(void)
{
#ifdef DEBUG
    u32 val = 0;
#endif    
    /*release master & slave serdese output from reset*/
    /*master & slave serdese output reset bit [22 20] = 0*/
    icbCtrlRegs->ctrl.bits.mst_o_reset = 0;
    icbCtrlRegs->ctrl.bits.slv_o_reset = 0;
#ifdef DEBUG
    val = icbCtrlRegs->ctrl.reg;
    DEBUGF("%s: icbCtrlRegs->ctrl.reg val=0x%08x \n",__func__,val);
#endif
    /*wait*/
    udelay(ICB_25000US_DELAY);

    /*Master&Slave input reset bit [23 21] = 0*/
    icbCtrlRegs->ctrl.bits.mst_i_reset = 0;
    icbCtrlRegs->ctrl.bits.slv_i_reset = 0;
#ifdef DEBUG
    val = icbCtrlRegs->ctrl.reg;
    DEBUGF("%s: icbCtrlRegs->ctrl.reg val=0x%08x \n",__func__,val);
#endif

    /*wait*/
    udelay(ICB_25000US_DELAY);
    
    /*set master ready [bit 16] = 1*/
    icbCtrlRegs->ctrl.bits.mst_ready = 1;
#ifdef DEBUG
    val = icbCtrlRegs->ctrl.reg;
    DEBUGF("%s: icbCtrlRegs->ctrl.reg val=0x%08x \n",__func__,val);
#endif
    /*set global enabled flag to true*/
    gICBEnabled = true;

    /*wait*/
    udelay(ICB_25000US_DELAY);
}

/*
 * Disable ICB 
 * Held Master & Slave I/O serdes in reset
 */
static void _icb_disable(void)
{
#ifdef DEBUG    
    u32 val = 0;
#endif    
    
    /*held master & slave output serdese in reset*/
    /*master & slave output reset bit [22 20] = 1*/
    icbCtrlRegs->ctrl.bits.mst_o_reset = 1;
    icbCtrlRegs->ctrl.bits.slv_o_reset = 1;
#ifdef DEBUG
    val = icbCtrlRegs->ctrl.reg;
    DEBUGF("%s: icbCtrlRegs->ctrl.reg val=0x%08x \n",__func__,val);
#endif

    /*wait*/
    udelay(ICB_25000US_DELAY);
    
    /*held master & slave input reset bit [23 21] = 1*/
    icbCtrlRegs->ctrl.bits.mst_i_reset = 1;
    icbCtrlRegs->ctrl.bits.slv_i_reset = 1;
#ifdef DEBUG
    val = icbCtrlRegs->ctrl.reg;
    DEBUGF("%s: icbCtrlRegs->ctrl.reg val=0x%08x \n",__func__,val);
#endif
   
    /*wait*/
    udelay(ICB_25000US_DELAY);

    /*clear icb master ready bit [bit 16] = 0*/
    icbCtrlRegs->ctrl.bits.mst_ready = 0;
#ifdef DEBUG
    val = icbCtrlRegs->ctrl.reg;
    DEBUGF("%s: icbCtrlRegs->ctrl.reg val=0x%08x \n",__func__,val);
#endif
    
    /*set global enabled flag to false*/
    gICBEnabled = false;
    
    /*wait*/
    udelay(ICB_25000US_DELAY);
}


/*
 * check state of ICB 
 */
static bool _is_icb_enabled(void)
{
    return gICBEnabled;
}

/*
 * check ICB PLL LOCK state
 */
static bool _is_icb_pll_locked(void)
{
    u32 val = 0;
    bool ret = false;
    ulong time = 0;
        
    time = get_timer(0);
    do
    {
        /*Master and Slave PLL Locked Bit[4:5]*/
        val = icbCtrlRegs->ctrl.bits.mst_pll_lock;
        val &= icbCtrlRegs->ctrl.bits.slv_pll_lock;
        if(get_timer(time) > ICB_1000MS_TIMEOUT)
            break;
    }
    while(!val);

    if(!val)
    {
        ret = false;
        printf("ICB PLL Locked Timeout!! val=0x%08x ret=%d \n",val,ret);
    }
    else
    {
        ret = true;
        DEBUGF("ICB PLL Locked val=0x%08x ret=%d\n", val, ret);
    }

    return val;
}

static bool _is_icb_calDone(void)
{
    u32 val = 0;
#ifdef DEBUG
    u32 debugVal = 0;
#endif
    bool ret = false;
    ulong time = 0;
    
    val = icbCtrlRegs->ctrl.bits.mst_cal_done;
    val |= icbCtrlRegs->ctrl.bits.slv_cal_done;
    DEBUGF("ICB CalDone val=0x%08x ret=%d \n", val, ret);
    
    time = get_timer(0);
    do
    {
        /*master and slave CalDone Bit[6:7]*/
        val = icbCtrlRegs->ctrl.bits.mst_cal_done;
        val |= icbCtrlRegs->ctrl.bits.slv_cal_done;
        if(get_timer(time) > ICB_1000MS_TIMEOUT)
            break;
    }
    while(!val);

#ifdef DEBUG
    /*TODO: Remove this regiseter after KF update the ICB memory map*/
    debugVal = icbCtrlRegs->idealycntval;
#endif

    if(!val)
    {
        ret = false;
        printf("ICB CalDone Timeout!! val=0x%08x ret=%d  \n", val, ret);
        DEBUGF("ICB CNT VAL=0x%08x\n", debugVal);
    }
    else
    {
        ret = true;
        DEBUGF("ICB CalDone val=0x%08x ret=%d \n", val, ret);
        DEBUGF("ICB CNT VAL=0x%08x\n", debugVal);
    }

    return ret;
}

bool _is_slv_fw_rdy(void)
{
    bool ret = false;
    u32 val = 0;
    ulong time = 0;

    if(gSlaveFwReady == false)
    {
        return false;
    }

    /*waiting for slave fw ready*/
    time = get_timer(0);
    do
    {
        val = icbCtrlRegs->ctrl.bits.slv_fw_ready;
        /*timeout*/
        if(get_timer(time) > ICB_5000MS_TIMEOUT)
            break;
            
    }while(!val);

    if(val)
    {
        DEBUGF("Slave FW is Ready 0x%08x\n", val);
        ret = true;
        gSlaveFwReady = ret;
    }
    else
    {
        ret = false;
        gSlaveFwReady = ret;
    }

    return ret;
}

bool _is_slv_fpga_rdy(void)
{
    bool ret = false;
    u32 val = 0;
    ulong time = 0;

    if(gSlaveFpgaReady == false)
    {
        return false;
    }

    //check if slave zynq is ready
    time = get_timer(0);
    do
    {
        val = icbCtrlRegs->ctrl.bits.slave_fpga_ready;
        //timeout
        if(get_timer(time) > ICB_5000MS_TIMEOUT)
            break;
    }
    while(!val);

    if(val)
    {
        DEBUGF("Slave FPGA is Ready 0x%08x\n", val);
        ret = true;
        gSlaveFpgaReady = true;
    }
    else
    {
        ret = false;
        gSlaveFpgaReady = ret;
    }

    return ret;
}

static void _nai_slv_reset(u8 mode)
{
    if(mode)
    {
        /*release slave from reset*/
        icbCtrlRegs->ctrl.bits.slv_resetn = 1;
        printf("Release slave Zynq from reset \n");
    }
    else
    {
        /*held slave in reset*/
        icbCtrlRegs->ctrl.bits.slv_resetn = 0;
        printf("Held slave Zynq in reset \n");
    }
}
/*
 * Extern Functions
 */
/*
 * Check if ICB is enabled
 */
bool nai_is_icb_enabled(void)
{
    return _is_icb_enabled();
}
/*
 * Disable ICB
 */
void nai_icb_disable(void)
{
    return _icb_disable();
}

/*
 * Enable ICB
 */
void nai_icb_enable(void)
{
    bool slaveFPGAReady = false;

    /*check mb slave fpga ready state*/
    slaveFPGAReady = _is_slv_fpga_rdy();

    if(!_is_icb_enabled() && slaveFPGAReady)
    {
        /*Disable ICB */
        _icb_disable();
        
        //verify ICB PLL Locked
        if(!_is_icb_pll_locked())
        {
            /*if icb pll locked is not ready*/
            /*ERROR: TODO Write to ERROR status register in MB common*/
            printf("ICB PLL Lock Failed!! \n");
        }
        
        /*enable ICB*/
        _icb_enable();

        /*init ICB_CAL*/
        _icb_init_cal();
        
        /*ICB_CAL Word Aligh*/
        _icb_wa_all();

        /* Verify ICB CalDone*/
        if(!_is_icb_calDone())
        {
            /*ERROR: TODO Write to ERROR status register in MB common*/
            printf("ICB CAL DONE Failed!! \n");
        }
    }
    else
    {
        printf("%s:slv fpga not rdy\n",__func__);
    }
}

bool nai_is_slv_fw_rdy(void)
{
    return _is_slv_fw_rdy();
}

bool nai_is_slv_fpga_rdy(void)
{
    return _is_slv_fpga_rdy();
}

void nai_slv_reset(u8 mode)
{
    _nai_slv_reset(mode);
}

#endif /*CONFIG_NAI_ICB_MST*/

/*
 * Get ICB Tap Error Count
 * Parameter:
 *    channel, lane, sample
 * Return:
 *    return error or count
 */
static s32 _icb_get_tap_error(u8 chan, u8 lane, u8 tap, u32 sample)
{
    ulong time = 0;
    s32 ret = ICB_ERROR;
    u32 tmp = 0;
    
    /*wait for iserdes out of reset*/
    if(_is_icb_iserdes_in_reset()){
        printf("%s: Error ISERDES is in Reset\n\r",__func__);
        return ret;
    }
    
    /*Clear ICB Calibration Done Bit*/
    icbCalRegs->cal_done[chan].bits.ready = 0;
    
    /*2. Set sample number per channel*/
    /*TODO remove the hardcode value for number of sample */
    icbCalRegs->sample_num_ch[chan] = sample;
    
    /*3. Held error counter logic in reset */
    icbCalRegs->err_cnt_rst[chan].bits.reset = 1;
    
    /*4. Load tap Parameter */
    /*set tap value output channel*/
    icbCalRegs->idcv_out[chan].reg[lane] = tap;
    /*load tap value */
    icbCalRegs->idcv_ld[chan].reg[lane] = 1;
    /*set data match */
    tmp = lane + 1;
    icbCalRegs->datatmatch_sel[chan].bits.select = tmp;
    
    /*read input channel*/
    tmp = icbCalRegs->idcv_in[chan].reg[lane];
    
    if(tmp != tap)
    {
        printf("%s: failed to read back on delay input channel \n",__func__);
        goto error;
    }

    tmp = icbCalRegs->datatmatch_sel[chan].bits.select;
    if(tmp != lane + 1)
    {
        printf("%s: set data match channel failed\n",__func__);
        goto error;
    }
    
    /*Release error counter logic from reset*/
    icbCalRegs->err_cnt_rst[chan].bits.reset = 0;
    
    /*wait for error sample counter logic*/
    time = get_timer(0);
    do
    {
        tmp = icbCalRegs->sample_cnt_ch[chan];

        if(get_timer(time) > ICB_20000MS_TIMEOUT)
        {
            printf("%s: wait for error sample count timeout\n",__func__);
            goto error;
        }
  
    }
    while(tmp != sample);
    
    /*read error count*/
    ret = icbCalRegs->err_cnt_ch[chan];
    
    /* Held error counter logic in reset */
    icbCalRegs->err_cnt_rst[chan].bits.reset = 1;
    
error:
    return ret;
}

/*
 * Find ICB calibration data
 *    A valid calibration data must meet the following
 *    requirements:
 *        1. It must have 2 eyes in a 32 taps scan
 *        2. The width of a valid eye is between 5 and 12
 *        3. Use the first center of eye in the 32 tap as the valid calibration data
 * Note: This requirement is based on the ICB calibration implementation in FPGA.
 * 
 * Returns:
 *     success - return calibration data
 *     failed - return ICB_ERROR
 */
static s32 _icb_find_cal(u8 chan, u8 lane, u32 sample, bool verbose)
{
    int tap = 0;
    int currErr = 0, prevErr = 0, startEyeErrCnt = 0, endEyeErrCnt = 0;
    int endEyeTap = 0, startEyeTap = 0, eyeCenter = 0, eyeWidth = 0;
    int foundEyeCnt = 0;
    s32 ret = ICB_ERROR;
    bool foundStartEye = false, foundEndEye = false;
    
    /*scan error count from 32 taps*/
    for(tap = 0; tap < ICB_NUM_TAPS; tap++)
    {
        currErr = _icb_get_tap_error(chan,lane,tap,sample);
        
        if(currErr != ICB_ERROR)
        {
            if(verbose)
            {
                printf("Chan = %02d Lane = %02d Tap Index = %02d Error Count = %d \n",chan,lane,tap,currErr);
            }
            if(false == foundStartEye || false == foundEndEye)
            {
                if((0 == currErr) && (0 != prevErr) && (false == foundStartEye))
                {
                    startEyeErrCnt = prevErr;
                    startEyeTap = tap;
                    foundStartEye = true;
                }
            
                if((0 != currErr) && (0 == prevErr) && (true == foundStartEye))
                {
                    endEyeErrCnt = currErr;
                    endEyeTap = tap - 1;
                    foundEndEye = true;
                }
            
                prevErr = currErr;
                
                /*found an eye*/
                if(true == foundStartEye && true == foundEndEye)
                {
                    /*calucatue the eye width*/
                    eyeWidth = (endEyeTap - startEyeTap) + 1;
                    if((ICB_MIN_EYE_WIDTH < eyeWidth) && (ICB_MAX_EYE_WIDTH > eyeWidth))
                    {
                       if((eyeWidth % 2) == 1)
                       {
                          /*eye width is odd then take the middle tap*/
                          eyeCenter = startEyeTap + (eyeWidth/2);
                       }
                       else
                       {
                           /* eye width is even and the start 
                            * of a eye error count is less 
                            * than end of a eye error count than
                            * use the tap close to the start of a eye as
                            * the center of a eye*/
                           if( startEyeErrCnt < endEyeErrCnt )
                           {
                               eyeCenter = (startEyeTap + (eyeWidth/2)) - 1;
                           }
                           else
                           {
                              eyeCenter = startEyeTap + (eyeWidth/2);
                           }
                       }
                       /*found the first valid eye*/
                       if(foundEyeCnt == 0)
                           ret = eyeCenter;
                    }//if
                    /*reset and look for another eye*/
                    foundStartEye = false;
                    foundEndEye = false;
                    /*increment eye count*/
                    foundEyeCnt++;
                }//if
            }//if
         }//if
         else
         {
              ret = ICB_ERROR;
              goto error;
         }
         
    }//for loop

    if(foundEyeCnt != ICB_MAX_EYE)
        ret = ICB_ERROR;

#ifdef DEBUG
    printf("EyeStartTap: %2u, ",startEyeTap);
    printf("EyeEndTap: %2u, ",endEyeTap);
    printf("EyeCenterTap: %d \n",ret);
    printf("EyeFoundCnt: %2u \n",foundEyeCnt);
    printf("EyeWidth: %2u, ",eyeWidth);
    printf("EyeStartErrTap: %2u, ",startEyeTap-1);
    printf("EyeEndErrTap: %2u, \n",endEyeTap+1);
#endif

error:
    return ret;
}

void nai_icb_print_cal()
{
    int chan = 0, lane = 0;
    
    /*print calibration data*/
    for(chan = 0; chan < ICB_NUM_CHAN; chan++)
    {
        printf("Channel :%d \n", chan); 
        for(lane = 0; lane < ICB_NUM_LANE; lane++)
        {
            printf("%2d,",_icb_read_cal(chan,lane)); 
        }
            printf("\n");
    }
}

void nai_icb_gen_all_cal(u32 sample, bool verbose)
{
    int chan = 0;
    int lane = 0;
    u32 ret = 0;
    s8 cal[ICB_NUM_CHAN][ICB_NUM_LANE];
    
    for(chan = 0; chan < ICB_NUM_CHAN; chan++)
    {
        for(lane = 0; lane < ICB_NUM_LANE; lane++)
        {
            cal[chan][lane] = _icb_find_cal(chan, lane, sample, verbose);
        }
    }
    
    /*load cal data*/
    for(chan = 0; chan < ICB_NUM_CHAN; chan++)
    {
        for(lane = 0; lane < ICB_NUM_LANE; lane++)
        {
            /*load cal data from global static array*/
            if(cal[chan][lane] != ICB_ERROR)
            {
                _icb_load_cal(chan, lane, cal[chan][lane]);
            }
            else
            {
                printf("Cal Err: Channel %02d Lane %02d Data %2d \n",chan,lane,cal[chan][lane]);
            }
        }
    }
    
    /*run word alignment*/
    ret = _icb_wa_all();
    
    if(ret == ICB_ERROR)
        printf("Cal Err: WA Failed \n");
}

void nai_icb_gen_cal(u8 chan, u8 lane, u32 sample, bool verbose)
{
    s32 ret = ICB_ERROR;
    
    if(chan > ICB_NUM_CHAN || lane > ICB_NUM_LANE)
    {
        printf("Cal Err: Invalid Channel %2d Lane %2d \n",chan,lane);
    }
    ret = _icb_find_cal(chan, lane, sample, verbose);
    
    if(ret == ICB_ERROR)
        printf("Cal Err: Channel %2d Lane %2d Data %2d \n",chan,lane,ret);
    else
        printf("Channel %2d Lane %2d Data %2d \n",chan,lane,ret);
}

/*
 * Init ICB Calibration data
 */
void nai_icb_init_cal(void)
{
    s32 ret = 0;

    /*init ICB cal data*/
    _icb_init_cal();
    /*icb word alignment*/
    ret = _icb_wa_all();
    if (ret == ICB_ERROR)
    {
        printf("ICB WA Failed\n");
    }
}

/*
 * check ICB input SEREDES reset state
 */
bool nai_is_icb_iserdes_in_reset(void){
    return  _is_icb_iserdes_in_reset();
}
