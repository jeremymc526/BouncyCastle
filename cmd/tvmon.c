/*
 * Command for monitoring temperature and voltage.
 *
 * Copyright (C) 2014 North Atlantic Industries, Inc.
 */

#include <common.h>
#include <command.h>
#include <console.h>
#include <asm/io.h>


/* +++ Definitions from xparameters.h ***************************************************************************/
#define XPAR_XADCPS_0_DEVICE_ID 0
#define XPAR_XADCPS_0_BASEADDR 0xF8007100
/* --- Definitions from xparameters.h ***************************************************************************/


/* +++ Typedefs, etc from standalone_v4_1 BSP *******************************************************************/
typedef int XStatus;

#define XST_SUCCESS                     0L
#define XST_FAILURE                     1L

#define Xil_AssertNonvoid(x)
#define Xil_AssertVoid(x)

#ifndef TRUE
#  define TRUE		1
#endif
#ifndef FALSE
#  define FALSE		0
#endif

#define XIL_COMPONENT_IS_READY     0x11111111  /**< component has been initialized */
/* --- Typedefs, etc from standalone_v4_1 BSP *******************************************************************/


/* +++ Definitions, etc from xadcps_v2_0 driver *****************************************************************/
#define XADCPS_CH_TEMP		0x0  /**< On Chip Temperature */
#define XADCPS_CH_VPVN		0x3  /**< VP/VN Dedicated analog inputs */
#define XADCPS_CH_VBRAM		0x6  /**< On-chip VBRAM Data Reg, 7 series */
#define XADCPS_CH_VCCPINT	0x0D /**< On-chip PS VCCPINT Channel , Zynq */
#define XADCPS_CH_VCCPAUX	0x0E /**< On-chip PS VCCPAUX Channel , Zynq */
#define XADCPS_CH_VCCPDRO	0x0F /**< On-chip PS VCCPDRO Channel , Zynq */
#define XADCPS_CH_AUX_MAX	 31 /**< Channel number for Last Aux channel */

#define XADCPS_SEQ_MODE_SAFE		0  /**< Default Safe Mode */
#define XADCPS_SEQ_MODE_ONEPASS		1  /**< Onepass through Sequencer */
#define XADCPS_CFR1_SEQ_SHIFT		  12     /**< Sequence bit shift */

#define XADCPS_CFR1_OFFSET	0x41	/**< Configuration Register 1 */
#define XADCPS_CFR1_SEQ_VALID_MASK	  0xF000 /**< Sequence bit Mask */

#define XADCPS_TEMP_OFFSET		  0x00 /**< On-chip Temperature Reg */

typedef struct {
	u16  DeviceId;		/**< Unique ID of device */
	u32  BaseAddress;	/**< Device base address */
} XAdcPs_Config;
typedef struct {
	XAdcPs_Config Config;	/**< XAdcPs_Config of current device */
	u32  IsReady;		/**< Device is initialized and ready  */
} XAdcPs;

#define XAdcPs_RawToTemperature(AdcData)				\
	((((float)(AdcData)/65536.0f)/0.00198421639f ) - 273.15f)
#define XAdcPs_RawToVoltage(AdcData) 					\
	((((float)(AdcData))* (3.0f))/65536.0f)

#define XADCPS_JTAG_DATA_MASK		0x0000FFFF /**< Mask for the Data */
#define XADCPS_JTAG_ADDR_MASK		0x03FF0000 /**< Mask for the Addr */
#define XADCPS_JTAG_ADDR_SHIFT		16	   /**< Shift for the Addr */
#define XADCPS_JTAG_CMD_WRITE_MASK	0x08000000 /**< Mask for CMD Write */
#define XADCPS_JTAG_CMD_READ_MASK	0x04000000 /**< Mask for CMD Read */

#define XAdcPs_FormatWriteData(RegOffset, Data, ReadWrite) 	    \
    ((ReadWrite ? XADCPS_JTAG_CMD_WRITE_MASK : XADCPS_JTAG_CMD_READ_MASK ) | \
     ((RegOffset << XADCPS_JTAG_ADDR_SHIFT) & XADCPS_JTAG_ADDR_MASK) | 	     \
     (Data & XADCPS_JTAG_DATA_MASK))

#define XADCPS_CFG_OFFSET	 0x00 /**< Configuration Register */
#define XADCPS_CMDFIFO_OFFSET	 0x10 /**< Command FIFO Register */
#define XADCPS_RDFIFO_OFFSET	 0x14 /**< Read FIFO Register */
#define XADCPS_MCTL_OFFSET	 0x18 /**< Misc control register */

#define XADCPS_CFG_ENABLE_MASK	 0x80000000 /**< Enable access from PS mask */
#define XADCPS_CFG_CFIFOTH_MASK  0x00F00000 /**< Command FIFO Threshold mask */
#define XADCPS_CFG_DFIFOTH_MASK  0x000F0000 /**< Data FIFO Threshold mask */

#define XAdcPs_ReadReg(BaseAddress, RegOffset) \
			(readl((BaseAddress) + (RegOffset)))
#define XAdcPs_WriteReg(BaseAddress, RegOffset, Data) \
		(writel((Data), (BaseAddress) + (RegOffset)))

#define XAdcPs_ReadFifo(InstancePtr)				\
	XAdcPs_ReadReg((InstancePtr)->Config.BaseAddress,	\
			  XADCPS_RDFIFO_OFFSET);
#define XAdcPs_WriteFifo(InstancePtr, Data)				\
	XAdcPs_WriteReg((InstancePtr)->Config.BaseAddress,		\
			  XADCPS_CMDFIFO_OFFSET, Data);

#define XADCPS_UNLK_OFFSET	 0x034 /**< Unlock Register */
#define XADCPS_UNLK_VALUE	 0x757BDF0D /**< Unlock Value */

#define XADCPS_CFR0_OFFSET	0x40	/**< Configuration Register 0 */

#define XADCPS_CFR0_AVG_SHIFT	 	12     /**< Averaging bits shift */
#define XADCPS_CFR0_AVG_VALID_MASK	0x3000 /**< Averaging bit Mask */

#define XADCPS_AVG_16_SAMPLES	1  /**< Average 16 samples */
#define XADCPS_AVG_256_SAMPLES	3  /**< Average 256 samples */

#define XADCPS_SEQ_MODE_SIMUL_SAMPLING	4  /**< Simultaneous sampling */
#define XADCPS_SEQ_MODE_INDEPENDENT	8  /**< Independent mode */

#define XADCPS_SEQ00_OFFSET	0x48 /**< Seq Reg 00 Adc Channel Selection */
#define XADCPS_SEQ01_OFFSET	0x49 /**< Seq Reg 01 Adc Channel Selection */

#define XADCPS_SEQ00_CH_VALID_MASK  0x7FE1 /**< Mask for the valid channels */
#define XADCPS_SEQ01_CH_VALID_MASK  0xFFFF /**< Mask for the valid channels */

#define XADCPS_SEQ_CH_AUX_SHIFT	16 /**< Shift for the Aux Channel */

#define XADCPS_SEQ_CH_VCCPINT	0x00000020 /**< VCCPINT, Zynq Only */
#define XADCPS_SEQ_CH_VCCPAUX	0x00000040 /**< VCCPAUX, Zynq Only */
#define XADCPS_SEQ_CH_VCCPDRO	0x00000080 /**< VCCPDRO, Zynq Only */
#define XADCPS_SEQ_CH_TEMP	0x00000100 /**< On Chip Temperature Channel */
#define XADCPS_SEQ_CH_VPVN	0x00000800 /**< VP/VN analog inputs Channel */

static XAdcPs_Config XAdcPs_ConfigTable[] =
{
	{
		XPAR_XADCPS_0_DEVICE_ID,
		XPAR_XADCPS_0_BASEADDR
	}
};

static XAdcPs_Config *XAdcPs_LookupConfig(u16 DeviceId);
static int XAdcPs_CfgInitialize(XAdcPs *InstancePtr, XAdcPs_Config *ConfigPtr, u32 EffectiveAddr);
static void XAdcPs_SetAvg(XAdcPs *InstancePtr, u8 Average);
static int XAdcPs_SetSeqChEnables(XAdcPs *InstancePtr, u32 ChEnableMask);
static u8 XAdcPs_GetSequencerMode(XAdcPs *InstancePtr);
static void XAdcPs_SetSequencerMode(XAdcPs *InstancePtr, u8 SequencerMode);
static u16 XAdcPs_GetAdcData(XAdcPs *InstancePtr, u8 Channel);
static u32 XAdcPs_ReadInternalReg(XAdcPs *InstancePtr, u32 RegOffset);
static void XAdcPs_WriteInternalReg(XAdcPs *InstancePtr, u32 RegOffset, u32 Data);
/* --- Definitions, etc from xadcps_v2_0 driver *****************************************************************/


/* LM60 temperature sensor definitions */
#define LM60_INT2FLOAT      ((0.015259 * 3) / 2)    // 1000 * 1/1^16 * 2/3: 1000mv/v * 1/2^16--16 bits * resistor divider of 2/3
#define LM60_ZERO_OFFSET    424                     // Temp sensor generates 424mv at 0 deg. C
#define LM60_GAIN           6.25                    // Temp sensor has a gain of 6.25mv/deg. C

/* Global variables */
static XAdcPs XAdcInstance;

/* Local functions */
static XStatus if_temp_volt_init(void);
static void if_volt_temp_read(u8 ch, int *vt_i, int *vt_f);
static int XAdcFractionToInt(float fp_num);
static float XAdcRawToTemp(u32 data);


static int do_tvmon(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    static bool init = false;
    int  vt_i[5], vt_f[5];

    if (argc > 1)
        return CMD_RET_USAGE;

    if (!init)
    {
        if_temp_volt_init();
        init = true;
    }

    puts("\n Board_T   Core_T VccCore  VccAux  VccDDR\n");
    puts("-------- -------- ------- ------- -------\n");
    while (1)
    {
        if_volt_temp_read(XADCPS_CH_VPVN, &vt_i[0], &vt_f[0]);
        if_volt_temp_read(XADCPS_CH_TEMP, &vt_i[1], &vt_f[1]);
        if_volt_temp_read(XADCPS_CH_VCCPINT, &vt_i[2], &vt_f[2]);
        if_volt_temp_read(XADCPS_CH_VCCPAUX, &vt_i[3], &vt_f[3]);
        if_volt_temp_read(XADCPS_CH_VCCPDRO, &vt_i[4], &vt_f[4]);

        printf("%3d.%03dC %3d.%03dC %2d.%03dV %2d.%03dV %2d.%03dV\r",
            vt_i[0], vt_f[0], vt_i[1], vt_f[1], vt_i[2], vt_f[2],
            vt_i[3], vt_f[3], vt_i[4], vt_f[4]);

        mdelay(1000);
        if (ctrlc())
            break;
    }
    puts("\n\n");

    return CMD_RET_SUCCESS;
}


static XStatus if_temp_volt_init(void)
{
    XAdcPs_Config   *cfg = NULL;
    XStatus         rc;

    /* Lookup XADC configuration */
    cfg = XAdcPs_LookupConfig(XPAR_XADCPS_0_DEVICE_ID);
    if (!cfg)
        return XST_FAILURE;

    /* Initialize XADC driver */
    memset(&XAdcInstance, 0, sizeof(XAdcInstance));
    rc = XAdcPs_CfgInitialize(&XAdcInstance, cfg, cfg->BaseAddress);
    if (rc)
        return rc;

    /* Setup Channel Sequencer to read desired channels */
    XAdcPs_SetSequencerMode(&XAdcInstance, XADCPS_SEQ_MODE_SAFE);
    XAdcPs_SetAvg(&XAdcInstance, XADCPS_AVG_16_SAMPLES);
    rc = XAdcPs_SetSeqChEnables(&XAdcInstance, XADCPS_SEQ_CH_TEMP | XADCPS_SEQ_CH_VPVN |
         XADCPS_SEQ_CH_VCCPINT | XADCPS_SEQ_CH_VCCPAUX | XADCPS_SEQ_CH_VCCPDRO);
    XAdcPs_SetSequencerMode(&XAdcInstance, XADCPS_SEQ_MODE_ONEPASS);

    return rc;
}


static void if_volt_temp_read(u8 ch, int *vt_i, int *vt_f)
{
    u32     data;
    float   vt;

    /* Sequence channel select registers */
    XAdcPs_SetSequencerMode(&XAdcInstance, XADCPS_SEQ_MODE_SAFE);
    XAdcPs_SetSequencerMode(&XAdcInstance, XADCPS_SEQ_MODE_ONEPASS);

    /* Read data from ADC registers */
    data = XAdcPs_GetAdcData(&XAdcInstance, ch);

    /* Convert it to voltage/temperature */
    if (ch == XADCPS_CH_TEMP)
        vt = XAdcPs_RawToTemperature(data);
    else if (ch == XADCPS_CH_VPVN)
        vt = XAdcRawToTemp(data);
    else
        vt = XAdcPs_RawToVoltage(data);

    if (vt_i)
        *vt_i = (int)vt;
    if (vt_f)
        *vt_f = XAdcFractionToInt(vt);
}


static int XAdcFractionToInt(float fp_num)
{
    float   temp;

    if (fp_num < 0)
        temp = -fp_num;
    else
        temp = fp_num;

    return(((int)((temp - (float)((int)temp)) * (1000.0f))));
}


static float XAdcRawToTemp(u32 data)
{
    float temp;

    /* Convert to float, subtract offset, scale by gain */
    temp = ((float)(data * LM60_INT2FLOAT) - LM60_ZERO_OFFSET) / LM60_GAIN;

    return (temp);
}


/* +++ Functions from xadcps_v2_0 driver ************************************************************************/
static XAdcPs_Config *XAdcPs_LookupConfig(u16 DeviceId)
{
	XAdcPs_Config *CfgPtr = NULL;
	u32 Index;

	for (Index=0; Index < 1; Index++) {
		if (XAdcPs_ConfigTable[Index].DeviceId == DeviceId) {
			CfgPtr = &XAdcPs_ConfigTable[Index];
			break;
		}
	}

	return CfgPtr;
}


static int XAdcPs_CfgInitialize(XAdcPs *InstancePtr, XAdcPs_Config *ConfigPtr,
				u32 EffectiveAddr)
{

	u32 RegValue;
	/*
	 * Assert the input arguments.
	 */
	Xil_AssertNonvoid(InstancePtr != NULL);
	Xil_AssertNonvoid(ConfigPtr != NULL);

	/*
	 * Set the values read from the device config and the base address.
	 */
	InstancePtr->Config.DeviceId = ConfigPtr->DeviceId;
	InstancePtr->Config.BaseAddress = EffectiveAddr;

	/* Write Unlock value to Device Config Unlock register */
	XAdcPs_WriteReg((InstancePtr)->Config.BaseAddress,
				XADCPS_UNLK_OFFSET, XADCPS_UNLK_VALUE);

	/* Enable the PS access of xadc and set FIFO thresholds */

	RegValue = XAdcPs_ReadReg((InstancePtr)->Config.BaseAddress,
			XADCPS_CFG_OFFSET);

	RegValue = RegValue | XADCPS_CFG_ENABLE_MASK |
			XADCPS_CFG_CFIFOTH_MASK | XADCPS_CFG_DFIFOTH_MASK;

	XAdcPs_WriteReg((InstancePtr)->Config.BaseAddress,
					XADCPS_CFG_OFFSET, RegValue);

	/* Release xadc from reset */

	XAdcPs_WriteReg((InstancePtr)->Config.BaseAddress,
						XADCPS_MCTL_OFFSET, 0x00);

	/*
	 * Indicate the instance is now ready to use and
	 * initialized without error.
	 */
	InstancePtr->IsReady = XIL_COMPONENT_IS_READY;

	return XST_SUCCESS;
}


static void XAdcPs_SetAvg(XAdcPs *InstancePtr, u8 Average)
{
	u32 RegData;

	/*
	 * Assert the arguments.
	 */
	Xil_AssertVoid(InstancePtr != NULL);
	Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
	Xil_AssertVoid(Average <= XADCPS_AVG_256_SAMPLES);

	/*
	 * Write the averaging value into the Configuration Register 0.
	 */
	RegData = XAdcPs_ReadInternalReg(InstancePtr,
					XADCPS_CFR0_OFFSET) &
					(~XADCPS_CFR0_AVG_VALID_MASK);

	RegData |=  (((u32) Average << XADCPS_CFR0_AVG_SHIFT));
	XAdcPs_WriteInternalReg(InstancePtr, XADCPS_CFR0_OFFSET,
					RegData);

}


static int XAdcPs_SetSeqChEnables(XAdcPs *InstancePtr, u32 ChEnableMask)
{
	/*
	 * Assert the arguments.
	 */
	Xil_AssertNonvoid(InstancePtr != NULL);
	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	/*
	 * The sequencer must be disabled for writing any of these registers
	 * Return XST_FAILURE if the channel sequencer is enabled.
	 */
	if ((XAdcPs_GetSequencerMode(InstancePtr) != XADCPS_SEQ_MODE_SAFE)) {
		return XST_FAILURE;
	}

	/*
	 * Enable the specified channels in the ADC Channel Selection Sequencer
	 * Registers.
	 */
	XAdcPs_WriteInternalReg(InstancePtr,
				XADCPS_SEQ00_OFFSET,
				(ChEnableMask & XADCPS_SEQ00_CH_VALID_MASK));

	XAdcPs_WriteInternalReg(InstancePtr,
				XADCPS_SEQ01_OFFSET,
				(ChEnableMask >> XADCPS_SEQ_CH_AUX_SHIFT) &
				XADCPS_SEQ01_CH_VALID_MASK);

	return XST_SUCCESS;
}


static u8 XAdcPs_GetSequencerMode(XAdcPs *InstancePtr)
{
	/*
	 * Assert the arguments.
	 */
	Xil_AssertNonvoid(InstancePtr != NULL);
	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

	/*
	 * Read the channel sequencer mode from the Configuration Register 1.
	 */
	return ((u8) ((XAdcPs_ReadInternalReg(InstancePtr,
			XADCPS_CFR1_OFFSET) & XADCPS_CFR1_SEQ_VALID_MASK) >>
			XADCPS_CFR1_SEQ_SHIFT));

}


static void XAdcPs_SetSequencerMode(XAdcPs *InstancePtr, u8 SequencerMode)
{
	u32 RegValue;

	/*
	 * Assert the arguments.
	 */
	Xil_AssertVoid(InstancePtr != NULL);
	Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
	Xil_AssertVoid((SequencerMode <= XADCPS_SEQ_MODE_SIMUL_SAMPLING) ||
			(SequencerMode == XADCPS_SEQ_MODE_INDEPENDENT));

	/*
	 * Set the specified sequencer mode in the Configuration Register 1.
	 */
	RegValue = XAdcPs_ReadInternalReg(InstancePtr,
					XADCPS_CFR1_OFFSET);
	RegValue &= (~ XADCPS_CFR1_SEQ_VALID_MASK);
	RegValue |= ((SequencerMode  << XADCPS_CFR1_SEQ_SHIFT) &
					XADCPS_CFR1_SEQ_VALID_MASK);
	XAdcPs_WriteInternalReg(InstancePtr, XADCPS_CFR1_OFFSET,
				RegValue);

}


static u16 XAdcPs_GetAdcData(XAdcPs *InstancePtr, u8 Channel)
{

	u32 RegData;

	/*
	 * Assert the arguments.
	 */
	Xil_AssertNonvoid(InstancePtr != NULL);
	Xil_AssertNonvoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);
	Xil_AssertNonvoid((Channel <= XADCPS_CH_VBRAM) ||
			 ((Channel >= XADCPS_CH_VCCPINT) &&
			 (Channel <= XADCPS_CH_AUX_MAX)));

	RegData = XAdcPs_ReadInternalReg(InstancePtr,
						(XADCPS_TEMP_OFFSET +
						Channel));
	return (u16) RegData;
}


static u32 XAdcPs_ReadInternalReg(XAdcPs *InstancePtr, u32 RegOffset)
{

	u32 RegData;

	RegData = XAdcPs_FormatWriteData(RegOffset, 0x0, FALSE);

	/* Read cmd to FIFO*/
	XAdcPs_WriteFifo(InstancePtr, RegData);

	/* Do a Dummy read */
	RegData = XAdcPs_ReadFifo(InstancePtr);

	/* Do a Dummy write to get the actual read */
	XAdcPs_WriteFifo(InstancePtr, RegData);

	/* Do the Actual read */
	RegData = XAdcPs_ReadFifo(InstancePtr);

	return RegData;

}


static void XAdcPs_WriteInternalReg(XAdcPs *InstancePtr, u32 RegOffset, u32 Data)
{
	u32 RegData;

	/*
	 * Write the Data into the FIFO Register.
	 */
	RegData = XAdcPs_FormatWriteData(RegOffset, Data, TRUE);

	XAdcPs_WriteFifo(InstancePtr, RegData);

	/* Read the Read FIFO after any write since for each write
	 * one location of Read FIFO gets updated
	 */
	XAdcPs_ReadFifo(InstancePtr);

}
/* --- Functions from xadcps_v2_0 driver ************************************************************************/


U_BOOT_CMD(
    tvmon, 1, 0, do_tvmon,
    "Temperature and Voltage Monitor",
    ""
);

