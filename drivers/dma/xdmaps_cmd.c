/*****************************************************************************
*
* (c) Copyright 2009-2013 Xilinx, Inc. All rights reserved.
*
* This file contains confidential and proprietary information of Xilinx, Inc.
* and is protected under U.S. and international copyright and other
* intellectual property laws.
*
* DISCLAIMER
* This disclaimer is not a license and does not grant any rights to the
* materials distributed herewith. Except as otherwise provided in a valid
* license issued to you by Xilinx, and to the maximum extent permitted by
* applicable law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND WITH ALL
* FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES AND CONDITIONS, EXPRESS,
* IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF
* MERCHANTABILITY, NON-INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE;
* and (2) Xilinx shall not be liable (whether in contract or tort, including
* negligence, or under any other theory of liability) for any loss or damage
* of any kind or nature related to, arising under or in connection with these
* materials, including for any direct, or any indirect, special, incidental,
* or consequential loss or damage (including loss of data, profits, goodwill,
* or any type of loss or damage suffered as a result of any action brought by
* a third party) even if such damage or loss was reasonably foreseeable or
* Xilinx had been advised of the possibility of the same.
*
* CRITICAL APPLICATIONS
* Xilinx products are not designed or intended to be fail-safe, or for use in
* any application requiring fail-safe performance, such as life-support or
* safety devices or systems, Class III medical devices, nuclear facilities,
* applications related to the deployment of airbags, or any other applications
* that could lead to death, personal injury, or severe property or
* environmental damage (individually and collectively, "Critical
* Applications"). Customer assumes the sole risk and liability of any use of
* Xilinx products in Critical Applications, subject only to applicable laws
* and regulations governing limitations on product liability.
*
* THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS PART OF THIS FILE
* AT ALL TIMES.
*
*****************************************************************************/

#include "xdmaps.h"

#include <command.h>

#define DMA_DEVICE_ID 	XPAR_XDMAPS_1_DEVICE_ID
#define TEST_ROUNDS	1	/* Number of loops that the Dma transfers run.*/
#define TIMEOUT_LIMIT 	0x2000	/* Loop count for timeout */

#define XDMAPS_PL_CHANNEL_START 0
#define XDMAPS_PL_CHANNEL_END   1

/*NAI_MODIFIED*/
/*static int XDmaPs_Transaction(u16 DeviceId, int *Src, int *Dst, int Len);*/
static int XDmaPs_Transaction(u16 DeviceId, int *Src, int *Dst, int Len, u8 isSrcFIFO, u8 isDestFIFO);
/*END NAI_MODIFIED*/

static void DmaDoneHandler(unsigned int Channel, XDmaPs_Cmd *DmaCmd,
			   void *CallbackRef);
static int XDmaPs_PollStatus(XDmaPs *DmaInst);

static XDmaPs DmaInstance;
static int DMA_LENGTH;

static int XDmaPs_PollStatus(XDmaPs *DmaInst) {
	u32 Fsm;
	u32 Fsc;
	u32 Stat;
	int handled = 0;
	u32 BaseAddr = DmaInst->Config.BaseAddress;

	Fsm = XDmaPs_ReadReg(BaseAddr, XDMAPS_FSM_OFFSET) & 0x01;
	Fsc = XDmaPs_ReadReg(BaseAddr, XDMAPS_FSC_OFFSET) & 0xFF;
	Stat = XDmaPs_ReadReg(BaseAddr, XDMAPS_INTSTATUS_OFFSET) & 0xFF;

	if (Fsm || Fsc) {
		XDmaPs_FaultISR(DmaInst);
		handled = 2;
	}
	else if (Stat) {
		if (Stat & 0x01)
			XDmaPs_DoneISR_0(DmaInst);
		if (Stat & 0x02)
			XDmaPs_DoneISR_1(DmaInst);
		if (Stat & 0x04)
			XDmaPs_DoneISR_2(DmaInst);
		if (Stat & 0x08)
			XDmaPs_DoneISR_3(DmaInst);
		if (Stat & 0x10)
			XDmaPs_DoneISR_4(DmaInst);
		if (Stat & 0x20)
			XDmaPs_DoneISR_5(DmaInst);
		if (Stat & 0x40)
			XDmaPs_DoneISR_6(DmaInst);
		if (Stat & 0x80)
			XDmaPs_DoneISR_7(DmaInst);

		handled = 1;
	}

	return handled;
}
/*NAI_MODIFIED*/
/*static int XDmaPs_Transaction(u16 DeviceId, int *Src, int *Dst, int Len)*/
static int XDmaPs_Transaction(u16 DeviceId, int *Src, int *Dst, int Len, u8 isSrcFIFO, u8 isDestFIFO)
/*END NAI_MODIFIED*/
{
	unsigned int Channel = 0;
	int Status;
	int TestStatus;
	int TestRound;
	int TimeOutCnt;
	volatile int Checked[XDMAPS_CHANNELS_PER_DEV];
	XDmaPs_Config *DmaCfg;
	XDmaPs *DmaInst = &DmaInstance;
	XDmaPs_Cmd DmaCmd;

	DMA_LENGTH = Len / sizeof(int);

	memset(&DmaCmd, 0, sizeof(XDmaPs_Cmd));

	DmaCmd.ChanCtrl.SrcBurstSize = 4;
	DmaCmd.ChanCtrl.SrcBurstLen = 4;
	DmaCmd.ChanCtrl.SrcInc = 1;
	DmaCmd.ChanCtrl.DstBurstSize = 4;
	DmaCmd.ChanCtrl.DstBurstLen = 4;
	DmaCmd.ChanCtrl.DstInc = 1;
	DmaCmd.BD.SrcAddr = (u32) Src;
	DmaCmd.BD.DstAddr = (u32) Dst;
	DmaCmd.BD.Length = DMA_LENGTH * sizeof(int);

/*NAI_ADDED*/
	if (isSrcFIFO > 0)
		DmaCmd.ChanCtrl.SrcInc = 0;
	if (isDestFIFO > 0)
		DmaCmd.ChanCtrl.DstInc = 0;
/*END NAI_ADDED*/

	/*
	 * Initialize the DMA Driver
	 */
	DmaCfg = XDmaPs_LookupConfig(DeviceId);
	if (DmaCfg == NULL) {
		return XST_FAILURE;
	}

	Status = XDmaPs_CfgInitialize(DmaInst,
				   DmaCfg,
				   DmaCfg->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	TestStatus = XST_SUCCESS;

	for (TestRound = 0; TestRound < TEST_ROUNDS; TestRound++) {
		xil_printf("Test round %d\r\n", TestRound);
		for (Channel = XDMAPS_PL_CHANNEL_START;
		     Channel < XDMAPS_PL_CHANNEL_END;
		     Channel++) {
			Checked[Channel] = 0;

			xil_printf("Testing channel %u\n", Channel);
			/* Set the Done interrupt handler */
			XDmaPs_SetDoneHandler(DmaInst,
					       Channel,
					       DmaDoneHandler,
					       (void *)Checked);


			Status = XDmaPs_Start(DmaInst, Channel, &DmaCmd, 0);
			if (Status != XST_SUCCESS) {
				xil_printf("Test start failed\n");
				return XST_FAILURE;
			}

			TimeOutCnt = 0;

			/* Now the DMA is done */
			while (!Checked[Channel]
			       && TimeOutCnt < TIMEOUT_LIMIT) {
				TestStatus = XDmaPs_PollStatus(DmaInst);
				if (!TestStatus) {
					TimeOutCnt++;
					continue;
				}
				else {
					TestStatus = XST_SUCCESS;
					break;
				}
			}

			if (TimeOutCnt >= TIMEOUT_LIMIT) {
				TestStatus = XST_FAILURE;
				xil_printf("zdma: channel %u fault\n", Channel);
			}
			else if (Checked[Channel] < 0) {
				/* DMA controller failed */
				TestStatus = XST_FAILURE;
				xil_printf("xdma: channel %u error\n", Channel);
			}
			else
				xil_printf("xdma: channel %u passed\n", Channel);
		}
	}

	return TestStatus;
}

static void DmaDoneHandler(unsigned int Channel, XDmaPs_Cmd *DmaCmd,
			   void *CallbackRef)
{

	/* done handler */
	volatile int *Checked = (volatile int *)CallbackRef;
	int Status = 1;
	/* We do not compare date as src or dst could be a peripheral,
	 * performing additional reads could have unwanted consequences.
	 */
#if 0
	int Index;
	int *Src;
	int *Dst;


	Src = (int *)DmaCmd->BD.SrcAddr;
	Dst = (int *)DmaCmd->BD.DstAddr;

	/* DMA successful */
	/* compare the src and dst buffer for ddr to ddr transactions */
	for (Index = 0; Index < DMA_LENGTH; Index++) {
		if ((Src[Index] != Dst[Index]) ||
		    (Dst[Index] != DMA_LENGTH - Index)) {
			Status = -XST_FAILURE;
		}
	}
#endif
	Checked[Channel] = Status;
}

static int do_xdma(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]) {
	int rc = 0;
	ulong src;
	ulong dst;
	ulong len;
/*NAI_ADDED */	
	u8 isSrcFIFO = 0;
	u8 isDestFIFO = 0;
/*END NAI_ADDED*/

	if (argc < 4)
		return cmd_usage(cmdtp);
	
	src = simple_strtoul(argv[1], NULL, 16);
	dst = simple_strtoul(argv[2], NULL, 16);
	len = simple_strtoul(argv[3], NULL, 16);
	
/*NAI_ADDED */		
	if (argc >= 5)
		isSrcFIFO = (u8)simple_strtoul(argv[4], NULL, 10);
	
	if (argc >= 6)
		isDestFIFO = (u8)simple_strtoul(argv[5], NULL, 10);
/*END NAI_ADDED*/
		
	if ((src & 0x1F) || (dst & 0x1F)) {
		printf("src and dst must be 32 byte aligned\n");
		return 0;
	}

	if (len & 0x03) {
		printf("len nust be a multiple of 4 bytes\n");
		return 0;
	}

	rc = XDmaPs_Transaction(XPAR_XDMAPS_1_DEVICE_ID,
				(int *)src, (int *)dst, len, isSrcFIFO, isDestFIFO);

	return rc;
}

/* NAI Modified - Added isSrcFIFO and isDestFIFO - allowing for DMA
 * to take place from and to FIFOs */
U_BOOT_CMD(
        xdma, CONFIG_SYS_MAXARGS, 1, do_xdma,
	"Execute DMA transactions",
	"<src> <dst> <len> <isSrcFIFO> <isDestFIFO> - Transfer len bytes from src to dst\n"
	"src, dst and len are base 16\n"
	"src and dst must be 32 byte aligned\n"
	"len must be a multiple of 4 bytes\n"
	"isSrcFIFO - 0 to increment Src addresses 1 to force no increment\n"
	"isDestFIFO - 0 to increment Dest addresses 1 to force no increment\n"
	);
