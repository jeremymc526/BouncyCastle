/*
 * Driver for the Zynq-7000 AXI I2C controller
 *
 */
#include <common.h>
#include <asm/io.h>
#include <i2c.h>
#include <linux/errno.h>
#include <asm/arch/hardware.h>
#include "zynq_axi_i2c_l.h"


#define ZYNQ_AXI_I2C_SPEED		400000
#define ZYNQ_AXI_I2C_SLAVE		0

static u32 const i2c_bases[] = {
	(u32)ZYNQ_AXI_I2C_BASEADDR0,
#if (CONFIG_NAI_MAX_AXI_I2C_PORT > 1)
	(u32)ZYNQ_AXI_I2C_BASEADDR1,
#endif	
#if (CONFIG_NAI_MAX_AXI_I2C_PORT > 2)
	(u32)ZYNQ_AXI_I2C_BASEADDR2,
#endif		
#if (CONFIG_NAI_MAX_AXI_I2C_PORT > 3)	
	(u32)ZYNQ_AXI_I2C_BASEADDR3,
#endif		
#if (CONFIG_NAI_MAX_AXI_I2C_PORT > 4)	
	(u32)ZYNQ_AXI_I2C_BASEADDR4,
#endif		
#if (CONFIG_NAI_MAX_AXI_I2C_PORT > 5)	
	(u32)ZYNQ_AXI_I2C_BASEADDR5,
#endif		
};

static u32 i2c_select(struct i2c_adapter *adap)
{
	DEBUGF("i2c_select i2c adapt# %d \n", adap->hwadapnr);
	return i2c_bases[adap->hwadapnr]; 
}

/* I2C init called by cmd_i2c when doing 'i2c reset'. */
static void zynq_axi_i2c_init(struct i2c_adapter *adap, int requested_speed,
			  int slaveadd)
{
	//TODO: AXI I2C Bus Init/Reset
}

static int xiic_ReadByte(struct i2c_adapter *adap, u8 dev, uint addr,
			 int alen, u8 *BufferPtr, u16 ByteCount)
{
	int ReceivedByteCount = 0;
	int SentByteCount = 0;
	int timeOut = XIIC_WAIT_TIMEOUT;
	u32 baseAddr = i2c_select(adap);
	
	DEBUGF("xiic_ReadByte  BaseAddr: 0x%x addr: 0x%x alen: 0x%d \n", baseAddr, addr, alen);
	do {
		SentByteCount = XIic_Send(baseAddr,
						dev,
						(u8 *)&addr,
						alen,
						XIIC_STOP);

		if (SentByteCount != alen) {
			/* Send is aborted so reset Tx FIFO */
			XIic_WriteReg(baseAddr,
					XIIC_CR_REG_OFFSET,
					XIIC_CR_TX_FIFO_RESET_MASK);
			XIic_WriteReg(baseAddr,
					XIIC_CR_REG_OFFSET,
					XIIC_CR_ENABLE_DEVICE_MASK);
		}
	} while (SentByteCount != alen && timeOut--);
	
	if(timeOut <= 0){
		DEBUGF("xiic_ReadByte SentBytCount %d Timeout !!!\n",SentByteCount );
		return -ETIMEDOUT;
	}
		
	ReceivedByteCount = XIic_Recv(baseAddr, dev,
					BufferPtr, ByteCount, XIIC_STOP);
	
	DEBUGF("xiic_ReadByte ByteCount %d ReceivedByteCount: %d SentByteCount %d timeOut %d\n",ByteCount,ReceivedByteCount, SentByteCount, timeOut);
	return ByteCount == ReceivedByteCount ? 0 :  -ETIMEDOUT;
}

static int xiic_Writebyte(struct i2c_adapter *adap, u8 dev, uint addr,
			 int alen, u8 *BufferPtr, u16 ByteCount)
{
	int SentByteCount = 0;
	int AckByteCount = 0;
	int Index = 0;
	int timeOut = XIIC_WAIT_TIMEOUT;;
	u32 baseAddr = i2c_select(adap);
	u8 WriteBuffer[alen + ByteCount];
	WriteBuffer[0] = (u8)(addr);

	DEBUGF("xiic_Writebyte  BaseAddr: 0x%x \n", baseAddr);
	
	/*
	 * Put the data in the write buffer following the address.
	 */
	for (Index = 0; Index < ByteCount; Index++) {
		WriteBuffer[alen + Index] = BufferPtr[Index];
	}
	
	/*
	 * Set the address register to the specified address by writing
	 * the address to the device, this must be tried until it succeeds
	 * because a previous write to the device could be pending and it
	 * will not ack until that write is complete.
	 */
	do {
		SentByteCount = XIic_Send(baseAddr,
					dev,
					(u8 *)&addr, alen,
					XIIC_STOP);
		if (SentByteCount != alen) {

			/* Send is aborted so reset Tx FIFO */
			XIic_WriteReg(baseAddr,  XIIC_CR_REG_OFFSET,
					XIIC_CR_TX_FIFO_RESET_MASK);
			XIic_WriteReg(baseAddr, XIIC_CR_REG_OFFSET,
					XIIC_CR_ENABLE_DEVICE_MASK);
		}

	} while (SentByteCount != alen && timeOut-- );
	
	if(timeOut <= 0){
		DEBUGF("xiic_Writebyte SentBytCount %d Timeout !!!\n",SentByteCount );
		return -ETIMEDOUT;
	}
	/*
	 * Write a data at the specified address. 
	 */
	SentByteCount = XIic_Send(baseAddr, dev,
				  WriteBuffer, (alen + ByteCount),
				  XIIC_STOP);

	/*
	 * Wait for the write to be complete by trying to do a write and
	 * the device will not ack if the write is still active.
	 */
	timeOut = XIIC_WAIT_TIMEOUT;
	do {
		AckByteCount = XIic_Send(baseAddr, dev,
					(u8 *)&addr, alen,
					XIIC_STOP);
		if (AckByteCount != alen) {

			/* Send is aborted so reset Tx FIFO */
			XIic_WriteReg(baseAddr,  XIIC_CR_REG_OFFSET,
					XIIC_CR_TX_FIFO_RESET_MASK);
			XIic_WriteReg(baseAddr, XIIC_CR_REG_OFFSET,
					XIIC_CR_ENABLE_DEVICE_MASK);
		}

	} while (AckByteCount != alen && timeOut--);
	
	if(timeOut <= 0){
		DEBUGF("xiic_Writebyte AckByteCount %d Timeout !!!\n",AckByteCount );
		return -ETIMEDOUT;
	}
		
	DEBUGF("xiic_Writebyte  SentByteCount: %d AckByteCount %d \n", SentByteCount, AckByteCount);
	/*
	 * Return the number of bytes written to the a slave
	 */
	return ByteCount == (SentByteCount - alen) ? 0 :  -ETIMEDOUT;;
}

/*
 * I2C probe called by cmd_i2c when doing 'i2c probe'.
 * Begin read, nak data byte, end.
 */
static int zynq_axi_i2c_probe(struct i2c_adapter *adap, u8 dev)
{
	u8 BufferPtr[1];
	u16 ByteCount = 1; 
	int ReceivedByteCount = 0;
	
	DEBUGF("xiic_i2c_probe DeviceAddr: %x \n",dev);
	ReceivedByteCount = xiic_ReadByte(adap, dev, 0, 1, BufferPtr, ByteCount);
	DEBUGF("xiic_i2c_probe ReceivedByteCount: %d \n",ReceivedByteCount);

	return ReceivedByteCount ? -ETIMEDOUT : 0;
}

/*
 * I2C read called by cmd_i2c when doing 'i2c read' and by cmd_eeprom.c
 * Begin write, send address byte(s), begin read, receive data bytes, end.
 */
static int zynq_axi_i2c_read(struct i2c_adapter *adap, u8 dev, uint addr,
			 int alen, u8 *data, int length)
{
	
	u8 *cur_data = data;
	u16 ByteCount = (u16)length;
	int ReceivedByteCount;
	DEBUGF("xiic_i2c_read DevAddr: %x addr %x len %x \n",dev,addr,length);
	
	if (ByteCount <= 0 || alen < 1 || alen > 1)
		return -EINVAL;	
	
	ReceivedByteCount = xiic_ReadByte(adap, dev, addr, alen, cur_data, ByteCount);
	DEBUGF("xiic_i2c_read ReceivedByteCount %d \n",ReceivedByteCount);
	return ReceivedByteCount ? -ETIMEDOUT : 0;
}

/*
 * I2C write called by cmd_i2c when doing 'i2c write' and by cmd_eeprom.c
 * Begin write, send address byte(s), send data bytes, end.
 */
static int zynq_axi_i2c_write(struct i2c_adapter *adap, u8 dev, uint addr,
			  int alen, u8 *data, int length)
{
	u8 *cur_data = data;
	int SendByteCount;
	u16 ByteCount = (u16)length;
	
	if (ByteCount <= 0 || alen < 1 || alen > 1)
		return -EINVAL;
		
	DEBUGF("xiic_i2c_write DevAddr: %x addr %x alen %d len %x \n",dev,addr, alen, length);
	SendByteCount = xiic_Writebyte(adap, dev, addr, alen, cur_data, ByteCount);
	DEBUGF("xiic_i2c_write SendByteCount %d \n",SendByteCount);
	return SendByteCount ? -ETIMEDOUT : 0;
}

static unsigned int zynq_axi_i2c_set_bus_speed(struct i2c_adapter *adap,
			unsigned int speed)
{
	//AXI I2C Bus Speed not supported
	return -ENOSYS;
}

U_BOOT_I2C_ADAP_COMPLETE(zynq_axi_0, zynq_axi_i2c_init, zynq_axi_i2c_probe, zynq_axi_i2c_read,
			 zynq_axi_i2c_write, zynq_axi_i2c_set_bus_speed,
			 ZYNQ_AXI_I2C_SPEED, ZYNQ_AXI_I2C_SLAVE,
			 0)
#if (CONFIG_NAI_MAX_AXI_I2C_PORT > 1)
U_BOOT_I2C_ADAP_COMPLETE(zynq_axi_1, zynq_axi_i2c_init, zynq_axi_i2c_probe, zynq_axi_i2c_read,
			 zynq_axi_i2c_write, zynq_axi_i2c_set_bus_speed,
			 ZYNQ_AXI_I2C_SPEED, ZYNQ_AXI_I2C_SLAVE,
			 1)
#endif			 
#if (CONFIG_NAI_MAX_AXI_I2C_PORT > 2)
U_BOOT_I2C_ADAP_COMPLETE(zynq_axi_2, zynq_axi_i2c_init, zynq_axi_i2c_probe, zynq_axi_i2c_read,
			 zynq_axi_i2c_write, zynq_axi_i2c_set_bus_speed,
			 ZYNQ_AXI_I2C_SPEED, ZYNQ_AXI_I2C_SLAVE,
			 2)
#endif			
#if (CONFIG_NAI_MAX_AXI_I2C_PORT > 3)
U_BOOT_I2C_ADAP_COMPLETE(zynq_axi_3, zynq_axi_i2c_init, zynq_axi_i2c_probe, zynq_axi_i2c_read,
			 zynq_axi_i2c_write, zynq_axi_i2c_set_bus_speed,
			 ZYNQ_AXI_I2C_SPEED, ZYNQ_AXI_I2C_SLAVE,
			 3)
#endif			 
#if (CONFIG_NAI_MAX_AXI_I2C_PORT > 4)
U_BOOT_I2C_ADAP_COMPLETE(zynq_axi_4, zynq_axi_i2c_init, zynq_axi_i2c_probe, zynq_axi_i2c_read,
			 zynq_axi_i2c_write, zynq_axi_i2c_set_bus_speed,
			 ZYNQ_AXI_I2C_SPEED, ZYNQ_AXI_I2C_SLAVE,
			 4)	
#endif	
#if (CONFIG_NAI_MAX_AXI_I2C_PORT > 5)				 
U_BOOT_I2C_ADAP_COMPLETE(zynq_axi_5, zynq_axi_i2c_init, zynq_axi_i2c_probe, zynq_axi_i2c_read,
			 zynq_axi_i2c_write, zynq_axi_i2c_set_bus_speed,
			 ZYNQ_AXI_I2C_SPEED, ZYNQ_AXI_I2C_SLAVE,
			 5)			 
#endif
