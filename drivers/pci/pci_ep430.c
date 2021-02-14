/*
 * EP430 PCI Bus Init
 * Description: This PCI host bridge driver is for Altera Cyclone5 SOC
 * with Eureka EP430 PCI IP
 *
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
#include <asm/processor.h>
#include <asm/io.h>
#include <pci.h>
#include <nai_mb_fpga_address.h>
#include <pci_ep430.h>

DECLARE_GLOBAL_DATA_PTR;

#undef DEBUG
//#define DEBUG
#ifdef DEBUG
#define DEBUGF(x...) printf(x)
#else
#define DEBUGF(x...)
#endif /* DEBUG */

static void non_prefetch_read(unsigned int addr, unsigned int cmd,
			      unsigned int *data);
static void non_prefetch_write(unsigned int addr, unsigned int cmd,
			       unsigned int data);

#define PCI_MEM_BUS		PS2FPGA_CPCI_PORT_ADDRESS
#define PCI_MEM_PHY		0x00000000
#define PCI_MEM_SIZE		0x04000000

#define	PCI_IO_BUS		0x00000000
#define PCI_IO_PHY		0x00000000
#define PCI_IO_SIZE		0x00010000 

/* build address value for config sycle */
static unsigned int pci_config_addr(pci_dev_t bdf, unsigned int reg) {
	unsigned int addr;

	addr = bdf | (reg & ~3);
	DEBUGF("pci_config_addr cycle type 1 addr %x\n", addr);
	
	return addr;
}

static void non_prefetch_read(unsigned int addr, unsigned int cmd,
			      unsigned int *data) {
	unsigned int config_addr = 0;
	unsigned int enable_config_addr = 0;
	unsigned int enable_config_data = 0;
	unsigned int enable_mem_op = 0;

	config_addr = PCI_EP430_PCI_CONFIG_DATA_RW_EN | addr;
	DEBUGF("non_prefetch_read config_addr %x\n", config_addr);

	//enable slave byte read and CS1
	//0x00f00010
	enable_config_addr = PCI_CORE_SLAVE_READ_BYTE_EN | PCI_CORE_SLAVE_CS1;
	DEBUGF("non_prefetch_read enable_config_addr %x\n",
	       enable_config_addr);
	//change to config_addr
	writel(enable_config_addr, PS2FPGA_CPCI_CORE_SLAVE_REG_OFFSET);

	//set device addr to config_addr
	writel(config_addr, PS2FPGA_CPCI_PORT_ADDRESS);	

	//change to config_data
	//0x00f00020
	enable_config_data = PCI_CORE_SLAVE_READ_BYTE_EN | PCI_CORE_SLAVE_CS2;
	DEBUGF("non_prefetch_read enable_config_data %x\n",
	       enable_config_data);
	writel(enable_config_data, PS2FPGA_CPCI_CORE_SLAVE_REG_OFFSET);

	//read from config_data
	*data = readl(PS2FPGA_CPCI_PORT_ADDRESS);

	//change chip select to cs0 and set mio to 1 for memory mode
	// bit 3 ~ 0
	// 3 = cs0
	// 2 = cmd1
	// 1 = cmd0
	// 0 = mio
	enable_mem_op = PCI_CORE_SLAVE_SLAVE_UW(PCI_UPPER_WINDOW_BYTE) |
			PCI_CORE_SLAVE_READ_BYTE_EN |
			PCI_CORE_SLAVE_CS0 |
			PCI_CORE_SLAVE_MIO;
	DEBUGF("non_prefetch_read enable_mem_op %x\n", enable_mem_op);
	writel(enable_mem_op, PS2FPGA_CPCI_CORE_SLAVE_REG_OFFSET);

	return;
}

static void non_prefetch_write(unsigned int addr, unsigned int cmd,
			       unsigned int data)
{
	unsigned int config_addr = 0;
	unsigned int enable_config_addr = 0;
	unsigned int enable_config_data = 0;
	unsigned int enable_mem_op = 0;

	/*setup config register*/
	config_addr = PCI_EP430_PCI_CONFIG_DATA_RW_EN | addr;
	DEBUGF("non_prefetch_write config_addr %x\n", config_addr);

	//enable slave byte read and CS1
	//0x00f00010
	enable_config_addr = PCI_CORE_SLAVE_READ_BYTE_EN | PCI_CORE_SLAVE_CS1;
	DEBUGF("non_prefetch_write enable_config_addr %x\n",
	       enable_config_addr);
	//change to config_addr register
	writel(enable_config_addr, PS2FPGA_CPCI_CORE_SLAVE_REG_OFFSET);

	//set addr to config_addr register
	writel(config_addr, PS2FPGA_CPCI_PORT_ADDRESS);	

	//change to config_data register
	//0x00f00020
	enable_config_data = PCI_CORE_SLAVE_READ_BYTE_EN | PCI_CORE_SLAVE_CS2;
	DEBUGF("non_prefetch_write enable_config_data %x\n",
	       enable_config_data);
	writel(enable_config_data, PS2FPGA_CPCI_CORE_SLAVE_REG_OFFSET);

	//write to config_data register
	writel(data, PS2FPGA_CPCI_PORT_ADDRESS);

	//change chip select to cs0 and set mio to 1 for memory mode
	// bit 3 ~ 0
	// 3 = cs0
	// 2 = cmd1
	// 1 = cmd0
	// 0 = mio
	enable_mem_op = PCI_CORE_SLAVE_SLAVE_UW(PCI_UPPER_WINDOW_BYTE) |
			PCI_CORE_SLAVE_READ_BYTE_EN |
			PCI_CORE_SLAVE_CS0 |
			PCI_CORE_SLAVE_MIO;
	DEBUGF("non_prefetch_write enable_mem_op %x\n", enable_mem_op);
	writel(enable_mem_op, PS2FPGA_CPCI_CORE_SLAVE_REG_OFFSET);
}

static int pci_ep430_hose_read_config_dword(struct pci_controller *hose,
				            pci_dev_t bdf, int where,
					    unsigned int *val)
{
	unsigned int retval;
	unsigned int addr;
	int stat = 0;

	DEBUGF("pci_ep430_hose_read_config_dword: bdf %x, reg %x \n",
	       bdf, where);
	addr = pci_config_addr(bdf, where);
	non_prefetch_read(addr, 0, &retval);
	*val = retval;

	if ( *val < 0 )
		stat = -1;

	DEBUGF("pci_ep430_hose_read_config_dword: val %x stat %x \n",
	       *val, stat);

	return stat;
}

static int pci_ep430_hose_read_config_word(struct pci_controller *hose,
				  pci_dev_t bdf, int where, unsigned short *val)
{
	unsigned int retval;
	unsigned int addr;
	unsigned int n;
	int stat = 0;

	n = where % 4;
	DEBUGF("pci_ep430_hose_read_config_word: bdf %x, reg %x \n",
	       bdf, where);
	addr = pci_config_addr(bdf, where);
	non_prefetch_read(addr, 0, &retval);
	*val = retval >> (8 * n);

	if ( *val < 0 )
		stat = -1;

	DEBUGF("pci_ep430_hose_read_config_word: val %x stat %x \n",
	       *val, stat);

	return stat;
}

static int pci_ep430_hose_read_config_byte(struct pci_controller *hose,
				  pci_dev_t bdf, int where, unsigned char *val)
{
	unsigned int retval;
	unsigned int addr;
	unsigned int n;
	int stat = 0;

	n = where % 4;
	DEBUGF("pci_ep430_hose_read_config_byte: bdf %x, reg %x \n",
	       bdf, where);
	addr = pci_config_addr(bdf, where);
	non_prefetch_read(addr, 0, &retval);
	*val = retval >> (8 * n);
	DEBUGF("pci_ep430_hose_read_config_byte: val %x \n", *val);

	if ( *val < 0 )
		stat = -1;

	DEBUGF("pci_ep430_hose_read_config_byte: val %x stat %x \n",
	       *val, stat);

	return stat;
}

static int pci_ep430_hose_write_config_byte(struct pci_controller *hose,
				   pci_dev_t bdf, int where, unsigned char val)
{
	unsigned int addr;
	unsigned int n;
	unsigned int ldata;
	int stat = 0;

	n = where % 4;
	ldata = val << (8 * n);
	addr = pci_config_addr(bdf, where);
	non_prefetch_write(addr, 0, ldata);
	
	return stat;
}

static int pci_ep430_hose_write_config_word(struct pci_controller *hose,
				   	    pci_dev_t bdf, int where,
					    unsigned short val) {
	unsigned int addr;
	unsigned int n;
	unsigned int ldata;
	int stat = 0;

	n = where % 4;
	ldata = val << (8 * n);
	addr = pci_config_addr(bdf, where);
	non_prefetch_write(addr, 0, ldata);

	return stat;	
}

static int pci_ep430_hose_write_config_dword(struct pci_controller *hose,
				    	     pci_dev_t bdf, int where,
					     unsigned int val) {
	unsigned int addr;
	int stat = 0;

	addr = pci_config_addr(bdf, where);
	non_prefetch_write(addr, 0, val);
	
	return stat;
}

void pci_ep430_init(struct pci_controller *hose) {
#ifdef CONFIG_NAI_HPS_PAGING
	/*
	 *  Config HPS Page to PCI
	 */
	unsigned int data;

	writel(PCI_PAGE_ENABLE, PS2FPGA_PS_PAGE_ADDR_OFFSET);
	data = readl(PS2FPGA_PS_PAGE_ADDR_OFFSET);
	DEBUGF("pci_ep430_init: HPS2FPGA_PAGE_DATA: %08x \n", data);
#endif	
	DEBUGF("pci_ep430_init \n");

	hose->first_busno = 0;
	hose->last_busno = 0;

	/* PCI memory space */
	pci_set_region(hose->regions + 0,
		       PCI_MEM_BUS,
		       PCI_MEM_PHY, PCI_MEM_SIZE, PCI_REGION_MEM);
	/* PCI I/O space */
	pci_set_region(hose->regions + 1,
		       PCI_IO_BUS, PCI_IO_PHY, PCI_IO_SIZE, PCI_REGION_IO);

	hose->region_count = 2;

	pci_set_ops(hose,
		    pci_ep430_hose_read_config_byte,
		    pci_ep430_hose_read_config_word,
		    pci_ep430_hose_read_config_dword,
		    pci_ep430_hose_write_config_byte,
		    pci_ep430_hose_write_config_word,
		    pci_ep430_hose_write_config_dword);

	pci_register_hose(hose);
	hose->last_busno = pci_hose_scan(hose);
}
