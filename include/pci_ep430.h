/*
 * EP430 PCI Bus Init
 * Description: This PCI host bridge driver is for Altera Cyclone5 SOC with Eureka EP430 PCI IP
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

#ifndef _PCI_430_H
#define _PCI_430_H

#define OK	0
#define ERROR	-1

struct pci_controller;
extern void pci_ep430_init(struct pci_controller *hose);

#ifdef CONFIG_NAI_HPS_PAGING
#define PCI_PAGE_ENABLE 			BIT(1)
#endif

#define PCI_UPPER_WINDOW_BYTE			(PS2FPGA_CPCI_PORT_ADDRESS >> 24)

#define PCI_CORE_SLAVE_MIO			BIT(0)
#define PCI_CORE_SLAVE_CMD0 			BIT(1)
#define PCI_CORE_SLAVE_CMD1   			BIT(2)	
#define PCI_CORE_SLAVE_CS0   			BIT(3)
#define PCI_CORE_SLAVE_CS1   			BIT(4)
#define PCI_CORE_SLAVE_CS2   			BIT(5)
#define PCI_CORE_SLAVE_FAST   			BIT(6)
#define PCI_CORE_SLAVE_READ_BYTE_EN   		(0xf << 20)
#define PCI_CORE_SLAVE_SLAVE_UW(x)   		((x) << 24) 

#define PCI_CORE_STATUS_READ_ERR		BIT(0)
#define PCI_CORE_STATUS_READY			BIT(1)
#define PCI_CORE_STATUS_DATA_PARITY_ERR		BIT(2)
#define PCI_CORE_STATUS_SIGNAL_TARGET_ABORT	BIT(3)
#define PCI_CORE_STATUS_RECEIVED_TARGET_ABORT	BIT(4)
#define PCI_CORE_STATUS_RECEIVED_MASTER_ABORT	BIT(5)
#define PCI_CORE_STATUS_SIGNAL_SYSTEM_ERR	BIT(6)
#define PCI_CORE_STATUS_DETECTED_PARITY_ERR	BIT(7)

/*EUREKA EP430 Register bit definitions*/
#define PCI_EP430_PCI_CONFIG_DATA_RW_EN  	BIT(31)

#endif
