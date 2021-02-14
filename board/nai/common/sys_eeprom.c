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
#include <command.h>
#include <i2c.h>
#include <linux/ctype.h>

#define MAX_NUM_PORTS	2

#undef DEBUG 
/**
 * static eeprom: EEPROM layout for NAI Maximillion AT24CS02-MAHM-T
 */
static struct __attribute__ ((__packed__)) eeprom {
	u8 mb_opcode[4];     	/* 0x00 MB Special Option Code */
	u8 pcb_date[4];        	/* 0x04 PCB Build Date */
	u8 pcb_revision[4];    	/* 0x08 PCB Revision */
	u8 not_used[4];    		/* 0x0C Not Used*/
	u8 prod_sn[4];     		/* 0x10 Product Serial Number*/
	u8 platform[4];     	/* 0x14 Platform */
	u8 model[4];     		/* 0x18 Model */
	u8 gen[4];     			/* 0x1C Generation */
	u8 proc_count[2];     	/* 0x20 proccessor Count */
	u8 eth_count[2];     	/* 0x22 Eth Port Count */
	u8 module_count[4];    	/* 0x24 Module Slot on MB Count */
	u8 eth0_mac[8];    		/* 0x28 ETH0 MAC Addr */
	u8 eth0_name[8];    	/* 0x30-0x34 ETH0 MAC Name */
	u8 eth0_ipaddr[4];  	/* 0x38	ETH0 Static IP Addr */
	u8 eth0_subnet_mask[4]; /* 0x3C ETH0 Subnet Mask */
	u8 eth0_gateway[4];  	/* 0x40 ETH0 Gateway */
	u8 eth0_ipv6addr[16];  	/* 0x44 ETH0 IPv6 Addr */
	u8 eth0_ipv6prefix[4];  /* 0x54 ETH0 IPv6 Prefix */
	u8 eth1_mac[8];    		/* 0x58 ETH1 MAC Addr */
	u8 eth1_name[8];    	/* 0x60 ETH1 MAC Name */
	u8 eth1_ipaddr[4];  	/* 0x68	ETH1 Static IP Addr */
	u8 eth1_subnet_mask[4]; /* 0x6C ETH1 Subnet Mask */
	u8 eth1_gateway[4];  	/* 0x70 ETH1 Gateway */
	u8 eth1_ipv6addr[16];  	/* 0x74 ETH1 IPv6 Addr */
	u8 eth1_ipv6prefix[4];  /* 0x84 ETH1 IPv6 Prefix */
	u8 boot_count[4];		/* 0x88 Boot Count */ 
	u8 underfiled_date[4];	/* 0x8C Underfilled Date */
	u8 coated_date[4];		/* 0x90 Conformal Coated Date */
	u8 reword1_date[4];		/* 0x94 Rework 1 Date */
	u8 reword1_code[4];		/* 0x98 Rework 1 Code */
	u8 reword2_date[4];		/* 0x9C Rework 2 Date */
	u8 reword2_code[4];		/* 0xA0 Rework 2 code */
	u8 reword3_date[4];		/* 0xA4 Rework 3 Date */
	u8 reword3_code[4];		/* 0xA8 Rework 3 Code */
	u8 reword4_date[4];		/* 0xAC Rework 4 Date */
	u8 reword4_code[4];		/* 0xB0 Rework 4 Date */
#ifndef CONFIG_DISABLE_EEPROM_CRC	
	u32 crc;          		/* 0xB4 CRC32 checksum */
#endif	
} e;

/* Set to 1 if we've read EEPROM into memory */
static int has_been_read = 0;

/**
 * show_eeprom - display the contents of the EEPROM
 */
static void show_eeprom(void)
{
#ifdef DEBUG
	int i;
#endif

#ifndef CONFIG_DISABLE_EEPROM_CRC	
	unsigned int crc;
#endif
	if (!has_been_read) {
		printf("Please read the EEPROM ('read') first.\n");
		return;
	}
	
	/* MB Special Option Code */
	printf("MB Special Option Code: %s \n",
			e.mb_opcode);
	/* PCB Build Date */
	printf("PCB Date: %c%c%c%c \n",
			e.pcb_date[0],e.pcb_date[1],e.pcb_date[2],e.pcb_date[3]);
	/* PCB Revision */
	printf("PCB Revison: %c%c%c%c \n", 
			e.pcb_revision[0],e.pcb_revision[1],e.pcb_revision[2],e.pcb_revision[3]);
	/* Product Serial number */
	printf("Product SN: %d%d%d%d \n", 
			e.prod_sn[0], e.prod_sn[1],e.prod_sn[2],e.prod_sn[3]);
	/* Platform */
	printf("Platform: %s \n", 
			e.platform);
	/* Model */
	printf("Model: %s \n", 
			e.model);
	/* Generation */
	printf("Generation: %s \n", 
			e.gen);
	/* Proccessor Count  */
	printf("Proccessor Count: %d%d \n", 
			e.proc_count[1],e.proc_count[0]);
	/* Eth Port Count */
	printf("Eth Port Count: %d%d \n", 
			e.eth_count[1], e.eth_count[0]);
	/* Module_count Count */
	printf("Module Count: %d%d \n", 
			e.module_count[1],e.module_count[0]);
	/* ETH0 MAC Addr */
	printf("ETH0 MAC Addr: %02x:%02x:%02x:%02x:%02x:%02x \n", 
			e.eth0_mac[0], e.eth0_mac[1],e.eth0_mac[2],e.eth0_mac[3],e.eth0_mac[4],e.eth0_mac[5]);
	/* ETH0 MAC Name */
	printf("ETH0 MAC Name: %s \n", 
			e.eth0_name);
	/* ETH0 Static IP Addr */
	printf("ETH0 Static IP Addr: %d.%d.%d.%d \n",
			e.eth0_ipaddr[0],e.eth0_ipaddr[1],e.eth0_ipaddr[2],e.eth0_ipaddr[3]);  	
	/* ETH0 Subnet Mask */
	printf("ETH0 Subnet Mask: %d.%d.%d.%d \n",
			e.eth0_subnet_mask[0],e.eth0_subnet_mask[1],e.eth0_subnet_mask[2],e.eth0_subnet_mask[3]);  	
	/* ETH0 Gateway */
	printf("ETH0 Gateway: %d.%d.%d.%d \n",
			e.eth0_gateway[0],e.eth0_gateway[1],e.eth0_gateway[2],e.eth0_gateway[3]);
	/* ETH0 IPv6 Addr */
	printf("ETH0 IPv6 Addr: %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x \n",
			e.eth0_ipv6addr[0],e.eth0_ipv6addr[1],e.eth0_ipv6addr[2],e.eth0_ipv6addr[3],
			e.eth0_ipv6addr[4],e.eth0_ipv6addr[5],e.eth0_ipv6addr[6],e.eth0_ipv6addr[7],
			e.eth0_ipv6addr[8],e.eth0_ipv6addr[9],e.eth0_ipv6addr[10],e.eth0_ipv6addr[11],
			e.eth0_ipv6addr[12],e.eth0_ipv6addr[13],e.eth0_ipv6addr[14],e.eth0_ipv6addr[15]);
	/* ETH0 IPv6 Prefix */
	printf("ETH0 IPv6 Prefix: %s \n",
			e.eth0_ipv6prefix);
	
	/* ETH1 MAC Addr */
	printf("ETH1 MAC Addr: %02x:%02x:%02x:%02x:%02x:%02x \n", 
			e.eth1_mac[0], e.eth1_mac[1], e.eth1_mac[2],e.eth1_mac[3],e.eth1_mac[4],e.eth1_mac[5]);
	/* ETH1 MAC Name */
	printf("ETH1 MAC Name: %s \n", 
			e.eth1_name);
	/* ETH1 Static IP Addr */
	printf("ETH1 Static IP Addr: %d.%d.%d.%d \n",
			e.eth1_ipaddr[0],e.eth1_ipaddr[1],e.eth1_ipaddr[2],e.eth1_ipaddr[3]);  	
	/* ETH1 Subnet Mask */
	printf("ETH1 Subnet Mask: %d.%d.%d.%d \n",
			e.eth1_subnet_mask[0],e.eth1_subnet_mask[1],e.eth1_subnet_mask[2],e.eth1_subnet_mask[3]); 
	/* ETH1 Gateway */
	printf("ETH1 Gateway: %d.%d.%d.%d \n",
			e.eth1_gateway[0],e.eth1_gateway[1],e.eth1_gateway[2],e.eth1_gateway[3]);
	/* ETH1 IPv6 Prefix */
	printf("ETH1 IPv6 Addr: %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x \n",
			e.eth1_ipv6addr[0],e.eth1_ipv6addr[1],e.eth1_ipv6addr[2],e.eth1_ipv6addr[3],
			e.eth1_ipv6addr[4],e.eth1_ipv6addr[5],e.eth1_ipv6addr[6],e.eth1_ipv6addr[7],
			e.eth1_ipv6addr[8],e.eth1_ipv6addr[9],e.eth1_ipv6addr[10],e.eth1_ipv6addr[11],
			e.eth1_ipv6addr[12],e.eth1_ipv6addr[13],e.eth1_ipv6addr[14],e.eth1_ipv6addr[15]);
	/* ETH1 IPv6 Prefix */
	printf("ETH1 IPv6 Prefix: %s \n",
			e.eth0_ipv6prefix);
	
#ifndef CONFIG_DISABLE_EEPROM_CRC		
	crc = crc32(0, (void *)&e, sizeof(e) - 4);

	if (crc == be32_to_cpu(e.crc))
		printf("CRC: %08x\n", be32_to_cpu(e.crc));
	else
		printf("CRC: %08x (should be %08x)\n",
			be32_to_cpu(e.crc), crc);
#endif

#ifdef DEBUG
	printf("EEPROM dump: (0x%x bytes)\n", sizeof(e));
	for (i = 0; i < sizeof(e); i++) {
		if ((i % 16) == 0)
			printf("%02X: ", i);
		printf("%02X ", ((u8 *)&e)[i]);
		if (((i % 16) == 15) || (i == sizeof(e) - 1))
			printf("\n");
	}
#endif
}

/**
 * read_eeprom - read the EEPROM into memory (eeprom structure)
 */
static int read_eeprom(void)
{
	int ret;
#ifdef CONFIG_SYS_EEPROM_BUS_NUM
	unsigned int bus;
#endif

	if (has_been_read)
		return 0;

#ifdef CONFIG_SYS_EEPROM_BUS_NUM
	bus = i2c_get_bus_num();
	i2c_set_bus_num(CONFIG_SYS_EEPROM_BUS_NUM);
#endif

	ret = i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR, 0, CONFIG_SYS_I2C_EEPROM_ADDR_LEN,
		(void *)&e, sizeof(e));

#ifdef CONFIG_SYS_EEPROM_BUS_NUM
	i2c_set_bus_num(bus);
#endif

	has_been_read = (ret == 0) ? 1 : 0;

#ifdef DEBUG
	show_eeprom();
#endif

	return ret;
}

#ifndef CONFIG_DISABLE_EEPROM_CRC	
/**
 *  update_crc - update the CRC
 *
 *  This function should be called after each update to the EEPROM structure,
 *  to make sure the CRC is always correct.
 */
static void update_crc(void)
{
	u32 crc;

	crc = crc32(0, (void *)&e, sizeof(e) - 4);
	e.crc = cpu_to_be32(crc);
}
#endif

/**
 * prog_eeprom - write the EEPROM from memory eeprom structure
 */
static int prog_eeprom(void)
{
	int ret = 0;
	int i;
	void *p;
#ifdef CONFIG_SYS_EEPROM_BUS_NUM
	unsigned int bus;
#endif

#ifndef CONFIG_DISABLE_EEPROM_CRC	
	update_crc();
#endif

#ifdef CONFIG_SYS_EEPROM_BUS_NUM
	bus = i2c_get_bus_num();
	i2c_set_bus_num(CONFIG_SYS_EEPROM_BUS_NUM);
#endif

	/*
	 * The AT24C02 datasheet says that data can only be written in page
	 * mode, which means 8 bytes at a time, and it takes up to 5ms to
	 * complete a given write.
	 */
	for (i = 0, p = &e; i < sizeof(e); i += 8, p += 8) {
		ret = i2c_write(CONFIG_SYS_I2C_EEPROM_ADDR, i, CONFIG_SYS_I2C_EEPROM_ADDR_LEN,
			p, min((sizeof(e) - i), 8U));
		if (ret)
			break;
		udelay(5000);	/* 5ms write cycle timing */
	}

	if (!ret) {
		/* Verify the write by reading back the EEPROM and comparing */
		struct eeprom e2;
		printf("Verify Write\n");
		ret = i2c_read(CONFIG_SYS_I2C_EEPROM_ADDR, 0,
			CONFIG_SYS_I2C_EEPROM_ADDR_LEN, (void *)&e2, sizeof(e2));
		if (!ret && memcmp(&e, &e2, sizeof(e)))
			ret = -1;
	}

#ifdef CONFIG_SYS_EEPROM_BUS_NUM
	i2c_set_bus_num(bus);
#endif

	if (ret) {
		printf("Programming failed.\n");
		has_been_read = 0;
		return -1;
	}

	printf("Programming passed.\n");
	return 0;
}

/**
 * h2i - converts hex character into a number
 *
 * This function takes a hexadecimal character (e.g. '7' or 'C') and returns
 * the integer equivalent.
 */
static inline u8 h2i(char p)
{
	if ((p >= '0') && (p <= '9'))
		return p - '0';

	if ((p >= 'A') && (p <= 'F'))
		return (p - 'A') + 10;

	if ((p >= 'a') && (p <= 'f'))
		return (p - 'a') + 10;

	return 0;
}

#if 0
/**
 * set_pcb_date - stores the build date into the EEPROM
 *
 * This function takes a pointer to a string in the format "YYWW"
 * (2-digit year, 2-digit week of the year, etc), converts it to a 4-byte BCD string,
 * and stores it in the build date field of the EEPROM local copy.
 */
static void set_pcb_date(const char *string)
{
	unsigned int i;

	if (strlen(string) != 4) {
		printf("Usage: mac date YYWW\n");
		return;
	}

	for (i = 0; i < 4; i++)
		e.pcb_date[i] = h2i(string[2 * i]) << 4 | h2i(string[2 * i + 1]);

#ifndef CONFIG_DISABLE_EEPROM_CRC	
	update_crc();
#endif

}
#endif

/**
 * set_mac_address - stores a MAC address into the EEPROM
 *
 * This function takes a pointer to MAC address string
 * (i.e."XX:XX:XX:XX:XX:XX", where "XX" is a two-digit hex number) and
 * stores it in one of the MAC address fields of the EEPROM local copy.
 */
static void set_mac_address(unsigned int index, const char *string)
{
	char *p = (char *) string;
	unsigned int i;

	if ((index >= MAX_NUM_PORTS) || !string) {
		printf("Usage: mac [0~%d] XX:XX:XX:XX:XX:XX\n",(MAX_NUM_PORTS-1));
		return;
	}
	
	
	for (i = 0; *p && (i < 6); i++) {
		
		if (index == 0)
			e.eth0_mac[i] = simple_strtoul(p, &p, 16);
		else if (index == 1)
			e.eth1_mac[i] = simple_strtoul(p, &p, 16);
		else
			break;
			
		if (*p == ':')
			p++;
	}
	
	printf("ETH0 MAC Addr: %02x:%02x:%02x:%02x:%02x:%02x\n", 
			e.eth0_mac[0], e.eth0_mac[1], e.eth0_mac[2],e.eth0_mac[3],e.eth0_mac[4],e.eth0_mac[5]);
	printf("ETH1 MAC Addr: %02x:%02x:%02x:%02x:%02x:%02x\n", 
			e.eth1_mac[0], e.eth1_mac[1], e.eth1_mac[2],e.eth1_mac[3],e.eth1_mac[4],e.eth1_mac[5]);

#ifndef CONFIG_DISABLE_EEPROM_CRC		
	update_crc();
#endif	
}

int do_mac(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char cmd;
	u8 eth_count;

	cmd = argv[1][0];
	
	if (argc == 1) {
		show_eeprom();
		return 0;
	}

	if (cmd == 'r') {
		read_eeprom();
		return 0;
	}
	
	if (cmd == 'm') {
		mac_read_from_eeprom();
		return 0;
	}
	
	if (cmd == 'i') {
		printf("EEPROM ('i') is not available \n");
		return 0;
	}

	if (!has_been_read) {
		printf("Please read the EEPROM ('read') first.\n");
		return 0;
	}

	if (argc == 2) {
		switch (cmd) {
		case 's':	/* save */
			prog_eeprom();
			break;
		default:
			return cmd_usage(cmdtp);
		}

		return 0;
	}

	/* We know we have at least one parameter  */

	switch (cmd) {
	case 'n':	/* serial number */
		printf("EEPROM ('n') Serial Number Update is not available \n");
#if 0
		memset(e.prod_sn, 0, sizeof(e.prod_sn));
		strncpy((char *)e.prod_sn, argv[2], sizeof(e.prod_sn) - 1);
#ifndef CONFIG_DISABLE_EEPROM_CRC	
		update_crc();
#endif
#endif		
		break;
	case 'e':	/* errata */
		printf("EEPROM ('e') Errat Update is not available \n");
		break;
	case 'd':	/* date BCD format YYWW */
		printf("EEPROM ('d') Build Date Update is not available \n");
#if 0
		set_pcb_date(argv[2]);
#endif		
		break;
	case 'p':	/* Number of Ethport */
		eth_count = simple_strtoul(argv[2], NULL, 16);
		if (eth_count > MAX_NUM_PORTS){
			printf("EEPROM ('p') Max Eth Ports is %d \n", MAX_NUM_PORTS);
			break;
		}
		e.eth_count[0] = eth_count;
#ifndef CONFIG_DISABLE_EEPROM_CRC	
		update_crc();
#endif		
		break;
	case '0' ... '9':	/* "mac 0" through "mac 22" */
		set_mac_address(simple_strtoul(argv[1], NULL, 10), argv[2]);
		break;
	case 'h':	/* help */
	default:
		return cmd_usage(cmdtp);
	}

	return 0;
}

/**
 * mac_read_from_eeprom - read the MAC addresses from EEPROM
 *
 * This function reads the MAC addresses from EEPROM and sets the
 * appropriate environment variables for each one read.
 *
 * The environment variables are only set if they haven't been set already.
 * This ensures that any user-saved variables are never overwritten.
 *
 * This function must be called after relocation.
 *
 */
int mac_read_from_eeprom(void)
{
	unsigned int i;
#ifndef CONFIG_DISABLE_EEPROM_CRC		
	u32 crc, crc_offset = offsetof(struct eeprom, crc);
	u32 *crcp; /* Pointer to the CRC in the data read from the EEPROM */
#endif	
	char ethaddr[18];
	char enetvar[9];
	
	puts("EEPROM: \n");

	if (read_eeprom()) {
		printf("Read failed.\n");
		return -1;
	}
#ifndef CONFIG_DISABLE_EEPROM_CRC	
	crc = crc32(0, (void *)&e, crc_offset);
	crcp = (void *)&e + crc_offset;
	if (crc != be32_to_cpu(*crcp)) {
		printf("CRC mismatch (%08x != %08x)\n", crc, be32_to_cpu(e.crc));
		return -1;
	}
#endif
	for (i = 0; i < MAX_NUM_PORTS; i++) {
			
		if (i == 0) {
			/*Verify MAC Addr */
			if (memcmp(&e.eth0_mac, "\0\0\0\0\0\0", 6) &&
				memcmp(&e.eth0_mac, "\xFF\xFF\xFF\xFF\xFF\xFF", 6)) {
				
				sprintf(ethaddr, "%02X:%02X:%02X:%02X:%02X:%02X",
					e.eth0_mac[0],
					e.eth0_mac[1],
					e.eth0_mac[2],
					e.eth0_mac[3],
					e.eth0_mac[4],
					e.eth0_mac[5]);
				sprintf(enetvar, i ? "eth%daddr" : "ethaddr", i);
				printf("%s ethaddr: %s \n",enetvar, ethaddr);
				/* Only initialize environment variables that are blank
				* (i.e. have not yet been set)
				*/
				if (!env_get(enetvar))
					env_set(enetvar, ethaddr);
			}
			
		} else if (i == 1) {
			/*Verify MAC Addr */
			if (memcmp(&e.eth1_mac, "\0\0\0\0\0\0", 6) &&
				memcmp(&e.eth1_mac, "\xFF\xFF\xFF\xFF\xFF\xFF", 6)) {
				

				sprintf(ethaddr, "%02X:%02X:%02X:%02X:%02X:%02X",
					e.eth1_mac[0],
					e.eth1_mac[1],
					e.eth1_mac[2],
					e.eth1_mac[3],
					e.eth1_mac[4],
					e.eth1_mac[5]);
				sprintf(enetvar, i ? "eth%daddr" : "ethaddr", i);
				printf("%s ethaddr: %s \n",enetvar, ethaddr);
				/* Only initialize environment variables that are blank
				* (i.e. have not yet been set)
				*/
				if (!env_get(enetvar))
					env_set(enetvar, ethaddr);
			}
		} else {
			break;
		}
	}			
	return 0;
}
