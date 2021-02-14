/*
 * Command for testing slave Zynq.
 *
 * Copyright (C) 2015 North Atlantic Industries, Inc.
 */

#include <common.h>
#include <command.h>
#include <malloc.h>
#include <master_slave.h>

static int do_slave(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    u8      op;
    u32     count = 1;
    u32     width = 0;
    u32     addr = 0;
    u32     mask = 0;
    u32     val = 0;
    char    *str = NULL;
    u8      *pBuf;
    u32     bytes;

    if (argc < 3)
        return CMD_RET_USAGE;

    // Get operation
    op = *argv[1];
    if (op == 'e')
    {
        // 3 args
        if (argc != 3)
            return CMD_RET_USAGE;
    }
    else if (op == 'r')
    {
        // Max 4 args
        if (argc > 4)
            return CMD_RET_USAGE;
    }
    else if (op == 'w')
    {
        // 4 args
        if (argc != 4)
            return CMD_RET_USAGE;
    }
    else if (op == 's' || op == 'f')
    {
        // 5 args
        if (argc != 5)
            return CMD_RET_USAGE;
    }
    else
    {
        return CMD_RET_USAGE;
    }

    // Get width
    if (op != 'e' && op != 'f')
    {
        width = simple_strtoul(&argv[1][1], NULL, 10);
        if (width != 8 && width != 16 && width != 32)
            return CMD_RET_USAGE;
    }
    width /= 8;

    if (op == 'e')
    {
        // Get string to send
        str = argv[2];
    }
    else
    {
        // Get address
        addr = simple_strtoul(argv[2], NULL, 16);
    }

    if (argc > 3)
    {
        if (op == 'r')
        {
            // Get count
            count = simple_strtoul(argv[3], NULL, 10);
        }
        else if (op == 'w')
        {
            // Get value
            val = simple_strtoul(argv[3], NULL, 16);
        }
        else if (op == 's')
        {
            // Get mask
            mask = simple_strtoul(argv[3], NULL, 16);
        }
        else if (op == 'f')
        {
            // Get count
            count = simple_strtoul(argv[3], NULL, 16);
        }
    }

    if (argc > 4)
    {
        // Get value
        val = simple_strtoul(argv[4], NULL, 16);
    }

    // All good - do it!!!
    if (op == 'e')
    {
        slave_echo((u8 *)str, strlen(str), NULL, TRUE);
    }
    else if (op == 'r')
    {
        slave_read(addr, width, count, NULL, TRUE);
    }
    else if (op == 'w')
    {
        slave_write(addr, width, count, &val, TRUE);
    }
    else if (op == 's')
    {
        slave_setclr(addr, width, mask, val, TRUE);
    }
    else if (op == 'f')
    {
        puts("\n");
        if (addr > 0x1000000)
        {
            puts("Offset > flash size (0x1000000)\n");
            return CMD_RET_FAILURE;
        }
        if (count % 0x10000)
        {
            puts("Count not multiple of flash sector size (0x10000)\n");
            return CMD_RET_FAILURE;
        }
        pBuf = (u8 *)val;
        addr += 0xFC000000;
        bytes = 4 * MAX_RW_COUNT;
        while (count)
        {
            if ((count & 0xFFFF) == 0)
                puts(".");
            if (!slave_write(addr, 4, MAX_RW_COUNT, (u32 *)pBuf, FALSE))
            {
                puts("\n\nslave_write() failed\n");
                return CMD_RET_FAILURE;
            }
            addr += bytes;
            pBuf += bytes;
            count -= bytes;
        }
        puts("\n\nDone\n");
    }
    puts("\n");

    return CMD_RET_SUCCESS;
}


U_BOOT_CMD(
    slave, 5, 0, do_slave,
    "Slave Zynq test",
    "echo <str> - Echo input string\n"
    "    str    : String to send\n"
    "slave r<8/16/32> addr [count] - Read (8/16/32 bit) memory/registers\n"
    "    addr   : Address in HEX\n"
    "    count  : Count in DEC; default = 1\n"
    "slave w<8/16/32> addr val - Write (8/16/32 bit) memory/registers\n"
    "    addr   : Address in HEX\n"
    "    val    : Value in HEX\n"
    "slave s<8/16/32> addr mask val - Set/Clear (8/16/32 bit) memory/registers\n"
    "    addr   : Address in HEX\n"
    "    mask   : Mask in HEX\n"
    "    val    : Value in HEX\n"
    "slave fu offset len buf - Flash Update\n"
    "    offset : Offset in HEX\n"
    "    len    : Length in HEX\n"
    "    buf    : Buffer address in HEX\n"
);

