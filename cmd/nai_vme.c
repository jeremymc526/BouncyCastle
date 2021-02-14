/*
 * (C) Copyright 2015 NAII, nai@naii.com
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <command.h>
#include <malloc.h>
#include <asm/io.h>
#include <nai_vme.h>

int do_vme(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    u8 op;

    if(argc < 2)
        return CMD_RET_USAGE;
     
    // Get operation
    op = *argv[1];
    if (op == 'c')
    {
        if (argc < 2)
            return CMD_RET_USAGE;
    }
    else
    {
        return CMD_RET_USAGE;
    }
    /*I'm done let do it*/
    
    switch(op)
    {
        case 'c':    /*dump vme configuration*/
          vme_get_config();
          break;
        default:
          return CMD_RET_USAGE;
    }
    return 0;
}

U_BOOT_CMD(
	vme,	2,	1,	do_vme,
	"VME Command",
    "config :Get VME Config\n"
);
