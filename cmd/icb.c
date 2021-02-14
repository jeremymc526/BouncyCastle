/*
 * Command for testing slave Zynq.
 *
 * Copyright (C) 2015 North Atlantic Industries, Inc.
 */

#include <common.h>
#include <command.h>
#include <malloc.h>
#include <nai_icb.h>

static int do_icb(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    u8   op;
    u32  chan = 0;
    u32  lane = 0;
    u32  sample = 0xFFFFFF;
    bool verbose = false;
    
    if (argc < 2)
    {
        return CMD_RET_USAGE;
    }

    if (argc > 1 && argc < 4)
    {
        op = *argv[1];
        if(op == 'd')
        {
            /*dump calibration data*/
            nai_icb_print_cal();
            return CMD_RET_SUCCESS;
        }
        
        sample = simple_strtoul(argv[1], NULL, 16);
        if(argc == 3)
        {
             op = *argv[2];
             if(op == 'y' || op == 'Y')
                 verbose = true;
             else
                 verbose = false;
        }
        
        printf("waiting...\n");
        nai_icb_gen_all_cal(sample, verbose);
        nai_icb_print_cal();
        
        return CMD_RET_SUCCESS;
    }
    else if (argc > 3 && argc < 6)
    {
        chan = simple_strtoul(argv[1], NULL, 10);
        lane = simple_strtoul(argv[2], NULL, 10);
        sample = simple_strtoul(argv[3], NULL, 16);
        if(argc == 5)
        {
             op = *argv[4];
             if(op == 'y' || op == 'Y')
                 verbose = true;
             else
                 verbose = false;
        }

        nai_icb_gen_cal(chan, lane, sample, verbose);

        return CMD_RET_SUCCESS;
    }
    else
        return CMD_RET_USAGE;
}


U_BOOT_CMD(
    icbcal, 5, 0, do_icb,
    "ICB Calibration",
    "dump - Dump calibration data \n"
    "icbcal [sample] <verbose> - Run a complete calibration cycle \n"
    "    sample     : Sample count in HEX; from 0x1 ~ 0xFFFFFF\n"
    "    verbose    : Verbose mode in CHAR; [Y|N]; Default = N \n"
    "icbcal [chan] [lane] [sample] <verbose> - Run calibration on a specific channel and lane\n"
    "    chan       : channel in DEC from 0 ~ 1; Default = 0 \n"
    "    lane       : lane in DEC from 0 ~ 8; Default = 0 \n"
    "    sample     : Sample count in HEX; from 0x1 ~ 0xFFFFFF\n"
    "    verbose    : Verbose mode in CHAR;[Y|N]; Default = N \n"
);

