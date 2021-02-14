/*
 * Command to get NAI MB and Module information.
 *
 * Copyright (C) 2015 North Atlantic Industries, Inc.
 */

#include <common.h>
#include <command.h>
#include <malloc.h>
#include <nai_mb_fpga_address.h>
#include <nai_common.h>
#include <nai_mb.h>
#include <nai_module_ext.h>
#include <nai_pci.h>

#ifdef CONFIG_NAI_MB
static void _show_mb_info(Nai_mb mb);
#endif

#ifdef CONFIG_NAI_MODULE_SUPPORT
static int do_mod_rdy_chk(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]){
    /* This is hss module recovery check function.
     * In this function if a hss module is not ready 
     * then it will perform a reset on that module.
     * This is only a temporary workaround, this will
     * add addition delay to system bootup time
    */
    nai_chk_hss_mod_rdy_state();
    
    return CMD_RET_SUCCESS;
}    
#endif

static int do_naiinfo(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
#ifdef CONFIG_NAI_MB    
    Nai_mb mb;
#endif    
      
    // Get operation
    if (strcmp(argv[1], "mb") == 0)
    {
#ifdef CONFIG_NAI_MB
        memset(&mb, 0, sizeof(mb));
        nai_get_mb_info(&mb);
        _show_mb_info(mb);
#endif        
    }
    else if (strcmp(argv[1], "mod") == 0)
    {
        nai_print_module_info();
    }
    else
    {
        return CMD_RET_USAGE;
    }
    return CMD_RET_SUCCESS;
}

#ifdef CONFIG_NAI_MB
static void _show_mb_info(Nai_mb mb)
{
    printf("-------- -------- MB Info -------- --------\n");
    printf("MB-Master:\n");
    
    printf("\tFSBL Rev: %05d.%05d\n", 
        mb.masterVer.fsblMajor, mb.masterVer.fsblMinor);

    printf("\tFSBL Build Time: %s\n", mb.masterVer.fsblBuildTimeStr);
    
    printf("\tUBOOT Rev: %05d.%05d\n", 
        mb.masterVer.ubootMajor, mb.masterVer.ubootMinor);

    printf("\tUBOOT Build Time: %s\n", mb.masterVer.ubootBuildTimeStr);
    
    printf("\tFPGA Rev: %05d.%05d \n",(mb.masterVer.fpgaRev >> 16 & 0xFFFF),(mb.masterVer.fpgaRev & 0xFFFF));
    printf("\tFPGA Build Time: %02d/%02d/%04d at %02d:%02d:%02d\n", 
        mb.masterVer.fpgaDecodedBuildTime.month,
        mb.masterVer.fpgaDecodedBuildTime.day,
        mb.masterVer.fpgaDecodedBuildTime.year,
        mb.masterVer.fpgaDecodedBuildTime.hour,
        mb.masterVer.fpgaDecodedBuildTime.minute,
        mb.masterVer.fpgaDecodedBuildTime.second);

#ifdef CONFIG_NAI_ZYNQ_SLAVE        
    printf("MB-Slave:\n");
    
    printf("\tFSBL Rev: %05d.%05d\n", 
        mb.slaveVer.fsblMajor, mb.slaveVer.fsblMinor);

    printf("\tFSBL Build Time: %s\n", mb.slaveVer.fsblBuildTimeStr);

    printf("\tUBOOT Rev: %05d.%05d\n", 
        mb.slaveVer.ubootMajor, mb.slaveVer.ubootMinor);

    printf("\tUBOOT Build Time: %s\n", mb.slaveVer.ubootBuildTimeStr);
    
    printf("\tFW Rev: %05d.%05d\n", 
        mb.slaveVer.fwRevMajor,
        mb.slaveVer.fwRevMinor);
    
    printf("\tFW Build Time: %s\n", mb.slaveVer.fwBuildTimeStr);
    
    printf("\tFPGA Rev: %05d.%05d \n",(mb.slaveVer.fpgaRev >> 16 & 0xFFFF),(mb.slaveVer.fpgaRev & 0xFFFF));

    printf("\tFPGA Build Time: %02d/%02d/%04d at %02d:%02d:%02d\n", 
        mb.slaveVer.fpgaDecodedBuildTime.month,
        mb.slaveVer.fpgaDecodedBuildTime.day,
        mb.slaveVer.fpgaDecodedBuildTime.year,
        mb.slaveVer.fpgaDecodedBuildTime.hour,
        mb.slaveVer.fpgaDecodedBuildTime.minute,
        mb.slaveVer.fpgaDecodedBuildTime.second);
#endif
    
}
#endif

#ifdef CONFIG_NAI_MODULE_SUPPORT
U_BOOT_CMD(
    modrdy, 2, 0, do_mod_rdy_chk,
    "NAI module ready check",
    "modrdy [slot] - Check module ready state\n"
    "    slot   : module slot in DEC; default = check all module slot\n"
);
#endif

U_BOOT_CMD(
    naiinfo, 3, 0, do_naiinfo,
    "NAI Board Info",
    "mb - Get motherboard info\n"
    "naiinfo mod - Get interface module info\n"
);

