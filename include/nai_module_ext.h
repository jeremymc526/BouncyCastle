#ifndef _NAI_MODULE_E_H
#define _NAI_MODULE_E_H

#include <asm/io.h>
#include <nai_common.h>

#define NAI_MODULE_ERROR -1
#define NAI_MODULE_SUCCESS 0

#ifdef CONFIG_NAI_MAX_MODULE_SLOT
#define MAX_MODULE_SLOT CONFIG_NAI_MAX_MODULE_SLOT
#else
#define MAX_MODULE_SLOT 6 //default to max 6 slots
#endif /*CONFIG_NAI_MAX_MODULE_SLOT*/

#define NAI_MODULE_SLOT_1    0x00
#define NAI_MODULE_SLOT_2    0x01
#define NAI_MODULE_SLOT_3    0x02
#define NAI_MODULE_SLOT_4    0x03
#define NAI_MODULE_SLOT_5    0x04
#define NAI_MODULE_SLOT_6    0x05
#define NAI_MODULE_ALL_SLOT    0xFF

/*Total module slot defined in the MB common area*/
typedef enum
{
    MOD_SLOT_1 = 0,
    MOD_SLOT_2,
    MOD_SLOT_3,
    MOD_SLOT_4,
    MOD_SLOT_5,
    MOD_SLOT_6,
    MOD_SLOT_7,
    MOD_SLOT_8,
    NUM_MOD_SLOT
} MOD_SLOT;

/*MB common area module status 0x43C4 03D8*/
typedef volatile struct {
                                  /*0x3D8 ~ 0x3F4: module common status*/
    struct {
        union {
            u32 reg;
            struct {
                u32 detected          : 1;  /*bit 0 1=module detected 0=no module detected */
                u32 power             : 1;  /*bit 1 1=module powered 0=module not powered*/
                u32 _reserved1        : 6;  /*bit 2 ~ 7 reserved*/
                u32 hss_det_done      : 1;  /*bit 8 */
                u32 hss_link_done     : 1;  /*bit 9 */
                u32 hss_fw_ready      : 1;  /*bit 10 */
                u32 _reserved2        :21;  /*bit 11-31 reserved*/
            } bits;
        }status;
    } mod[NUM_MOD_SLOT];

    u32 _reserved1;                   /*0x3F8: reserverd*/
    
    u32 module_addr_rdy;              /*0x3FC: module slot addressing ready*/
                     
    u32 mod_addr[NUM_MOD_SLOT];       /*0x400-0x41C: module address*/
    
    u32 _reserved2[4];                /*0x420-0x42C: reserved*/
    
    u32 mod_size[NUM_MOD_SLOT];       /*0x430-0x44C: module size*/
    
    u32 _reserved3[4];                /*0x450-0x45C: reserved*/
    
    u32 mod_id[NUM_MOD_SLOT];         /*0x460-0x47C: module id\*/
    
} MB_COMMON_MODULE;

extern MB_COMMON_MODULE *pCommonModule;

void nai_init_module(void);
void nai_module_reset(void);
void nai_enable_module(void);
void nai_disable_module(void);
void nai_chk_hss_mod_rdy_state(void);

void nai_print_module_info(void);

#endif
