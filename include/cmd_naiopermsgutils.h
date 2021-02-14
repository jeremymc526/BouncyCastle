#ifndef __NAIOPERMSGUTILS_H__
#define __NAIOPERMSGUTILS_H__

#include "NAISerdes.h"

#ifndef __UBOOT
#include <stdlib.h>
#endif

/* Operational Message Utils */
int32_t nai_init_msg_utils(uint8_t ucID);
int32_t nai_init_slot_addressing(void);

/* Addressing Scheme */
int32_t nai_read_reg16_request(uint32_t unAddress, uint16_t *pusValue);
int32_t nai_write_reg16_request(uint32_t unAddress, uint16_t usValue);

int32_t nai_read_reg32_request(uint32_t unAddress, uint32_t *punValue);
int32_t nai_write_reg32_request(uint32_t unAddress, uint32_t unValue);

int32_t nai_read_block16_request(uint32_t unStartAddress, uint32_t usCount, uint8_t ucStride, uint16_t *pusDataBuf);
int32_t nai_write_block16_request(uint32_t unStartAddress, uint32_t usCount, uint8_t ucStride, uint16_t *pusDataBuf);

int32_t nai_read_block32_request(uint32_t unStartAddress, uint32_t usCount, uint8_t ucStride, uint32_t *punDataBuf);
int32_t nai_write_block32_request(uint32_t unStartAddress, uint32_t usCount, uint8_t ucStride, uint32_t *punDataBuf);

/* Module Slot Scheme */
int32_t nai_read_reg16_by_slot_request(uint8_t ucSlotID, uint32_t unModuleOffset, uint16_t *pusValue);
int32_t nai_write_reg16_by_slot_request(uint8_t ucSlotID, uint32_t unModuleOffset, uint16_t usValue);

int32_t nai_read_reg32_by_slot_request(uint8_t ucSlotID, uint32_t unModuleOffset, uint32_t *punValue);
int32_t nai_write_reg32_by_slot_request(uint8_t ucSlotID, uint32_t unModuleOffset, uint32_t unValue);

int32_t nai_read_block16_by_slot_request(uint8_t ucSlotID, uint32_t unModuleOffsetStart, uint32_t usCount, uint8_t ucStride, uint16_t *pusDataBuf);
int32_t nai_write_block16_by_slot_request(uint8_t ucSlotID, uint32_t unModuleOffsetStart, uint32_t usCount, uint8_t ucStride, uint16_t *pusDataBuf);

int32_t nai_read_block32_by_slot_request(uint8_t ucSlotID, uint32_t unModuleOffsetStart, uint32_t usCount, uint8_t ucStride, uint32_t *punDataBuf);
int32_t nai_write_block32_by_slot_request(uint8_t ucSlotID, uint32_t unModuleOffsetStart, uint32_t usCount, uint8_t ucStride, uint32_t *punDataBuf);

#endif /* __NAIOPERMSGUTILS_H__ */

