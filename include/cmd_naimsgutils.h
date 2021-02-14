#ifndef __NAIMSGUTILS_H__
#define __NAIMSGUTILS_H__

#include "NAISerdes.h"

#ifndef __UBOOT
#include <stdlib.h>
#endif

/* MessageUtils */
uint32_t convert_bytes_to_words(uint32_t ulBytes);
int32_t do_naisleep(uint32_t unSeconds);
uint32_t nai_get_timer(uint32_t ulTimer);

/* MessageProcessing */
#ifdef __UBOOT
void nai_reset_module(void);
#endif
int32_t nai_perform_init_slot_addressing(void);
int32_t nai_get_module_id_and_address(uint32_t unAddress, uint8_t *pucModuleID, uint32_t *punModuleAddress);
void nai_write16(uint32_t unAddress, uint16_t usValue);
uint16_t nai_read16(uint32_t unAddress);
void nai_write32(uint32_t unAddress, uint32_t unValue);
uint32_t nai_read32(uint32_t unAddress);

#if defined(__UBOOT) || defined(__BAREMETAL)
void nai_common_write32(uint32_t unAddress, uint32_t unValue);
uint32_t nai_common_read32(uint32_t unAddress);
#endif

#if defined(__UBOOT) || defined(__BAREMETAL)
void nai_dt2_write32(uint32_t unAddress, uint32_t unValue);
uint32_t nai_dt2_read32(uint32_t unAddress);
void nai_dt2_cal_write32(uint32_t unAddress, uint32_t unValue);
uint32_t nai_dt2_cal_read32(uint32_t unAddress);
#endif

uint32_t nai_get_tx_fifo_address(uint8_t ucRequesterID, uint8_t ucCompleterID);
uint32_t nai_get_tx_fifo_pkt_ready_address(uint8_t ucRequesterID, uint8_t ucCompleterID);
uint32_t nai_get_rx_fifo_address(uint8_t ucCompleterID);
uint32_t nai_get_rx_fifo_num_words_address(uint8_t ucRequesterID, uint8_t ucCompleterID);
BOOL nai_tx_fifo_empty(uint8_t ucRequesterID, uint8_t ucCompleterID);
BOOL nai_rx_fifo_empty(uint8_t ucRequesterID, uint8_t ucCompleterID);
BOOL nai_rx_fifo_pkt_ready(uint8_t ucRequesterID, uint8_t ucCompleterID);
void nai_rx_fifo_clear_pkt_ready(uint8_t ucRequesterID, uint8_t ucCompleterID);

#ifdef __UBOOT
int32_t probe_qspi(void);
int32_t erase_qspi(uint32_t ulOffset, uint32_t ulLen);
int32_t write_to_qspi( uint32_t ulAddr, uint32_t ulOffset, uint32_t ulLen);
int32_t read_from_qspi( uint32_t ulAddr, uint32_t ulOffset, uint32_t ulLen);
#endif

#if defined(__UBOOT) || defined(__BAREMETAL)
unsigned int getNaiAddr(void);
int32_t write_at24c(uint8_t chip, uint32_t addr, uint8_t *buf, int32_t len);
#endif

#ifndef __UBOOT
uint32_t crc32(uint32_t crc, const void *buf, size_t size);
#endif

/* HighLevelAPI */
void nai_assign_hard_coded_module_slot(uint8_t ucSlotID);
int32_t nai_retreive_module_slots_status_request(uint16_t *pusSlotIDs);
int32_t nai_init_as_slot(uint8_t ucSlotID);
uint16_t nai_get_max_slot_count(void);

#endif /* __NAIMSGUTILS_H__ */

