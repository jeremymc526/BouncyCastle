#ifndef __NAICONFIGMSGUTILS_H__
#define __NAICONFIGMSGUTILS_H__

#include "NAIComms.h"

#ifndef __UBOOT
#include <stdlib.h>
#endif

#define _MICROCONTROLLER 1

/* MessageCreation */
MsgPacket * create_nai_msg_packet(uint16_t* pusData);
MsgPacket * clone_nai_msg_packet_sans_payload(MsgPacket *ptMsgPacket);
MsgPacketList * create_nai_msg_packet_list(void);
MsgPacketList * find_nai_msg_packet_list(MsgList *ptMsgList, uint16_t usTranID);
void init_nai_msgs(MsgList *ptMsgList);
void create_and_append_nai_msg_packet(MsgPacketList *ptMsgPacketList, uint16_t* pusData);
void delete_nai_msg_packets(MsgPacketList *ptMsgPacketList);
void add_nai_msg(MsgList *ptMsgList, MsgPacketList *ptMsgPacketList);
void delete_nai_msgs(MsgList *ptMsgList);

/* PrintMessage */
void print_nai_msgs(MsgList *ptMsgList, BOOL bPrintPayLd);
void print_nai_msg(MsgPacketList *ptMsgPacketList, BOOL bPrintPayLd);
void print_nai_msg_payload(MsgPacket *ptMsgPacket);

/* MessageValidation */
uint32_t compute_nai_msg_crc(MsgPacketList *ptMsgPacketList);
uint32_t compute_crc_and_update_packet_list(MsgPacketList *ptMsgPacketList);
int32_t validate_nai_msgs(MsgList *ptMsgList);
int32_t validate_nai_msg(MsgPacketList *ptMsgPacketList);

/* MessageProcessing */
BOOL nai_msg_requires_finished_response(MsgPacketList *ptMsgPacketList);
int32_t nai_send_msg_finished_response(MsgPacketList *ptMsgPacketList, int32_t nExecutionStatus);
int32_t deal_with_nai_msg(MsgPacketList *ptMsgPacketList);
int32_t nai_send_msgs(MsgList *ptMsgList);
int32_t nai_send_msg(MsgPacketList *ptMsgPackets);
int32_t nai_receive_msg_packet(uint8_t ucRequesterID, uint8_t ucCompleterID, MsgList *ptMsgList);
uint8_t nai_get_serdes_completer_id(MsgPacket *ptMsgPacket);
uint8_t nai_get_serdes_requester_id(MsgPacket *ptMsgPacket);
uint16_t calculate_nai_expected_sequence_count(uint32_t ulPayloadWordLength);

#ifdef __UBOOT
/* FlashUtils */
int32_t nai_erase_flash(MsgPacketList *ptMsgPacketList);
int32_t nai_write_to_flash(MsgPacketList *ptMsgPacketList);
int32_t nai_read_from_flash(MsgPacketList *ptMsgPacketList);
#endif

#if defined(__UBOOT) || defined(__BAREMETAL)
/* EEPROMUtils */
int32_t nai_copy_eeprom_to_common(uint16_t usChipID);
int32_t nai_write_to_eeprom(MsgPacketList *ptMsgPacketList);
int32_t nai_read_from_eeprom(MsgPacketList *ptMsgPacketList);
#endif

/* HighLevelAPI */
#if defined(__UBOOT) || defined(__LINUX)
int32_t nai_read_module_eeprom_request(uint16_t usChipID, uint8_t ucRequesterID, uint8_t ucCompleterID, uint32_t unEepromOffset, uint8_t *pucBuf, int32_t nLen);
int32_t nai_write_module_eeprom_request(uint16_t usChipID, uint8_t ucRequesterID, uint8_t ucCompleterID, uint32_t unEepromOffset, uint8_t *pucBuf, int32_t nLen);
int32_t nai_erase_flash_request(uint8_t ucRequesterID, uint8_t ucCompleterID, uint32_t unFlashOffset, uint8_t ucNumPages);
int32_t nai_read_module_flash_request(uint8_t ucRequesterID, uint8_t ucCompleterID, uint32_t unFlashOffset, uint8_t *pucBuf, int32_t nLen);
int32_t nai_write_module_flash_request(uint8_t ucRequesterID, uint8_t ucCompleterID, uint32_t unFlashOffset, uint8_t *pucBuf, int32_t nLen);
int32_t nai_exit_module_config_mode_request(uint8_t ucSlotID);
int32_t nai_reset_module_request(uint8_t ucSlotID);

#ifdef _MICROCONTROLLER
/* ST Microcontroller */
int32_t nai_write_micro_request(uint8_t ucRequesterID, uint8_t ucCompleterID, uint8_t ucChannel, uint32_t unOffset, uint8_t *pucBuf, int32_t nLen);
int32_t nai_get_micro_request(uint8_t ucRequesterID, uint8_t ucCompleterID, uint8_t ucChannel, uint8_t *pucBuf, int32_t nLen);
int32_t nai_erase_micro_request(uint8_t ucRequesterID, uint8_t ucCompleterID, uint8_t ucChannel);
#endif /*_MICROCONTROLLER*/
#endif /*__UBOOT || __LINUX*/

int32_t nai_init_all_module_slots(uint16_t usMaxSlotCount);
int32_t nai_assign_module_slot_number_request(uint8_t ucSlotID);
int32_t nai_retrieve_slot(MsgPacketList *ptMsgPacketList);
int32_t nai_retrieve_module_slot_number_request(uint8_t *pucSlotID);

#endif /* __NAICONFIGMSGUTILS_H__ */
