#ifndef __NAICOMMS_H__
#define __NAICOMMS_H__

#include "NAISerdes.h"

/** \def Start address of where Interface (bottom) module EEPROM data will be placed in the module "Common" area */
#define INTERFACE_MODULE_COMMON_EEPROM_START 0x0F00

/** \def Start address of where Functional (top) module EEPROM data will be placed in the module "Common" area */
#define FUNCTIONAL_MODULE_COMMON_EEPROM_START 0x0E00

/** \def Handshake address in "Common" area where module indicates to MB various states */
#define MODULE_COMMON_HANDSHAKE_ADDR  0x0000025C

/** \def I2C chip ID of the EEPROM found on the Interface (bottom) module */
#define I2C_INTERFACE_MODULE_EEPROM	0x50

/** \def I2C chip ID of the EEPROM found on the Functional (top) module */
#define I2C_FUNCTIONAL_MODULE_EEPROM	0x51

/** \def Total number of words comprising the entire packet header: 6W-Serdes, 8W-Transport, 6W-Command **/
#define CONFIG_TOTAL_PKT_HDR_IN_WORDS	 20

/** \def Total number of words in TRANSPORT header in configuration mode **/
#define CONFIG_TOTAL_TRANSPORT_HDR_IN_WORDS 8 

/** \def Total number of words in COMMAND header in configuration mode **/
#define CONFIG_TOTAL_COMMAND_HDR_IN_WORDS 6

/** \def Maximum number of words that can comprise the payload when in configuration mode **/
#define CONFIG_MAX_PAYLOAD_IN_WORDS    236


/*Command Types (implemented this way since enum will force use of 4 bytes which is a waste of space)*/
/** \def READ EEPROM command : used to read from the EEPROM storage **/
#define	COMMAND_TYPECODE_READEEPROM			0x0001
/** \def WRITE EEPROM command : used to write to the EEPROM storage **/
#define	COMMAND_TYPECODE_WRITEEEPROM		0x0002
/** \def ERASE FLASH command : used to erase desired number of FLASH pages (each page is 65536 bytes) **/
#define COMMAND_TYPECODE_ERASEFLASH			0x0003
/** \def READ FLASH command : used to read from the FLASH storage **/
#define	COMMAND_TYPECODE_READFLASH			0x0004
/** \def WRITE FLASH command : used to write to the FLASH storage **/
#define	COMMAND_TYPECODE_WRITEFLASH			0x0005
/** \def ASSIGN SLOT command : used to notify a particular module what slot it is located in **/
#define COMMAND_TYPECODE_ASSIGNSLOT			0x0006
/** \def RETRIEVE SLOT command : used to retrieve the current slot  **/
#define COMMAND_TYPECODE_RETRIEVESLOT		0x0007
/** \def EXIT CONFIG MODE command : used to force exit of the CONFIG MODE application  **/
#define COMMAND_TYPECODE_EXIT_CONFIG_MODE	0x0008
/** \def RESET command : used to reset this module's CPU  **/
#define COMMAND_TYPECODE_RESET_MODULE		0x0009
/** \def REQUEST FINISHED command : used to send response back to caller when a given request has finished processing **/
#define COMMAND_TYPECODE_REQUEST_FINISHED   0x000A
/** \def CONFIG microcontroller command : used to configure all microcontroller channels on a module **/
#define COMMAND_TYPECODE_CONFIG_MICRO 		0x000B
/** \def GET microcontroller command : used to fetch version of bootloader and supported bootloader commands **/
#define COMMAND_TYPECODE_GET_MICRO    		0x000C
/** \def ERASE microcontroller command : used to erase entire EPROM in microcontroller **/
#define COMMAND_TYPECODE_ERASE_MICRO  		0x000D

/** \def DEBUG command : used to run some debug tests  **/
#define COMMAND_TYPECODE_DEBUG				0x0FFF
/* #pragma pack(push,1) */

//**************************************************************************************************************
/**
\struct TransportHdr
This struct defines the TRANSPORT HEADER layer of a message packet. (size of message in words, sequence number,
CRC etc..)
*/
//**************************************************************************************************************
typedef struct
{
   /** Unique packet identifier : 2 Bytes*/
   uint16_t usID;           			/* 2 Bytes */
   /** Sequence number (messages > CONFIG_MAX_PAYLOAD_IN_WORDS will be broken up until multiple message packets. Sequence num dictates order of packets) : 2 Bytes*/
   uint16_t usSequenceNum;  			/* 2 Bytes */
   /** Total message length including headers : 4 Bytes */
   uint32_t  unMsgLength;	  			/* 4 Bytes - Includes Headers - Total Msg Length in WORDS including Headers*/
   /** Number of BYTES found in payload (no headers) : 2 Bytes */
   uint16_t usPacketPayLdLength; 		/* 2 Bytes - Packet Payload - Number of BYTES found in payload (no headers) */
   /** Expected number of packets to be returned : 2 Bytes */
   uint16_t usExpectedSequenceCount; 	/* 2 Bytes */
   /** CRC value of entire packet : 4 Bytes*/
   uint32_t  unCRC;          			/* 4 Bytes */
} __attribute__ ((packed,aligned(1)))    TransportHdr;

//**************************************************************************************************************
/**
\struct CommandHdr
This struct defines the COMMAND HEADER layer of a message packet. This layer provides high level application
information about the packet such as command type and Offset of where to retrieve or write data.
*/
//**************************************************************************************************************
typedef struct
{
   /** Identifier of Command to be invoked : 2 Bytes */
   uint16_t  usCommandType;  		/* 2 Bytes */
   /** ID of EEPROM to read/write to : 2 Bytes */
   uint16_t  usChipID;		     	/* 2 Bytes */
   /** Offset into either FLASH or EEPROM of where to start writing/reading : 4 Bytes */
   uint32_t   unOffset;  		    /* 4 Bytes */
   /** Number of BYTES being requested in a read operation : 4 Bytes */
   uint32_t   unPayLdRequestLength; /* 4 Bytes Full Message Requested Number of Bytes */
}__attribute__ ((packed,aligned(1)))    CommandHdr;

//**************************************************************************************************************
/**
\struct CommandPayLd
This struct defines the COMMAND PAYLOAD layer of a message packet. This is the storage representation of the 
actual packet payload.
*/
//**************************************************************************************************************
typedef struct
{
   /**Array of payload data of size: CONFIG_MAX_PAYLOAD_IN_WORDS */
   uint16_t usData[CONFIG_MAX_PAYLOAD_IN_WORDS];
} __attribute__ ((packed,aligned(1)))    CommandPayLd;

//**************************************************************************************************************
/**
\struct TransportPayLd
This struct defines the TRANSPORT PAYLOAD layer of a message packet. This storage representation encapsulates 
the COMMAND HEADER (CommmandHdr) and COMMAND PAYLOAD (CommandPayLd).
*/
//**************************************************************************************************************
typedef struct
{
   /** CommandHdr encapsulates application specific information */
   CommandHdr   tCommandHdr;    /*   6 Words */
   /** CommandPayLd holds the actual message payload */
   CommandPayLd tCommandPayLd;  /* 236 Words */
} __attribute__ ((packed,aligned(1)))    TransportPayLd;

//**************************************************************************************************************
/**
\struct SerdesConfigPayLd
This struct defines the SERDES PAYLOAD layer of a message packet. This storage representation encapsulates 
the TRANSPORT HEADER (TransportHdr) and TRANSPORT PAYLOAD (TransportPayLd).
*/
//**************************************************************************************************************
typedef struct
{
   /** TransportHdr encapsulates transport information to be able to break up large messages over mutliple packets */
   TransportHdr   tTransportHdr;    /*   8 Words */
   /** TransportPayLd encapsulates application specific information */
   TransportPayLd tTransportPayLd;  /* 242 Words */
} __attribute__ ((packed,aligned(1)))    SerdesConfigPayLd;


//**************************************************************************************************************
/**
\struct NAIMsg
This struct defines all the information that comprises a NAIMsg. This structure is unioned with an array to
make it easy to read and write to a FIFO. 
*/
//**************************************************************************************************************
typedef union
{
   /** Array representation of the entire NAIMsg */
   uint16_t msg[MAX_SERDES_MSG_IN_WORDS];
   struct
   {
	  /** SerdesHdr encapsulates SERDES specific information */
      SerdesHdr   tSerdesHdr;    /*   6 Words */
	  /** SerdesConfigPayLd encapsulates all of the other layers of the message protocol including the Transport mechanism and Command information */
      SerdesConfigPayLd tSerdesPayLd;  /* 250 Words */
   } __attribute__ ((packed,aligned(1)))   ;
} NAIMsg;

/* Restore the original alignment rules from the internal stack */
/* #pragma pack(pop) */

/***************************************************************************/
/* The following are structs used to easily create Linked linked lists of  */
/* NAI messages.							   							   */
/***************************************************************************/
//**************************************************************************************************************
/**
\struct MsgPacket
This struct defines a single message packet
*/
//**************************************************************************************************************
struct _MsgPacket
{
   /** Actual message content */
   NAIMsg tNAIMsg;
   /** Pointer to the Next message packet */
   struct _MsgPacket *ptNext;
}  __attribute__ ((packed,aligned(1)))  ;

typedef struct _MsgPacket MsgPacket;

//**************************************************************************************************************
/**
\struct MsgPacketList
This struct defines a single entire message (may have 1 or more message packets)
*/
//**************************************************************************************************************
struct _MsgPacketList
{   
   /** Indicates the number of messages stored in this linked list abstraction */
   int32_t nCount;
   /** Indicates how many words are left to read */
   int32_t unWordsLeftToRead; /* Used to know how many words we have left to fulfill the msg - comes into play if packets from multiple messages arrive interleaved */
   /** Pointer to the First message packet */
   MsgPacket *ptStart;
   /** Pointer to the Last message packet */
   MsgPacket *ptEnd;
    /** Pointer to the Next message */
   struct _MsgPacketList *ptNext;
}   __attribute__ ((packed,aligned(1)))  ;

typedef struct _MsgPacketList MsgPacketList;

//**************************************************************************************************************
/**
\struct MsgList
This struct defines the abstraction of an entire list (linked list) of complete messages (messages that may contain
one or more packets).
*/
//**************************************************************************************************************
typedef struct
{
   /** The number of MsgPacketList entries */
   int32_t nCount;
   /** Pointer to the First message */
   MsgPacketList *ptStart;
   /** Pointer to the Last message */
   MsgPacketList *ptEnd;
}   __attribute__ ((packed,aligned(1)))   MsgList;

#endif /* __NAICOMMS_H__ */
