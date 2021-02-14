#ifndef __NAISERDES_H__
#define __NAISERDES_H__

/* #define _VERBOSE 1 */ /*WARNING - USING THIS HAS BEEN KNOWN TO ADVERSLY AFFECT SERDES PROCESSING AT TIMES - PLEASE ATTEMPT WITHOUT VERBOSE TO SEE IF PROCESSING COMPLETES AS EXPECTED */
/* #define _DEBUG_X 1 */

/*#define __BAREMETAL 	1 */
/*#define __LINUX 		1 */
/*#define __CYGWIN 		1 */
 #define __UBOOT 		1 
/*#define __VXWORKS		1 */

#ifdef __LINUX
	#define _USE_MUTEX 1
#endif

#define _QSPI_SUPPORT 1
//#define _PERFORM_QSPI_ERASE_ON_FLASH_WRITE 1
#define _IGNORE_MISSING_TOP_MODULE 1
#define COMPLETION_TIMEOUT 5000  /*In milliseconds*/

/*#define __EXTERNAL_PROCESSOR 1*/ /*Define __EXTERNAL_PROCESSOR if building for a processor other than the processor for the HPS. */

#ifdef __BAREMETAL
	#include <hwlib.h>
#elif __UBOOT 
	#include <compiler.h>
#elif __LINUX
	#include <sys/types.h>
	#include <hwlib.h>
#elif __VXWORKS
	#include <types/vxTypes.h>
#endif

#ifndef BOOL
	/* typedef enum {false=0, true=1} bool; */
	typedef int BOOL;
#endif

#ifndef TRUE
	#define TRUE 	1
#endif

#ifndef FALSE
	#define FALSE 	0
#endif

#ifndef MIN
	#define MIN(a,b) (((a)<(b))?(a):(b))
#endif

#ifndef MAX
	#define MAX(a,b) (((a)>(b))?(a):(b))
#endif

/** \def Address of HPS Handshake area **/
//#define HPS_OP_HANDSHAKE_AREA_ADDRESS    0xFF230000

/** \def Address of HPS Page Register **/
#define HPS_PAGE_ADDRESS 0xFF210000

/** \def Motherboard Module start address */
#define COMMON_MBINFO_READY_ADDR			0x000003FC
#define COMMON_MBINFO_MODULESTART_ADDR      0x00000400

/** \def Total number of words in SERDES header **/
#define TOTAL_SERDES_READWRITEREQ_HDR_IN_WORDS 6

/** \def Maximum number of words that can comprise the payload when in operational mode **/
#define OPER_MAX_PAYLOAD_IN_WORDS	250 

/** \def Maximum number of words in the SERDES message **/
#define MAX_SERDES_MSG_IN_WORDS 	256

/*SERDES Command Types (implemented this way since enum will force use of 4 bytes which is a waste of space)*/
#define SERDES_READREG  0x0002
#define SERDES_WRITEREG 0x0003

/*SERDES Byte Enable*/
#define SERDES_32BITDATA 0x0F

/*Error Codes*/
/** \def NAI_SUCCESS : indicates no errrors  **/
#define NAI_SUCCESS 				 		 0
#define NAI_ERROR_WRONG_SLOT_NUM 			-8201
#define NAI_INVALID_SLOT_ID					-8202
#define NAI_SERDES_UNEXPECTED_PAYLOAD_COUNT -8203
#define NAI_MODULE_NOT_FOUND				-8204
#define NAI_MIS_ALIGNED_BYTE_ENABLE         -8205
#define NAI_INVALID_PARAMETER_VALUE			-8206
#define NAI_SYSTEM_NOT_READY				-8207
#define NAI_MODULE_NOT_READY				-8208
#define NAI_UNABLE_TO_ALLOCATE_MEMORY		-8209
#define NAI_COMMAND_NOT_RECOGNIZED		   	-8210
#define NAI_TX_FIFO_NOT_EMPTY_TIMEOUT	   	-8211
#define NAI_RX_FIFO_PKT_NOT_READY_TIMEOUT  	-8212
#define NAI_DETECT_MODULES_TIMEOUT		   	-8213
#define NAI_I2C_DEVICE_NOT_FOUND		   	-8214
#define NAI_UNABLE_TO_LOCK_MUTEX   	   	   	-8215
#define NAI_UNABLE_TO_UNLOCK_MUTEX		   	-8216
#define NAI_STRIDE_CAUSES_MISALIGNMENT		-8217
#define NAI_USER_COPY_FAILED				-8218
#define NAI_MODULE_DETECT_READY_TIMEOUT		-8219
#define NAI_MODULE_LINK_DETECT_TIMEOUT		-8220
#define NAI_ENTER_CONFIG_MODE_TIMEOUT       -8221
#define NAI_STM_TX_TIMEOUT					-8222
#define NAI_STM_RX_TIMEOUT					-8223
#define NAI_ACK_NOT_RECEIVED				-8224
#define NAI_POTENTIAL_BUFFER_OVERRUN		-8225
#define NAI_CPLD_PROGRAMMING_ERROR			-8226
#define NAI_INVALID_PAYLOAD_LENGTH			-8227
#define NAI_INVALID_PACKET_TYPE				-8228
#define NAI_RESPONSE_COMMAND_MISMATCH		-8229
#define NAI_COMMAND_FAILED				    -8230
#define NAI_NOT_SUPPORTED					-8231

/*Misc*/
#define MB_SLOT								0x00
#define MODULE_1_SLOT						0x01
#define MODULE_2_SLOT						0x02
#define MODULE_3_SLOT						0x03
#define MODULE_4_SLOT						0x04
#define MODULE_5_SLOT						0x05
#define MODULE_6_SLOT						0x06
#define PPC_MB_SLOT							0x0A
#define ASSIGNED_SLOT_ID					0xFE
#define INVALID_SLOT_ID						0xFF

/**************************************************************************************************************/
/**
\struct SerdesHdr
This struct defines the SERDES HEADER layer of a message packet. 
*/
/**************************************************************************************************************/
#ifdef __VXWORKS 
/*BIG ENDIAN*/
typedef struct
{
   /** - Bits  0-3 : Type 
	   - Bit     4 : Credit Limit
	   - Bit     5 : SPARE
       - Bit     6 : SeqNumTx 
       - Bit     7 : SeqNumRx 
       - Bits 8-15 : Byte Enable */
   union
   {
   		uint16_t usSERDES0;	/* 2 Bytes */ 
		struct
		{
			uint8_t ucDataMode		: 1;
			uint8_t ucSpare3		: 1;
			uint8_t ucSpare2		: 1;
			uint8_t ucSpare1		: 1;														
			uint8_t ucByteEnable 	: 4;
			uint8_t ucSeqNumRx 		: 1;
			uint8_t ucSeqNumTx 		: 1;
			uint8_t ucToHPS 		: 1;
			uint8_t ucCreditLimit 	: 1;
			uint8_t ucType 			: 4;						
		} __attribute__ ((packed,aligned(1)))   ;
   };

   /** - Bits   0-7  : Payload Max Length
	   - Bits   8-11 : Requester ID
	   - Bits  12-15 : Completer ID */
   union
   {
		uint16_t usSERDES1;    /* 2 Bytes */
		struct
		{			
			uint8_t ucCompleterID 	: 4;
			uint8_t ucRequesterID 	: 4;			
			uint8_t ucPayloadLength	: 8;
		} __attribute__ ((packed,aligned(1)))   ;
   };
   

   /** - Bits  0-1 : Reserved 
	   - Bits 2-15 : Address Lo*/       
   union
   {
   		uint16_t usSERDES2;	/* 2 Bytes */		
		uint16_t usAddressLo;
   };

   /** - Bits 0-15 : Address Hi */
   union
   {
   		uint16_t usSERDES3;	/* 2 Bytes */
		uint16_t usAddressHi;
   };

   /** - Bits  0-7  : Block Address Type 
	   - Bits  8-15 : Spare */
   union
   {
   		uint16_t usSERDES4;	/* 2 Bytes */
		struct
		{
			uint8_t usSPARE1			: 8;
			uint8_t ucBlockAddrIncrVal 	: 8;
		}__attribute__ ((packed,aligned(1)))   ;
   };

   /** - Bits  0-15 : Spare */
   union
   {
   		uint16_t usSERDES5;	/* 2 Bytes */
		uint16_t usSPARE2; 
   };
} __attribute__ ((packed,aligned(1)))    SerdesHdr;


//**************************************************************************************************************
/**
\struct SerdesOperPayLd
This struct defines the SERDES PAYLOAD layer of a message packet.
*/
//**************************************************************************************************************
typedef struct
{
   /**Array of payload data of size: OPER_MAX_PAYLOAD_IN_WORDS */
   uint16_t usData[OPER_MAX_PAYLOAD_IN_WORDS];
}__attribute__ ((packed,aligned(1)))    SerdesOperPayLd;


//**************************************************************************************************************
/**
\struct NAIOperMsg
This struct defines all the information that comprises a NAIOperMsg. This structure is unioned with an array to
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
	  /** SerdesOperPayLd reflects the actual payload to be transmitted or received. */
      SerdesOperPayLd tSerdesPayLd;  /* 250 Words */
   } __attribute__ ((packed,aligned(1)))   ;
}NAIOperMsg;

//**************************************************************************************************************
/**
\struct FIFOValue
This struct defines a single FIFO value which is made up of 2 words. This struct helps take 2 words and easily
write a single long to the FIFO or read back a LONG and easily parse it to its 2 word counterparts.
*/
//**************************************************************************************************************
typedef union
{
   /** Int (32 bit) value which is comprised of the union'ed Lo Word and Hi Word */
   uint32_t unValue;
   struct
   {
	  /** Hi Word : upper 16 bits  */
	  uint16_t usHiWord;	   
	  /** Lo Word : lower 16 bits */
	  uint16_t usLoWord;	   
   }__attribute__ ((packed,aligned(1)))   ;
} FIFOValue;

//**************************************************************************************************************
/**
\struct WORDValue
This struct defines a single WORD (16 bit) value which is made up of 2 Bytes. 
*/
//**************************************************************************************************************
typedef union
{
	/** Word (uint16_t) 16 bit value which is comprised of the union'ed Lo Byte and Hi Byte */
	uint16_t usValue;
	struct
	{
		/** Hi Byte : upper 8 bits */
		uint8_t ucHiByte;		
	    /** Lo Byte : lower 8 bits */
		uint8_t ucLoByte;		
	}__attribute__ ((packed,aligned(1)))   ;
} WORDValue;
#define MSB(x)	(((x) >> 8) & 0xff)	  /* most signif byte of 2-byte integer */
#define LSB(x)	((x) & 0xff)		  /* least signif byte of 2-byte integer*/
#define MSW(x) (((x) >> 16) & 0xffff) /* most signif word of 2-word integer */
#define LSW(x) ((x) & 0xffff) 		  /* least signif byte of 2-word integer*/

/* swap the MSW with the LSW of a 32 bit integer */
#define NAI_WORDSWAP(x) (MSW(x) | (LSW(x) << 16))

#define LLSB(x)	((x) & 0xff)		/* 32bit word byte/word swap macros */
#define LNLSB(x) (((x) >> 8) & 0xff)
#define LNMSB(x) (((x) >> 16) & 0xff)
#define LMSB(x)	 (((x) >> 24) & 0xff)
#define NAI_LONGSWAP(x) ((LLSB(x) << 24) | \
		     (LNLSB(x) << 16)| \
		     (LNMSB(x) << 8) | \
		     (LMSB(x)))
#else /*Little Endian*/

typedef struct
{
   /** - Bits  0-3 : Type 
	   - Bit     4 : Credit Limit
	   - Bit     5 : SPARE
       - Bit     6 : SeqNumTx 
       - Bit     7 : SeqNumRx 
       - Bits 8-15 : Byte Enable */
   union
   {
   		uint16_t usSERDES0;	/* 2 Bytes */ 
		struct
		{
			uint8_t ucType 			: 4;
			uint8_t ucCreditLimit 	: 1;
			uint8_t ucToHPS 		: 1;
			uint8_t ucSeqNumTx 		: 1;
			uint8_t ucSeqNumRx 		: 1;
			uint8_t ucByteEnable 	: 4;
			uint8_t ucSpare1		: 1;
			uint8_t ucSpare2		: 1;
			uint8_t ucSpare3		: 1;
			uint8_t ucDataMode		: 1;
		} __attribute__ ((packed,aligned(1)))   ;
   };

   /** - Bits    0-7 : Payload Max Length
	   - Bits   8-11 : Requester ID
	   - Bits  12-15 : Completer ID 	   
	   */
   union
   {
		uint16_t usSERDES1;    /* 2 Bytes */
		struct
		{
			uint8_t ucPayloadLength  : 8;
			uint8_t ucRequesterID 	 : 4;
			uint8_t ucCompleterID 	 : 4;			
		} __attribute__ ((packed,aligned(1)))   ;
   };
   

   /** - Bits  0-1 : Reserved 
	   - Bits 2-15 : Address Lo*/       
   union
   {
   		uint16_t usSERDES2;	/* 2 Bytes */
		uint16_t usAddressLo;
   };

   /** - Bits 0-15 : Address Hi */
   union
   {
   		uint16_t usSERDES3;	/* 2 Bytes */
		uint16_t usAddressHi;
   };

   /** - Bits   0-7 : Block Address Increment Value
	   - Bits  8-15 : Spare */
   union
   {
   		uint16_t usSERDES4;	/* 2 Bytes */
		struct
		{
			uint8_t ucBlockAddrIncrVal : 8;
			uint8_t usSPARE1		   : 8;
		}__attribute__ ((packed,aligned(1)))   ;
   };

   /** - Bits  0-15 : Spare */
   union
   {
   		uint16_t usSERDES5;	/* 2 Bytes */
		uint16_t usSPARE2; 
   };
} __attribute__ ((packed,aligned(1)))    SerdesHdr;


//**************************************************************************************************************
/**
\struct SerdesOperPayLd
This struct defines the SERDES PAYLOAD layer of a message packet.
*/
//**************************************************************************************************************
typedef struct
{
   /**Array of payload data of size: OPER_MAX_PAYLOAD_IN_WORDS */
   uint16_t usData[OPER_MAX_PAYLOAD_IN_WORDS];
}__attribute__ ((packed,aligned(1)))    SerdesOperPayLd;


//**************************************************************************************************************
/**
\struct NAIOperMsg
This struct defines all the information that comprises a NAIOperMsg. This structure is unioned with an array to
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
	  /** SerdesOperPayLd reflects the actual payload to be transmitted or received. */
      SerdesOperPayLd tSerdesPayLd;  /* 250 Words */
   }__attribute__ ((packed,aligned(1)))   ;
} NAIOperMsg;

//**************************************************************************************************************
/**
\struct FIFOValue
This struct defines a single FIFO value which is made up of 2 words. This struct helps take 2 words and easily
write a single long to the FIFO or read back a LONG and easily parse it to its 2 word counterparts.
*/
//**************************************************************************************************************
typedef union
{
   /** Int (32 bit) value which is comprised of the union'ed Lo Word and Hi Word */
   uint32_t unValue;
   struct
   {
	  /** Lo Word : lower 16 bits */
      uint32_t usLoWord : 16;
      /** Hi Word : upper 16 bits  */
      uint32_t usHiWord : 16;
   }__attribute__ ((packed,aligned(1)))   ;
} FIFOValue;

//**************************************************************************************************************
/**
\struct WORDValue
This struct defines a single WORD (16 bit) value which is made up of 2 Bytes. 
*/
//**************************************************************************************************************
typedef union
{
	/** Word (uint16_t) 16 bit value which is comprised of the union'ed Lo Byte and Hi Byte */
	uint16_t usValue;
	struct
	{
	    /** Lo Byte : lower 8 bits */
		uint8_t ucLoByte : 8;
		/** Hi Byte : upper 8 bits */
		uint8_t ucHiByte : 8;
	}__attribute__ ((packed,aligned(1)))   ;
} WORDValue;

#endif

#endif /* __NAISERDES_H__ */

