/*
 * Master/slave communication definitions.
 *
 * Copyright (C) 2014-2015 North Atlantic Industries, Inc.
 */

#ifndef __MASTER_SLAVE_H__
#define __MASTER_SLAVE_H__

/*
 * HDLC protocol definitions
 */
#define FRAME_BUFFER_SIZE       0x1000
#define HDLC_FRAME_DELIMITER    0x7E
#define HDLC_CONTROL_ESCAPE     0x7D
#define HDLC_ESCAPE_BIT         (1 << 5)

#define MAX_RW_COUNT            256 // Max 256 values per command

#define ADDR_SIZE               4   // 4 bytes for address
#define WIDTH_SIZE              1   // 1 byte for width
#define COUNT_SIZE              2   // 2 bytes for count
#define MIN_PAYLOAD_LEN         1   // 1 byte (echo command)
#define MAX_PAYLOAD_LEN         (ADDR_SIZE + WIDTH_SIZE + COUNT_SIZE + (MAX_RW_COUNT * sizeof(u32)))

#define CMD_SIZE                1   // 1 byte for command
#define LEN_SIZE                2   // 2 bytes for length
#define MIN_CMD_LEN             (CMD_SIZE + LEN_SIZE + MIN_PAYLOAD_LEN)
#define MAX_CMD_LEN             (CMD_SIZE + LEN_SIZE + MAX_PAYLOAD_LEN)

#define SOF_SIZE                1   // 1 byte for SOF
#define EOF_SIZE                1   // 1 byte for EOF
#define CRC_SIZE                2   // 2 bytes for CRC
#define MIN_FRAME_LEN           (SOF_SIZE + MIN_CMD_LEN + CRC_SIZE + EOF_SIZE)
#define MAX_FRAME_LEN           (SOF_SIZE + (2 * (MAX_CMD_LEN + CRC_SIZE)) + EOF_SIZE)

/*
 * Command codes
 */
#define CMD_ECHO                0x01    // Echo request
#define CMD_READ                0x02    // Read request
#define CMD_WRITE               0x03    // Write request
#define CMD_SETCLR              0x04    // Set/clear request
#define CMD_READ_RESP           0x05    // Read response
#define CMD_ACK                 0x06    // ACK/NACK

/*
 * Error codes
 */
#define E_OK                    0x00    // All good
#define E_BAD_CMD_LEN           0x01    // Invalid command length
#define E_BAD_CMD               0x02    // Invalid command
#define E_BAD_PAYLOAD_LEN       0x03    // Invalid payload length
#define E_BAD_ADDR              0x04    // Invalid address
#define E_NOT_ALIGNED           0x05    // Address not aligned
#define E_BAD_WIDTH             0x06    // Invalid width
#define E_BAD_COUNT             0x07    // Invalid count
#define E_FLASH_READ_ERR        0x08    // Flash read failed
#define E_FLASH_ERASE_ERR       0x09    // Flash erase failed
#define E_FLASH_WRITE_ERR       0x0A    // Flash write failed

/*
 * Command layout
 */
#pragma pack(1)
typedef struct {
    u8  cmd;                                /* Command */
    u16 len;                                /* Payload length */
    union {
        u8  payload[MAX_PAYLOAD_LEN];       /* Payload */

        /* Read request */
        struct {
            u32 addr;                       /* Address */
            u8  width;                      /* Width */
            u16 count;                      /* Count */
        } read;

        /* Write request / Read response */
        struct {
            u32 addr;                       /* Address */
            u8  width;                      /* Width */
            u16 count;                      /* Count */
            union {
                u8  val8[MAX_RW_COUNT];     /* 8-bit values */
                u16 val16[MAX_RW_COUNT];    /* 16-bit values */
                u32 val32[MAX_RW_COUNT];    /* 32-bit values */
            };
        } write, read_resp;

        /* Set/clear request */
        struct {
            u32 addr;                       /* Address */
            u8  width;                      /* Width */
            union {
                struct {
                    u8  mask;               /* 8-bit mask */
                    u8  val;                /* 8-bit value */
                } u8;
                struct {
                    u16 mask;               /* 16-bit mask */
                    u16 val;                /* 16-bit value */
                } u16;
                struct {
                    u32 mask;               /* 32-bit mask */
                    u32 val;                /* 32-bit value */
                } u32;
            };
        } setclr;

        /* ACK/NACK */
        struct {
            u8  req;                        /* Request to ACK/NACK */
            u8  err;                        /* Error code */
        } ack;
    };
} CMD;
#pragma pack()

/*
 * Master UART definitions
 */
#define UART_FIFO_TIMEOUT       10  // 10 ms (it takes < 1 ms to send 64 bytes at MAX baud rate)

#ifndef TRUE
    #define TRUE		1
#endif
#ifndef FALSE
    #define FALSE		0
#endif

#define Xil_AssertNonvoid(x) \
	({ if (!(x)) \
		__assert_fail(#x, __FILE__, __LINE__, __func__); })

/*
 * Function prototypes
 */
u32 slave_echo(u8 *buf, u32 len, u8 *outbuf, bool verbose);
u32 slave_read(u32 addr, u32 width, u32 count, u32 *buf, bool verbose);
u32 slave_write(u32 addr, u32 width, u32 count, u32 *buf, bool verbose);
u32 slave_setclr(u32 addr, u32 width, u32 mask, u32 val, bool verbose);
u32 slave_rx(u8 *buf, bool verbose);
u32 slave_tx(u8 *buf, u32 len, bool verbose);
void uart1_init(void);
void uart1_loopback(bool en);

#endif /* __MASTER_SLAVE_H__ */

