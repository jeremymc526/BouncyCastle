/*
 * Master/slave communication functions.
 *
 * Copyright (C) 2014-2015 North Atlantic Industries, Inc.
 */
 
#include <common.h>
#include <malloc.h>
#include <dm/uclass.h>
#include <dm/device.h>
#include <serial.h>
#include <watchdog.h>
#include <master_slave.h>

/* Local functions */
static u32 send_req(CMD *req, u32 len, bool verbose);
static u32 get_resp(CMD *resp, bool verbose);
static void dump_cmd(CMD *resp, u32 len);
static u32 uart1_rx(u8 *buf);
static u32 uart1_tx(u8 *buf, u32 len);
static u16 crc_ccitt(u8 *buf, u32 len);
static u32 stuff_bytes(u8 *inbuf, u32 inlen, u8 *outbuf);
static u32 encode_frame(u8 *inbuf, u32 inlen, u8 *outbuf, bool verbose);
static u32 decode_frame(u8 *inbuf, u32 inlen, u8 *outbuf, bool verbose);

/* Local variables */
static struct udevice *uart1_dev = NULL;
static struct dm_serial_ops *uart1_ops = NULL;
static int uart1_initialized = 0;

u32 slave_echo(u8 *buf, u32 len, u8 *outbuf, bool verbose)
{
    CMD *req, *resp;
    u32 txlen, rxlen;

    /* Allocate memory for request buffer */
    req = (CMD *)malloc(sizeof(CMD));
    if (!req)
    {
        printf("%s: no memory!\n", __FUNCTION__);
        return 0;
    }

    /* Allocate memory for response buffer */
    resp = (CMD *)malloc(sizeof(CMD));
    if (!resp)
    {
        printf("%s: no memory!\n", __FUNCTION__);
        free(req);
        return 0;
    }

    /* Prepare request command */
    req->cmd = CMD_ECHO;
    req->len = len;
    memcpy(req->payload, buf, len);
    txlen = CMD_SIZE + LEN_SIZE + len;

    /* Send request to slave */
    send_req(req, txlen, verbose);

    /* Wait for response from slave */
    rxlen = get_resp(resp, verbose);

    /* Valid response? */
    if (rxlen && resp->cmd == CMD_ECHO && resp->len == len)
    {
        if (outbuf)
            memcpy(outbuf, resp, len);
    }
    else
    {
        len = 0;
    }

    /* Free memory */
    free(req);
    free(resp);

    return len;
}


u32 slave_read(u32 addr, u32 width, u32 count, u32 *buf, bool verbose)
{
    CMD *req, *resp;
    u32 txlen, rxlen;

    /* Allocate memory for request buffer */
    req = (CMD *)malloc(sizeof(CMD));
    if (!req)
    {
        printf("%s: no memory!\n", __FUNCTION__);
        return 0;
    }

    /* Allocate memory for response buffer */
    resp = (CMD *)malloc(sizeof(CMD));
    if (!resp)
    {
        printf("%s: no memory!\n", __FUNCTION__);
        free(req);
        return 0;
    }

    /* Prepare request command */
    req->cmd = CMD_READ;
    req->len = ADDR_SIZE + WIDTH_SIZE + COUNT_SIZE;
    req->read.addr = addr;
    req->read.width = width;
    req->read.count = count;
    txlen = CMD_SIZE + LEN_SIZE + req->len;

    /* Send request to slave */
    send_req(req, txlen, verbose);

    /* Wait for response from slave */
    rxlen = get_resp(resp, verbose);

    /* Valid response? */
    if (rxlen && resp->cmd == CMD_READ_RESP && resp->read_resp.addr == addr &&
        resp->read_resp.width == width && resp->read_resp.count == count)
    {
        if (buf)
            memcpy(buf, resp->read_resp.val8, width * count);
    }
    else
    {
        count = 0;
    }

    /* Free memory */
    free(req);
    free(resp);

    return count;
}


u32 slave_write(u32 addr, u32 width, u32 count, u32 *buf, bool verbose)
{
    CMD *req, *resp;
    u32 txlen, rxlen, i;
    u32 *p32;
    u16 *p16;
    u8  *p8;

    Xil_AssertNonvoid(buf);

    /* Allocate memory for request buffer */
    req = (CMD *)malloc(sizeof(CMD));
    if (!req)
    {
        printf("%s: no memory!\n", __FUNCTION__);
        return 0;
    }

    /* Allocate memory for response buffer */
    resp = (CMD *)malloc(sizeof(CMD));
    if (!resp)
    {
        printf("%s: no memory!\n", __FUNCTION__);
        free(req);
        return 0;
    }

    /* Prepare request command */
    req->cmd = CMD_WRITE;
    req->len = ADDR_SIZE + WIDTH_SIZE + COUNT_SIZE + (width * count);
    req->write.addr = addr;
    req->write.width = width;
    req->write.count = count;

    if (width == 1)
    {
        p8 = (u8 *)buf;
        for (i = 0; i < count; ++i)
        {
            req->write.val8[i] = p8[i];
        }
    }
    else if (width == 2)
    {
        p16 = (u16 *)buf;
        for (i = 0; i < count; ++i)
        {
            req->write.val16[i] = p16[i];
        }
    }
    else
    {
        p32 = (u32 *)buf;
        for (i = 0; i < count; ++i)
        {
            req->write.val32[i] = p32[i];
        }
    }

    txlen = CMD_SIZE + LEN_SIZE + req->len;

    /* Send request to slave */
    send_req(req, txlen, verbose);

    /* Wait for response from slave */
    rxlen = get_resp(resp, verbose);

    /* Valid response? */
    if (!rxlen || resp->cmd != CMD_ACK || resp->ack.req != CMD_WRITE || resp->ack.err != E_OK)
        count = 0;

    /* Free memory */
    free(req);
    free(resp);

    return count;
}


u32 slave_setclr(u32 addr, u32 width, u32 mask, u32 val, bool verbose)
{
    CMD *req, *resp;
    u32 txlen, rxlen, len;

    /* Allocate memory for request buffer */
    req = (CMD *)malloc(sizeof(CMD));
    if (!req)
    {
        printf("%s: no memory!\n", __FUNCTION__);
        return 0;
    }

    /* Allocate memory for response buffer */
    resp = (CMD *)malloc(sizeof(CMD));
    if (!resp)
    {
        printf("%s: no memory!\n", __FUNCTION__);
        free(req);
        return 0;
    }

    /* Prepare request command */
    req->cmd = CMD_SETCLR;
    req->len = ADDR_SIZE + WIDTH_SIZE + (width * 2);
    req->setclr.addr = addr;
    req->setclr.width = width;

    if (width == 1)
    {
        req->setclr.u8.mask = mask;
        req->setclr.u8.val = val;
    }
    else if (width == 2)
    {
        req->setclr.u16.mask = mask;
        req->setclr.u16.val = val;
    }
    else
    {
        req->setclr.u32.mask = mask;
        req->setclr.u32.val = val;
    }

    txlen = CMD_SIZE + LEN_SIZE + req->len;

    /* Send request to slave */
    send_req(req, txlen, verbose);

    /* Wait for response from slave */
    rxlen = get_resp(resp, verbose);

    /* Valid response? */
    if (rxlen && resp->cmd == CMD_ACK && resp->ack.req == CMD_SETCLR && resp->ack.err == E_OK)
        len = 1;
    else
        len = 0;

    /* Free memory */
    free(req);
    free(resp);

    return len;
}


static u32 send_req(CMD *req, u32 len, bool verbose)
{
    if (verbose)
    {
        puts("\nSending request:\n");
        print_buffer((u32)req, (void *)req, 1, len, 0);
    }

    /* Send request to slave */
    len = slave_tx((u8 *)req, len, FALSE);

    return len;
}


static u32 get_resp(CMD *resp, bool verbose)
{
    ulong   time = 0;
    u32     len;

    time = get_timer(0);
    do {
        len = slave_rx((u8 *)resp, FALSE);

        /* Timeout? */
        if (get_timer(time) > 3000)
            break;
    } while (!len);

    if (verbose)
    {
        if (!len)
            puts("No response from slave!\n");
        else
            dump_cmd(resp, len);
    }

    return len;
}


static void dump_cmd(CMD *resp, u32 len)
{
    u32 *pload;
    u32 plen;

    /* Print response */
    puts("\nReceived response:\n");
    printf("cmd = %u\n", resp->cmd);
    printf("len = %u\n", resp->len);
    if (resp->cmd == CMD_ECHO)
    {
        puts("ECHO\n");
        puts("payload:\n");
        print_buffer((u32)resp->payload, (void *)resp->payload, 1, resp->len, 0);
    }
    else if (resp->cmd == CMD_READ_RESP)
    {
        puts("READ_RESP\n");
        printf("addr  = %lX\n", (ulong)resp->read_resp.addr);
        printf("width = %u\n", resp->read_resp.width);
        printf("count = %u\n", resp->read_resp.count);
        puts("val:\n");
        plen = resp->read_resp.width * resp->read_resp.count;
        pload = (u32 *)malloc(plen);
        if (!pload)
        {
            printf("%s: no memory!\n", __FUNCTION__);
            return;
        }
        memcpy(pload, resp->read_resp.val8, plen);
        print_buffer((u32)resp->read_resp.addr, (void *)pload, resp->read_resp.width, resp->read_resp.count, 0);
        free(pload);
    }
    else if (resp->cmd == CMD_ACK)
    {
        puts("ACK\n");
        printf("req = %u\n", resp->ack.req);
        printf("err = %u\n", resp->ack.err);
    }
    else
    {
        puts("Bad CMD\n");
        print_buffer((u32)resp, (void *)resp, 1, len, 0);
    }
    puts("\n");
}


u32 slave_rx(u8 *buf, bool verbose)
{
    u8  *rxbuf;
    u32 rxlen;
    u32 len = 0;

    /* Allocate memory for RX buffer */
    rxbuf = (u8 *)malloc(FRAME_BUFFER_SIZE);
    if (!rxbuf)
    {
        printf("%s: no memory!\n", __FUNCTION__);
        return 0;
    }

    /* Receive encoded frame */
    rxlen = uart1_rx(rxbuf);

    /* Decode it */
    if (rxlen)
        len = decode_frame(rxbuf, rxlen, buf, verbose);

    /* Free memory */
    free(rxbuf);

    return len;
}


u32 slave_tx(u8 *buf, u32 len, bool verbose)
{
    u8  *encbuf;
    u32 enclen;

    Xil_AssertNonvoid(buf);
    Xil_AssertNonvoid((len >= MIN_CMD_LEN) && (len <= MAX_CMD_LEN));

    /* Allocate memory for encode buffer */
    encbuf = (u8 *)malloc(FRAME_BUFFER_SIZE);
    if (!encbuf)
    {
        printf("%s: no memory!\n", __FUNCTION__);
        return 0;
    }

    /* Encode it */
    enclen = encode_frame(buf, len, encbuf, verbose);

    /* Send encoded frame */
    if (enclen)
        len = uart1_tx(encbuf, enclen);
    else
        len = 0;

    /* Free memory */
    free(encbuf);

    return len;
}

/****************************** UART Routines ******************************/
static void zynq_uart1_init(void) {
    int ret;

    ret = uclass_get_device(UCLASS_SERIAL, 1, &uart1_dev);

    if (ret == 0) {
        uart1_ops = serial_get_ops(uart1_dev);
	if (uart1_ops->setbrg(uart1_dev, CONFIG_ZYNQ_SERIAL_BAUDRATE1) == 0)
        	uart1_initialized = 1;
    }
    else
        printf("Failed to initialize uart1 deice, err = %d\n", ret);
}

static int zynq_uart1_tstc(void) {
    int ret = 1;

    if (uart1_initialized)
    	if (uart1_ops->pending)
        	ret = uart1_ops->pending(uart1_dev, true);

    return ret;
}

static int zynq_uart1_getc(void) {
    int err = -EAGAIN;

    if (uart1_initialized) {
        do {
            err = uart1_ops->getc(uart1_dev);
            if (err == -EAGAIN)
                WATCHDOG_RESET();
        } while (err == -EAGAIN);
    }

    return (err >= 0 ? err : 0);
}

static void zynq_uart1_putc(const char c) {
    int err;

    if (uart1_initialized) {
        do {
            err = uart1_ops->putc(uart1_dev, c);
        }
        while (err == -EAGAIN);
    }
}
/*
static void zynq_uart1_puts(const char *s) {
    if (uart1_initialized)
        while (*s)
            zynq_uart1_putc(*s++);
}
*/
void uart1_init(void)
{
    /* Initialize UART1 */
    zynq_uart1_init();
}


void uart1_loopback(bool en)
{
    /* Enable/disable local loopback mode */
    printf("uart1_loopback is not supported\n");
}


static u32 uart1_rx(u8 *buf)
{
    u32     len = 0;
    u32     count, last_count = 0;
    ulong   time = 0;

    /* Received anything? */
    if (!zynq_uart1_tstc())
        return 0;

    /* Get it */
    while (1)
    {
        count = zynq_uart1_tstc();
        if (count)
            buf[len] = zynq_uart1_getc();

        /* Finished receiving? */
        if (count == 0)
        {
            /* Got EOF? */
            if ((len > 1) && (buf[len - 1] == HDLC_FRAME_DELIMITER))
                break;

            if (last_count)
            {
                /* Start EOF timer */
                time = get_timer(0);
            }
            else
            {
                /* Timeout? */
                if (get_timer(time) > UART_FIFO_TIMEOUT)
                    break;
            }
        }

        /* Update counters */
        last_count = count;
        len += count;
    }

    return len;
}


static u32 uart1_tx(u8 *buf, u32 len)
{
    u32 count = 0;

    /* Send it */
    while (count < len)
    {
        zynq_uart1_putc(buf[count]);
        ++count;
    }

    return count;
}

/****************************** HDLC Routines ******************************/

static u16 crc_ccitt(u8 *buf, u32 len)
{
    u16 crc = 0xFFFF;
    u8  *pBuf = buf;

    while (len--)
    {
        crc = (crc >> 8) | (crc << 8);
        crc ^= *pBuf++;
        crc ^= (crc & 0xFF) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0xFF) << 5;
    }

    return crc;
}


static u32 stuff_bytes(u8 *inbuf, u32 inlen, u8 *outbuf)
{
    u8  data;
    u32 i;
    u32 count = 0;

    for (i = 0; i < inlen; ++i)
    {
        data = inbuf[i];
        if (data == HDLC_FRAME_DELIMITER || data == HDLC_CONTROL_ESCAPE)
        {
            data ^= HDLC_ESCAPE_BIT;
            *outbuf++ = HDLC_CONTROL_ESCAPE;
            count++;
        }

        *outbuf++ = data;
        count++;
    }

    return count;
}


static u32 encode_frame(u8 *inbuf, u32 inlen, u8 *outbuf, bool verbose)
{
    u16 crc;
    u32 len = 0;
    u32 bytes;
    u8  *pBuf = outbuf;

    /* Insert SOF */
    *pBuf++ = HDLC_FRAME_DELIMITER;
    len++;

    /* Encode data */
    bytes = stuff_bytes(inbuf, inlen, pBuf);
    pBuf += bytes;
    len += bytes;

    /* Encode CRC */
    crc = crc_ccitt(inbuf, inlen);
    bytes = stuff_bytes((u8 *)&crc, sizeof(crc), pBuf);
    pBuf += bytes;
    len += bytes;

    /* Insert EOF */
    *pBuf++ = HDLC_FRAME_DELIMITER;
    len++;

    if (verbose)
    {
        printf("\n%s, inbuf (%lu bytes):\n", __FUNCTION__, (ulong)inlen);
        print_buffer((u32)inbuf, (void *)inbuf, 1, inlen, 0);
        printf("\n%s: outbuf (%lu bytes):\n", __FUNCTION__, (ulong)len);
        print_buffer((u32)outbuf, (void *)outbuf, 1, len, 0);
    }

    return len;
}


static u32 decode_frame(u8 *inbuf, u32 inlen, u8 *outbuf, bool verbose)
{
    u16 crc, crc_calc;
    u8  data;
    u32 i;
    u32 len = 0;
    u8  *pBuf = outbuf;
    u32 err = 0;

    /* Verify frame length */
    if (inlen < MIN_FRAME_LEN || inlen > MAX_FRAME_LEN)
    {
        printf("%s: Invalid len (%lu)\n", __FUNCTION__, (ulong)inlen);
        err = 1;
        goto cleanup;
    }

    /* Check for SOF */
    if (inbuf[0] != HDLC_FRAME_DELIMITER)
    {
        printf("%s: No SOF (0x%02X)\n", __FUNCTION__, inbuf[0]);
        err = 1;
        goto cleanup;
    }

    /* Check for EOF */
    if (inbuf[inlen - 1] != HDLC_FRAME_DELIMITER)
    {
        printf("%s: No EOF (0x%02X)\n", __FUNCTION__, inbuf[inlen - 1]);
        err = 1;
        goto cleanup;
    }

    /* Decode data + CRC, skip SOF and EOF */
    for (i = 1; i < (inlen - 1); ++i)
    {
        data = inbuf[i];
        if (data == HDLC_CONTROL_ESCAPE)
        {
            data = inbuf[++i];
            data ^= HDLC_ESCAPE_BIT;
        }

        *pBuf++ = data;
        len++;
    }

    /* Verify CRC */
    len -= sizeof(crc);
    memmove(&crc, &outbuf[len], sizeof(crc));
    crc_calc = crc_ccitt(outbuf, len);
    if (crc != crc_calc)
    {
        printf("%s: Bad CRC (rx=0x%04X, calc=0x%04X)\n", __FUNCTION__, crc, crc_calc);
        err = 1;
    }

cleanup:
    if (err || verbose)
    {
        printf("\n%s: inbuf (%lu bytes):\n", __FUNCTION__, (ulong)inlen);
        print_buffer((u32)inbuf, (void *)inbuf, 1, inlen, 0);
        printf("\n%s: outbuf (%lu bytes):\n", __FUNCTION__, (ulong)len);
        print_buffer((u32)outbuf, (void *)outbuf, 1, len, 0);
    }

    return (err ? 0 : len);
}

