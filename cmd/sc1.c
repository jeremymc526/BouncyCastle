/*
 * Command for testing SCx modules.
 *
 * Copyright (C) 2014 North Atlantic Industries, Inc.
 */

#include <common.h>
#include <command.h>
#include <console.h>
#include "NAIComms.h"
#include "cmd_naimsgutils.h"
#include "cmd_naiconfigmsgutils.h"
#include "cmd_naiopermsgutils.h"


#define TYPE_SC1                1
#define TYPE_SC3                3

#define NUM_CHANNELS            ((type == TYPE_SC1) ? 4 : 8)
#define FIFO_SIZE               ((type == TYPE_SC1) ? 0x200000UL : 0x100000UL)

#define LOGICAL_SLOT_SIZE       0x4000UL

#define IRQ_REGS_BASE           (((l_slot - 1) * LOGICAL_SLOT_SIZE) + 0x800UL)
#define IRQ_NUM_REGS            4
#define IRQ_STATUS_REG(ch)      (IRQ_REGS_BASE + (ch * 0x10))

#define SC_REGS_BASE            (((l_slot - 1) * LOGICAL_SLOT_SIZE) + 0x1000UL)
#define SC_NUM_REGS             25
#define CH_STRIDE               ((type == TYPE_SC1) ? 4 : 0x80)
#define REG_STRIDE              ((type == TYPE_SC1) ? 0x10 : 4)
#define CH_REG(ch, reg)         (SC_REGS_BASE + reg + ((ch - 1) * CH_STRIDE))

#define TX_FIFO_REG(ch)         CH_REG(ch, 0 * REG_STRIDE)
#define RX_FIFO_REG(ch)         CH_REG(ch, 1 * REG_STRIDE)
#define TX_COUNT_REG(ch)        CH_REG(ch, 2 * REG_STRIDE)
#define RX_COUNT_REG(ch)        CH_REG(ch, 3 * REG_STRIDE)
#define PROTO_REG(ch)           CH_REG(ch, 4 * REG_STRIDE)
#define CLK_MODE_REG(ch)        CH_REG(ch, 5 * REG_STRIDE)
#define IFL_REG(ch)             CH_REG(ch, 6 * REG_STRIDE)
#define TXRX_REG(ch)            CH_REG(ch, 7 * REG_STRIDE)
#define CTRL_REG(ch)            CH_REG(ch, 8 * REG_STRIDE)
#define BAUD_REG(ch)            CH_REG(ch, 10 * REG_STRIDE)

// Tx-Rx Configuration register
#define TXRX_CH_ENABLE          (1UL << 24)
#define TXRX_RUN_BIT            (1UL << 27)

// Control register
#define CTRL_TRISTATE_TX        (1UL << 8)
#define CTRL_TX_INIT            (1UL << 16)
#define CTRL_RCVR_EN            (1UL << 18)

// Channel Status register
#define STATUS_RX_DATA_AVAIL    (1UL <<  4)
#define STATUS_TX_COMPLETE      (1UL <<  9)
#define STATUS_BIT_PASSED       (1UL << 30)
#define STATUS_CH_READY         (1UL << 31)

#define BUF_SIZE                64  // 125 DWORDs max (max SERDES payload is 250 WORDs)
#define STR_SC_REG              "  %p: %-16s= 0x%08lX\n"

// Global variables
static ulong gbuf[BUF_SIZE];
static uchar type = TYPE_SC1;
static uchar block_rd;
static uchar block_wr;
static uchar l_slot = 1;

// Function prototypes
static int sc_test(int argc, char * const argv[]);
static void reg_dump(uchar ch);
static void print_regs(ulong addr, ulong count, uchar stride, char **regs);
static int init_slot(uchar slot);
static int read32(ulong addr, ulong count, uchar stride, ulong *buf);
static int write32(ulong addr, ulong count, uchar stride, ulong *buf);

char *irq_regs[IRQ_NUM_REGS] = {
    "raw_status",
    "status",
    "mask",
    "edge_level"
};

char *sc_regs[SC_NUM_REGS] = {
    "tx_fifo",
    "rx_fifo",
    "tx_count",
    "rx_count",
    "proto",
    "clk_mode",
    "if_levels",
    "tx_rx_cfg",
    "ctrl",
    "data_cfg",
    "baud",
    "preamble",
    "tx_fifo_a_empty",
    "rx_fifo_a_full",
    "rx_fifo_hi",
    "rx_fifo_lo",
    "hdlc_rx_char",
    "hdlc_tx_char",
    "xon_char",
    "xoff_char",
    "term_char",
    "timeout",
    "fifo_stat",
    "tx_fifo_size",
    "rx_fifo_size"
};

static int do_sc1(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    int rc;

    type = TYPE_SC1;
    rc = sc_test(argc, argv);
    return rc;
}


static int do_sc3(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    int rc;

    type = TYPE_SC3;
    rc = sc_test(argc, argv);
    return rc;
}


static int sc_test(int argc, char * const argv[])
{
    uchar   slot;
    uchar   ch;
    uchar   mode;
    uchar   ifl = 0x44;
    uchar   clk = 0;
    uchar   data = 0x55;
    ulong   len = FIFO_SIZE;
    ulong   baud = 1000000;
    ulong   i, val, temp, fifo_len, count, pattern;
    ulong   stat_mask = STATUS_TX_COMPLETE | STATUS_RX_DATA_AVAIL;
    ulong   time;

    // Default to block operations
    block_rd = TRUE;
    block_wr = TRUE;

    if (argc < 4)
        return CMD_RET_USAGE;

    // Get module slot
    slot = simple_strtoul(argv[1], NULL, 10);
    l_slot = 1;
    if (slot > 10)
    {
        l_slot = slot % 10;
        slot /= 10;
    }
    if (slot < 1 || slot > 6 || l_slot < 1 || l_slot > 4)
        return CMD_RET_USAGE;

    // Get channel
    ch = simple_strtoul(argv[2], NULL, 10);
    if (ch < 1 || ch > NUM_CHANNELS)
        return CMD_RET_USAGE;

    // Run BIT?
    if (*argv[3] == 'b')
    {
        if (argc > 4)
            return CMD_RET_USAGE;

        if (!init_slot(slot))
        {
            // Run BIT on a given channel
            printf("\nRunning BIT on Ch%d... ", ch);
            gbuf[0] = TXRX_RUN_BIT;
            write32(TXRX_REG(ch), 1, 0, gbuf);
            time = get_timer(0);
            do {
                read32(TXRX_REG(ch), 1, 0, &val);
                // It should take < 1ms for BIT to finish
                if (get_timer(time) > 100)
                    break;
            } while (val & TXRX_RUN_BIT);
            if (val & TXRX_RUN_BIT)
            {
                // Turn off BIT
                gbuf[0] = 0;
                write32(TXRX_REG(ch), 1, 0, gbuf);
                puts("(timeout)\n\n");
            }
            else
            {
                // Check if BIT passed
                read32(IRQ_STATUS_REG(ch), 1, 0, gbuf);
                if (gbuf[0] & STATUS_BIT_PASSED)
                    puts("PASSED\n\n");
                else
                    puts("FAILED\n\n");
            }
        }

        // We are done
        return CMD_RET_SUCCESS;
    }

    // Register dump?
    if (strncmp(argv[3], "re", 2) == 0)
    {
        if (argc > 4)
        {
            if (*argv[4] == 'b')
                block_rd = TRUE;
            else if (*argv[4] == 's')
                block_rd = FALSE;
            else
                return CMD_RET_USAGE;
        }

        if (!init_slot(slot))
            reg_dump(ch);

        // We are done
        return CMD_RET_SUCCESS;
    }

    // Need at least 4 args to continue
    if ((argc < 4) || (argc > 11))
        return CMD_RET_USAGE;

    // Get mode
    mode = *argv[3];
    if ((mode == 'h') || (mode == 'a'))
        ifl = 0x44;
    else if (mode == 'r')
        ifl = 2;
    else
        return CMD_RET_USAGE;

    // Get length
    if (argc > 4)
        len = simple_strtoul(argv[4], NULL, 16);

    // Get baudrate
    if (argc > 5)
        baud = simple_strtoul(argv[5], NULL, 10);

    // Get interface levels
    if (argc > 6)
    {
        ifl = simple_strtoul(argv[6], NULL, 16);
        if (ifl < 0 || ifl > 0x47)
            return CMD_RET_USAGE;
    }

    // Get clock mode
    if (argc > 7)
    {
        if (*argv[7] == 'i')
            clk = 0;
        else if (*argv[7] == 'e')
            clk = 1;
        else
            return CMD_RET_USAGE;
    }

    // Get data pattern
    if (argc > 8)
        data = simple_strtoul(argv[8], NULL, 16);

    // Get Block/Single writes
    if (argc > 9)
    {
        if (*argv[9] == 'b')
            block_wr = TRUE;
        else if (*argv[9] == 's')
            block_wr = FALSE;
        else
            return CMD_RET_USAGE;
    }

    // Get Block/Single reads
    if (argc > 10)
    {
        if (*argv[10] == 'b')
            block_rd = TRUE;
        else if (*argv[10] == 's')
            block_rd = FALSE;
        else
            return CMD_RET_USAGE;
    }

    // All good - do it!!!
    putc('\n');
    printf("slot = %u\n", slot);
    printf("chan = %u\n", ch);
    printf("mode = ");
    if (mode == 'h')
        puts("HDLC\n");
    else if (mode == 'a')
        puts("ASync\n");
    else if (mode == 'r')
        puts("Rx Only\n");
    printf("len  = 0x%lX\n", len);
    printf("baud = %lu\n", baud);
    printf("ifl  = 0x%02X\n", ifl);
    printf("clk  = %s\n", clk ? "External" : "Internal");
    printf("data = 0x%02X\n\n", data);

    if (init_slot(slot))
        return CMD_RET_SUCCESS;

    // Check if BIT passed
    read32(IRQ_STATUS_REG(ch), 1, 0, gbuf);
    if (!(gbuf[0] & STATUS_BIT_PASSED))
    {
        puts("Channel BIT failed - aborting!\n\n");
        return CMD_RET_SUCCESS;
    }

    // Set configuration registers
    gbuf[0] = ifl;
    write32(IFL_REG(ch), 1, 0, gbuf);
    write32(BAUD_REG(ch), 1, 0, &baud);
    gbuf[0] = clk;
    write32(CLK_MODE_REG(ch), 1, 0, gbuf);
    if (mode == 'h')
    {
        gbuf[0] = 0x3;
        temp = 0x40000;
    }
    else
    {
        gbuf[0] = 0;
        temp = 0;
    }
    write32(PROTO_REG(ch), 1, 0, gbuf);

    // Enable channel
    gbuf[0] = temp | TXRX_CH_ENABLE;
    write32(TXRX_REG(ch), 1, 0, gbuf);
    time = get_timer(0);
    do {
        read32(IRQ_STATUS_REG(ch), 1, 0, &val);
        // It should take less than 10 ms for channel to be configured
        if (get_timer(time) > 10)
            break;
    } while (!(val & STATUS_CH_READY));
    if (!(val & STATUS_CH_READY))
    {
        // Disable channel
        gbuf[0] = 0;
        write32(TXRX_REG(ch), 1, 0, gbuf);

        puts("Channel not ready - aborting!\n\n");
        return CMD_RET_SUCCESS;
    }

    // TX count should equal 0
    read32(TX_COUNT_REG(ch), 1, 0, &val);
    if (val != 0)
        printf("-TX FIFO write: TX count != 0; count=%08lX\n", val);

    // RX count should equal 0
    read32(RX_COUNT_REG(ch), 1, 0, &val);
    if (val != 0)
        printf("-TX FIFO write: RX count != 0; count=%08lX\n", val);

    // If we are in RX only mode, this loop runs until aborted by user
    if (mode == 'r')
    {
        // Enable Receiver
        gbuf[0] = CTRL_TRISTATE_TX | CTRL_RCVR_EN;
        write32(CTRL_REG(ch), 1, 0, gbuf);

        // Wait for data
        printf("Receiving... ");
        while (1)
        {
            read32(RX_COUNT_REG(ch), 1, 0, &val);
            if (val)
            {
                // Get data and print it
                read32(RX_FIFO_REG(ch), 1, 0, &temp);
                printf("%02lX ", temp & 0xFF);
            }

            // Check for abort
            if (ctrlc())
                break;
        };

        // Stop and disable channel
        gbuf[0] = CTRL_TRISTATE_TX;
        write32(CTRL_REG(ch), 1, 0, gbuf);
        gbuf[0] = 0;
        write32(TXRX_REG(ch), 1, 0, gbuf);

        puts("\n\n");
        return CMD_RET_SUCCESS;
    }

    // Fill with known unique pattern
    printf("Writing TX FIFO... ");
    pattern = data;
    fifo_len = len;
    while (fifo_len)
    {
        count = (fifo_len > BUF_SIZE) ? BUF_SIZE : fifo_len;

        // Fill the buffer
        for (i = 0; i < count; i++)
        {
            gbuf[i] = pattern & 0xFF;
            ++pattern;
        }

        if (block_wr)
        {
            // Block write
            if (write32(TX_FIFO_REG(ch), count, 0, gbuf))
                break;
        }
        else
        {
            // Single write
            for (i = 0; i < count; i++)
            {
                if (write32(TX_FIFO_REG(ch), 1, 0, &gbuf[i]))
                    break;
            }
        }

        fifo_len -= count;

        // Check for abort
        if (ctrlc())
            break;
    }
    puts("Done\n");

    // TX count should equal len
    read32(TX_COUNT_REG(ch), 1, 0, &val);
    if (val != len)
        printf("-TX/RX EN: TX count != len; len=%08lX, count=%08lX\n", len, val);

    // RX count should equal 0
    read32(RX_COUNT_REG(ch), 1, 0, &val);
    if (val != 0)
        printf("-TX/RX EN: RX count != 0; count=%08lX\n", val);

    // Enable RX and initiate TX
    gbuf[0] = CTRL_TRISTATE_TX | CTRL_RCVR_EN | CTRL_TX_INIT;
    write32(CTRL_REG(ch), 1, 0, gbuf);

    // Wait for TX complete and RX data available
    printf("Transmitting... ");
    time = get_timer(0);
    do {
        read32(IRQ_STATUS_REG(ch), 1, 0, &val);
        // It should take ~19hrs to send 0x200000 DWORDs at 300 baud; Abort on timeout or key press
        if ((get_timer(time) > 70000000UL) || ctrlc())
            break;
    } while ((val & stat_mask) != stat_mask);
    time = get_timer(time);
    printf("%s (status=0x%08lX, time=%ldms)\n", (val & STATUS_RX_DATA_AVAIL) ? "COMPLETED" : "FAILED!", val, time);

    // Stop
    gbuf[0] = CTRL_TRISTATE_TX;
    write32(CTRL_REG(ch), 1, 0, gbuf);

    // TX count should equal 0
    read32(TX_COUNT_REG(ch), 1, 0, &val);
    if (val != 0)
        printf("-RX FIFO read: TX count != 0; count=%08lX\n", val);

    // RX count should equal len
    read32(RX_COUNT_REG(ch), 1, 0, &val);
    if (val != len)
        printf("-RX FIFO read: RX count != len; len=%08lX, count=%08lX\n", len, val);

    // Read back and verify
    printf("Verifying... ");
    pattern = data;
    fifo_len = len;
    while (fifo_len)
    {
        count = (fifo_len > BUF_SIZE) ? BUF_SIZE : fifo_len;

        if (block_rd)
        {
            // Block read
            if (read32(RX_FIFO_REG(ch), count, 0, gbuf))
                break;
        }
        else
        {
            // Single read
            for (i = 0; i < count; i++)
            {
                if (read32(RX_FIFO_REG(ch), 1, 0, &gbuf[i]))
                    break;
            }
        }

        for (i = 0; i < count; ++i)
        {
            val = gbuf[i];
            if (mode == 'h')
            {
                if ((fifo_len - i) == len)
                {
                    if ((val & 0x4000) == 0)
                        printf("\nRX FIFO word 0: 0x4000 not set, data=%08lX\n", val);
                    val &= ~0x4000;
                }
                else if ((fifo_len - i) == 1)
                {
                    if ((val & 0x200) == 0)
                        printf("\nRX FIFO word 0x%lX: 0x200 not set, data=%08lX\n", i, val);
                    val &= ~0x200;
                }
            }
            else if (mode == 'a')
            {
                val &= ~0x200;  // Mask parity bit for now
            }

            temp = pattern & 0xFF;
            ++pattern;
            if (val != temp)
            {
                // Disable channel
                gbuf[0] = 0;
                write32(TXRX_REG(ch), 1, 0, gbuf);

                printf("\nData mismatch at word %08lX: expected %08lX, got %08lX\n\n", i, temp, val);
                return CMD_RET_SUCCESS;
            }
        }

        fifo_len -= count;

        // Check for abort
        if (ctrlc())
            break;
    }
    puts("Done\n");

    // TX count should equal 0
    read32(TX_COUNT_REG(ch), 1, 0, &val);
    if (val != 0)
        printf("+RX FIFO read: TX count != 0; count=%08lX\n", val);

    // RX count should equal 0
    read32(RX_COUNT_REG(ch), 1, 0, &val);
    if (val != 0)
        printf("+RX FIFO read: RX count != 0; count=%08lX\n", val);

    // Disable channel
    gbuf[0] = 0;
    write32(TXRX_REG(ch), 1, 0, gbuf);

    if (fifo_len == 0)
        puts("\nPASSED (RX data = TX data)\n");

    putc('\n');
    return CMD_RET_SUCCESS;
}


static void reg_dump(uchar ch)
{
    // Enable channel so we can read all SC registers
    gbuf[0] = TXRX_CH_ENABLE;
    write32(TXRX_REG(ch), 1, 0, gbuf);

    printf("\nCh %d IRQ registers:\n", ch);
    print_regs(IRQ_STATUS_REG(ch), IRQ_NUM_REGS, 4, irq_regs);

    printf("\nCh %d SC%d registers:\n", ch, type);
    print_regs(CH_REG(ch, 0), SC_NUM_REGS, REG_STRIDE, sc_regs);

    // Disable channel
    gbuf[0] = 0;
    write32(TXRX_REG(ch), 1, 0, gbuf);

    putc('\n');
}


static void print_regs(ulong addr, ulong count, uchar stride, char **regs)
{
    ulong   offset, i;

    // Read them
    offset = addr;
    if (block_rd)
    {
        // Block read
        if (read32(offset, count, stride, gbuf))
            return;
    }
    else
    {
        // Single read
        for (i = 0; i < count; i++)
        {
            if (read32(offset, 1, 0, &gbuf[i]))
                return;
            offset += stride;
        }
    }

    // Print them
    offset = addr;
    for (i = 0; i < count; ++i)
    {
        printf(STR_SC_REG, (void *)offset, regs[i], gbuf[i]);
        offset += stride;
    }
}


static int do_mod(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    uchar   slot;
    uchar   mode;
    bool    incr = false;
    ulong   data = 0;
    ulong   start = 0;
    ulong   len = 0x400;
    ulong   i, offset, count;
    uchar   stride = 0xFF;


    if (argc < 3)
        return CMD_RET_USAGE;

    // Get module slot
    slot = simple_strtoul(argv[1], NULL, 10);
    l_slot = 1;
    if (slot > 10)
    {
        l_slot = slot % 10;
        slot /= 10;
    }
    if (slot < 1 || slot > 6 || l_slot < 1 || l_slot > 4)
        return CMD_RET_USAGE;

    // Write or read
    mode = *argv[2];
    if (mode == 'w')
    {
        // Max 7 args
        if (argc > 7)
            return CMD_RET_USAGE;
    }
    else if (mode == 'r')
    {
        // Max 5 args
        if (argc > 5)
            return CMD_RET_USAGE;
    }
    else
    {
        return CMD_RET_USAGE;
    }

    // Block/FIFO operations?
    if (argv[2][1] == 'b')
        stride = 4;
    else if (argv[2][1] == 'f')
        stride = 0;
    else if (argv[2][1])
        return CMD_RET_USAGE;

    // Get start offset
    if (argc > 3)
        start = simple_strtoul(argv[3], NULL, 16);

    // Get length
    if (argc > 4)
        len = simple_strtoul(argv[4], NULL, 16);

    // Get data pattern
    if (argc > 5)
        data = simple_strtoul(argv[5], NULL, 16);

    // Get increment
    if (argc > 6)
    {
        if (*argv[6] == 'i')
            incr = true;
        else
            return CMD_RET_USAGE;
    }

    // All good - do it!!!
    if (init_slot(slot))
        return CMD_RET_SUCCESS;

    putc('\n');
    offset = ((l_slot - 1) * LOGICAL_SLOT_SIZE) + start;
    if (mode == 'w')
    {
        printf("Writing... ");
        while (len)
        {
            count = (len > BUF_SIZE) ? BUF_SIZE : len;

            // Fill the buffer
            for (i = 0; i < count; i++)
            {
                gbuf[i] = data;

                // Increment data pattern
                if (incr)
                    ++data;
            }

            if (stride != 0xFF)
            {
                // Block write
                if (write32(offset, count, stride, gbuf))
                    break;
            }
            else
            {
                // Single write
                for (i = 0; i < count; i++)
                {
                    if (write32(offset + (i * 4), 1, 0, &gbuf[i]))
                        break;
                }
            }

            len -= count;
            offset += (count * 4);

            // Check for abort
            if (ctrlc())
                break;
        }
        puts("Done\n\n");
    }
    else
    {
        while (len)
        {
            count = (len > BUF_SIZE) ? BUF_SIZE : len;

            if (stride != 0xFF)
            {
                // Block read
                if (read32(offset, count, stride, gbuf))
                    break;
            }
            else
            {
                // Single read
                for (i = 0; i < count; i++)
                {
                    if (read32(offset + (i * 4), 1, 0, &gbuf[i]))
                        break;
                }
            }

            if (print_buffer(offset, gbuf, 4, count, 0))
                break;

            len -= count;
            offset += (count * 4);

            // Check for abort
            if (ctrlc())
                break;
        }
        putc('\n');
    }

    return CMD_RET_SUCCESS;
}


static int init_slot(uchar slot)
{
    int ret;

	ret = nai_init_as_slot(MB_SLOT);
    if (ret == NAI_SUCCESS)
    {
        nai_assign_hard_coded_module_slot(slot);
        ret = nai_perform_init_slot_addressing();
    }

    if (ret != NAI_SUCCESS)
        printf("init_slot failed (%d)\n", ret);

    return ret;
}


static int read32(ulong addr, ulong count, uchar stride, ulong *buf)
{
    int ret;

    if (count > 1)
        ret = nai_read_block32_request(addr, count, stride, (uint32_t *)buf);
    else
        ret = nai_read_reg32_request(addr, (uint32_t *)buf);   

    if (ret != NAI_SUCCESS)
        printf("read32 failed (%d)\n", ret);

    return ret;
}


static int write32(ulong addr, ulong count, uchar stride, ulong *buf)
{
    int ret;

    if (count > 1)
        ret = nai_write_block32_request(addr, count, stride, (uint32_t *)buf);
    else
        ret = nai_write_reg32_request(addr, (uint32_t)*buf);   

    if (ret != NAI_SUCCESS)
        printf("write32 failed (%d)\n", ret);

    return ret;
}


U_BOOT_CMD(
    sc1, 11, 0, do_sc1,
    "SC1 module test",
    "s[l] ch regs [b/s] - Dump SC1 registers\n"
    "    s[l]    : Slot number (1-6) [Logical slot number (1-4)]\n"
    "    ch      : Channel number (1-4)\n"
    "    regs    : Register dump\n"
    "    b/s     : Block/Single reads; default = Block\n"
    "sc1 s[l] ch bit - Run BIT\n"
    "    s[l]    : Slot number (1-6) [Logical slot number (1-4)]\n"
    "    ch      : Channel number (1-4)\n"
    "    bit     : Built-In Test\n"
    "sc1 s[l] ch h/a/r [len] [baud] [ifl] [i/e] [data] [b/s(Tx)] [b/s(Rx)]\n"
    "    s[l]    : Slot number (1-6) [Logical slot number (1-4)]\n"
    "    ch      : Channel number (1-4)\n"
    "    h/a/r   : HDLC/ASync/RxOnly\n"
    "    len     : Length in HEX, up to 2M (0x200000); default = 2M (0x200000)\n"
    "    baud    : Baud rate in DEC; default = 1M (1000000)\n"
    "    ifl     : Interface levels (0-0x47); default = 0x44\n"
    "    i/e     : Internal/External clock; default = Internal\n"
    "    data    : Starting data pattern; default = 0x55\n"
    "    b/s(Tx) : Block/Single Tx FIFO access; default = Block\n"
    "    b/s(Rx) : Block/Single Rx FIFO access; default = Block\n"
);


U_BOOT_CMD(
    sc3, 11, 0, do_sc3,
    "SC3 module test",
    "s[l] ch regs [b/s] - Dump SC3 registers\n"
    "    s[l]    : Slot number (1-6) [Logical slot number (1-4)]\n"
    "    ch      : Channel number (1-8)\n"
    "    regs    : Register dump\n"
    "    b/s     : Block/Single reads; default = Block\n"
    "sc3 s[l] ch bit - Run BIT\n"
    "    s[l]    : Slot number (1-6) [Logical slot number (1-4)]\n"
    "    ch      : Channel number (1-8)\n"
    "    bit     : Built-In Test\n"
    "sc3 s[l] ch h/a/r [len] [baud] [ifl] [i/e] [data] [b/s(Tx)] [b/s(Rx)]\n"
    "    s[l]    : Slot number (1-6) [Logical slot number (1-4)]\n"
    "    ch      : Channel number (1-8)\n"
    "    h/a/r   : HDLC/ASync/RxOnly\n"
    "    len     : Length in HEX, up to 1M (0x100000); default = 1M (0x100000)\n"
    "    baud    : Baud rate in DEC; default = 1M (1000000)\n"
    "    ifl     : Interface levels (0-0x47); default = 0x44\n"
    "    i/e     : Internal/External clock; default = Internal\n"
    "    data    : Starting data pattern; default = 0x55\n"
    "    b/s(Tx) : Block/Single Tx FIFO access; default = Block\n"
    "    b/s(Rx) : Block/Single Rx FIFO access; default = Block\n"
);


U_BOOT_CMD(
    mod, 7, 0, do_mod,
    "Generic module test",
    "s[l] r[b/f] [start] [len] - Read module memory/registers\n"
    "    s[l]   : Slot number (1-6) [Logical slot number (1-4)]\n"
    "    r[b/f] : Read [Block/Fifo] memory/registers\n"
    "    start  : Start offset in HEX; default = 0\n"
    "    len    : Length in HEX; default = 1K (0x400)\n"
    "mod s[l] w[b/f] [start] [len] [data] [i] - Write module memory/registers\n"
    "    s[l]   : Slot number (1-6) [Logical slot number (1-4)]\n"
    "    w[b/f] : Write [Block/Fifo] memory/registers\n"
    "    start  : Start offset in HEX; default = 0\n"
    "    len    : Length in HEX; default = 1K (0x400)\n"
    "    data   : Starting data pattern; default = 0\n"
    "    i      : Increment data pattern; default = no\n"
);

