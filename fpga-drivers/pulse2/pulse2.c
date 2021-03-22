/*
 * Name: pulse2.c
 *
 * Description: Dual pulse generator
 *
 * Copyright:   Copyright (C) 2014-2021 Demand Peripherals, Inc.
 *              All rights reserved.
 *
 * License:     This program is free software; you can redistribute it and/or
 *              modify it under the terms of the Version 2 of the GNU General
 *              Public License as published by the Free Software Foundation.
 *              GPL2.txt in the top level directory is a copy of this license.
 *              This program is distributed in the hope that it will be useful,
 *              but WITHOUT ANY WARRANTY; without even the implied warranty of
 *              MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *              GNU General Public License for more details. 
 *
 *              Please contact Demand Peripherals if you wish to use this code
 *              in a non-GPLv2 compliant manner. 
 * 
 */


/*
 *    The dual pulse generator can generate two non-overlapping pulse 
 *  signals.  The counters for the pulses runs at 100MHz.  The _period_
 *  can be up to 1024 clock cycles long.  Pulse 1 starts at a period
 *  count of 1 and lasts for _p1width_ clock cycles.  Both outputs are
 *  low for _offset_ cycles and then p2 goes high for _p2width_ cycles.
 *  It is strictly required that period > p1width + offset + p2width.
 *
 *  A waveform for the output might look something like this:
 *     p1p:     ____|------|____________________________________|------|___
 *     p1n:     ----|______|------------------------------------|______|---
 *     p2p:     _________________|----|____________________________________
 *     p2n:     -----------------|____|------------------------------------
 * With parameters:
 *     period       |-------------------------------------------|
 *     p1width      |------|           
 *     p2offset            |-----|
 *     p2width                   |----|
 *
 *   The hardware registers do not map directly to the UI parameters.  This
 *  helps keep the size of the Verilog peripheral small.
 *  Hardware  Registers:
 *    Reg 0:  Period in units of 10 ns (high byte = reg 0) (max=1023)
 *    Reg 2:  Width of output p1 in units of 10 ns (max=1023)
 *    Reg 4:  Start count of output p2 in units of 10 ns (max=1023)
 *    Reg 6:  Stop count of output p2 in units of 10 ns (max=1023)
 *
 *
 * Resources:
 *    The config resource sets the period, p1width, p2offset, and p2width
 * All parameters are given in nanoseconds.  
 *
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <syslog.h>
#include <errno.h>
#include <string.h>
#include <sys/fcntl.h>
#include <sys/types.h>
#include "daemon.h"
#include "readme.h"

/**************************************************************
 *  - Limits and defines
 **************************************************************/
        // PULSE2 register definitions
#define PULSE2_REG_PERIOD   0
#define PULSE2_REG_P1WIDTH  2
#define PULSE2_REG_P2START  4
#define PULSE2_REG_P2STOP   6
        // misc constants
#define MAX_LINE_LEN        100
        // resource names and numbers
#define FN_CONFIG           "config"
#define RSC_CONFIG          0
        // Maximum period in nanoseconds
#define MAXNS               10230


/**************************************************************
 *  - Data structures
 **************************************************************/
    // All state info for an instance of an pulse2
typedef struct
{
    void    *pslot;    // handle to peripheral's slot info
    int      period;   // period in nanoseconds
    int      p1width;  // P1 pulse width in nanoseconds
    int      p2offset; // Pause between P1 and P2 in nanoseconds
    int      p2width;  // P2 pulse width in nanoseconds
    void    *ptimer;   // timer to watch for dropped ACK packets
} PULSE2DEV;


/**************************************************************
 *  - Function prototypes
 **************************************************************/
static void packet_hdlr(SLOT *, DP_PKT *, int);
static void pulse2user(int, int, char*, SLOT*, int, int*, char*);
static void noAck(void *, PULSE2DEV *);
static int  pulse2tofpga(PULSE2DEV *, int rscid);
extern int  dpi_tx_pkt(CORE *pcore, DP_PKT *inpkt, int len);


/**************************************************************
 * Initialize():  - Allocate our permanent storage and set up
 * the read/write callbacks.
 **************************************************************/
int Initialize(
    SLOT *pslot)       // points to the SLOT for this peripheral
{
    PULSE2DEV *pctx;    // our local device context

    // Allocate memory for this peripheral
    pctx = (PULSE2DEV *) malloc(sizeof(PULSE2DEV));
    if (pctx == (PULSE2DEV *) 0) {
        // Malloc failure this early?
        dplog("memory allocation failure in pulse2 initialization");
        return (-1);
    }

    // Init our PULSE2DEV structure
    pctx->pslot = pslot;       // our instance of a peripheral
    pctx->period = 5000;
    pctx->p1width = 1000;
    pctx->p2offset = 1000;
    pctx->p2width = 1000;
    pctx->ptimer = 0;          // set while waiting for a response

    // Register this slot's packet handler and private data
    (pslot->pcore)->pcb  = packet_hdlr;
    pslot->priv = pctx;

    // Add the handlers for the user visible resources
    pslot->rsc[RSC_CONFIG].name = "config";
    pslot->rsc[RSC_CONFIG].flags = IS_READABLE | IS_WRITABLE;
    pslot->rsc[RSC_CONFIG].bkey = 0;
    pslot->rsc[RSC_CONFIG].pgscb = pulse2user;
    pslot->rsc[RSC_CONFIG].uilock = -1;
    pslot->rsc[RSC_CONFIG].slot = pslot;
    pslot->name = "pulse2";
    pslot->desc = "Dual non-overlapping pulse generator";
    pslot->help = README;

    return (0);
}

/**************************************************************
 * packet_hdlr():  - Handle incoming packets from the FPGA board
 **************************************************************/
static void packet_hdlr(
    SLOT   *pslot,     // handle for our slot's internal info
    DP_PKT *pkt,       // the received packet
    int     len)       // number of bytes in the received packet
{
    PULSE2DEV *pctx;    // our local info

    pctx = (PULSE2DEV *)(pslot->priv);  // Our "private" data is a PULSE2DEV

    // Clear the timer on write response packets
    if ((pkt->cmd & DP_CMD_OP_MASK) == DP_CMD_OP_WRITE) {
        if (pctx->ptimer) {
            del_timer(pctx->ptimer);  //Got the ACK
            pctx->ptimer = 0;
        }
        return;
    }

    // There are no other packets from the pulse2 FPGA code
    // so if we get here there is a problem.  Log the error.
    dplog("invalid pulse2 packet from board to host");

    return;
}


/**************************************************************
 * pulse2user():  - The user is reading or writing the pulse widths.
 * Get the value and update the pulse2 on the BaseBoard or read the
 * value and write it into the supplied buffer.
 **************************************************************/
static void pulse2user(
    int      cmd,      //==DPGET if a read, ==DPSET on write
    int      rscid,    // ID of resource being accessed
    char    *val,      // new value for the resource
    SLOT    *pslot,    // pointer to slot info.
    int      cn,       // Index into UI table for requesting conn
    int     *plen,     // size of buf on input, #char in buf on output
    char    *buf)
{
    PULSE2DEV *pctx;   // our local info
    int      ret;      // return count
    int      txret;    // ==0 if the packet went out OK
    int      newperiod;
    int      newp1width;
    int      newp2offset;
    int      newp2width;

    pctx = (PULSE2DEV *) pslot->priv;

    // print configuration
    if (cmd == DPGET) {
        ret = snprintf(buf, *plen, "%d %d %d %d\n", pctx->period,
                 pctx->p1width, pctx->p2offset, pctx->p2width);
        *plen = ret;  // (errors are handled in calling routine)
        return;
    }

    // Must be a dpset
    else {
        ret = sscanf(val, "%d %d %d %d", &newperiod, &newp1width,
                     &newp2offset, &newp2width);
        // Sanity check the values.  All must be zero or  positive
        // and the sum of the last three must be less than the first
        if ((newperiod < 0) || (newperiod > MAXNS) ||
            (newp1width < 0) || (newp1width > MAXNS) ||
            (newp2offset < 0) || (newp2offset > MAXNS) ||
            (newp2width < 0) || (newp2width > MAXNS) ||
            (newperiod <= (newp1width + newp2offset + newp2width))) {
            ret = snprintf(buf, *plen,  E_BDVAL, pslot->rsc[rscid].name);
            *plen = ret;
            return;
        }
        // Valid values.  Save and update the FPGA
        pctx->period = newperiod;
        pctx->p1width = newp1width;
        pctx->p2offset = newp2offset;
        pctx->p2width = newp2width;
    }

    txret =  pulse2tofpga(pctx, rscid);   // This peripheral's context
    if (txret != 0) {
        // the send of the new value did not succeed.  This probably
        // means the input buffer to the USB port is full.  Tell the
        // user of the problem.
        ret = snprintf(buf, *plen, E_WRFPGA);
        *plen = ret;  // (errors are handled in calling routine)
        return;
    }

    // Start timer to look for a write response.
    if (pctx->ptimer == 0)
        pctx->ptimer = add_timer(DP_ONESHOT, 100, noAck, (void *) pctx);

    return;
}


/**************************************************************
 * pulse2tofpga():  - Send pulse widths to the FPGA card.
 * Return zero on success
 **************************************************************/
int pulse2tofpga(
    PULSE2DEV *pctx,    // This peripheral's context
    int        rscid)
{
    DP_PKT   pkt;      // send write and read cmds to the pulse2
    SLOT    *pmyslot;  // This peripheral's slot info
    CORE    *pmycore;  // FPGA peripheral info
    int      txret;    // ==0 if the packet went out OK
    int      p2start;  // clock count when P2 rises
    int      p2stop;   // clock count when P2 falls

    pmyslot = pctx->pslot;
    pmycore = pmyslot->pcore;

    // Got new pulse widths.  Send down to the card.
    pkt.cmd = DP_CMD_OP_WRITE | DP_CMD_AUTOINC;
    pkt.core = pmycore->core_id;
    pkt.reg = PULSE2_REG_PERIOD;
    pkt.count = 8;  // four two-byte parameters
    pkt.data[0] = ((pctx->period / 10) >> 8) & 0x000000ff;  // high
    pkt.data[1] = (pctx->period / 10) & 0x000000ff;         // low
    pkt.data[2] = ((pctx->p1width / 10) >> 8) & 0x000000ff; // high
    pkt.data[3] = (pctx->p1width / 10) & 0x000000ff;        // low
    p2start = pctx->p1width + pctx->p2offset;
    pkt.data[4] = ((p2start / 10) >> 8) & 0x000000ff;       // high
    pkt.data[5] = (p2start / 10) & 0x000000ff;              // low
    p2stop = p2start + pctx->p2width;
    pkt.data[6] = ((p2stop / 10) >> 8) & 0x000000ff;        // high
    pkt.data[7] = (p2stop / 10) & 0x000000ff;               // low

    txret = dpi_tx_pkt(pmycore, &pkt, 4 + pkt.count); // 4 header + data

    return(txret);
}


/**************************************************************
 * noAck():  Wrote to the board but did not get a reply.  Handle
 * the timeout for this.
 **************************************************************/
static void noAck(
    void      *timer,   // handle of the timer that expired
    PULSE2DEV *pctx)    // points to instance of this peripheral
{
    // Log the missing ack
    dplog(E_NOACK);

    return;
}
// end of pulse2.c
