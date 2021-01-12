/*
 *  Name: pwmin4.c
 *
 *  Description: Quad PWM input
 *
 *  How the FPGA peripherals works:
 *      Imagine four inputs with periodic signals on each.  Each signal
 *  has at least three transitions during the sample period.  That means
 *  there are at most 12 intervals in the input stream with a transition
 *  occurring at the end of each interval.
 *      Registers store the values of the inputs at the start of an interval
 *  and the duration in clock counts of the interval.  At the end of a cycle
 *  the values are sent up to the host and a new cycle is started.  A new
 *  cycle starts on the first input transition after sending up to the host.
 *  An input is ignored after it has made three transitions.  This prevents
 *  a faster input from filling the transition table and thus ignoring the
 *  other inputs.
 *
 *  Hardware Registers:
  *  Registers:
 *      Reg 0:  Interval 0 duration in clk counts              (16 bits)
 *      Reg 2:  Input values at the start of the interval (4 bits)
 *      Reg 4:  Interval 1 duration in clk counts              (16 bits)
 *      Reg 6:  Input values at the start of the interval (4 bits)
 *      Reg 8:  Interval 2 duration in clk counts              (16 bits)
 *      Reg 10: Input values at the start of the interval (4 bits)
 *      Reg 12: Interval 3 duration in clk counts              (16 bits)
 *      Reg 14: Input values at the start of the interval (4 bits)
 *      Reg 16: Interval 4 duration in clk counts              (16 bits)
 *      Reg 18: Input values at the start of the interval (4 bits)
 *      Reg 20: Interval 5 duration in clk counts              (16 bits)
 *      Reg 22: Input values at the start of the interval (4 bits)
 *      Reg 24: Interval 6 duration in clk counts              (16 bits)
 *      Reg 26: Input values at the start of the interval (4 bits)
 *      Reg 28: Interval 7 duration in clk counts              (16 bits)
 *      Reg 30: Input values at the start of the interval (4 bits)
 *      Reg 32: Interval 8 duration in clk counts              (16 bits)
 *      Reg 34: Input values at the start of the interval (4 bits)
 *      Reg 36: Interval 9 duration in clk counts              (16 bits)
 *      Reg 38: Input values at the start of the interval (4 bits)
 *      Reg 40: Interval 10 duration in clk counts             (16 bits)
 *      Reg 42: Input values at the start of the interval (4 bits)
 *      Reg 44: Interval 11 duration in clk counts             (16 bits)
 *      Reg 46: Input values at the start of the interval (4 bits)
 *      Reg 48: Clk source in the lower 4 bits, then the number of intervals
 *              in use, and the start output values in the next 4 bits
 *
 *  The clock source is selected by the lower 4 bits of register 48:
 *      0:  Off
 *      1:  20 MHz
 *      2:  10 MHz
 *      3:  5 MHz
 *      4:  1 MHz
 *      5:  500 KHz
 *      6:  100 KHz
 *      7:  50 KHz
 *      8:  10 KHz
 *      9   5 KHz
 *     10   1 KHz
 *     11:  500 Hz
 *     12:  100 Hz
 *     13:  50 Hz
 *     14:  10 Hz
 *     15:  5 Hz
 *
 *  Resources:
 *    counts    - low and high counts for each input.  A zero means there were
 *                not three transitions before the 16 bit counter overflowed.
 *    clock_rate - Clock rate for the 16 bit timer.  A full period of the
 *                input signal must occur with the time of clock_rate * 2^^16.
 */

/*
 * Copyright:   Copyright (C) 2014-2019 Demand Peripherals, Inc.
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
        // PWMIN4 register definitions
#define PWMIN4_REG_COUNT0   0x00
#define CLKSRC_REG          0x30
        // resource names and numbers
#define FN_COUNTS           "counts"
#define FN_FREQ             "clock_rate"
#define RSC_COUNTS          0
#define RSC_FREQ            1
#define NPWMPINS            4
#define NPWMEDGES           (NPWMPINS *3)  /* 3 edges per cycle */
#define PLEN                150   /* maximum output line length */
        // helps walk the returned data to get the times
#define STATE_BEFORE        1
#define STATE_IN            2
#define STATE_DONE          3


/**************************************************************
 *  - Data structures
 **************************************************************/
    // All state info for an instance of an pwmin4
typedef struct
{
    void    *pslot;    // handle to peripheral's slot info
    int      freq;     // clock frequency in Hertz
    void    *ptimer;   // timer to watch for dropped ACK packets
} PWMIN4DEV;


/**************************************************************
 *  - Function prototypes
 **************************************************************/
static void packet_hdlr(SLOT *, DP_PKT *, int);
static void userclksrc(int, int, char*, SLOT*, int, int*, char*);
static void noAck(void *, PWMIN4DEV *);
static void sendconfigtofpga(PWMIN4DEV *, int *plen, char *buf);
static void gethighlow(int, int [], int [], int [], int []);
extern int  dpi_tx_pkt(CORE *pcore, DP_PKT *inpkt, int len);


/**************************************************************
 * Initialize():  - Allocate our permanent storage and set up
 * the read/write callbacks.
 **************************************************************/
int Initialize(
    SLOT *pslot)       // points to the SLOT for this peripheral
{
    PWMIN4DEV *pctx;    // our local device context

    // Allocate memory for this peripheral
    pctx = (PWMIN4DEV *) malloc(sizeof(PWMIN4DEV));
    if (pctx == (PWMIN4DEV *) 0) {
        // Malloc failure this early?
        dplog("memory allocation failure in pwmin4 initialization");
        return (-1);
    }

    // Init our PWMIN4DEV structure
    pctx->pslot = pslot;       // our instance of a peripheral
    pctx->freq = 0;            // default value matches power up default==off

    // Register this slot's packet handler and private data
    (pslot->pcore)->pcb  = packet_hdlr;
    pslot->priv = pctx;

    // Add the handlers for the user visible resources
    pslot->rsc[RSC_COUNTS].name = FN_COUNTS;
    pslot->rsc[RSC_COUNTS].flags = CAN_BROADCAST;
    pslot->rsc[RSC_COUNTS].bkey = 0;
    pslot->rsc[RSC_COUNTS].pgscb = 0;
    pslot->rsc[RSC_COUNTS].uilock = -1;
    pslot->rsc[RSC_COUNTS].slot = pslot;
    pslot->rsc[RSC_FREQ].name = FN_FREQ;
    pslot->rsc[RSC_FREQ].flags = IS_READABLE | IS_WRITABLE;
    pslot->rsc[RSC_FREQ].bkey = 0;
    pslot->rsc[RSC_FREQ].pgscb = userclksrc;
    pslot->rsc[RSC_FREQ].uilock = -1;
    pslot->rsc[RSC_FREQ].slot = pslot;
    pslot->name = "pwmin4";
    pslot->desc = "Quad PWM input";
    pslot->help = README;


    // Send the value, direction and interrupt setting to the card.
    // Ignore return value since there's no user connection and
    // system errors are sent to the logger.
    sendconfigtofpga(pctx, (int *) 0, (char *) 0);  // send pins, dir, intr

    return (0);
}

/**************************************************************
 * packet_hdlr():  - Handle incoming packets from the FPGA board
 **************************************************************/
static void packet_hdlr(
    SLOT    *pslot,    // handle for our slot's internal info
    DP_PKT  *pkt,      // the received packet
    int      len)      // number of bytes in the received packet
{
    PWMIN4DEV *pctx;   // our local info
    RSC      *prsc;    // pointer to this slot's counts resource
    char      pstr[PLEN];  // eight 5 digit ints
    int       plen;    // length of PWM period strings
    int       nedges;  // number of valid edges recorded in packet
    int       i;       // loop counter for nedges
    int       interval[NPWMEDGES+1];  // number of clock cycles in interval
    int       pinval[NPWMEDGES+1]; // pin values at start of interval
    int       hightime[NPWMPINS];  // number of clock cycles the pin was high
    int       lowtime[NPWMPINS];   // numver of clock cycles the pin was low


    pctx = (PWMIN4DEV *)(pslot->priv);  // Our "private" data is a GPIO4DEV
    prsc = &(pslot->rsc[RSC_COUNTS]);

    // Clear the timer on write response packets
    if ((pkt->cmd & DP_CMD_OP_MASK) == DP_CMD_OP_WRITE) {
        del_timer(pctx->ptimer);  // Got the ACK
        pctx->ptimer = 0;
        return;
    }

    // Do a sanity check on the received packet.  Only reads from
    // the counters should come in since we don't ever read the period
    // (twenty-four 16 bit numbers plus edge count takes _49_ bytes.)
    if ((pkt->reg != PWMIN4_REG_COUNT0) || (pkt->count != 49)) {
        dplog("invalid pwmin4 packet from board to host");
        return;
    }

    // Process of elimination makes this an autosend packet.
    // The packet seems good.  Build a table of times and pin values.
    // First get the number of edges the FPGA found from high nibble
    nedges =  pkt->data[CLKSRC_REG] >> 4;
    for (i = 0; i < nedges; i++) {
        interval[i+1] = (256 * pkt->data[i*4]) + pkt->data[i*4 + 1];
        pinval[i+1]   = pkt->data[i*4 + 2];  // four bytes per edge.
    }
    // The first pin value is in the high four bit of the last reported sample.
    pinval[0] = pinval[nedges] >> 4;
    interval[0] = 0;

    // scan the intervals and values to get the high and low times
    gethighlow(nedges, interval, pinval, hightime, lowtime);

    // Print results to anyone listening
    plen = snprintf(pstr, PLEN, "%d %d %d %d %d %d %d %d\n", lowtime[0], hightime[0],
             lowtime[1], hightime[1], lowtime[2], hightime[2], lowtime[3], hightime[3]);

    // Broadcast it if any UI are monitoring it.
    if (prsc->bkey != 0) {
        // bkey will return cleared if UIs are no longer monitoring us
        bcst_ui(pstr, plen, &(prsc->bkey));
        return;
    }

    return;
}


/**************************************************************
 * gethighlow():  - convert edges and intervals to high/low times
 * The interval[] and pinval[] arrays should have the number of
 * edges plus one elements.  The first element is the initial
 * value of the pins with a zero interval.
 **************************************************************/
static void gethighlow(int nedges, int interval[], int pinval[],
                       int hightime[], int lowtime[])
{
    int       i,j;            // loop counter for edges and pins
    int       statelow;       // is STATE_BEFORE, STATE_IN, or STATE_DONE
    int       statehigh;      // 

    // Scan the table by pin number to get the low and high times
    for (j = 0; j < NPWMPINS; j++) {
        // no accumulated high or low times to start
        hightime[j] = 0;  // number of clock cycles the pin was high
        lowtime[j] = 0;   // numver of clock cycles the pin was low

        // to measure the low time we go from the falling edge to
        // rising edge.  Bound to have both if given 3 edges.
        statelow = STATE_BEFORE;
        statehigh = STATE_BEFORE;

        for (i = 1; i < nedges+1; i++) {
            // update low or high times during low or high period
            if (statelow == STATE_IN)
                lowtime[j] = lowtime[j] + interval[i];
            else if (statehigh == STATE_IN)
                hightime[j] = hightime[j] + interval[i];

            // manage the low period measurement state
            if ((statelow == STATE_BEFORE) &&
                (((pinval[i-1]   >> j) & 0x1) == 1) &&
                (((pinval[i] >> j) & 0x1) == 0)) { // at falling edge
                statelow = STATE_IN;
            }
            else if ((statelow == STATE_IN) &&
                (((pinval[i-1]   >> j) & 0x1) == 0) &&
                (((pinval[i] >> j) & 0x1) == 1)) { // at rising edge
                statelow = STATE_DONE;
            }
            // manage the high period measurement state
            if ((statehigh == STATE_BEFORE) &&
                (((pinval[i-1]   >> j) & 0x1) == 0) &&
                (((pinval[i] >> j) & 0x1) == 1)) { // at rising edge
                statehigh = STATE_IN;
            }
            else if ((statehigh == STATE_IN) &&
                (((pinval[i-1]   >> j) & 0x1) == 1) &&
                (((pinval[i] >> j) & 0x1) == 0)) { // at falling edge
                statehigh = STATE_DONE;
            }
            // Done when we have both low and high times
            if ((statelow == STATE_DONE) && (statehigh == STATE_DONE)) {
                break;
            }
        }
    }

    return;
}


/**************************************************************
 * userclksrc():  - The user is reading or setting the sample period
 **************************************************************/
static void userclksrc(
    int      cmd,      //==DPGET if a read, ==DPSET on write
    int      rscid,    // ID of resource being accessed
    char    *val,      // new value for the resource
    SLOT    *pslot,    // pointer to slot info.
    int      cn,       // Index into UI table for requesting conn
    int     *plen,     // size of buf on input, #char in buf on output
    char    *buf)
{
    PWMIN4DEV *pctx;   // our local info
    int      ret;      // return count
    int      newfreq;  // new value for the clock 

    pctx = (PWMIN4DEV *) pslot->priv;

    if (cmd == DPGET) {
        ret = snprintf(buf, *plen, "%d\n", pctx->freq);
        *plen = ret;  // (errors are handled in calling routine)
        return;
    }
    else if ((cmd == DPSET) && (rscid == RSC_FREQ)) {
        ret = sscanf(val, "%d", &newfreq);
        // frequency must be one of the valid values
        if ((ret != 1) || 
            ((newfreq != 20000000) && (newfreq != 10000000) &&
             (newfreq != 5000000)  && (newfreq != 1000000) &&
             (newfreq != 500000)   && (newfreq != 100000) &&
             (newfreq != 50000)    && (newfreq != 10000) &&
             (newfreq != 5000)     && (newfreq != 1000) &&
             (newfreq != 500)      && (newfreq != 100) &&
             (newfreq != 50)       && (newfreq != 10) &&
             (newfreq != 5)        && (newfreq != 0)))
        {
            ret = snprintf(buf, *plen,  E_BDVAL, pslot->rsc[rscid].name);
            *plen = ret;
            return;
        }
        // Valid new frequency
        pctx->freq = newfreq;
        sendconfigtofpga(pctx, plen, buf);
    }

    return;
}


/**************************************************************
 * sendconfigtofpga():  - Send sample period to the FPGA card. 
 * Put error messages into buf and update plen.
 **************************************************************/
static void sendconfigtofpga(
    PWMIN4DEV *pctx,    // This peripheral's context
    int      *plen,    // size of buf on input, #char in buf on output
    char     *buf)     // where to store user visible error messages
{
    DP_PKT   pkt;      // send write and read cmds to the pwmin4
    SLOT    *pslot;    // This peripheral's slot info
    CORE    *pmycore;  // FPGA peripheral info
    int      txret;    // ==0 if the packet went out OK
    int      ret;      // generic return value

    pslot = pctx->pslot;
    pmycore = pslot->pcore;

    // Write the values for the pins, direction, and interrupt mask
    // down to the card.
    pkt.cmd = DP_CMD_OP_WRITE | DP_CMD_AUTOINC;
    pkt.core = pmycore->core_id;
    pkt.reg = CLKSRC_REG;
    pkt.count = 1;
    // map frequency to the four bit constant
    pkt.data[0] =
        (pctx->freq == 20000000) ? 1 :
        (pctx->freq == 10000000) ? 2 :
        (pctx->freq ==  5000000) ? 3 :
        (pctx->freq ==  1000000) ? 4 :
        (pctx->freq ==   500000) ? 5 :
        (pctx->freq ==   100000) ? 6 :
        (pctx->freq ==    50000) ? 7 :
        (pctx->freq ==    10000) ? 8 :
        (pctx->freq ==     5000) ? 9 :
        (pctx->freq ==     1000) ? 10 :
        (pctx->freq ==      500) ? 11 :
        (pctx->freq ==      100) ? 12 :
        (pctx->freq ==       50) ? 13 :
        (pctx->freq ==       10) ? 14 :
        (pctx->freq ==        5) ? 15 :
                                   0;  // zero turns the clock off

    txret = dpi_tx_pkt(pmycore, &pkt, 4 + pkt.count); // 4 header + data
    if (txret != 0) {
        // the send of the new pin values did not succeed.  This
        // probably means the input buffer to the USB port is full.
        // Tell the user of the problem.
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
 * noAck():  Wrote to the board but did not get a reply.  Handle
 * the timeout for this.
 **************************************************************/
static void noAck(
    void     *timer,   // handle of the timer that expired
    PWMIN4DEV *pctx)    // Send pin values of this pwmin4 to the FPGA
{
    // Log the missing ack
    dplog(E_NOACK);

    return;
}

// end of pwmin4.c
