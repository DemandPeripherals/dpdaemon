/*
 *  Name: ws2812.c
 *
 *  Description: Device driver for the ws28 peripheral
 *
 *  Hardware Registers:
 *    0: LED0      - LED string #1 hex data -- use noautoinc when writing
 *    1: LED1      - LED string #2 hex data
 *    2: LED2      - LED string #3 hex data
 *    3: LED3      - LED string #4 hex data
 *    3: config    - set LSB to 1 to invert outputs
 * 
 *  Resources:
 *    led : Which string and the hex value to write to that
 *    LED string.  Use three bytes per LED for RGB and four
 *    bytes per LED for RGBW LEDs.  The first parameter is
 *    which of the four LED strings to address and the second
 *    parameter is a sequence of hex characters to write to
 *    the string.  The number of hex characters must be even
 *    since LEDs have 3 or 4 bytes of LED data.
 *    config : A 1 to invert the outputs.  Default is 0.
 *
 * Copyright:   Copyright (C) 2014-2020 Demand Peripherals, Inc.
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
        // WS28 register definitions
#define WS28_REG_LED0       0x00
#define WS28_REG_CONF       0x04
#define FN_LED              "led"
#define FN_CONF             "config"
        // Resource index numbers
#define RSC_LED             0
#define RSC_CONF            1
        // Max data payload size
#define MXDAT               256


/**************************************************************
 *  - Data structures
 **************************************************************/
    // All state info for an instance of an ws28
typedef struct
{
    void    *pslot;    // handle to peripheral's slot info
    void    *ptimer;   // timer to watch for dropped ACK packets
    uint8_t  leddata[MXDAT];  // date to send in packet
    int      string;   // which string (0-3) of last send
    int      count;    // number of bytes in leddata
    int      invertoutput;  // set to invert output at FPGA
} WS28DEV;


/**************************************************************
 *  - Function prototypes
 **************************************************************/
static void packet_hdlr(SLOT *, DP_PKT *, int);
static void ws28user(int, int, char*, SLOT*, int, int*, char*);
static void noAck(void *, WS28DEV *);
static int  ws28tofpga(WS28DEV *);
static void sendconfigtofpga(WS28DEV *, int *plen, char *buf);
extern int  dpi_tx_pkt(CORE *pcore, DP_PKT *inpkt, int len);


/**************************************************************
 * Initialize():  - Allocate our permanent storage and set up
 * the read/write callbacks.
 **************************************************************/
int Initialize(
    SLOT *pslot)       // points to the SLOT for this peripheral
{
    WS28DEV *pctx;    // our local device context

    // Allocate memory for this peripheral
    pctx = (WS28DEV *) malloc(sizeof(WS28DEV));
    if (pctx == (WS28DEV *) 0) {
        // Malloc failure this early?
        dplog("memory allocation failure in ws28 initialization");
        return (-1);
    }

    // Init our WS28DEV structure
    pctx->pslot = pslot;       // our instance of a peripheral
    pctx->count = 0;           // no bytes to send yet
    pctx->ptimer = 0;          // set while waiting for a response
    pctx->invertoutput = 0;    // hardware default is no inversion


    // Register this slot's packet handler and private data
    (pslot->pcore)->pcb  = packet_hdlr;
    pslot->priv = pctx;

    // Add the handlers for the user visible resources
    pslot->rsc[RSC_LED].name = "led";
    pslot->rsc[RSC_LED].flags = IS_WRITABLE;
    pslot->rsc[RSC_LED].bkey = 0;
    pslot->rsc[RSC_LED].pgscb = ws28user;
    pslot->rsc[RSC_LED].uilock = -1;
    pslot->rsc[RSC_LED].slot = pslot;
    pslot->rsc[RSC_CONF].name = FN_CONF;
    pslot->rsc[RSC_CONF].flags = IS_READABLE | IS_WRITABLE;
    pslot->rsc[RSC_CONF].bkey = 0;
    pslot->rsc[RSC_CONF].pgscb = ws28user;
    pslot->rsc[RSC_CONF].uilock = -1;
    pslot->rsc[RSC_CONF].slot = pslot;
    pslot->name = "ws28";
    pslot->desc = "Quad WS2812 LED driver";
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
    WS28DEV *pctx;    // our local info

    pctx = (WS28DEV *)(pslot->priv);  // Our "private" data is a WS28DEV

    // Clear the timer on write response packets
    if ((pkt->cmd & DP_CMD_OP_MASK) == DP_CMD_OP_WRITE) {
        if (pctx->ptimer) {
            del_timer(pctx->ptimer);  //Got the ACK
            pctx->ptimer = 0;
        }
        return;
    }

    // Do a sanity check on the received packet.
    if ((pkt->reg != pctx->string) || (pkt->count != pctx->count)) {
        dplog("invalid ws28 packet from board to host");
        return;
    }

    return;
}


/**************************************************************
 * ws28user():  - The user is sending RGB(W) data to a string
 * of ws2812/sk6812 LEDs.
 **************************************************************/
static void ws28user(
    int      cmd,      //==DPGET if a read, ==DPSET on write
    int      rscid,    // ID of resource being accessed
    char    *val,      // new value for the resource
    SLOT    *pslot,    // pointer to slot info.
    int      cn,       // Index into UI table for requesting conn
    int     *plen,     // size of buf on input, #char in buf on output
    char    *buf)
{
    WS28DEV *pctx;     // our local info
    int      ret;      // return count
    int      strid;    // which LED string to send to
    int      txret;    // ==0 if the packet went out OK
    int      len;      // length of hex string in input
    int      i;        // hex value loop counter
    int      hidx;     // index into array of hex values to send
    char     c;        // hex character to convert
    uint8_t  hex;
    int      invout;   // set == 1 if we're to invert the FPGA output


    pctx = (WS28DEV *) pslot->priv;

    // Look for a config command to set or get the invert output config
    if ((cmd == DPGET) && (rscid == RSC_CONF)) {
        // value of 1 or 0
        ret = snprintf(buf, *plen, "%d\n", pctx->invertoutput);
        *plen = ret;  // (errors are handled in calling routine)
        return;
    }
    else if ((cmd == DPSET) && (rscid == RSC_CONF)) {
        // value of 1 or 0
        ret = sscanf(val, "%d", &invout);
        if ((ret != 1) || (invout < 0) || (invout > 1)) {
            ret = snprintf(buf, *plen, E_BDVAL, pslot->rsc[rscid].name);
            *plen = ret;
            return;
        }

        pctx->invertoutput = invout;
        sendconfigtofpga(pctx, plen, buf);  // send down new config
        return;
    }

    // only command still available is dpset on an LED.  Get LED string
    // ID and bytes to send.  Format of val should be something like
    // "2 aabbcc112233".  Scan the first character, skip white space,
    // then get the hex values.
    len = strlen(val);

    // get and check string id.  Must be 1 to 4.
    strid = val[0] - '0';
    if ((strid < 1) || (strid > 4)) {
        ret = snprintf(buf, *plen,  E_BDVAL, pslot->rsc[rscid].name);
        *plen = ret;
        return;
    }
    pctx->string = strid;

    // skip white space
    for (i = 1; i < len; i++) {
        if (val[i] != ' ')
            break;
    }
    if (i == len) {
        // getting here means the input line had a string id but no hex values
        ret = snprintf(buf, *plen,  E_BDVAL, pslot->rsc[rscid].name);
        *plen = ret;
        return;
    }

    // Convert string of hex character into up to 256 bytes of data to send
    hidx = 0;      // init index into array of hex values
    for ( ; i < len; i++) {
        c = val[i++];
        hex = ((c <= '9') && (c >= '0')) ? (c - '0') :
              ((c <= 'f') && (c >= 'a')) ? (10 + (c - 'a')): -1;
        if ((hex == -1) || (i == len))
            break;
        pctx->leddata[hidx] = hex << 4;
        c = val[i];
        hex = ((c <= '9') && (c >= '0')) ? (c - '0') :
              ((c <= 'f') && (c >= 'a')) ? (10 + (c - 'a')): -1;
        if (hex == -1)
            break;
        pctx->leddata[hidx] += hex;
        hidx++;
        if (hidx >= MXDAT)
            break;
    }
    if (i != len) {
        // to get here we must have had an illegal char in hex string
        ret = snprintf(buf, *plen,  E_BDVAL, pslot->rsc[rscid].name);
        *plen = ret;
        return;
    }

    // hex byte count is hidx
    pctx->count = hidx;

    txret =  ws28tofpga(pctx);   // This peripheral's context
    if (txret != 0) {
        // the send of the RGB data did not succeed.  This probably
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
 * ws28tofpga():  - Send hex RGB data to the FPGA card.  Return
 * zero on success
 **************************************************************/
static int ws28tofpga(
    WS28DEV *pctx)    // This peripheral's context
{
    DP_PKT   pkt;      // send write and read cmds to the ws28
    SLOT    *pmyslot;  // This peripheral's slot info
    CORE    *pmycore;  // FPGA peripheral info
    int      i;        // loop counter to copy RGB hex data
    int      txret;    // ==0 if the packet went out OK

    pmyslot = pctx->pslot;
    pmycore = pmyslot->pcore;

    // Got a new value for the outputs.  Send down to the card.
    // Build and send the write command to set the ws28.
    pkt.cmd = DP_CMD_OP_WRITE | DP_CMD_NOAUTOINC;
    pkt.core = pmycore->core_id;
    // LED strings are number 1 to 4 but the addresses are 0 to 3
    pkt.reg = pctx->string - 1;
    pkt.count = pctx->count % 256;  // send up to 256 bytes
    for (i = 0; i < pctx->count; i++) {
        pkt.data[i] = pctx->leddata[i];
    }
    txret = dpi_tx_pkt(pmycore, &pkt, 4 + pkt.count); // 4 header + data

    return(txret);
}


/**************************************************************
 * sendconfigtofpga():  - Send invertoutput status to the FPGA. 
 * Put error messages into buf and update plen.
 **************************************************************/
static void sendconfigtofpga(
    WS28DEV  *pctx,    // This peripheral's context
    int      *plen,    // size of buf on input, #char in buf on output
    char     *buf)     // where to store user visible error messages
{
    DP_PKT   pkt;      // send write and read cmds to the qtr
    SLOT    *pslot;    // This peripheral's slot info
    CORE    *pmycore;  // FPGA peripheral info
    int      txret;    // ==0 if the packet went out OK
    int      ret;      // generic return value

    pslot = pctx->pslot;
    pmycore = pslot->pcore;

    // Write the value the invertoutput in reg #4 bit #0
    pkt.cmd = DP_CMD_OP_WRITE | DP_CMD_AUTOINC;
    pkt.core = pmycore->core_id;
    pkt.reg = WS28_REG_CONF;      // send invertoutput
    pkt.count = 1;
    pkt.data[0] = pctx->invertoutput;
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
    void    *timer,   // handle of the timer that expired
    WS28DEV *pctx)    // points to instance of this peripheral
{
    // Log the missing ack
    dplog(E_NOACK);

    return;
}
// end of ws28.c
