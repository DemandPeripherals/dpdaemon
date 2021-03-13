/*
 *  Name: hostserial.c
 *
 *  Description: Driver for the serial host interface.  The FPGA Tx
 *        line _to_ the host is on the first FPGA pin.  The Rx line
 *        for data _from_ the host is on the second FPGA pin.  The
 *        third and fourth FPGA pins are unused or are debug values.
 *           The system default to using the USB interface.  To use
 *        the serial interface set it to enabled with the correct
 *        baud rate.  Only one interface, USB or serial, can be enabled
 *        at a time.
 *
 *  Hardware Registers:
 *    0:  The baud rate and the enabled flag.  The low two bit are the
 *        baud rate as:  
 *                      00   460800
 *                      01   230400
 *                      10   153600
 *                      11   115200
 *        Bit 2 is the enable flag.  Setting it to one enables the serial
 *        interface and disables the USB interface.  Clearing it enables
 *        the USB interface.
 *           While not directly user visible, an autosend packet from the
 *        board is sent when there is a buffer overflow.  This is a serious
 *        error as it indicates that the link is overloaded and data is
 *        being discarded.  An error log message is generated on overflow.
 * 
 *  Resources:
 *    config - space separate baud rate and enabled flag.
 *              dpset hostserial config 115200 e
 *
 * Copyright:   Copyright (C) 2015-2021 Demand Peripherals, Inc.
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
        // HSR register definitions
#define HSR_REG_CONFIG      0x00
        // max line length from user messages and input
#define MAX_LINE_LEN        100
        // Resource index numbers
#define RSC_CONFIG          0


/**************************************************************
 *  - Data structures
 **************************************************************/
    // All state info for an instance of an hsr
typedef struct
{
    void    *pslot;    // handle to peripheral's slot info
    void    *ptimer;   // timer to watch for dropped ACK packets
    int      baud;     // two bit value encoded as described above
    int      enabled;  // ==1 if the serial interface is enabled
} HSRDEV;


/**************************************************************
 *  - Function prototypes
 **************************************************************/
static void packet_hdlr(SLOT *, DP_PKT *, int);
static void userconfig(int, int, char*, SLOT*, int, int*, char*);
static int  tofpga(HSRDEV *);
static void noAck(void *, HSRDEV *);
extern int  dpi_tx_pkt(CORE *pcore, DP_PKT *inpkt, int len);


/**************************************************************
 * Initialize():  - Allocate our permanent storage and set up
 * the read/write callbacks.
 **************************************************************/
int Initialize(
    SLOT *pslot)       // points to the SLOT for this peripheral
{
    HSRDEV *pctx;      // our local device context

    // Allocate memory for this peripheral
    pctx = (HSRDEV *) malloc(sizeof(HSRDEV));
    if (pctx == (HSRDEV *) 0) {
        // Malloc failure this early?
        dplog("memory allocation failure in hostserial initialization");
        return (-1);
    }

    // Init our HSRDEV structure
    pctx->pslot = pslot;       // our instance of a peripheral
    pctx->baud  = 3;           // Matches Verilog default value (115200)
    pctx->enabled = 0;         // Matches Verilog default (disabled)


    // Register this slot's packet handler and private data
    (pslot->pcore)->pcb  = packet_hdlr;
    pslot->priv = pctx;

    // Add handlers for user visible resources
    pslot->rsc[RSC_CONFIG].name = "config";
    pslot->rsc[RSC_CONFIG].flags = IS_READABLE | IS_WRITABLE;
    pslot->rsc[RSC_CONFIG].bkey = 0;
    pslot->rsc[RSC_CONFIG].pgscb = userconfig;
    pslot->rsc[RSC_CONFIG].uilock = -1;
    pslot->rsc[RSC_CONFIG].slot = pslot;
    pslot->name = "hostserial";
    pslot->desc = "Serial host interface";
    pslot->help = README;

    (void) tofpga(pctx);    // init the interrupt register

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
    HSRDEV *pctx;      // our local info

    pctx = (HSRDEV *)(pslot->priv);  // Our "private" data is a HSRDEV

    // Clear the timer on a write response packet
    if ((pkt->cmd & DP_CMD_OP_MASK) == DP_CMD_OP_WRITE) {
        if (pctx->ptimer) {
            del_timer(pctx->ptimer);  //Got the ACK
            pctx->ptimer = 0;
        }
        return;
    }

    // Should be an autosend buffer overflow message
    if (((pkt->cmd & DP_CMD_OP_MASK) == DP_CMD_OP_READ) &&
        (pkt->reg == HSR_REG_CONFIG) && (pkt->count == 1)) {
        dplog("Host Serial Buffer Overflow Error");
    }
    else          // Sanity check: error if none of the above
        dplog("invalid hostserial packet from board to host");

    return;
}


/**************************************************************
 * userconfig():  - The user is setting the baud rate or the
 * enable flag.
 **************************************************************/
static void userconfig(
    int      cmd,      //==DPGET if a read, ==DPSET on write
    int      rscid,    // ID of resource being accessed
    char    *val,      // new value for the resource
    SLOT    *pslot,    // pointer to slot info.
    int      cn,       // Index into UI table for requesting conn
    int     *plen,     // size of buf on input, #char in buf on output
    char    *buf)
{
    HSRDEV  *pctx;     // our local info
    int      newbaud;  // new baud rate from user
    char     newenabled; // new enabled status from user
    int      ret;      // return count
    int      txret;    // send status


    pctx = (HSRDEV *) pslot->priv;

    if (cmd == DPGET) {
        ret = snprintf(buf, *plen, "%s %c\n",
             ((pctx->baud == 0) ? "460800" :
              (pctx->baud == 1) ? "230400" :
              (pctx->baud == 0) ? "153600" : "115200"),
              (pctx->enabled == 1) ? 'e' : 'd');
        *plen = ret;  // (errors are handled in calling routine)
        return;
    }

    // User is updating the configuration
    ret = sscanf(val, "%d %c", &newbaud, &newenabled);
    if ((ret != 2) ||
        ((newenabled != 'e') && (newenabled != 'd')) ||
        ((newbaud != 460800) && (newbaud != 230400) &&
         (newbaud != 153600) && (newbaud != 115200)))
    {
        ret = snprintf(buf, *plen,  E_BDVAL, pslot->rsc[rscid].name);
        *plen = ret;
        return;
    }
    pctx->baud = (newbaud == 460800) ? 0 :
                 (newbaud == 230400) ? 1 :
                 (newbaud == 153600) ? 2 : 3;
    pctx->enabled = (newenabled == 'e') ? 1 : 0;

    // Got a new value for the config.  Send down to the card.
    txret = tofpga(pctx);
    if (txret != 0) {
        // the send of the new config did not succeed.  This
        // probably means the input buffer to the USB port is full.
        // Tell the user of the problem.
        ret = snprintf(buf, *plen, E_WRFPGA);
        *plen = ret;  // (errors are handled in calling routine)
        return;
    }

    return;
}


/**************************************************************
 * tofpga():  Send config down to the FPGA 
 **************************************************************/
int tofpga(
    HSRDEV *pctx)      // Send config to this peripheral
{
    int      txret;    // ==0 if the packet went out OK
    DP_PKT   pkt;      // send write and read cmds to the hsr
    SLOT    *pmyslot;  // This peripheral's slot info
    CORE    *pmycore;  // FPGA peripheral info

    pmyslot = pctx->pslot;
    pmycore = pmyslot->pcore;

    // Build and send the write command to set the interrupt config
    pkt.cmd = DP_CMD_OP_WRITE | DP_CMD_NOAUTOINC;
    pkt.core = pmycore->core_id;
    pkt.reg = HSR_REG_CONFIG;
    pkt.count = 1;
    pkt.data[0] = (pctx->enabled << 2) | pctx->baud ;
    txret = dpi_tx_pkt(pmycore, &pkt, 4 + pkt.count); // 4 header + 1 data

    // Start timer to look for a write response.
    if ((txret ==0) && (pctx->ptimer == 0)) {
        pctx->ptimer = add_timer(DP_ONESHOT, 100, noAck, (void *) pctx);
    }

    return(txret);
}


/**************************************************************
 * noAck():  Wrote to the board but did not get a reply.  Handle
 * the timeout for this.
 **************************************************************/
static void noAck(
    void   *timer,   // handle of the timer that expired
    HSRDEV *pctx)    // No response from this peripheral
{
    // Log the missing ack
    dplog(E_NOACK);

    return;
}
// end of hostserial.c
