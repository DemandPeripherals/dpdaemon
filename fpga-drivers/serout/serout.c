/*
 *  Name: serout.c
 *
 *  Description: Driver for the quad/octal serial output peripheral
 *    The serout peripheral provides four or eight channels of low
 *    speed serial output.  All four/eight channels share the same
 *    configuration.
 *
 *  Hardware Registers: 8 bit, read-write
 *      Reg 0:  Serial port 0 FIFO.
 *      Reg 1:  Serial port 1 FIFO.
 *      Reg 2:  Serial port 2 FIFO.
 *      Reg 3:  Serial port 3 FIFO.
 *      Reg 4:  Serial port 4 FIFO, or Config on quad peripheral
 *      Reg 5:  Serial port 5 FIFO.
 *      Reg 6:  Serial port 6 FIFO.
 *      Reg 7:  Serial port 7 FIFO.
 *      Reg 4/8:  Bits 0-3 set the baud rate as follows:
 *                     0: 38400
 *                     1: 19200
 *                     3: 9600
 *                     7: 4800
 *                     f: 2400
 *                Bits 4-5 set the number of stop bits:
 *                     0: 1 stop bit
 *                     1: 2 stop bits
 *                     2: 3 stop bits
 *                     3: 4 stop bits
 * 
 *  Resources:
 *     config : Baud rate as one of 38400, 19200, 9600, 4800, or
 *        2400, followed by the number of stop bits in the range
 *        of 1 to 4.  All generated baud rates are within 0.2
 *        percent of the target rate.  
 *
 *     text : Characters to send to a port.
 *        Specify the port and the ASCII printable character to sen
 *
 *     hex : Characters to send as 8 bit hex values.
 *        Specify the port and the hexadecimal values to send.
 *      
 *
 * Copyright:   Copyright (C) 2021 Demand Peripherals, Inc.
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
#include <stdint.h>
#include <sys/fcntl.h>
#include <sys/types.h>
#include <ctype.h>
#include "daemon.h"
#include "readme.h"


/**************************************************************
 *  - Limits and defines
 **************************************************************/
#ifndef NPORT
#define NPORT              4       // number of ports
#endif

        // SO register definitions
#define SO_FIFO0           0x00    // text/hex to port #0
#define SO_CONFIG          (NPORT) // common configuration
        // resource names and numbers
#define FN_CONFIG          "config"
#define FN_TEXT            "text"
#define FN_HEX             "hex"
#define RSC_CONFIG         0
#define RSC_TEXT           1
#define RSC_HEX            2
        // Size of our local per port FIFO
#define FIFOSZ             256
        // Highest baudrate possible
#define BAUDZERO           38400
        // MS timeout for resending chars on a full peripheral buffer
        // This is for 38400 and is scaled up for lower baud rates to
        // prevent repeatedly trying to add one character to the buffer.
#define SORETRYTIME        8
        // FPGA FIFO size is set by `LB2BUFSZ in the .v file.  A value
        // 5 means that the FPGA fifo has 32 bytes.  We use this to limit
        // how full the packets we send to the FPGA.  Change this value
        // if you change `LB2BUFSZ.
#define FIFOBUFSZ          32


/**************************************************************
 *  - Data structures
 **************************************************************/
typedef struct
{
    uint8_t  data[FIFOSZ]; // data queued to send
    int      nxwrt;        // index of next char to write
                           // full when nxwrt+1 == nxrd
    int      nxrd;         // index of next char to read
                           // empty when nxrd == nxwrt
    int      intransit;    // waiting for write response if set
} SOFIFO;

typedef struct
{
    void    *pslot;        // handle to peripheral's slot info
    void    *ptimer;       // timer to watch for dropped ACK packets
    void    *pxmittmr;     // retransmit timer if FIFO not empty
    int      baud;         // baud rate as 0,1,3,7,f
    int      nstop;        // number of stop bits (1-4)
    SOFIFO   fifo[NPORT];  // FIFO with read/write indicies
} SODEV;


/**************************************************************
 *  - Function prototypes and static definitions
 **************************************************************/
static void packet_hdlr(SLOT *, DP_PKT *, int);
static void user_hdlr(int, int, char*, SLOT*, int, int*, char*);
static void noAck(void *timer, SODEV *);
extern int  dpi_tx_pkt(CORE *pcore, DP_PKT *inpkt, int len);
static void sendconfigfpga(SODEV  *);
static void serxmit(void *timer, SODEV *);
static void noAck(void *timer, SODEV *);


/**************************************************************
 * Initialize():  - Allocate our permanent storage and set up
 * the read/write callbacks.
 **************************************************************/
int Initialize(
    SLOT *pslot)       // points to the SLOT for this peripheral
{
    SODEV *pctx;       // our local device context
    int    fidx;       // fifo index

    // Allocate memory for this peripheral
    pctx = (SODEV *) malloc(sizeof(SODEV));
    if (pctx == (SODEV *) 0) {
        // Malloc failure this early?
        dplog("memory allocation failure in serout initialization");
        return (-1);
    }

    // Init our SODEV structure
    pctx->pslot = pslot;       // our instance of a peripheral
    pctx->ptimer = 0;          // set while waiting for a response
    pctx->baud = 0;            // 38400
    pctx->nstop = 1;           // 1 stop bit
    for (fidx = 0; fidx < NPORT; fidx++) {
        pctx->fifo[fidx].nxrd = 0;            // init read pointer
        pctx->fifo[fidx].nxwrt = 0;           // init write pointer
        pctx->fifo[fidx].intransit = 0;       // nothing in transit to start
    }

    // Register this slot's packet handler and private data
    (pslot->pcore)->pcb  = packet_hdlr;
    pslot->priv = pctx;

    // Add the handlers for the user visible resources
    pslot->rsc[RSC_CONFIG].name = FN_CONFIG;
    pslot->rsc[RSC_CONFIG].flags = IS_READABLE | IS_WRITABLE;
    pslot->rsc[RSC_CONFIG].bkey = 0;
    pslot->rsc[RSC_CONFIG].pgscb = user_hdlr;
    pslot->rsc[RSC_CONFIG].uilock = -1;
    pslot->rsc[RSC_CONFIG].slot = pslot;
    pslot->rsc[RSC_TEXT].name = FN_TEXT;
    pslot->rsc[RSC_TEXT].flags = IS_WRITABLE;
    pslot->rsc[RSC_TEXT].bkey = 0;
    pslot->rsc[RSC_TEXT].pgscb = user_hdlr;
    pslot->rsc[RSC_TEXT].uilock = -1;
    pslot->rsc[RSC_TEXT].slot = pslot;
    pslot->rsc[RSC_HEX].name = FN_HEX;
    pslot->rsc[RSC_HEX].flags = IS_WRITABLE;
    pslot->rsc[RSC_HEX].bkey = 0;
    pslot->rsc[RSC_HEX].pgscb = user_hdlr;
    pslot->rsc[RSC_HEX].uilock = -1;
    pslot->rsc[RSC_HEX].slot = pslot;

#if NPORT == 4
    pslot->name = "serout4";
    pslot->desc = "Quad low speed serial output";
#else
    pslot->name = "serout8";
    pslot->desc = "Octal low speed serial output";
#endif
    pslot->help = README;

    return (0);
}


/**************************************************************
 * packet_hdlr():  - Handle incoming packets from the FPGA board
 **************************************************************/
static void packet_hdlr(
    SLOT    *pslot,    // handle for our slot's internal info
    DP_PKT *pkt,       // the received packet
    int     len)       // number of bytes in the received packet
{
    SODEV  *pctx;      // our local context
    int     fifoidx;   // which fifo is ACKing
    int     scount;    // number of chars ACKed
    int     nchar;     // number of character in the fifo
    int     retryms;   // retry time in milliseconds

    /* We expect one kind of packet from the host:
     * - write response: to acknowledge write to one of the FIFOs.
     *   A write to the FIFO may return indicating that not all
     *   characters were written.  Adjust our local FIFO read pointer
     *   to remove the characters that were sent.  If our FIFO is not
     *   empty, set a timer to schedule another write.
     */

    pctx = (SODEV *)(pslot->priv);  // Our "private" data is a SODEV

    // Clear the packet timer on write response packets
    if ((pctx->ptimer) && ((pkt->cmd & DP_CMD_OP_MASK) == DP_CMD_OP_WRITE)) {
        del_timer(pctx->ptimer);  //Got the ACK
        pctx->ptimer = 0;
    }

    //  We can just return if a write response to a config write
    if (pkt->reg == SO_CONFIG) {
        return;
    }

    // if this is an ACK to a FIFO write then we want to remove the
    // sent characters from the FIFO.  If there are still more chars
    // in the FIFO we also want to set up a retransmit timer.
    fifoidx = pkt->reg;
    if (fifoidx >= NPORT) {  // valid port ID?
        dplog("invalid serout fifo write response from board to host");
        return;
    }

    // clear intransit flag for this fifo
    pctx->fifo[fifoidx].intransit = 0;    // clear intransit flag

    // Adjust the next read pointer in the fifo.
    // Recall that write responses have a "write remaining" byte (data[0])
    // to say how many bytes did not get into the FIFO.
    scount = pkt->count - pkt->data[0];   // num sent - num remaining

    // As a sanity check verity that scount is less than the number of
    // characters currently in the fifo.
    nchar = (pctx->fifo[fifoidx].nxwrt + FIFOSZ - pctx->fifo[fifoidx].nxrd) % FIFOSZ;
    if (scount > nchar) {
        dplog("invalid serout fifo index.  Clearing FIFO on port %d", fifoidx);
        pctx->fifo[fifoidx].nxwrt = 0;
        pctx->fifo[fifoidx].nxrd = 0;
        return;
    }

    // Adjust the read index
    pctx->fifo[fifoidx].nxrd = (pctx->fifo[fifoidx].nxrd + scount) % FIFOSZ;

    // Set a timer if the buffer is not empty
    if ( pctx->fifo[fifoidx].nxrd != pctx->fifo[fifoidx].nxwrt) {
        // only set the timer if one is not already set
        if (pctx->pxmittmr == 0) {
            // slow retry for slow links.  Recall baud is one of 0,1,3,7,f for
            // rates of 38400, 19200, 9600, 4800, and 2400.
            retryms = SORETRYTIME * (pctx->baud + 1);
            pctx->pxmittmr = add_timer(DP_ONESHOT, retryms, serxmit, (void *) pctx);
        }
    }

    return;
}


/**************************************************************
 * user_hdlr():  - The user is reading or setting a resource
 **************************************************************/
static void user_hdlr(
    int      cmd,      //==DPGET if a read, ==DPSET on write
    int      rscid,    // ID of resource being accessed
    char    *val,      // new value for the resource
    SLOT    *pslot,    // pointer to slot info.
    int      cn,       // Index into UI table for requesting conn
    int     *plen,     // size of buf on input, #char in buf on output
    char    *buf)
{
    SODEV   *pctx;     // context for this peripheral instance
    int      ret;      // return count
    int      baud;     // baud rate as an integer
    int      stop;     // number of stop bits
    int      port;     // port number for text and hex
    int      vallen;   // number of characters in val
    int      txtlen;   // number of characters to send
    char    *ptxt;     // points to start of text to send
    int      fifofree; // free space in fifo 'port'
    int      widx;     // copy of fifo write index
    int      i;        // generic loop variables
    int      tmp1;     // used for hex conversion
 

    pctx = (SODEV *) pslot->priv;

    // Sanity check on lenght of input string
    if (cmd == DPSET) {
        vallen = strlen(val);
        if (vallen < 3) {     // minimum # chars in DPSET
            ret = snprintf(buf, *plen,  E_BDVAL, pslot->rsc[rscid].name);
            *plen = ret;
            return;
        }
    }

    // Get/show baud rate and number of stop bits if CONFIG
    if (rscid == RSC_CONFIG) {
        if (cmd == DPGET) {
            baud = BAUDZERO / (pctx->baud + 1);
            ret = snprintf(buf, *plen, "%d %d\n", baud, pctx->nstop);
            *plen = ret;  // (errors are handled in calling routine)
            return;
        }
        else if (cmd == DPSET) {
            ret = sscanf(val, "%d %d", &baud, &stop);
            if ((ret != 2) || (baud < 2400) || (baud > 38400) ||
                (stop < 1) || (stop > 4)) {
                ret = snprintf(buf, *plen,  E_BDVAL, pslot->rsc[rscid].name);
                *plen = ret;
                return;
            }
            pctx->nstop = stop;
            pctx->baud = (BAUDZERO / baud) - 1;
            sendconfigfpga(pctx);      // send our config down to fpga
            return;
        }
    }
    else if (rscid == RSC_TEXT) {
        // get the port number
        ret = sscanf(val, "%d", &port);
        if ((ret != 1) || (port < 0) || (port >= NPORT)) {
            ret = snprintf(buf, *plen,  E_BDVAL, pslot->rsc[rscid].name);
            *plen = ret;
            return;
        }
        // We want to accept the whole message from the user or none of it.
        // So we compute the free space in the fifo for port 'port' and 
        // reject the command if the text won't fit.
        txtlen = vallen - 2;          // ASSUME: 1 char for port, 1 for separator
        ptxt   = val + 2;             // ASSUME: 1 char for port, 1 for separator
        fifofree = ((pctx->fifo[port].nxrd + FIFOSZ - (pctx->fifo[port].nxwrt + 1)) % FIFOSZ);
        if ((txtlen <= 0) || (txtlen > fifofree)) {
            ret = snprintf(buf, *plen,  E_NBUFF, pslot->rsc[rscid].name);
            *plen = ret;
            return;
        }

        // It'll fit in the fifo.  Copy it, updating next write pointer as we go
        for (i = 0; i < txtlen; i++) {
            pctx->fifo[port].data[pctx->fifo[port].nxwrt] = ptxt[i];
            pctx->fifo[port].nxwrt = (pctx->fifo[port].nxwrt + 1) % FIFOSZ;
        }

        // call serxmit to send the data
        serxmit((void *) 0, pctx);
    }
    else if (rscid == RSC_HEX) {
        // get the port number
        ret = sscanf(val, "%d", &port);
        if ((ret != 1) || (port < 0) || (port >= NPORT)) {
            ret = snprintf(buf, *plen,  E_BDVAL, pslot->rsc[rscid].name);
            *plen = ret;
            return;
        }

        // We want to accept the whole message from the user or none of it.
        // We do not know how many character are in the hex string since we
        // don't know how much white space it has.  Make a copy of the write
        // index and add the characters into the fifo while checking to see
        // if the fifo is full.  If all characters fit then set the write
        // index to our working copy.  
        widx = pctx->fifo[port].nxwrt;  
        txtlen = vallen - 2;          // ASSUME: 1 char for port, 1 for separator
        ptxt = val + 2;
        i = 0;         // index into txt
        while (1) {
            // error out if needed
            if (((widx + 1) % FIFOSZ) == pctx->fifo[port].nxrd) {
                ret = snprintf(buf, *plen,  E_NBUFF, pslot->rsc[rscid].name);
                *plen = ret;
                return;
            }
            pctx->fifo[port].data[widx] = 0;  // start at zero
            while (i < txtlen) {
                if (isxdigit(ptxt[i]))
                    break;
                i++;
            }
            // i now points at a hex char
            while (i < txtlen) {
                tmp1 = (isdigit(ptxt[i])) ? (int)(ptxt[i] - '0') : 
                                           (int)(toupper(ptxt[i]) - 'A' + 10);
                // add hex digit to low 4 bits
                pctx->fifo[port].data[widx] = (pctx->fifo[port].data[widx] << 4) + tmp1;
                i++;
                if (! isxdigit(ptxt[i]))
                    break;
            }
            // At this point we have successfully gotten a hex value into fifo.
            // Break out of the loop if needed or get next hex character
            widx++;    // char is added to FIFO.  Increment count
            if (i == txtlen)
                break;
        }
        // To get here means the data could fit into the FIFO.  Update the write pointer
        pctx->fifo[port].nxwrt = widx;

        // call serxmit to send the data
        serxmit((void *) 0, pctx);
    }

    return;
}


/**************************************************************
 * sendconfigfpga():  Send the port configuration down to the FPGA
 **************************************************************/
static void sendconfigfpga(
    SODEV  *pctx)      // our instance of a SODEV
{
    DP_PKT   pkt;      // send character to the serial output peripheral
    SLOT    *pslot;    // This peripheral's slot info
    CORE    *pmycore;  // FPGA peripheral info
    int      txret;    // ==0 if the packet went out OK
    int      pktlen;   // size of outgoing packet

    pslot = pctx->pslot;
    pmycore = pslot->pcore;

    pkt.cmd = DP_CMD_OP_WRITE | DP_CMD_NOAUTOINC;
    pkt.core = (pslot->pcore)->core_id;
    pkt.reg = SO_CONFIG;       // config register
    pkt.count = 1;             // just one config byte
    pktlen = 4 + pkt.count;    // 4 header + data
    // baud in low four bits.  nstop in next two
    // stop bits are offset by 1 so '0' means one stop bit
    pkt.data[0] = (uint8_t) (pctx->baud  | ((pctx->nstop - 1) << 4));

    // Packet is built.  Send it and start an ACK timer if needed.
    txret = dpi_tx_pkt(pmycore, &pkt, pktlen);
    if (txret != 0) {
        // the send of the FIFO characters did not succeed.  This
        // probably means the input buffer to the USB port is full.
        // Tell the user of the problem.
        dplog("Serial config failed to send packet. Link overloaded?");
        return;
    }
    if (pctx->ptimer == 0) {
        pctx->ptimer = add_timer(DP_ONESHOT, 100, noAck, (void *) pctx);
    }

    return;
}



/**************************************************************
 * serxmit():  Scan the FIFOs looking for data to send.  Send
 * full packets if possible.
 **************************************************************/
static void serxmit(
    void   *timer,   // handle of the timer that expired
    SODEV  *pctx)    // the peripheral with a timeout
{
    DP_PKT   pkt;      // send character to the serial output peripheral
    SLOT    *pslot;    // This peripheral's slot info
    CORE    *pmycore;  // FPGA peripheral info
    int      fidx;     // FIFO index
    int      txret;    // ==0 if the packet went out OK
    int      pktlen;   // size of outgoing packet
    int      i;        // loop counter
    int      didx;     // index into fifo data[]

    pslot = pctx->pslot;
    pmycore = pslot->pcore;
    pkt.cmd = DP_CMD_OP_WRITE | DP_CMD_NOAUTOINC;
    pkt.core = (pslot->pcore)->core_id;

    // Cancel the retransmit timer if one is set
    if (pctx->pxmittmr != (void *) 0) {
        del_timer(pctx->pxmittmr);
        pctx->pxmittmr = (void *) 0;
    }

    // Scan through the FIFOs looking for characters to send
    for (fidx = 0; fidx < NPORT; fidx++) {
        // Are there characters to send?
        if (pctx->fifo[fidx].nxrd == pctx->fifo[fidx].nxwrt) {
            continue;        // empty fifo
        }

        // Is a send packet response still outstanding?
        if (pctx->fifo[fidx].intransit == 1) {
            continue;        // data still in transit
        }

        pkt.reg = fidx;      // FIFO index is also the port register number
        pkt.count = (pctx->fifo[fidx].nxwrt + FIFOSZ - pctx->fifo[fidx].nxrd) % FIFOSZ;
        // send the lesser of FIFOBUFSZ or number in our FIFO
        pkt.count = (pkt.count > FIFOBUFSZ) ? FIFOBUFSZ : pkt.count;
        pktlen = 4 + pkt.count;    // 4 header + data
        // load data into the packet
        for (i = 0; i < pkt.count; i++) {
            didx = (pctx->fifo[fidx].nxrd + i) % FIFOSZ;
            pkt.data[i] = pctx->fifo[fidx].data[didx];
        }

        // Packet is built.  Send it and start an ACK timer if needed.
        txret = dpi_tx_pkt(pmycore, &pkt, pktlen);
        if (txret != 0) {
            // the send of the FIFO characters did not succeed.  This
            // probably means the input buffer to the USB port is full.
            // Tell the user of the problem.
            dplog("Serial Out failed to send packet. Link overloaded?");
            return;
        }
        if (pctx->ptimer == 0) {
            pctx->ptimer = add_timer(DP_ONESHOT, 100, noAck, (void *) pctx);
        }

        // Mark FIFO as having data in transit
        pctx->fifo[fidx].intransit = 1;
    }

    return;
}


/**************************************************************
 * noAck():  Wrote to the board but did not get a reply.  Handle
 * the timeout for this.
 **************************************************************/
static void noAck(
    void   *timer,   // handle of the timer that expired
    SODEV  *pctx)    // the peripheral with a timeout
{
    // Log the missing ack
    dplog(E_NOACK);

    return;
}

// end of serout.c
