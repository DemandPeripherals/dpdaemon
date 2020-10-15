/*
 *  Name: qtr4.c
 *
 *  Description: Interface to four Pololu QTR-RC proximity sensors
 *
 *  Hardware Registers:
 *    0  :     - Low four bits have sensor status.  1==dark  0==light
 *    1  :     - Sensitivity.  An 8 bit value of 10us steps to wait before reading
 *    2  :     - Sample period in units of 10ms from 1 to 15.  0 turns sensor off
 * 
 *  Resources:
 *    qtrval   - QTR status as a single hex number - a set bit is black
 *    update_period - update period in ms. _0_ is off. Range is 10 to 150.
 *    sensitivity - 1 to 250 with higher number being more sensitive to white
 */

/*
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

/*
 *    The qtr4 provides four channels of input from a Pololu QTR-RC sensor.
 *    The sensors work by charging a capacitor to Vcc (3.3 volts in our case)
 *    and monitoring the capacitor discharge.  The discharge rate depends on
 *    the amount of IR light reflected off a surface and onto a phototransistor.
 *    A sensor is considered light if the capacitor has discharged below the
 *    logic '1' level of the FPGA and is considered dark if the voltage is
 *    still above the logic 1 level.  Sensitivity is controlled by a delay in
 *    reading the inputs.  The longer the delay, the longer the cap has to
 *    discharge.  Sensitiviy is in units of 10us but since the discharge of
 *    a capacitor is exponential the sensitivity is very non-linear. Values
 *    in the range of 5 to 25 seem to work fairly well.
 *      
 *    The minimum update period is 10 milliseconds and the maximum is 150.
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
        // QTR4 register definitions
#define QTR4_DATA           0x00
#define QTR4_SENS           0x01
#define QTR4_UPDATE         0x02
        // resource names and numbers
#define FN_DATA             "qtrval"
#define FN_SENS             "sensitivity"
#define FN_UPDATE           "update_period"
#define RSC_DATA            0
#define RSC_SENS            1
#define RSC_UPDATE          2


/**************************************************************
 *  - Data structures
 **************************************************************/
    // All state info for an instance of an qtr4
typedef struct
{
    void    *pslot;    // handle to peripheral's slot info
    uint8_t  update;   // update rate for sampling 0=off, 1=10ms, ....
    uint8_t  sensitivity;   // sensitivity in range of 1 to 250
    void    *ptimer;   // timer to watch for dropped ACK packets
} QTR4DEV;


/**************************************************************
 *  - Function prototypes
 **************************************************************/
static void packet_hdlr(SLOT *, DP_PKT *, int);
static void userconfig(int, int, char*, SLOT*, int, int*, char*);
static void noAck(void *, QTR4DEV *);
static void sendconfigtofpga(QTR4DEV *, int *plen, char *buf);


/**************************************************************
 * Initialize():  - Allocate our permanent storage and set up
 * the read/write callbacks.
 **************************************************************/
int Initialize(
    SLOT *pslot)       // points to the SLOT for this peripheral
{
    QTR4DEV *pctx;    // our local device context

    // Allocate memory for this peripheral
    pctx = (QTR4DEV *) malloc(sizeof(QTR4DEV));
    if (pctx == (QTR4DEV *) 0) {
        // Malloc failure this early?
        dplog("memory allocation failure in qtr4 initialization");
        return (-1);
    }

    // Init our QTR4DEV structure
    pctx->pslot = pslot;       // our instance of a peripheral
    pctx->update = 0;          // default value matches power up default==off
    pctx->ptimer = 0;          // set while waiting for a response

    // Register this slot's packet handler and private data
    (pslot->pcore)->pcb  = packet_hdlr;
    pslot->priv = pctx;

    // Add the handlers for the user visible resources
    pslot->rsc[RSC_DATA].name = FN_DATA;
    pslot->rsc[RSC_DATA].flags = CAN_BROADCAST;
    pslot->rsc[RSC_DATA].bkey = 0;
    pslot->rsc[RSC_DATA].pgscb = 0;
    pslot->rsc[RSC_DATA].uilock = -1;
    pslot->rsc[RSC_DATA].slot = pslot;
    pslot->rsc[RSC_SENS].name = FN_SENS;
    pslot->rsc[RSC_SENS].flags = IS_READABLE | IS_WRITABLE;
    pslot->rsc[RSC_SENS].bkey = 0;
    pslot->rsc[RSC_SENS].pgscb = userconfig;
    pslot->rsc[RSC_SENS].uilock = -1;
    pslot->rsc[RSC_SENS].slot = pslot;
    pslot->rsc[RSC_UPDATE].name = FN_UPDATE;
    pslot->rsc[RSC_UPDATE].flags = IS_READABLE | IS_WRITABLE;
    pslot->rsc[RSC_UPDATE].bkey = 0;
    pslot->rsc[RSC_UPDATE].pgscb = userconfig;
    pslot->rsc[RSC_UPDATE].uilock = -1;
    pslot->rsc[RSC_UPDATE].slot = pslot;
    pslot->name = "qtr4";
    pslot->desc = "Quad QTR-RC input";
    pslot->help = README;


    // Send the update rate to the peripheral to turn it off.
    // Ignore return value since there's no user connection and
    // system errors are sent to the logger.
    sendconfigtofpga(pctx, (int *) 0, (char *) 0);

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
    QTR4DEV  *pctx;    // our local info
    RSC      *prsc;    // pointer to this slot's counts resource
    uint8_t   qtrval;  // data from sensor
    char      qstr[10]; // one char and a newline
    int       qlen;    // length of line to send


    pctx = (QTR4DEV *)(pslot->priv);  // Our "private" data
    prsc = &(pslot->rsc[RSC_DATA]);

    // Clear the timer on write response packets
    if ((pkt->cmd & DP_CMD_OP_MASK) == DP_CMD_OP_WRITE) {
        del_timer(pctx->ptimer);  // Got the ACK
        pctx->ptimer = 0;
        return;
    }

    // Do a sanity check on the received packet.  Only reads from
    // the data register should come in since we don't ever read
    // the sensitivity or update period
    // (one 8 bit number takes _1_ byte.)
    if ((pkt->reg != QTR4_DATA) || (pkt->count != 1)) {
        dplog("invalid qtr4 packet from board to host");
        return;
    }

    // Get counts and timestamps
    qtrval = pkt->data[0] & 0x0F;

    // Process of elimination makes this an autosend packet.
    // Broadcast it if any UI are monitoring it.
    if (prsc->bkey != 0) {
        qlen = sprintf(qstr, "%1x\n", qtrval);
        // bkey will return cleared if UIs are no longer monitoring us
        bcst_ui(qstr, qlen, &(prsc->bkey));
        return;
    }

    return;
}


/**************************************************************
 * userconfig():  - The user is reading or setting the configuration
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
    QTR4DEV *pctx;     // our local info
    int      ret;      // return count
    int      newsens;  // new value to assign the sensitivity
    int      newupdate;  // new value to assign the update period

    pctx = (QTR4DEV *) pslot->priv;

    if ((cmd == DPGET) && (rscid == RSC_SENS)) {
        // value of 1 to 250
        ret = snprintf(buf, *plen, "%d\n", pctx->sensitivity);
        *plen = ret;  // (errors are handled in calling routine)
        return;
    }
    else if ((cmd == DPSET) && (rscid == RSC_SENS)) {
        // value of 1 to 250
        ret = sscanf(val, "%d", &newsens);
        if ((ret != 1) || (newsens < 1) || (newsens > 250)) {
            ret = snprintf(buf, *plen, E_BDVAL, pslot->rsc[rscid].name);
            *plen = ret;
            return;
        }

        pctx->sensitivity = newsens;
        sendconfigtofpga(pctx, plen, buf);  // send down new config
    }
    else if ((cmd == DPGET) && (rscid == RSC_UPDATE)) {
        // Update period in milliseconds
        ret = snprintf(buf, *plen, "%d\n", (pctx->update * 10));
        *plen = ret;  // (errors are handled in calling routine)
        return;
    }
    else if ((cmd == DPSET) && (rscid == RSC_UPDATE)) {
        // Update period is 0 to 150 
        ret = sscanf(val, "%d", &newupdate);
        if ((ret != 1) || (newupdate < 0) || (newupdate > 150)) {
            ret = snprintf(buf, *plen, E_BDVAL, pslot->rsc[rscid].name);
            *plen = ret;
            return;
        }

        pctx->update = newupdate / 10;      // steps of 10 ms each
        sendconfigtofpga(pctx, plen, buf);  // send down new config
    }

    return;
}


/**************************************************************
 * sendconfigtofpga():  - Send sample period to the FPGA card. 
 * Put error messages into buf and update plen.
 **************************************************************/
static void sendconfigtofpga(
    QTR4DEV  *pctx,    // This peripheral's context
    int      *plen,    // size of buf on input, #char in buf on output
    char     *buf)     // where to store user visible error messages
{
    DP_PKT   pkt;      // send write and read cmds to the qtr4
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
    pkt.reg = QTR4_SENS;      // send sensitivity and update period
    pkt.count = 2;
    pkt.data[0] = pctx->sensitivity;
    pkt.data[1] = pctx->update;
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
    QTR4DEV  *pctx)    // Send pin values of this qtr4 to the FPGA
{
    // Log the missing ack
    dplog(E_NOACK);

    return;
}

// end of qtr4.c
