/*
 * EvtMsgIDs.h
 *
 *  Created on: 21 ���. 2017 �.
 *      Author: Kreyl
 */

#pragma once

enum EvtMsgId_t {
    evtIdNone = 0, // Always

	evtIdCheckMeasure,


    // Pretending to eternity
    evtIdShellCmd,
    evtIdEverySecond,

    // Usb
    evtIdUsbConnect,
    evtIdUsbDisconnect,
    evtIdUsbReady,
    evtIdUsbNewCmd,
    evtIdUsbInDone,
    evtIdUsbOutDone,
};
