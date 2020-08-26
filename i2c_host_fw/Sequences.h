/*
 * Sequences.h
 *
 *  Created on: 09 џэт. 2015 у.
 *      Author: Kreyl
 */

#pragma once

#include "ChunkTypes.h"

const LedRGBChunk_t lsqStart[] = {
        {csSetup, 99, clRed},
        {csSetup, 99, clBlue},
        {csSetup, 99, clGreen},
//        {csSetup, 99, clBlack},
//        {csSetup, 0, {7,0,0}},
        {csEnd},
};

const LedRGBChunk_t lsqPillIn[] = {
        {csSetup, 0, clBlue},
        {csEnd},
};

const LedRGBChunk_t lsqNoPill[] = {
        {csSetup, 0, clGreen},
        {csEnd},
};

const LedRGBChunk_t lsqFailure[] = {
        {csSetup, 0, clRed},
        {csWait, 99},
        {csSetup, 0, clBlack},
        {csWait, 99},
        {csSetup, 0, clRed},
        {csWait, 99},
        {csSetup, 0, clBlack},
        {csWait, 99},
        {csSetup, 0, clRed},
        {csWait, 99},
        {csSetup, 0, clBlack},
        {csWait, 135},
        {csGoto, 0}
};

const LedRGBChunk_t lsqUsbReady[] = {
        {csSetup, 99, {0,9,0}},
        {csEnd},
};

const LedRGBChunk_t lsqUSBCmd[] = {
        {csSetup, 0, clBlue},
        {csWait, 135},
        {csSetup, 0, {0,9,0}},
        {csEnd}
};
