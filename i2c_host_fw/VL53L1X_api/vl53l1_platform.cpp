
/*
* This file is part of VL53L1 Platform
*
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>

#include "kl_i2c.h"
#define VL53L1_i2c			i2c2

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
	uint8_t Result = 0;
//    uint8_t RegAddr[2];
//    RegAddr[0] = index >> 8;
//    RegAddr[1] = index & 0xFF;
//    Status |= Dev->I2cHandle->Write(Dev->I2cDevAddr, RegAddr, 2);
//    Status |= Dev->I2cHandle->Write(Dev->I2cDevAddr, pdata, count);
    uint8_t _I2CBuffer[256];
    _I2CBuffer[0] = index >> 8;
    _I2CBuffer[1] = index & 0xFF;
    memcpy(&_I2CBuffer[2], pdata, count);
    Result |= VL53L1_i2c.Write(dev, _I2CBuffer, count+2);
    return Result;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){
    uint8_t RegAddr[2];
    RegAddr[0] = index >> 8;
    RegAddr[1] = index & 0xFF;
    return VL53L1_i2c.WriteRead(dev, RegAddr, 2, pdata, count);
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
    uint8_t SendData[3];
    SendData[0] = index >> 8;
    SendData[1] = index & 0xFF;
    SendData[2] = data;
    return VL53L1_i2c.Write(dev, SendData, 3);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
    uint8_t SendData[3];
    SendData[0] = index >> 8;
    SendData[1] = index & 0xFF;
    SendData[2] = data >> 8;
    SendData[3] = data & 0xFF;
    return VL53L1_i2c.Write(dev, SendData, 4);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
    uint8_t SendData[5];
    SendData[0] = index >> 8;
    SendData[1] = index & 0xFF;
    SendData[2] = data >> 24;
    SendData[3] = (data >> 16) & 0xFF;
    SendData[4] = (data >> 8) & 0xFF;
    SendData[5] = data & 0xFF;
    return VL53L1_i2c.Write(dev, SendData, 6);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
    uint8_t RegAddr[2];
    RegAddr[0] = index >> 8;
    RegAddr[1] = index & 0xFF;
	return VL53L1_i2c.WriteRead(dev, RegAddr, 2, data, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
	uint8_t Result = 0;
    uint8_t RegAddr[2], ReadData[2];
    RegAddr[0] = index >> 8;
    RegAddr[1] = index & 0xFF;
    Result = VL53L1_i2c.WriteRead(dev, RegAddr, 2, ReadData, 2);
    if (Result == 0)
        *data = ((uint16_t)ReadData[0]<<8)|ReadData[1];
    return Result;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
    uint8_t RegAddr[2], ReadData[4];
    RegAddr[0] = index >> 8;
    RegAddr[1] = index & 0xFF;
    uint8_t Result = VL53L1_i2c.WriteRead(dev, RegAddr, 2, ReadData, 4);
    if (Result == 0) {
        *data = (uint32_t)ReadData[0]<<24;
        *data |= (uint32_t)ReadData[1]<<16;
        *data |= (uint16_t)ReadData[2]<<8;
        *data |= ReadData[3];
    }
    return Result;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	chThdSleepMilliseconds(wait_ms);
	return 0; // to be implemented
}
