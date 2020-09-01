/*
 * VL53L1X.h
 *
 *  Created on: 26 авг. 2020 г.
 *      Author: Elessar
 */

#pragma once

#include "VL53L1X_register_map.h"
#include "kl_i2c.h"
//#include "evt_mask.h"
#include "uart.h"

#define VL_i2c			i2c2
#define VL_I2C_DeffAddr	0x29//52
#define NonBlockingImplement

#define VL_MODEL_ID		0xEACC

/* Define polling delays */
#define VL53L1_BOOT_COMPLETION_POLLING_TIMEOUT_MS	500
#define VL53L1_RANGE_COMPLETION_POLLING_TIMEOUT_MS  2000
#define VL53L1_TEST_COMPLETION_POLLING_TIMEOUT_MS   60000

#define VL53L1_POLLING_DELAY_US                     1000
	/*!< 1000us delay for register polling */
#define VL53L1_SOFTWARE_RESET_DURATION_US           100
	/*!< 100us software reset duration */
#define VL53L1_FIRMWARE_BOOT_TIME_US                1200
	/*!< Duration of firmware boot time for which I2C
	 access is blocked. Real Device 1ms, FPGA 15ms */
#define VL53L1_ENABLE_POWERFORCE_SETTLING_TIME_US   250
	/*!< After enabling power force a delay is required
		 to bring regulator, bandgap, oscillator time
		 to power up and settle */

// value used in measurement timing budget calculations
// assumes PresetMode is LOWPOWER_AUTONOMOUS
//
// vhv = LOWPOWER_AUTO_VHV_LOOP_DURATION_US + LOWPOWERAUTO_VHV_LOOP_BOUND
//       (tuning parm default) * LOWPOWER_AUTO_VHV_LOOP_DURATION_US
//     = 245 + 3 * 245 = 980
// TimingGuard = LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING +
//               LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING + vhv
//             = 1448 + 2100 + 980 = 4528
static const uint32_t TimingGuard = 4528;

typedef enum {
    io2V8,
	io1V8
} VL_IO_mode_t;

typedef enum {
	dmShort,	// Up to 1.3 m
    dmMedium,	// Up to 3 m
	dmLong		// Up to 4 m
} VLDistanceMode_t;

typedef enum {
	ipLow = 0,
	ipHigh = 1
} VLInterruptPolarity_t;

//extern i2c_t VL_i2c;

class VL53L1X_t { //: private IrqHandler_t {
private:
    uint16_t fast_osc_frequency;
    uint16_t osc_calibrate_val;
	struct {
		uint32_t MeasTiming_US;
		uint16_t MeasPeriod_MS;
		bool IsMeasurement = false;
	} SensState;
	uint8_t I2C_ADDR = VL_I2C_DeffAddr;
    uint8_t ReadReg(uint16_t ARegAddr, uint8_t *AValue) {
        uint8_t RegAddr[2];
        RegAddr[0] = ARegAddr >> 8;
        RegAddr[1] = ARegAddr & 0x00FF;
    	return VL_i2c.WriteRead(I2C_ADDR, RegAddr, 2, AValue, 1);
    }
    uint8_t ReadReg16(uint16_t ARegAddr, uint16_t *AValue) {
        uint8_t RegAddr[2], ReadData[2];
        RegAddr[0] = ARegAddr >> 8;
        RegAddr[1] = ARegAddr & 0x00FF;
        uint8_t Result = VL_i2c.WriteRead(I2C_ADDR, RegAddr, 2, ReadData, 2);
        if (Result == retvOk)
            *AValue = (ReadData[0]<<8)|ReadData[1];
        return Result;
    }
    uint8_t WriteReg(uint16_t ARegAddr, uint8_t AValue) {
        uint8_t Data[3];
        Data[0] = ARegAddr >> 8;
        Data[1] = ARegAddr & 0x00FF;
        Data[2] = AValue;
        return VL_i2c.Write(I2C_ADDR, Data, 3);
    }
    uint8_t WriteReg16(uint16_t ARegAddr, uint16_t AValue) {
        uint8_t Data[3];
        Data[0] = ARegAddr >> 8;
        Data[1] = ARegAddr & 0x00FF;
        Data[2] = AValue >> 8;
        Data[3] = AValue & 0x00FF;
        return VL_i2c.Write(I2C_ADDR, Data, 4);
    }
    uint8_t WriteReg32(uint16_t ARegAddr, uint32_t AValue) {
            uint8_t Data[5];
            Data[0] = ARegAddr >> 8;
            Data[1] = ARegAddr & 0x00FF;
            Data[2] = AValue >> 24;
            Data[3] = (AValue >> 16) & 0x000000FF;
            Data[4] = (AValue >> 8) & 0x000000FF;
            Data[5] = AValue & 0x000000FF;
            return VL_i2c.Write(I2C_ADDR, Data, 6);
        }

    bool IsBootComplete() {
    	uint8_t FW_SysStatus = 0;
    	/**
    	 * Determines if the firmware finished booting by reading
    	 * bit 0 of firmware__system_status register
    	 */
    	/* read current range interrupt state */
    	if (ReadReg(VL53L1_FIRMWARE__SYSTEM_STATUS, &FW_SysStatus) == retvOk)
    		return FW_SysStatus;
    	else
    		return false;
    }
    uint8_t WaitFwBootComplete() {
#ifdef NonBlockingImplement
    	/* implement non blocking version below */
	    systime_t start = chVTGetSystemTimeX();
    	while (!IsBootComplete()) {
    		chThdSleepMicroseconds(VL53L1_POLLING_DELAY_US);
    	    if(chVTTimeElapsedSinceX(start) > VL53L1_BOOT_COMPLETION_POLLING_TIMEOUT_MS) {
    	    	Printf("VL53L1 boot polling TimeOut\r");
    	    	return retvTimeout;
    	    }
    	}
#else
    	chThdSleepMilliseconds(VL53L1_BOOT_COMPLETION_POLLING_TIMEOUT_MS); // wait for firmware boot to complete
#endif
    	return retvOk;
    }

    // Period in milliseconds determining how often the sensor takes a measurement.
    uint8_t SetMeasurmentPeriod(uint32_t MeasPeriod_MS) {
    	// реализация из библиотеки STSW-IMG009 - ultra lite driver
    	return WriteReg32(VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, MeasPeriod_MS * ((osc_calibrate_val&0x3FF)*43)/40);
#if 0   // реализация из библиотеки AP_RangeFinder_VL53L1X
        // fix for actual measurement period shorter than set
        uint32_t adjusted_period_ms = MeasPeriod_MS + (MeasPeriod_MS * 64 / 1000);
        // from VL53L1_set_inter_measurement_period_ms(); VL53L1_SetInterMeasurementPeriodMilliSeconds()
        return WriteReg32(VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, adjusted_period_ms * osc_calibrate_val);
#endif
    }

    // Calculate macro period in microseconds (12.12 format) with given VCSEL period
    // assumes fast_osc_frequency has been read and stored
    // based on VL53L1_calc_macro_period_us()
    uint32_t calcMacroPeriod(uint8_t vcsel_period)
    {
      // from VL53L1_calc_pll_period_us()
      // fast osc frequency in 4.12 format; PLL period in 0.24 format
      uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;
      // from VL53L1_decode_vcsel_period()
      uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;
      // VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
      uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
      macro_period_us >>= 6;
      macro_period_us *= vcsel_period_pclks;
      macro_period_us >>= 6;
      return macro_period_us;
    }
    // Convert sequence step timeout from microseconds to macro periods with given
    // macro period in microseconds (12.12 format)
    // based on VL53L1_calc_timeout_mclks()
    uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us) {
        return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
    }
    // Convert sequence step timeout from macro periods to microseconds with given
    // macro period in microseconds (12.12 format)
    // based on VL53L1_calc_timeout_us()
    uint32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us) {
        return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
    }
    // Encode sequence step timeout register value from timeout in MCLKs
    // based on VL53L1_encode_timeout()
    uint16_t encodeTimeout(uint32_t timeout_mclks)
    {
        // encoded format: "(LSByte * 2^MSByte) + 1"
        uint32_t ls_byte = 0;
        uint16_t ms_byte = 0;
        if (timeout_mclks > 0) {
            ls_byte = timeout_mclks - 1;
            while ((ls_byte & 0xFFFFFF00) > 0) {
                ls_byte >>= 1;
                ms_byte++;
            }
            return (ms_byte << 8) | (ls_byte & 0xFF);
        }
        else {
            return 0;
        }
    }
    // Decode sequence step timeout in MCLKs from register value
    // based on VL53L1_decode_timeout()
    uint32_t decodeTimeout(uint16_t reg_val) {
        return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
    }

#if 0
    uint8_t UserRegister;
    thread_t *IPAppThd;
    eventmask_t EvtEnd;
    virtual_timer_t TmrReadMeas;

    uint8_t WriteCommand(uint8_t Command) {
        return i2c1.Write(SHT_I2C_ADDR, &Command, 1);
    }
    uint8_t StartMeasurement(MeasureType_t AMeasType) {
        uint8_t Result = retvFail;

        return Result;
    }
    uint8_t ReadMeasurement(int32_t *value) {
        MeasData_t MeasData;
        uint8_t Result = retvOk;

        return Result;
    }

    void IIrqHandler() {
        chEvtSignalI(IPAppThd, EvtEnd);
    }
#endif
public:
    uint8_t Init(VL_IO_mode_t IO_mode = io2V8);
    uint8_t SetI2CAddress(uint8_t NewAddress) {
    	return WriteReg(VL53L1_I2C_SLAVE__DEVICE_ADDRESS, NewAddress);
    }

    // VL53L1_GetCalibrationData()
    // VL53L1_SetCalibrationData()

    uint8_t SetMeasTimingBudget_US(uint32_t MeasTiming_US);
    uint8_t GetMeasTimingBudget_US(uint32_t *MeasTiming_US);

    uint8_t SetDistanceMode(VLDistanceMode_t DistanceMode);

    // Start continuous ranging measurements, with the given inter-measurement.
    uint8_t StartMeasurement(uint32_t MeasPeriod_MS = 0) {	    // VL53L1_ClearInterruptAndStartMeasurement()
    	uint8_t Result = retvOk;
    	Result |= SetMeasurmentPeriod(MeasPeriod_MS);
        Result |= ClearInterrupt();
		Result |= WriteReg(VL53L1_SYSTEM__MODE_START, 0x40);		// mode_range__timed
		return Result;
    }
    uint8_t StopMeasurement() {
    	return WriteReg(VL53L1_SYSTEM__MODE_START, 0x00);
    }
    uint8_t CheckForDataReady(bool *isDataReady) {// VL53L1X_CheckForDataReady()
    	uint8_t Result = retvOk;
		uint8_t RegVal;
		VLInterruptPolarity_t IntPol;
		Result |= GetInterruptPolarity(&IntPol);
		Result |= ReadReg(VL53L1_GPIO__TIO_HV_STATUS, &RegVal);
		/* Read in the register to check if a new value is available */
		if (Result == retvOk) {
			if ((RegVal & 1) == IntPol)
				*isDataReady = true;
			else
				*isDataReady = false;
			return retvOk;
		}
		else return retvFail;
    }
    uint8_t GetDistance(uint16_t *PDistance_MM) {
    	return ReadReg16(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, PDistance_MM);
    }

    uint8_t ClearInterrupt() {
    	return WriteReg(VL53L1_SYSTEM__INTERRUPT_CLEAR, 0x01);	// sys_interrupt_clear_range
    }
    uint8_t SetInterruptPolarity(VLInterruptPolarity_t NewPolarity) {
    	uint8_t RegVal;
    	if (ReadReg(VL53L1_GPIO_HV_MUX__CTRL, &RegVal) == retvOk) {
    		RegVal = RegVal & 0xEF;
    		if (WriteReg(VL53L1_GPIO_HV_MUX__CTRL, RegVal | NewPolarity << 4) == retvOk) {
    			return retvOk;
    		}
    		else return retvFail;
    	}
    	else return retvFail;
    }
    uint8_t GetInterruptPolarity(VLInterruptPolarity_t *PIntPol) {
    	uint8_t RegVal;
    	if (ReadReg(VL53L1_GPIO_HV_MUX__CTRL, &RegVal) == retvOk) {
    		RegVal = RegVal & 0x10;
        	*(uint8_t*)PIntPol = !(RegVal>>4);
        	return retvOk;
    	}
    	else return retvFail;
    }


    // VL53L1_WaitMeasurementDataReady()

//    void SetupSeqEndEvt(eventmask_t AEvt) {
//        IPAppThd = chThdGetSelfX();
//        EvtEnd = AEvt;
//    }
//
    uint8_t Reset() {	// VL53L1_SoftwareReset()
    	uint8_t Result = retvOk;
    	Result |= WriteReg(VL53L1_SOFT_RESET, 0x00);	// apply reset - note despite the name soft reset is active low!
    	chThdSleepMicroseconds(VL53L1_SOFTWARE_RESET_DURATION_US);	// wait for a while before releasing the reset
    	Result |= WriteReg(VL53L1_SOFT_RESET, 0x01);	// release reset
    	Result |= WaitFwBootComplete();
    	return Result;
    }
};
