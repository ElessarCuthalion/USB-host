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

const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {
	0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
	0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
	0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
	0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
	0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
	0x00, /* 0x32 : not user-modifiable */
	0x02, /* 0x33 : not user-modifiable */
	0x08, /* 0x34 : not user-modifiable */
	0x00, /* 0x35 : not user-modifiable */
	0x08, /* 0x36 : not user-modifiable */
	0x10, /* 0x37 : not user-modifiable */
	0x01, /* 0x38 : not user-modifiable */
	0x01, /* 0x39 : not user-modifiable */
	0x00, /* 0x3a : not user-modifiable */
	0x00, /* 0x3b : not user-modifiable */
	0x00, /* 0x3c : not user-modifiable */
	0x00, /* 0x3d : not user-modifiable */
	0xff, /* 0x3e : not user-modifiable */
	0x00, /* 0x3f : not user-modifiable */
	0x0F, /* 0x40 : not user-modifiable */
	0x00, /* 0x41 : not user-modifiable */
	0x00, /* 0x42 : not user-modifiable */
	0x00, /* 0x43 : not user-modifiable */
	0x00, /* 0x44 : not user-modifiable */
	0x00, /* 0x45 : not user-modifiable */
	0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
	0x0b, /* 0x47 : not user-modifiable */
	0x00, /* 0x48 : not user-modifiable */
	0x00, /* 0x49 : not user-modifiable */
	0x02, /* 0x4a : not user-modifiable */
	0x0a, /* 0x4b : not user-modifiable */
	0x21, /* 0x4c : not user-modifiable */
	0x00, /* 0x4d : not user-modifiable */
	0x00, /* 0x4e : not user-modifiable */
	0x05, /* 0x4f : not user-modifiable */
	0x00, /* 0x50 : not user-modifiable */
	0x00, /* 0x51 : not user-modifiable */
	0x00, /* 0x52 : not user-modifiable */
	0x00, /* 0x53 : not user-modifiable */
	0xc8, /* 0x54 : not user-modifiable */
	0x00, /* 0x55 : not user-modifiable */
	0x00, /* 0x56 : not user-modifiable */
	0x38, /* 0x57 : not user-modifiable */
	0xff, /* 0x58 : not user-modifiable */
	0x01, /* 0x59 : not user-modifiable */
	0x00, /* 0x5a : not user-modifiable */
	0x08, /* 0x5b : not user-modifiable */
	0x00, /* 0x5c : not user-modifiable */
	0x00, /* 0x5d : not user-modifiable */
	0x01, /* 0x5e : not user-modifiable */
	0xcc, /* 0x5f : not user-modifiable */
	0x0f, /* 0x60 : not user-modifiable */
	0x01, /* 0x61 : not user-modifiable */
	0xf1, /* 0x62 : not user-modifiable */
	0x0d, /* 0x63 : not user-modifiable */
	0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
	0x68, /* 0x65 : Sigma threshold LSB */
	0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
	0x80, /* 0x67 : Min count Rate LSB */
	0x08, /* 0x68 : not user-modifiable */
	0xb8, /* 0x69 : not user-modifiable */
	0x00, /* 0x6a : not user-modifiable */
	0x00, /* 0x6b : not user-modifiable */
	0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
	0x00, /* 0x6d : Intermeasurement period */
	0x0f, /* 0x6e : Intermeasurement period */
	0x89, /* 0x6f : Intermeasurement period LSB */
	0x00, /* 0x70 : not user-modifiable */
	0x00, /* 0x71 : not user-modifiable */
	0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
	0x00, /* 0x73 : distance threshold high LSB */
	0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
	0x00, /* 0x75 : distance threshold low LSB */
	0x00, /* 0x76 : not user-modifiable */
	0x01, /* 0x77 : not user-modifiable */
	0x0f, /* 0x78 : not user-modifiable */
	0x0d, /* 0x79 : not user-modifiable */
	0x0e, /* 0x7a : not user-modifiable */
	0x0e, /* 0x7b : not user-modifiable */
	0x00, /* 0x7c : not user-modifiable */
	0x00, /* 0x7d : not user-modifiable */
	0x02, /* 0x7e : not user-modifiable */
	0xc7, /* 0x7f : ROI center, use SetROI() */
	0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
	0x9B, /* 0x81 : not user-modifiable */
	0x00, /* 0x82 : not user-modifiable */
	0x00, /* 0x83 : not user-modifiable */
	0x00, /* 0x84 : not user-modifiable */
	0x01, /* 0x85 : not user-modifiable */
	0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
	0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
};
//#define VL51L1X_DEFAULT_CONFIGURATION_SIZE	sizeof(VL51L1X_DEFAULT_CONFIGURATION)
static const uint8_t status_rtn[24] = {
	255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
	255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
	255, 255, 11, 12
};

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
    	    if(chVTTimeElapsedSinceX(start) > MS2ST(VL53L1_BOOT_COMPLETION_POLLING_TIMEOUT_MS)) {
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

    void IIrqHandler() {
        chEvtSignalI(IPAppThd, EvtEnd);
    }
#endif
public:
    uint8_t Init(VL_IO_mode_t IO_mode = io2V8);
    uint8_t Init_lite();
    uint8_t InitLiteAndStart() {
    	uint8_t Result = retvOk;
    	Result |= Init_lite();
    	Result |= SetDistanceMode_lite(dmLong);
    	Result |= SetMeasTimingBudget_lite_MS(100);
    	Result |= StartMeasurement(100);
    	return Result;
    }
    uint8_t SetI2CAddress(uint8_t NewAddress) {
    	return WriteReg(VL53L1_I2C_SLAVE__DEVICE_ADDRESS, NewAddress);
    }

    // VL53L1_GetCalibrationData()
    // VL53L1_SetCalibrationData()

    uint8_t SetMeasTimingBudget_US(uint32_t MeasTiming_US);
    uint8_t GetMeasTimingBudget_US(uint32_t *MeasTiming_US);
    uint8_t SetMeasTimingBudget_lite_MS(uint16_t MeasTiming_MS) {
    	uint8_t Result = retvOk;
    	VLDistanceMode_t DistanceMode;
    	Result |= GetDistanceMode_lite(&DistanceMode);
    	if (DistanceMode == dmShort) {
    		switch (MeasTiming_MS) {
    		case 15: /* only available in short distance mode */
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01D);
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0027);
    			break;
    		case 20:
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0051);
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
    			break;
    		case 33:
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00D6);
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
    			break;
    		case 50:
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x1AE);
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01E8);
    			break;
    		case 100:
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02E1);
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0388);
    			break;
    		case 200:
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x03E1);
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0496);
    			break;
    		case 500:
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0591);
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x05C1);
    			break;
    		default:
    			Printf("VL MeasTiming_MS is BadValue\r");
    			return retvBadValue;
    			break;
    		}
    	}
		if (DistanceMode == dmLong) {
    		switch (MeasTiming_MS) {
    		case 20:
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x001E);
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0022);
    			break;
    		case 33:
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0060);
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
    			break;
    		case 50:
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00AD);
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x00C6);
    			break;
    		case 100:
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01CC);
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01EA);
    			break;
    		case 200:
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02D9);
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x02F8);
    			break;
    		case 500:
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x048F);
    			Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x04A4);
    			break;
    		default:
    			Printf("VL MeasTiming_MS is BadValue\r");
    			return retvBadValue;
    			break;
    		}
    	}
    	return Result;
    }
    uint8_t GetMeasTimingBudget_lite_MS(uint16_t *MeasTiming_MS) {
    	uint16_t RegVaule;
    	if (ReadReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, &RegVaule) == retvOk) {
			switch (RegVaule) {
				case 0x001D :
					*MeasTiming_MS = 15;
					break;
				case 0x0051 :
				case 0x001E :
					*MeasTiming_MS = 20;
					break;
				case 0x00D6 :
				case 0x0060 :
					*MeasTiming_MS = 33;
					break;
				case 0x1AE :
				case 0x00AD :
					*MeasTiming_MS = 50;
					break;
				case 0x02E1 :
				case 0x01CC :
					*MeasTiming_MS = 100;
					break;
				case 0x03E1 :
				case 0x02D9 :
					*MeasTiming_MS = 200;
					break;
				case 0x0591 :
				case 0x048F :
					*MeasTiming_MS = 500;
					break;
				default:
					Printf("VL GetMeasTimingBudget_Simple Failed\r");
					return retvFail;
					break;
			}
			return retvOk;
    	}
    	else
    		return retvFail;
    }

    uint8_t SetDistanceMode(VLDistanceMode_t DistanceMode);
    uint8_t SetDistanceMode_lite(VLDistanceMode_t DistanceMode) {
    	uint8_t Result = retvOk;
        // save existing timing budget
        uint16_t MeasTiming_MS;
        Result |= GetMeasTimingBudget_lite_MS(&MeasTiming_MS);
        switch (DistanceMode) {
          case dmShort:
      			// timing config
      			Result |= WriteReg(VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
                Result |= WriteReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
                Result |= WriteReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
                Result |= WriteReg(VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
    			// dynamic config
    			Result |= WriteReg(VL53L1_SD_CONFIG__WOI_SD0, 0x07);
    			Result |= WriteReg(VL53L1_SD_CONFIG__WOI_SD1, 0x05);
    			Result |= WriteReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 0x06);	// tuning parm default
    			Result |= WriteReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 0x06);	// tuning parm default
                break;
            case dmLong:
            	// timing config
            	Result |= WriteReg(VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
            	Result |= WriteReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
    			Result |= WriteReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
    			Result |= WriteReg(VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
    			// dynamic config
    			Result |= WriteReg(VL53L1_SD_CONFIG__WOI_SD0, 0x0F);
    			Result |= WriteReg(VL53L1_SD_CONFIG__WOI_SD1, 0x0D);
    			Result |= WriteReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 0x0E);	// tuning parm default
    			Result |= WriteReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 0x0E);	// tuning parm default
                break;
            default:
            	Printf("VL DistanceMode is BadValue\r");
            	return retvBadValue;
            	break;
        }
        // reapply timing budget
        if (Result == retvOk)
        	Result = SetMeasTimingBudget_lite_MS(MeasTiming_MS);
        return Result;
    }
    uint8_t GetDistanceMode_lite(VLDistanceMode_t *PDistanceMode) {
    	uint8_t RegVal;
    	if (ReadReg(VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP, &RegVal) == retvOk) {
			switch (RegVal) {
				case 0x14:
					*PDistanceMode = dmShort;
					break;
				case 0x0A:
					*PDistanceMode = dmLong;
					break;
				default:
					Printf("VL DistanceMode is BadValue\r");
					return retvBadValue;
					break;
			}
            return retvOk;
    	}
    	else
    		return retvFail;
    }


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
    bool CheckForDataReady() {	// VL53L1X_CheckForDataReady()
    	uint8_t Result = retvOk;
		uint8_t RegVal;
		VLInterruptPolarity_t IntPol;
		Result |= GetInterruptPolarity(&IntPol);
		Result |= ReadReg(VL53L1_GPIO__TIO_HV_STATUS, &RegVal);
		/* Read in the register to check if a new value is available */
		if ( (Result == retvOk) and ((RegVal & 1) == IntPol) )
			return 1;
		else
			return 0;
    }
    uint8_t GetDistance(uint16_t *PDistance_MM) {
//    	uint16_t Distance_MM = 0;
//    	ReadReg16(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &Distance_MM);
//    	Printf("Distance %u\r", Distance_MM);
    	return ReadReg16(VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, PDistance_MM);
    }
    uint8_t GetRangeStatus(uint8_t *RangeStatus) {
    	uint8_t RgSt;
    	*RangeStatus = 255;
    	if (ReadReg(VL53L1_RESULT__RANGE_STATUS, &RgSt) == retvOk) {
			RgSt = RgSt & 0x1F;
			if (RgSt < 24)
				*RangeStatus = status_rtn[RgSt];
			return retvOk;
    	}
    	else return retvFail;
    }

    uint8_t ClearInterrupt() {
    	return WriteReg(VL53L1_SYSTEM__INTERRUPT_CLEAR, 0x01);	// sys_interrupt_clear_range
    }
    uint8_t SetInterruptPolarity(VLInterruptPolarity_t NewPolarity) {
    	uint8_t RegVal;
    	if (ReadReg(VL53L1_GPIO_HV_MUX__CTRL, &RegVal) == retvOk) {
    		RegVal = RegVal & 0xEF;
    		if (WriteReg(VL53L1_GPIO_HV_MUX__CTRL, RegVal | (!NewPolarity) << 4) == retvOk) {
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
