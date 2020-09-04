/*
 * VL53L1X.cpp
 *
 *  Created on: 26 рту. 2020 у.
 *      Author: Elessar
 */

#include "VL53L1X.h"


uint8_t VL53L1X_t::Init(VL_IO_mode_t IO_mode) {
	uint8_t Result = retvOk;
	Result |= Reset();
// VL53L1_WaitDeviceBooted() begin
	Result |= WaitFwBootComplete();
	// check model ID and module type registers
	uint16_t DevID = 0;
	Result |= ReadReg16(VL53L1_IDENTIFICATION__MODEL_ID, &DevID);
	if (DevID != VL_MODEL_ID) {
		Printf("VL53L1 device ID does not match, %X\r", DevID);
		return retvFail;
	}
	if (Result != retvOk) { Printf("WaitDeviceBooted Fail\r"); return retvFail; }

// VL53L1_DataInit() begin
	// sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
	/* 2V8 power mode selection codex 447463 */
	if (IO_mode == io2V8) {
		uint8_t RegValue = 0;
		Result |= ReadReg(VL53L1_PAD_I2C_HV__EXTSUP_CONFIG, &RegValue);
		if (Result == retvOk) {
			RegValue = (RegValue & 0xfe) | 0x01;
			Result |= WriteReg(VL53L1_PAD_I2C_HV__EXTSUP_CONFIG, RegValue);
		}
	}

	// store oscillator info for later use
	Result |= ReadReg16(VL53L1_OSC_MEASURED__FAST_OSC__FREQUENCY, &fast_osc_frequency);
    Result |= ReadReg16(VL53L1_RESULT__OSC_CALIBRATE_VAL, &osc_calibrate_val);

	if (Result != retvOk) { Printf("DataInit Fail\r"); return retvFail; }

// VL53L1_StaticInit() begin
	Result |= WriteReg16(VL53L1_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, 0x0A00); // should already be this value after reset
	Result |= WriteReg(VL53L1_GPIO__TIO_HV_STATUS, 0x02);
	Result |= WriteReg(VL53L1_SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8); // tuning parm default
	Result |= WriteReg(VL53L1_SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16); // tuning parm default
	Result |= WriteReg(VL53L1_ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
	Result |= WriteReg(VL53L1_ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
	Result |= WriteReg(VL53L1_ALGO__RANGE_MIN_CLIP, 0); // tuning parm default
	Result |= WriteReg(VL53L1_ALGO__CONSISTENCY_CHECK__TOLERANCE, 2); // tuning parm default
	// general config
	Result |= WriteReg16(VL53L1_SYSTEM__THRESH_RATE_HIGH, 0x0000);
	Result |= WriteReg16(VL53L1_SYSTEM__THRESH_RATE_LOW, 0x0000);
	Result |= WriteReg(VL53L1_DSS_CONFIG__APERTURE_ATTENUATION, 0x38);
	// timing config
	// most of these settings will be determined later by distance and timing
	// budget configuration
	Result |= WriteReg16(VL53L1_RANGE_CONFIG__SIGMA_THRESH, 360); // tuning parm default
	Result |= WriteReg16(VL53L1_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192); // tuning parm default
	// dynamic config
	Result |= WriteReg(VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
	Result |= WriteReg(VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
	Result |= WriteReg(VL53L1_SD_CONFIG__QUANTIFIER, 2); // tuning parm default

	// from VL53L1_preset_mode_timed_ranging_*
	// GPH is 0 after reset, but writing GPH0 and GPH1 above seem to set GPH to 1,
	// and things don't seem to work if we don't set GPH back to 0 (which the API
	// does here).
	Result |= WriteReg(VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
	Result |= WriteReg(VL53L1_SYSTEM__SEED_CONFIG, 1); // tuning parm default
	// from VL53L1_config_low_power_auto_mode
	Result |= WriteReg(VL53L1_SYSTEM__SEQUENCE_CONFIG, 0x8B); // VHV, PHASECAL, DSS1, RANGE
	Result |= WriteReg16(VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
	Result |= WriteReg(VL53L1_DSS_CONFIG__ROI_MODE_CONTROL, 2); // REQUESTED_EFFFECTIVE_SPADS
	uint16_t RegValue16 = 0;
	Result |= ReadReg16(VL53L1_MM_CONFIG__OUTER_OFFSET_MM, &RegValue16);
	Result |= SetDistanceMode(dmShort);
	Result |= SetMeasTimingBudget_US(40000);
	// the API triggers this change in VL53L1_init_and_start_range() once a
	// measurement is started; assumes MM1 and MM2 are disabled
	Result |= WriteReg16(VL53L1_ALGO__PART_TO_PART_RANGE_OFFSET_MM, RegValue16 * 4);
	Result |= StartMeasurement();

	if (Result != retvOk) { Printf("StaticInit Fail\r"); return retvFail; }
	return Result;
}

uint8_t VL53L1X_t::Init_lite() {
	uint8_t Result = retvOk;
	Result |= Reset();
	Result |= WaitFwBootComplete();

	for (uint8_t RegAddr = 0x2D; RegAddr <= 0x87; RegAddr++)
		Result |= WriteReg(RegAddr, VL51L1X_DEFAULT_CONFIGURATION[RegAddr - 0x2D]);

    Result |= StartMeasurement();
    systime_t start = chVTGetSystemTimeX();
	while (CheckForDataReady() == 0) {
		chThdSleepMicroseconds(VL53L1_POLLING_DELAY_US);
		if(chVTTimeElapsedSinceX(start) > MS2ST(2000)) {
			Printf("VL53L1 CheckDataReady TimeOut\r");
//			return retvTimeout;
			break;
		}
	}
	Result |= ClearInterrupt();
	Result |= StopMeasurement();
	Result |= WriteReg(VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two bounds VHV */
	Result |= WriteReg(VL53L1_VHV_CONFIG__INIT, 0); /* start VHV from the previous temperature */

	return Result;
}

uint8_t VL53L1X_t::SetMeasTimingBudget_US(uint32_t MeasTiming_US) {	// VL53L1_SetMeasurementTimingBudgetMicroSeconds()
	uint8_t Result = retvOk;
	if ( (MeasTiming_US<20000) or (MeasTiming_US>1000000) or (MeasTiming_US<(SensState.MeasPeriod_MS+4)*1000) ) {
		Printf("Measurement Timing Budget is incorrect value: 20000 < %u < 1000000", MeasTiming_US);
		return retvBadValue;
	}

    // assumes PresetMode is LOWPOWER_AUTONOMOUS
    uint32_t range_config_timeout_us = (MeasTiming_US - TimingGuard) / 2;

    // VL53L1_calc_timeout_register_values() begin
    uint8_t range_config_vcsel_period = 0;
    Result |= ReadReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, &range_config_vcsel_period);
    // "Update Macro Period for Range A VCSEL Period"
    uint32_t macro_period_us = calcMacroPeriod(range_config_vcsel_period);

    // "Update Phase timeout - uses Timing A"
    // Timeout of 1000 is tuning parm default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
    // via VL53L1_get_preset_mode_timing_cfg().
    uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
    if (phasecal_timeout_mclks > 0xFF) {
        phasecal_timeout_mclks = 0xFF;
    }
    Result |= WriteReg(VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);
	// "Update MM Timing A timeout"
	// Timeout of 1 is tuning parm default (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
	// via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
	// actually ends up with a slightly different value because it gets assigned,
	// retrieved, recalculated with a different macro period, and reassigned,
	// but it probably doesn't matter because it seems like the MM ("mode
	// mitigation"?) sequence steps are disabled in low power auto mode anyway.
    Result |= WriteReg16(VL53L1_MM_CONFIG__TIMEOUT_MACROP_A_HI, encodeTimeout(timeoutMicrosecondsToMclks(1, macro_period_us)));
    // "Update Range Timing A timeout"
    Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, encodeTimeout(timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));
    // "Update Macro Period for Range B VCSEL Period"
    Result |= WriteReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, range_config_vcsel_period);
    // "Update MM Timing B timeout"
    // (See earlier comment about MM Timing A timeout.)
    Result |= WriteReg16(VL53L1_MM_CONFIG__TIMEOUT_MACROP_B_HI, encodeTimeout(timeoutMicrosecondsToMclks(1, macro_period_us)));
    // "Update Range Timing B timeout"
    Result |= WriteReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, encodeTimeout(timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));
    return Result;
}
uint8_t VL53L1X_t::GetMeasTimingBudget_US(uint32_t *MeasTiming_US) {	// VL53L1_GetMeasurementTimingBudgetMicroSeconds()
	uint8_t Result = retvOk;
    // assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
    // enabled: VHV, PHASECAL, DSS1, RANGE

    // "Update Macro Period for Range A VCSEL Period"
    uint8_t range_config_vcsel_period_a = 0;
    Result |= ReadReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, &range_config_vcsel_period_a);
    uint32_t macro_period_us = calcMacroPeriod(range_config_vcsel_period_a);
    uint16_t timeout_macrop_a = 0;
    Result |= ReadReg16(VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, &timeout_macrop_a);
    if (Result == retvOk) {
		// "Get Range Timing A timeout"
		uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(decodeTimeout(timeout_macrop_a), macro_period_us);
		*MeasTiming_US = 2 * range_config_timeout_us + TimingGuard;
    }
    else Printf("GetMeasTimingBudget_US() Failed!");
	return Result;
}

uint8_t VL53L1X_t::SetDistanceMode(VLDistanceMode_t DistanceMode) {	// VL53L1_SetDistanceMode()
	uint8_t Result = retvOk;
    // save existing timing budget
    uint32_t MeasTiming_US;
    Result |= GetMeasTimingBudget_US(&MeasTiming_US);

    switch (DistanceMode) {
      case dmShort:
            // from VL53L1_preset_mode_standard_ranging_short_range()
            // timing config
            Result |= WriteReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
            Result |= WriteReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
            Result |= WriteReg(VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
			// dynamic config
			Result |= WriteReg(VL53L1_SD_CONFIG__WOI_SD0, 0x07);
			Result |= WriteReg(VL53L1_SD_CONFIG__WOI_SD1, 0x05);
			Result |= WriteReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 0x06);	// tuning parm default
			Result |= WriteReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 0x06);	// tuning parm default
            break;
        case dmMedium:
            // from VL53L1_preset_mode_standard_ranging()
        	// timing config
        	Result |= WriteReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
			Result |= WriteReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
			Result |= WriteReg(VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);
			// dynamic config
			Result |= WriteReg(VL53L1_SD_CONFIG__WOI_SD0, 0x0B);
			Result |= WriteReg(VL53L1_SD_CONFIG__WOI_SD1, 0x09);
			Result |= WriteReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 0x0A);	// tuning parm default
			Result |= WriteReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 0x0A);	// tuning parm default
            break;
        case dmLong:
            // from VL53L1_preset_mode_standard_ranging_long_range()
        	// timing config
        	Result |= WriteReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
			Result |= WriteReg(VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
			Result |= WriteReg(VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
			// dynamic config
			Result |= WriteReg(VL53L1_SD_CONFIG__WOI_SD0, 0x0F);
			Result |= WriteReg(VL53L1_SD_CONFIG__WOI_SD1, 0x0D);
			Result |= WriteReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 0x0E);	// tuning parm default
			Result |= WriteReg(VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 0x0E);	// tuning parm default
            break;
    }

    // reapply timing budget
    if (Result == retvOk)
    	Result = SetMeasTimingBudget_US(MeasTiming_US);
    return Result;
}
