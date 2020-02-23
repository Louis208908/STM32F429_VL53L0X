#include "VL53L0X.h"
#include "i2c.h"

extern VL53 vl53[];
extern int qq,AmountOfVL53;

void setAddress(struct VL53L0X *vl53, uint8_t new_addr) {
	writeReg(vl53, I2C_SLAVE_DEVICE_ADDRESS, new_addr);
	vl53->address = new_addr << 1;
}

void setI2CStdMode(struct VL53L0X *vl53) {
	writeReg(vl53, 0x88, 0x00);
	writeReg(vl53, 0x80, 0x01);
	writeReg(vl53, 0xFF, 0x01);
	writeReg(vl53, 0x00, 0x00);
	vl53->param.stop_variable = readReg(vl53, 0x91);
	writeReg(vl53, 0x00, 0x01);
	writeReg(vl53, 0xFF, 0x00);
	writeReg(vl53, 0x80, 0x00);
}

bool Init_Calibration(struct VL53L0X *vl53, bool io_2v8, float finalRangeSignalRate) {
	// check model ID register (value specified in datasheet)

//	vl53 ,which->qqq = readReg(vl53 ,  vl53 ,which->vl53[which].regAddr.IDENTIFICATION_MODEL_ID);
	if (readReg(vl53, IDENTIFICATION_MODEL_ID) != 238) {
		return false;
	}
	// vl53L0X_DataInit() begin

	// sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
	if (vl53->param.io_2v8) {
		writeReg(vl53, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
				readReg(vl53, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
	}

	// "Set I2C standard mode"
	setI2CStdMode(vl53);

	// disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
	writeReg(vl53, MSRC_CONFIG_CONTROL,
			readReg(vl53, MSRC_CONFIG_CONTROL) | 0x12);

	// set final range signal rate limit to 0.25 MCPS (million counts per second)
	setSignalRateLimit(vl53, finalRangeSignalRate);

	//checked ok
	writeReg(vl53, SYSTEM_SEQUENCE_CONFIG, 0xFF);

	// vl53[which]L0X_DataInit() end

	// vl53[which]L0X_StaticInit() begin

	uint8_t spad_count;
	bool spad_type_is_aperture;
//  vl53 ,which->checkTF[44] = false;

	if (!getSpadInfo(vl53, &spad_count, &spad_type_is_aperture)) {
		return false;
	}

	// The SPAD map (RefGoodSpadMap) is read by vl53[which]L0X_get_info_from_device() in
	// the API, but the same data seems to be more easily readable from
	// GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
	uint8_t ref_spad_map[6];
	readMulti(vl53, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	// -- vl53[which]L0X_set_reference_spads() begin (assume NVM values are valid)

	writeReg(vl53, 0xFF, 0x01);
	writeReg(vl53, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
	writeReg(vl53, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
	writeReg(vl53, 0xFF, 0x00);
	writeReg(vl53, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

	uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
	uint8_t spads_enabled = 0;
	uint8_t i = 0;
	for (i = 0; i < 48; i++) {
		if (i < first_spad_to_enable || spads_enabled == spad_count) {
			// This bit is lower than the first one that should be enabled, or
			// (reference_spad_count) bits have already been enabled, so zero this bit
			ref_spad_map[i / 8] &= ~(1 << (i % 8));
		} else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
			spads_enabled++;
		}
	}

	writeMulti(vl53, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

	// -- vl53[which]L0X_set_reference_spads() end

	// -- vl53[which]L0X_load_tuning_settings() begin
	// DefaultTuningSettings from vl53[which]l0x_tuning.h

	VL53L0X_load_tuning_settings(vl53);

	// -- vl53[which]L0X_load_tuning_settings() end

	// "Set interrupt config to new sample ready"
	// -- vl53[which]L0X_SetGpioConfig() begin

	writeReg(vl53, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
	writeReg(vl53, GPIO_HV_MUX_ACTIVE_HIGH,
			readReg(vl53, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
	writeReg(vl53, SYSTEM_INTERRUPT_CLEAR, 0x01);

	// -- vl53[which]L0X_SetGpioConfig() end

	vl53->param.measurement_timing_budget_us = getMeasurementTimingBudget(vl53);

	// "Disable MSRC and TCC by default"
	// MSRC = Minimum Signal Rate Check
	// TCC = Target CentreCheck
	// -- vl53[which]L0X_SetSequenceStepEnable() begin

	writeReg(vl53, SYSTEM_SEQUENCE_CONFIG, 0xE8);

	// -- vl53[which]L0X_SetSequenceStepEnable() end

	// "Recalculate timing budget"
	setMeasurementTimingBudget(vl53, vl53->param.measurement_timing_budget_us);
//	setMeasurementTimingBudget(vl53 ,which , 33849);
	// vl53[which]L0X_StaticInit() end

	// vl53[which]L0X_PerformRefCalibration() begin (vl53[which]L0X_perform_ref_calibration())

	// -- vl53[which]L0X_perform_vhv_calibration() begin

	writeReg(vl53, SYSTEM_SEQUENCE_CONFIG, 0x01);
	if (!performSingleRefCalibration(vl53, 0x40)) {
		return false;
	}
	// -- vl53[which]L0X_perform_vhv_calibration() end

	// -- vl53[which]L0X_perform_phase_calibration() begin

	writeReg(vl53, SYSTEM_SEQUENCE_CONFIG, 0x02);
	if (!performSingleRefCalibration(vl53, 0x00)) {
		return false;
	}

	// -- vl53[which]L0X_perform_phase_calibration() end

	// "restore the previous Sequence Config"
	writeReg(vl53, SYSTEM_SEQUENCE_CONFIG, 0xE8);
	// vl53[which]L0X_PerformRefCalibration() end
	return true;
}

void writeReg(struct VL53L0X *vl53, uint8_t reg, uint8_t value) {
	//HAL_I2C_Mem_write(vl53->I2cLine,devAddress,memAddress,I2C_MEMADD_SIZE_8BIT,fromhere,size,timeout);
	//HAL_I2C_Mem_read(vl53->I2cLine,devAddress,fromThisMemAddress,I2C_MEMADD_SIZE_8BIT,tohere,size,timeout);
	HAL_I2C_Mem_Write(vl53->I2cLine, vl53->address, reg, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) &value, 1, 1);
}

void writeReg16Bit(struct VL53L0X *vl53, uint8_t reg, uint16_t value) {
	int valueShift8Bit = (value >> 8) & 0xFF;
	HAL_I2C_Mem_Write(vl53->I2cLine, vl53->address, reg, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) &valueShift8Bit, 1, 1);
	HAL_I2C_Mem_Write(vl53->I2cLine, vl53->address, reg, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) &value, 1, 1);
}

// Write a 32-bit register
void writeReg32Bit(struct VL53L0X *vl53, uint8_t reg, uint32_t value) {
	int valueShift24Bit = (value >> 24) & 0xFF;
	int valueShift16Bit = (value >> 16) & 0xFF;
	int valueShift8Bit = (value >> 8) & 0xFF;
	int valueNoShift = value & 0xFF;
	HAL_I2C_Mem_Write(vl53->I2cLine, vl53->address, reg, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) &valueShift24Bit, 1, 1);
	HAL_I2C_Mem_Write(vl53->I2cLine, vl53->address, reg + 1, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) &valueShift16Bit, 1, 1);
	HAL_I2C_Mem_Write(vl53->I2cLine, vl53->address, reg + 2, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) &valueShift8Bit, 1, 1);
	HAL_I2C_Mem_Write(vl53->I2cLine, vl53->address, reg + 3, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) &valueNoShift, 1, 1);
}

uint8_t readReg(struct VL53L0X *vl53, uint8_t reg) {
	uint8_t value;
	//HAL_I2C_Mem_read(vl53->I2cLine,devAddress,fromThisMemAddress,I2C_MEMADD_SIZE_8BIT,tohere,size,timeout);
	HAL_I2C_Mem_Read(vl53->I2cLine, vl53->address, reg, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) &value, 1, 1);
	return value;
}

uint16_t readReg16Bit(struct VL53L0X *vl53, uint8_t reg) {
	uint16_t valueArr[2];
	HAL_I2C_Mem_Read(vl53->I2cLine, vl53->address, reg, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) &valueArr[0], 1, 1);
	HAL_I2C_Mem_Read(vl53->I2cLine, vl53->address, reg + 1, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) &valueArr[1], 1, 1);
	valueArr[0] = valueArr[0] << 8;

//	valueArr[0] = (uint16_t)valueArr[0];
	valueArr[0] |= valueArr[1];
	return valueArr[0];
}

uint32_t readReg32Bit(struct VL53L0X *vl53, uint8_t reg) {
	uint32_t valueArr[4];
	HAL_I2C_Mem_Read(vl53->I2cLine, vl53->address, reg, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) &valueArr[0], 1, 1);
	HAL_I2C_Mem_Read(vl53->I2cLine, vl53->address, reg, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) &valueArr[1], 1, 1);
	HAL_I2C_Mem_Read(vl53->I2cLine, vl53->address, reg, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) &valueArr[2], 1, 1);
	HAL_I2C_Mem_Read(vl53->I2cLine, vl53->address, reg, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) &valueArr[3], 1, 1);
	valueArr[0] = (uint32_t) valueArr[0] << 24;
	valueArr[0] |= (uint32_t) valueArr[1] << 16;
	valueArr[0] |= (uint32_t) valueArr[2] << 8;
	valueArr[0] |= (uint16_t) valueArr[3];
	return valueArr[0];
}

void writeMulti(struct VL53L0X *vl53, uint8_t reg, uint8_t const *src,
		uint8_t count) {
	int count1 = 0;
	int valArr[count];
	memset(valArr, 0, count);
	for (count1 = 0; count1 < count; count1++) {
		valArr[count1] = *(src++);
//		vl53->valueCheck[count1] = valArr[count1];
	}
//	HAL_I2C_Mem_Write(vl53->I2cLine, vl53->address, reg, I2C_MEMADD_SIZE_8BIT,
//			(uint8_t *) vl53->valueCheck, count, 1);
	HAL_I2C_Mem_Write(vl53->I2cLine, vl53->address, reg, I2C_MEMADD_SIZE_8BIT,
			(uint8_t*) valArr, count, 1);
}

void readMulti(struct VL53L0X *vl53, uint8_t reg, uint8_t *dst, uint8_t count) {
	int num = 0;
	for (num = 0; num < count; num++) {
		HAL_I2C_Mem_Read(vl53->I2cLine, vl53->address, reg, I2C_MEMADD_SIZE_8BIT,
				&dst[num], 1, 1);
	}

}

bool setSignalRateLimit(struct VL53L0X *vl53, float limit_Mcps) {
	if (limit_Mcps < 0 || limit_Mcps > 511.99)
		return false;
	// Q9.7 fixed point format (9 integer bits, 7 fractional bits)
	writeReg16Bit(vl53, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
			limit_Mcps * (1 << 7));
	return true;
}

float getSignalRateLimit(struct VL53L0X *vl53) {
	return (float) readReg16Bit(vl53,
			FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

bool setMeasurementTimingBudget(struct VL53L0X *vl53, uint32_t budget_us) {

	uint16_t const StartOverhead = 1910;
	uint16_t const EndOverhead = 960;
	uint16_t const MsrcOverhead = 660;
	uint16_t const TccOverhead = 590;
	uint16_t const DssOverhead = 690;
	uint16_t const PreRangeOverhead = 660;
	uint16_t const FinalRangeOverhead = 550;
	uint32_t const MinTimingBudget = 20000;
	if (budget_us < MinTimingBudget)
		return false;
	uint32_t used_budget_us = StartOverhead + EndOverhead;
	getSequenceStepEnables(vl53, &(vl53->sequenceStepEnables));
	getSequenceStepTimeouts(vl53, &(vl53->sequenceStepEnables),
			&(vl53->sequenceStepTimeouts));
	if (vl53->sequenceStepEnables.tcc) {
		used_budget_us += (vl53->sequenceStepTimeouts.msrc_dss_tcc_us
				+ TccOverhead);
	}
	if (vl53->sequenceStepEnables.dss)
		used_budget_us += 2
				* (vl53->sequenceStepTimeouts.msrc_dss_tcc_us + DssOverhead);
	else if (vl53->sequenceStepEnables.msrc)
		used_budget_us += (vl53->sequenceStepTimeouts.msrc_dss_tcc_us
				+ MsrcOverhead);

	if (vl53->sequenceStepEnables.pre_range)
		used_budget_us += (vl53->sequenceStepTimeouts.pre_range_us
				+ PreRangeOverhead);
	if (vl53->sequenceStepEnables.final_range) {
		used_budget_us += FinalRangeOverhead;
		if (used_budget_us > budget_us) {
			// "Requested timeout too big."
			return false;
		}

		uint32_t final_range_timeout_us = budget_us - used_budget_us;

		// set_sequence_step_timeout() begin
		// (SequenceStepId == vl53[which]L0X_SEQUENCESTEP_FINAL_RANGE)

		// "For the final range timeout, the pre-range timeout
		//  must be added. To do this both final and pre-range
		//  timeouts must be expressed in macro periods MClks
		//  because they have different vcsel periods."

		uint32_t final_range_timeout_mclks = timeoutMicrosecondsToMclks(vl53,
				final_range_timeout_us,
				vl53->sequenceStepTimeouts.final_range_vcsel_period_pclks);

		if (vl53->sequenceStepEnables.pre_range) {
			final_range_timeout_mclks +=
					vl53->sequenceStepTimeouts.pre_range_mclks;
		}

		writeReg16Bit(vl53, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
				encodeTimeout(final_range_timeout_mclks));

		// set_sequence_step_timeout() end

		vl53->param.measurement_timing_budget_us = budget_us; // store for internal reuse
	}
	return true;
}

uint32_t getMeasurementTimingBudget(struct VL53L0X *vl53) {
	uint16_t const StartOverhead = 1910;
	uint16_t const EndOverhead = 960;
	uint16_t const MsrcOverhead = 660;
	uint16_t const TccOverhead = 590;
	uint16_t const DssOverhead = 690;
	uint16_t const PreRangeOverhead = 660;
	uint16_t const FinalRangeOverhead = 550;
	uint32_t budget_us = StartOverhead + EndOverhead;
	getSequenceStepEnables(vl53, &(vl53->sequenceStepEnables));
	getSequenceStepTimeouts(vl53, &(vl53->sequenceStepEnables),
			&(vl53->sequenceStepTimeouts));
	if (vl53->sequenceStepEnables.tcc)
		budget_us += (vl53->sequenceStepTimeouts.msrc_dss_tcc_us + TccOverhead);
	if (vl53->sequenceStepEnables.dss)
		budget_us += 2
				* (vl53->sequenceStepTimeouts.msrc_dss_tcc_us + DssOverhead);
	else if (vl53->sequenceStepEnables.msrc)
		budget_us +=
				(vl53->sequenceStepTimeouts.msrc_dss_tcc_us + MsrcOverhead);
	if (vl53->sequenceStepEnables.pre_range)
		budget_us +=
				(vl53->sequenceStepTimeouts.pre_range_us + PreRangeOverhead);

	if (vl53->sequenceStepEnables.final_range)
		budget_us += (vl53->sequenceStepTimeouts.final_range_us
				+ FinalRangeOverhead);
	vl53->param.measurement_timing_budget_us = budget_us; // store for internal reuse
	return budget_us;
}

bool setVcselPulsePeriod(struct VL53L0X *vl53, enum VcselPeriodType type,
		uint8_t period_pclks) {
	uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);
	getSequenceStepEnables(vl53, &(vl53->sequenceStepEnables));
	getSequenceStepTimeouts(vl53, &(vl53->sequenceStepEnables),
			&(vl53->sequenceStepTimeouts));
	if (type == VcselPeriodPreRange) {
		// "Set phase check limits"
		switch (period_pclks) {
		case 12:
			writeReg(vl53, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
			break;

		case 14:
			writeReg(vl53, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
			break;
		case 16:
			writeReg(vl53, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
			break;

		case 18:
			writeReg(vl53, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
			break;

		default:
			// invalid period
			return false;
		}
		writeReg(vl53, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
		writeReg(vl53, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
		uint16_t new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(vl53,
				vl53->sequenceStepTimeouts.pre_range_us, period_pclks);
		writeReg16Bit(vl53, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
				encodeTimeout(new_pre_range_timeout_mclks));
		uint16_t new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(vl53,
				vl53->sequenceStepTimeouts.msrc_dss_tcc_us, period_pclks);
		writeReg(vl53, MSRC_CONFIG_TIMEOUT_MACROP,
				(new_msrc_timeout_mclks > 256) ?
						255 : (new_msrc_timeout_mclks - 1));
	} else if (type == VcselPeriodFinalRange) {
		switch (period_pclks) {
		case 8:
			writeReg(vl53, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
			writeReg(vl53, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
			writeReg(vl53, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
			writeReg(vl53, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
			writeReg(vl53, 0xFF, 0x01);
			writeReg(vl53, ALGO_PHASECAL_LIM, 0x30);
			writeReg(vl53, 0xFF, 0x00);
			break;
		case 10:
			writeReg(vl53, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
			writeReg(vl53, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
			writeReg(vl53, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
			writeReg(vl53, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
			writeReg(vl53, 0xFF, 0x01);
			writeReg(vl53, ALGO_PHASECAL_LIM, 0x20);
			writeReg(vl53, 0xFF, 0x00);
			break;
		case 12:
			writeReg(vl53, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
			writeReg(vl53, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
			writeReg(vl53, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
			writeReg(vl53, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
			writeReg(vl53, 0xFF, 0x01);
			writeReg(vl53, ALGO_PHASECAL_LIM, 0x20);
			writeReg(vl53, 0xFF, 0x00);
			break;
		case 14:
			writeReg(vl53, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
			writeReg(vl53, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
			writeReg(vl53, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
			writeReg(vl53, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
			writeReg(vl53, 0xFF, 0x01);
			writeReg(vl53, ALGO_PHASECAL_LIM, 0x20);
			writeReg(vl53, 0xFF, 0x00);
			break;
		default:
			// invalid period
			return false;
		}
		writeReg(vl53, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
		uint16_t new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(
				vl53, vl53->sequenceStepTimeouts.final_range_us, period_pclks);
		if (vl53->sequenceStepEnables.pre_range)
			new_final_range_timeout_mclks +=
					vl53->sequenceStepTimeouts.pre_range_mclks;
		writeReg16Bit(vl53, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
				encodeTimeout(new_final_range_timeout_mclks));
	} else
		return false;
	setMeasurementTimingBudget(vl53, vl53->param.measurement_timing_budget_us);
	uint8_t sequence_config = readReg(vl53, SYSTEM_SEQUENCE_CONFIG);
	writeReg(vl53, SYSTEM_SEQUENCE_CONFIG, 0x02);
	performSingleRefCalibration(vl53, 0x0);
	writeReg(vl53, SYSTEM_SEQUENCE_CONFIG, sequence_config);
	return true;
}

uint8_t getVcselPulsePeriod(struct VL53L0X *vl53, enum VcselPeriodType type) {
	if (type == VcselPeriodPreRange)
		return decodeVcselPeriod(readReg(vl53, PRE_RANGE_CONFIG_VCSEL_PERIOD));
	else if (type == VcselPeriodFinalRange)
		return decodeVcselPeriod(readReg(vl53, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
	else
		return 255;
}

void startContinuous(struct VL53L0X *vl53, uint32_t period_ms) {
	writeReg(vl53, 0x80, 0x01);
	writeReg(vl53, 0xFF, 0x01);
	writeReg(vl53, 0x00, 0x00);
	writeReg(vl53, 0x91, vl53->param.stop_variable);
	writeReg(vl53, 0x00, 0x01);
	writeReg(vl53, 0xFF, 0x00);
	writeReg(vl53, 0x80, 0x00);
	if (period_ms != 0) {
		// continuous timed mode

		// vl53L0X_SetInterMeasurementPeriodMilliSeconds() begin

		uint16_t osc_calibrate_val = readReg16Bit(vl53, OSC_CALIBRATE_VAL);

		if (osc_calibrate_val != 0)
			period_ms *= osc_calibrate_val;

		writeReg32Bit(vl53, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

		// vl53[which]L0X_SetInterMeasurementPeriodMilliSeconds() end

		writeReg(vl53, SYSRANGE_START, 0x04); // vl53L0X_REG_SYSRANGE_MODE_TIMED
	} else {
		// continuous back-to-back mode
		writeReg(vl53, SYSRANGE_START, 0x02); // vl53L0X_REG_SYSRANGE_MODE_BACKTOBACK
	}
}

void stopContinuous(struct VL53L0X *vl53) {
	writeReg(vl53, SYSRANGE_START, 0x01);
	writeReg(vl53, 0xFF, 0x01);
	writeReg(vl53, 0x00, 0x00);
	writeReg(vl53, 0x91, 0x00);
	writeReg(vl53, 0x00, 0x01);
	writeReg(vl53, 0xFF, 0x00);
}

uint16_t readRangeContinuousMillimeters(struct VL53L0X *vl53) {
	int startTime;
	startTime = HAL_GetTick();
//	HAL_I2C_Mem_Write(vl53->I2cLine,vl53 ,which->vl53[which].address,vl53 ,which->vl53[which].regAddr.RESULT_INTERRUPT_STATUS,I2C_MEMADD_SIZE_8BIT,(uint8_t *)&qaqq,1,1);
//	writeReg(vl53 , vl53 ,which->vl53[which].regAddr.RESULT_INTERRUPT_STATUS,68);
	while ((readReg(vl53, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
		if (vl53->param.io_timeout > 0
				&& ((uint16_t) (HAL_GetTick() - startTime) > vl53->param.io_timeout)) {
			vl53->param.did_timeout = true;
			return 65535;
		}
	}
	uint16_t range = readReg16Bit(vl53, RESULT_RANGE_STATUS + 10);
//	uint16_t range = readReg(vl53 , 
//			vl53 ,which->vl53[which].regAddr.RESULT_RANGE_STATUS);

	writeReg(vl53, SYSTEM_INTERRUPT_CLEAR, 0x01);

	return range;
}

uint16_t readRangeSingleMillimeters(struct VL53L0X *vl53) {
	writeReg(vl53, 0x80, 0x01);
	writeReg(vl53, 0xFF, 0x01);
	writeReg(vl53, 0x00, 0x00);
	writeReg(vl53, 0x91, vl53->param.stop_variable);
	writeReg(vl53, 0x00, 0x01);
	writeReg(vl53, 0xFF, 0x00);
	writeReg(vl53, 0x80, 0x00);

	writeReg(vl53, SYSRANGE_START, 0x01);
	int startTime;
	startTime = HAL_GetTick();
	while (readReg(vl53, SYSRANGE_START) & 0x01) {
		if (vl53->param.io_timeout > 0
				&& ((uint16_t) (HAL_GetTick() - startTime) > vl53->param.io_timeout)) {
			vl53->param.did_timeout = true;
			return 65535;
		}
	}
	return readRangeContinuousMillimeters(vl53);
}

bool timeoutOccured(struct VL53L0X *vl53, int which) {
	bool tmp = vl53->param.did_timeout;
	vl53->param.did_timeout = false;
	return tmp;
}

bool getSpadInfo(struct VL53L0X *vl53, uint8_t *count,
bool *type_is_aperture) {
	uint8_t tmp;
	getSpadInfoInit(vl53);
	int startTime;
	startTime = HAL_GetTick();
	while (readReg(vl53, 0x83) == 0x00) {
		if (vl53->param.io_timeout > 0
				&& ((uint16_t) (HAL_GetTick() - startTime) > vl53->param.io_timeout)) {
			vl53->param.did_timeout = true;
			return false;
		}

	}
	writeReg(vl53, 0x83, 0x01);
	tmp = readReg(vl53, 0x92);
	*count = tmp & 0x7f;
	*type_is_aperture = (tmp >> 7) & 0x01;
	writeReg(vl53, 0x81, 0x00);
	writeReg(vl53, 0xFF, 0x06);
	writeReg(vl53, 0x83, readReg(vl53, 0x83) & ~0x04);
	writeReg(vl53, 0xFF, 0x01);
	writeReg(vl53, 0x00, 0x01);

	writeReg(vl53, 0xFF, 0x00);
	writeReg(vl53, 0x80, 0x00);

	return true;

}

void getSpadInfoInit(struct VL53L0X *vl53) {
	writeReg(vl53, 0x80, 0x01);
	writeReg(vl53, 0xFF, 0x01);
	writeReg(vl53, 0x00, 0x00);
	writeReg(vl53, 0xFF, 0x06);
	writeReg(vl53, 0x83, readReg(vl53, 0x83) | 0x04);
	writeReg(vl53, 0xFF, 0x07);
	writeReg(vl53, 0x81, 0x01);
	writeReg(vl53, 0x80, 0x01);
	writeReg(vl53, 0x94, 0x6b);
	writeReg(vl53, 0x83, 0x00);
}

void getSequenceStepEnables(struct VL53L0X *vl53,
		struct SequenceStepEnables *enables) {
	uint8_t sequence_config = readReg(vl53, SYSTEM_SEQUENCE_CONFIG);

	enables->tcc = (sequence_config >> 4) & 0x1;
	enables->dss = (sequence_config >> 3) & 0x1;
	enables->msrc = (sequence_config >> 2) & 0x1;
	enables->pre_range = (sequence_config >> 6) & 0x1;
	enables->final_range = (sequence_config >> 7) & 0x1;
}

void getSequenceStepTimeouts(struct VL53L0X *vl53,
		struct SequenceStepEnables const *enables,
		struct SequenceStepTimeouts *timeouts) {
	timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(vl53,
			vcselPeriodType);
	timeouts->msrc_dss_tcc_mclks = readReg(vl53, MSRC_CONFIG_TIMEOUT_MACROP)
			+ 1;
	timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds(vl53,
			timeouts->msrc_dss_tcc_mclks,
			timeouts->pre_range_vcsel_period_pclks);
	timeouts->pre_range_mclks = decodeTimeout(
			readReg16Bit(vl53, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
	timeouts->pre_range_us = timeoutMclksToMicroseconds(vl53,
			timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);
	timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(vl53,
			vcselPeriodType);
	timeouts->final_range_mclks = decodeTimeout(
			readReg16Bit(vl53, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
	if (enables->pre_range)
		timeouts->final_range_mclks -= timeouts->pre_range_mclks;

	timeouts->final_range_us = timeoutMclksToMicroseconds(vl53,
			timeouts->final_range_mclks,
			timeouts->final_range_vcsel_period_pclks);
}

uint16_t decodeTimeout(uint16_t reg_val) {
	return (uint16_t) ((reg_val & 0x00FF)
			<< (uint16_t) ((reg_val & 0xFF00) >> 8)) + 1;
}

uint16_t encodeTimeout(uint32_t timeout_mclks) {

	uint32_t ls_byte = 0;
	uint16_t ms_byte = 0;
	if (timeout_mclks > 0) {
		ls_byte = timeout_mclks - 1;

		while ((ls_byte & 0xFFFFFF00) > 0) {
			ls_byte >>= 1;
			ms_byte++;
		}
		return (ms_byte << 8) | (ls_byte & 0xFF);
	} else
		return 0;
}

uint32_t timeoutMclksToMicroseconds(struct VL53L0X *vl53,
		uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks) {
	uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
	return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

uint32_t timeoutMicrosecondsToMclks(struct VL53L0X *vl53,
		uint32_t timeout_period_us, uint8_t vcsel_period_pclks) {
	uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

	return (((timeout_period_us * 1000) + (macro_period_ns / 2))
			/ macro_period_ns);
}

bool performSingleRefCalibration(struct VL53L0X *vl53, uint8_t vhv_init_byte) {
	writeReg(vl53, SYSRANGE_START, 0x01 | vhv_init_byte); // vl53[which]L0X_REG_SYSRANGE_MODE_START_STOP
	int startTime;
	startTime = HAL_GetTick();
	while ((readReg(vl53, RESULT_INTERRUPT_STATUS) & 0x07)) {
		if (vl53->param.io_timeout > 0
				&& ((uint16_t) (HAL_GetTick() - startTime) > vl53->param.io_timeout))
			return false;
	}

	writeReg(vl53, SYSTEM_INTERRUPT_CLEAR, 0x01);
	writeReg(vl53, SYSRANGE_START, 0x00);
	return true;
}

uint8_t getAddress(struct VL53L0X *vl53) {
	return readReg(vl53, I2C_SLAVE_DEVICE_ADDRESS);
}

void setTimeout(struct VL53L0X *vl53, uint16_t timeout) {
	vl53->param.io_timeout = timeout;
}
uint16_t getTimeout(struct VL53L0X *vl53) {
	return vl53->param.io_timeout;
}

void VL53L0X_load_tuning_settings(struct VL53L0X *vl53) {
	writeReg(vl53, 0xFF, 0x01);
	writeReg(vl53, 0x00, 0x00);

	writeReg(vl53, 0xFF, 0x00);
	writeReg(vl53, 0x09, 0x00);
	writeReg(vl53, 0x10, 0x00);
	writeReg(vl53, 0x11, 0x00);

	writeReg(vl53, 0x24, 0x01);
	writeReg(vl53, 0x25, 0xFF);
	writeReg(vl53, 0x75, 0x00);

	writeReg(vl53, 0xFF, 0x01);
	writeReg(vl53, 0x4E, 0x2C);
	writeReg(vl53, 0x48, 0x00);
	writeReg(vl53, 0x30, 0x20);

	writeReg(vl53, 0xFF, 0x00);
	writeReg(vl53, 0x30, 0x09);
	writeReg(vl53, 0x54, 0x00);
	writeReg(vl53, 0x31, 0x04);
	writeReg(vl53, 0x32, 0x03);
	writeReg(vl53, 0x40, 0x83);
	writeReg(vl53, 0x46, 0x25);
	writeReg(vl53, 0x60, 0x00);
	writeReg(vl53, 0x27, 0x00);
	writeReg(vl53, 0x50, 0x06);
	writeReg(vl53, 0x51, 0x00);
	writeReg(vl53, 0x52, 0x96);
	writeReg(vl53, 0x56, 0x08);
	writeReg(vl53, 0x57, 0x30);
	writeReg(vl53, 0x61, 0x00);
	writeReg(vl53, 0x62, 0x00);
	writeReg(vl53, 0x64, 0x00);
	writeReg(vl53, 0x65, 0x00);
	writeReg(vl53, 0x66, 0xA0);

	writeReg(vl53, 0xFF, 0x01);
	writeReg(vl53, 0x22, 0x32);
	writeReg(vl53, 0x47, 0x14);
	writeReg(vl53, 0x49, 0xFF);
	writeReg(vl53, 0x4A, 0x00);

	writeReg(vl53, 0xFF, 0x00);
	writeReg(vl53, 0x7A, 0x0A);
	writeReg(vl53, 0x7B, 0x00);
	writeReg(vl53, 0x78, 0x21);

	writeReg(vl53, 0xFF, 0x01);
	writeReg(vl53, 0x23, 0x34);
	writeReg(vl53, 0x42, 0x00);
	writeReg(vl53, 0x44, 0xFF);
	writeReg(vl53, 0x45, 0x26);
	writeReg(vl53, 0x46, 0x05);
	writeReg(vl53, 0x40, 0x40);
	writeReg(vl53, 0x0E, 0x06);
	writeReg(vl53, 0x20, 0x1A);
	writeReg(vl53, 0x43, 0x40);

	writeReg(vl53, 0xFF, 0x00);
	writeReg(vl53, 0x34, 0x03);
	writeReg(vl53, 0x35, 0x44);

	writeReg(vl53, 0xFF, 0x01);
	writeReg(vl53, 0x31, 0x04);
	writeReg(vl53, 0x4B, 0x09);
	writeReg(vl53, 0x4C, 0x05);
	writeReg(vl53, 0x4D, 0x04);

	writeReg(vl53, 0xFF, 0x00);
	writeReg(vl53, 0x44, 0x00);
	writeReg(vl53, 0x45, 0x20);
	writeReg(vl53, 0x47, 0x08);
	writeReg(vl53, 0x48, 0x28);
	writeReg(vl53, 0x67, 0x00);
	writeReg(vl53, 0x70, 0x04);
	writeReg(vl53, 0x71, 0x01);
	writeReg(vl53, 0x72, 0xFE);
	writeReg(vl53, 0x76, 0x00);
	writeReg(vl53, 0x77, 0x00);

	writeReg(vl53, 0xFF, 0x01);
	writeReg(vl53, 0x0D, 0x01);

	writeReg(vl53, 0xFF, 0x00);
	writeReg(vl53, 0x80, 0x01);
	writeReg(vl53, 0x01, 0xF8);

	writeReg(vl53, 0xFF, 0x01);
	writeReg(vl53, 0x8E, 0x01);
	writeReg(vl53, 0x00, 0x01);
	writeReg(vl53, 0xFF, 0x00);
	writeReg(vl53, 0x80, 0x00);
}

void VL53Ranging_DMA(struct VL53L0X *vl53) {
	HAL_I2C_Mem_Read_DMA(vl53->I2cLine, vl53->address, RESULT_RANGE_STATUS + 10,
	I2C_MEMADD_SIZE_8BIT, (uint8_t*) &(vl53->result.resultStorage), 2);
}

uint32_t VL53L0X_Ranging_Result_Decipher(struct VL53L0X *vl53) {
	uint32_t result = 0;
	if (vl53->result.resultStorage[0] <= 3) {
		result = vl53->result.resultStorage[0];
		result = result << 8;
		result |= vl53->result.resultStorage[1];
	} else
		result = 1073741824;
	return result;

}

void VL53VarInit(VL53 *vl53l0x) {
	for (int count = 0; count < AmountOfVL53; count++) {
		vl53l0x[count].param.io_timeout = 500;
		vl53l0x[count].param.did_timeout = false;
		vl53l0x[count].param.io_2v8 = true;
		vl53l0x[count].result.resultStorage[0] = 0;
		vl53l0x[count].result.resultStorage[1] = 0;
		vl53l0x[count].address = 0x52;
		vl53l0x[count].result.VL53RangingResult = 0;
		vl53l0x[count].param.dataAmount = 0;
	}
	VL53SystemInit(vl53l0x);

}

void VL53SystemInit(VL53 *vl53l0x) {
	for(int count = 0; count < AmountOfVL53;count++){
		HAL_GPIO_WritePin(vl53l0x[count].GPIO_Port, vl53l0x[count].GPIO_Pin, GPIO_PIN_RESET);
	}
	for(int count = 0; count < AmountOfVL53; count++){
		HAL_GPIO_WritePin(vl53l0x[count].GPIO_Port, vl53l0x[count].GPIO_Pin, GPIO_PIN_SET);
		HAL_Delay(50);
		vl53l0x[count].InitCheck = Init_Calibration(&(vl53l0x[count]),
				vl53l0x[count].param.io_2v8, vl53l0x[count].finalRangeSignalRate);
		setAddress(&(vl53l0x[count]), vl53l0x[count].targetAddress);
	}
	for (int count = 0; count < AmountOfVL53; count++)
		startContinuous(&(vl53l0x[count]), 0);

}

void VL53Filter(VL53 * vl53l0x){
	vl53l0x->param.dataSum += vl53l0x->result.VL53RangingResult;
	if(vl53l0x->param.dataAmount == 5){
		vl53l0x->result.VL53FilterResult = vl53l0x->param.dataSum / 5;
		vl53l0x->param.dataSum = 0;
		vl53l0x->param.dataAmount = 0;
	}

}

