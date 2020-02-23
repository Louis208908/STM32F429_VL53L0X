#ifndef VL53L0X_h
#define VL53L0X_h
#include "header.h"
#include "i2c.h"
#include "gpio.h"

#define SamplingAmount 10
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)


//typedef struct VL53TargetAddress{
//	uint8_t VL53Address;
//}VL53Addr;


enum regAddr
{
  SYSRANGE_START                              = 0x00,

  SYSTEM_THRESH_HIGH                          = 0x0C,
  SYSTEM_THRESH_LOW                           = 0x0E,

  SYSTEM_SEQUENCE_CONFIG                      = 0x01,
  SYSTEM_RANGE_CONFIG                         = 0x09,
  SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

  SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

  GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

  SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

  RESULT_INTERRUPT_STATUS                     = 0x13,
  RESULT_RANGE_STATUS                         = 0x14,

  RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
  RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
  RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
  RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
  RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

  ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

  I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

  MSRC_CONFIG_CONTROL                         = 0x60,

  PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
  PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
  PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
  PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

  FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
  FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
  FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
  FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

  PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
  PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

  PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

  SYSTEM_HISTOGRAM_BIN                        = 0x81,
  HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
  HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

  FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
  CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

  MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

  SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
  IDENTIFICATION_MODEL_ID                     = 0xC0,
  IDENTIFICATION_REVISION_ID                  = 0xC2,

  OSC_CALIBRATE_VAL                           = 0xF8,

  GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

  GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
  DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
  DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
  POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

  VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

  ALGO_PHASECAL_LIM                           = 0x30,
  ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
};
// register addresses from API vl53l0x_device.h (ordered as listed there)

typedef struct RESULT{
	volatile uint8_t resultStorage[2];
	uint32_t VL53RangingResult;
	uint32_t VL53FilterResult;
}Result;

typedef struct InnerParam{
	bool did_timeout;
	uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
	uint32_t measurement_timing_budget_us;
	uint8_t last_status; // status of last I2C transmission
	uint16_t io_timeout;
	bool io_2v8;
	uint32_t dataSum;
	int dataAmount;

}InnerParam;

typedef struct VL53L0X {
	uint8_t targetAddress;
	uint8_t address;
	struct SequenceStepEnables {
		bool tcc, msrc, dss, pre_range, final_range;
	} sequenceStepEnables;

	struct SequenceStepTimeouts {
		uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
		uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
		uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
	} sequenceStepTimeouts;


	float finalRangeSignalRate;
	int VL53_RxCplt;

	bool InitCheck;
	I2C_HandleTypeDef * I2cLine;
	GPIO_TypeDef * GPIO_Port;
	uint16_t GPIO_Pin;
	Result result;
	InnerParam param;
}VL53;


enum VcselPeriodType {
	VcselPeriodPreRange = 0, VcselPeriodFinalRange
} vcselPeriodType;



bool Init_Calibration(VL53 * vl53, bool io_2v8, float finalRangeSiganalRate);
void setAddress(VL53 * vl53, uint8_t new_addr);
uint8_t getAddress(VL53 * vl53);

void writeReg(VL53 * vl53, uint8_t reg, uint8_t value);
void writeReg16Bit(VL53 * vl53, uint8_t reg, uint16_t value);
void writeReg32Bit(VL53 * vl53, uint8_t reg, uint32_t value);
uint8_t readReg(VL53 * vl53, uint8_t reg);
uint16_t readReg16Bit(VL53 * vl53, uint8_t reg);
uint32_t readReg32Bit(VL53 * vl53, uint8_t reg);

void writeMulti(VL53 * vl53, uint8_t reg, uint8_t const * src,
		uint8_t count);
void readMulti(VL53 * vl53, uint8_t reg, uint8_t * dst, uint8_t count);

bool setSignalRateLimit(VL53 * vl53, float limit_Mcps);
float getSignalRateLimit(VL53 * vl53);

bool setMeasurementTimingBudget(VL53 * vl53, uint32_t budget_us);
uint32_t getMeasurementTimingBudget(VL53 * vl53);

bool setVcselPulsePeriod(VL53 * vl53, enum VcselPeriodType type,
		uint8_t period_pclks);
uint8_t getVcselPulsePeriod(VL53 * vl53, enum VcselPeriodType type);

void startContinuous(VL53 * vl53, uint32_t period_ms);
void stopContinuous(VL53 * vl53);
uint16_t readRangeContinuousMillimeters(VL53 * vl53);
uint16_t readRangeSingleMillimeters(VL53 * vl53);
void setTimeout(VL53 * vl53, uint16_t timeout);
uint16_t getTimeout(VL53 * vl53);
bool timeoutOccurred(void);

//	// TCC: Target CentreCheck
//	// MSRC: Minimum Signal Rate Check
//	// DSS: Dynamic Spad Selection

bool getSpadInfo(VL53 * vl53, uint8_t * count, bool * type_is_aperture);

void getSequenceStepEnables(VL53 * vl53,
		struct SequenceStepEnables * enables);
void getSequenceStepTimeouts(VL53 * vl53,
		struct SequenceStepEnables const * enables,
		struct SequenceStepTimeouts * timeouts);

bool performSingleRefCalibration(VL53 * vl53, uint8_t vhv_init_byte);

uint16_t decodeTimeout(uint16_t value);
uint16_t encodeTimeout(uint32_t timeout_mclks);
uint32_t timeoutMclksToMicroseconds(VL53 * vl53,
		uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
uint32_t timeoutMicrosecondsToMclks(VL53 * vl53,
		uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
void VL53L0X_load_tuning_settings(VL53 * vl53);
void setI2CStdMode(VL53 * vl53);
void getSpadInfoInit(VL53 * vl53);
void VL53Ranging_DMA(VL53 * vl53);
uint32_t VL53L0X_Ranging_Result_Decipher(VL53 * vl53);
void VL53SystemInit(VL53 * vl53);
void VL53VarInit(VL53 * vl53);
void VL53Filter(VL53 * vl53l0x);

#endif

