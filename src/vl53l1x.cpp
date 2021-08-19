/*
  This is a library written for the VL53L1X I2C Distance sensor.

  Ported by Eric Slaghuis @ Xeni Robotics, 18 August 2021
  from the VL53L1X library for Arduino, Version 1.3.0, published
  by https://www.pololu.com on https://github.com/pololu/vl53l1x-arduino

  The sensor uses I2C to communicate, as well as a single (optional)
  interrupt line that is not currently supported in this driver.

  Development environment specifics:
  Raspberry Pi 3
  Ubuntu 20.04.2 LTS

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Most of the functionality of this library is based on the VL53L1X API
// provided by ST (STSW-IMG007), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2356), and
// VL53L1X datasheet.

#include <iostream>
#include <string>

#include "vl53l1x.h"

// Constructor /////////////////////////////////////////////////////
Vl53l1x::Vl53l1x()
  : address(AddressDefault)
  , io_timeout(0) // no timeout
  , did_timeout(false)
  , calibrated(false)
  , saved_vhv_init(0)
  , saved_vhv_timeout(0)
  , last_status(0)
  , distance_mode(Unknown)
{
}

// Public Methods //////////////////////////////////////////////////

//Open the i2c port for reading and writing
//and initialise the sensor
bool Vl53l1x::init() {
  // Open it i2c port
  if ((fd = open("/dev/i2c-1", O_RDWR)) < 0) {
      return false;
  }

  // Setup flow control
  if (ioctl(fd, I2C_SLAVE, address) < 0) {
    return false;
  }

  // Check model ID and module type registers (values specified in datasheet)
  uint16_t modelID = readReg16Bit(VL53L1_IDENTIFICATION__MODEL_ID);
  if (modelID != 0xEACC) return false;

  // VL53L1_software_reset() begin
  writeReg(SOFT_RESET, 0x00);
  std::this_thread::sleep_for(std::chrono::microseconds(100));
  writeReg(SOFT_RESET, 0x01);

  // give it some time to boot; otherwise the sensor NACKs during the readReg()
  // call below and the Arduino 101 doesn't seem to handle that well
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  // VL53L1_poll_for_boot_completion() begin

  startTimeout();

  // check last_status in case we still get a NACK to try to deal with it correctly
  while ((readReg(FIRMWARE__SYSTEM_STATUS) & 0x01) == 0 || last_status != 0)
  {
    if (checkTimeoutExpired()) {
      did_timeout = true;
      return false;
    }
  }
  // VL53L1_poll_for_boot_completion() end

  // VL53L1_software_reset() end

  // VL53L1_DataInit() begin
  // store oscillator info for later use
  fast_osc_frequency = readReg16Bit(OSC_MEASURED__FAST_OSC__FREQUENCY);
  osc_calibrate_val = readReg16Bit(RESULT__OSC_CALIBRATE_VAL);

  // VL53L1_DataInit() end


  // VL53L1_StaticInit() begin

  // Note that the API does not actually apply the configuration settings below
  // when VL53L1_StaticInit() is called: it keeps a copy of the sensor's
  // register contents in memory and doesn't actually write them until a
  // measurement is started. Writing the configuration here means we don't have
  // to keep it all in memory and avoids a lot of redundant writes later.

  // the API sets the preset mode to LOWPOWER_AUTONOMOUS here:
  // VL53L1_set_preset_mode() begin

  // VL53L1_preset_mode_standard_ranging() begin

  // values labeled "tuning parm default" are from vl53l1_tuning_parm_defaults.h
  // (API uses these in VL53L1_init_tuning_parm_storage_struct())

  // static config
  // API resets PAD_I2C_HV__EXTSUP_CONFIG here, but maybe we don't want to do
  // that? (seems like it would disable 2V8 mode)
  writeReg16Bit(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TargetRate); // should already be this value after reset
  writeReg(GPIO__TIO_HV_STATUS, 0x02);
  writeReg(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8); // tuning parm default
  writeReg(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16); // tuning parm default
  writeReg(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
  writeReg(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
  writeReg(ALGO__RANGE_MIN_CLIP, 0); // tuning parm default
  writeReg(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2); // tuning parm default

  // general config
  writeReg16Bit(SYSTEM__THRESH_RATE_HIGH, 0x0000);
  writeReg16Bit(SYSTEM__THRESH_RATE_LOW, 0x0000);
  writeReg(DSS_CONFIG__APERTURE_ATTENUATION, 0x38);

  // timing config
  // most of these settings will be determined later by distance and timing
  // budget configuration
  writeReg16Bit(RANGE_CONFIG__SIGMA_THRESH, 360); // tuning parm default
  writeReg16Bit(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192); // tuning parm default

  // dynamic config
  writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
  writeReg(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
  writeReg(SD_CONFIG__QUANTIFIER, 2); // tuning parm default

  // VL53L1_preset_mode_standard_ranging() end

  // from VL53L1_preset_mode_timed_ranging_*
  // GPH is 0 after reset, but writing GPH0 and GPH1 above seem to set GPH to 1,
  // and things don't seem to work if we don't set GPH back to 0 (which the API
  // does here).
  writeReg(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
  writeReg(SYSTEM__SEED_CONFIG, 1); // tuning parm default

  // from VL53L1_config_low_power_auto_mode
  writeReg(SYSTEM__SEQUENCE_CONFIG, 0x8B); // VHV, PHASECAL, DSS1, RANGE
  writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
  writeReg(DSS_CONFIG__ROI_MODE_CONTROL, 2); // REQUESTED_EFFFECTIVE_SPADS

  // VL53L1_set_preset_mode() end

  // default to long range, 50 ms timing budget
  // note that this is different than what the API defaults to
  setDistanceMode(Long);
  setMeasurementTimingBudget(50000);

  // VL53L1_StaticInit() end

  // the API triggers this change in VL53L1_init_and_start_range() once a
  // measurement is started; assumes MM1 and MM2 are disabled
  writeReg16Bit(ALGO__PART_TO_PART_RANGE_OFFSET_MM,
    readReg16Bit(MM_CONFIG__OUTER_OFFSET_MM) * 4);

  return true;
}

void Vl53l1x::setAddress(uint8_t new_addr)
{
  uint8_t na = new_addr & 0x7F;
  writeReg(I2C_SLAVE__DEVICE_ADDRESS, na);
  address = new_addr;
}


// set distance mode to Short, Medium, or Long
// based on VL53L1_SetDistanceMode()
bool Vl53l1x::setDistanceMode(DistanceMode mode)
{
  // save existing timing budget
  uint32_t budget_us = getMeasurementTimingBudget();

  switch (mode)
  {
    case Short:
      // from VL53L1_preset_mode_standard_ranging_short_range()

      // timing config
      writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
      writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
      writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);

      // dynamic config
      writeReg(SD_CONFIG__WOI_SD0, 0x07);
      writeReg(SD_CONFIG__WOI_SD1, 0x05);
      writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 6); // tuning parm default
      writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 6); // tuning parm default

      break;

    case Medium:
      // from VL53L1_preset_mode_standard_ranging()

      // timing config
      writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
      writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
      writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);

      // dynamic config
      writeReg(SD_CONFIG__WOI_SD0, 0x0B);
      writeReg(SD_CONFIG__WOI_SD1, 0x09);
      writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 10); // tuning parm default
      writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 10); // tuning parm default

      break;

    case Long: // long
      // from VL53L1_preset_mode_standard_ranging_long_range()

      // timing config
      writeReg(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
      writeReg(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
      writeReg(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);

      // dynamic config
      writeReg(SD_CONFIG__WOI_SD0, 0x0F);
      writeReg(SD_CONFIG__WOI_SD1, 0x0D);
      writeReg(SD_CONFIG__INITIAL_PHASE_SD0, 14); // tuning parm default
      writeReg(SD_CONFIG__INITIAL_PHASE_SD1, 14); // tuning parm default

      break;

    default:
      // unrecognized mode - do nothing
      return false;
  }

  // reapply timing budget
  setMeasurementTimingBudget(budget_us);

  // save mode so it can be returned by getDistanceMode()
  distance_mode = mode;

  return true;
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement. A longer timing budget allows for more accurate
// measurements.
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
bool Vl53l1x::setMeasurementTimingBudget(uint32_t budget_us)
{
  // assumes PresetMode is LOWPOWER_AUTONOMOUS

  if (budget_us <= TimingGuard) { return false; }

  uint32_t range_config_timeout_us = budget_us -= TimingGuard;
  if (range_config_timeout_us > 1100000) { return false; } // FDA_MAX_TIMING_BUDGET_US * 2

  range_config_timeout_us /= 2;

  // VL53L1_calc_timeout_register_values() begin

  uint32_t macro_period_us;

  // "Update Macro Period for Range A VCSEL Period"
  macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_A));

  // "Update Phase timeout - uses Timing A"
  // Timeout of 1000 is tuning parm default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
  // via VL53L1_get_preset_mode_timing_cfg().
  uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
  if (phasecal_timeout_mclks > 0xFF) { phasecal_timeout_mclks = 0xFF; }
  writeReg(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);

  // "Update MM Timing A timeout"
  // Timeout of 1 is tuning parm default (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
  // via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
  // actually ends up with a slightly different value because it gets assigned,
  // retrieved, recalculated with a different macro period, and reassigned,
  // but it probably doesn't matter because it seems like the MM ("mode
  // mitigation"?) sequence steps are disabled in low power auto mode anyway.
  writeReg16Bit(MM_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
    timeoutMicrosecondsToMclks(1, macro_period_us)));

  // "Update Range Timing A timeout"
  writeReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
    timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

  // "Update Macro Period for Range B VCSEL Period"
  macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_B));

  // "Update MM Timing B timeout"
  // (See earlier comment about MM Timing A timeout.)
  writeReg16Bit(MM_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
    timeoutMicrosecondsToMclks(1, macro_period_us)));

  // "Update Range Timing B timeout"
  writeReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
    timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

  // VL53L1_calc_timeout_register_values() end

  return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
uint32_t Vl53l1x::getMeasurementTimingBudget()
{
  // assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
  // enabled: VHV, PHASECAL, DSS1, RANGE

  // VL53L1_get_timeouts_us() begin

  // "Update Macro Period for Range A VCSEL Period"
  uint32_t macro_period_us = calcMacroPeriod(readReg(RANGE_CONFIG__VCSEL_PERIOD_A));

  // "Get Range Timing A timeout"

  uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(decodeTimeout(
    readReg16Bit(RANGE_CONFIG__TIMEOUT_MACROP_A)), macro_period_us);

  // VL53L1_get_timeouts_us() end

  return  2 * range_config_timeout_us + TimingGuard;
}


// Start continuous ranging measurements, with the given inter-measurement
// period in milliseconds determining how often the sensor takes a measurement.
void Vl53l1x::startContinuous(uint32_t period_ms)
{
  // from VL53L1_set_inter_measurement_period_ms()
  writeReg32Bit(SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * osc_calibrate_val);

  writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
  writeReg(SYSTEM__MODE_START, 0x40); // mode_range__timed
}

// Stop continuous measurements
// based on VL53L1_stop_range()
void Vl53l1x::stopContinuous()
{
  writeReg(SYSTEM__MODE_START, 0x80); // mode_range__abort

  // VL53L1_low_power_auto_data_stop_range() begin

  calibrated = false;

  // "restore vhv configs"
  if (saved_vhv_init != 0)
  {
    writeReg(VHV_CONFIG__INIT, saved_vhv_init);
  }
  if (saved_vhv_timeout != 0)
  {
     writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, saved_vhv_timeout);
  }

  // "remove phasecal override"
  writeReg(PHASECAL_CONFIG__OVERRIDE, 0x00);

  // VL53L1_low_power_auto_data_stop_range() end
}

// Returns a range reading in millimeters when continuous mode is active. If
// blocking is true (the default), this function waits for a new measurement to
// be available. If blocking is false, it will try to return data immediately.
// (readSingle() also calls this function after starting a single-shot range
// measurement)
uint16_t Vl53l1x::read_range(bool blocking)
{
  if (blocking)
  {
    startTimeout();
    while (!dataReady())
    {
      if (checkTimeoutExpired())
      {
        did_timeout = true;
        return 0;
      }
    }
  }

  readResults();

  if (!calibrated)
  {
    setupManualCalibration();
    calibrated = true;
  }

  updateDSS();

  getRangingData();

  writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range

  return ranging_data.range_mm;
}

// Starts a single-shot range measurement. If blocking is true (the default),
// this function waits for the measurement to finish and returns the reading.
// Otherwise, it returns 0 immediately.
uint16_t Vl53l1x::readSingle(bool blocking)
{
  writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
  writeReg(SYSTEM__MODE_START, 0x10); // mode_range__single_shot

  if (blocking)
  {
    return read_range(true);
  }
  else
  {
    return 0;
  }
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
bool Vl53l1x::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

// PRIVATE FUNCTIONS //////////////////////////////////////////////////////////

// "Setup ranges after the first one in low power auto mode by turning off
// FW calibration steps and programming static values"
// based on VL53L1_low_power_auto_setup_manual_calibration()
void Vl53l1x::setupManualCalibration()
{
  // "save original vhv configs"
  saved_vhv_init = readReg(VHV_CONFIG__INIT);
  saved_vhv_timeout = readReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND);

  // "disable VHV init"
  writeReg(VHV_CONFIG__INIT, saved_vhv_init & 0x7F);

  // "set loop bound to tuning param"
  writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
    (saved_vhv_timeout & 0x03) + (3 << 2)); // tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)

  // "override phasecal"
  writeReg(PHASECAL_CONFIG__OVERRIDE, 0x01);
  writeReg(CAL_CONFIG__VCSEL_START, readReg(PHASECAL_RESULT__VCSEL_START));
}

//Write an 8-bit register
void Vl53l1x::writeReg(uint16_t reg, uint8_t value)
{
  uint8_t writeBuffer[3];

  writeBuffer[0] = ((reg >> 8) & 0xFF);  // reg high byte
  writeBuffer[1] = ( reg       & 0xFF);  // reg low byte
  writeBuffer[2] = value;

  auto result = write(fd, writeBuffer, sizeof(writeBuffer));
  last_status = (result > 0) ? 0 : 1;
}

//Write an 16-bit register
void Vl53l1x::writeReg16Bit(uint16_t reg, uint16_t value)
{
  uint8_t writeBuffer[4];

  writeBuffer[0] = ((reg >> 8)   & 0xFF);  // reg high byte
  writeBuffer[1] = ( reg         & 0xFF);  // reg low byte
  writeBuffer[2] = ((value >> 8) & 0xFF);  // value high byte
  writeBuffer[3] = ( value       & 0xFF);  // value low byte

  auto result = write(fd, writeBuffer, sizeof(writeBuffer));
  last_status = (result > 0) ? 0 : 1;
}


// Write a 32-bit register
void Vl53l1x::writeReg32Bit(uint16_t reg, uint32_t value)
{
  uint8_t writeBuffer[6];

  writeBuffer[0] = ((reg >> 8)   & 0xFF);  // reg high byte
  writeBuffer[1] = ( reg         & 0xFF);  // reg low byte
  writeBuffer[2] = ((value >> 24) & 0xFF);  // value highest byte
  writeBuffer[3] = ((value >> 16) & 0xFF);
  writeBuffer[4] = ((value >>  8) & 0xFF);
  writeBuffer[5] = ( value       & 0xFF);  // value lowest byte

  auto result = write(fd, writeBuffer, sizeof(writeBuffer));
  last_status = (result > 0) ? 0 : 1;
}

// Read an 8-bit register
uint8_t Vl53l1x::readReg(uint16_t reg)
{
  uint8_t  writeBuffer[2];

  writeBuffer[0] = ((reg >> 8) & 0xFF);  // reg high byte
  writeBuffer[1] = (reg        & 0xFF);  // reg low byte


  if( write(fd, writeBuffer, sizeof(writeBuffer)) < sizeof(writeBuffer)) {
    //something went wrong
    std::cout << "DEBUG: ERROR on write to i2c" << std::endl;
    return 0x00;
  }

  uint8_t readBuffer[1];
  read(fd, readBuffer, sizeof(readBuffer));

  return readBuffer[0];
}

//Read 16 bits from a register
uint16_t Vl53l1x::readReg16Bit(uint16_t reg) {

  uint8_t  writeBuffer[2];
  writeBuffer[0] = ((reg >> 8) & 0xFF);  // reg high byte
  writeBuffer[1] = (reg        & 0xFF);  // reg low byte

  if( write(fd, writeBuffer, sizeof(writeBuffer)) < sizeof(writeBuffer)) {
    //something went wrong
    std::cout << "ERROR on write to i2c" << std::endl;
    return 0x0000;
  }

  uint8_t readBuffer[2];
  if( read(fd, readBuffer, sizeof(readBuffer)) < sizeof(readBuffer)) {
    std::cout << "Error on read from i2c" << std::endl;
    return 0x0000;
  }

  uint16_t value = (uint16_t)readBuffer[0] << 8;  //value high byte
  value |= readBuffer[1];                         // value low byte

  return value;
}

void Vl53l1x::startTimeout() {
  timeout_start_ms = std::chrono::system_clock::now();
}

bool Vl53l1x::checkTimeoutExpired() {
    // record end time
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> diff = end - timeout_start_ms;

    return (io_timeout > 0) && ((uint16_t)(diff.count()*1000) > io_timeout);
}

// Calculate macro period in microseconds (12.12 format) with given VCSEL period
// assumes fast_osc_frequency has been read and stored
// based on VL53L1_calc_macro_period_us()
uint32_t Vl53l1x::calcMacroPeriod(uint8_t vcsel_period)
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
uint32_t Vl53l1x::timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
  return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

// Convert sequence step timeout from macro periods to microseconds with given
// macro period in microseconds (12.12 format)
// based on VL53L1_calc_timeout_us()
uint32_t Vl53l1x::timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
  return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L1_decode_timeout()
uint32_t Vl53l1x::decodeTimeout(uint16_t reg_val)
{
  return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L1_encode_timeout()
uint16_t Vl53l1x::encodeTimeout(uint32_t timeout_mclks)
{
  // encoded format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// read measurement results into buffer
void Vl53l1x::readResults()
{
  uint8_t  writeBuffer[2];
  writeBuffer[0] = ((RESULT__RANGE_STATUS >> 8) & 0xFF);  // reg high byte
  writeBuffer[1] = ( RESULT__RANGE_STATUS       & 0xFF);  // reg low byte

  if( write(fd, writeBuffer, sizeof(writeBuffer)) < sizeof(writeBuffer)) {
    //something went wrong
    std::cout << "ERROR on write to i2c" << std::endl;
    return;
  }

  uint8_t readBuffer[17];
  if( read(fd, readBuffer, sizeof(readBuffer)) < sizeof(readBuffer)) {
    std::cout << "Error on read from i2c" << std::endl;
    return;
  }

  results.range_status = readBuffer[0];
  //[1] report_status: not used
  results.stream_count = readBuffer[2];

  results.dss_actual_effective_spads_sd0  = (uint16_t)readBuffer[3] << 8; // high byte
  results.dss_actual_effective_spads_sd0 |=           readBuffer[4];      // low byte

  //[5] peak_signal_count_rate_mcps_sd0: not used
  //[6]

  results.ambient_count_rate_mcps_sd0  = (uint16_t)readBuffer[7] << 8; // high byte
  results.ambient_count_rate_mcps_sd0 |=           readBuffer[8];      // low byte

  //[9] sigma_sd0: not used
  //[10]

  //[11] phase_sd0: not used
  //[12]

  results.final_crosstalk_corrected_range_mm_sd0  = (uint16_t)readBuffer[13] << 8; // high byte
  results.final_crosstalk_corrected_range_mm_sd0 |=           readBuffer[14];      // low byte

  results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0  = (uint16_t)readBuffer[15] << 8; // high byte
  results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 |=           readBuffer[16];      // low byte
}

// perform Dynamic SPAD Selection calculation/update
// based on VL53L1_low_power_auto_update_DSS()
void Vl53l1x::updateDSS()
{
  uint16_t spadCount = results.dss_actual_effective_spads_sd0;

  if (spadCount != 0)
  {
    // "Calc total rate per spad"

    uint32_t totalRatePerSpad =
      (uint32_t)results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 +
      results.ambient_count_rate_mcps_sd0;

    // "clip to 16 bits"
    if (totalRatePerSpad > 0xFFFF) { totalRatePerSpad = 0xFFFF; }

    // "shift up to take advantage of 32 bits"
    totalRatePerSpad <<= 16;

    totalRatePerSpad /= spadCount;

    if (totalRatePerSpad != 0)
    {
      // "get the target rate and shift up by 16"
      uint32_t requiredSpads = ((uint32_t)TargetRate << 16) / totalRatePerSpad;

      // "clip to 16 bit"
      if (requiredSpads > 0xFFFF) { requiredSpads = 0xFFFF; }

      // "override DSS config"
      writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads);
      // DSS_CONFIG__ROI_MODE_CONTROL should already be set to REQUESTED_EFFFECTIVE_SPADS

      return;
    }
  }

  // If we reached this point, it means something above would have resulted in a
  // divide by zero.
  // "We want to gracefully set a spad target, not just exit with an error"

   // "set target to mid point"
   writeReg16Bit(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
}


// get range, status, rates from results buffer
// based on VL53L1_GetRangingMeasurementData()
void Vl53l1x::getRangingData()
{
  // VL53L1_copy_sys_and_core_results_to_range_results() begin

  uint16_t range = results.final_crosstalk_corrected_range_mm_sd0;

  // "apply correction gain"
  // gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
  // Basically, this appears to scale the result by 2011/2048, or about 98%
  // (with the 1024 added for proper rounding).
  ranging_data.range_mm = ((uint32_t)range * 2011 + 0x0400) / 0x0800;

  // VL53L1_copy_sys_and_core_results_to_range_results() end

  // set range_status in ranging_data based on value of RESULT__RANGE_STATUS register
  // mostly based on ConvertStatusLite()
  switch(results.range_status)
  {
    case 17: // MULTCLIPFAIL
    case 2: // VCSELWATCHDOGTESTFAILURE
    case 1: // VCSELCONTINUITYTESTFAILURE
    case 3: // NOVHVVALUEFOUND
      // from SetSimpleData()
      ranging_data.range_status = HardwareFail;
      break;

    case 13: // USERROICLIP
     // from SetSimpleData()
      ranging_data.range_status = MinRangeFail;
      break;

    case 18: // GPHSTREAMCOUNT0READY
      ranging_data.range_status = SynchronizationInt;
      break;

    case 5: // RANGEPHASECHECK
      ranging_data.range_status =  OutOfBoundsFail;
      break;

    case 4: // MSRCNOTARGET
      ranging_data.range_status = SignalFail;
      break;

    case 6: // SIGMATHRESHOLDCHECK
      ranging_data.range_status = SigmaFail;
      break;

    case 7: // PHASECONSISTENCY
      ranging_data.range_status = WrapTargetFail;
      break;

    case 12: // RANGEIGNORETHRESHOLD
      ranging_data.range_status = XtalkSignalFail;
      break;

    case 8: // MINCLIP
      ranging_data.range_status = RangeValidMinRangeClipped;
      break;

    case 9: // RANGECOMPLETE
      // from VL53L1_copy_sys_and_core_results_to_range_results()
      if (results.stream_count == 0)
      {
        ranging_data.range_status = RangeValidNoWrapCheckFail;
      }
      else
      {
        ranging_data.range_status = RangeValid;
      }
      break;

    default:
      ranging_data.range_status = None;
  }

  // from SetSimpleData()
  ranging_data.peak_signal_count_rate_MCPS =
    countRateFixedToFloat(results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
  ranging_data.ambient_count_rate_MCPS =
    countRateFixedToFloat(results.ambient_count_rate_mcps_sd0);
}
