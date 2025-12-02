#include "esphome/core/log.h"
#include "xensiv_dps3xx_base.h"
#include "dps3xx_config.h"

namespace esphome {
namespace xensiv_dps3xx_base {
static const char *const TAG = "xensiv_dps3xx.component";

// Scaling factors for oversampling rates
static const int32_t SCALING_FACTORS[8] = {
    524288,   // OSR 1
    1572864,  // OSR 2
    3670016,  // OSR 4
    7864320,  // OSR 8
    253952,   // OSR 16
    516096,   // OSR 32
    1040384,  // OSR 64
    2088960,  // OSR 128
};

void XensivDPS3xxComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up DPS3xx...");

  // Initialize the sensor
  if (!this->init_sensor_()) {
    ESP_LOGE(TAG, "Failed to initialize sensor");
    this->mark_failed();
    return;
  }

  // Configure interrupt pin if specified
  if (this->interrupt_pin_ != nullptr) {
    this->interrupt_pin_->setup();

    // Enable pressure and temperature ready interrupts (INT_SEL = 011 = 3)
    if (!this->write_byte_bitfield_(dps3xx::registers[dps3xx::INT_SEL], 3)) {
      ESP_LOGE(TAG, "Failed to set INT_SEL");
      this->mark_failed();
      return;
    }

    // Set interrupt polarity
    uint8_t int_hl = this->interrupt_polarity_high_ ? 1 : 0;
    if (!this->write_byte_bitfield_(dps3xx::registers[dps3xx::INT_HL], int_hl)) {
      ESP_LOGE(TAG, "Failed to set INT_HL");
      this->mark_failed();
      return;
    }
  }

  ESP_LOGCONFIG(TAG, "DPS3xx setup complete");
}

bool XensivDPS3xxComponent::init_sensor_() {
  // Wait for sensor to be ready
  delay(50);

  // Read product ID
  if (!this->read_byte(dps3xx::registers[dps3xx::PROD_ID].regAddress, &this->product_id_)) {
    ESP_LOGE(TAG, "Failed to read product ID");
    return false;
  }
  ESP_LOGD(TAG, "Product ID: 0x%02X", this->product_id_);

  // Read revision ID
  if (!this->read_byte(dps3xx::registers[dps3xx::REV_ID].regAddress, &this->revision_id_)) {
    ESP_LOGE(TAG, "Failed to read revision ID");
    return false;
  }
  ESP_LOGD(TAG, "Revision ID: 0x%02X", this->revision_id_);

  // Read temperature sensor recommendation
  uint8_t temp_coef_srce = 0;
  if (!this->read_byte_bitfield_(dps3xx::registers[dps3xx::TEMP_SENSORREC], &temp_coef_srce)) {
    ESP_LOGE(TAG, "Failed to read temperature sensor recommendation");
    return false;
  }
  this->temp_sensor_ = temp_coef_srce;
  ESP_LOGD(TAG, "Temperature sensor: %d", this->temp_sensor_);

  // Read calibration coefficients
  if (!this->read_calibration_coefficients_()) {
    ESP_LOGE(TAG, "Failed to read calibration coefficients");
    return false;
  }

  // Configure sensor
  if (!this->configure_sensor_()) {
    ESP_LOGE(TAG, "Failed to configure sensor");
    return false;
  }

  // Perform initial temperature measurement (needed for accurate pressure)
  this->measure_now_();

  return true;
}

bool XensivDPS3xxComponent::read_calibration_coefficients_() {
  // Read 18 bytes of calibration coefficients starting from 0x10
  uint8_t buffer[18];
  if (!this->read_bytes(dps3xx::coeffBlock.regAddress, buffer, dps3xx::coeffBlock.length)) {
    return false;
  }

  // Extract c0 [3:0] (12-bit)
  int32_t c0 = ((uint32_t) buffer[0] << 4) | (((uint32_t) buffer[1] >> 4) & 0x0F);
  c0 = this->get_twos_complement_(c0, 12);
  this->c0_half_ = c0 / 2;

  // Extract c1 [19:8] (12-bit)
  int32_t c1 = (((uint32_t) buffer[1] & 0x0F) << 8) | (uint32_t) buffer[2];
  this->c1_ = this->get_twos_complement_(c1, 12);

  // Extract c00 [19:0] (20-bit)
  int32_t c00 = ((uint32_t) buffer[3] << 12) | ((uint32_t) buffer[4] << 4) | (((uint32_t) buffer[5] >> 4) & 0x0F);
  this->c00_ = this->get_twos_complement_(c00, 20);

  // Extract c10 [19:0] (20-bit)
  int32_t c10 = (((uint32_t) buffer[5] & 0x0F) << 16) | ((uint32_t) buffer[6] << 8) | (uint32_t) buffer[7];
  this->c10_ = this->get_twos_complement_(c10, 20);

  // Extract c01 [15:0] (16-bit)
  int32_t c01 = ((uint32_t) buffer[8] << 8) | (uint32_t) buffer[9];
  this->c01_ = this->get_twos_complement_(c01, 16);

  // Extract c11 [15:0] (16-bit)
  int32_t c11 = ((uint32_t) buffer[10] << 8) | (uint32_t) buffer[11];
  this->c11_ = this->get_twos_complement_(c11, 16);

  // Extract c20 [15:0] (16-bit)
  int32_t c20 = ((uint32_t) buffer[12] << 8) | (uint32_t) buffer[13];
  this->c20_ = this->get_twos_complement_(c20, 16);

  // Extract c21 [15:0] (16-bit)
  int32_t c21 = ((uint32_t) buffer[14] << 8) | (uint32_t) buffer[15];
  this->c21_ = this->get_twos_complement_(c21, 16);

  // Extract c30 [15:0] (16-bit)
  int32_t c30 = ((uint32_t) buffer[16] << 8) | (uint32_t) buffer[17];
  this->c30_ = this->get_twos_complement_(c30, 16);

  ESP_LOGD(TAG, "Calibration coefficients:");
  ESP_LOGD(TAG, "  c0=%d, c1=%d", this->c0_half_ * 2, this->c1_);
  ESP_LOGD(TAG, "  c00=%d, c10=%d", this->c00_, this->c10_);
  ESP_LOGD(TAG, "  c01=%d, c11=%d", this->c01_, this->c11_);
  ESP_LOGD(TAG, "  c20=%d, c21=%d, c30=%d", this->c20_, this->c21_, this->c30_);

  return true;
}

bool XensivDPS3xxComponent::configure_sensor_() {
  // Configure temperature measurement rate and oversampling
  uint8_t tmp_cfg = (this->temp_mr_ << 4) | this->temp_osr_;
  if (!this->write_byte(dps::config_registers[dps::TEMP_MR].regAddress, tmp_cfg)) {
    return false;
  }

  // Configure pressure measurement rate and oversampling
  uint8_t prs_cfg = (this->prs_mr_ << 4) | this->prs_osr_;
  if (!this->write_byte(dps::config_registers[dps::PRS_MR].regAddress, prs_cfg)) {
    return false;
  }

  // Enable shift for temperature if OSR > 8
  if (this->temp_osr_ > DPS__OVERSAMPLING_RATE_8) {
    if (!this->write_byte_bitfield_(dps::config_registers[dps::TEMP_SHIFT_EN], 1)) {
      return false;
    }
  }

  // Enable shift for pressure if OSR > 8
  if (this->prs_osr_ > DPS__OVERSAMPLING_RATE_8) {
    if (!this->write_byte_bitfield_(dps::config_registers[dps::PRS_SHIFT_EN], 1)) {
      return false;
    }
  }

  // Set temperature sensor (internal/external)
  if (!this->write_byte_bitfield_(dps3xx::registers[dps3xx::TEMP_SENSOR], this->temp_sensor_)) {
    return false;
  }

  ESP_LOGD(TAG, "Sensor configured: TMP_CFG=0x%02X, PRS_CFG=0x%02X", tmp_cfg, prs_cfg);
  return true;
}

bool XensivDPS3xxComponent::read_pressure_temperature_(float *pressure, float *temperature) {
  // Read 3 bytes of pressure starting at 0x00
  uint8_t prs_buffer[3];
  if (!this->read_bytes(dps::registerBlocks[dps::PRS].regAddress, prs_buffer, 3)) {
    return false;
  }

  // Read 3 bytes of temperature starting at 0x03
  uint8_t tmp_buffer[3];
  if (!this->read_bytes(dps::registerBlocks[dps::TEMP].regAddress, tmp_buffer, 3)) {
    return false;
  }

  // Extract raw pressure (24-bit two's complement)
  int32_t raw_prs = ((uint32_t) prs_buffer[0] << 16) | ((uint32_t) prs_buffer[1] << 8) | (uint32_t) prs_buffer[2];
  raw_prs = this->get_twos_complement_(raw_prs, 24);

  // Extract raw temperature (24-bit two's complement)
  int32_t raw_tmp = ((uint32_t) tmp_buffer[0] << 16) | ((uint32_t) tmp_buffer[1] << 8) | (uint32_t) tmp_buffer[2];
  raw_tmp = this->get_twos_complement_(raw_tmp, 24);

  // Calculate scaled temperature and pressure
  *temperature = this->calculate_temperature_(raw_tmp);
  *pressure = this->calculate_pressure_(raw_prs, *temperature);

  return true;
}

float XensivDPS3xxComponent::calculate_temperature_(int32_t raw_temp) {
  float scaled_temp = (float) raw_temp / SCALING_FACTORS[this->temp_osr_];
  float compensated_temp = (float) this->c0_half_ + scaled_temp * (float) this->c1_;
  return compensated_temp;
}

float XensivDPS3xxComponent::calculate_pressure_(int32_t raw_pressure, float temperature) {
  float scaled_prs = (float) raw_pressure / SCALING_FACTORS[this->prs_osr_];
  float scaled_temp = temperature;  // Already compensated

  float compensated_prs = (float) this->c00_;
  compensated_prs +=
      scaled_prs * ((float) this->c10_ + scaled_prs * ((float) this->c20_ + scaled_prs * (float) this->c30_));
  compensated_prs +=
      scaled_temp * ((float) this->c01_ + scaled_prs * ((float) this->c11_ + scaled_prs * (float) this->c21_));

  return compensated_prs;
}

void XensivDPS3xxComponent::measure_now_() {
  // Trigger a single pressure measurement (which also triggers temperature)
  // Write to MSR_CTRL register (0x08) with CMD_PRS (0x01)
  this->write_byte(dps::config_registers[dps::MSR_CTRL].regAddress, dps::CMD_PRS);
}

bool XensivDPS3xxComponent::read_byte_bitfield_(const RegMask_t &reg_mask, uint8_t *value) {
  uint8_t reg_value;
  if (!this->read_byte(reg_mask.regAddress, &reg_value)) {
    return false;
  }
  *value = (reg_value & reg_mask.mask) >> reg_mask.shift;
  return true;
}

bool XensivDPS3xxComponent::write_byte_bitfield_(const RegMask_t &reg_mask, uint8_t value) {
  uint8_t reg_value;
  if (!this->read_byte(reg_mask.regAddress, &reg_value)) {
    return false;
  }
  reg_value &= ~reg_mask.mask;
  reg_value |= (value << reg_mask.shift) & reg_mask.mask;
  return this->write_byte(reg_mask.regAddress, reg_value);
}

int32_t XensivDPS3xxComponent::get_twos_complement_(int32_t val, uint8_t bits) {
  if (val & ((uint32_t) 1 << (bits - 1))) {
    return val - ((uint32_t) 1 << bits);
  }
  return val;
}

void XensivDPS3xxComponent::update() {
  if (this->is_failed()) {
    return;
  }

  // Trigger a measurement
  this->measure_now_();

  // Wait for measurement to complete (depends on OSR settings)
  // For now, use a simple delay based on typical measurement time
  delay(50);

  float pressure = NAN;
  float temperature = NAN;

  // Read pressure and temperature
  if (!this->read_pressure_temperature_(&pressure, &temperature)) {
    ESP_LOGW(TAG, "Failed to read sensor data");
    this->status_set_warning();
    return;
  }

  ESP_LOGD(TAG, "Temperature: %.2f°C, Pressure: %.2f hPa", temperature, pressure);

  // Publish values
  if (this->pressure_sensor_ != nullptr) {
    this->pressure_sensor_->publish_state(pressure);
  }

  if (this->temperature_sensor_ != nullptr) {
    this->temperature_sensor_->publish_state(temperature);
  }

  this->status_clear_warning();
}

void XensivDPS3xxComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "DPS3xx:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with DPS3xx failed!");
  }
  ESP_LOGCONFIG(TAG, "  Product ID: 0x%02X", this->product_id_);
  ESP_LOGCONFIG(TAG, "  Revision ID: 0x%02X", this->revision_id_);
  ESP_LOGCONFIG(TAG, "  Temperature Sensor: %d", this->temp_sensor_);
  LOG_UPDATE_INTERVAL(this);

  if (this->interrupt_pin_ != nullptr) {
    LOG_PIN("  Interrupt Pin: ", this->interrupt_pin_);
  }
}

}  // namespace xensiv_dps3xx_base
}  // namespace esphome
}

void ESPHomeDps3xx::set_i2c_device(std::function<bool(uint8_t, uint8_t *, size_t)> read_fn,
                                   std::function<bool(uint8_t, const uint8_t *, size_t)> write_fn) {
  this->esphome_read_ = read_fn;
  this->esphome_write_ = write_fn;
}

int16_t ESPHomeDps3xx::readByte(uint8_t reg_address) {
  uint8_t data;
  if (this->esphome_read_(reg_address, &data, 1)) {
    return data;
  }
  return DPS__FAIL_UNKNOWN;
}

int16_t ESPHomeDps3xx::readBlock(RegBlock_t reg_block, uint8_t *buffer) {
  if (this->esphome_read_(reg_block.regAddress, buffer, reg_block.length)) {
    return reg_block.length;
  }
  return 0;
}

int16_t ESPHomeDps3xx::writeByte(uint8_t reg_address, uint8_t data, uint8_t check) {
  if (!this->esphome_write_(reg_address, &data, 1)) {
    return DPS__FAIL_UNKNOWN;
  }

  if (check) {
    uint8_t read_back;
    if (!this->esphome_read_(reg_address, &read_back, 1) || read_back != data) {
      return DPS__FAIL_UNKNOWN;
    }
  }

  return DPS__SUCCEEDED;
}

// XensivDPS3xx implementation
void XensivDPS3xx::setup() {
  ESP_LOGCONFIG(TAG, "Setting up XENSIV DPS3xx...");

  // Initialize the ESPHome-compatible DPS3xx sensor
  uint8_t address = this->get_i2c_address();
  this->dps_sensor_.init_esphome_i2c(address);

  // Set up I2C communication callbacks
  this->dps_sensor_.set_i2c_device(
      [this](uint8_t reg, uint8_t *data, size_t len) { return this->read_bytes(reg, data, len); },
      [this](uint8_t reg, const uint8_t *data, size_t len) { return this->write_bytes(reg, data, len); });

  // Initialize the sensor using the library's init method
  this->dps_sensor_.init_sensor();

  // Check product ID
  uint8_t prod_id = this->dps_sensor_.getProductId();
  uint8_t rev_id = this->dps_sensor_.getRevisionId();

  if (prod_id == 0xFF) {
    ESP_LOGE(TAG, "Failed to communicate with DPS3xx sensor");
    this->failure_reason_ += "Sensor not responding;";
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "DPS3xx found - Product ID: 0x%02X, Revision ID: 0x%02X", prod_id, rev_id);

  // Configure interrupts if pin is provided
  if (this->interrupt_pin_ != nullptr) {
    this->interrupt_pin_->setup();
    this->interrupt_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
    this->interrupt_pin_->attach_interrupt(XensivDPS3xx::gpio_intr, this, gpio::INTERRUPT_FALLING_EDGE);

    // Enable interrupts for both pressure and temperature measurements
    // Polarity: 0 = active low (for SDO high / I2C address 0x77)
    int16_t ret = this->dps_sensor_.setInterruptSources(DPS3xx_BOTH_INTR, 0);
    if (ret != DPS__SUCCEEDED) {
      ESP_LOGW(TAG, "Failed to configure interrupts: %d", ret);
    } else {
      ESP_LOGD(TAG, "Interrupts enabled: PRS | TMP (active LOW)");
    }
  }
}

void XensivDPS3xx::loop() {
  // Check if data is ready via interrupt
  if (this->data_ready_) {
    this->data_ready_ = false;  // Clear flag

    float pressure = NAN;
    float temperature = NAN;

    // Read temperature
    int16_t ret = this->dps_sensor_.getSingleResult(temperature);
    if (ret == DPS__SUCCEEDED) {
      if (this->temperature_sensor_ != nullptr) {
        this->temperature_sensor_->publish_state(temperature);
        ESP_LOGD(TAG, "Temperature: %.2f °C", temperature);
      }
    } else {
      ESP_LOGW(TAG, "Failed to read temperature: %d", ret);
    }

    // Read pressure
    ret = this->dps_sensor_.getSingleResult(pressure);
    if (ret == DPS__SUCCEEDED) {
      if (this->pressure_sensor_ != nullptr) {
        this->pressure_sensor_->publish_state(pressure / 100.0f);  // Convert Pa to hPa
        ESP_LOGD(TAG, "Pressure: %.2f hPa", pressure / 100.0f);
      }
    } else {
      ESP_LOGW(TAG, "Failed to read pressure: %d", ret);
    }
  }
}

void XensivDPS3xx::gpio_intr(XensivDPS3xx *arg) { arg->data_ready_ = true; }

bool XensivDPS3xx::measure_now() {
  ESP_LOGD(TAG, "Starting single-shot measurement");

  // Trigger single-shot pressure measurement using library method
  int16_t ret = this->dps_sensor_.startMeasurePressureOnce();

  if (ret == DPS__SUCCEEDED) {
    ESP_LOGD(TAG, "Single-shot measurement triggered successfully");
    return true;
  } else {
    ESP_LOGE(TAG, "Failed to start measurement: %d", ret);
    return false;
  }
}

void XensivDPS3xx::dump_config() {
  ESP_LOGCONFIG(TAG, "XENSIV DPS3xx Pressure Sensor:");

  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with DPS3xx failed!");
  }
  if (!this->failure_reason_.empty()) {
    ESP_LOGW(TAG, "Failure reason(s): %s", this->failure_reason_.c_str());
  }

  if (this->pressure_sensor_ != nullptr) {
    LOG_SENSOR("  ", "Pressure", this->pressure_sensor_);
  }

  if (this->temperature_sensor_ != nullptr) {
    LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  }

  if (this->interrupt_pin_ != nullptr) {
    LOG_PIN("  Interrupt Pin: ", this->interrupt_pin_);
  } else {
    ESP_LOGCONFIG(TAG, "  Interrupt Pin: Not configured");
  }
}

}  // namespace xensiv_dps3xx_base
}  // namespace esphome
