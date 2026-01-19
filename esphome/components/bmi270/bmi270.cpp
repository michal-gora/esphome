#include "bmi270.h"
#include "bmi270_config.h"
#include "esphome/core/log.h"

namespace esphome {
namespace bmi270 {

static const char *const TAG = "bmi270";

// BMI270 Register addresses
#define BMI2_CHIP_ID_ADDR 0x00
#define BMI2_PWR_CONF_ADDR 0x7C
#define BMI2_PWR_CTRL_ADDR 0x7D
#define BMI2_INIT_CTRL_ADDR 0x59
#define BMI2_INIT_ADDR_0 0x5B
#define BMI2_INIT_ADDR_1 0x5C
#define BMI2_INIT_DATA_ADDR 0x5E
#define BMI2_ACC_CONF_ADDR 0x40
#define BMI2_ACC_RANGE_ADDR 0x41
#define BMI2_GYR_CONF_ADDR 0x42
#define BMI2_GYR_RANGE_ADDR 0x43
#define BMI2_ACC_DATA_ADDR 0x0C
#define BMI2_GYR_DATA_ADDR 0x12
#define BMI2_INTERNAL_STATUS_ADDR 0x21
#define BMI2_STATUS_ADDR 0x03
#define BMI2_NV_CONF_ADDR 0x70
#define BMI2_GYR_OFF_COMP_3_ADDR 0x74
#define BMI2_GYR_OFF_COMP_6_ADDR 0x77
#define BMI2_OFFSET_ADDR 0x77

#define BMI2_CHIP_ID 0x24
#define BMI2_INIT_DATA_SIZE sizeof(bmi270_config_file)

// BMI270 API function implementations
int8_t bmi270_init(bmi2_dev *dev) {
  uint8_t chip_id;
  int8_t rslt = dev->read(BMI2_CHIP_ID_ADDR, &chip_id, 1, dev->intf_ptr);
  if (rslt != BMI2_OK)
    return rslt;

  if (chip_id != BMI2_CHIP_ID)
    return BMI2_E_COM_FAIL;

  dev->chip_id = chip_id;

  // Disable advanced power save mode (required for config upload)
  uint8_t pwr_conf = 0x00;
  rslt = dev->write(BMI2_PWR_CONF_ADDR, &pwr_conf, 1, dev->intf_ptr);
  if (rslt != BMI2_OK)
    return rslt;
  dev->delay_us(450, dev->intf_ptr);

  // Disable config loading (INIT_CTRL = 0)
  uint8_t init_ctrl = 0x00;
  rslt = dev->write(BMI2_INIT_CTRL_ADDR, &init_ctrl, 1, dev->intf_ptr);
  if (rslt != BMI2_OK)
    return rslt;

  // Upload config file in chunks with proper addressing
  // The BMI270 requires setting the word address before each chunk
  const uint16_t chunk_size = 32;  // Bytes per chunk (must be even)
  for (uint16_t index = 0; index < BMI2_INIT_DATA_SIZE; index += chunk_size) {
    // Calculate word address (index / 2)
    uint16_t word_addr = index / 2;
    uint8_t addr_array[2];
    addr_array[0] = (uint8_t) (word_addr & 0x0F);         // Lower 4 bits
    addr_array[1] = (uint8_t) ((word_addr >> 4) & 0xFF);  // Upper 8 bits

    // Write the address to INIT_ADDR registers
    rslt = dev->write(BMI2_INIT_ADDR_0, addr_array, 2, dev->intf_ptr);
    if (rslt != BMI2_OK)
      return rslt;

    // Calculate actual chunk length
    uint16_t len = (BMI2_INIT_DATA_SIZE - index) > chunk_size ? chunk_size : (BMI2_INIT_DATA_SIZE - index);

    // Write config data chunk
    rslt = dev->write(BMI2_INIT_DATA_ADDR, &bmi270_config_file[index], len, dev->intf_ptr);
    if (rslt != BMI2_OK)
      return rslt;
  }

  // Enable config loading (INIT_CTRL = 1)
  init_ctrl = 0x01;
  rslt = dev->write(BMI2_INIT_CTRL_ADDR, &init_ctrl, 1, dev->intf_ptr);
  if (rslt != BMI2_OK)
    return rslt;

  // Wait for initialization to complete (150ms as per datasheet)
  dev->delay_us(150000, dev->intf_ptr);

  // Check internal status to verify config load was successful
  uint8_t internal_status = 0;
  rslt = dev->read(BMI2_INTERNAL_STATUS_ADDR, &internal_status, 1, dev->intf_ptr);
  if (rslt != BMI2_OK)
    return rslt;

  // Bit 0 should be 1 (INIT_OK) for successful initialization
  if ((internal_status & 0x01) != 0x01) {
    return BMI2_E_CONFIG_LOAD;
  }

  return BMI2_OK;
}

int8_t bmi270_sensor_enable(const uint8_t *sens_list, uint8_t n_sens, bmi2_dev *dev) {
  // Enable accelerometer (0x04), gyroscope (0x02), and temperature (0x08)
  uint8_t pwr_ctrl = 0x0E;  // TEMP (0x08) + ACC (0x04) + GYR (0x02) = 0x0E
  return dev->write(BMI2_PWR_CTRL_ADDR, &pwr_ctrl, 1, dev->intf_ptr);
}

int8_t bmi270_set_sensor_config(bmi2_sens_config *sens_cfg, uint8_t n_sens, bmi2_dev *dev) {
  if (sens_cfg->type == BMI2_ACCEL) {
    uint8_t acc_conf = sens_cfg->cfg.acc.odr | (sens_cfg->cfg.acc.bw << 4);
    uint8_t acc_range = sens_cfg->cfg.acc.range;
    int8_t rslt = dev->write(BMI2_ACC_CONF_ADDR, &acc_conf, 1, dev->intf_ptr);
    if (rslt != BMI2_OK)
      return rslt;
    return dev->write(BMI2_ACC_RANGE_ADDR, &acc_range, 1, dev->intf_ptr);
  } else if (sens_cfg->type == BMI2_GYRO) {
    uint8_t gyr_conf = sens_cfg->cfg.gyr.odr | (sens_cfg->cfg.gyr.bw << 4);
    uint8_t gyr_range = sens_cfg->cfg.gyr.range;
    int8_t rslt = dev->write(BMI2_GYR_CONF_ADDR, &gyr_conf, 1, dev->intf_ptr);
    if (rslt != BMI2_OK)
      return rslt;
    return dev->write(BMI2_GYR_RANGE_ADDR, &gyr_range, 1, dev->intf_ptr);
  }
  return BMI2_OK;
}

int8_t bmi2_get_sensor_data(bmi2_sensor_data *sensor_data, uint8_t n_sens, bmi2_dev *dev) {
  uint8_t data[12];

  // Read accelerometer data
  int8_t rslt = dev->read(BMI2_ACC_DATA_ADDR, data, 6, dev->intf_ptr);
  if (rslt != BMI2_OK)
    return rslt;

  sensor_data[0].sens_data.acc.x = (int16_t) (data[0] | (data[1] << 8));
  sensor_data[0].sens_data.acc.y = (int16_t) (data[2] | (data[3] << 8));
  sensor_data[0].sens_data.acc.z = (int16_t) (data[4] | (data[5] << 8));

  // Read gyroscope data
  rslt = dev->read(BMI2_GYR_DATA_ADDR, data, 6, dev->intf_ptr);
  if (rslt != BMI2_OK)
    return rslt;

  sensor_data[1].sens_data.gyr.x = (int16_t) (data[0] | (data[1] << 8));
  sensor_data[1].sens_data.gyr.y = (int16_t) (data[2] | (data[3] << 8));
  sensor_data[1].sens_data.gyr.z = (int16_t) (data[4] | (data[5] << 8));

  return BMI2_OK;
}

void BMI270Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up BMI270...");

  this->sensor_.intf_ptr = this;
  this->sensor_.intf = BMI2_I2C_INTF;
  this->sensor_.read = read_bytes;
  this->sensor_.write = write_bytes;
  this->sensor_.delay_us = delay_usec;

  int8_t rslt;

  // Initialize the BMI270
  rslt = bmi270_init(&this->sensor_);
  if (rslt != BMI2_OK) {
    if (rslt == BMI2_E_CONFIG_LOAD) {
      ESP_LOGE(TAG, "BMI270 config load failed");
      uint8_t internal_status = 0;
      this->read_register(BMI2_INTERNAL_STATUS_ADDR, &internal_status, 1);
      char buf[64];
      snprintf(buf, sizeof(buf), "Config load failed, INTERNAL_STATUS=0x%02X; ", internal_status);
      this->failure_reason_ += buf;
    } else {
      ESP_LOGE(TAG, "BMI270 initialization failed: %d", rslt);
      this->failure_reason_ += "Initialization failed; ";
    }
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "BMI270 initialization succeeded");

  // Enable accelerometer and gyroscope
  uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};
  rslt = bmi270_sensor_enable(sens_list, 2, &this->sensor_);
  if (rslt != BMI2_OK) {
    ESP_LOGE(TAG, "Failed to enable accelerometer and gyroscope: %d", rslt);
    this->failure_reason_ += "Sensor enable failed; ";
    this->mark_failed();
    return;
  }

  // Verify PWR_CTRL was written correctly
  uint8_t pwr_ctrl_readback = 0;
  uint8_t pwr_conf_readback = 0;
  this->read_register(BMI2_PWR_CTRL_ADDR, &pwr_ctrl_readback, 1);
  this->read_register(BMI2_PWR_CONF_ADDR, &pwr_conf_readback, 1);

  char buf[64];
  snprintf(buf, sizeof(buf), "PWR_CTRL=0x%02X PWR_CONF=0x%02X; ", pwr_ctrl_readback, pwr_conf_readback);
  this->failure_reason_ += buf;

  // Configure accelerometer
  this->accel_cfg_.type = BMI2_ACCEL;
  this->accel_cfg_.cfg.acc.odr = BMI2_ACC_ODR_100HZ;
  this->accel_cfg_.cfg.acc.range = BMI2_ACC_RANGE_2G;
  this->accel_cfg_.cfg.acc.bw = BMI2_ACC_NORMAL_AVG4;
  this->accel_cfg_.cfg.acc.perf_mode = BMI2_PERF_OPT_MODE;
  rslt = bmi270_set_sensor_config(&this->accel_cfg_, 1, &this->sensor_);
  if (rslt != BMI2_OK) {
    ESP_LOGE(TAG, "Failed to configure accelerometer: %d", rslt);
    this->failure_reason_ += "Accelerometer config failed; ";
    this->mark_failed();
    return;
  }

  // Read back accelerometer config
  uint8_t acc_conf_readback = 0;
  uint8_t acc_range_readback = 0;
  this->read_register(BMI2_ACC_CONF_ADDR, &acc_conf_readback, 1);
  this->read_register(BMI2_ACC_RANGE_ADDR, &acc_range_readback, 1);

  snprintf(buf, sizeof(buf), "ACC_CONF=0x%02X ACC_RANGE=0x%02X; ", acc_conf_readback, acc_range_readback);
  this->failure_reason_ += buf;

  // Configure gyroscope
  this->gyro_cfg_.type = BMI2_GYRO;
  this->gyro_cfg_.cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
  this->gyro_cfg_.cfg.gyr.range = BMI2_GYR_RANGE_2000;
  this->gyro_cfg_.cfg.gyr.bw = BMI2_GYR_NORMAL_MODE;
  this->gyro_cfg_.cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
  this->gyro_cfg_.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
  rslt = bmi270_set_sensor_config(&this->gyro_cfg_, 1, &this->sensor_);
  if (rslt != BMI2_OK) {
    ESP_LOGE(TAG, "Failed to configure gyroscope: %d", rslt);
    this->failure_reason_ += "Gyroscope config failed; ";
    this->mark_failed();
    return;
  }

  // Read back gyroscope config
  uint8_t gyr_conf_readback = 0;
  uint8_t gyr_range_readback = 0;
  this->read_register(BMI2_GYR_CONF_ADDR, &gyr_conf_readback, 1);
  this->read_register(BMI2_GYR_RANGE_ADDR, &gyr_range_readback, 1);

  snprintf(buf, sizeof(buf), "GYR_CONF=0x%02X GYR_RANGE=0x%02X; ", gyr_conf_readback, gyr_range_readback);
  this->failure_reason_ += buf;

  // Enable gyroscope offset compensation
  // NV_CONF register (0x70): bit 6 = gyr_off_en
  uint8_t nv_conf = 0;
  this->read_register(BMI2_NV_CONF_ADDR, &nv_conf, 1);
  nv_conf |= 0x40;  // Set bit 6 to enable gyro offset compensation
  this->write_register(BMI2_NV_CONF_ADDR, &nv_conf, 1);
  ESP_LOGI(TAG, "Enabled gyroscope offset compensation (NV_CONF=0x%02X)", nv_conf);

  // Perform software gyro bias calibration
  // Wait for sensor to stabilize, then take average readings
  ESP_LOGI(TAG, "Calibrating gyroscope bias (keep device still)...");
  delay(10);  // Wait for sensor to stabilize

  int32_t gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;
  const int num_samples = 32;

  for (int i = 0; i < num_samples; i++) {
    uint8_t gyro_data[6];
    this->read_register(BMI2_GYR_DATA_ADDR, gyro_data, 6);
    int16_t gx = (int16_t) ((gyro_data[1] << 8) | gyro_data[0]);
    int16_t gy = (int16_t) ((gyro_data[3] << 8) | gyro_data[2]);
    int16_t gz = (int16_t) ((gyro_data[5] << 8) | gyro_data[4]);
    gyro_bias_x += gx;
    gyro_bias_y += gy;
    gyro_bias_z += gz;
    delay(10);  // 100Hz sample rate
  }

  this->gyro_bias_x_ = gyro_bias_x / num_samples;
  this->gyro_bias_y_ = gyro_bias_y / num_samples;
  this->gyro_bias_z_ = gyro_bias_z / num_samples;

  ESP_LOGI(TAG, "Gyro bias calibration complete: X=%d Y=%d Z=%d LSB", this->gyro_bias_x_, this->gyro_bias_y_,
           this->gyro_bias_z_);

  this->is_initialized_ = true;

  // Verify sensors are actually active by checking status register
  uint8_t status = 0;
  i2c::ErrorCode err = this->read_register(0x03, &status, 1);  // Read STATUS_ADDR (0x03)

  snprintf(buf, sizeof(buf), "STATUS=0x%02X (i2c_err=%d) ACC_DRDY=%d GYR_DRDY=%d", status, err, (status >> 7) & 1,
           (status >> 6) & 1);
  this->failure_reason_ += buf;

  if (err == i2c::ERROR_OK && (status & 0xC0)) {
    this->sensors_active_ = true;
  }
}

void BMI270Component::update() {
  if (!this->is_initialized_)
    return;

  // Check status register to see if data is ready
  uint8_t status = 0;
  i2c::ErrorCode err = this->read_register(0x03, &status, 1);

  struct bmi2_sensor_data sensor_data[2] = {};
  sensor_data[0].type = BMI2_ACCEL;
  sensor_data[1].type = BMI2_GYRO;

  int8_t rslt = bmi2_get_sensor_data(sensor_data, 2, &this->sensor_);
  if (rslt != BMI2_OK) {
    ESP_LOGW(TAG, "Failed to read sensor data: %d", rslt);
    return;
  }

  // Log raw sensor values and status for debugging
  ESP_LOGD(TAG, "Status=0x%02X (i2c_err=%d) | Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d", status, err,
           sensor_data[0].sens_data.acc.x, sensor_data[0].sens_data.acc.y, sensor_data[0].sens_data.acc.z,
           sensor_data[1].sens_data.gyr.x, sensor_data[1].sens_data.gyr.y, sensor_data[1].sens_data.gyr.z);

  // Accelerometer: At ±2g range, sensitivity is 16384 LSB/g
  // Convert to SI units: m/s² (multiply g by 9.80665)
  constexpr float ACCEL_SCALE = 9.80665f / 16384.0f;  // LSB to m/s²
  if (this->accel_x_sensor_ != nullptr)
    this->accel_x_sensor_->publish_state(sensor_data[0].sens_data.acc.x * ACCEL_SCALE);
  if (this->accel_y_sensor_ != nullptr)
    this->accel_y_sensor_->publish_state(sensor_data[0].sens_data.acc.y * ACCEL_SCALE);
  if (this->accel_z_sensor_ != nullptr)
    this->accel_z_sensor_->publish_state(sensor_data[0].sens_data.acc.z * ACCEL_SCALE);

  // Gyroscope: At ±2000°/s range, sensitivity is 16.4 LSB/°/s
  // Output in °/s (degrees per second), with bias correction
  constexpr float GYRO_SCALE = 1.0f / 16.4f;  // LSB to °/s
  if (this->gyro_x_sensor_ != nullptr)
    this->gyro_x_sensor_->publish_state((sensor_data[1].sens_data.gyr.x - this->gyro_bias_x_) * GYRO_SCALE);
  if (this->gyro_y_sensor_ != nullptr)
    this->gyro_y_sensor_->publish_state((sensor_data[1].sens_data.gyr.y - this->gyro_bias_y_) * GYRO_SCALE);
  if (this->gyro_z_sensor_ != nullptr)
    this->gyro_z_sensor_->publish_state((sensor_data[1].sens_data.gyr.z - this->gyro_bias_z_) * GYRO_SCALE);

  // Temperature: Registers 0x22 (LSB) and 0x23 (MSB)
  // Resolution: 1/512 °C/LSB, with 0x0000 = 23°C
  if (this->temperature_sensor_ != nullptr) {
    uint8_t temp_data[2];
    if (this->read_register(0x22, temp_data, 2) == i2c::ERROR_OK) {
      int16_t temp_raw = (int16_t) ((temp_data[1] << 8) | temp_data[0]);
      float temperature = (temp_raw / 512.0f) + 23.0f;
      this->temperature_sensor_->publish_state(temperature);
    }
  }
}

void BMI270Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BMI270:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Initialization failed: %s", this->failure_reason_.c_str());
  }
  ESP_LOGCONFIG(TAG, "  Sensors active: %s", this->sensors_active_ ? "Yes" : "No");
  ESP_LOGCONFIG(TAG, "  Initialized: %s", this->is_initialized_ ? "Yes" : "No");
}

float BMI270Component::get_setup_priority() const { return setup_priority::DATA; }

int8_t BMI270Component::read_bytes(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
  auto *component = reinterpret_cast<BMI270Component *>(intf_ptr);
  if (component->read_register(reg_addr, data, len) != i2c::ERROR_OK) {
    return BMI2_E_COM_FAIL;
  }
  return BMI2_OK;
}

int8_t BMI270Component::write_bytes(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
  auto *component = reinterpret_cast<BMI270Component *>(intf_ptr);
  if (component->write_register(reg_addr, data, len) != i2c::ERROR_OK) {
    return BMI2_E_COM_FAIL;
  }
  return BMI2_OK;
}

void BMI270Component::delay_usec(uint32_t period, void *) { delay_microseconds_safe(period); }

}  // namespace bmi270
}  // namespace esphome
