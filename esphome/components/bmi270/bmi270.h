#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/hal.h"

// BMI270 API type definitions and constants
#define BMI2_OK 0
#define BMI2_E_COM_FAIL 1
#define BMI2_E_CONFIG_LOAD 2

#define BMI2_I2C_INTF 0
#define BMI2_ACCEL 0
#define BMI2_GYRO 1

#define BMI2_ACC_ODR_100HZ 0x08
#define BMI2_ACC_RANGE_2G 0x00
#define BMI2_ACC_NORMAL_AVG4 0x01
#define BMI2_PERF_OPT_MODE 0x00

#define BMI2_GYR_ODR_100HZ 0x08
#define BMI2_GYR_RANGE_2000 0x04
#define BMI2_GYR_NORMAL_MODE 0x00
#define BMI2_POWER_OPT_MODE 0x00

namespace esphome {
namespace bmi270 {

// Forward declarations for BMI270 API structures
struct bmi2_dev {
  uint8_t chip_id;
  uint8_t intf;
  void *intf_ptr;
  int8_t (*read)(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
  int8_t (*write)(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
  void (*delay_us)(uint32_t period, void *intf_ptr);
};

struct bmi2_accel_config {
  uint8_t odr;
  uint8_t range;
  uint8_t bw;
  uint8_t perf_mode;
};

struct bmi2_gyro_config {
  uint8_t odr;
  uint8_t range;
  uint8_t bw;
  uint8_t noise_perf;
  uint8_t filter_perf;
};

struct bmi2_sens_config {
  uint8_t type;
  union {
    bmi2_accel_config acc;
    bmi2_gyro_config gyr;
  } cfg;
};

struct bmi2_sens_axes_data {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct bmi2_sensor_data {
  uint8_t type;
  union {
    bmi2_sens_axes_data acc;
    bmi2_sens_axes_data gyr;
  } sens_data;
};

// BMI270 API function declarations
int8_t bmi270_init(bmi2_dev *dev);
int8_t bmi270_sensor_enable(const uint8_t *sens_list, uint8_t n_sens, bmi2_dev *dev);
int8_t bmi270_set_sensor_config(bmi2_sens_config *sens_cfg, uint8_t n_sens, bmi2_dev *dev);
int8_t bmi2_get_sensor_data(bmi2_sensor_data *sensor_data, uint8_t n_sens, bmi2_dev *dev);

// Power save mode enumeration
enum PowerSaveMode {
  POWER_SAVE_MODE_NORMAL = 0,
  POWER_SAVE_MODE_LOW_POWER = 1,
  // POWER_SAVE_MODE_PERFORMANCE = 2, // Optional
};

class BMI270Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override;

  void set_accel_x_sensor(sensor::Sensor *accel_x_sensor) { accel_x_sensor_ = accel_x_sensor; }
  void set_accel_y_sensor(sensor::Sensor *accel_y_sensor) { accel_y_sensor_ = accel_y_sensor; }
  void set_accel_z_sensor(sensor::Sensor *accel_z_sensor) { accel_z_sensor_ = accel_z_sensor; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_gyro_x_sensor(sensor::Sensor *gyro_x_sensor) { gyro_x_sensor_ = gyro_x_sensor; }
  void set_gyro_y_sensor(sensor::Sensor *gyro_y_sensor) { gyro_y_sensor_ = gyro_y_sensor; }
  void set_gyro_z_sensor(sensor::Sensor *gyro_z_sensor) { gyro_z_sensor_ = gyro_z_sensor; }

  void set_power_save_mode(PowerSaveMode mode) { power_save_mode_ = mode; }

 protected:
  bool bmi270_init_config_file();
  void apply_power_save_mode();

  // Static callback functions for BMI270 API
  static int8_t read_bytes(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
  static int8_t write_bytes(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
  static void delay_usec(uint32_t period, void *intf_ptr);

  sensor::Sensor *accel_x_sensor_{nullptr};
  sensor::Sensor *accel_y_sensor_{nullptr};
  sensor::Sensor *accel_z_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *gyro_x_sensor_{nullptr};
  sensor::Sensor *gyro_y_sensor_{nullptr};
  sensor::Sensor *gyro_z_sensor_{nullptr};

  float accel_sensitivity_{0.0f};
  float gyro_sensitivity_{0.0f};
  PowerSaveMode power_save_mode_{POWER_SAVE_MODE_NORMAL};
  bool sensors_active_{false};

  // Gyroscope bias calibration values (in LSB)
  int16_t gyro_bias_x_{0};
  int16_t gyro_bias_y_{0};
  int16_t gyro_bias_z_{0};

  // BMI270 device structure and configuration
  bmi2_dev sensor_{};
  bmi2_sens_config accel_cfg_{};
  bmi2_sens_config gyro_cfg_{};
  bool is_initialized_{false};

  std::string failure_reason_{""};
};

}  // namespace bmi270
}  // namespace esphome
