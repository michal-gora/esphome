#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/gpio.h"
#include "dps_config.h"
#include "dps3xx_config.h"

namespace esphome {
namespace xensiv_dps3xx_base {

class XensivDPS3xx : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_pressure_sensor(sensor::Sensor *sensor) { pressure_sensor_ = sensor; }
  void set_temperature_sensor(sensor::Sensor *sensor) { temperature_sensor_ = sensor; }
  void set_interrupt_pin(InternalGPIOPin *pin) { interrupt_pin_ = pin; }

  bool measure_now();

 protected:
  sensor::Sensor *pressure_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};

  // Sensor state
  uint8_t product_id_{0};
  uint8_t revision_id_{0};
  uint8_t temp_sensor_{0};

  // Calibration coefficients
  int32_t c00_{0};
  int32_t c10_{0};
  int32_t c01_{0};
  int32_t c11_{0};
  int32_t c20_{0};
  int32_t c21_{0};
  int32_t c30_{0};
  int32_t c0_half_{0};
  int32_t c1_{0};

  // Measurement configuration
  uint8_t temp_mr_{DPS__MEASUREMENT_RATE_4};
  uint8_t temp_osr_{DPS__OVERSAMPLING_RATE_8};
  uint8_t prs_mr_{DPS__MEASUREMENT_RATE_4};
  uint8_t prs_osr_{DPS__OVERSAMPLING_RATE_8};

  static void gpio_intr(XensivDPS3xx *arg);

  // Sensor initialization and configuration
  bool init_sensor_();
  bool read_calibration_coefficients_();
  bool configure_sensor_();

  // Measurement functions
  bool read_pressure_temperature_(float &pressure, float &temperature);
  float calculate_temperature_(int32_t raw);
  float calculate_pressure_(int32_t raw_prs, int32_t raw_temp);

  // Register access helpers
  int16_t read_byte_bitfield_(RegMask_t reg_mask);
  int16_t write_byte_bitfield_(uint8_t data, RegMask_t reg_mask);
  void get_twos_complement_(int32_t *raw, uint8_t length);

  // Pure virtual I2C methods - implemented by I2C subclass
  virtual bool read_byte(uint8_t reg, uint8_t *data) = 0;
  virtual bool read_bytes(uint8_t reg, uint8_t *data, size_t len) = 0;
  virtual bool write_byte(uint8_t reg, uint8_t value) = 0;

  InternalGPIOPin *interrupt_pin_{nullptr};
  volatile bool data_ready_{false};

  std::string failure_reason_;
};

}  // namespace xensiv_dps3xx_base
}  // namespace esphome
