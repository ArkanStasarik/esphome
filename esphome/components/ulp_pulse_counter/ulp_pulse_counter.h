#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"

namespace esphome {
namespace ulp_pulse_counter {

class ULPPulseCounterSensor : public sensor::Sensor, public PollingComponent {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

  void set_total_sensor(sensor::Sensor *total_sensor);
  void update_pulse_counter();
  uint32_t get_edges(void);
  double get_duration_us();
  uint32_t add_to_total_edges(uint32_t increment);

  sensor::Sensor *total_sensor_{nullptr};
};

}  // namespace ulp_pulse_counter
}  // namespace esphome
