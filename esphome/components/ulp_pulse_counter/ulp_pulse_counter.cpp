#include "ulp_pulse_counter.h"

#include "esphome/core/log.h"

#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "esp32/ulp.h"
#include "soc/soc_ulp.h"
#include "soc/rtc.h"
#include "esp_sleep.h"

namespace esphome {
namespace ulp_pulse_counter {

static const char *const TAG = "ulp_pulse_counter";

sensor::Sensor *total_sensor_{nullptr};

const size_t rtc_slow_mem_total_size = 2048;  // 8kB = 4*2048
const size_t rtc_slow_mem_vars_space = 256;   // 1kB = 4*256
const size_t rtc_slow_mem_vars_start = rtc_slow_mem_total_size - rtc_slow_mem_vars_space;
const size_t rtc_slow_mem_prog_start = 0;
const size_t rtc_slow_mem_prog_space = rtc_slow_mem_total_size - rtc_slow_mem_vars_space;  // 7kB

const size_t ulp_io_number_var_offset = 0;
const size_t ulp_next_edge_var_offset = 1;
const size_t ulp_debounce_max_count_var_offset = 2;
const size_t ulp_debounce_counter_var_offset = 3;
const size_t ulp_edge_count_var_offset = 4;

const size_t edge_count_var_offset = 5;
const size_t duration_var_offset = 6;
const size_t last_time_var_offset = 7;

const size_t edges_total_var_offset = 8;

const size_t ulp_io_number_var_address = ulp_io_number_var_offset + rtc_slow_mem_vars_start;
const size_t ulp_next_edge_var_address = ulp_next_edge_var_offset + rtc_slow_mem_vars_start;
const size_t ulp_debounce_max_count_var_address = ulp_debounce_max_count_var_offset + rtc_slow_mem_vars_start;
const size_t ulp_debounce_counter_var_address = ulp_debounce_counter_var_offset + rtc_slow_mem_vars_start;
const size_t ulp_edge_count_var_address = ulp_edge_count_var_offset + rtc_slow_mem_vars_start;

const size_t edge_count_var_address = edge_count_var_offset + rtc_slow_mem_vars_start;
const size_t duration_var_address = duration_var_offset + rtc_slow_mem_vars_start;
const size_t last_time_var_address = last_time_var_offset + rtc_slow_mem_vars_start;

const size_t edges_total_var_address = edges_total_var_offset + rtc_slow_mem_vars_start;

void ULPPulseCounterSensor::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ULPPulseCounter '%s'...", this->name_.c_str());
  if (esp_sleep_get_wakeup_cause() != 0) {
    return;
  }
  // clear program space
  memset(RTC_SLOW_MEM, 0, rtc_slow_mem_prog_space * sizeof(uint32_t));

  // Define ULP program
  const ulp_insn_t ulp_prog[] = {
      M_LABEL(1),  // entry
      /* Load io_number */
      I_MOVI(R3, ulp_io_number_var_address), I_LD(R3, R3, 0),
      /* Lower 16 IOs and higher need to be handled separately,
       * because r0-r3 registers are 16 bit wide.
       * Check which IO this is.
       */
      I_MOVR(R0, R3), M_BGE(2, 16),  // jumpr read_io_high, 16, ge - Jump to read_io_high if R0 >= 16
      /* Read the value of lower 16 RTC IOs into R0 */
      I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S,
               RTC_GPIO_IN_NEXT_S + 16 -
                   1),  // READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16) -- high bit is low_bit + bit_width - 1
      I_RSHR(R0, R0, R3),
      M_BX(3),  // jump read_done

      M_LABEL(2),  // read_io_high
      /* Read the value of RTC IOs 16-17, into R0 */
      I_RD_REG(
          RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + 16,
          (RTC_GPIO_IN_NEXT_S + 16) + 2 -
              1),  // READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S + 16, 2) -- high bit is low_bit + bit_width - 1
      I_SUBI(R3, R3, 16), I_RSHR(R0, R0, R3),

      M_LABEL(3),         // read_done
      I_ANDI(R0, R0, 1),  // isolate single bit
      /* State of input changed? */
      I_MOVI(R3, ulp_next_edge_var_address), I_LD(R3, R3, 0),
      I_ADDR(R3, R0, R3),  // If io + next_edge equal each other result is 10b or 0b in which case the next and will be
                           // 0 and jump condition (last ALU op is zero) will be true
      I_ANDI(R3, R3, 1),
      M_BXZ(4),  // jump changed, eq

      /* Not changed */
      /* Reset debounce_counter to debounce_max_count */
      I_MOVI(R3, ulp_debounce_max_count_var_address), I_MOVI(R2, ulp_debounce_counter_var_address), I_LD(R3, R3, 0),
      I_ST(R3, R2, 0),
      /* End program */
      I_HALT(),

      M_LABEL(4),  // changed
      /* Input state changed */
      /* Has debounce_counter reached zero? */
      I_MOVI(R3, ulp_debounce_counter_var_address), I_LD(R2, R3, 0),
      I_ADDI(R2, R2, 0), /* dummy ADD to use "jump if ALU result is zero" */
      M_BXZ(5),          // jump edge_detected, eq
      /* Not yet. Decrement debounce_counter */
      I_SUBI(R2, R2, 1), I_ST(R2, R3, 0),
      /* End program */
      I_HALT(),

      M_LABEL(5),  // edge_detected
      /* Reset debounce_counter to debounce_max_count */
      I_MOVI(R3, ulp_debounce_max_count_var_address), I_MOVI(R2, ulp_debounce_counter_var_address), I_LD(R3, R3, 0),
      I_ST(R3, R2, 0),
      /* Flip next_edge */
      I_MOVI(R3, ulp_next_edge_var_address), I_LD(R2, R3, 0), I_ADDI(R2, R2, 1), I_ANDI(R2, R2, 1), I_ST(R2, R3, 0),
      /* Increment edge_count */
      I_MOVI(R3, ulp_edge_count_var_address), I_LD(R2, R3, 0), I_ADDI(R2, R2, 1), I_ST(R2, R3, 0),
      /* End program */
      I_HALT()};

  /* GPIO used for pulse counting. */
  gpio_num_t gpio_num = GPIO_NUM_27;
  int rtcio_num = rtc_io_number_get(gpio_num);
  assert(rtc_gpio_is_valid_gpio(gpio_num) && "GPIO used for pulse counting must be an RTC IO");

  // set variables
  RTC_SLOW_MEM[ulp_io_number_var_address] = rtcio_num;
  RTC_SLOW_MEM[ulp_next_edge_var_offset] = 0;
  RTC_SLOW_MEM[ulp_debounce_max_count_var_address] = 3;
  RTC_SLOW_MEM[ulp_debounce_counter_var_address] = RTC_SLOW_MEM[ulp_debounce_max_count_var_address];
  RTC_SLOW_MEM[ulp_edge_count_var_address] = 0;

  /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
  rtc_gpio_init(gpio_num);
  rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pulldown_dis(gpio_num);
  rtc_gpio_pullup_dis(gpio_num);
  rtc_gpio_hold_en(gpio_num);

  /* Set ULP wake up period to T = 20ms.
   * Minimum pulse width has to be T * (ulp_debounce_counter + 1) = 80ms.
   */
  ulp_set_wakeup_period(0, 20000);

  // set some more
  RTC_SLOW_MEM[last_time_var_address] = rtc_time_get();
  RTC_SLOW_MEM[edges_total_var_address] = 0;

  // run ulp program
  size_t size = sizeof(ulp_prog) / sizeof(ulp_insn_t);

  ulp_process_macros_and_load(rtc_slow_mem_prog_start, ulp_prog, &size);
  ulp_run(rtc_slow_mem_prog_start);

  ESP_LOGCONFIG(TAG, "Completed setting ULPPulseCounter '%s'...", this->name_.c_str());
}

void ULPPulseCounterSensor::dump_config() {
  LOG_SENSOR("", "ULPPulseCounter", this);
  LOG_UPDATE_INTERVAL(this);
}

void ULPPulseCounterSensor::update() {
  ESP_LOGVV(TAG, "update");
  update_pulse_counter();
  uint32_t edges = get_edges();
  double duration = get_duration_us() / (1000 * 1000 * 60);  // min
  double edges_per_minute = edges / duration;

  this->publish_state(edges_per_minute);
  if (this->total_sensor_ != nullptr) {
    total_sensor_->publish_state(add_to_total_edges(edges));
  }
}

void ULPPulseCounterSensor::set_total_sensor(sensor::Sensor *total_sensor) { total_sensor_ = total_sensor; }

float ULPPulseCounterSensor::get_setup_priority() const { return setup_priority::DATA; }

void ULPPulseCounterSensor::update_pulse_counter(void) {
  uint32_t last_time = RTC_SLOW_MEM[last_time_var_address];
  RTC_SLOW_MEM[last_time_var_address] = rtc_time_get();

  RTC_SLOW_MEM[edge_count_var_address] = (RTC_SLOW_MEM[ulp_edge_count_var_address] & UINT16_MAX);
  RTC_SLOW_MEM[ulp_edge_count_var_address] = 0;

  RTC_SLOW_MEM[duration_var_address] = RTC_SLOW_MEM[last_time_var_address] - last_time;
}

uint32_t ULPPulseCounterSensor::get_edges(void) { return RTC_SLOW_MEM[edge_count_var_address]; }

double ULPPulseCounterSensor::get_duration_us(void) { return ((double) RTC_SLOW_MEM[duration_var_address]) * 6.666666; }

uint32_t ULPPulseCounterSensor::add_to_total_edges(uint32_t increment) {
  RTC_SLOW_MEM[edges_total_var_address] = RTC_SLOW_MEM[edges_total_var_address] + increment;
  return RTC_SLOW_MEM[edges_total_var_address];
}

}  // namespace ulp_pulse_counter
}  // namespace esphome
