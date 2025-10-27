import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_TOTAL,
    ICON_PULSE,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_PULSES,
    UNIT_PULSES_PER_MINUTE,
)

ulp_pulse_counter_ns = cg.esphome_ns.namespace("ulp_pulse_counter")
ULPPulseCounterSensor = ulp_pulse_counter_ns.class_(
    "ULPPulseCounterSensor", sensor.Sensor, cg.PollingComponent
)

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        ULPPulseCounterSensor,
        unit_of_measurement=UNIT_PULSES_PER_MINUTE,
        icon=ICON_PULSE,
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            cv.Optional(CONF_TOTAL): sensor.sensor_schema(
                unit_of_measurement=UNIT_PULSES,
                icon=ICON_PULSE,
                accuracy_decimals=0,
                state_class=STATE_CLASS_TOTAL_INCREASING,
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
)


async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)
    if CONF_TOTAL in config:
        sens = await sensor.new_sensor(config[CONF_TOTAL])
        cg.add(var.set_total_sensor(sens))
