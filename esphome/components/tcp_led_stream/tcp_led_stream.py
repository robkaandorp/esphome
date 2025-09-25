import esphome.codegen as cg
from esphome.components import light, sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_LIGHT_ID, CONF_PORT

# Conditionally import binary_sensor to avoid forcing dependency
try:
    from esphome.components import binary_sensor

    HAS_BINARY_SENSOR = True
except ImportError:
    HAS_BINARY_SENSOR = False

DEPENDENCIES = ["network"]
# Do not auto-load sensor/binary_sensor per ESPHome contribution guidelines; they are optional.
AUTO_LOAD = ["light"]

CONF_PIXEL_FORMAT = "pixel_format"
CONF_TIMEOUT = "timeout"  # ms inactivity before connection dropped

PIXEL_FORMATS = {
    "RGB": 3,
    "RGBW": 4,
    "GRB": 3,  # alternative ordering
    "GRBW": 4,
    "BGR": 3,
}

tcp_led_stream_ns = cg.esphome_ns.namespace("tcp_led_stream")
TCPLedStreamComponent = tcp_led_stream_ns.class_("TCPLedStreamComponent", cg.Component)

PixelFormat = tcp_led_stream_ns.enum("PixelFormat")

PIXEL_FORMAT_ENUM = {
    "RGB": PixelFormat.RGB,
    "RGBW": PixelFormat.RGBW,
    "GRB": PixelFormat.GRB,
    "GRBW": PixelFormat.GRBW,
    "BGR": PixelFormat.BGR,
}

CONF_FRAME_RATE = "frame_rate"
CONF_BYTES_RECEIVED = "bytes_received"
CONF_CONNECTS = "connects"
CONF_DISCONNECTS = "disconnects"
CONF_OVERLAPS = "overlaps"
CONF_FRAME_COMPLETION_INTERVAL = "frame_completion_interval"
CONF_CLIENT_CONNECTED = "client_connected"
CONF_COMPLETION_MODE = "completion_mode"
CONF_SHOW_TIME_PER_LED_US = "show_time_per_led_us"

COMPLETION_MODES = ["heuristic", "estimate"]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(TCPLedStreamComponent),
        cv.Required(CONF_LIGHT_ID): cv.use_id(light.AddressableLightState),
        cv.Optional(CONF_PORT, default=7777): cv.port,
        cv.Optional(CONF_PIXEL_FORMAT, default="RGB"): cv.one_of(
            *PIXEL_FORMAT_ENUM.keys(), upper=True
        ),
        cv.Optional(CONF_TIMEOUT, default=5000): cv.int_range(min=0, max=60000),
        cv.Optional(CONF_FRAME_COMPLETION_INTERVAL, default=15): cv.int_range(
            min=1, max=100
        ),
        cv.Optional(CONF_FRAME_RATE): sensor.sensor_schema(
            unit_of_measurement="fps", accuracy_decimals=2
        ),
        cv.Optional(CONF_BYTES_RECEIVED): sensor.sensor_schema(
            unit_of_measurement="B", accuracy_decimals=0
        ),
        cv.Optional(CONF_CONNECTS): sensor.sensor_schema(
            unit_of_measurement="", accuracy_decimals=0
        ),
        cv.Optional(CONF_DISCONNECTS): sensor.sensor_schema(
            unit_of_measurement="", accuracy_decimals=0
        ),
        cv.Optional(CONF_OVERLAPS): sensor.sensor_schema(
            unit_of_measurement="", accuracy_decimals=0
        ),
        cv.Optional(CONF_COMPLETION_MODE, default="heuristic"): cv.one_of(
            *COMPLETION_MODES, lower=True
        ),
        cv.Optional(CONF_SHOW_TIME_PER_LED_US, default=30): cv.int_range(
            min=1, max=200
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

# Add binary_sensor schema only if available
if HAS_BINARY_SENSOR:
    CONFIG_SCHEMA = CONFIG_SCHEMA.extend(
        {cv.Optional(CONF_CLIENT_CONNECTED): binary_sensor.binary_sensor_schema()}
    )


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    light_state = await cg.get_variable(config[CONF_LIGHT_ID])
    cg.add(var.set_light(light_state))
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_pixel_format(PIXEL_FORMAT_ENUM[config[CONF_PIXEL_FORMAT]]))
    cg.add(var.set_timeout(config[CONF_TIMEOUT]))
    cg.add(var.set_frame_completion_interval(config[CONF_FRAME_COMPLETION_INTERVAL]))
    cg.add(var.set_completion_mode(config[CONF_COMPLETION_MODE]))
    cg.add(var.set_show_time_per_led_us(config[CONF_SHOW_TIME_PER_LED_US]))

    if CONF_FRAME_RATE in config:
        sens = await sensor.new_sensor(config[CONF_FRAME_RATE])
        cg.add(var.set_frame_rate_sensor(sens))
    if CONF_BYTES_RECEIVED in config:
        sens = await sensor.new_sensor(config[CONF_BYTES_RECEIVED])
        cg.add(var.set_bytes_received_sensor(sens))
    if CONF_CONNECTS in config:
        sens = await sensor.new_sensor(config[CONF_CONNECTS])
        cg.add(var.set_connects_sensor(sens))
    if CONF_DISCONNECTS in config:
        sens = await sensor.new_sensor(config[CONF_DISCONNECTS])
        cg.add(var.set_disconnects_sensor(sens))
    if CONF_OVERLAPS in config:
        sens = await sensor.new_sensor(config[CONF_OVERLAPS])
        cg.add(var.set_overlaps_sensor(sens))
    if CONF_CLIENT_CONNECTED in config and HAS_BINARY_SENSOR:
        bs = await binary_sensor.new_binary_sensor(config[CONF_CLIENT_CONNECTED])
        cg.add(var.set_client_connected_binary_sensor(bs))
