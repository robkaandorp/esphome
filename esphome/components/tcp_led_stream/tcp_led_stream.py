import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PORT, CONF_LIGHT_ID
from esphome.components import light

DEPENDENCIES = ["network"]
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
TCPLedStreamComponent = tcp_led_stream_ns.class_(
    "TCPLedStreamComponent", cg.Component
)

PixelFormat = tcp_led_stream_ns.enum("PixelFormat")

PIXEL_FORMAT_ENUM = {
    "RGB": PixelFormat.RGB,
    "RGBW": PixelFormat.RGBW,
    "GRB": PixelFormat.GRB,
    "GRBW": PixelFormat.GRBW,
    "BGR": PixelFormat.BGR,
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(TCPLedStreamComponent),
        cv.Required(CONF_LIGHT_ID): cv.use_id(light.AddressableLightState),
        cv.Optional(CONF_PORT, default=7777): cv.port,
        cv.Optional(CONF_PIXEL_FORMAT, default="RGB"): cv.one_of(*PIXEL_FORMAT_ENUM.keys(), upper=True),
        cv.Optional(CONF_TIMEOUT, default=5000): cv.int_range(min=0, max=60000),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    l = await cg.get_variable(config[CONF_LIGHT_ID])
    cg.add(var.set_light(l))
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_pixel_format(PIXEL_FORMAT_ENUM[config[CONF_PIXEL_FORMAT]]))
    cg.add(var.set_timeout(config[CONF_TIMEOUT]))
