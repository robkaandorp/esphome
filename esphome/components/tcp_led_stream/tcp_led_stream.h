#pragma once
#include "esphome/core/defines.h"
#ifdef USE_NETWORK
#include "esphome/core/component.h"
#include "esphome/components/socket/socket.h"
#include "esphome/components/light/addressable_light.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace tcp_led_stream {

enum PixelFormat { RGB = 0, RGBW = 1, GRB = 2, GRBW = 3, BGR = 4 };

class TCPLedStreamComponent : public Component {
 public:
  void set_light(light::AddressableLightState *light) { light_ = light; }
  void set_port(uint16_t port) { port_ = port; }
  void set_pixel_format(PixelFormat fmt) { format_ = fmt; }
  void set_timeout(uint32_t timeout) { timeout_ms_ = timeout; }
  void set_frame_completion_interval(uint32_t ms) { frame_completion_interval_ms_ = ms; }

  // Sensor setters
  void set_frame_rate_sensor(sensor::Sensor *s) { frame_rate_sensor_ = s; }
  void set_bytes_received_sensor(sensor::Sensor *s) { bytes_received_sensor_ = s; }
  void set_connects_sensor(sensor::Sensor *s) { connects_sensor_ = s; }
  void set_disconnects_sensor(sensor::Sensor *s) { disconnects_sensor_ = s; }
  void set_overlaps_sensor(sensor::Sensor *s) { overlaps_sensor_ = s; }
  void set_client_connected_binary_sensor(binary_sensor::BinarySensor *b) { client_connected_binary_sensor_ = b; }
  void set_completion_mode(const std::string &m) { completion_mode_ = m; }
  void set_show_time_per_led_us(uint32_t v) { show_time_per_led_us_ = v; }

  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }
  void setup() override;
  void loop() override;

 protected:
  bool read_frame_();
  bool apply_pixels_(const uint8_t *data, uint32_t count);
  void publish_stats_();

  light::AddressableLightState *light_{nullptr};
  uint16_t port_{7777};
  PixelFormat format_{RGB};
  uint32_t timeout_ms_{5000};
  uint32_t frame_completion_interval_ms_{15};  // heuristic frame render completion window

  std::unique_ptr<socket::Socket> server_;
  std::unique_ptr<socket::Socket> client_;
  uint32_t last_activity_{0};
  std::vector<uint8_t> rx_buffer_;

  // Stats
  uint32_t frame_count_{0};
  uint32_t bytes_received_{0};
  uint32_t connects_{0};
  uint32_t disconnects_{0};
  uint32_t overlaps_{0};
  uint32_t last_stats_publish_{0};
  uint32_t last_frame_time_{0};
  bool frame_in_progress_{false};
  std::string completion_mode_{"heuristic"};
  uint32_t show_time_per_led_us_{30};  // microseconds per LED (estimate mode)

  // Sensors
  sensor::Sensor *frame_rate_sensor_{nullptr};
  sensor::Sensor *bytes_received_sensor_{nullptr};
  sensor::Sensor *connects_sensor_{nullptr};
  sensor::Sensor *disconnects_sensor_{nullptr};
  sensor::Sensor *overlaps_sensor_{nullptr};
  binary_sensor::BinarySensor *client_connected_binary_sensor_{nullptr};
};

}  // namespace tcp_led_stream
}  // namespace esphome
#endif
