#pragma once
#include "esphome/core/defines.h"
#ifdef USE_NETWORK
#include "esphome/core/component.h"
#include "esphome/components/socket/socket.h"
#include "esphome/components/light/addressable_light.h"
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

namespace esphome {
namespace tcp_led_stream {

enum PixelFormat { RGB = 0, RGBW = 1, GRB = 2, GRBW = 3, BGR = 4 };

class TCPLedStreamComponent : public Component {
 public:
  void set_light(light::AddressableLightState *light) { this->light_ = light; }
  void set_port(uint16_t port) { this->port_ = port; }
  void set_pixel_format(PixelFormat fmt) { this->format_ = fmt; }
  void set_timeout(uint32_t timeout) { this->timeout_ms_ = timeout; }
  void set_frame_completion_interval(uint32_t ms) { this->frame_completion_interval_ms_ = ms; }

  // Sensor setters
#ifdef USE_SENSOR
  void set_frame_rate_sensor(sensor::Sensor *s) { this->frame_rate_sensor_ = s; }
  void set_bytes_received_sensor(sensor::Sensor *s) { this->bytes_received_sensor_ = s; }
  void set_connects_sensor(sensor::Sensor *s) { this->connects_sensor_ = s; }
  void set_disconnects_sensor(sensor::Sensor *s) { this->disconnects_sensor_ = s; }
  void set_overlaps_sensor(sensor::Sensor *s) { this->overlaps_sensor_ = s; }
#else
  void set_frame_rate_sensor(void *s) {}
  void set_bytes_received_sensor(void *s) {}
  void set_connects_sensor(void *s) {}
  void set_disconnects_sensor(void *s) {}
  void set_overlaps_sensor(void *s) {}
#endif
  void set_client_connected_binary_sensor(
#ifdef USE_BINARY_SENSOR
      binary_sensor::BinarySensor *b
#else
      void *b
#endif
  ) {
#ifdef USE_BINARY_SENSOR
    this->client_connected_binary_sensor_ = b;
#endif
  }
  void set_completion_mode(const std::string &m) { this->completion_mode_ = m; }
  void set_show_time_per_led_us(uint32_t v) { this->show_time_per_led_us_ = v; }

  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }
  void setup() override;
  void loop() override;
  void dump_config() override;

 protected:
  bool read_frame_();
  bool apply_pixels_(const uint8_t *data, uint32_t count);
  void publish_stats_();
  void reset_receive_state_();

  light::AddressableLightState *light_{nullptr};
  uint16_t port_{7777};
  PixelFormat format_{RGB};
  uint32_t timeout_ms_{5000};
  uint32_t frame_completion_interval_ms_{15};  // heuristic frame render completion window

  std::unique_ptr<socket::Socket> server_;
  std::unique_ptr<socket::Socket> client_;
  uint32_t last_activity_{0};
  std::vector<uint8_t> rx_buffer_;

  // TCP stream buffering state
  enum ReceiveState { WAITING_HEADER, WAITING_PAYLOAD };
  ReceiveState receive_state_{WAITING_HEADER};
  uint8_t header_buffer_[10];
  size_t header_bytes_received_{0};
  uint32_t expected_payload_size_{0};
  size_t payload_bytes_received_{0};

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
#ifdef USE_SENSOR
  sensor::Sensor *frame_rate_sensor_{nullptr};
  sensor::Sensor *bytes_received_sensor_{nullptr};
  sensor::Sensor *connects_sensor_{nullptr};
  sensor::Sensor *disconnects_sensor_{nullptr};
  sensor::Sensor *overlaps_sensor_{nullptr};
#endif
#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *client_connected_binary_sensor_{nullptr};
#endif
};

}  // namespace tcp_led_stream
}  // namespace esphome
#endif
