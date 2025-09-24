#pragma once
#include "esphome/core/defines.h"
#ifdef USE_NETWORK
#include "esphome/core/component.h"
#include "esphome/components/socket/socket.h"
#include "esphome/components/light/addressable_light.h"

namespace esphome {
namespace tcp_led_stream {

enum PixelFormat { RGB = 0, RGBW = 1, GRB = 2, GRBW = 3, BGR = 4 };

class TCPLedStreamComponent : public Component {
 public:
  void set_light(light::AddressableLightState *light) { light_ = light; }
  void set_port(uint16_t port) { port_ = port; }
  void set_pixel_format(PixelFormat fmt) { format_ = fmt; }
  void set_timeout(uint32_t timeout) { timeout_ms_ = timeout; }

  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }
  void setup() override;
  void loop() override;

 protected:
  bool read_frame_();
  bool apply_pixels_(const uint8_t *data, uint32_t count);

  light::AddressableLightState *light_{nullptr};
  uint16_t port_{7777};
  PixelFormat format_{RGB};
  uint32_t timeout_ms_{5000};

  std::unique_ptr<socket::Socket> server_;
  std::unique_ptr<socket::Socket> client_;
  uint32_t last_activity_{0};
  std::vector<uint8_t> rx_buffer_;
};

}  // namespace tcp_led_stream
}  // namespace esphome
#endif
