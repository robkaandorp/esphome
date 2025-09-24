#include "tcp_led_stream.h"
#ifdef USE_NETWORK
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace tcp_led_stream {

static const char *const TAG = "tcp_led_stream";
// Frame format:
//  0-3  Magic bytes 'LEDS'
//  4    Version (0x01)
//  5-8  Pixel count (uint32_t big endian)
//  9    Pixel format enum
// 10-?  Pixel data (count * (3|4) bytes)
// No checksum for now (could add CRC32 later)
// Connection sends full frame each time.

void TCPLedStreamComponent::setup() {
  if (light_ == nullptr) {
    ESP_LOGE(TAG, "No light configured");
    this->mark_failed();
    return;
  }
  server_ = socket::socket_ip_loop_monitored(SOCK_STREAM, 0);
  if (!server_) {
    ESP_LOGE(TAG, "Failed to create server socket");
    this->mark_failed();
    return;
  }
  int enable = 1;
  if (server_->setsockopt(SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) != 0) {
    ESP_LOGW(TAG, "setsockopt reuseaddr failed errno=%d", errno);
  }
  if (server_->setblocking(false) != 0) {
    ESP_LOGE(TAG, "Failed to set nonblocking");
    this->mark_failed();
    return;
  }
  struct sockaddr_storage addr;
  socklen_t sl = socket::set_sockaddr_any((struct sockaddr *) &addr, sizeof(addr), port_);
  if (sl == 0) {
    ESP_LOGE(TAG, "Failed to set sockaddr errno=%d", errno);
    this->mark_failed();
    return;
  }
  if (server_->bind((struct sockaddr *) &addr, sl) != 0) {
    ESP_LOGE(TAG, "Bind failed errno=%d", errno);
    this->mark_failed();
    return;
  }
  if (server_->listen(1) != 0) {
    ESP_LOGE(TAG, "Listen failed errno=%d", errno);
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Listening on port %u for LED frames", port_);
  last_stats_publish_ = millis();
  if (client_connected_binary_sensor_ != nullptr) {
    client_connected_binary_sensor_->publish_state(false);
  }
}

bool TCPLedStreamComponent::apply_pixels_(const uint8_t *data, uint32_t count) {
  if (light_ == nullptr)
    return false;
  // AddressableLightState doesn't expose a get_addressable() helper; obtain the underlying output
  auto *addr = static_cast<light::AddressableLight *>(light_->get_output());
  if (addr == nullptr)
    return false;
  uint32_t maxn = std::min(count, (uint32_t) addr->size());
  uint8_t r, g, b, w = 0;
  for (uint32_t i = 0; i < maxn; i++) {
    switch (format_) {
      case RGB:
        r = data[i * 3 + 0];
        g = data[i * 3 + 1];
        b = data[i * 3 + 2];
        w = (r + g + b) / 3;
        break;
      case GRB:
        g = data[i * 3 + 0];
        r = data[i * 3 + 1];
        b = data[i * 3 + 2];
        w = (r + g + b) / 3;
        break;
      case BGR:
        b = data[i * 3 + 0];
        g = data[i * 3 + 1];
        r = data[i * 3 + 2];
        w = (r + g + b) / 3;
        break;
      case RGBW:
        r = data[i * 4 + 0];
        g = data[i * 4 + 1];
        b = data[i * 4 + 2];
        w = data[i * 4 + 3];
        break;
      case GRBW:
        g = data[i * 4 + 0];
        r = data[i * 4 + 1];
        b = data[i * 4 + 2];
        w = data[i * 4 + 3];
        break;
    }
    (*addr)[i].set(Color(r, g, b, w));
  }
  addr->schedule_show();
  return true;
}

bool TCPLedStreamComponent::read_frame_() {
  // attempt to read header (10 bytes)
  uint8_t header[10];
  ssize_t r = client_->read(header, sizeof(header));
  if (r == -1)
    return false;  // no data yet
  if (r != sizeof(header)) {
    ESP_LOGW(TAG, "Short header %d closing", (int) r);
    return false;  // drop connection
  }
  if (memcmp(header, "LEDS", 4) != 0) {
    ESP_LOGW(TAG, "Bad magic");
    return false;
  }
  if (header[4] != 0x01) {
    ESP_LOGW(TAG, "Unsupported version %u", header[4]);
    return false;
  }
  uint32_t count = (header[5] << 24) | (header[6] << 16) | (header[7] << 8) | header[8];
  PixelFormat frame_fmt = (PixelFormat) header[9];
  if (count == 0 || count > 5000) {  // safety limit
    ESP_LOGW(TAG, "Invalid pixel count %u", (unsigned) count);
    return false;
  }
  size_t bpp = (frame_fmt == RGBW || frame_fmt == GRBW) ? 4 : 3;  // treat others as 3
  size_t need = count * bpp;
  rx_buffer_.resize(need);
  size_t got = 0;
  while (got < need) {
    ssize_t n = client_->read(rx_buffer_.data() + got, need - got);
    if (n == -1) {
      // wait for more (nonblocking) - but avoid busy loop
      delay(0);
      continue;
    }
    if (n == 0) {
      ESP_LOGW(TAG, "Client closed during frame");
      return false;
    }
    got += (size_t) n;
  }
  // Overlap detection: if a new frame arrives before previous presumed completion window elapsed
  uint32_t now = millis();
  // Determine dynamic completion window if estimate mode
  uint32_t window_ms = frame_completion_interval_ms_;
  if (completion_mode_ == "estimate") {
    auto *addr = static_cast<light::AddressableLight *>(light_->get_output());
    if (addr != nullptr) {
      // Approximate: number_of_leds * per_led_us / 1000 -> ms, plus small overhead (2ms)
      uint32_t est = (uint64_t) addr->size() * show_time_per_led_us_ / 1000ULL + 2;
      window_ms = est > 1 ? est : 1;
    }
  }
  if (frame_in_progress_ && (now - last_frame_time_ < window_ms)) {
    overlaps_++;
  }
  frame_in_progress_ = true;
  last_frame_time_ = now;
  apply_pixels_(rx_buffer_.data(), count);
  frame_count_++;
  bytes_received_ += (uint32_t) (10 + rx_buffer_.size());
  last_activity_ = now;
  return true;
}

void TCPLedStreamComponent::loop() {
  // Accept new client if none
  if (!client_ && server_ && server_->ready()) {
    struct sockaddr_storage src;
    socklen_t sl = sizeof(src);
    auto sock = server_->accept_loop_monitored((struct sockaddr *) &src, &sl);
    if (sock) {
      client_ = std::move(sock);
      client_->setblocking(false);
      last_activity_ = millis();
      ESP_LOGI(TAG, "Client connected %s", client_->getpeername().c_str());
      connects_++;
      if (client_connected_binary_sensor_ != nullptr) {
        client_connected_binary_sensor_->publish_state(true);
      }
    }
  }
  if (client_) {
    if (!read_frame_()) {
      // either no data yet or error; check timeout
      if (timeout_ms_ && (millis() - last_activity_ > timeout_ms_)) {
        ESP_LOGI(TAG, "Connection timeout");
        client_->close();
        client_.reset();
        disconnects_++;
        frame_in_progress_ = false;
        if (client_connected_binary_sensor_ != nullptr) {
          client_connected_binary_sensor_->publish_state(false);
        }
      }
    }
  }

  // Heuristic: mark frame complete when window elapsed
  uint32_t window_ms2 = frame_completion_interval_ms_;
  if (completion_mode_ == "estimate") {
    auto *addr = static_cast<light::AddressableLight *>(light_->get_output());
    if (addr != nullptr) {
      uint32_t est = (uint64_t) addr->size() * show_time_per_led_us_ / 1000ULL + 2;
      window_ms2 = est > 1 ? est : 1;
    }
  }
  if (frame_in_progress_ && (millis() - last_frame_time_ >= window_ms2)) {
    frame_in_progress_ = false;  // next frame within window will count as overlap
  }

  publish_stats_();
}

void TCPLedStreamComponent::publish_stats_() {
  uint32_t now = millis();
  if (now - last_stats_publish_ < 1000)  // publish every ~1s
    return;
  float seconds = (now - last_stats_publish_) / 1000.0f;
  last_stats_publish_ = now;
  if (frame_rate_sensor_ != nullptr) {
    // compute fps from frames since last publish using diff of frame_count_
    static uint32_t last_frame_count = 0;
    uint32_t diff = frame_count_ - last_frame_count;
    last_frame_count = frame_count_;
    frame_rate_sensor_->publish_state(diff / seconds);
  }
  if (bytes_received_sensor_ != nullptr) {
    bytes_received_sensor_->publish_state(bytes_received_);
  }
  if (connects_sensor_ != nullptr) {
    connects_sensor_->publish_state(connects_);
  }
  if (disconnects_sensor_ != nullptr) {
    disconnects_sensor_->publish_state(disconnects_);
  }
  if (overlaps_sensor_ != nullptr) {
    overlaps_sensor_->publish_state(overlaps_);
  }
}

}  // namespace tcp_led_stream
}  // namespace esphome
#endif
