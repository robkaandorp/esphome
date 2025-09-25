#include "tcp_led_stream.h"
#ifdef USE_NETWORK
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/application.h"

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
  if (this->light_ == nullptr) {
    ESP_LOGE(TAG, "No light configured");
    this->mark_failed();
    return;
  }
  this->server_ = socket::socket_ip_loop_monitored(SOCK_STREAM, 0);
  if (!this->server_) {
    ESP_LOGE(TAG, "Failed to create server socket");
    this->mark_failed();
    return;
  }
  int enable = 1;
  if (this->server_->setsockopt(SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable)) != 0) {
    ESP_LOGW(TAG, "setsockopt reuseaddr failed errno=%d", errno);
  }
  if (this->server_->setblocking(false) != 0) {
    ESP_LOGE(TAG, "Failed to set nonblocking");
    this->mark_failed();
    return;
  }
  struct sockaddr_storage addr;
  socklen_t sl = socket::set_sockaddr_any((struct sockaddr *) &addr, sizeof(addr), this->port_);
  if (sl == 0) {
    ESP_LOGE(TAG, "Failed to set sockaddr errno=%d", errno);
    this->mark_failed();
    return;
  }
  if (this->server_->bind((struct sockaddr *) &addr, sl) != 0) {
    ESP_LOGE(TAG, "Bind failed errno=%d", errno);
    this->mark_failed();
    return;
  }
  if (this->server_->listen(1) != 0) {
    ESP_LOGE(TAG, "Listen failed errno=%d", errno);
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Listening on port %u for LED frames", this->port_);
  this->last_stats_publish_ = App.get_loop_component_start_time();
#ifdef USE_BINARY_SENSOR
  if (this->client_connected_binary_sensor_ != nullptr) {
    this->client_connected_binary_sensor_->publish_state(false);
  }
#endif
}

bool TCPLedStreamComponent::apply_pixels_(const uint8_t *data, uint32_t count) {
  if (this->light_ == nullptr)
    return false;
  auto *addr = static_cast<light::AddressableLight *>(this->light_->get_output());
  if (addr == nullptr)
    return false;
  uint32_t maxn = std::min(count, (uint32_t) addr->size());
  uint8_t r, g, b, w = 0;
  for (uint32_t i = 0; i < maxn; i++) {
    switch (this->format_) {
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
  // State machine static data (kept as members if expanded later)
  uint8_t header[10];
  ssize_t r = this->client_->read(header, sizeof(header));
  if (r == -1) {
    return false;  // nothing this loop
  }
  if (r != sizeof(header)) {
    ESP_LOGW(TAG, "Short/partial header (%d) - closing", (int) r);
    return false;
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
  if (count == 0 || count > 5000) {
    ESP_LOGW(TAG, "Invalid pixel count %u", (unsigned) count);
    return false;
  }
  size_t bpp = (frame_fmt == RGBW || frame_fmt == GRBW) ? 4 : 3;
  size_t need = count * bpp;
  this->rx_buffer_.resize(need);
  size_t got = 0;
  while (got < need) {
    ssize_t n = this->client_->read(this->rx_buffer_.data() + got, need - got);
    if (n == -1) {
      // exit early, will continue next loop iteration (non-blocking)
      break;
    }
    if (n == 0) {
      ESP_LOGW(TAG, "Client closed during frame");
      return false;
    }
    got += (size_t) n;
  }
  if (got < need) {
    // Incomplete frame this iteration – treat as not ready yet.
    return true;  // keep connection, but don't process
  }
  // Overlap detection: if a new frame arrives before previous presumed completion window elapsed
  uint32_t now = App.get_loop_component_start_time();
  // Determine dynamic completion window if estimate mode
  uint32_t window_ms = this->frame_completion_interval_ms_;
  if (this->completion_mode_ == "estimate") {
    auto *addr = static_cast<light::AddressableLight *>(this->light_->get_output());
    if (addr != nullptr) {
      uint32_t est = (uint64_t) addr->size() * this->show_time_per_led_us_ / 1000ULL + 2;
      window_ms = est > 1 ? est : 1;
    }
  }
  if (this->frame_in_progress_ && (now - this->last_frame_time_ < window_ms)) {
    this->overlaps_++;
  }
  this->frame_in_progress_ = true;
  this->last_frame_time_ = now;
  this->apply_pixels_(this->rx_buffer_.data(), count);
  this->frame_count_++;
  this->bytes_received_ += (uint32_t) (10 + this->rx_buffer_.size());
  this->last_activity_ = now;
  return true;
}

void TCPLedStreamComponent::loop() {
  // Accept new client if none
  if (!this->client_ && this->server_ && this->server_->ready()) {
    struct sockaddr_storage src;
    socklen_t sl = sizeof(src);
    auto sock = this->server_->accept_loop_monitored((struct sockaddr *) &src, &sl);
    if (sock) {
      this->client_ = std::move(sock);
      this->client_->setblocking(false);
      this->last_activity_ = App.get_loop_component_start_time();
      ESP_LOGI(TAG, "Client connected %s", this->client_->getpeername().c_str());
      this->connects_++;
#ifdef USE_BINARY_SENSOR
      if (this->client_connected_binary_sensor_ != nullptr) {
        this->client_connected_binary_sensor_->publish_state(true);
      }
#endif
    }
  }
  if (this->client_) {
    if (!this->read_frame_()) {
      if (this->timeout_ms_ && (App.get_loop_component_start_time() - this->last_activity_ > this->timeout_ms_)) {
        ESP_LOGI(TAG, "Connection timeout");
        this->client_->close();
        this->client_.reset();
        this->disconnects_++;
        this->frame_in_progress_ = false;
#ifdef USE_BINARY_SENSOR
        if (this->client_connected_binary_sensor_ != nullptr) {
          this->client_connected_binary_sensor_->publish_state(false);
        }
#endif
      }
    }
  }

  // Heuristic: mark frame complete when window elapsed
  uint32_t window_ms2 = this->frame_completion_interval_ms_;
  if (this->completion_mode_ == "estimate") {
    auto *addr = static_cast<light::AddressableLight *>(this->light_->get_output());
    if (addr != nullptr) {
      uint32_t est = (uint64_t) addr->size() * this->show_time_per_led_us_ / 1000ULL + 2;
      window_ms2 = est > 1 ? est : 1;
    }
  }
  if (this->frame_in_progress_ && (App.get_loop_component_start_time() - this->last_frame_time_ >= window_ms2)) {
    this->frame_in_progress_ = false;
  }

  this->publish_stats_();
}

void TCPLedStreamComponent::publish_stats_() {
  uint32_t now = App.get_loop_component_start_time();
  if (now - this->last_stats_publish_ < 1000)
    return;
  float seconds = (now - this->last_stats_publish_) / 1000.0f;
  this->last_stats_publish_ = now;
  static uint32_t last_frame_count = 0;  // acceptable static for diff calculation
  if (this->frame_rate_sensor_ != nullptr) {
    uint32_t diff = this->frame_count_ - last_frame_count;
    this->frame_rate_sensor_->publish_state(diff / seconds);
  }
  last_frame_count = this->frame_count_;
  if (this->bytes_received_sensor_ != nullptr) {
    this->bytes_received_sensor_->publish_state(this->bytes_received_);
  }
  if (this->connects_sensor_ != nullptr) {
    this->connects_sensor_->publish_state(this->connects_);
  }
  if (this->disconnects_sensor_ != nullptr) {
    this->disconnects_sensor_->publish_state(this->disconnects_);
  }
  if (this->overlaps_sensor_ != nullptr) {
    this->overlaps_sensor_->publish_state(this->overlaps_);
  }
}

void TCPLedStreamComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "TCP LED Stream:");
  ESP_LOGCONFIG(TAG, "  Port: %u", this->port_);
  ESP_LOGCONFIG(TAG, "  Timeout (ms): %u", this->timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Completion mode: %s", this->completion_mode_.c_str());
  ESP_LOGCONFIG(TAG, "  Frame completion interval (ms): %u", this->frame_completion_interval_ms_);
  ESP_LOGCONFIG(TAG, "  Show time per LED (us): %u", this->show_time_per_led_us_);
}

}  // namespace tcp_led_stream
}  // namespace esphome
#endif
