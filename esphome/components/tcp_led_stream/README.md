# TCP LED Stream Component (experimental)

Stream full frames of addressable LED pixel data to ESPHome over a raw TCP connection.

Frame format (single write per frame):
- Bytes 0-3: ASCII `LEDS`
- Byte 4: Protocol version (0x01)
- Bytes 5-8: Pixel count (big-endian uint32)
- Byte 9: Pixel format enum (0=RGB,1=RGBW,2=GRB,3=GRBW,4=BGR)
- Bytes 10..: Pixel data tightly packed (count * bytes_per_pixel)

Bytes per pixel: 3 for RGB/GRB/BGR, 4 for RGBW/GRBW.

Example YAML:
```yaml
# Required component declarations for tcp_led_stream
socket:         # Required for TCP networking
sensor:         # Required if using any sensor entities (frame_rate, bytes_received, etc.)
binary_sensor:  # Required if using client_connected binary sensor

light:
  - platform: neopixelbus
    id: strip
    pin: GPIO3
    num_leds: 1200

tcp_led_stream:
  id: led_stream
  light_id: strip
  port: 7777
  pixel_format: RGB
  timeout: 5000
```

Send a frame from Python:
```python
import socket, struct
HOST='esp.local'; PORT=7777
count=1200
pixels=bytearray([0,0,0]*count)  # fill with your RGB data
hdr=b'LEDS'+bytes([1])+struct.pack('>I', count)+bytes([0])
s=socket.create_connection((HOST,PORT))
s.sendall(hdr+pixels)
```

Future ideas: optional CRC, chunked streaming, gzip, authentication, multi-client broadcast.

## Diagnostics & Sensors

**Important**: When using any sensor entities, you must explicitly declare the required component dependencies in your YAML configuration. ESPHome requires these declarations even when components are used indirectly by other components:

- `socket`: Always required for TCP networking functionality
- `sensor`: Required only if using sensor entities (`frame_rate`, `bytes_received`, `connects`, `disconnects`, `overlaps`)
- `binary_sensor`: Required only if using the `client_connected` binary sensor entity

You can expose runtime statistics as sensors:

```yaml
# Required component declarations
socket:         # Required for TCP networking
sensor:         # Required for sensor entities
binary_sensor:  # Required for binary sensor entities

tcp_led_stream:
  id: led_stream
  light_id: strip
  port: 7777
  pixel_format: RGB
  timeout: 5000
  frame_completion_interval: 15   # ms window to consider frame rendering "busy"
  completion_mode: heuristic       # or 'estimate'
  show_time_per_led_us: 30         # only used in estimate mode (microseconds per LED)
  frame_rate:
    name: LED Stream FPS
  bytes_received:
    name: LED Stream Bytes
  connects:
    name: LED Stream Connects
  disconnects:
    name: LED Stream Disconnects
  overlaps:
    name: LED Stream Overlaps
  client_connected:
    name: LED Stream Client Connected
```

Sensors:
- `frame_rate` (fps): Frames applied per second (rolling, published ~1s).
- `bytes_received` (B): Cumulative bytes received including headers.
- `connects`: Total successful TCP client connections.
- `disconnects`: Total disconnects/timeouts.
- `overlaps`: Count of frames that arrived before the previous frame's completion window elapsed.
- `client_connected` (binary): True while a TCP client is actively connected.

`frame_completion_interval` (heuristic mode) defines a fixed window (ms) after a frame is received during which a new frame counts as an overlap.

`completion_mode`:
- `heuristic` (default): Uses the fixed `frame_completion_interval`.
- `estimate`: Dynamically estimates completion time from LED count * `show_time_per_led_us` + small overhead (2ms). This can better reflect long strips where refresh time scales with length.

`show_time_per_led_us` is the assumed microseconds each LED requires for transfer + latch (e.g. WS2812 ~30 µs/LED including reset overhead averaged). Adjust based on your physical strip and driver method. A too-small value underestimates overlap; a too-large value may over-count overlaps.
