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
