# Changelog

## [v1.1.0] - 2024-03-12
### Added
- ESP32-C3 SuperMini support (RISC-V, single-core)
  - Automatic board detection via `CONFIG_IDF_TARGET_ESP32C3`
  - UART1 used for RS485 (UART0 reserved for USB/debug)
  - WS2812B RGB LED driver (no external library needed)
  - Adjusted SPI pins to avoid flash memory conflict
- ESP32-S3 environment in platformio.ini
- GitHub Actions CI/CD: build all 3 boards on every push
- Automatic firmware binary release on git tag `v*`
- Python bridge demo mode tested in CI

### Changed
- `Serial2` replaced with `RS485_SERIAL` macro for board-agnostic code
- `platformio.ini` restructured with `[common]` section for shared deps
- Fixed MCP2515 library reference (GitHub URL instead of registry)

### Fixed
- LED blink logic unified across boards via `led_set_color()` helper

## [v1.0.0] - 2024-03-10
### Added
- Initial release
- ESP32 DevKit v1 firmware (RS485 XanBus decoder + MCP2515 CAN + MQTT)
- Python host bridge with FastAPI REST/WebSocket server
- Real-time web dashboard (single-file HTML)
- SQLite + CSV data logging
- Full NMEA 2000 / XanBus PGN decoder
- Schneider proprietary PGN support (0x1FF00 range)
