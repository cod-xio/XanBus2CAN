# Wiring Reference

## ESP32 + MAX3485 (RS485) + MCP2515 (CAN)

### Schematic Overview

```
                   ┌─────────────────────────────────────┐
                   │           ESP32 DevKit v1            │
                   │                                      │
  XanBus A (+) ───►│ MAX3485 ──► GPIO16(RX2)             │
  XanBus B (−) ───►│ MAX3485 ──► GPIO17(TX2)             │
                   │            GPIO4  ──► DE (HIGH=TX)   │
                   │            GPIO5  ──► /RE (LOW=RX)   │
                   │                                      │
  CAN Bus H   ◄────│ MCP2515 ◄── GPIO18(SCK)             │
  CAN Bus L   ◄────│           ◄── GPIO23(MOSI)           │
                   │           ──► GPIO19(MISO)           │
                   │            GPIO5  ──► CS             │
                   │            GPIO15 ◄── INT            │
                   │                                      │
                   │ WiFi ──────────────────► MQTT Broker │
                   └─────────────────────────────────────┘
```

### Detailed Pin Table

| ESP32 GPIO | Direction | Function | Connect To |
|-----------|-----------|----------|------------|
| GPIO 16 (RX2) | IN | RS485 receive | MAX3485 RO (pin 1) |
| GPIO 17 (TX2) | OUT | RS485 transmit | MAX3485 DI (pin 4) |
| GPIO 4 | OUT | RS485 Driver Enable | MAX3485 DE (pin 3) |
| GPIO 5 | OUT | RS485 Receiver Enable | MAX3485 /RE (pin 2) — tie to LOW |
| GPIO 18 | OUT | SPI Clock | MCP2515 SCK (pin 13) |
| GPIO 23 | OUT | SPI MOSI | MCP2515 SI (pin 14) |
| GPIO 19 | IN | SPI MISO | MCP2515 SO (pin 15) |
| GPIO 5 | OUT | SPI Chip Select | MCP2515 CS (pin 16) |
| GPIO 15 | IN | CAN Interrupt | MCP2515 INT (pin 12) |
| GPIO 2 | OUT | Status LED | Built-in LED |
| 3.3V | PWR | VCC | MAX3485 VCC, MCP2515 VCC |
| GND | PWR | Ground | All GNDs |

> Note: DE and /RE can be tied together if the module supports it (half-duplex auto-switch). In this firmware they are separate for full control.

### MAX3485 Pinout Reference

```
         ┌────────┐
  RO  1 ─┤        ├─ 8  VCC (3.3V)
  /RE 2 ─┤MAX3485 ├─ 7  B (−)
  DE  3 ─┤        ├─ 6  A (+)
  DI  4 ─┤        ├─ 5  GND
         └────────┘
```

### MCP2515 Module Pinout (SPI breakout board)

```
INT  ── GPIO 15
SCK  ── GPIO 18
SI   ── GPIO 23 (MOSI)
SO   ── GPIO 19 (MISO)
CS   ── GPIO  5
GND  ── GND
VCC  ── 3.3V (some modules need 5V — check your board!)
```

> ⚠️ If your MCP2515 module has a 5V regulator and 5V-tolerant logic (common Chinese modules with TJA1050), power it from 5V but use a voltage divider or level shifter on MISO. Or use a 3.3V-native module.

### CAN Bus Wiring

```
  ESP32/MCP2515          CAN Bus Devices
  ─────────────         ─────────────────────────────
  CANH ──────────────┬── Device 1 CANH ── Device 2 CANH ──┐
                     │                                    [120Ω]
  CANL ──────────────┴── Device 1 CANL ── Device 2 CANL ──┘
  [120Ω termination at THIS end]
```

### XanBus Network Topology

```
  Schneider XW+          Schneider MPPT        SCP Panel
  ──────────────         ──────────────         ──────────
  XanBus OUT ────────── XanBus IN/OUT ───────── XanBus IN
  (RJ45)                 (RJ45)                 (RJ45)
                              │
                         Tap here for
                         RS485 A/B wires
                         → MAX3485
```

Connect your RS485 transceiver anywhere along the XanBus daisy-chain. XanBus uses a bus topology — you can tap in at any point. Make sure to only have 120Ω termination at the **physical ends** of the bus.

### Power Budget

| Component | Current Draw |
|-----------|-------------|
| ESP32 (WiFi active) | ~240 mA peak, ~80 mA avg |
| MAX3485 | < 1 mA |
| MCP2515 module | ~15 mA |
| **Total** | **~100 mA avg from 3.3V** |

The XanBus RJ45 Pin 4 provides +12V @ 500 mA — enough to power the ESP32 via a small 12V→5V buck converter + LDO.
