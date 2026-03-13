#!/usr/bin/env python3
"""
XanBus to CAN Bridge - Python Host Software
Schneider Electric / Xantrex XanBus Protocol Decoder & Analyzer

Features:
  - Full XanBus (NMEA 2000 over RS485) decoding
  - CAN bus output via python-can
  - MQTT publishing
  - Real-time data logging (SQLite + CSV)
  - FastAPI REST interface for GUI
  - Protocol dissector with all Schneider PGNs

Requirements:
  pip install pyserial python-can paho-mqtt fastapi uvicorn sqlalchemy aiofiles
"""

import asyncio
import struct
import logging
import time
import json
import math
import sqlite3
import csv
from   pathlib      import Path
from   dataclasses  import dataclass, field, asdict
from   typing       import Optional, Dict, List, Callable, Any
from   datetime     import datetime
from   enum         import IntEnum

import serial
import serial.threaded
import paho.mqtt.client as mqtt
from   fastapi      import FastAPI, WebSocket, WebSocketDisconnect
from   fastapi.responses import HTMLResponse, FileResponse
from   fastapi.staticfiles import StaticFiles
import uvicorn
import threading

try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False
    logging.warning("python-can not available – CAN output disabled")

# ─── LOGGING ──────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S"
)
log = logging.getLogger("xanbus")

# ─── CONFIGURATION ────────────────────────────────────────────────────────────
class Config:
    # RS485 Serial Port
    RS485_PORT     = "/dev/ttyUSB0"      # or "COM3" on Windows
    RS485_BAUDRATE = 250000              # XanBus speed
    RS485_TIMEOUT  = 0.1

    # CAN Interface
    CAN_INTERFACE  = "socketcan"         # or "kvaser", "pcan", "virtual"
    CAN_CHANNEL    = "can0"
    CAN_BITRATE    = 250000

    # MQTT
    MQTT_BROKER    = "localhost"
    MQTT_PORT      = 1883
    MQTT_USER      = ""
    MQTT_PASS      = ""
    MQTT_QOS       = 1

    # Logging
    LOG_DIR        = Path("logs")
    DB_FILE        = Path("xanbus_data.db")
    CSV_LOG        = True
    DB_LOG         = True

    # API Server
    API_HOST       = "0.0.0.0"
    API_PORT       = 8080

    # Frame timeouts (ms)
    DATA_STALE_MS  = 5000


# ─── XANBUS PGN TABLE ─────────────────────────────────────────────────────────
class PGN(IntEnum):
    # Standard NMEA 2000
    ISO_ADDR_CLAIM       = 0x0EE00   # 60416
    ISO_REQUEST          = 0x0EA00   # 59904
    PRODUCT_INFO         = 0x1EF00   # 126996
    HEARTBEAT            = 0x1EF01   # 126997
    SYS_TIME             = 0x1F010   # 126992
    VESSEL_HEADING       = 0x1F112   # 127250  (not typical but possible)
    DC_BATTERY_STATUS    = 0x1F214   # 127508
    DC_DETAILED_STATUS   = 0x1F213   # 127507
    CHARGER_STATUS       = 0x1F21C   # 127516
    CHARGER_AC_STATUS    = 0x1F21E   # 127518
    INVERTER_STATUS      = 0x1F21D   # 127517
    INVERTER_AC_STATUS   = 0x1F21F   # 127519
    AC_INPUT             = 0x1F211   # 127505
    AC_OUTPUT            = 0x1F212   # 127506
    SOLAR_CONTROLLER     = 0x1F218   # 127512
    SOLAR_HISTORY        = 0x1F219   # 127513
    TEMPERATURE          = 0x1FD08   # 130312
    # Schneider/Xantrex proprietary
    XB_BATTERY_EXT      = 0x1FF00
    XB_SYSTEM_MODE       = 0x1FF01
    XB_FAULT_STATUS      = 0x1FF02
    XB_CONFIG_REQUEST    = 0x1FF10
    XB_CONFIG_RESPONSE   = 0x1FF11
    XB_LOAD_STATUS       = 0x1FF20
    XB_GRID_STATUS       = 0x1FF21


PGN_NAMES = {v: v.name for v in PGN}

# Fast-packet PGNs (>8 bytes data)
FAST_PACKET_PGNS = {
    PGN.PRODUCT_INFO, PGN.CHARGER_STATUS, PGN.INVERTER_STATUS,
    PGN.CHARGER_AC_STATUS, PGN.INVERTER_AC_STATUS, PGN.SOLAR_HISTORY,
}

# ─── CAN ID MAP ───────────────────────────────────────────────────────────────
CAN_ID = {
    "battery_pack"    : 0x100,
    "battery_soc"     : 0x101,
    "battery_temp"    : 0x102,
    "solar_basic"     : 0x110,
    "solar_detail"    : 0x111,
    "inverter_ac"     : 0x120,
    "inverter_state"  : 0x121,
    "charger_dc"      : 0x130,
    "charger_state"   : 0x131,
    "system_status"   : 0x140,
    "fault_warning"   : 0x150,
}


# ─── DATA CLASSES ─────────────────────────────────────────────────────────────
@dataclass
class BatteryData:
    voltage:      Optional[float] = None   # V
    current:      Optional[float] = None   # A
    soc:          Optional[float] = None   # %
    temperature:  Optional[float] = None   # °C
    capacity_ah:  Optional[float] = None   # Ah
    state:        str             = "unknown"
    instance:     int             = 0
    ts:           float           = field(default_factory=time.time)

@dataclass
class SolarData:
    pv_voltage:      Optional[float] = None
    pv_current:      Optional[float] = None
    pv_power:        Optional[float] = None
    output_voltage:  Optional[float] = None
    output_current:  Optional[float] = None
    daily_yield_wh:  Optional[float] = None
    total_yield_kwh: Optional[float] = None
    controller_state: str            = "Off"
    instance:        int             = 0
    ts:              float           = field(default_factory=time.time)

@dataclass
class InverterData:
    ac_voltage:    Optional[float] = None
    ac_current:    Optional[float] = None
    ac_frequency:  Optional[float] = None
    ac_power:      Optional[float] = None
    dc_input_v:    Optional[float] = None
    efficiency:    Optional[float] = None
    state:         str             = "Off"
    instance:      int             = 0
    ts:            float           = field(default_factory=time.time)

@dataclass
class ChargerData:
    output_voltage: Optional[float] = None
    output_current: Optional[float] = None
    input_voltage:  Optional[float] = None
    input_current:  Optional[float] = None
    input_frequency:Optional[float] = None
    mode:           str             = "Off"
    efficiency:     Optional[float] = None
    instance:       int             = 0
    ts:             float           = field(default_factory=time.time)

@dataclass
class XanBusFrame:
    priority:    int
    pgn:         int
    source:      int
    destination: int
    data:        bytes
    ts:          float = field(default_factory=time.time)
    pgn_name:    str   = ""


# ─── CRC16 (CCITT / XanBus variant) ──────────────────────────────────────────
def crc16_xanbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


# ─── DECODE HELPERS ───────────────────────────────────────────────────────────
def d_u16(data: bytes, off: int, res: float = 1.0) -> Optional[float]:
    """Decode unsigned 16-bit LE with resolution. Returns None if NA."""
    if off + 2 > len(data): return None
    raw = struct.unpack_from("<H", data, off)[0]
    if raw == 0xFFFF: return None
    return raw * res

def d_i16(data: bytes, off: int, res: float = 1.0) -> Optional[float]:
    if off + 2 > len(data): return None
    raw = struct.unpack_from("<h", data, off)[0]
    if raw == 0x7FFF or raw == -32768: return None
    return raw * res

def d_u32(data: bytes, off: int, res: float = 1.0) -> Optional[float]:
    if off + 4 > len(data): return None
    raw = struct.unpack_from("<I", data, off)[0]
    if raw == 0xFFFFFFFF: return None
    return raw * res

def d_u8(data: bytes, off: int) -> Optional[int]:
    if off >= len(data): return None
    return data[off]

def d_bits(byte: int, start: int, count: int) -> int:
    return (byte >> start) & ((1 << count) - 1)


# ─── XANBUS DECODER ───────────────────────────────────────────────────────────
class XanBusDecoder:
    """
    Decodes XanBus (NMEA 2000 over RS485) frames from raw serial bytes.
    Handles both single-frame (≤8 bytes) and fast-packet (multi-frame) PGNs.
    """

    SOF = 0xAA
    EOF = 0x55

    CHARGER_MODES  = ["Off", "Bulk", "Absorption", "Float", "Equalize", "Constant-V"]
    INVERTER_STATES= ["Off", "Standby", "Inverting", "Fault", "Test"]
    SOLAR_STATES   = ["Off", "MPPT", "Bulk", "Absorption", "Float", "Equalize", "Current-Limit"]
    BATTERY_STATES = ["Unknown", "Charging", "Discharging", "Float", "Equalizing"]

    def __init__(self):
        self._buf: bytearray = bytearray()
        self._fp_bufs: Dict[int, dict] = {}   # fast-packet reassembly per source
        self.callbacks: Dict[str, List[Callable]] = {}
        self.raw_frame_cb: Optional[Callable] = None

        # Decoded data
        self.battery  = BatteryData()
        self.solar    = SolarData()
        self.inverter = InverterData()
        self.charger  = ChargerData()

        # Stats
        self.stats = {
            "frames_total": 0, "frames_ok": 0, "crc_errors": 0,
            "parse_errors": 0, "unknown_pgns": 0, "fast_packets": 0
        }

    def on(self, event: str, callback: Callable):
        self.callbacks.setdefault(event, []).append(callback)

    def _emit(self, event: str, data: Any):
        for cb in self.callbacks.get(event, []):
            try: cb(data)
            except Exception as e: log.error(f"Callback error [{event}]: {e}")

    def feed(self, raw: bytes):
        """Feed raw bytes from serial port."""
        self._buf.extend(raw)
        self._process_buf()

    def _process_buf(self):
        while len(self._buf) >= 6:
            # Find SOF
            try:
                sof_idx = self._buf.index(self.SOF)
            except ValueError:
                self._buf.clear()
                return
            if sof_idx > 0:
                self._buf = self._buf[sof_idx:]

            # Try to find EOF (minimum frame = SOF + 4hdr + 0data + 2crc + EOF = 8 bytes)
            if len(self._buf) < 8:
                break

            # Search for EOF within reasonable frame size (max ~240 bytes)
            eof_idx = None
            for i in range(7, min(len(self._buf), 250)):
                if self._buf[i] == self.EOF:
                    eof_idx = i
                    break

            if eof_idx is None:
                if len(self._buf) > 250:
                    self._buf = self._buf[1:]  # Skip bad SOF
                break

            frame_raw = bytes(self._buf[1:eof_idx])   # Strip SOF/EOF
            self._buf  = self._buf[eof_idx + 1:]

            self.stats["frames_total"] += 1

            # Validate CRC (last 2 bytes of frame_raw)
            if len(frame_raw) < 6:
                self.stats["parse_errors"] += 1
                continue

            payload = frame_raw[:-2]
            recv_crc = struct.unpack_from(">H", frame_raw, len(frame_raw) - 2)[0]
            calc_crc = crc16_xanbus(payload)

            if recv_crc != calc_crc:
                self.stats["crc_errors"] += 1
                log.debug(f"CRC error: expected {calc_crc:04X} got {recv_crc:04X}")
                continue

            self.stats["frames_ok"] += 1
            self._parse_frame(payload)

    def _parse_frame(self, payload: bytes):
        """Parse a validated frame payload (header + data)."""
        if len(payload) < 4:
            self.stats["parse_errors"] += 1
            return

        # 29-bit CAN-style header
        hdr = struct.unpack_from(">I", payload)[0]
        priority    = (hdr >> 26) & 0x07
        pgn_raw     = (hdr >> 8)  & 0x03FFFF
        source      = hdr & 0xFF
        destination = 0xFF
        data        = payload[4:]

        # PDU1: destination encoded in byte 2 of PGN
        if (pgn_raw >> 8) < 240:
            destination = (pgn_raw >> 8) & 0xFF
            pgn_raw     = pgn_raw & 0x03FF00

        pgn = pgn_raw
        pgn_name = PGN_NAMES.get(pgn, f"0x{pgn:05X}")

        frame = XanBusFrame(
            priority=priority, pgn=pgn, source=source,
            destination=destination, data=data, pgn_name=pgn_name
        )

        if self.raw_frame_cb:
            self.raw_frame_cb(frame)

        # Fast-packet check
        if pgn in FAST_PACKET_PGNS and len(data) > 0:
            complete_data = self._reassemble_fast_packet(pgn, source, data)
            if complete_data is not None:
                frame.data = complete_data
                self._dispatch_pgn(frame)
        else:
            self._dispatch_pgn(frame)

    def _reassemble_fast_packet(self, pgn: int, source: int, data: bytes) -> Optional[bytes]:
        key = (pgn, source)
        byte0    = data[0]
        seq_id   = (byte0 >> 5) & 0x07
        frame_no = byte0 & 0x1F

        if frame_no == 0:
            if len(data) < 2: return None
            total_bytes = data[1]
            self._fp_bufs[key] = {
                "seq":      seq_id,
                "total":    total_bytes,
                "buf":      bytearray(total_bytes),
                "received": 0,
                "ts":       time.time()
            }
            chunk = data[2:]
            fp = self._fp_bufs[key]
            end = min(len(chunk), total_bytes)
            fp["buf"][:end] = chunk[:end]
            fp["received"] += end
            self.stats["fast_packets"] += 1
            return None

        fp = self._fp_bufs.get(key)
        if fp is None or fp["seq"] != seq_id:
            return None
        if time.time() - fp["ts"] > 0.5:  # Stale
            del self._fp_bufs[key]
            return None

        offset = 6 + (frame_no - 1) * 7
        chunk  = data[1:]
        end    = min(offset + len(chunk), fp["total"])
        if end > offset:
            fp["buf"][offset:end] = chunk[:end - offset]
            fp["received"] += end - offset

        if fp["received"] >= fp["total"]:
            result = bytes(fp["buf"])
            del self._fp_bufs[key]
            return result
        return None

    def _dispatch_pgn(self, frame: XanBusFrame):
        pgn = frame.pgn
        d   = frame.data
        src = frame.source

        try:
            if   pgn == PGN.DC_BATTERY_STATUS:   self._parse_battery_status(d, src)
            elif pgn == PGN.DC_DETAILED_STATUS:  self._parse_dc_detailed(d, src)
            elif pgn == PGN.SOLAR_CONTROLLER:    self._parse_solar(d, src)
            elif pgn == PGN.SOLAR_HISTORY:       self._parse_solar_history(d, src)
            elif pgn == PGN.INVERTER_STATUS:     self._parse_inverter(d, src)
            elif pgn == PGN.INVERTER_AC_STATUS:  self._parse_inverter_ac(d, src)
            elif pgn == PGN.CHARGER_STATUS:      self._parse_charger(d, src)
            elif pgn == PGN.CHARGER_AC_STATUS:   self._parse_charger_ac(d, src)
            elif pgn == PGN.AC_INPUT:            self._parse_ac_input(d, src)
            elif pgn == PGN.AC_OUTPUT:           self._parse_ac_output(d, src)
            elif pgn == PGN.TEMPERATURE:         self._parse_temperature(d, src)
            elif pgn == PGN.XB_BATTERY_EXT:      self._parse_xb_battery_ext(d, src)
            elif pgn == PGN.XB_FAULT_STATUS:     self._parse_xb_fault(d, src)
            elif pgn == PGN.XB_SYSTEM_MODE:      self._parse_xb_system_mode(d, src)
            elif pgn == PGN.PRODUCT_INFO:        self._parse_product_info(d, src)
            else:
                self.stats["unknown_pgns"] += 1
                self._emit("unknown_pgn", {"pgn": pgn, "source": src,
                                            "data": d.hex(), "ts": time.time()})
        except Exception as e:
            self.stats["parse_errors"] += 1
            log.error(f"Parse error PGN 0x{pgn:05X}: {e}", exc_info=True)

    # ── PGN Parsers ────────────────────────────────────────────────────────────

    def _parse_battery_status(self, d: bytes, src: int):
        """PGN 127508 - Battery Status"""
        self.battery.instance    = d[0] if d else 0
        self.battery.voltage     = d_u16(d, 1, 0.01)
        self.battery.current     = d_i16(d, 3, 0.1)
        k = d_u16(d, 5, 0.01)
        self.battery.temperature = round(k - 273.15, 1) if k is not None else None
        self.battery.ts          = time.time()
        self._emit("battery", asdict(self.battery))
        log.debug(f"Battery: {self.battery.voltage:.2f}V {self.battery.current:.1f}A SOC={self.battery.soc}%")

    def _parse_dc_detailed(self, d: bytes, src: int):
        """PGN 127507 - DC Detailed Status (SOC)"""
        if len(d) < 7: return
        soc = d_u16(d, 2, 0.004)
        if soc is not None:
            self.battery.soc = round(soc, 1)
        self.battery.capacity_ah = d_u16(d, 4, 1.0)
        self._emit("battery", asdict(self.battery))

    def _parse_solar(self, d: bytes, src: int):
        """PGN 127512 - Solar Controller"""
        if len(d) < 8: return
        self.solar.instance        = d[0]
        state_raw                  = d_bits(d[1], 0, 4)
        self.solar.controller_state= self.SOLAR_STATES[state_raw] if state_raw < len(self.SOLAR_STATES) else "Unknown"
        self.solar.output_voltage  = d_u16(d, 2, 0.01)
        self.solar.output_current  = d_i16(d, 4, 0.1)
        self.solar.pv_voltage      = d_u16(d, 6, 0.01)
        if self.solar.output_voltage and self.solar.output_current:
            self.solar.pv_power = round(self.solar.output_voltage * self.solar.output_current, 0)
        self.solar.ts              = time.time()
        self._emit("solar", asdict(self.solar))

    def _parse_solar_history(self, d: bytes, src: int):
        """PGN 127513 - Solar History (daily/total yield)"""
        if len(d) < 12: return
        self.solar.daily_yield_wh  = d_u32(d, 0, 1.0)
        self.solar.total_yield_kwh = d_u32(d, 4, 0.001)
        self._emit("solar", asdict(self.solar))

    def _parse_inverter(self, d: bytes, src: int):
        """PGN 127517 - Inverter Status"""
        if len(d) < 6: return
        self.inverter.instance  = d[0]
        state_raw               = d_bits(d[1], 0, 4)
        self.inverter.state     = self.INVERTER_STATES[state_raw] if state_raw < len(self.INVERTER_STATES) else "Unknown"
        self.inverter.ac_voltage= d_u16(d, 2, 0.01)
        self.inverter.ac_current= d_i16(d, 4, 0.1)
        if self.inverter.ac_voltage and self.inverter.ac_current:
            self.inverter.ac_power = round(self.inverter.ac_voltage * self.inverter.ac_current, 0)
        self.inverter.ts        = time.time()
        self._emit("inverter", asdict(self.inverter))

    def _parse_inverter_ac(self, d: bytes, src: int):
        """PGN 127519 - Inverter AC Status"""
        if len(d) < 8: return
        self.inverter.ac_voltage   = d_u16(d, 0, 0.01)
        self.inverter.ac_current   = d_i16(d, 2, 0.1)
        self.inverter.ac_frequency = d_u16(d, 4, 0.01)
        self._emit("inverter", asdict(self.inverter))

    def _parse_charger(self, d: bytes, src: int):
        """PGN 127516 - Charger Status"""
        if len(d) < 10: return
        self.charger.instance       = d[0]
        mode_raw                    = d_bits(d[1], 0, 4)
        self.charger.mode           = self.CHARGER_MODES[mode_raw] if mode_raw < len(self.CHARGER_MODES) else "Unknown"
        self.charger.output_voltage = d_u16(d, 2, 0.01)
        self.charger.output_current = d_i16(d, 4, 0.1)
        self.charger.input_voltage  = d_u16(d, 6, 0.01)
        self.charger.input_current  = d_i16(d, 8, 0.1)
        self.charger.ts             = time.time()
        self._emit("charger", asdict(self.charger))

    def _parse_charger_ac(self, d: bytes, src: int):
        """PGN 127518 - Charger AC Status"""
        if len(d) < 8: return
        self.charger.input_voltage   = d_u16(d, 0, 0.01)
        self.charger.input_current   = d_i16(d, 2, 0.1)
        self.charger.input_frequency = d_u16(d, 4, 0.01)
        self._emit("charger", asdict(self.charger))

    def _parse_ac_input(self, d: bytes, src: int):
        if len(d) < 8: return
        self.inverter.dc_input_v   = d_u16(d, 2, 0.01)
        self.inverter.ac_frequency = d_u16(d, 6, 0.01)

    def _parse_ac_output(self, d: bytes, src: int):
        if len(d) < 8: return
        self.inverter.ac_voltage   = d_u16(d, 0, 0.01)
        self.inverter.ac_frequency = d_u16(d, 6, 0.01)
        self._emit("inverter", asdict(self.inverter))

    def _parse_temperature(self, d: bytes, src: int):
        if len(d) < 6: return
        source_type = d[1]  # 0=sea, 1=outside, 2=inside, 4=engine, 8=battery
        temp_k = d_u16(d, 2, 0.01)
        if temp_k and source_type == 8:  # Battery temperature
            self.battery.temperature = round(temp_k - 273.15, 1)

    def _parse_xb_battery_ext(self, d: bytes, src: int):
        """Schneider proprietary: Extended Battery Data"""
        if len(d) < 12: return
        self.battery.soc         = d_u16(d, 0, 0.01)
        self.battery.capacity_ah = d_u16(d, 2, 0.1)
        state_raw                = d[8] & 0x0F
        self.battery.state       = self.BATTERY_STATES[state_raw] if state_raw < len(self.BATTERY_STATES) else "Unknown"
        self._emit("battery", asdict(self.battery))

    def _parse_xb_fault(self, d: bytes, src: int):
        """Schneider proprietary: Fault/Warning"""
        if len(d) < 4: return
        fault_code = struct.unpack_from("<H", d, 0)[0]
        severity   = d[2] & 0x03  # 0=info, 1=warning, 2=fault, 3=critical
        sev_names  = ["Info", "Warning", "Fault", "Critical"]
        self._emit("fault", {
            "source": src, "code": fault_code,
            "severity": sev_names[severity],
            "ts": time.time()
        })
        log.warning(f"XanBus {sev_names[severity]} from 0x{src:02X}: code 0x{fault_code:04X}")

    def _parse_xb_system_mode(self, d: bytes, src: int):
        """Schneider proprietary: System Mode"""
        if len(d) < 2: return
        mode = d[0]
        modes = {0: "Off", 1: "Sell", 2: "Buy", 3: "Grid-Tie", 4: "Island", 5: "AC-Coupled"}
        self._emit("system_mode", {"mode": modes.get(mode, f"Mode_{mode}"), "ts": time.time()})

    def _parse_product_info(self, d: bytes, src: int):
        """PGN 126996 - Product Information"""
        if len(d) < 14: return
        # Bytes 2-33: manufacturer name (fixed 32-char)
        # Bytes 34-65: model (fixed 32-char)
        model = d[34:66].decode("ascii", errors="ignore").rstrip("\x00").strip()
        sw    = d[66:74].decode("ascii", errors="ignore").rstrip("\x00").strip() if len(d) > 74 else ""
        self._emit("device_info", {"source": src, "model": model, "sw": sw, "ts": time.time()})
        log.info(f"Device 0x{src:02X}: model='{model}' sw='{sw}'")


# ─── CAN OUTPUT ───────────────────────────────────────────────────────────────
class CANOutput:
    def __init__(self):
        self.bus: Optional[can.BusABC] = None

    def connect(self, interface: str, channel: str, bitrate: int):
        if not CAN_AVAILABLE:
            log.warning("CAN: python-can not available")
            return
        try:
            self.bus = can.interface.Bus(channel=channel, bustype=interface,
                                          bitrate=bitrate)
            log.info(f"CAN: connected {interface}/{channel} @ {bitrate}")
        except Exception as e:
            log.error(f"CAN connect failed: {e}")

    def send(self, can_id: int, data: bytes, extended: bool = False):
        if not self.bus: return
        try:
            msg = can.Message(arbitration_id=can_id, data=data,
                               is_extended_id=extended)
            self.bus.send(msg)
        except Exception as e:
            log.debug(f"CAN send error: {e}")

    def send_battery(self, b: BatteryData):
        if b.voltage is None: return
        v = int((b.voltage or 0) * 100)
        i = int((b.current or 0) * 10)
        s = int((b.soc     or 0) * 10)
        t = int(((b.temperature or 0) + 273.15) * 100)
        self.send(CAN_ID["battery_pack"], struct.pack("<hhhH", v, i, t, s))

    def send_solar(self, s: SolarData):
        if s.pv_voltage is None: return
        pv_v = int((s.pv_voltage     or 0) * 100)
        pv_i = int((s.pv_current     or 0) * 100)
        pv_p = int(s.pv_power        or 0)
        ov   = int((s.output_voltage or 0) * 100)
        self.send(CAN_ID["solar_basic"], struct.pack("<hhHH", pv_v, pv_i, pv_p, ov))

    def send_inverter(self, inv: InverterData):
        if inv.ac_voltage is None: return
        av = int((inv.ac_voltage   or 0) * 10)
        ai = int((inv.ac_current   or 0) * 10)
        af = int((inv.ac_frequency or 0) * 100)
        ap = int(inv.ac_power      or 0)
        self.send(CAN_ID["inverter_ac"], struct.pack("<hhhH", av, ai, af, ap))

    def send_charger(self, c: ChargerData):
        if c.output_voltage is None: return
        ov = int((c.output_voltage or 0) * 100)
        oi = int((c.output_current or 0) * 10)
        iv = int((c.input_voltage  or 0) * 10)
        ii = int((c.input_current  or 0) * 10)
        self.send(CAN_ID["charger_dc"], struct.pack("<hhhh", ov, oi, iv, ii))


# ─── MQTT CLIENT ─────────────────────────────────────────────────────────────
class MQTTLogger:
    def __init__(self, broker: str, port: int, user: str = "", pwd: str = ""):
        self.client = mqtt.Client(client_id=f"xanbus_py_{int(time.time())}")
        if user:
            self.client.username_pw_set(user, pwd)
        self.client.on_connect    = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self._connected = False
        try:
            self.client.connect(broker, port, keepalive=30)
            self.client.loop_start()
        except Exception as e:
            log.error(f"MQTT connect failed: {e}")

    def _on_connect(self, c, ud, flags, rc):
        self._connected = rc == 0
        if self._connected:
            log.info("MQTT: connected")
            self.publish("xanbus/status", {"event": "online", "ts": time.time()})
        else:
            log.warning(f"MQTT: connect rc={rc}")

    def _on_disconnect(self, c, ud, rc):
        self._connected = False
        log.warning(f"MQTT: disconnected rc={rc}")

    def publish(self, topic: str, payload: Any, qos: int = 1, retain: bool = False):
        if not self._connected: return
        if isinstance(payload, dict):
            payload = json.dumps(payload, default=lambda x: None if x is None else x)
        self.client.publish(topic, payload, qos=qos, retain=retain)


# ─── DATA LOGGER (SQLite + CSV) ───────────────────────────────────────────────
class DataLogger:
    def __init__(self, db_file: Path, log_dir: Path):
        log_dir.mkdir(parents=True, exist_ok=True)
        self.db   = sqlite3.connect(str(db_file), check_same_thread=False)
        self.lock = threading.Lock()
        self._init_db()
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = open(log_dir / f"xanbus_{ts}.csv", "w", newline="")
        self.csv      = csv.writer(self.csv_file)
        self.csv.writerow([
            "timestamp", "type",
            "bat_v","bat_a","bat_soc","bat_temp",
            "sol_pv_v","sol_pv_i","sol_pv_w","sol_state",
            "inv_ac_v","inv_ac_a","inv_ac_hz","inv_state",
            "chg_out_v","chg_out_a","chg_in_v","chg_mode"
        ])

    def _init_db(self):
        c = self.db.cursor()
        c.executescript("""
            CREATE TABLE IF NOT EXISTS battery (
                ts REAL, voltage REAL, current REAL, soc REAL,
                temperature REAL, capacity_ah REAL, state TEXT, instance INT
            );
            CREATE TABLE IF NOT EXISTS solar (
                ts REAL, pv_voltage REAL, pv_current REAL, pv_power REAL,
                output_voltage REAL, output_current REAL,
                daily_yield_wh REAL, total_yield_kwh REAL, state TEXT, instance INT
            );
            CREATE TABLE IF NOT EXISTS inverter (
                ts REAL, ac_voltage REAL, ac_current REAL, ac_frequency REAL,
                ac_power REAL, dc_input_v REAL, state TEXT, instance INT
            );
            CREATE TABLE IF NOT EXISTS charger (
                ts REAL, output_voltage REAL, output_current REAL,
                input_voltage REAL, input_current REAL, mode TEXT, instance INT
            );
            CREATE TABLE IF NOT EXISTS faults (
                ts REAL, source INT, code INT, severity TEXT
            );
        """)
        self.db.commit()

    def log_all(self, b: BatteryData, s: SolarData, inv: InverterData, c: ChargerData):
        with self.lock:
            cur = self.db.cursor()
            ts  = time.time()
            cur.execute("INSERT INTO battery VALUES (?,?,?,?,?,?,?,?)",
                (ts, b.voltage, b.current, b.soc, b.temperature, b.capacity_ah, b.state, b.instance))
            cur.execute("INSERT INTO solar VALUES (?,?,?,?,?,?,?,?,?,?)",
                (ts, s.pv_voltage, s.pv_current, s.pv_power,
                 s.output_voltage, s.output_current,
                 s.daily_yield_wh, s.total_yield_kwh, s.controller_state, s.instance))
            cur.execute("INSERT INTO inverter VALUES (?,?,?,?,?,?,?,?)",
                (ts, inv.ac_voltage, inv.ac_current, inv.ac_frequency,
                 inv.ac_power, inv.dc_input_v, inv.state, inv.instance))
            cur.execute("INSERT INTO charger VALUES (?,?,?,?,?,?,?)",
                (ts, c.output_voltage, c.output_current,
                 c.input_voltage, c.input_current, c.mode, c.instance))
            self.db.commit()
            self.csv.writerow([
                datetime.fromtimestamp(ts).isoformat(), "periodic",
                b.voltage, b.current, b.soc, b.temperature,
                s.pv_voltage, s.pv_current, s.pv_power, s.controller_state,
                inv.ac_voltage, inv.ac_current, inv.ac_frequency, inv.state,
                c.output_voltage, c.output_current, c.input_voltage, c.mode
            ])
            self.csv_file.flush()

    def get_history(self, table: str, minutes: int = 60) -> List[dict]:
        with self.lock:
            since = time.time() - minutes * 60
            cur   = self.db.cursor()
            cur.execute(f"SELECT * FROM {table} WHERE ts > ? ORDER BY ts DESC LIMIT 1000", (since,))
            cols = [d[0] for d in cur.description]
            return [dict(zip(cols, row)) for row in cur.fetchall()]


# ─── FASTAPI WEB SERVER ───────────────────────────────────────────────────────
app          = FastAPI(title="XanBus Monitor", version="1.0")
_xb_decoder  = None
_data_logger = None
_ws_clients  = set()

def set_globals(decoder: XanBusDecoder, logger: DataLogger):
    global _xb_decoder, _data_logger
    _xb_decoder  = decoder
    _data_logger = logger

@app.get("/api/status")
def api_status():
    if not _xb_decoder: return {"error": "not initialized"}
    return {
        "battery":  asdict(_xb_decoder.battery),
        "solar":    asdict(_xb_decoder.solar),
        "inverter": asdict(_xb_decoder.inverter),
        "charger":  asdict(_xb_decoder.charger),
        "stats":    _xb_decoder.stats,
    }

@app.get("/api/history/{table}")
def api_history(table: str, minutes: int = 60):
    if table not in ("battery", "solar", "inverter", "charger"):
        return {"error": "invalid table"}
    if not _data_logger: return []
    return _data_logger.get_history(table, minutes)

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    _ws_clients.add(ws)
    try:
        while True:
            await asyncio.sleep(1)
    except WebSocketDisconnect:
        _ws_clients.discard(ws)

async def broadcast_ws(data: dict):
    dead = set()
    for ws in _ws_clients:
        try:
            await ws.send_json(data)
        except:
            dead.add(ws)
    _ws_clients -= dead

@app.get("/", response_class=HTMLResponse)
def serve_gui():
    return HTMLResponse(content=open("web/index.html").read())


# ─── MAIN APPLICATION ─────────────────────────────────────────────────────────
class XanBusBridge:
    def __init__(self, cfg: Config = Config()):
        self.cfg     = cfg
        self.decoder = XanBusDecoder()
        self.can_out = CANOutput()
        self.mqtt    = None
        self.logger  = None
        self.serial  = None
        self._running= False

        # Wire up events
        self.decoder.on("battery",  self._on_battery)
        self.decoder.on("solar",    self._on_solar)
        self.decoder.on("inverter", self._on_inverter)
        self.decoder.on("charger",  self._on_charger)
        self.decoder.on("fault",    self._on_fault)

    def start(self):
        self._running = True
        self.cfg.LOG_DIR.mkdir(exist_ok=True)
        self.logger = DataLogger(self.cfg.DB_FILE, self.cfg.LOG_DIR)
        set_globals(self.decoder, self.logger)

        # CAN
        self.can_out.connect(self.cfg.CAN_INTERFACE, self.cfg.CAN_CHANNEL, self.cfg.CAN_BITRATE)

        # MQTT
        self.mqtt = MQTTLogger(self.cfg.MQTT_BROKER, self.cfg.MQTT_PORT,
                                self.cfg.MQTT_USER,   self.cfg.MQTT_PASS)

        # Serial
        try:
            self.serial = serial.Serial(
                self.cfg.RS485_PORT, self.cfg.RS485_BAUDRATE,
                timeout=self.cfg.RS485_TIMEOUT
            )
            log.info(f"RS485: opened {self.cfg.RS485_PORT} @ {self.cfg.RS485_BAUDRATE}")
        except serial.SerialException as e:
            log.error(f"RS485 open failed: {e}. Running in demo mode.")
            self._start_demo_mode()
            return

        # Periodic logger thread
        threading.Thread(target=self._periodic_log, daemon=True).start()

        # Read loop
        threading.Thread(target=self._read_loop, daemon=True).start()

        log.info("XanBus Bridge started. Web GUI: http://localhost:8080")

    def _read_loop(self):
        while self._running:
            try:
                raw = self.serial.read(256)
                if raw:
                    self.decoder.feed(raw)
            except serial.SerialException as e:
                log.error(f"Serial error: {e}")
                time.sleep(1)

    def _periodic_log(self):
        while self._running:
            time.sleep(5)
            try:
                self.logger.log_all(self.decoder.battery, self.decoder.solar,
                                    self.decoder.inverter, self.decoder.charger)
            except Exception as e:
                log.error(f"DB log error: {e}")

    def _on_battery(self, data: dict):
        self.can_out.send_battery(self.decoder.battery)
        if self.mqtt:
            self.mqtt.publish("xanbus/battery", data)

    def _on_solar(self, data: dict):
        self.can_out.send_solar(self.decoder.solar)
        if self.mqtt:
            self.mqtt.publish("xanbus/solar", data)

    def _on_inverter(self, data: dict):
        self.can_out.send_inverter(self.decoder.inverter)
        if self.mqtt:
            self.mqtt.publish("xanbus/inverter", data)

    def _on_charger(self, data: dict):
        self.can_out.send_charger(self.decoder.charger)
        if self.mqtt:
            self.mqtt.publish("xanbus/charger", data)

    def _on_fault(self, data: dict):
        if self.mqtt:
            self.mqtt.publish("xanbus/fault", data, qos=2, retain=True)

    def _start_demo_mode(self):
        """Generate synthetic XanBus data for testing without hardware."""
        log.info("DEMO MODE: generating synthetic XanBus data")
        set_globals(self.decoder, self.logger)

        def demo_loop():
            t = 0
            while self._running:
                t += 0.1
                # Simulate realistic Schneider XW+ system
                b = self.decoder.battery
                s = self.decoder.solar
                inv = self.decoder.inverter
                c = self.decoder.charger

                b.voltage     = 48.0 + 4.0 * math.sin(t / 60)
                b.current     = 15.0 * math.sin(t / 30) - 5.0
                b.soc         = 60.0 + 30.0 * math.sin(t / 120)
                b.temperature = 25.0 + 5.0 * math.sin(t / 200)
                b.capacity_ah = 200.0
                b.state       = "Charging" if b.current > 0 else "Discharging"
                b.ts          = time.time()

                pv_cycle = max(0, math.sin((t % 86400) / 86400 * math.pi))
                s.pv_voltage     = 120.0 + 30.0 * pv_cycle
                s.pv_current     = 20.0 * pv_cycle
                s.pv_power       = round(s.pv_voltage * s.pv_current, 0)
                s.output_voltage = b.voltage
                s.output_current = s.pv_current * 0.95
                s.controller_state = "MPPT" if pv_cycle > 0.1 else "Off"
                s.daily_yield_wh = s.pv_power * 6.0
                s.ts             = time.time()

                inv.ac_voltage   = 230.0 + 2.0 * math.sin(t * 2)
                inv.ac_current   = 5.0 + 2.0 * abs(math.sin(t / 20))
                inv.ac_frequency = 50.0 + 0.05 * math.sin(t)
                inv.ac_power     = round(inv.ac_voltage * inv.ac_current, 0)
                inv.dc_input_v   = b.voltage
                inv.state        = "Inverting"
                inv.ts           = time.time()

                c.output_voltage = b.voltage
                c.output_current = max(0, 10.0 * math.sin(t / 40))
                c.input_voltage  = 230.0
                c.input_current  = c.output_current * b.voltage / 230.0
                c.mode           = "Bulk" if b.soc < 80 else "Float"
                c.ts             = time.time()

                self._on_battery(asdict(b))
                self._on_solar(asdict(s))
                self._on_inverter(asdict(inv))
                self._on_charger(asdict(c))

                time.sleep(1)

        threading.Thread(target=demo_loop, daemon=True).start()
        threading.Thread(target=self._periodic_log, daemon=True).start()


# ─── ENTRY POINT ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="XanBus to CAN Bridge")
    parser.add_argument("--port",    default=Config.RS485_PORT,    help="RS485 serial port")
    parser.add_argument("--baud",    default=Config.RS485_BAUDRATE, type=int)
    parser.add_argument("--mqtt",    default=Config.MQTT_BROKER,    help="MQTT broker")
    parser.add_argument("--can",     default=Config.CAN_CHANNEL,    help="CAN channel")
    parser.add_argument("--demo",    action="store_true",            help="Demo mode (no hardware)")
    args = parser.parse_args()

    Config.RS485_PORT   = args.port
    Config.RS485_BAUDRATE = args.baud
    Config.MQTT_BROKER  = args.mqtt
    Config.CAN_CHANNEL  = args.can

    bridge = XanBusBridge()
    if args.demo:
        bridge._start_demo_mode()
    else:
        bridge.start()

    uvicorn.run(app, host=Config.API_HOST, port=Config.API_PORT, log_level="warning")
