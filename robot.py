#!/usr/bin/env python3
import os, sys, time, math, struct
import threading
from typing import Optional

import serial  # pip install pyserial
from evdev import InputDevice, ecodes, list_devices  # pip install evdev

# WS2811 via rpi_ws281x
from rpi_ws281x import PixelStrip, Color  # pip install rpi_ws281x

# =====================
# Config
# =====================
GAMEPAD_NAME_CONTAINS = "8BitDo Lite 2"

# Arduino serial
SERIAL_PORT_CANDIDATES = [
    "/dev/ttyACM0",
    "/dev/ttyACM1",
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
]
SERIAL_BAUD = 115200

# LEDs
LED_COUNT = 32
LED_GPIO = 18   # BCM 18 == physical pin 12
LED_FREQ_HZ = 800000
LED_DMA = 10
LED_BRIGHTNESS = 128   # 0-255
LED_INVERT = False
LED_CHANNEL = 0
LED_STRIP_TYPE = None  # rpi_ws281x internal type; we'll set GRB via Color helper

# Each quarter of the ring corresponds to a wheel (8 LEDs each, starting front-center, clockwise)
LEDS_PER_WHEEL = LED_COUNT // 4
Q_FR = (0, LEDS_PER_WHEEL)                   # Front-Right quarter starts just after front-center heading clockwise
Q_RR = (LEDS_PER_WHEEL, 2 * LEDS_PER_WHEEL)  # Rear-Right
Q_RL = (2 * LEDS_PER_WHEEL, 3 * LEDS_PER_WHEEL)  # Rear-Left
Q_FL = (3 * LEDS_PER_WHEEL, 4 * LEDS_PER_WHEEL)  # Front-Left (wraps to end)

# Motion tuning
DEADZONE = 0.08
MAX_ACCEL_PER_SEC = 1.5  # limit change per second for motor commands
LOOP_HZ = 100.0

# Restricted mode limit
RESTRICTED_TURN_SCALE = 0.75

# Motor command scaling to Arduino: we send 4 float32 values in range [-1.0, +1.0]
PACK_FMT = "<4f"  # fr, rr, rl, fl (little-endian)

# Flip signs per-motor if needed to honor "positive -> clockwise"
MOTOR_SIGN = {
    'fr': +1.0,
    'rr': +1.0,
    'rl': +1.0,
    'fl': +1.0,
}

# =====================
# Helper functions
# =====================

def _color_grb(r, g, b):
    """WS2811 string is GRB order; rpi_ws281x's Color expects RGB, so we swap."""
    return Color(g, r, b)


def ease_out_quint(x):
    return 1 - pow(1 - x, 5)


def apply_deadzone(x):
    return 0.0 if abs(x) < DEADZONE else x


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def norm_0_254_to_neg1_pos1_left_right(v):
    # 0 = left (-1), 254 = right (+1)
    return clamp((v - 127.0) / 127.0, -1.0, 1.0)


def norm_0_254_to_neg1_pos1_forward_reverse(v):
    # 0 = forward (+1), 254 = reverse (-1)
    return clamp((127.0 - v) / 127.0, -1.0, 1.0)


def find_gamepad() -> Optional[InputDevice]:
    while True:
        try:
            for path in list_devices():
                d = InputDevice(path)
                if GAMEPAD_NAME_CONTAINS in (d.name or ""):
                    print(f"[gamepad] Using {d.name} at {path}")
                    try:
                        d.grab()  # optional
                    except Exception:
                        pass
                    # d.set_nonblocking(True)
                    return d
            print("[gamepad] Not found; retrying in 2s...")
            time.sleep(2)
        except Exception as e:
            print(f"[gamepad] search error: {e}")
            time.sleep(2)


def find_serial() -> Optional[serial.Serial]:
    while True:
        for p in SERIAL_PORT_CANDIDATES:
            try:
                s = serial.Serial(p, SERIAL_BAUD, timeout=0.01)
                print(f"[serial] Connected {p}")
                return s
            except Exception:
                pass
        print("[serial] No Arduino; retrying in 2s...")
        time.sleep(2)


class LEDDriver:
    def __init__(self):
        self.strip = PixelStrip(LED_COUNT, LED_GPIO, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL, LED_STRIP_TYPE)
        self.strip.begin()
        self.enabled = True
        self.mode_restricted = False
        self.t0 = time.time()

    def set_enabled(self, en: bool):
        self.enabled = en
        if not en:
            self.strip.setBrightness(0)
            for i in range(LED_COUNT):
                self.strip.setPixelColor(i, 0)
            self.strip.show()
        else:
            self.strip.setBrightness(LED_BRIGHTNESS)

    def render_normal(self, fr, rr, rl, fl):
        if not self.enabled:
            return
        # Clear
        for i in range(LED_COUNT):
            self.strip.setPixelColor(i, 0)

        # For each wheel quarter, draw a small animated bar near the wheel region with color by sign
        now = time.time()
        phase = (now * 4.0) % LEDS_PER_WHEEL  # simple chase

        def draw_quarter(q, value):
            start, end = q
            mag = abs(value)
            # forward green, reverse red
            if value >= 0:
                base = (0, int(200 * mag), 0)
            else:
                base = (int(200 * mag), 0, 0)
            pos = int((start + int(phase)) % LED_COUNT)
            for k, scale in zip(range(3), (1.0, 0.6, 0.3)):
                idx = (pos + k) % LED_COUNT
                r = int(base[0] * scale)
                g = int(base[1] * scale)
                b = int(base[2] * scale)
                self.strip.setPixelColor(idx, _color_grb(r, g, b))

        draw_quarter(Q_FR, fr)
        draw_quarter(Q_RR, rr)
        draw_quarter(Q_RL, rl)
        draw_quarter(Q_FL, fl)

        self.strip.show()

    def render_restricted_pulse(self):
        if not self.enabled:
            return
        # Soft white pulse
        t = time.time() - self.t0
        pulse = 0.5 + 0.1 * ease_out_quint(0.5 * (1 + math.sin(2 * math.pi * 0.7 * t)))
        val = int(220 * pulse)
        col = _color_grb(val, val, val)
        for i in range(LED_COUNT):
            self.strip.setPixelColor(i, col)
        self.strip.show()


class MotorSmoother:
    def __init__(self):
        self.last = {'fr': 0.0, 'rr': 0.0, 'rl': 0.0, 'fl': 0.0}
        self.last_ts = time.time()

    def step(self, desired):
        now = time.time()
        dt = max(1e-3, now - self.last_ts)
        self.last_ts = now
        max_delta = MAX_ACCEL_PER_SEC * dt
        out = {}
        for k in self.last.keys():
            target = clamp(desired[k], -1.0, 1.0)
            cur = self.last[k]
            delta = clamp(target - cur, -max_delta, max_delta)
            cur += delta
            self.last[k] = cur
            out[k] = cur
        return out


class Controller:
    def __init__(self):
        self.gamepad: Optional[InputDevice] = None
        self.serial: Optional[serial.Serial] = None
        self.leds = LEDDriver()
        self.smoother = MotorSmoother()

        # state
        self.restricted = False
        self.led_enabled = True

        # axes raw (0..254)
        self.raw = {
            'ABS_X': 127,   # left stick left/right (strafe)
            'ABS_Y': 127,   # left stick forward/reverse (drive)
            'ABS_Z': 127,   # right stick left/right (turn)
            'ABS_RZ': 127,  # right stick forward/reverse (unused)
        }

        self.thread = threading.Thread(target=self.loop, daemon=True)

    def start(self):
        self.thread.start()
        while True:
            time.sleep(1)

    def _ensure_links(self):
        if self.gamepad is None:
            self.gamepad = find_gamepad()
        if self.serial is None or not self.serial.is_open:
            self.serial = find_serial()

    def _read_events_nonblocking(self):
        if self.gamepad is None:
            return
        try:
            # Drain all pending events using non-blocking read_one()
            while True:
                e = self.gamepad.read_one()
                if e is None:
                    break
                if e.type == ecodes.EV_ABS:
                    if e.code in (ecodes.ABS_X, ecodes.ABS_Y, ecodes.ABS_Z, ecodes.ABS_RZ):
                        self.raw[ecodes.bytype[ecodes.EV_ABS][e.code]] = int(e.value)
                elif e.type == ecodes.EV_KEY and e.value == 1:  # press
                    code = e.code
                    if code == ecodes.BTN_TL:
                        self.restricted = True
                        self.leds.mode_restricted = True
                    elif code == ecodes.BTN_TR:
                        self.restricted = False
                        self.leds.mode_restricted = False
                    elif code == ecodes.BTN_WEST:   # X
                        self.led_enabled = False
                        self.leds.set_enabled(False)
                    elif code == ecodes.BTN_NORTH:  # Y
                        self.led_enabled = True
                        self.leds.set_enabled(True)

        except OSError:
            # device likely disconnected
            try:
                if self.gamepad is not None:
                    self.gamepad.close()
            except Exception:
                pass
            self.gamepad = None
            print("[gamepad] disconnected")

    def _compute_motors(self):
        # Normalize axes to [-1..1]
        lx = norm_0_254_to_neg1_pos1_left_right(self.raw['ABS_X'])    # strafe left/right
        ly = norm_0_254_to_neg1_pos1_forward_reverse(self.raw['ABS_Y'])  # forward/back
        rx = norm_0_254_to_neg1_pos1_left_right(self.raw['ABS_Z'])    # rotation (right stick left/right)

        # deadzone
        lx, ly, rx = map(apply_deadzone, (lx, ly, rx))

        if self.restricted:
            # Only allow turning, scaled
            lx, ly = 0.0, 0.0
            rx *= RESTRICTED_TURN_SCALE

        # Mecanum mix (FR, RR, RL, FL)
        fr = -ly + rx - lx
        rr = -ly + rx + lx
        rl = ly + rx + lx
        fl = ly + rx - lx

        # Normalize to [-1,1] if any exceeds 1
        maxmag = max(1.0, abs(fr), abs(rr), abs(rl), abs(fl))
        fr /= maxmag
        rr /= maxmag
        rl /= maxmag
        fl /= maxmag

        # Apply motor sign convention
        fr *= MOTOR_SIGN['fr']
        rr *= MOTOR_SIGN['rr']
        rl *= MOTOR_SIGN['rl']
        fl *= MOTOR_SIGN['fl']

        return {'fr': fr, 'rr': rr, 'rl': rl, 'fl': fl}

    def _send_to_arduino(self, m):
        if self.serial is None:
            return
        try:
            pkt = struct.pack(PACK_FMT, m['fr'], m['rr'], m['rl'], m['fl'])
            self.serial.write(pkt)
        except (OSError, serial.SerialException):
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None
            print("[serial] disconnected")

    def loop(self):
        period = 1.0 / LOOP_HZ
        while True:
            self._ensure_links()
            t0 = time.time()

            self._read_events_nonblocking()
            desired = self._compute_motors()
            smooth = self.smoother.step(desired)

            # LEDs
            if self.restricted:
                self.leds.render_restricted_pulse()
            else:
                self.leds.render_normal(smooth['fr'], smooth['rr'], smooth['rl'], smooth['fl'])

            # Serial out
            self._send_to_arduino(smooth)

            # Sleep to maintain cadence
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)


if __name__ == '__main__':
    Controller().start()
