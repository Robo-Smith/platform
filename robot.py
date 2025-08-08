import time
import serial
import threading
from evdev import InputDevice, ecodes, list_devices
from rpi_ws281x import PixelStrip, Color, ws

last_time = time.time()

# Persistent motor output state
motor_outputs = {
    'FR': 0.0,
    'RR': 0.0,
    'RL': 0.0,
    'FL': 0.0
}

# === LED Setup ===
LED_COUNT = 32
LED_PIN = 12  # GPIO18 (physical pin 12)
LED_FREQ_HZ = 800000
LED_DMA = 10
LED_BRIGHTNESS = 100
LED_INVERT = False
LED_CHANNEL = 0

# === Motor Control ===
arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# === LED Quadrants ===
FRONT_RIGHT = range(0, 8)
REAR_RIGHT = range(8, 16)
REAR_LEFT = range(16, 24)
FRONT_LEFT = range(24, 32)

leds_enabled = True
pulse_enabled = False
pulse_state = 0
pulse_up = True

# === Setup Strip ===
strip = PixelStrip(
    LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA,
    LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL,
    strip_type=ws.WS2811_STRIP_GRB
)
strip.begin()


def set_all_leds(color):
    for i in range(LED_COUNT):
        strip.setPixelColor(i, color)
    strip.show()


def clear_leds():
    set_all_leds(Color(0, 0, 0))


def pulse_green():
    global pulse_state, pulse_up
    step = 5
    if pulse_up:
        pulse_state += step
        if pulse_state >= 100:
            pulse_state = 100
            pulse_up = False
    else:
        pulse_state -= step
        if pulse_state <= 0:
            pulse_state = 0
            pulse_up = True
    green = int(pulse_state * 255 / 100)
    set_all_leds(Color(green, green, green))


def update_direction_leds(motor_vals):
    if not leds_enabled or pulse_enabled:
        return

    quadrant_colors = {
        'FR': Color(0, 255, 0) if motor_vals['FR'] > 0 else Color(255, 0, 0),
        'RR': Color(0, 255, 0) if motor_vals['RR'] > 0 else Color(255, 0, 0),
        'RL': Color(0, 255, 0) if motor_vals['RL'] > 0 else Color(255, 0, 0),
        'FL': Color(0, 255, 0) if motor_vals['FL'] > 0 else Color(255, 0, 0),
    }

    for i in FRONT_RIGHT:
        strip.setPixelColor(i, quadrant_colors['FR'])
    for i in REAR_RIGHT:
        strip.setPixelColor(i, quadrant_colors['RR'])
    for i in REAR_LEFT:
        strip.setPixelColor(i, quadrant_colors['RL'])
    for i in FRONT_LEFT:
        strip.setPixelColor(i, quadrant_colors['FL'])

    strip.show()


def scale(val, deadzone=0.05):
    if abs(val) < deadzone:
        return 0
    return int(val * 255)

def curve(val, exp=2.0):
    return (abs(val) ** exp) * (1 if val >= 0 else -1)


def normalize(val):
    return max(min((val - 128) / 127.0, 1.0), -1.0)



def find_gamepad():
    for path in list_devices():
        dev = InputDevice(path)
        if '8BitDo' in dev.name:
            print(f"Found gamepad at {path} ({dev.name})")
            return InputDevice(path)
    raise RuntimeError("8BitDo controller not found")


def gamepad_loop():
    global leds_enabled, pulse_enabled

    axis_states = {
        ecodes.ABS_X: 128,    # Left stick X (strafe)
        ecodes.ABS_Y: 128,    # Left stick Y (forward/back)
        ecodes.ABS_Z: 128     # Right stick X (rotation)
    }

    gamepad = find_gamepad()
    gamepad.grab()

    for event in gamepad.read_loop():
        if event.type == ecodes.EV_ABS and event.code in axis_states:
            axis_states[event.code] = event.value

        elif event.type == ecodes.EV_KEY:
            if event.code == ecodes.BTN_SOUTH and event.value:  # A button
                pulse_enabled = True
            elif event.code == ecodes.BTN_WEST and event.value:  # X button
                leds_enabled = False
                pulse_enabled = False
                clear_leds()
            elif event.code == ecodes.BTN_NORTH and event.value:  # Y button
                leds_enabled = True
                pulse_enabled = False

        # Normalize stick inputs
        ly = normalize(axis_states[ecodes.ABS_Y])    # Forward/back  (used to be strafe)
        lx = normalize(axis_states[ecodes.ABS_X])    # Strafe left/right (was forward)
        rx = normalize(axis_states[ecodes.ABS_Z])   # Rotation

        ly = curve(normalize(axis_states[ecodes.ABS_Y]))  # Forward/back
        lx = curve(normalize(axis_states[ecodes.ABS_X]))  # Strafe
        rx = curve(normalize(axis_states[ecodes.ABS_Z]))  # Rotation

        # === Compute raw motor output ===
        target_fr = ly + rx - lx
        target_rr = ly + rx + lx
        target_rl = -ly + rx + lx
        target_fl = -ly + rx - lx

        # === Normalize ===
        max_val = max(abs(target_fr), abs(target_rr), abs(target_rl), abs(target_fl), 1.0)
        target_fr /= max_val
        target_rr /= max_val
        target_rl /= max_val
        target_fl /= max_val


        def smooth_limit(name, prev, target, alpha=0.3, max_delta=0.1, zero_cutoff=0.1): #FILTERED INPUT
            # Smooth
            smoothed = (1 - alpha) * prev + alpha * target

            # Limit acceleration
            delta = smoothed - prev
            if abs(delta) > max_delta:
                smoothed = prev + max_delta * (1 if delta > 0 else -1)

            # Snap to zero if close enough
            if abs(smoothed) < zero_cutoff:
                smoothed = 0.0

            return smoothed


        motor_outputs['FR'] = smooth_limit('FR', motor_outputs['FR'], target_fr)
        motor_outputs['RR'] = smooth_limit('RR', motor_outputs['RR'], target_rr)
        motor_outputs['RL'] = smooth_limit('RL', motor_outputs['RL'], target_rl)
        motor_outputs['FL'] = smooth_limit('FL', motor_outputs['FL'], target_fl)

        # === Scale to [-255, 255] and send ===
        fr = scale(motor_outputs['FR'])
        rr = scale(motor_outputs['RR'])
        rl = scale(motor_outputs['RL'])
        fl = scale(motor_outputs['FL'])

        motor_vals = {'FR': fr, 'RR': rr, 'RL': rl, 'FL': fl}
        msg = f"FR:{fr},RR:{rr},RL:{rl},FL:{fl}\n"
        arduino.write(msg.encode())


        if leds_enabled:
            if pulse_enabled:
                pulse_green()
            else:
                update_direction_leds(motor_vals)

        time.sleep(0.02)


if __name__ == '__main__':
    print("Starting robot control...")
    threading.Thread(target=gamepad_loop, daemon=True).start()
    while True:
        time.sleep(1)
