import time
import pygame
import serial

# -------------------------
# CONFIG
# -------------------------
SERIAL_PORT = "/dev/ttyUSB0"   # your Arduino/ESP32 serial port
BAUD_RATE = 115200
DEADZONE = 0.18
MAX_SPEED = 4000               # steps/sec, match Arduino MAX_ABS_SPEED

# -------------------------
# INIT SERIAL
# -------------------------
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
except Exception as e:
    raise SystemExit(f"❌ Cannot open serial port {SERIAL_PORT}: {e}")

# -------------------------
# INIT CONTROLLER
# -------------------------
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    raise SystemExit("❌ No controller detected. Pair it first.")

js = pygame.joystick.Joystick(0)
js.init()
print(f"🎮 Controller detected: {js.get_name()}")

# -------------------------
# HELPER FUNCTIONS
# -------------------------
def dz(value, dead=DEADZONE):
    """Apply deadzone to joystick input"""
    if abs(value) < dead:
        return 0.0
    return (value - dead if value > 0 else value + dead) / (1 - dead)

def shaped(value):
    """Optional cubic shaping for smoother control"""
    return value * value * value

# -------------------------
# MAIN LOOP
# -------------------------
try:
    while True:
        pygame.event.pump()

        # Xbox mapping: left stick X = axis 0 (pan), right stick Y = axis 4 (tilt)
        pan = dz(js.get_axis(0))
        tilt = dz(js.get_axis(1))

        pan_s = int(shaped(pan) * MAX_SPEED)
        tilt_s = int(shaped(-tilt) * MAX_SPEED)  # invert so up = tilt up

        # -------------------------
        # DEBUG PRINT
        # -------------------------
        print(f"Pan: {pan_s:5d}, Tilt: {tilt_s:5d}")

        # -------------------------
        # SEND TO ARDUINO
        # -------------------------
        ser.write(f"SPD {pan_s} {tilt_s}\n".encode())

        time.sleep(0.05)

except KeyboardInterrupt:
    # Stop motors when exiting
    ser.write("SPD 0 0\n".encode())
    print("\n🛑 Script stopped, motors set to 0.")
