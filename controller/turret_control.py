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

B_BUTTON = 1
A_BUTTON = 0
a_last = 0
led_red = False
LT_AXIS = 5
RT_AXIS = 4
laser_on = False
water_on = False

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
        lt_value = js.get_axis(LT_AXIS)
        # Convert from [-1, 1] → [0, 1]
        lt_normalized = (lt_value + 1) / 2
        trigger_pressed = lt_normalized > 0.2
        
        rt_value = js.get_axis(RT_AXIS)
        # Convert from [-1, 1] → [0, 1]
        rt_normalized = (rt_value + 1) / 2
        trigger_pressed_r = rt_normalized > 0.2

        pan_s = int(shaped(pan) * MAX_SPEED)
        tilt_s = int(shaped(-tilt) * MAX_SPEED)  # invert so up = tilt up

        b_now = js.get_button(B_BUTTON)
        if b_now == 1:
            ser.write("LED_OFF\n".encode())
            print("LED OFF")

        a_now = js.get_button(A_BUTTON)

        if a_now == 1 and a_last == 0:
            led_red = not led_red

            if led_red:
                ser.write("LED_RED\n".encode())
                print("LED RED")
            else:
                ser.write("LED_GREEN\n".encode())
                print("LED GREEN")

        a_last = a_now
        # -------------------------
        # DEBUG PRINT
        # -------------------------
        print(f"Pan: {pan_s:5d}, Tilt: {tilt_s:5d}")
        print(f"LT raw: {lt_value:.2f}, normalized: {lt_normalized:.2f}")
        print(f"RT raw: {rt_value:.2f}, normalized: {rt_normalized:.2f}")
        # -------------------------
        # SEND TO ARDUINO
        # -------------------------
        ser.write(f"SPD {pan_s} {tilt_s}\n".encode())
        if trigger_pressed and not laser_on:
            ser.write(f"L1\n".encode())
            laser_on = True
        elif not trigger_pressed and laser_on:
            ser.write(f"L0\n".encode())
            laser_on = False
        if trigger_pressed_r and not water_on:
            ser.write(f"S1\n".encode())
            water_on = True
        elif not trigger_pressed_r and water_on:
            ser.write(f"S0\n".encode())
            water_on = False

        time.sleep(0.05)

except KeyboardInterrupt:
    # Stop motors when exiting
    ser.write("SPD 0 0\n".encode())
    print("\n🛑 Script stopped, motors set to 0.")
