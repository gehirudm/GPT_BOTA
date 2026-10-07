#!/usr/bin/env python3
"""
Diagnostic tool to calibrate and verify the EV3 Color Sensor for line and edge detection.
Compatible with Python 3.5 (EV3 Stretch image).
"""
import sys
import time
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import ColorSensor

print("=" * 60)
print("     EV3 Color Sensor & Edge Detection Calibration Tool     ")
print("=" * 60)

# Try connecting to Color Sensor on Port 1 first, then fallback to other ports
color_sensor = None
connected_port = None

for port_name, port_obj in [("INPUT_1", INPUT_1), ("INPUT_2", INPUT_2), ("INPUT_3", INPUT_3), ("INPUT_4", INPUT_4)]:
    try:
        sensor = ColorSensor(port_obj)
        color_sensor = sensor
        connected_port = port_name
        print("\n[SUCCESS] ColorSensor detected on port {}!".format(port_name))
        break
    except Exception:
        pass

if not color_sensor:
    print("\n[ERROR] No ColorSensor detected on any input port (1-4).")
    print("Please check your RJ12 cable connections and try again.")
    sys.exit(1)

def categorize_state(refl):
    """
    Categorizes the reflected light intensity into edge-following states:
    - Reflection < 8: Outside Line (Black Background)
    - 8 <= Reflection <= 25: On Edge of White Line (Target Zone)
    - Reflection > 25: Inside White Line
    """
    if refl < 8:
        return "Outside (Black Mat)"
    elif refl <= 25:
        return "ON EDGE (Target Zone)"
    else:
        return "Inside (White Tape)"

# Set sensor to measure reflected light intensity (0-100)
color_sensor.mode = 'COL-REFLECT'

print("\nStarting live readings...")
print("Physically move your robot across: Black Mat -> Line Edge -> White Tape")
print("Press Ctrl+C to stop.\n")
print("{:<15} {:<30}".format("Reflected Light %", "Categorized State"))
print("-" * 50)

try:
    while True:
        refl = color_sensor.reflected_light_intensity
        state_label = categorize_state(refl)
        
        print("{:<15} {:<30}".format(refl, state_label))
        time.sleep(0.2)

except KeyboardInterrupt:
    print("\n" + "=" * 60)
    print("Calibration stopped by user.")
    print("=" * 60)
