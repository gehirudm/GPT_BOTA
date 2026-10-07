#!/usr/bin/env python3
"""
Diagnostic script to test EV3 Ultrasonic Sensor connectivity and detect connected sensors on all ports.
Compatible with Python 3.5 (EV3 Stretch image).
"""
import glob
import os
import time
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import UltrasonicSensor

print("=" * 50)
print("EV3 Sensor & Ultrasonic Diagnostic Tool")
print("=" * 50)

# 1. Inspect sysfs directly for detected sensors
sensor_paths = glob.glob('/sys/class/lego-sensor/sensor*')
if not sensor_paths:
    print("\n[!] No sensors detected in /sys/class/lego-sensor/")
    print("    Check if cables are securely attached to the EV3 brick.")
else:
    print("\nDetected sensor(s) in sysfs:")
    for path in sorted(sensor_paths):
        try:
            with open(os.path.join(path, 'address'), 'r') as f:
                address = f.read().strip()
            with open(os.path.join(path, 'driver_name'), 'r') as f:
                driver = f.read().strip()
            with open(os.path.join(path, 'mode'), 'r') as f:
                mode = f.read().strip()
            print("  - Port: {:<15} Driver: {:<18} Mode: {}".format(address, driver, mode))
        except Exception as e:
            print("  - {}: Error reading attributes ({})".format(path, e))

print("\n" + "-" * 50)

# 2. Test InfraredSensor on Port 3 specifically
print("Testing InfraredSensor on Port 3 (INPUT_3)...")
try:
    from ev3dev2.sensor.lego import InfraredSensor
    ir = InfraredSensor(INPUT_3)
    print("SUCCESS: InfraredSensor connected on Port 3!")
    print("Reading IR proximity (0-100) (Press Ctrl+C to stop)...\n")
    while True:
        prox = ir.proximity
        print("  IR Proximity: {} %".format(prox))
        time.sleep(0.5)
except KeyboardInterrupt:
    print("\nStopped by user.")
except Exception as err:
    print("FAILED on Port 3: {}".format(err))
    
    # Check other input ports as fallback
    print("\nScanning ports 1, 2, 3 for UltrasonicSensor...")
    found = False
    for port_name, port in [("INPUT_1", INPUT_1), ("INPUT_2", INPUT_2), ("INPUT_3", INPUT_3)]:
        try:
            us = UltrasonicSensor(port)
            print("--> FOUND UltrasonicSensor on {}! Update line 21 in G4_4.py to use {}.".format(port_name, port_name))
            found = True
            break
        except Exception:
            pass
            
    if not found:
        print("\nTroubleshooting checklist:")
        print(" 1. Cable Connection: Ensure the cable is pushed in until it clicks on both Port 4 and the sensor.")
        print(" 2. Port Placement: Verify the sensor is plugged into Input Port 4 (numbered 1-4 on bottom of EV3), NOT Motor Port A/B/C/D.")
        print(" 3. Sensor Type: Confirm it is an EV3 Ultrasonic sensor (looks like eyes) and not an Infrared sensor (InfraredSensor).")
        print(" 4. Bad Cable: Try swapping the RJ12 cable with another working sensor cable.")

