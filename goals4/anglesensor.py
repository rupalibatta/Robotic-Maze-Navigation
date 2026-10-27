#!/usr/bin/env python3
import pigpio
import time
import math

# Magnetometer Pins
LATCH = 27
ADDRESS = 4
READY = 17
DATA_PINS = [9, 10, 11, 12, 22, 23, 24, 25]

class ADC:
    def __init__(self, io):
        self.io = io
        self.io.set_mode(LATCH, pigpio.OUTPUT)
        self.io.set_mode(ADDRESS, pigpio.OUTPUT)
        self.io.set_mode(READY, pigpio.INPUT)
        for pin in DATA_PINS:
            self.io.set_mode(pin, pigpio.INPUT)

    def readaac(self, address):
        self.io.write(LATCH, 0)
        self.io.write(ADDRESS, address)
        self.io.write(LATCH, 1)
        self.io.write(LATCH, 0)
        self.io.write(LATCH, 1)
        while self.io.read(READY) == 0:
            pass
        bits = [self.io.read(pin) for pin in DATA_PINS]
        value = sum([bit << i for i, bit in enumerate(bits)])
        return value

class AngleSensor:
    def __init__(self, adc):
        self.adc = adc
        self.min0 = 135
        self.max0 = 205
        self.min1 = 96
        self.max1 = 200

    def readangle(self):
        value0 = self.adc.readaac(0)
        value1 = self.adc.readaac(1)

        scaled0 = 2 * (value0 - self.min0) / (self.max0 - self.min0) - 1
        scaled1 = 2 * (value1 - self.min1) / (self.max1 - self.min1) - 1

        phi = math.atan2(scaled0, scaled1)
        phi = math.degrees(phi)
        return phi

    def calibrate_spin(self, drive, sensor, spin_style='SPIN_LEFT', min_time=6):
        """
        Spins the robot and updates min/max calibration values.
        """
        print("Starting magnetometer calibration spin...")

        t0 = time.time()
        min0, max0 = 255, 0
        min1, max1 = 255, 0

        drive.drive(spin_style)

        while True:
            tnow = time.time()

            val0 = self.adc.readaac(0)
            val1 = self.adc.readaac(1)

            min0 = min(min0, val0)
            max0 = max(max0, val0)
            min1 = min(min1, val1)
            max1 = max(max1, val1)

            _, m, _ = sensor.read()
            
            if (tnow - t0) > min_time and (m == 1):
                break

            time.sleep(0.01)

        drive.stop()

        print(f"Calibration complete.")
        print(f"Channel 0: min={min0}, max={max0}")
        print(f"Channel 1: min={min1}, max={max1}")

        # Save the new scaling
        self.min0 = min0
        self.max0 = max0
        self.min1 = min1
        self.max1 = max1
