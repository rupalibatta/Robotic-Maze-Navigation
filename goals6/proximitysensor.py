#!/usr/bin/env python3
import pigpio
import sys
import time
import traceback
from time import sleep

# Default GPIO pins for the ultrasound sensors
PIN_TRIGGER_LEFT = 13
PIN_ECHO_LEFT = 16
PIN_TRIGGER_MIDDLE = 19
PIN_ECHO_MIDDLE = 20
PIN_TRIGGER_RIGHT = 26
PIN_ECHO_RIGHT = 21

class Ultrasound:
    """
    Ultrasound(io, pintrig, pinecho)

    Represents a single ultrasound sensor connected to GPIO pins.
    Measures distance based on time-of-flight between trigger and echo.
    """
    def __init__(self, io, pintrig, pinecho):
        self.io = io
        self.pintrig = pintrig
        self.pinecho = pinecho

        # initialize attributes to avoid missing attributes
        self.risetick = 0
        self.distance = float('inf')

        # Set up the two pins as output/input.
        io.set_mode(pintrig, pigpio.OUTPUT)
        io.set_mode(pinecho, pigpio.INPUT)

        # Set up the callbacks for echo rising/falling edges.
        io.callback(pinecho, pigpio.RISING_EDGE, self.rising)
        io.callback(pinecho, pigpio.FALLING_EDGE, self.falling)

    def trigger(self):
        """
        Fire a 10µs trigger pulse to start an ultrasonic measurement.
        """
        self.io.write(self.pintrig, 1)
        # minimum pulse width is guaranteed by Python speed
        self.io.write(self.pintrig, 0)

    def rising(self, pin, level, ticks):
        """
        Callback for rising edge on ECHO: record start tick.
        """
        self.risetick = ticks

    def falling(self, pin, level, ticks):
        """
        Callback for falling edge on ECHO: compute distance.
        """
        deltatick = ticks - self.risetick
        if deltatick < 0:
            deltatick += 2 ** 32
        # speed of sound 343 m/s, ticks in µs
        self.distance = (343 * (deltatick / 1e6)) / 2

    def read(self):
        """
        Return the most recent distance measurement (meters).
        """
        return self.distance

class ProximitySensor:
    """
    ProximitySensor(io)

    Combines three Ultrasound sensors (left, middle, right) and
    provides a unified interface to trigger and read all distances.
    """
    def __init__(self, io):
        self.left   = Ultrasound(io, PIN_TRIGGER_LEFT, PIN_ECHO_LEFT)
        self.middle = Ultrasound(io, PIN_TRIGGER_MIDDLE, PIN_ECHO_MIDDLE)
        self.right  = Ultrasound(io, PIN_TRIGGER_RIGHT, PIN_ECHO_RIGHT)

    def trigger(self):
        """Trigger all three sensors simultaneously."""
        self.left.trigger()
        self.middle.trigger()
        self.right.trigger()

    def read(self):
        """Read distances from left, middle, and right sensors."""
        return (
            self.left.read(),
            self.middle.read(),
            self.right.read()
        )

# Main/test code
def main():
    # Initialize GPIO interface
    print("Setting up GPIO for proximity sensors...")
    io = pigpio.pi()
    if not io.connected:
        print("Failed to connect to pigpio daemon!")
        sys.exit(1)

    sensor = ProximitySensor(io)
    print("Proximity sensors ready.")

    try:
        while True:
            sensor.trigger()
            sleep(0.05)
            distances = sensor.read()
            print("Distances = (%6.3f m, %6.3f m, %6.3f m)" % distances)
    except KeyboardInterrupt:
        print("Proximity test interrupted by user.")
    except Exception as ex:
        print("Error during proximity test:", repr(ex))
        traceback.print_exc()
    finally:
        print("Shutting down proximity sensors.")
        io.stop()

if __name__ == "__main__":
    main()
