#!/usr/bin/env python3
import pigpio
import sys
import time
import threading
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
    def __init__(self, io, pintrig, pinecho):
        self.io = io
        self.pintrig = pintrig
        self.pinecho = pinecho
        self.risetick = 0
        self.distance = float('inf')

        io.set_mode(pintrig, pigpio.OUTPUT)
        io.set_mode(pinecho, pigpio.INPUT)
        io.callback(pinecho, pigpio.RISING_EDGE, self.rising)
        io.callback(pinecho, pigpio.FALLING_EDGE, self.falling)

    def trigger(self):
        self.io.write(self.pintrig, 1)
        self.io.write(self.pintrig, 0)

    def rising(self, pin, level, ticks):
        self.risetick = ticks

    def falling(self, pin, level, ticks):
        deltatick = ticks - self.risetick
        if deltatick < 0:
            deltatick += 2 ** 32
        self.distance = (343 * (deltatick / 1e6)) / 2

    def read(self):
        return self.distance


class ProximitySensor:
    def __init__(self, io):
        self.left   = Ultrasound(io, PIN_TRIGGER_LEFT, PIN_ECHO_LEFT)
        self.middle = Ultrasound(io, PIN_TRIGGER_MIDDLE, PIN_ECHO_MIDDLE)
        self.right  = Ultrasound(io, PIN_TRIGGER_RIGHT, PIN_ECHO_RIGHT)

        # Start triggering thread
        print("Starting triggering thread...")
        self.triggering = True
        self.thread = threading.Thread(name="TriggerThread", target=self.run)
        self.thread.start()
        time.sleep(0.1)  # wait for first readings

    def run(self):
        while self.triggering:
            self.trigger()
            time.sleep(0.05)

    def trigger(self):
        self.left.trigger()
        self.middle.trigger()
        self.right.trigger()

    def read(self):
        return (
            self.left.read(),
            self.middle.read(),
            self.right.read()
        )

    def shutdown(self):
        self.triggering = False
        print("Waiting for triggering thread to finish...")
        self.thread.join()
        print("Triggering thread returned.")


# Main/test code
def main():
    print("Setting up GPIO for proximity sensors...")
    io = pigpio.pi()
    if not io.connected:
        print("Failed to connect to pigpio daemon!")
        sys.exit(1)

    sensor = ProximitySensor(io)
    print("Proximity sensors ready.")

    try:
        while True:
            distances = sensor.read()
            print("Distances = (%6.3f m, %6.3f m, %6.3f m)" % distances)
            sleep(0.1)
    except KeyboardInterrupt:
        print("Proximity test interrupted by user.")
    except Exception as ex:
        print("Error during proximity test:", repr(ex))
        traceback.print_exc()
    finally:
        print("Shutting down proximity sensors.")
        sensor.shutdown()
        io.stop()


if __name__ == "__main__":
    main()
