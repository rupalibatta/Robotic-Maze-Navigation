#!/usr/bin/env python3

import pigpio
import traceback
from drivesystem import DriveSystem
from linesensor import LineSensor
from anglesensor import ADC, AngleSensor
from behaviors import Behaviors
from brain import brain
from map import Map

def main():
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connect to GPIO!")
        return

    try:
        adc = ADC(io)
        anglesensor = AngleSensor(adc)
        drive = DriveSystem(io)
        sensor = LineSensor(io)
        anglesensor.calibrate_spin(drive, sensor)

        behaviors = Behaviors(drive, sensor)

        map = Map()

        print("Robot initialized. Ready to run.")
        brain(behaviors, map, anglesensor)

    except BaseException as ex:
        print("Ending due to exception:", repr(ex))
        traceback.print_exc()

    finally:
        print("Shutting down robot.")
        drive.stop()
        io.stop()

if __name__ == "__main__":
    main()
