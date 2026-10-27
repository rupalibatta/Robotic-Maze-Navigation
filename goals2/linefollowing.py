#!/usr/bin/env python3

import pigpio
import traceback
from drivesystem import DriveSystem
from linesensor import LineSensor


FEEDBACK_TABLE = {
    (0, 1, 0): 'STRAIGHT',   
    (0, 1, 1): 'VEER_RIGHT',   
    (0, 0, 1): 'STEER_RIGHT',
    (1, 1, 0): 'VEER_LEFT',
    (1, 0, 0): 'STEER_LEFT', 
    (1, 1, 1): 'STRAIGHT', 
    (1, 0, 1): 'STRAIGHT',
    (0, 0, 0): 'STOP' 
}


def run(drive, sensor):
    """
    Main feedback loop: continually read the line sensors and apply feedback law.
    """
    while True:
        reading = sensor.read()
        action = FEEDBACK_TABLE.get(reading, 'STOP')

        if action == 'STOP':
            print("No valid line detected. Stopping.")
            drive.stop()
        else:
            drive.drive(action)


def main():
    """
    Sets up GPIO, creates robot objects, and starts the line-following loop.
    Includes a try-except block for safe shutdown.
    """
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connect to pigpio daemon.")
        return

    drive = DriveSystem(io)
    sensor = LineSensor(io)

    try:
        run(drive, sensor)
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
    finally:
        drive.stop()
        io.stop()

if __name__ == "__main__":
    main()
