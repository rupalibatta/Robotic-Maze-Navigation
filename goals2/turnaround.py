#!/usr/bin/env python3

import pigpio
import traceback
from drivesystem import DriveSystem
from linesensor import LineSensor


FEEDBACK_TABLE = {
    (0, 1, 0): 'STRAIGHT',   
    (0, 1, 1): 'TURN_RIGHT',   
    (0, 0, 1): 'HOOK_RIGHT',
    (1, 1, 0): 'TURN_LEFT',
    (1, 0, 0): 'HOOK_LEFT', 
    (1, 1, 1): 'STRAIGHT', 
    (1, 0, 1): 'STRAIGHT',
    (0, 0, 0): 'SPIN_LEFT' #placeholder
}

def run(drive, sensor):
    """
    Main feedback loop: continually read the line sensors and apply feedback law.
    """
    last_reading = (0, 1, 0)  # default to "centered"

    while True:
        reading = sensor.read()
        
        if reading == (0, 0, 0):
            # off-tape recovery
            if last_reading in [(0, 1, 1), (0, 0, 1)]:
                action = 'SPIN_RIGHT'
            elif last_reading in [(1, 1, 0), (1, 0, 0), (1, 1, 1), (1, 0, 1), (0, 1, 0)]:
                action = 'SPIN_LEFT'
            else:
                action = 'STOP'
        else:
            action = FEEDBACK_TABLE.get(reading, 'STOP')
            last_reading = reading
        
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
