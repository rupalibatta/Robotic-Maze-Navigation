#!/usr/bin/env python3
#
#   irdemo.py
#
#   This shows how to read the IR sensors via the GPIO (general
#   purpose I/O) pins.  Please use as an example, but change to suit
#   the goals!
#

# Imports
import pigpio
import sys
import time
import traceback


# Define the IR pins.
PIN_IR_LEFT   = 14      # Default GPIO Channel for Left   IR Detector
PIN_IR_MIDDLE = 15      # Default GPIO Channel for Middle IR Detector
PIN_IR_RIGHT  = 18      # Default GPIO Channel for Right  IR Detector


#
#   Main
#
if __name__ == "__main__":

    ############################################################
    # Prepare the GPIO interface/connection (to command the motors).
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)
    print("GPIO ready...")

    # Set up the IR pins as input (reading the sensor).
    io.set_mode(PIN_IR_LEFT,   pigpio.INPUT)
    io.set_mode(PIN_IR_MIDDLE, pigpio.INPUT)
    io.set_mode(PIN_IR_RIGHT,  pigpio.INPUT)

    print("Sensors ready...")


    ############################################################
    # Read.
    # Place in a try-except structure, so we can shut down cleanly.
    try:
        # Example: Continually read and report all three pins.
        while True:
            irl = io.read(PIN_IR_LEFT)
            irm = io.read(PIN_IR_MIDDLE)
            irr = io.read(PIN_IR_RIGHT)
        
            print("IRs: L %d  M %d  R %d" % (irl, irm, irr))

    except BaseException as ex:
        # Report the error, then continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()


    ############################################################
    # Turn Off.
    # Nothing to do for the sensors.
    print("Turning off...")

    # Also stop the interface.
    io.stop()
