#!/usr/bin/env python3
#
#   motordemo.py
#
#   This shows how to interface with the GPIO (general purpose I/O)
#   pins and how to drive the PWM for the motors.  Please use as an
#   example, but change to suit the goals!
#

# Imports
import pigpio
import sys
import time
import traceback

# Define the motor pins.
PIN_MOTOR1_LEGA = 8
PIN_MOTOR1_LEGB = 7

PIN_MOTOR2_LEGA = 5
PIN_MOTOR2_LEGB = 6


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

    # Set up the four pins as output (commanding the motors).
    io.set_mode(PIN_MOTOR1_LEGA, pigpio.OUTPUT)
    io.set_mode(PIN_MOTOR1_LEGB, pigpio.OUTPUT)
    io.set_mode(PIN_MOTOR2_LEGA, pigpio.OUTPUT)
    io.set_mode(PIN_MOTOR2_LEGB, pigpio.OUTPUT)

    # Prepare the PWM.  The range gives the maximum value for 100%
    # duty cycle, using integer commands (1 up to max).
    io.set_PWM_range(PIN_MOTOR1_LEGA, 255)
    io.set_PWM_range(PIN_MOTOR1_LEGB, 255)
    io.set_PWM_range(PIN_MOTOR2_LEGA, 255)
    io.set_PWM_range(PIN_MOTOR2_LEGB, 255)
    
    # Set the PWM frequency to 1000Hz.
    io.set_PWM_frequency(PIN_MOTOR1_LEGA, 1000)
    io.set_PWM_frequency(PIN_MOTOR1_LEGB, 1000)
    io.set_PWM_frequency(PIN_MOTOR2_LEGA, 1000)
    io.set_PWM_frequency(PIN_MOTOR2_LEGB, 1000)

    # Clear all pins, just in case.
    io.set_PWM_dutycycle(PIN_MOTOR1_LEGA, 0)
    io.set_PWM_dutycycle(PIN_MOTOR1_LEGB, 0)
    io.set_PWM_dutycycle(PIN_MOTOR2_LEGA, 0)
    io.set_PWM_dutycycle(PIN_MOTOR2_LEGB, 0)

    print("Motors ready...")

    
    ############################################################
    # Drive.
    # Place this is a try-except structure, so we can turn off the
    # motors even if the code crashes.
    try:
        # Example 1: Ramp ONE PIN up/down.  Keep the other pin at zero.
        print("Ramping Motor 2 (backward) up/down...") 
        pinNonzero = PIN_MOTOR2_LEGB
        pinZero    = PIN_MOTOR2_LEGA

        for pwmlevel in [50, 100, 150, 200, 255, 200, 150, 100, 50, 0]:
            print("Pin %d at level %3d, Pin %d at zero" %
                  (pinNonzero, pwmlevel, pinZero))
            io.set_PWM_dutycycle(pinNonzero, pwmlevel)
            io.set_PWM_dutycycle(pinZero, 0)
            time.sleep(1)

        # Example 2: Drive ONE motor forward/backward.
        print("Driving Motor 1 forward, stopping, then reversing...")
        io.set_PWM_dutycycle(PIN_MOTOR1_LEGA, 170)
        io.set_PWM_dutycycle(PIN_MOTOR1_LEGB,   0)
        time.sleep(1)

        io.set_PWM_dutycycle(PIN_MOTOR1_LEGA,   0)
        io.set_PWM_dutycycle(PIN_MOTOR1_LEGB,   0)
        time.sleep(1)

        io.set_PWM_dutycycle(PIN_MOTOR1_LEGA,   0)
        io.set_PWM_dutycycle(PIN_MOTOR1_LEGB, 170)
        time.sleep(1)

        io.set_PWM_dutycycle(PIN_MOTOR1_LEGA,   0)
        io.set_PWM_dutycycle(PIN_MOTOR1_LEGB,   0)
        time.sleep(1)

    except BaseException as ex:
        # Report the error, but continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()
        

    ############################################################
    # Turn Off.
    # Note the PWM will stay at the last commanded value.  So you want
    # to be sure to set to zero before the program closes.  Else your
    # robot will run away...
    print("Turning off...")

    # Clear the PINs (commands).
    io.set_PWM_dutycycle(PIN_MOTOR1_LEGA, 0)
    io.set_PWM_dutycycle(PIN_MOTOR1_LEGB, 0)
    io.set_PWM_dutycycle(PIN_MOTOR2_LEGA, 0)
    io.set_PWM_dutycycle(PIN_MOTOR2_LEGB, 0)
    
    # Also stop the interface.
    io.stop()
