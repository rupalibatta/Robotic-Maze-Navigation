#!/usr/bin/env python3

import pigpio
import sys
import time
import traceback


PIN_MOTOR1_LEGA = 8
PIN_MOTOR1_LEGB = 7
PIN_MOTOR2_LEGA = 5
PIN_MOTOR2_LEGB = 6

class Motor:
    """
    Motor(io, pin_lega, pin_legb)

    Controls a single DC motor using two GPIO pins.
    Maps a float level [-1.0, 1.0] to PWM duty cycles to control speed and direction.
    Positive values spin forward, negative spins backward, zero stops the motor.
    """
    def __init__(self, io, pin_lega, pin_legb):
        self.io = io
        self.pin_lega = pin_lega
        self.pin_legb = pin_legb
        io.set_mode(pin_lega, pigpio.OUTPUT)
        io.set_mode(pin_legb, pigpio.OUTPUT)
        io.set_PWM_range(pin_lega, 255)
        io.set_PWM_range(pin_legb, 255)
        io.set_PWM_frequency(pin_lega, 1000)
        io.set_PWM_frequency(pin_legb, 1000)
        self.stop()

    def setlevel(self, level):
        pwm = int(abs(level) * 255)
        if level > 0:
            self.io.set_PWM_dutycycle(self.pin_lega, pwm)
            self.io.set_PWM_dutycycle(self.pin_legb, 0)
        elif level < 0:
            self.io.set_PWM_dutycycle(self.pin_lega, 0)
            self.io.set_PWM_dutycycle(self.pin_legb, pwm)
        else:
            self.io.set_PWM_dutycycle(self.pin_lega, 0)
            self.io.set_PWM_dutycycle(self.pin_legb, 0)
    
    def stop(self):
        self.setlevel(0)

if __name__ == "__main__":
    try:
        io = pigpio.pi()
        if not io.connected:
            print("Unable to connect to pigpio daemon!")
            sys.exit(0)

        left_motor = Motor(io, PIN_MOTOR1_LEGA, PIN_MOTOR1_LEGB)
        right_motor = Motor(io, PIN_MOTOR2_LEGA, PIN_MOTOR2_LEGB)

        straight_level_left = 0.92
        straight_level_right = 0.95
        one_meter_duration = 2.35

        turn_level_left = 0.92
        turn_level_right = -0.95
        ninety_degree_duration = 0.46  # seconds to turn 90 degrees


        # Drive in a square
        for _ in range(4):
            # Drive straight
            left_motor.setlevel(straight_level_left)
            right_motor.setlevel(straight_level_right)
            time.sleep(one_meter_duration)

            # Drive backwards for a tiny bit
            left_motor.setlevel(-straight_level_left)
            right_motor.setlevel(-straight_level_right)
            time.sleep(0.05)

            # Stop motors 
            left_motor.stop()
            right_motor.stop()
            time.sleep(0.1)
            # Turn right 90 degrees
            left_motor.setlevel(turn_level_left)
            right_motor.setlevel(turn_level_right)
            time.sleep(ninety_degree_duration)

             # Drive backwards for a tiny bit
            left_motor.setlevel(-straight_level_left)
            right_motor.setlevel(-straight_level_right)
            time.sleep(0.05)

            # Stop motors 
            left_motor.stop()
            right_motor.stop()
            time.sleep(0.1)
        left_motor.stop
        right_motor.stop

    except BaseException as ex:
        print("Ending due to exception:", repr(ex))
        traceback.print_exc()
    finally:
        left_motor.stop()
        right_motor.stop()
        io.stop()