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


class DriveSystem:
    """
    DriveSystem(io)

    High-level control for robot movement.
    Internally manages two Motor objects and maps 11 driving styles to motor commands.
    Allows the robot to drive in arcs ranging from straight to spinning in place.
    """
    def __init__(self, io):
        self.left_motor = Motor(io, 8, 7)
        self.right_motor = Motor(io, 5, 6)

        self.motions = {
            'SPIN_LEFT':   (-0.71, 0.70), # -81, 80
            'HOOK_LEFT':   (0.00, 0.99),
            'TURN_LEFT':   (0.6, 1), 
            'STEER_LEFT':  (0.79, 0.97), 
            'VEER_LEFT':   (0.80, 0.92), 
            'STRAIGHT':    (0.81, 0.79), # 0.91, 0.89
            'VEER_RIGHT':  (0.94, 0.79),
            'STEER_RIGHT': (0.99, 0.74), 
            'TURN_RIGHT':  (1, 0.51),
            'HOOK_RIGHT':  (0.99, 0.00),
            'SPIN_RIGHT':  (0.71, -0.72) # 81, -82
        }

    def drive(self, style):
        if style not in self.motions:
            print(f"[ERROR] Unknown drive style: {style}")
            self.stop()
            return
        left, right = self.motions[style]
        self.left_motor.setlevel(left)
        self.right_motor.setlevel(right)

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
