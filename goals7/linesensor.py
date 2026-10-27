#!/usr/bin/env python3
import pigpio


# Default GPIO pins for the IR sensors
PIN_IR_LEFT   = 14
PIN_IR_MIDDLE = 15
PIN_IR_RIGHT  = 18

class IR:
    """
    IR(io, pin)

    Represents a single IR sensor connected to a GPIO pin.
    Provides a digital readout: 0 = white surface, 1 = black line.
    """
    def __init__(self, io, pin):
        self.io = io
        self.pin = pin
        self.io.set_mode(pin, pigpio.INPUT)

    def read(self):
        return self.io.read(self.pin)


class LineSensor:
    """
    LineSensor(io)

    Reads all three IR sensors (Left, Middle, Right) and returns a tuple (L, M, R)
    with values 0 or 1.
    """
    def __init__(self, io):
        self.left   = IR(io, PIN_IR_LEFT)
        self.middle = IR(io, PIN_IR_MIDDLE)
        self.right  = IR(io, PIN_IR_RIGHT)

    def read(self):
        l = self.left.read()
        m = self.middle.read()
        r = self.right.read()
        return (l, m, r)