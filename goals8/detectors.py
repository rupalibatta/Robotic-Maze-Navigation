#!/usr/bin/env python3
import time

class SideEstimator:
    """
    Estimates LEFT / CENTER / RIGHT from raw line readings.
    Uses exponential smoothing of raw lateral guess.
    """
    def __init__(self, T=0.05):
        self.T = T
        self.level = 0.0
        self.state = "CENTER"

    def update(self, reading, dt):
        if reading == (0, 0, 0):
            return self.state
        
        raw = 0.0
        if reading in [(1, 1, 0), (1, 0, 0)]:
            raw = -0.5 if reading == (1, 1, 0) else -1.0
        elif reading in [(0, 1, 1), (0, 0, 1)]:
            raw = 0.5 if reading == (0, 1, 1) else 1.0

        self.level += dt / self.T * (raw - self.level)

        if self.level < -0.3:
            self.state = "RIGHT"
        elif self.level > 0.3:
            self.state = "LEFT"
        else:
            self.state = "CENTER"
        return self.state


class IntersectionDetector:
    """
    Detects intersection when all sensors read black (1, 1, 1).
    """
    def __init__(self, T=0.1304, threshold=0.37):
        self.T = T
        self.threshold = threshold
        self.level = 0.0
        self.state = False

    def update(self, reading, dt):
        raw = 1.0 if reading == (1, 1, 1) else 0.0
        self.level += dt / self.T * (raw - self.level)

        if self.level > self.threshold:
            self.state = True
            self.level = 0.0
        elif self.level < 1 - self.threshold:
            self.state = False

        return self.state


class EndDetector:
    """
    Detects end of the line if (0, 0, 0) and the side is CENTER.
    """
    def __init__(self, T=0.125, threshold=0.37):
        self.T = T
        self.threshold = threshold
        self.level = 0.0
        self.state = False

    def update(self, reading, side_state, dt):
        raw = 1.0 if reading == (0, 0, 0) and side_state == "CENTER" else 0.0
        self.level += dt / self.T * (raw - self.level)

        if self.level > self.threshold:
            self.state = True
            self.level = 0.0
        elif self.level < 1 - self.threshold:
            self.state = False
        return self.state


class TurnDetector:
    """
    Detects when the robot leaves the current street and locks onto the new one
    using the leading IR sensor (left or right depending on turn direction).
    """
    def __init__(self, direction, T=0.13, thresh=0.37):
        self.T = T
        self.thresh = thresh
        self.level = 1.0
        self.state = "WAIT_OFF"
        self.done = False

        # Set which sensor is leading
        if direction.upper() == "LEFT":
            self.leader_index = 0 
        elif direction.upper() == "RIGHT":
            self.leader_index = 2  # Right sensor
        else:
            raise ValueError("Direction must be 'LEFT' or 'RIGHT'")

    def update(self, reading, dt):
        raw = reading[1]  # either l or r
        self.level += dt / self.T * (raw - self.level)

        if self.state == "WAIT_OFF":
            if self.level < self.thresh:
                self.state = "WAIT_ON"
        elif self.state == "WAIT_ON":
            if self.level > (1 - self.thresh):
                self.state = "DONE"
                self.done = True

        return self.done
