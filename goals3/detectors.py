#!/usr/bin/env python3
import time


class SideEstimator:
    """
    Estimates LEFT / CENTER / RIGHT from raw line readings.
    Uses exponential smoothing of raw lateral guess.
    """
    def __init__(self, T=0.1):
        self.T = T
        self.level = 0.0
        self.state = "CENTER"

    def update(self, reading, dt):
        raw = 0.0
        if reading in [(1, 1, 0), (1, 0, 0)]:
            raw = -0.5 if reading == (1, 1, 0) else -1.0
        elif reading in [(0, 1, 1), (0, 0, 1)]:
            raw = 0.5 if reading == (0, 1, 1) else 1.0

        self.level += dt / self.T * (raw - self.level)

        if self.level < -0.05:
            self.state = "RIGHT"
        elif self.level > 0.05:
            self.state = "LEFT"
        else:
            self.state = "CENTER"

        return self.state


class IntersectionDetector:
    """
    Detects intersection when all sensors read black (1, 1, 1).
    """
    def __init__(self, T=0.12, threshold=0.4):
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
    def __init__(self, T=0.1, threshold=0.5):
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
    def __init__(self, T=0.01, thresh=0.37):
        self.T = T
        self.level = 1.0
        self.state = "WAIT_OFF"
        self.thresh = thresh
        self.done = False

    def update(self, reading, dt):
        l, m, r = reading
        side_on = l or r
        raw = 1.0 if m == 1 else 0.0

        self.level += dt / self.T * (raw - self.level)

        if self.state == "WAIT_OFF":
            if m == 0 and not side_on:
                self.state = "WAIT_ON"
        
        elif self.state == "WAIT_ON":
            if self.level > self.thresh and side_on:
                self.state = "DONE"
                self.done = True

        return self.done