#!/usr/bin/env python3

import time
from detectors import SideEstimator, IntersectionDetector, EndDetector, TurnDetector


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

def line_following(drive, sensor):
    """
    Line following loop using detectors.
    Returns "intersection" or "end" to trigger higher-level behaviors.
    """
    side_est = SideEstimator()
    end_det = EndDetector()
    intersection_det = IntersectionDetector()
    last_time = time.time()

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        reading = sensor.read()
        side_state = side_est.update(reading, dt)
        end_state = end_det.update(reading, side_state, dt)
        intersection_state = intersection_det.update(reading, dt)

        if intersection_state:
            drive.stop()
            print("Intersection detected.")
            return "intersection"

        if end_state:
            drive.stop()
            print("End of street detected.")
            return "end"

        if reading == (0, 0, 0):
            if side_state == "LEFT":
                drive.drive("SPIN_RIGHT")
            elif side_state == "RIGHT":
                drive.drive("SPIN_LEFT")
            else:
                drive.drive("STRAIGHT")
        else:
            action = FEEDBACK_TABLE.get(reading, 'STOP')
            if action == 'STOP':
                drive.stop()
            else:
                drive.drive(action)


def pull_forward(drive, duration=0.1):
    """
    Drives forward briefly to place wheels over intersection.
    """
    print("Pulling forward...")
    t0 = time.time()
    while time.time() < t0 + duration:
        drive.drive("STRAIGHT")
    drive.stop()
    print("Pull-forward complete.")


def turn(drive, sensor, direction):
    """
    Turns robot left or right until the new street is found using a TurnDetector.
    Then corrects final orientation and backdrives.
    """
    print(f"Turning {direction.upper()}...")
    spin_style = 'SPIN_LEFT' if direction.upper() == 'LEFT' else 'SPIN_RIGHT'

    det = TurnDetector()
    t0 = time.time()
    last_time = time.time()

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        reading = sensor.read()
        if det.update(reading, dt):
            break
        drive.drive(spin_style)

    drive.stop()
    t1 = time.time()
    spin_time = t1 - t0
    print(f"Turn complete. Duration: {spin_time:.3f} seconds.")

    # correct if off-center
    side_est = SideEstimator()
    correction_time = time.time()
    last_time = correction_time

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        reading = sensor.read()
        side = side_est.update(reading, dt)
        if side == "CENTER":
            break
        if side == "LEFT":
            drive.drive("SPIN_RIGHT")
        elif side == "RIGHT":
            drive.drive("SPIN_LEFT")
        time.sleep(0.01)

    drive.stop()

    # backdrive for 0.05 seconds
    t_back = time.time()
    while time.time() - t_back < 0.05:
        drive.left_motor.setlevel(-0.5)
        drive.right_motor.setlevel(-0.5)
    drive.stop()

    return spin_time
