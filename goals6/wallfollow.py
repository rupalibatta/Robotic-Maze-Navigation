#!/usr/bin/env python3
import pigpio
import time
import sys
from drivesystem import DriveSystem
from proximitysensor import ProximitySensor


def wall_follow_behavior(drive: DriveSystem, sensor: ProximitySensor,
                         side='right'):
    """
    Discrete error-band wall-following on one side ('left' or 'right').

    Error bands (nominal distance d0=0.30m, max_err=0.10m, band=max_err/3):
      e = d - d0
      |e| > max_err  -> STOP
      e < -2*band    -> TURN_LEFT
      -2*band <= e < -band -> STEER_LEFT
      -band <= e < 0       -> VEER_LEFT
      0 <= e < band        -> STRAIGHT
      band <= e < 2*band   -> VEER_RIGHT
      2*band <= e < 3*band -> STEER_RIGHT
      3*band <= e <= max_err -> TURN_RIGHT
    """
    d0 = 0.30        # nominal distance (m)
    max_err = 0.10   # maximum allowed error (m)
    band = max_err / 3.0

    # initial trigger
    sensor.trigger()
    last_trigger = time.time()

    try:
        while True:
            now = time.time()
            # re-trigger every 50 ms
            if now - last_trigger >= 0.050:
                sensor.trigger()
                last_trigger = now

            # read distances
            dleft, dmid, dright = sensor.read()
            d = dleft if side == 'left' else dright
            e = d - d0

            # choose action based on error e
            if abs(e) > max_err:
                drive.stop()
            elif e < -2 * band:
                drive.drive('TURN_LEFT')
            elif e < -band: 
                drive.drive('STEER_LEFT')
            elif e < 0:
                drive.drive('VEER_LEFT')
            elif e < band:
                drive.drive('STRAIGHT')
            elif e < 2 * band:
                drive.drive('VEER_RIGHT')
            elif e < 3 * band:
                drive.drive('STEER_RIGHT')
            else:
                drive.drive('TURN_RIGHT')

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Wall-following interrupted by user.")
        drive.stop()


def wall_follow_prop(drive: DriveSystem, sensor: ProximitySensor, side='right'):
    d0, max_err = 0.30, 0.10

    cl, kl = 0.816, 2.011
    cr, kr = 0.853, -2.330

    sensor.trigger()
    last_t = time.time()
    try:
        while True:
            now = time.time()
            if now - last_t >= 0.050:
                sensor.trigger()
                last_t = now

            dl, _, dr = sensor.read()
            d = dl if side=='left' else dr
            e = d - d0

            if abs(e) > max_err:
                drive.stop()
            else:
                pwm_l = cl + kl * e
                pwm_r = cr + kr * e
                drive.pwm(pwm_l, pwm_r)

            time.sleep(0.01)
    except KeyboardInterrupt:
        drive.stop()


if __name__ == '__main__':
    io = pigpio.pi()
    if not io.connected:
        print("Failed to connect to pigpio daemon.")
        sys.exit(1)

    drive = DriveSystem(io)
    sensor = ProximitySensor(io)
    print("Starting discrete wall-following (right side). Press Ctrl+C to stop.")
    wall_follow_behavior(drive, sensor, side='right')

    print("Shutting down.")
    drive.stop()
    io.stop()