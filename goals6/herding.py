#!/usr/bin/env python3
import pigpio
import time
from drivesystem import DriveSystem
from proximitysensor import ProximitySensor


class HerdingDrive(DriveSystem):
    """
    Extends DriveSystem to allow reverse driving by negating motor levels.
    """
    def drive(self, style, reverse=False):
        if style not in self.motions:
            print(f"[ERROR] Unknown drive style: {style}")
            self.stop()
            return
        left_level, right_level = self.motions[style]
        if reverse:
            left_level, right_level = -left_level, -right_level
        self.left_motor.setlevel(left_level)
        self.right_motor.setlevel(right_level)


def herd_behavior(drive, sensor):
    """
    Repulsive (herding) behavior using three ultrasound sensors.
    Forward/backward movement is controlled by the middle sensor:
      - >20cm: move forward
      - 10-20cm: stop
      - <10cm: move backward
    Spin is controlled by side sensors (<20cm):
      - only left triggered: spin right
      - only right triggered: spin left
      - else: go straight
    """
    # trigger first pulse
    sensor.trigger()
    last_trigger = time.time()

    far_thr = 0.20  # meters
    near_thr = 0.10

    try:
        while True:
            now = time.time()
            # retrigger every 50ms
            if now - last_trigger >= 0.050:
                sensor.trigger()
                last_trigger = now

            dleft, dmid, dright = sensor.read()

            # decide forward/backward
            if dmid > far_thr:
                fwd = 'forward'
            elif dmid < near_thr:
                fwd = 'backward'
            else:
                fwd = 'stop'

            # decide spin
            left_close = dleft < far_thr
            right_close = dright < far_thr
            if left_close and not right_close:
                spin = 'right'
            elif right_close and not left_close:
                spin = 'left'
            else:
                spin = None

            # combine actions
            if fwd == 'forward':
                if spin == 'left':
                    drive.drive('STEER_LEFT')
                elif spin == 'right':
                    drive.drive('STEER_RIGHT')
                else:
                    drive.drive('STRAIGHT')

            elif fwd == 'stop':
                if spin == 'left':
                    drive.drive('SPIN_LEFT')
                elif spin == 'right':
                    drive.drive('SPIN_RIGHT')
                else:
                    drive.stop()

            elif fwd == 'backward':
                if spin == 'left':
                    drive.drive('STEER_LEFT', reverse=True)
                elif spin == 'right':
                    drive.drive('STEER_RIGHT', reverse=True)
                else:
                    drive.drive('STRAIGHT', reverse=True)

            # small sleep to avoid busy-loop
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Herding behavior interrupted by user.")
        drive.stop()


if __name__ == '__main__':
    # initialize GPIO and devices
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connect to pigpio daemon!")
        sys.exit(1)

    drive = HerdingDrive(io)
    sensor = ProximitySensor(io)
    print("Starting herding behavior. Press Ctrl+C to stop.")
    herd_behavior(drive, sensor)

    print("Shutting down.")
    drive.stop()
    io.stop()
