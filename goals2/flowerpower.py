#!/usr/bin/env python3

import pigpio
import time
import traceback
from drivesystem import DriveSystem


styles = [
    'SPIN_LEFT', 'HOOK_LEFT', 'TURN_LEFT', 'STEER_LEFT', 'VEER_LEFT',
    'STRAIGHT',
    'VEER_RIGHT', 'STEER_RIGHT', 'TURN_RIGHT', 'HOOK_RIGHT', 'SPIN_RIGHT'
]

'''def main():
    io = pigpio.pi()
    if not io.connected:
        print("Failed to connect to pigpio daemon.")
        return

    drive = DriveSystem(io)

    try:
        for style in styles:
            print(f"\nDriving style: {style}")
            drive.drive(style)
            time.sleep(3)
            drive.stop()
            input("Hit return to continue...")

    except BaseException as ex:
        print("Ending due to exception:", repr(ex))
        traceback.print_exc()
    finally:
        drive.stop()
        io.stop()

if __name__ == "__main__":
    main()'''


def main():
    io = pigpio.pi()
    if not io.connected:
        print("Failed to connect to pigpio daemon.")
        return

    drive = DriveSystem(io)

    print("Available drive styles:")
    for i, style in enumerate(styles):
        print(f"{i}: {style}")
    print("\nEnter a number (0-10) to test that style, or 'q' to quit.\n")

    try:
        while True:
            choice = input("Select style number: ").strip()
            if choice.lower() == 'q':
                break
            if not choice.isdigit() or not (0 <= int(choice) < len(styles)):
                print("Invalid input. Please enter a number between 0 and 10.")
                continue

            style = styles[int(choice)]

            # Pre-move forward
            print("\nPre-moving forward...")
            drive.left_motor.setlevel(0.75)
            drive.right_motor.setlevel(0.73)
            time.sleep(0.1)
            drive.stop()

            print(f"\nExecuting style: {style}")
            drive.drive(style)
            time.sleep(3)
            drive.stop()

    except BaseException as ex:
        print("Ending due to exception:", repr(ex))
        traceback.print_exc()
    finally:
        drive.stop()
        io.stop()

if __name__ == "__main__":
    main()