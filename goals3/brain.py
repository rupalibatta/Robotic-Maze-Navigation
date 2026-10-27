#!/usr/bin/env python3

import pigpio
from drivesystem import DriveSystem
from linesensor import LineSensor
from behaviors import line_following, pull_forward, turn


def main():
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connect to pigpio daemon.")
        return

    drive = DriveSystem(io)
    sensor = LineSensor(io)

    try:
        print("Robot initialized.")
        while True:
            print("\nOptions: [straight]  [left]  [right]  [q to quit]")
            command = input("Enter command: ").strip().lower()

            if command == "q":
                print("Shutting down.")
                break

            elif command == "straight":
                result = line_following(drive, sensor)
                if result == "intersection":
                    pull_forward(drive)

            elif command in ["left", "right"]:
                turn(drive, sensor, direction=command)

            else:
                print("Invalid input. Try: straight, left, right, q")

    except BaseException as ex:
        print("Error occurred:", repr(ex))
    finally:
        drive.stop()
        io.stop()

if __name__ == "__main__":
    main()
