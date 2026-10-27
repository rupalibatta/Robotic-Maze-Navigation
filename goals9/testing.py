#!/usr/bin/env python3
import pigpio
import time
import sys
import csv
from drivesystem import DriveSystem
from linesensor import LineSensor
from anglesensor import ADC, AngleSensor
from behaviors import Behaviors

CSV_FILENAME = 'turn_data.csv'


def normalize_angle(diff):
    # Wrap into [-180, 180]
    return (diff + 180) % 360 - 180


def main():
    # Initialize GPIO and hardware
    io = pigpio.pi()
    if not io.connected:
        print("Failed to connect to pigpio daemon.")
        sys.exit(1)

    drive = DriveSystem(io)
    line_sensor = LineSensor(io)
    adc = ADC(io)
    anglesensor = AngleSensor(adc)
    behaviors = Behaviors(drive, line_sensor)
    anglesensor.calibrate_spin(drive, line_sensor)

    # Prepare CSV file with header
    with open(CSV_FILENAME, mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['manual_est_deg', 'turn_time_s', 'magnetometer_deg'])

    print("Manual turn recorder started.")
    print("Commands: L=turn left, R=turn right, S=drive straight, Q=quit")

    try:
        while True:
            cmd = input("Enter command [L/R/S/Q]: ").strip().upper()
            if cmd == 'Q':
                print("Exiting.")
                break

            if cmd == 'S':
                print("Driving straight. Press ENTER to stop.")
                drive.drive('STRAIGHT')
                input()
                drive.stop()
                continue

            if cmd in ('L', 'R'):
                direction = 'LEFT' if cmd == 'L' else 'RIGHT'
                # Record initial state
                phi_start = anglesensor.readangle()
                t_start = time.time()

                # Perform the turn using Behaviors.turn()
                behaviors.turn(direction, anglesensor)

                # Record final state
                t_end = time.time()
                phi_end = anglesensor.readangle()

                # Compute metrics
                turn_time = t_end - t_start
                diff = normalize_angle(phi_end - phi_start)
                print(f"Turn: magnetometer={diff:.1f}°, time={turn_time:.3f}s")

                # Ask user for manual estimate
                while True:
                    try:
                        manual_est = float(input("Enter your estimated turn angle (degrees): "))
                        break
                    except ValueError:
                        print("Invalid number. Please enter a numeric angle.")

                # Append to CSV
                with open(CSV_FILENAME, mode='a', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow([f"{manual_est:.1f}", f"{turn_time:.3f}", f"{diff:.1f}"])
                print("Logged data to CSV.")
                continue

            print("Unknown command. Use L, R, S, or Q.")

    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        drive.stop()
        io.stop()
        print(f"Data saved in {CSV_FILENAME}.")


if __name__ == '__main__':
    main()
