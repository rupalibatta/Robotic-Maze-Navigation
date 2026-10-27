#!/usr/bin/env python3

import pigpio
import traceback
import pickle
from drivesystem import DriveSystem
from linesensor import LineSensor
from anglesensor import ADC, AngleSensor
from behaviors import Behaviors
from brain import brain
from map import Map

def main():
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connect to GPIO!")
        return

    try:
        # INIT GPIO-DEPENDENT OBJECTS FIRST
        adc = ADC(io)
        anglesensor = AngleSensor(adc)
        drive = DriveSystem(io)
        sensor = LineSensor(io)
        anglesensor.calibrate_spin(drive, sensor)

        # LOAD OR CREATE MAP
        load_ans = input("Load an existing map? (y/n): ").strip().lower()

        if load_ans == 'y':
            filename = input("Filename to load map from: ").strip()
            print("Loading the map from %s..." % filename)
            with open(filename, 'rb') as file:
                amap = pickle.load(file)
            print("Map loaded successfully.")
            
            # Show the loaded map
            amap.show()

            # Optionally adjust pose
            use_pose = input("Use saved robot pose? (y/n): ").strip().lower()
            if use_pose != 'y':
                amap.x = int(input("Enter robot's current X coordinate: "))
                amap.y = int(input("Enter robot's current Y coordinate: "))
                amap.heading = int(input("Enter robot's current heading [0–7]: "))

        else:
            # Create new map
            amap = Map()
            # Prompt to use default heading and position
            curr_pose = input("Use default robot pose (0, 0) and heading (0)? (y/n): ").strip().lower()
            if curr_pose != 'y':
                amap.x = int(input("Enter robot's current X coordinate: "))
                amap.y = int(input("Enter robot's current Y coordinate: "))
                amap.heading = int(input("Enter robot's current heading [0–7]: "))

        # START BEHAVIORS
        behaviors = Behaviors(drive, sensor)
        print("Robot initialized. Ready to run.")
        brain(behaviors, amap, anglesensor)

    except BaseException as ex:
        print("Ending due to exception:", repr(ex))
        traceback.print_exc()

    finally:
        #  SAVE MAP
        try:
            save_ans = input("Save map before quitting? (y/n): ").strip().lower()
            if save_ans == 'y':
                filename = input("Filename to save map to: ").strip()
                print("Saving the map to %s..." % filename)
                with open(filename, 'wb') as file:
                    pickle.dump(amap, file)
                print("Done.")
        except Exception:
            pass

        print("Shutting down robot.")
        drive.stop()
        io.stop()

if __name__ == "__main__":
    main()
