#!/usr/bin/env python3

from behaviors import Behaviors
from map import Map, STATUS
from anglesensor import AngleSensor
from drivesystem import DriveSystem
from linesensor import LineSensor
import pigpio
import traceback

def brain(behaviors, amap, anglesensor):
    print("Robot brain initialized.")

    while True:
        amap.show()  # 1. Show map at beginning of loop

        cmd = input("\nEnter command: [L]eft, [R]ight, [S]traight, [Q]uit → ").strip().upper()

        if cmd == 'Q':
            print("Quitting...")
            break

        elif cmd == 'L':
            old_heading = amap.heading
            turn_amt = behaviors.turn("LEFT", anglesensor)
            amap.updateheading(turn_amt)

            print("old heading:", old_heading)

            for i in range(1, (turn_amt) % 8):
                skipped = (old_heading + i) % 8
                print("skipped l: ", skipped)
                if amap.getstreet(amap.x, amap.y, skipped) != STATUS.CONNECTED:
                    amap.setstreet(amap.x, amap.y, skipped, STATUS.NONEXISTENT)

            if amap.getstreet(amap.x, amap.y, amap.heading) != STATUS.CONNECTED:
                amap.setstreet(amap.x, amap.y, amap.heading, STATUS.UNEXPLORED)

        elif cmd == 'R':
            old_heading = amap.heading

            turn_amt = behaviors.turn("RIGHT", anglesensor)
            amap.updateheading(turn_amt)

            adjusted_turn = (0 - turn_amt) % 8

            print("old heading:", old_heading)

            for i in range(1, adjusted_turn):
                skipped = (old_heading - i) % 8
                print("skipped r: ", skipped)
                if amap.getstreet(amap.x, amap.y, skipped) != STATUS.CONNECTED:
                    amap.setstreet(amap.x, amap.y, skipped, STATUS.NONEXISTENT)

            if amap.getstreet(amap.x, amap.y, amap.heading) != STATUS.CONNECTED:
                amap.setstreet(amap.x, amap.y, amap.heading, STATUS.UNEXPLORED)

        elif cmd == 'S':
            result = behaviors.line_following()

            if result == "intersection":
                # a. movement update
                prev_x, prev_y, prev_heading = amap.x, amap.y, amap.heading
                amap.updatemove()

                # a. set connection between prev and new
                amap.setstreet(prev_x, prev_y, prev_heading, STATUS.CONNECTED)
                reverse = (prev_heading + 4) % 8
                amap.setstreet(amap.x, amap.y, reverse, STATUS.CONNECTED)

                # mark unknown directions
                for h in range(8):
                    if h != reverse:
                        current_status = amap.getstreet(amap.x, amap.y, h)
                        if current_status == STATUS.UNKNOWN: 
                            amap.setstreet(amap.x, amap.y, h, STATUS.UNKNOWN)

                behaviors.pull_forward()

            elif result == "end":
                print("End of street. Performing U-turn...")

                old_heading = amap.heading
                turn_amt1 = behaviors.turn("LEFT", anglesensor)
                new_heading = (amap.heading + turn_amt1) % 8
                target_heading = (old_heading + 4) % 8

                total_turn = turn_amt1

                amap.updateheading(total_turn)

                # DEADEND mark
                amap.setstreet(amap.x, amap.y, old_heading, STATUS.DEADEND)

                result = behaviors.line_following()
                behaviors.pull_forward()

        else:
            print("Invalid command. Use [L], [R], [S], or [Q].")