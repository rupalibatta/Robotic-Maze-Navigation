#!/usr/bin/env python3
import pickle
import traceback
import time
from map import Map, STATUS
from drivesystem import DriveSystem
from linesensor import LineSensor
from anglesensor import ADC, AngleSensor
from behaviors import Behaviors

def brain(behaviors, amap, anglesensor):
    """
    Main decision loop.  Supports:
      - Manual mode: [L]eft, [R]ight, [S]traight, [G]oto, [Q]uit
      - Auto mode (after goto): follows optimal directions to goal
    """
    goal = None

    print("Robot brain initialized.")
    while True:
        # always redraw the map + robot
        amap.show()

        # prompt differs in manual vs auto
        cmd = input("\nEnter [L]eft, [R]ight, [S]traight, [G]oto, [Q]uit → ").strip().upper()
        
        # quit is always allowed
        if cmd == 'Q':
            print("Quitting...")
            break

        # ───── Manual Mode ───────────────────────────────────────────────────────
        if goal is None:
            if cmd in ('L', 'R'):
                # manual turn
                turn_dir = "LEFT" if cmd == 'L' else "RIGHT"
                old_h = amap.heading
                turn_amt = behaviors.turn(turn_dir, anglesensor)
                amap.updateheading(turn_amt)

                # mark skipped directions as NONEXISTENT
                skipped = amap.skipped_directions(turn_amt, turn_dir)
                for skip in skipped:
                    amap.setstreet(amap.x, amap.y, skip, STATUS.NONEXISTENT)

                # mark the new heading edge as UNEXPLORED if not already CONNECTED
                if amap.getstreet(amap.x, amap.y, amap.heading) != STATUS.CONNECTED:
                    amap.setstreet(amap.x, amap.y, amap.heading, STATUS.UNEXPLORED)

            elif cmd == 'S':
                # manual straight
                result = behaviors.line_following()
                if result == "intersection":
                    prev_x, prev_y, prev_h = amap.x, amap.y, amap.heading
                    amap.updatemove()
                    amap.setstreet(prev_x, prev_y, prev_h, STATUS.CONNECTED)
                    amap.setstreet(amap.x, amap.y, (prev_h+4)%8, STATUS.CONNECTED)
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

            elif cmd == 'G':
                # set a new goal and run Dijkstra
                try:
                    gx = int(input("Goal X: "))
                    gy = int(input("Goal Y: "))
                except ValueError:
                    print("Invalid coordinates.")
                    continue

                print(f"Computing optimal paths to ({gx},{gy})…")
                amap.cleargoal()
                amap.setgoal(gx, gy)

                print("GOAL is", goal)

                # test reachability from current node
                start = amap.getintersection(amap.x, amap.y)
                if start.direction_to_goal is None:
                    print("Goal unreachable from current map.")
                    amap.cleargoal()
                else:
                    print(f"Path found!  First step: heading {start.direction_to_goal}")
                    goal = (gx, gy)

                    print("GOAL is", goal)

            else:
                print("Invalid command. Use L/R/S/G/Q.")

        # ── Auto mode ────────────────────────────────────────────────────────────────
        if goal:
            while (amap.x, amap.y) != goal:
                #print(f"[DEBUG] Auto‐mode: pos=({amap.x},{amap.y}), heading={amap.heading}, goal={goal}")
                if cmd == 'C':
                    print("Auto‐drive cancelled.")
                    amap.cleargoal()
                    goal = None
                    continue

                node = amap.getintersection(amap.x, amap.y)
                dir_goal = node.direction_to_goal
                #print(f"[DEBUG] direction_to_goal = {dir_goal}")
                if dir_goal is None:
                    print("No path to goal (disconnected). Cancelling auto‐drive.")
                    amap.cleargoal()
                    goal = None
                    continue

                # 1) TURN again and again if needed
                if amap.heading != dir_goal:
                    print(f"[AUTO] Need heading {dir_goal}, currently {amap.heading} → aligning...")

                    # keep going until we hit the exact target
                    while amap.heading != dir_goal:
                        # recompute how many 45° steps and which way to spin
                        diff = (dir_goal - amap.heading) % 8
                        turn_dir = 'LEFT' if diff <= 4 else 'RIGHT'
                        # print(f"[AUTO]   → {turn_dir} by {diff*45}° (diff={diff})")

                        # mark skipped directions as NONEXISTENT
                        turn_amt = behaviors.turn(turn_dir, anglesensor)
                        skipped = amap.skipped_directions(turn_amt, turn_dir)
                        for skip in skipped:
                            amap.setstreet(amap.x, amap.y, skip, STATUS.NONEXISTENT)

                        amap.updateheading(turn_amt)
                        # print(f"[DEBUG]    spun {amt} steps → new heading={amap.heading}")
                        continue

                # 2) DRIVE forward one block
                print("Auto‐drive straight one block")
                result = behaviors.line_following()
                #print(f"[DEBUG] Auto line_following result: {result}")
                if result == "intersection":
                    px, py, ph = amap.x, amap.y, amap.heading
                    amap.updatemove()
                    print(f"Moved to ({amap.x},{amap.y})")
                    amap.setstreet(px, py, ph, STATUS.CONNECTED)
                    amap.setstreet(amap.x, amap.y, (ph+4)%8, STATUS.CONNECTED)
                    behaviors.pull_forward()
                elif result == "end":
                    print("Dead end encountered en route. Returning to last known intersection.")
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
                continue

            print("Goal reached!")

    print("Brain loop exited.")