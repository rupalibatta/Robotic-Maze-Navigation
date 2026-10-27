#!/usr/bin/env python3
import pickle
import time
import math
from enum import Enum, auto

from map import Map, STATUS, HEADING_DELTAS
from drivesystem import DriveSystem
from linesensor import LineSensor
from anglesensor import ADC, AngleSensor
from behaviors import Behaviors
from nfc import NFCSensor


class Mode(Enum):
    MANUAL = auto()
    EXPLORING = auto()
    DRIVE_TO_GOAL = auto()
    FETCH = auto()


class ActionType(Enum):
    TURN = auto()
    STRAIGHT = auto()
    SET_GOAL = auto()
    START_EXPLORATION = auto()
    QUIT = auto()
    NONE = auto()


class Action:
    def __init__(self, type_, **kwargs):
        self.type = type_
        self.params = kwargs


class State:
    def __init__(self, behaviors, amap, anglesensor, shared):
        self.behaviors = behaviors
        self.map = amap
        self.anglesensor = anglesensor
        self.shared = shared
        self.mode = Mode.MANUAL
        self.goal = None
        self.goal_set_by_explore = False
        self.temporary_goal = None
        self.last_position = None
        self.home_location = None
        self.last_spin_heading = None
        self.fetch_prize_id = None
        self.last_prize_distance = None
        self.last_nfc_intersection = None


def closest_viable_heading(current_heading, inter):
    """Return the heading closest to current_heading that is neither NONEXISTENT nor blocked."""
    candidates = [
        h for h, st in enumerate(inter.streets)
        if st not in (STATUS.NONEXISTENT,)
    ]
    if not candidates:
        return None
    return min(candidates,
               key=lambda h: min((h-current_heading)%8, (current_heading-h)%8))


def decide(state: State) -> Action:
    """Main decision dispatcher: MANUAL / EXPLORING / DRIVE_TO_GOAL (or DIRECTED)."""
    m = state.map
    m.show()

    # debug listing
    if state.mode == Mode.EXPLORING:
        unfinished = [
            coords for coords, node in m.intersections.items()
            if any(st in (STATUS.UNKNOWN, STATUS.UNEXPLORED) and not node.blocked[h]
                   for h, st in enumerate(node.streets))
        ]
        print(f"Unfinished intersections: {unfinished}")
        return _decide_explore(state)

    if state.mode == Mode.FETCH:
        print("enter fetch")
        action = _decide_fetch(state)
        state.mode = Mode.FETCH
        return action

    if state.mode == Mode.DRIVE_TO_GOAL:
        # if goal on map, classical Dijkstra‐drive
        if state.goal in m.intersections:
            if (m.x, m.y) == state.goal:
                if state.temporary_goal:
                    print("Reached temporary sub-goal intersection.")
                    state.temporary_goal = None
                    state.mode = Mode.DRIVE_TO_GOAL
                print("Reached goal intersection.")
                if state.goal_set_by_explore:
                    print("Goal was from exploration — resuming explore.")
                    state.mode = Mode.EXPLORING
                else:
                    print("Goal was user-defined — stopping.")
                    state.mode = Mode.MANUAL
                return Action(ActionType.NONE)

            node = m.getintersection(m.x, m.y)
            tgt = node.direction_to_goal
            if tgt is None:
                print(f"[INFO] No path to goal {state.goal} from {(m.x, m.y)} — re-seeding Dijkstra")
                m.cleargoal()
                m.setgoal(*state.goal)
                node = m.getintersection(m.x, m.y)
                tgt = node.direction_to_goal
                if tgt is None:
                    print("[FAIL] Still no path. Switching to EXPLORE.")
                    if state.goal_set_by_explore:
                        print("Goal was from exploration — switching to EXPLORE.")
                        state.mode = Mode.EXPLORING
                    else:
                        print("Goal was user-defined — stopping.")
                        state.mode = Mode.MANUAL
                    return Action(ActionType.NONE)
                    
            # if that heading is blocked/nonexistent, pick a fallback
            if node.streets[tgt] == STATUS.NONEXISTENT or node.blocked[tgt]:
                fb = closest_viable_heading(m.heading, node)
                if fb is not None:
                    tgt = fb
                else:
                    return Action(ActionType.NONE)

            if m.heading != tgt:
                diff = (tgt - m.heading) % 8
                return Action(ActionType.TURN,
                              direction='LEFT' if diff <= 4 else 'RIGHT')
            return Action(ActionType.STRAIGHT)

        # otherwise, directed explore toward unknown goal
        return _decide_directed_explore(state)

    # MANUAL: do nothing here (UI thread issues TURN/STRAIGHT via execute)
    return Action(ActionType.NONE)


def _decide_explore(state: State) -> Action:
    """Pure explore: spin to reveal UNKNOWN, then pick UNEXPLORED→CONNECTED, skipping blockages."""
    m = state.map
    inter = m.getintersection(m.x, m.y)

    # reset spin on arrival
    if state.last_position != (m.x, m.y):
        state.last_position = (m.x, m.y)
        state.last_spin_heading = None

    # spin to get rid of UNKNOWN
    if STATUS.UNKNOWN in inter.streets:
        print(f"Spinning to reveal UNKNOWN at {(m.x,m.y)}…")
        state.last_spin_heading = m.heading
        return Action(ActionType.TURN, direction='LEFT')

    # Streets still unexplored
    unexplored = [
        h for h, st in enumerate(inter.streets)
        if st == STATUS.UNEXPLORED and not inter.blocked[h]
    ]
    if unexplored:
        cur = m.heading
        target = min(unexplored, key=lambda h: min((h - cur) % 8, (cur - h) % 8))
        diff = (target - cur) % 8
        if diff:
            return Action(ActionType.TURN, direction='LEFT' if diff <= 4 else 'RIGHT')
        return Action(ActionType.STRAIGHT)

    # Go to closest reachable intersection with unexplored or unknown
    interesting = [
        (x, y) for (x, y), node in m.intersections.items()
        if any(st in (STATUS.UNEXPLORED, STATUS.UNKNOWN) and not node.blocked[h]
               for h, st in enumerate(node.streets))
    ]

    if state.mode == Mode.FETCH:
        print("Change to Fetch")
        return Action(ActionType.NONE)

    if not interesting:
        print("Exploration complete.")
        state.mode = Mode.MANUAL
        return Action(ActionType.NONE)

    # Try all reachable interesting intersections and pick closest by Dijkstra cost
    m.cleargoal()
    best_target = None
    best_cost = float('inf')

    for pos in sorted(interesting, key=lambda p: math.hypot(p[0] - m.x, p[1] - m.y)):
        m.setgoal(*pos)
        node = m.getintersection(m.x, m.y)
        if node.direction_to_goal is not None:
            best_target = pos
            break

    if best_target is None:
        print("No reachable unexplored intersections.")
        return Action(ActionType.NONE)

    print(f"Exploration: traveling to reachable unexplored intersection at {best_target}")
    state.goal = best_target
    state.goal_set_by_explore = True
    state.mode = Mode.DRIVE_TO_GOAL
    return Action(ActionType.NONE)


def _decide_directed_explore(state: State) -> Action:
    """
    Directed explore toward a goal not yet on the map:
      1) Locally, take any UNEXPLORED/UNKNOWN street roughly toward goal.
      2) Fallback to pure explore locally.
      3) Globally, pick the closest “interesting” intersection and re-seed Dijkstra.
    """
    print('in directed explore')
    m = state.map
    gx, gy = state.goal
    inter = m.getintersection(m.x, m.y)

    # reset spin on arrival
    if state.last_position != (m.x, m.y):
        state.last_position = (m.x, m.y)
        state.last_spin_heading = None

    # spin to get rid of UNKNOWN
    if STATUS.UNKNOWN in inter.streets:
        print(f"Spinning to reveal UNKNOWN at {(m.x,m.y)}…")
        state.last_spin_heading = m.heading
        return Action(ActionType.TURN, direction='LEFT')

    # desired heading toward goal
    dx, dy = gx - m.x, gy - m.y
    goal_angle = math.atan2(dy, dx)

    local = [
        h for h in range(8)
        if inter.streets[h] in (STATUS.UNEXPLORED, STATUS.UNKNOWN)
        and not inter.blocked[h]
    ]
    if local:
        # Find the unexplored heading closest to the goal direction
        def score(h):
            hd_dx, hd_dy = HEADING_DELTAS[h]
            street_angle = math.atan2(hd_dy, hd_dx)
            angle_diff = abs((street_angle - goal_angle + math.pi) % (2 * math.pi) - math.pi)
            turn_steps = min((h - m.heading) % 8, (m.heading - h) % 8)
            total_score = (3.0 * angle_diff) + turn_steps

            print(f"  Heading {h}: street_angle={math.degrees(street_angle):.1f}°, "
                  f"angle_diff={math.degrees(angle_diff):.1f}°, "
                  f"turn_steps={turn_steps}, score={total_score:.2f}")
            return total_score

        best_heading = min(local, key=score)
        diff = (best_heading - m.heading) % 8

        print(f"Selected heading {best_heading} with diff {diff} steps from current heading.")

        if diff:
            return Action(ActionType.TURN, direction='LEFT' if diff <= 4 else 'RIGHT')
        return Action(ActionType.STRAIGHT)

    # fallback to pure-explore local behavior
    act = _decide_explore(state)
    if act.type != ActionType.NONE:
        return act

    # global: find all interesting intersections
    interesting = [
        (x,y) for (x,y), node in m.intersections.items()
        if any(st in (STATUS.UNEXPLORED, STATUS.UNKNOWN)
               and not node.blocked[h]
               for h, st in enumerate(node.streets))
    ]
    if not interesting:
        print("Directed explore: nothing left.")
        state.mode = Mode.MANUAL
        return Action(ActionType.NONE)

    # Try all reachable interesting intersections and pick closest to goal by Dijkstra cost
    m.cleargoal()
    best_target = None

    for pos in sorted(interesting, key=lambda p: math.hypot(p[0] - gx, p[1] - gy)):
        m.setgoal(*pos)
        node = m.getintersection(m.x, m.y)
        if node.direction_to_goal is not None:
            best_target = pos
            break

    if best_target is None:
        print("No reachable unexplored intersections.")
        return Action(ActionType.NONE)

    print(f"Directed explore: navigating to interesting intersection at {best_target}")
    state.temporary_goal = best_target
    state.goal_set_by_explore = True
    state.mode = Mode.DRIVE_TO_GOAL
    return Action(ActionType.NONE)


def _decide_fetch(state: State) -> Action:
    """
    Fetch mode: use intersection prize distances to localize the prize.
        1) The robot explores locally until it finds a two prize distance (i.e. scans two separate NFC tag).
        2) If the distance increases, it reverses direction.
        3) If the distance decreases, it continues toward the closest known intersection with a prize distance.
        4) Once two prize distances read 0.5, it localizes the prize between those two intersections.
    """
    m = state.map
    prize_id = state.fetch_prize_id

    print("Fetch mode: seeking prize...")

    # Check for prize localization
    candidates = m.possible_prize_locations(prize_id)
    if candidates:
        print("Prize found! Localized between these intersections:")
        for i1, i2 in candidates:
            print(f"    - ({i1.x}, {i1.y}) and ({i2.x}, {i2.y})")
        print("Fetch complete. Returning to manual mode.")
        state.mode = Mode.MANUAL
        return Action(ActionType.NONE)

    # Check current intersection prize distance
    inter = m.getintersection(m.x, m.y)
    if inter:
        print("Yes")
    current_distance = inter.get_prize_distance(prize_id)

    if current_distance is not None:
        print(f"[FETCH] Prize distance at ({inter.x}, {inter.y}) is {current_distance}")

        if state.last_prize_distance is not None:
            if current_distance > state.last_prize_distance:
                print("[FETCH] Distance increased! Reversing direction.")
                state.last_prize_distance = current_distance
                desired_heading = (m.heading + 4) % 8
                if desired_heading != m.heading:
                    diff = (desired_heading - m.heading) % 8
                    return Action(ActionType.TURN, direction='LEFT' if diff <= 4 else 'RIGHT')
                return Action(ActionType.STRAIGHT)

        # Distance decreased or first reading
        state.last_prize_distance = current_distance
        state.last_nfc_intersection = (inter.x, inter.y)

    # Find intersections with known prize distance
    known_intersections = [
        inter for inter in m.intersections.values()
        if inter.get_prize_distance(prize_id) is not None
    ]
    print(f"Known intersections with prize distance: {len(known_intersections)}")

    if known_intersections:
        # Steepest descent: pick the intersection with the smallest distance
        target = min(known_intersections, key=lambda inter: inter.get_prize_distance(prize_id))
        print(f"Closest known intersection to prize is at ({target.x}, {target.y}), distance = {target.get_prize_distance(prize_id)}")

        if (m.x, m.y) == (target.x, target.y):
            print("At closest known intersection — locally exploring for new NFC reads.")
            state.mode = mode.FETCH
            action = _decide_explore(state)
            return action

        # Try direct path to the target
        m.cleargoal()
        m.setgoal(target.x, target.y)
        node = m.getintersection(m.x, m.y)
        tgt = node.direction_to_goal

        if tgt is not None:
            print(f"Navigating to intersection at ({target.x}, {target.y})")
            state.goal = (target.x, target.y)
            state.temporary_goal = None
            state.goal_set_by_explore = False
            state.mode = Mode.DRIVE_TO_GOAL
            return Action(ActionType.NONE)
        else:
            print("Direct path to target is blocked. Performing directed exploration.")

    # Directed explore toward unexplored intersections that may hold prize distances
    unexplored = m.unexplored_intersections_for_prize(prize_id)
    if unexplored:
        target = min(unexplored, key=lambda inter: math.hypot(inter.x - m.x, inter.y - m.y))
        print(f"Directed exploring toward unexplored intersection at ({target.x}, {target.y})")

        m.cleargoal()
        m.setgoal(target.x, target.y)

        state.goal = (target.x, target.y)
        state.temporary_goal = None
        state.goal_set_by_explore = True
        state.mode = Mode.DRIVE_TO_GOAL
        return Action(ActionType.NONE)

    print("No known or unexplored intersections remain — standing by.")
    return Action(ActionType.NONE)


def execute(action: Action, state: State):
    b, m, a_sensor = state.behaviors, state.map, state.anglesensor

    if action.type == ActionType.QUIT:
        print("Quitting...")
        return False

    if action.type == ActionType.START_EXPLORATION:
        state.mode = Mode.EXPLORING
        print("Starting autonomous exploration.")
        return True

    if action.type == ActionType.SET_GOAL:
        gx, gy = action.params['gx'], action.params['gy']
        print(f"Setting goal to ({gx},{gy})…")
        state.goal = (gx, gy)
        state.goal_set_by_explore = False
        state.mode = Mode.DRIVE_TO_GOAL
        print ("mode switched")
        m.cleargoal()
        print ("clear goal")
        # only seed Dijkstra if goal is already known
        if (gx, gy) in m.intersections:
            print("int exists")

            m.setgoal(gx, gy)
            print("goal set")
        else:
            print(f"Goal {state.goal} not yet on map → directed exploring")
        return True

    if action.type == ActionType.TURN:
        steps = b.turn(action.params['direction'], a_sensor)[0]
        print(f"TURN: {action.params['direction']} → {steps*45}°")
        m.updateheading(steps)
        m.auto_connect_opposing_unexplored()

        inter = m.getintersection(m.x, m.y)
        if inter.streets[m.heading] == STATUS.NONEXISTENT:
            fb = closest_viable_heading(m.heading, inter)
            if fb is not None:
                print(f"Nonexistent, switching to {fb}")
                m.updateheading((fb - m.heading)%8)
            else:
                print("No viable heading. Standing by.")

        # mark skipped and new edges
        for s in m.skipped_directions(steps):
            m.setstreet(m.x, m.y, s, STATUS.NONEXISTENT)
        if m.getstreet(m.x, m.y, m.heading) not in (
            STATUS.CONNECTED, STATUS.DEADEND, STATUS.NONEXISTENT
        ):
            m.setstreet(m.x, m.y, m.heading, STATUS.UNEXPLORED)
        for off in (-1,1):
            adj = (m.heading+off)%8
            if m.getstreet(m.x, m.y, adj) == STATUS.UNKNOWN:
                m.setstreet(m.x, m.y, adj, STATUS.NONEXISTENT)
        return True

    if action.type == ActionType.STRAIGHT:
        inter = m.getintersection(m.x, m.y)
        # avoid driving into blockages/nonexistent
        if m.getstreet(m.x, m.y, m.heading) == STATUS.NONEXISTENT:
            fb = closest_viable_heading(m.heading, inter)
            if fb is not None:
                print(f"Nonexistent, turning toward {fb}")
                return execute(
                    Action(ActionType.TURN,
                           direction='LEFT' if ((fb-m.heading)%8)<=4 else 'RIGHT'),
                    state
                )
            print("Everywhere nonexistent, standing by.")

        if inter.blocked[m.heading]:
            if state.mode == Mode.MANUAL:
                print("Blocked street, please wait until unblocked or choose a different command.")
                return Action(ActionType.NONE)
            else:
                fb = closest_viable_heading(m.heading, inter)
                if fb is not None:
                    print(f"Forward blocked; turning toward {fb}")
                    return execute(
                        Action(ActionType.TURN,
                            direction='LEFT' if ((fb - m.heading) % 8) <= 4 else 'RIGHT'),
                        state
                    )
                print("Nowhere unblocked — performing U-turn.")
                return execute(Action(ActionType.TURN, direction='LEFT'), state)

        res = b.line_following()
        if res == 'intersection':
            px, py, ph = m.x, m.y, m.heading
            m.updatemove()
            m.auto_connect_opposing_unexplored()
            m.setstreet(px, py, ph, STATUS.CONNECTED)
            m.setstreet(m.x, m.y, (ph + 4) % 8, STATUS.CONNECTED)
            for off in (-1,1):
                d = (ph + off) % 8
                if m.getstreet(px, py, d) == STATUS.UNKNOWN:
                    m.setstreet(px, py, d, STATUS.NONEXISTENT)
                od = ((ph + 4) + off) % 8
                if m.getstreet(m.x, m.y, od) == STATUS.UNKNOWN:
                    m.setstreet(m.x, m.y, od, STATUS.NONEXISTENT)
            pull = b.pull_forward()
            if pull == 'street_ahead':
                m.setstreet(m.x, m.y, m.heading, STATUS.UNKNOWN)
            else:
                m.setstreet(m.x, m.y, m.heading, STATUS.NONEXISTENT)
            try:
                nfc_sensor = state.behaviors.nfc_sensor if hasattr(state.behaviors, 'nfc_sensor') else None
                if nfc_sensor:
                    nfc_id = nfc_sensor.read()
                    if nfc_id is not None:
                        print(f"NFC read at intersection: {nfc_id}")

                        if hasattr(state.shared, 'inter_prize_distance_dict'):
                            prize_dict = state.shared.inter_prize_distance_dict
                            print(f"Current prize dictionary keys: {list(prize_dict.keys())}")
                            
                            if prize_dict:
                                m.update_intersection_prize_distances(nfc_id, prize_dict)
                                inter = m.getintersection(m.x, m.y)
                                print(f"Prize distances now stored at intersection ({inter.x}, {inter.y}): {inter.prize_distances}")

                                if state.mode == Mode.FETCH:
                                    prize_id = state.fetch_prize_id
                                    current_distance = inter.get_prize_distance(prize_id)
                                    if current_distance is not None:
                                        print(f"[FETCH - IMMEDIATE] Prize distance at ({inter.x}, {inter.y}) is {current_distance}")

                                        if state.last_prize_distance is not None:
                                            if current_distance > state.last_prize_distance:
                                                print("[FETCH - IMMEDIATE] Distance increased! Consider reversing direction soon.")
                                        state.last_prize_distance = current_distance
                                        state.last_nfc_intersection = (inter.x, inter.y) 
                            else:
                                print("[INFO] Prize dictionary is currently empty.")
                        else:
                            print("[WARN] Prize dictionary not yet received from ROS.")
            except Exception as e:
                print(f"[WARN] NFC/prize update failed: {e}")

        elif res == 'end':
            print("End encountered.")
            steps = b.turn('LEFT', a_sensor)[0]
            m.updateheading(steps)
            m.setstreet(m.x, m.y, (m.heading + 4) % 8, STATUS.DEADEND)
            b.line_following()
            pull = b.pull_forward()
            if pull == 'street_ahead':
                m.setstreet(m.x, m.y, m.heading, STATUS.UNKNOWN)
            else:
                m.setstreet(m.x, m.y, m.heading, STATUS.NONEXISTENT)

        elif res == 'uturn_blockage':
            print("Blocked street encountered, U-turn performed.")
            steps = b.turn('LEFT', a_sensor)[0]
            m.updateheading(steps)
            m.setstreet(m.x, m.y, (m.heading + 4) % 8, STATUS.UNEXPLORED)
            inter = m.getintersection(m.x, m.y)
            inter.set_blocked((m.heading + 4) % 8, True)
            b.line_following()
            pull = b.pull_forward()
            if pull == 'street_ahead':
                m.setstreet(m.x, m.y, m.heading, STATUS.UNKNOWN)
            else:
                m.setstreet(m.x, m.y, m.heading, STATUS.NONEXISTENT)

        return True

    if action.type == ActionType.NONE:
        return True

    return True


def brain(behaviors, amap, anglesensor):
    state = State(behaviors, amap, anglesensor)
    print("Robot brain initialized.\n")
    running = True
    while running:
        act = decide(state)
        running = execute(act, state)
    print("Brain loop exited.")
