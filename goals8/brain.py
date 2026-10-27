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


class Mode(Enum):
    MANUAL = auto()
    EXPLORING = auto()
    DRIVE_TO_GOAL = auto()


class ActionType(Enum):
    TURN = auto()
    STRAIGHT = auto()
    SET_GOAL = auto()
    START_EXPLORATION = auto()
    QUIT = auto()
    NONE = auto()  # no-op


class Action:
    def __init__(self, type_, **kwargs):
        self.type = type_
        self.params = kwargs


class State:
    def __init__(self, behaviors, amap, anglesensor):
        self.behaviors = behaviors
        self.map = amap
        self.anglesensor = anglesensor
        self.mode = Mode.MANUAL
        self.goal = None
        self.last_position = None
        self.last_spin_heading = None


def closest_viable_heading(current_heading, inter):
    """Return the heading closest to current_heading that is neither NONEXISTENT nor blocked."""
    candidates = [
        h for h, st in enumerate(inter.streets)
        if st not in (STATUS.NONEXISTENT,) and not inter.blocked[h]
    ]
    if not candidates:
        return None
    return min(candidates,
               key=lambda h: min((h-current_heading)%8, (current_heading-h)%8))


def decide(state: State) -> Action:
    """Main decision dispatcher: MANUAL / EXPLORING / DRIVE_TO_GOAL (or DIRECTED)."""
    m = state.map

    # always show current map
    m.show()

    # debug listing
    if state.mode == Mode.EXPLORING:
        unfinished = [
            coords for coords, node in m.intersections.items()
            if any(st in (STATUS.UNKNOWN, STATUS.UNEXPLORED) and not node.blocked[h]
                   for h, st in enumerate(node.streets))
        ]
        print(f"Unfinished intersections: {unfinished}")

    if state.mode == Mode.EXPLORING:
        return _decide_explore(state)

    if state.mode == Mode.DRIVE_TO_GOAL:
        # if goal on map, classical Dijkstra‐drive
        if state.goal in m.intersections:
            if (m.x, m.y) == state.goal:
                print("Reached goal intersection; switching to EXPLORING")
                state.mode = Mode.EXPLORING
                return Action(ActionType.NONE)

            node = m.getintersection(m.x, m.y)
            tgt = node.direction_to_goal
            if tgt is None:
                print("No Dijkstra path—falling back to EXPLORE")
                state.mode = Mode.EXPLORING
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

    # complete?
    missing = [
        (x,y) for (x,y), node in m.intersections.items()
        for h, st in enumerate(node.streets)
        if st in (STATUS.UNKNOWN, STATUS.UNEXPLORED) and not node.blocked[h]
    ]
    if not missing:
        print("Exploration complete.")
        state.mode = Mode.MANUAL
        return Action(ActionType.NONE)

    # reset spin on arrival
    if state.last_position != (m.x, m.y):
        state.last_position = (m.x, m.y)
        state.last_spin_heading = None

    # spin to get rid of UNKNOWN
    if STATUS.UNKNOWN in inter.streets:
        print(f"Spinning to reveal UNKNOWN at {(m.x,m.y)}…")
        state.last_spin_heading = m.heading
        return Action(ActionType.TURN, direction='LEFT')

    # pick UNEXPLORED first, then CONNECTED, skipping blocked
    candidates = [
        h for h, st in enumerate(inter.streets)
        if st == STATUS.UNEXPLORED and not inter.blocked[h]
    ] or [
        h for h, st in enumerate(inter.streets)
        if st == STATUS.CONNECTED and not inter.blocked[h]
    ]

    if candidates:
        cur = m.heading
        target = min(candidates, key=lambda h: min((h-cur)%8, (cur-h)%8))
        diff = (target - cur) % 8
        if diff:
            return Action(ActionType.TURN,
                          direction='LEFT' if diff<=4 else 'RIGHT')
        return Action(ActionType.STRAIGHT)

    print("No unblocked candidates—standing by.")
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

    # desired 45° bucket toward goal
    dx, dy = gx - m.x, gy - m.y
    desired = min(range(8),
                  key=lambda h: abs(
                      math.atan2(HEADING_DELTAS[h][1], HEADING_DELTAS[h][0])
                      - math.atan2(dy, dx)
                  ))

    local = [
        h for h in range(8)
        if inter.streets[h] in (STATUS.UNEXPLORED, STATUS.UNKNOWN)
        and not inter.blocked[h]
    ]
    if desired in local:
        diff = (desired - m.heading) % 8
        if diff:
            return Action(ActionType.TURN,
                          direction='LEFT' if diff<=4 else 'RIGHT')
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

    # pick the one closest to (gx,gy)
    target = min(interesting, key=lambda p: math.hypot(p[0]-gx, p[1]-gy))
    print(f"Directed explore: heading off to {target}")
    m.cleargoal()
    m.setgoal(*target)

    # now behave like DRIVE_TO_GOAL
    node = m.getintersection(m.x, m.y)
    tgt = node.direction_to_goal
    if m.heading != tgt:
        diff = (tgt - m.heading) % 8
        return Action(ActionType.TURN,
                      direction='LEFT' if diff<=4 else 'RIGHT')
    return Action(ActionType.STRAIGHT)


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

        inter = m.getintersection(m.x, m.y)
        if inter.streets[m.heading] == STATUS.NONEXISTENT or inter.blocked[m.heading]:
            fb = closest_viable_heading(m.heading, inter)
            if fb is not None:
                print(f"Blocked turn; switching to {fb}")
                m.updateheading((fb - m.heading)%8)
            else:
                print("No viable heading after blockage!")

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
        if inter.blocked[m.heading] or m.getstreet(m.x, m.y, m.heading) == STATUS.NONEXISTENT:
            fb = closest_viable_heading(m.heading, inter)
            if fb is not None:
                print(f"Forward blocked; turning toward {fb}")
                return execute(
                    Action(ActionType.TURN,
                           direction='LEFT' if ((fb-m.heading)%8)<=4 else 'RIGHT'),
                    state
                )
            print("Nowhere unblocked—standing by.")
            return True

        res = b.line_following()
        if res == 'intersection':
            px, py, ph = m.x, m.y, m.heading
            m.updatemove()
            m.setstreet(px, py, ph, STATUS.CONNECTED)
            m.setstreet(m.x, m.y, (ph+4)%8, STATUS.CONNECTED)
            for off in (-1,1):
                d = (ph+off)%8
                if m.getstreet(px, py, d) == STATUS.UNKNOWN:
                    m.setstreet(px, py, d, STATUS.NONEXISTENT)
                od = ((ph+4)+off)%8
                if m.getstreet(m.x, m.y, od) == STATUS.UNKNOWN:
                    m.setstreet(m.x, m.y, od, STATUS.NONEXISTENT)
            pull = b.pull_forward()
            if pull == 'street_ahead':
                m.setstreet(m.x, m.y, m.heading, STATUS.UNKNOWN)
            else:
                m.setstreet(m.x, m.y, m.heading, STATUS.NONEXISTENT)

        elif res == 'end':
            print("End encountered.")
            steps = b.turn('LEFT', a_sensor)[0]
            m.updateheading(steps)
            m.setstreet(m.x, m.y, (m.heading+4)%8, STATUS.DEADEND)
            b.line_following()
            pull = b.pull_forward()
            if pull == 'street_ahead':
                m.setstreet(m.x, m.y, m.heading, STATUS.UNKNOWN)
            else:
                m.setstreet(m.x, m.y, m.heading, STATUS.NONEXISTENT)

        elif res == 'uturn_blockage':
            print("Blocked street encountered, U-turn performed.")
            m.updatemove()
            m.updateheading((m.heading+4)%8)
            m.setstreet(m.x, m.y, (m.heading+4)%8, STATUS.UNEXPLORED)
            inter = m.getintersection(m.x, m.y)
            inter.set_blocked((m.heading+4)%8, True)
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
