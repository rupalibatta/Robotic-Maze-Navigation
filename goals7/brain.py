#!/usr/bin/env python3
import pickle
import traceback
import time
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
    NONE = auto()  # Added for no-op actions


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
    """Return the heading (0–7) closest to current_heading that is neither NONEXISTENT nor blocked."""
    candidates = [h for h, st in enumerate(inter.streets)
                  if st not in (STATUS.NONEXISTENT,) and not inter.blocked[h]]
    if not candidates:
        return None
    # minimize angular distance
    return min(candidates, key=lambda h: min((h-current_heading)%8, (current_heading-h)%8))


def decide(state: State) -> Action:
    # Always show map
    state.map.show()
    # debug: print unfinished intersection coords
    if state.mode == Mode.EXPLORING:
        unfinished = [coords for coords, node in state.map.intersections.items()
                      if any(st in (STATUS.UNKNOWN, STATUS.UNEXPLORED) for st in node.streets)]
        print(f"Unfinished intersections: {unfinished}")

    if state.mode == Mode.EXPLORING:
        current_pos = (state.map.x, state.map.y)
        inter = state.map.getintersection(*current_pos)

        # Check if global exploration is done
        nodes = []
        for (x, y), node in state.map.intersections.items():
            for h, status in enumerate(node.streets):
                if status in (STATUS.UNKNOWN, STATUS.UNEXPLORED) and not node.blocked[h]:
                    nodes.append((x, y))
        
        if not nodes:
            print("Exploration complete.")
            state.mode = Mode.MANUAL
            return Action(ActionType.NONE)

        # reset spin after arriving
        if state.last_position != current_pos:
            state.last_position = current_pos
            state.last_spin_heading = None

        # spin if unknown
        if STATUS.UNKNOWN in inter.streets:
            print(f"Spinning at {current_pos} to reveal unknown directions...")
            state.last_spin_heading = state.map.heading
            return Action(ActionType.TURN, direction='LEFT')

        # choose next heading
        unexplored = [h for h, st in enumerate(inter.streets) if st == STATUS.UNEXPLORED]
        connected = [h for h, st in enumerate(inter.streets) if st == STATUS.CONNECTED]

        # Filter streets to prevent robot's heading pointing towards a nonexistent street
        filtered_unexplored = [h for h in unexplored if inter.streets[h] != STATUS.NONEXISTENT and not inter.blocked[h]]
        filtered_connected = [h for h in connected if inter.streets[h] != STATUS.NONEXISTENT and not inter.blocked[h]]

        candidates = filtered_unexplored or filtered_connected
        if candidates:
            cur = state.map.heading
            target = min(candidates, key=lambda h: min((h-cur)%8, (cur-h)%8))
            diff = (target - cur) % 8
            if diff != 0:
                direction = 'LEFT' if diff <= 4 else 'RIGHT'
                return Action(ActionType.TURN, direction=direction)
            return Action(ActionType.STRAIGHT)

        print("No move candidates found. Standing by.")
        return Action(ActionType.NONE)

    if state.mode == Mode.DRIVE_TO_GOAL:
        if (state.map.x, state.map.y) == state.goal:
            print("Reached exploration target.")
            state.mode = Mode.EXPLORING
            return Action(ActionType.NONE)
        node = state.map.getintersection(state.map.x, state.map.y)
        tgt = node.direction_to_goal
        if tgt is None:
            state.mode = Mode.EXPLORING
            return Action(ActionType.NONE)
        if state.map.heading != tgt:
            inter = state.map.getintersection(state.map.x, state.map.y)
            # if the direct goal heading is blocked or nonexistent, pick a viable instead
            if inter.streets[tgt] == STATUS.NONEXISTENT or inter.blocked[tgt]:
                new_tgt = closest_viable_heading(state.map.heading, inter)
                if new_tgt is not None:
                    tgt = new_tgt
                else:
                    # no way forward; fallback to exploration or wait
                    return Action(ActionType.NONE)
            diff = (tgt - state.map.heading) % 8
            dir_ = 'LEFT' if diff <= 4 else 'RIGHT'
            return Action(ActionType.TURN, direction=dir_)
        return Action(ActionType.STRAIGHT)
    # Manual mode defaults to no action (UI thread drives manual commands)
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
        print(f"Setting goal to ({gx},{gy})...")
        state.goal = (gx, gy)
        state.mode = Mode.DRIVE_TO_GOAL
        m.cleargoal()
        m.setgoal(gx, gy)
        return True
    if action.type == ActionType.TURN:
        steps = b.turn(action.params['direction'], a_sensor)[0]
        print(f"TURN: direction={action.params['direction']}, steps={steps}, old_heading={m.heading}")
        m.updateheading(steps)

        inter = m.getintersection(m.x, m.y)
        # if we just turned into a blocked/nonexistent heading:
        if inter.streets[m.heading] == STATUS.NONEXISTENT or inter.blocked[m.heading]:
            fallback = closest_viable_heading(m.heading, inter)
            if fallback is not None:
                print(f"Heading {m.heading} blocked; switching to {fallback}")
                m.updateheading((fallback - m.heading) % 8)
            else:
                print("No viable turn after blockage!")

        print(f"New heading={m.heading}")
        for s in m.skipped_directions(steps):
            m.setstreet(m.x, m.y, s, STATUS.NONEXISTENT)
        if m.getstreet(m.x, m.y, m.heading) not in [STATUS.CONNECTED, STATUS.DEADEND, STATUS.NONEXISTENT]:
            m.setstreet(m.x, m.y, m.heading, STATUS.UNEXPLORED)
        for offset in (-1, 1):
            adj = (m.heading + offset) % 8
            if m.getstreet(m.x, m.y, adj) == STATUS.UNKNOWN:
                m.setstreet(m.x, m.y, adj, STATUS.NONEXISTENT)
        return True
    if action.type == ActionType.STRAIGHT:
        inter = m.getintersection(m.x, m.y)
        if inter.blocked[m.heading] or m.getstreet(m.x, m.y, m.heading) == STATUS.NONEXISTENT:
            fallback = closest_viable_heading(m.heading, inter)
            if fallback is not None:
                print(f"Forward blocked; turning toward {fallback} instead")
                return execute(Action(ActionType.TURN, direction='LEFT' if ((fallback-m.heading)%8)<=4 else 'RIGHT'), state)
            else:
                print("Nowhere unblocked to go.")
                return True

        res = b.line_following()
        if res == 'intersection':
            px, py, ph = m.x, m.y, m.heading
            m.updatemove()
            m.setstreet(px, py, ph, STATUS.CONNECTED)
            m.setstreet(m.x, m.y, (ph+4)%8, STATUS.CONNECTED)
            for offset in (-1, 1):
                d = (ph + offset) % 8
                if m.getstreet(px, py, d) == STATUS.UNKNOWN:
                    m.setstreet(px, py, d, STATUS.NONEXISTENT)
                od = ((ph+4)+offset)%8
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
        return True
    if action.type == ActionType.NONE:
        # no-op
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
