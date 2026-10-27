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


def _manual_action() -> Action:
    cmd = input("Enter [L]eft, [R]ight, [S]traight, [G]oto, [E]xplore, [Q]uit → ").strip().upper()
    if cmd == 'L': return Action(ActionType.TURN, direction='LEFT')
    if cmd == 'R': return Action(ActionType.TURN, direction='RIGHT')
    if cmd == 'S': return Action(ActionType.STRAIGHT)
    if cmd == 'G':
        try:
            gx = int(input("Goal X: "))
            gy = int(input("Goal Y: "))
            return Action(ActionType.SET_GOAL, gx=gx, gy=gy)
        except ValueError:
            print("Invalid coordinates.")
            return _manual_action()
    if cmd == 'E': return Action(ActionType.START_EXPLORATION)
    if cmd == 'Q': return Action(ActionType.QUIT)
    print("Invalid command.")
    return _manual_action()


def decide(state: State) -> Action:
    # display map
    state.map.show()
    # debug: print unfinished intersection coords
    unfinished = [coords for coords, node in state.map.intersections.items()
                  if any(st in (STATUS.UNKNOWN, STATUS.UNEXPLORED) for st in node.streets)]
    print(f"Unfinished intersections: {unfinished}")

    if state.mode == Mode.EXPLORING:
        current_pos = (state.map.x, state.map.y)
        inter = state.map.getintersection(*current_pos)

        # Check if global exploration is done BEFORE doing anything else
        nodes = [((x,y), sum(1 for st in node.streets if st in (STATUS.UNKNOWN, STATUS.UNEXPLORED)))
                for (x,y), node in state.map.intersections.items()]
        nodes = [(coords, count) for coords, count in nodes if count > 0]
        if not nodes:
            print("Exploration complete.")
            state.mode = Mode.MANUAL
            return _manual_action()

        # If we arrived at a new intersection, reset spin state
        if state.last_position != current_pos:
            state.last_position = current_pos
            state.last_spin_heading = None

        # If there are UNKNOWN streets, keep spinning
        if STATUS.UNKNOWN in inter.streets:
            print(f"Spinning at {current_pos} to reveal unknown directions...")
            state.last_spin_heading = state.map.heading  # Remember where we were
            return Action(ActionType.TURN, direction='LEFT')

        # All streets known, pick a direction to move
        unexplored = [h for h, st in enumerate(inter.streets) if st == STATUS.UNEXPLORED]
        connected = [h for h, st in enumerate(inter.streets) if st == STATUS.CONNECTED]

        # Filter streets to prevent robot's heading pointing towards a nonexistent street
        filtered_unexplored = [h for h in unexplored if inter.streets[h] != STATUS.NONEXISTENT]
        filtered_connected = [h for h in connected if inter.streets[h] != STATUS.NONEXISTENT]

        candidates = filtered_unexplored or filtered_connected  # Prefer unexplored, fallback to connected
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
        
        '''max_missing = max(count for _,count in nodes)
        candidates = [coords for coords,count in nodes if count==max_missing]
        # tie-break by proximity
        curx, cury = state.map.x, state.map.y
        target = min(candidates, key=lambda c: abs(c[0]-curx)+abs(c[1]-cury))
        print("\nTarget is: ", target[0], target[1])
        return Action(ActionType.SET_GOAL, gx=target[0], gy=target[1])'''

    if state.mode == Mode.DRIVE_TO_GOAL:
        if (state.map.x, state.map.y) == state.goal:
            state.mode = Mode.EXPLORING
            print("Reached exploration target.")
            return decide(state)
        node = state.map.getintersection(state.map.x, state.map.y)
        tgt = node.direction_to_goal
        if tgt is None:
            state.mode = Mode.EXPLORING
            return decide(state)
        if state.map.heading != tgt:
            diff = (tgt - state.map.heading) % 8
            dir_ = 'LEFT' if diff <= 4 else 'RIGHT'
            return Action(ActionType.TURN, direction=dir_)
        return Action(ActionType.STRAIGHT)
    
    return _manual_action()


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
        m.cleargoal(); m.setgoal(gx, gy)
        return True
    
    if action.type == ActionType.TURN:
        steps = b.turn(action.params['direction'], a_sensor)[0]
        print(f"TURN: direction={action.params['direction']}, steps={steps}, old_heading={m.heading}")
        m.updateheading(steps)
        print(f"New heading={m.heading}")
        # mark skipped directions
        for s in m.skipped_directions(steps):
            current_status = m.getstreet(m.x, m.y, s)
            if current_status == STATUS.UNKNOWN:
                m.setstreet(m.x, m.y, s, STATUS.NONEXISTENT)
        # mark the new heading edge as UNEXPLORED if not already CONNECTED, DEADEND, or NONEXISTENT
        if m.getstreet(m.x, m.y, m.heading) not in [STATUS.CONNECTED, STATUS.DEADEND, STATUS.NONEXISTENT]:
            m.setstreet(m.x, m.y, m.heading, STATUS.UNEXPLORED)
        # Assume no 45 degree adjacent streets exist/eliminate inside-corner adjacents
        for offset in (-1, 1):
            adj_heading = (m.heading + offset) % 8
            if m.getstreet(m.x, m.y, adj_heading) == STATUS.UNKNOWN:
                m.setstreet(m.x, m.y, adj_heading, STATUS.NONEXISTENT)
        return True

    if action.type == ActionType.STRAIGHT:
        if m.getstreet(m.x, m.y, m.heading) == STATUS.NONEXISTENT:
            print("ERROR - NONEXISTENT STREET AHEAD")
        res = b.line_following()
        if res == 'intersection':
            px, py, ph = m.x, m.y, m.heading
            m.updatemove()
            m.setstreet(px, py, ph, STATUS.CONNECTED)
            m.setstreet(m.x, m.y, (ph+4)%8, STATUS.CONNECTED)
            # Assume no 45 degree adjacent streets exist/eliminate inside-corner adjacents
            for offset in (-1, 1):
                diagonal = (ph + offset) % 8
                if m.getstreet(px, py, diagonal) == STATUS.UNKNOWN:
                    m.setstreet(px, py, diagonal, STATUS.NONEXISTENT)
                # Also mark the symmetrical 45° direction from the new location
                opposite_diagonal = ((ph + 4) + offset) % 8
                if m.getstreet(m.x, m.y, opposite_diagonal) == STATUS.UNKNOWN:
                    m.setstreet(m.x, m.y, opposite_diagonal, STATUS.NONEXISTENT)

            # Pull forward with street detector
            pull = b.pull_forward()
            if pull == 'street_ahead':
                m.setstreet(m.x, m.y, m.heading, STATUS.UNKNOWN)
            else:
                prev = m.getstreet(m.x, m.y, m.heading)
                m.setstreet(m.x, m.y, m.heading, STATUS.NONEXISTENT)
        elif res == 'end':
            print("End encountered.")
            steps = b.turn('LEFT', a_sensor)[0]
            m.updateheading(steps)
            prev = m.getstreet(m.x, m.y, (m.heading+4)%8)
            m.setstreet(m.x, m.y, (m.heading+4)%8, STATUS.DEADEND)
            b.line_following()
            pull = b.pull_forward()
            if pull == 'street_ahead':
                m.setstreet(m.x, m.y, m.heading, STATUS.UNKNOWN)
            else:
                prev = m.getstreet(m.x, m.y, m.heading)
                m.setstreet(m.x, m.y, m.heading, STATUS.NONEXISTENT)
        return True
    return True


def brain(behaviors, amap, anglesensor):
    state = State(behaviors, amap, anglesensor)
    print("Robot brain initialized.\n")
    running = True
    while running:
        act = decide(state)
        state.map.show()
        running = execute(act, state)
        state.map.show()
    print("Brain loop exited.")
