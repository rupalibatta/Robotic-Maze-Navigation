#!/usr/bin/env python3
import threading
import time
import pickle
import traceback
import pigpio
from drivesystem import DriveSystem
from linesensor import LineSensor
from proximitysensor import ProximitySensor  # ultrasound sensors
from anglesensor import ADC, AngleSensor
from behaviors import Behaviors
from brain import decide, execute, State, Action, ActionType, Mode
from map import Map, STATUS   # import STATUS to mark nonexistent streets

class SharedState:
    """Shared UI command state (thread-safe)."""
    def __init__(self):
        self.lock = threading.Lock()
        self.command = None
        self.params = []


def ui_thread(shared: SharedState):
    """Daemon thread: reads user commands."""
    print("UI thread started. Commands: explore, goal X Y, pause, step, resume, left, right, straight, save FILENAME, load FILENAME, pose X Y H, show, quit")
    while True:
        raw = input(">> ").strip().split()
        if not raw:
            continue
        cmd, *params = raw
        with shared.lock:
            shared.command = cmd.lower()
            shared.params = params
        if cmd.lower() == 'quit':
            break


def robot_thread(shared: SharedState, behaviors, amap, anglesensor, drive, line_sensor, prox_sensor):
    """Main robot/brain thread."""
    # Calibrate magnetometer at startup
    anglesensor.calibrate_spin(drive, line_sensor)

    state = State(behaviors, amap, anglesensor)
    print("Robot brain initialized. Waiting for commands...")

    state.paused = False
    running = True

    while running:
        # Harvest UI command
        with shared.lock:
            cmd = shared.command
            params = list(shared.params)
            shared.command = None
            shared.params = []

        state.map.show()
        # Process UI commands
        if cmd == 'explore':
            prev_mode = 'explore'
            state.mode = Mode.EXPLORING
            print("UI → start exploring")
        elif cmd == 'goal' and len(params) >= 2:
            prev_mode = 'goal'
            try:
                gx, gy = int(params[0]), int(params[1])
                state.goal = (gx, gy)
                state.mode = Mode.DRIVE_TO_GOAL
                state.map.cleargoal()
                state.map.setgoal(gx, gy)
                print(f"UI → drive to goal {state.goal}")
            except ValueError:
                print("Invalid goal coordinates")
        elif cmd == 'pause':
            state.paused = True
            print("UI → pause")
        elif cmd == 'resume':
            if prev_mode == 'explore':
                state.mode = Mode.EXPLORING
            if prev_mode == 'goal':
                state.mode = Mode.DRIVE_TO_GOAL
            state.paused = False
            print("UI → resume")
        elif cmd == 'step':
            state.paused = False
            print("UI → single step")
        elif cmd == 'left':
            running = execute(Action(ActionType.TURN, direction='LEFT'), state)
        elif cmd == 'right':
            running = execute(Action(ActionType.TURN, direction='RIGHT'), state)
        elif cmd == 'straight':
            running = execute(Action(ActionType.STRAIGHT), state)
        elif cmd == 'save' and params:
            fname = params[0]
            try:
                with open(fname, 'wb') as f:
                    pickle.dump(state.map, f)
                print(f"Map saved to {fname}")
            except Exception as e:
                print("Failed to save map:", e)
        elif cmd == 'load' and params:
            fname = params[0]
            try:
                with open(fname, 'rb') as f:
                    state.map = pickle.load(f)
                print(f"Map loaded from {fname}")
                for inter in state.map.intersections.values():
                    inter.clear_blocked()
                state.map.show()
            except Exception as e:
                print("Failed to load map:", e)
        elif cmd == 'pose' and len(params) >= 3:
            try:
                x, y, h = map(int, params[:3])
                state.map.x, state.map.y, state.map.heading = x, y, h
                print(f"Pose set to {(x,y,h)}")
                state.map.show()
            except ValueError:
                print("Invalid pose parameters")
        elif cmd == 'show':
            state.map.show()
        elif cmd == 'quit':
            print("UI → quit requested")
            break

        # If manual turn/straight, we already executed above; skip planning.
        if cmd in ('left', 'right', 'straight'):
            state.map.show()
            pass
        else:
            # If not in auto-mode, idle
            if state.mode not in (Mode.EXPLORING, Mode.DRIVE_TO_GOAL):
                time.sleep(0.1)
                continue
            # Handle pause/step
            if state.paused and cmd != 'step':
                time.sleep(0.1)
                continue
            if cmd == 'step':
                state.paused = True

            # Plan & execute auto action
            action = decide(state)
            # Block ahead check: avoid straight into blocked
            if action.type == ActionType.STRAIGHT:
                inter = state.map.getintersection(state.map.x, state.map.y)
                if inter.blocked[state.map.heading]:
                    print(f"Blocked ahead at heading {state.map.heading}, skipping forward.")
                    current_status = state.map.getstreet(state.map.x, state.map.y, state.map.heading)
                    state.map.setstreet(state.map.x, state.map.y, state.map.heading, STATUS.NONEXISTENT)
                    state.map.show()
                    continue
            running = execute(action, state)

        # After any action, check-ahead and mark blockage
        try:
            dists = prox_sensor.read()
            front = dists[1]
            thresh = 0.6 if (state.map.heading % 2) else 0.4
            is_blocked = front < thresh
            inter = state.map.getintersection(state.map.x, state.map.y)
            inter.set_blocked(state.map.heading, is_blocked)
            if is_blocked:
                print(f"Blocked ahead at heading {state.map.heading}, dist={front:.2f}m")
        except Exception:
            pass

    print("Robot thread exiting.")


def main():
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connect to pigpio daemon")
        return

    # Prompt for starting pose
    coord_input = input("Please enter the robot's starting coordinate (x y): ").strip().split()
    try:
        x0, y0 = int(coord_input[0]), int(coord_input[1])
    except:
        print("Invalid coordinate input, defaulting to (0,0)")
        x0, y0 = 0, 0
    try:
        h0 = int(input("Please enter the robot's starting heading [0-7]: ")) % 8
    except:
        print("Invalid heading, defaulting to 0")
        h0 = 0
    on_int = input("Is the robot starting on an intersection? (y/n): ").strip().lower()

    # Initialize hardware & logic
    drive = DriveSystem(io)
    line_sensor = LineSensor(io)
    prox_sensor = ProximitySensor(io)
    adc = ADC(io)
    anglesensor = AngleSensor(adc)
    behaviors = Behaviors(drive, line_sensor)
    amap = Map()

    # Set initial pose
    amap.x, amap.y, amap.heading = x0, y0, h0

    # If not on intersection: drive to first
    if on_int != 'y':
        print("Driving until first intersection...")
        res = behaviors.line_following()
        if res == 'intersection':
            amap.updatemove()
            amap.getintersection(amap.x, amap.y)
            print(f"Arrived at intersection {(amap.x, amap.y)}")
        else:
            print("Warning: did not detect intersection; starting map empty.")
    else:
        amap.getintersection(x0, y0)

    # Shared state & threads
    shared = SharedState()
    threading.Thread(target=ui_thread, args=(shared,), daemon=True).start()
    t_robot = threading.Thread(target=robot_thread,
        args=(shared, behaviors, amap, anglesensor, drive, line_sensor, prox_sensor))
    t_robot.start()

    try:
        t_robot.join()
    except KeyboardInterrupt:
        print("KeyboardInterrupt: stopping robot")
        drive.stop()
    finally:
        drive.stop()
        io.stop()

if __name__ == '__main__':
    main()
