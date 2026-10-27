#!/usr/bin/env python3
import threading
import time
import pickle
import traceback
import ctypes
import pigpio
from ros import runros
from drivesystem import DriveSystem
from linesensor import LineSensor
from proximitysensor import ProximitySensor
from anglesensor import ADC, AngleSensor
from behaviors import Behaviors
from nfc import NFCSensor
from brain import decide, execute, State, Action, ActionType, Mode
from map import Map, STATUS

class SharedState:
    """Shared UI & ROS command state (thread-safe)."""
    def __init__(self):
        self.lock = threading.Lock()
        self.command = None
        self.params = []
        # pose for ROS publishing
        self.robotx = 0
        self.roboty = 0
        self.robotheading = 0

    def acquire(self):
        return self.lock.acquire()

    def release(self):
        self.lock.release()


def ui_thread(shared: SharedState):
    """Daemon thread: reads user commands."""
    print("UI thread started. Commands: explore, fetch X, goal X Y, pause, step, resume, left, right, straight, save FILENAME, load FILENAME, pose X Y H, show, clear, quit")
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
    # 1) Calibrate magnetometer
    anglesensor.calibrate_spin(drive, line_sensor)
    nfc_sensor = NFCSensor()
    behaviors.nfc_sensor = nfc_sensor

    state = State(behaviors, amap, anglesensor, shared)
    print("Robot brain initialized. Waiting for commands...")
    state.paused = False
    running = True
    prev_mode = None
    last_blocked_status = {}

    last_mode = state.mode

    while running:
        # update shared pose for ROS
        if shared.acquire():
            shared.robotx = amap.x
            shared.roboty = amap.y
            shared.robotheading = amap.heading
            shared.release()

        # harvest command
        with shared.lock:
            cmd = shared.command
            params = list(shared.params)
            shared.command = None
            shared.params = []

        # display map
        amap.show()
        if state.mode != last_mode:
            print(f"Mode changed: {state.mode.name}")
            last_mode = state.mode

        # UI/ROS commands
        if cmd == 'explore':
            prev_mode = 'explore'
            state.mode = Mode.EXPLORING
            print("UI → start exploring")
        elif cmd == 'fetch' and len(params) >= 1:
            prev_mode = 'fetch'
            try:
                prize_id = int(params[0])
                print("Prize ID set")
            except ValueError:
                print("Invalid prize ID")
                continue

            state.fetch_prize_id = prize_id
            state.mode = Mode.FETCH
            state.home_location = (amap.x, amap.y)
            print(f"UI → fetch prize {state.fetch_prize_id}")
        elif cmd == 'goal' and len(params) >= 2:
            prev_mode = 'goal'
            try:
                gx, gy = int(params[0]), int(params[1])
            except ValueError:
                print("Invalid goal coordinates")
                continue

            state.goal = (gx, gy)
            state.goal_set_by_explore = False
            state.mode = Mode.DRIVE_TO_GOAL
            print(f"UI → go to goal {state.goal}")

            if (gx, gy) in amap.intersections:
                print("→ Known intersection: seeding Dijkstra")
                amap.cleargoal()
                amap.setgoal(gx, gy)
            else:
                print("→ Unknown intersection: will directed‐explore")
        elif cmd == 'pause':
            state.paused = True
            print("UI → pause")
        elif cmd == 'resume':
            if prev_mode == 'explore': state.mode = Mode.EXPLORING
            if prev_mode == 'goal':   state.mode = Mode.DRIVE_TO_GOAL
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
                    pickle.dump(amap, f)
                print(f"Map saved to {fname}")
            except Exception as e:
                print("Failed to save map:", e)
        elif cmd == 'load' and params:
            fname = params[0]
            try:
                with open(fname, 'rb') as f:
                    loaded_map = pickle.load(f)
                # load all intersections and robot pose from loaded map
                amap.intersections = loaded_map.intersections
                amap.x = loaded_map.x
                amap.y = loaded_map.y
                amap.heading = loaded_map.heading
                # clear all blockages
                for inter in amap.intersections.values():
                    inter.clear_blocked()
                print(f"Map loaded from {fname}")
                amap.show()
            except Exception as e:
                print("Failed to load map:", e)
        elif cmd == 'pose' and len(params) >= 3:
            try:
                x, y, h = map(int, params[:3])
                amap.x, amap.y, amap.heading = x, y, h
                print(f"Pose set to {(x,y,h)}")
                amap.show()
            except ValueError:
                print("Invalid pose parameters")
        elif cmd == 'show':
            amap.show()
        elif cmd == 'clear':
            for inter in amap.intersections.values():
                inter.clear_blocked()
            print("All blockages cleared")
            amap.show()
        elif cmd == 'quit':
            print("UI → quit requested")
            break

        # manual actions already executed above
        if cmd in ('left','right','straight'):
            continue

        # after action: check ahead and mark blockage
        try:
            dists = prox_sensor.read()
            front = dists[1]
            thresh = 0.7 if (amap.heading % 2)==1 else 0.4
            blocked = front < thresh
            inter = amap.getintersection(amap.x, amap.y)
            inter.set_blocked(amap.heading, blocked)
            # create a unique key for current position and heading
            key = (amap.x, amap.y, amap.heading)

            # only print if the blockage status has changed
            if last_blocked_status.get(key) != blocked:
                if blocked:
                    print(f"Blocked ahead at heading {amap.heading}, dist={front:.2f}m")
                else:
                    print(f"Path ahead at heading {amap.heading} is now clear.")

                last_blocked_status[key] = blocked

            # always update the blockage in the map
            inter.set_blocked(amap.heading, blocked)
            
        except Exception:
            pass

        # auto-mode idle
        if state.mode not in (Mode.EXPLORING, Mode.DRIVE_TO_GOAL, Mode.FETCH):
            continue

        # pause/step
        if state.paused and cmd != 'step':
            continue
        if cmd == 'step': state.paused = True

        action = decide(state)

        # execute
        running = execute(action, state)

    print("Robot thread exiting.")


def main():
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connect to pigpio daemon")
        return

    # starting pose
    inp = input("Please enter the robot's starting coordinate (x y): ").split()
    try: x0,y0 = int(inp[0]),int(inp[1])
    except:   x0,y0 = 0,0
    try: h0 = int(input("Please enter the robot's starting heading [0-7]: "))%8
    except: h0 = 0
    onint = input("Is the robot starting on an intersection? (y/n): ").lower()

    # init HW & logic
    drive       = DriveSystem(io)
    line_sensor = LineSensor(io)
    prox_sensor = ProximitySensor(io)
    adc         = ADC(io)
    anglesensor = AngleSensor(adc)
    behaviors = Behaviors(drive, line_sensor)
    behaviors.anglesensor = anglesensor
    behaviors.prox        = prox_sensor
    amap        = Map()
    amap.x,amap.y,amap.heading = x0,y0,h0

    # first intersection
    if onint!='y':
        print("Driving until first intersection...")
        if behaviors.line_following()=="intersection":
            amap.updatemove(); amap.getintersection(amap.x,amap.y)
            print(f"Arrived at intersection {(amap.x,amap.y)}")
        else:
            print("Warning: did not detect intersection; starting map empty.")
    else:
        amap.getintersection(x0,y0)

    shared = SharedState()
    # ROS thread
    rosthread = threading.Thread(name="ROSThread", target=runros, args=(shared,))
    rosthread.start()
    # UI thread
    threading.Thread(target=ui_thread, args=(shared,), daemon=True).start()
    # Robot thread
    t_robot = threading.Thread(target=robot_thread, args=(shared, behaviors, amap, anglesensor, drive, line_sensor, prox_sensor))
    t_robot.start()

    try:
        t_robot.join()
    except KeyboardInterrupt:
        print("KeyboardInterrupt: stopping robot")
        drive.stop()
    finally:
        drive.stop(); io.stop()
        # shutdown ROS
        ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(rosthread.ident), ctypes.py_object(KeyboardInterrupt))
        rosthread.join()

if __name__ == '__main__':
    main()