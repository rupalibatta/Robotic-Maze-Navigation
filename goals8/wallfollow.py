import pigpio
import threading
import time
import sys
from drivesystem import DriveSystem
from proximitysensor import ProximitySensor

# Shared mode: -1 = quit, 0 = stop, 1 = discrete, 2 = continuous
class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.mode = 0  # start in STOP mode

def ui(shared: SharedState):
    """Daemon thread for user input."""
    print("UI thread started. Type 'discrete', 'continuous', 'stop', or 'quit'.")
    while True:
        cmd = input("Command? ").strip().lower()
        with shared.lock:
            if cmd == 'discrete':
                shared.mode = 1
            elif cmd == 'continuous':
                shared.mode = 2
            elif cmd == 'stop':
                shared.mode = 0
            elif cmd == 'quit':
                shared.mode = -1
                break
            else:
                print("Unknown command. Valid options: discrete, continuous, stop, quit")

def wall_follow(discrete, continuous, stop, get_distances):
    """Generic wrapper that executes the correct wall-following behavior."""
    def wrapper():
        while True:
            mode = 0
            with shared.lock:
                mode = shared.mode

            if mode < 0:
                break
            elif mode == 0:
                stop()
            elif mode == 1:
                discrete(get_distances())
            elif mode == 2:
                continuous(get_distances())

            time.sleep(0.01)
    return wrapper

if __name__ == '__main__':
    io = pigpio.pi()
    if not io.connected:
        print("Failed to connect to pigpio daemon.")
        sys.exit(1)

    drive = DriveSystem(io)
    sensor = ProximitySensor(io)
    shared = SharedState()

    # Start UI thread
    thread_ui = threading.Thread(target=ui, args=(shared,), daemon=True)
    thread_ui.start()

    print("Starting multi-threaded wall-following...")

    def get_distances():
        return sensor.read()

    def stop():
        drive.stop()

    def discrete(distances):
        d0, max_err = 0.30, 0.10
        band = max_err / 3.0
        dleft, _, dright = distances
        d = dright  # right wall-following
        e = d - d0
        if abs(e) > max_err:
            drive.stop()
        elif e < -2 * band:
            drive.drive('TURN_LEFT')
        elif e < -band:
            drive.drive('STEER_LEFT')
        elif e < 0:
            drive.drive('VEER_LEFT')
        elif e < band:
            drive.drive('STRAIGHT')
        elif e < 2 * band:
            drive.drive('VEER_RIGHT')
        elif e < 3 * band:
            drive.drive('STEER_RIGHT')
        else:
            drive.drive('TURN_RIGHT')

    def continuous(distances):
        d0, max_err = 0.30, 0.10
        cl, kl = 0.816, 2.011
        cr, kr = 0.853, -2.330
        dleft, _, dright = distances
        d = dright
        e = d - d0
        if abs(e) > max_err:
            drive.stop()
        else:
            pwm_l = cl + kl * e
            pwm_r = cr + kr * e
            drive.pwm(pwm_l, pwm_r)

    try:
        loop = wall_follow(discrete, continuous, stop, get_distances)
        loop()
    except KeyboardInterrupt:
        print("Interrupted.")
    finally:
        sensor.shutdown()
        drive.stop()
        io.stop()
        print("Shutdown complete.")
