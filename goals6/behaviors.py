#!/usr/bin/env python3
import time
from detectors import SideEstimator, IntersectionDetector, EndDetector, TurnDetector


FEEDBACK_TABLE = {
    (0, 1, 0): 'STRAIGHT',
    (0, 1, 1): 'VEER_RIGHT',
    (0, 0, 1): 'STEER_RIGHT',
    (1, 1, 0): 'VEER_LEFT',
    (1, 0, 0): 'STEER_LEFT',
    (1, 1, 1): 'STRAIGHT',
    (1, 0, 1): 'STRAIGHT',
    (0, 0, 0): 'STOP'
}

class Behaviors:
    """
    Mid-level Behaviors Class:
    - line_following
    - pull_forward
    - turn
    """

    def __init__(self, drive, sensor):
        self.drive = drive
        self.sensor = sensor

    def line_following(self):
        """
        Line following behavior.
        Returns "intersection" or "end" when done.
        """
        side_est = SideEstimator()
        end_det = EndDetector()
        intersection_det = IntersectionDetector()
        last_time = time.time()
        turn_start_time = last_time

        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            reading = self.sensor.read()
            side_state = side_est.update(reading, dt)
            end_state = end_det.update(reading, side_state, dt)
            intersection_state = intersection_det.update(reading, dt)

            if intersection_state:
                self.drive.stop()
                print("Intersection detected.")
                return "intersection"

            if end_state:
                self.drive.stop()
                print("End of street detected.")
                print(reading)
                return "end"
            
            if reading == (0, 0, 0):  
                if side_state == "LEFT":
                    self.drive.drive("SPIN_RIGHT")
                elif side_state == "RIGHT":
                    self.drive.drive("SPIN_LEFT")
                else:
                    self.drive.drive("STRAIGHT")

            else:
                action = FEEDBACK_TABLE.get(reading, 'STOP')
                if action == 'STOP':
                    self.drive.stop()
                else:
                    self.drive.drive(action)


    def pull_forward(self, duration=0.375):
        """
        Drive forward slightly to center robot over intersection.
        """
        # print("Pulling forward.")

        end_det = EndDetector()
        side_est = SideEstimator()
        t0 = time.time()
        end_time = t0 + duration
        last_time = t0
        
        while time.time() < end_time:
            now = time.time()
            dt = now - last_time
            last_time = now

            self.drive.drive("STRAIGHT")

            reading = self.sensor.read()
            side_state = side_est.update(reading, dt)
            end_state = end_det.update(reading, side_state, dt)

            if end_state:
                break

        t_back = time.time()
        while time.time() - t_back < 0.05:
            self.drive.left_motor.setlevel(-1.0)
            self.drive.right_motor.setlevel(-1.0)
        self.drive.stop()

        if not end_state:
            print("Street detected ahead.")
            return "street_ahead"
        
        elif end_state:
            print("No street detected ahead.")
            return "no_street_ahead"


    def turn(self, direction, anglesensor):
        """
        Turns left or right using line sensors and magnetometer.
        Returns number of 45-degree steps turned.
        """
        print(f"\nStarting {direction.upper()} turn...")
        spin_style = 'SPIN_LEFT' if direction.upper() == 'LEFT' else 'SPIN_RIGHT'

        phi_start = anglesensor.readangle()

        det = TurnDetector(direction)
        last_time = time.time()
        turn_start_time = last_time

        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            reading = self.sensor.read()
            done = det.update(reading, dt)

            if not done:
               self.drive.drive(spin_style)
            else:
                break
    
        self.drive.stop()
       
        # backdrive
        t_back = time.time()
        while time.time() - t_back < 0.05:
            # Backdriving motors at 100% for left turns
            if direction.upper() == 'LEFT':
                self.drive.left_motor.setlevel(1.0)
                self.drive.right_motor.setlevel(-1.0)
            # Backdriving motors at 100% for right turns
            else:
                self.drive.left_motor.setlevel(-1.0)
                self.drive.right_motor.setlevel(1.0)
        self.drive.stop()

        # Align with the line
        self.align_with_line()

        # Calculating turn number
        phi_end = anglesensor.readangle()
        time_end = time.time()
        diff_mag = (phi_end - phi_start) % 360
        diff_time = time_end - turn_start_time

        if direction.upper() == 'LEFT':
            if diff_time < 0.450:
                angle_estimated = 0
            elif diff_time < 0.895:
                angle_estimated = 45
            elif diff_time < 1.335: 
                angle_estimated = 90
            elif diff_time < 1.722:
                angle_estimated = 135
            elif diff_time < 2.100: 
                angle_estimated = 180
            elif diff_time < 2.280:
                angle_estimated = 225
            elif diff_time < 3.000:
                angle_estimated = 270
            else:
                angle_estimated = 360
                
        if direction.upper() == 'RIGHT':
            if diff_time < 0.450:
                angle_estimated = 0
            elif diff_time < 0.895:
                angle_estimated = -45
            elif diff_time < 1.335: 
                angle_estimated = -90
            elif diff_time < 1.722:
                angle_estimated = -135
            elif diff_time < 2.100: 
                angle_estimated = -180
            elif diff_time < 2.280:
                angle_estimated = -225
            elif diff_time < 3.000:
                angle_estimated = -270
            else:
                angle_estimated = -360

        '''
        if direction.upper() == 'LEFT':
            # y = 0.00555 x + 0.546  ⇒  x = (y - 0.546)/0.00555
            angle_time = (diff_time - 0.546) / 0.00555
        else:
            # y = -0.00543 x + 0.587 ⇒ x = (y - 0.587)/(-0.00543)
            angle_time = (diff_time - 0.587) / -0.00543

        # y = 0.978 x + 0.187 ⇒ real_angle = 0.978*mag + 0.187
        angle_mag = 0.978 * diff_mag + 0.187

        # —— Fuse estimates (simple average) —— 
        angle_estimated = (angle_time + angle_mag) / 2.0
        '''
        
        # Choosing between an average of time and magnetometer reading or only time

        # For 360 degree turns
        if abs(angle_estimated) == 360 or abs(diff_mag) > 350:
            if direction.upper() == 'LEFT':
                diff = 0
            if direction.upper() == 'RIGHT':
                diff = 0
        else:
            if direction.upper() == 'LEFT':
                average = (diff_mag*0.3) + (angle_estimated*0.7)
            if direction.upper() == 'RIGHT':
                if diff_mag > 180:
                    diff_mag = diff_mag - 180
                average = (diff_mag*0.3) + abs((angle_estimated*0.7))

            diff = min(average, angle_estimated)
            if diff > 180:
                diff -= 360
            if diff < -180:
                diff += 360
            print(f"average {average}")

        # Ensure the turn number is positive for left turns, negative for right turns
        if direction.upper() == 'LEFT':
            turn_number = (round(diff / 45) + 8) % 8  # Left turn
        if direction.upper() == 'RIGHT':
            turn_number = ((round(diff / 45) + 8) % 8) - 8 # Right turn

        # If the turn is 360 degrees, give a turn number of 8 or -8 (depending if
        # the turn is left or right)
        if turn_number == 0 and diff_time > 0.350 and direction.upper() == 'LEFT':
            turn_number = 8
        if turn_number == 0 and diff_time > 0.350 and direction.upper() == 'RIGHT':
            turn_number = -8
        
        print(f"Turn amount: {diff:.1f}°, Turn number: {turn_number}")
        print(f"phi start {phi_start}")
        print(f"phi end {phi_end}")
        print(f"time start {turn_start_time}")
        print(f"time end {time_end}")
        print(f"time diff {diff_time}")
        print(f"mag diff {diff_mag}")
        #print(f"angle_mag {angle_mag}")
        #print(f"angle_time {angle_time}")
        return (turn_number, diff)
    

    def align_with_line(self, max_time=0.8):
        """
        Adjust robot to re-align with the line after a turn.
        Spins in place until the center sensor sees the line again.
        """
        start_time = time.time()
        
        while time.time() - start_time < max_time:
            reading = self.sensor.read()
            
            # Check for center line detection
            if reading == (0, 1, 0) or reading == (1, 1, 1):
                self.drive.stop()
                return True
            
            # If line is seen on one side, rotate accordingly
            if reading == (1, 0, 0) or reading == (1, 1, 0):
                self.drive.drive("SPIN_LEFT")
            elif reading == (0, 0, 1) or reading == (0, 1, 1):
                self.drive.drive("SPIN_RIGHT")
            else:
                self.drive.drive("SPIN_LEFT")

        self.drive.stop()
        return False
        