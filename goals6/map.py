import matplotlib.pyplot as plt
from enum import Enum
import math
from queue import PriorityQueue

# Heading deltas
HEADING_DELTAS = {
    0: (0, 1),    # North
    1: (-1, 1),   # Northwest
    2: (-1, 0),   # West
    3: (-1, -1),  # Southwest
    4: (0, -1),   # South
    5: (1, -1),   # Southeast
    6: (1, 0),    # East
    7: (1, 1),    # Northeast
}

DRAW_DELTAS = {h: (dx * 0.5, dy * 0.5) for h, (dx, dy) in HEADING_DELTAS.items()}


class STATUS(Enum):
    UNKNOWN     = 0
    NONEXISTENT = 1
    UNEXPLORED  = 2
    DEADEND     = 3
    CONNECTED   = 4


class Intersection:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.streets = [STATUS.UNKNOWN for _ in range(8)]
        self.cost = float('inf')
        self.direction_to_goal = None


class Map:
    def __init__(self):
        self.intersections = {}
        self.x = 0
        self.y = 0
        self.heading = 0


    def getintersection(self, x, y):
        '''if (x, y) == (self.x, self.y):
            return None'''
        if (x, y) not in self.intersections:
            self.intersections[(x, y)] = Intersection(x, y)
        return self.intersections[(x, y)]


    def getstreet(self, x, y, heading):
        return self.getintersection(x, y).streets[heading]


    def setstreet(self, x, y, heading, status):
        inter = self.getintersection(x, y)
        existing = inter.streets[heading]

        # Overwrite warning
        if existing == STATUS.CONNECTED and status != STATUS.CONNECTED:
            print(f"[WARN] Not downgrading street ({x}, {y}) h={heading}")
            return

        if existing == STATUS.DEADEND and status != STATUS.DEADEND:
            print(f"[WARN] Not downgrading street ({x}, {y}) h={heading}")
            return

        if existing == STATUS.NONEXISTENT and status != STATUS.NONEXISTENT:
            print(f"[WARN] Not downgrading street ({x}, {y}) h={heading}")
            return

        # Do not overwrite connected streets, dead ends, or nonexistent
        if existing in (STATUS.CONNECTED, STATUS.DEADEND, STATUS.NONEXISTENT):
            if status != existing:
                return

        inter.streets[heading] = status


    def updateheading(self, turn_amt):
        self.heading = (self.heading + turn_amt) % 8


    def updatemove(self):
        dx, dy = HEADING_DELTAS[self.heading]
        self.x += dx
        self.y += dy


    def showwithrobot(self, xpose, ypose, hpose, fname="optimus_map.png"):
        plt.ion()
        """
        Plot the map, including streets and robot heading,
        then save to a PNG and pause so it renders on‐screen.
        """
        plt.clf()
        ax = plt.axes()
        ax.set_xlim(-5.5, 5.5)
        ax.set_ylim(-5.5, 5.5)
        ax.set_aspect('equal')
        ax.grid(False)

        # draw grid dots
        for xi in range(-6, 6):
            for yi in range(-6, 6):
                plt.plot(xi, yi, 'o', markersize=3, color='lightgray')

        # draw each street
        for (ix, iy), inter in self.intersections.items():
            for h, status in enumerate(inter.streets):
                color_map = {
                    STATUS.UNKNOWN:     'black',
                    STATUS.NONEXISTENT: 'gray',
                    STATUS.UNEXPLORED:  'blue',
                    STATUS.DEADEND:     'red',
                    STATUS.CONNECTED:   'green',
                }
                c = color_map[status]
                dx, dy   = DRAW_DELTAS[h]
                xfrom, yfrom = ix, iy
                xto,   yto   = ix + dx, iy + dy
                # plotting lines in street colors
                plt.plot([xfrom, xto], [yfrom, yto], color=c)

            # optionally draw Dijkstra arrows
            if inter.direction_to_goal is not None:
                dx, dy = DRAW_DELTAS[inter.direction_to_goal]
                plt.arrow(ix, iy, dx, dy, head_width=0.1, color='orange')

        # draw heading arrow
        dxh, dyh = HEADING_DELTAS[hpose]
        mag = math.hypot(dxh, dyh)
        ux, uy = (dxh / mag, dyh / mag) if mag else (0, 0)
        length = 0.5
        xtip = xpose + ux * length
        ytip = ypose + uy * length
        plt.arrow(xpose, ypose, xtip - xpose, ytip - ypose,
                  width=0.2, head_width=0.3, head_length=0.1, color='magenta')

        # ----- new: save to disk -----
        plt.savefig(fname)

        # pause so interactive window updates
        plt.pause(0.001)


    def show(self, fname="map.png"):
        self.showwithrobot(self.x, self.y, self.heading, fname)


    def cleargoal(self):
        # Clear any existing Dijkstra data so auto‐drive is off.
        for inter in self.intersections.values():
            inter.cost = float('inf')
            inter.direction_to_goal = None
        

    def setgoal(self, xgoal, ygoal):
        """
        Implementing Dijkstra's algorithm via PQs.
        """
        # 1. Reset all costs/directions
        print(f"Clearing previous goal data…")
        self.cleargoal()

        if (xgoal, ygoal) not in self.intersections:
            print(f"Goal ({xgoal},{ygoal}) not on map; cannot set goal.")
            return

        # 2. Seed the priority queue with the goal
        goal = self.getintersection(xgoal, ygoal)
        goal.cost = 0
        print(f"Goal node at ({xgoal},{ygoal}) cost set to 0")
        from queue import PriorityQueue
        q = PriorityQueue()
        q.put((0, xgoal, ygoal))

        # 3. Main loop
        while not q.empty():
            cost, x, y = q.get()
            inter = self.getintersection(x, y)
            # If this cost is stale (we already found a better path), skip it
            if cost > inter.cost:
                print(f"[DEBUG] Stale entry for ({x},{y}), skipping (stored cost {inter.cost})")
                continue

            # 4. Relax all CONNECTED neighbors
            for h in range(8):
                if inter.streets[h] == STATUS.CONNECTED:
                    dx, dy = HEADING_DELTAS[h]
                    xn, yn = x + dx, y + dy
                    neighbor = self.getintersection(xn, yn)
                    new_cost = cost + math.hypot(dx, dy)
                    if new_cost < neighbor.cost:
                        neighbor.cost = new_cost
                        neighbor.direction_to_goal = (h + 4) % 8
                        q.put((new_cost, xn, yn))
            self.showwithrobot(self.x, self.y, self.heading)

        # 5. At the end, check if the goal itself was ever connected back
        if goal.cost == float('inf'):
            print(f"[WARN] Goal ({xgoal},{ygoal}) remains at infinite cost—unreachable.")
        else:
            print(f"Done. Goal tree built, goal cost={goal.cost:.1f}")
    

    def skipped_directions(self, turn_number):
        """
        Take a signed turn amount (in 45° steps) and return the headings
        that were passed over (and should be marked NONEXISTENT).
        """
        skipped = []
        old_heading = self.heading

        if turn_number > 0: # left turn (positive)
            for i in range(1, (turn_number)):
                # Calculate number of nonexistent directions
                skipped_direction = (old_heading - i) % 8
                skipped.append(skipped_direction)
        elif turn_number < 0: # right turn (negative)
            for i in range(1, -(turn_number)):
                # Calculate number of nonexistent directions
                skipped_direction = (old_heading + i) % 8
                skipped.append(skipped_direction)
        else: # No turn or turn amount is zero
            pass

        return skipped