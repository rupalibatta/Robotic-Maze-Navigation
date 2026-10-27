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

# Drawing deltas are half-length for visualization
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
        self.blocked = [False for _ in range(8)]  # ← blockage flags per heading
        self.cost = float('inf')
        self.direction_to_goal = None

    def set_blocked(self, heading, is_blocked=True):
        """Mark a given heading as blocked or unblocked."""
        self.blocked[heading] = is_blocked

    def clear_blocked(self):
        """Clear all blocked flags."""
        self.blocked = [False for _ in range(8)]


class Map:
    def __init__(self):
        self.intersections = {}
        self.x = 0
        self.y = 0
        self.heading = 0

    def clear_all_blockages(self):
        """UI ‘clear’ command: remove every blockage flag from every intersection."""
        for inter in self.intersections.values():
            inter.clear_blocked()

    def getintersection(self, x, y):
        if (x, y) not in self.intersections:
            self.intersections[(x, y)] = Intersection(x, y)
        return self.intersections[(x, y)]

    def getstreet(self, x, y, heading):
        return self.getintersection(x, y).streets[heading]

    def setstreet(self, x, y, heading, status):
        inter = self.getintersection(x, y)
        existing = inter.streets[heading]
        # Do not downgrade CONNECTED/DEADEND/NONEXISTENT
        if existing in (STATUS.CONNECTED, STATUS.DEADEND, STATUS.NONEXISTENT) and existing != status:
            print(f"[WARN] Not downgrading street ({x}, {y}) h={heading}")
            return
        inter.streets[heading] = status

    def updateheading(self, turn_amt):
        self.heading = (self.heading + turn_amt) % 8

    def updatemove(self):
        dx, dy = HEADING_DELTAS[self.heading]
        self.x += dx
        self.y += dy

    def showwithrobot(self, xpose, ypose, hpose, fname="map.png"):
        """
        Plot the map, including streets and robot heading,
        then save to a PNG and pause so it renders on‐screen.
        """
        plt.ion()
        plt.clf()
        ax = plt.axes()
        ax.set_xlim(-5.5, 5.5)
        ax.set_ylim(-5.5, 5.5)
        ax.set_aspect('equal')
        ax.grid(False)

        # draw grid dots
        for xi in range(-6, 6):
            for yi in range(-6, 6):
                ax.plot(xi, yi, 'o', markersize=3, color='lightgray')

        # draw each street and blockage
        for (ix, iy), inter in self.intersections.items():
            for h, status in enumerate(inter.streets):
                dx, dy = DRAW_DELTAS[h]
                xfrom, yfrom = ix, iy
                xto, yto = ix + dx, iy + dy
                # choose color by status
                color_map = {
                    STATUS.UNKNOWN:     'black',
                    STATUS.NONEXISTENT: 'lightgray',
                    STATUS.UNEXPLORED:  'blue',
                    STATUS.DEADEND:     'red',
                    STATUS.CONNECTED:   'green',
                }
                c = color_map[status]
                # always solid line
                ax.plot([xfrom, xto], [yfrom, yto], '-', color=c)

                # overlay blockage marker if blocked
                if inter.blocked[h]:
                    mx, my = (xfrom + xto) / 2, (yfrom + yto) / 2
                    ax.plot(mx, my, marker='x', markersize=8, color='red', markeredgewidth=2)

            # optionally draw Dijkstra arrows
            if inter.direction_to_goal is not None:
                dx, dy = DRAW_DELTAS[inter.direction_to_goal]
                ax.arrow(ix, iy, dx, dy, head_width=0.1, color='orange')

        # draw the robot’s own pose
        dxh, dyh = HEADING_DELTAS[hpose]
        mag = math.hypot(dxh, dyh) or 1
        ux, uy = dxh/mag, dyh/mag
        length = 0.5
        ax.arrow(xpose, ypose, ux*length, uy*length,
                 width=0.2, head_width=0.3, head_length=0.1,
                 color='magenta')

        # save to disk
        plt.pause(0.001)
        if fname:
            plt.savefig(fname)

    def show(self, fname="map.png"):
        self.showwithrobot(self.x, self.y, self.heading, fname)

    def cleargoal(self):
        for inter in self.intersections.values():
            inter.cost = float('inf')
            inter.direction_to_goal = None

    def setgoal(self, xgoal, ygoal):
        print(f"Clearing previous goal data…")
        self.cleargoal()
        if (xgoal, ygoal) not in self.intersections:
            print(f"Goal ({xgoal},{ygoal}) not on map; cannot set goal.")
            return
        goal = self.getintersection(xgoal, ygoal)
        goal.cost = 0
        print(f"Goal node at ({xgoal},{ygoal}) cost set to 0")
        q = PriorityQueue()
        q.put((0, xgoal, ygoal))
        while not q.empty():
            cost, x, y = q.get()
            inter = self.getintersection(x, y)
            if cost > inter.cost:
                continue
            for h in range(8):
                if inter.streets[h] == STATUS.CONNECTED and not inter.blocked[h]:
                    dx, dy = HEADING_DELTAS[h]
                    xn, yn = x + dx, y + dy
                    neighbor = self.getintersection(xn, yn)
                    new_cost = cost + math.hypot(dx, dy)
                    if new_cost < neighbor.cost:
                        neighbor.cost = new_cost
                        neighbor.direction_to_goal = (h + 4) % 8
                        q.put((new_cost, xn, yn))
            self.showwithrobot(self.x, self.y, self.heading)
        if goal.cost == float('inf'):
            print(f"[WARN] Goal ({xgoal},{ygoal}) unreachable.")
        else:
            print(f"Done. Goal tree built, goal cost={goal.cost:.1f}")

    def skipped_directions(self, turn_number):
        skipped = []
        old = self.heading
        if turn_number > 0:
            for i in range(1, turn_number):
                skipped.append((old - i) % 8)
        elif turn_number < 0:
            for i in range(1, -turn_number):
                skipped.append((old + i) % 8)
        return skipped
