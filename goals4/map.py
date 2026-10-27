import matplotlib.pyplot as plt
from enum import Enum
import math

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


intersections = {}

def getintersection(x, y):
    """
    Retrieve the Intersection at (x, y), creating it if necessary.
    """
    if (x, y) not in intersections:
        intersections[(x, y)] = Intersection(x, y)
    return intersections[(x, y)]

def calcmove(x, y, heading):
    """
    Given old (x, y) and heading ∈ [0..7], return new (x, y).
    """
    dx, dy = HEADING_DELTAS[heading]
    return x + dx, y + dy

def calcturn(oldheading, turn):
    """
    Update heading by a turn amount (0..7), wrapping 0..7.
    """
    return (oldheading + turn) % 8

def calcuturn(oldheading):
    """
    Perform a U-turn (180°), equivalent to adding 4 mod 8.
    """
    return (oldheading + 4) % 8

def show(xpose, ypose, hpose):
    """
    Draw the full map and current robot pose:
    - Grid points (-3..3)
    - All intersections & street segments colored by status
    - A magenta arrow at (xpose, ypose) showing heading
    """
    plt.clf()
    ax = plt.axes()
    ax.set_xlim(-3.5, 3.5)
    ax.set_ylim(-3.5, 3.5)
    ax.set_aspect('equal')
    ax.grid(False)

    # Draw all grid points
    for xi in range(-3, 4):
        for yi in range(-3, 4):
            plt.plot(xi, yi, marker='o', markersize=8, color='gray')

    # Draw each street segment for every known intersection
    for (ix, iy), inter in intersections.items():
        for h, status in enumerate(inter.streets):
            color_map = {
                STATUS.UNKNOWN: 'black',
                STATUS.NONEXISTENT: 'gray',
                STATUS.UNEXPLORED: 'blue',
                STATUS.DEADEND: 'red',
                STATUS.CONNECTED: 'green',
            }
            c = color_map[status]
            dx, dy = DRAW_DELTAS[h]
            xfrom, yfrom = ix, iy
            xto, yto     = ix + dx, iy + dy
            plt.plot([xfrom, xto], [yfrom, yto], color=c)

    # Draw robot pose arrow
    dxh, dyh = HEADING_DELTAS[hpose]
    mag = math.hypot(dxh, dyh)
    ux, uy = (dxh / mag, dyh / mag) if mag else (0, 0)
    length = 0.5
    xtip = xpose + ux * length
    ytip = ypose + uy * length
    plt.arrow(xpose, ypose,
              xtip - xpose, ytip - ypose,
              width=0.2,
              head_width=0.3,
              head_length=0.1,
              color='magenta')

    plt.pause(0.001)

class Map:
    """
    Maintains the robot's pose and the global intersection graph.
    """
    def __init__(self):
        # Robot starts at origin facing North (0)
        self.x = 0
        self.y = 0
        self.heading = 0
        # Ensure origin intersection exists
        getintersection(0, 0)

    def getstreet(self, x, y, heading):
        # Implement the logic to retrieve the street based on x, y, heading
        return f"Street data at ({x}, {y}) with heading {heading}"

    def show(self):
        show(self.x, self.y, self.heading)

    def updateheading(self, turn):
        self.heading = calcturn(self.heading, turn)

    def updatemove(self):
        self.x, self.y = calcmove(self.x, self.y, self.heading)

    def setstreet(self, x, y, heading, status):
        inter = getintersection(x, y)
        existing_status = inter.streets[heading]

        # Do not overwrite connected streets or deadends
        if existing_status in (STATUS.CONNECTED, STATUS.DEADEND):
            if status != STATUS.CONNECTED:
                return

        # Avoid downgrading
        if existing_status != STATUS.UNKNOWN and status == STATUS.UNKNOWN:
            return

        inter.streets[heading] = status