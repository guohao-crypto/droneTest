"""Example planner: A* around the obstacles, then follow the path.

1. Lay a 1 m grid over the terrain and block every cell that is inside an
   obstacle, plus a safety margin.
2. Run A* from the drone to the human.
3. Drop the waypoints that are not needed (straight-line shortcuts).
4. Fly waypoint to waypoint, holding a fixed height above the ground.

This uses the obstacle positions Godot sends on connect, i.e. a known map.
For methods that must discover obstacles themselves, use state["ranges"].

    python3 fly_astar.py
"""
import heapq
import math

from drone_sim import DroneSim

CELL = 1.0            # grid resolution, m
MARGIN = 1.5          # keep this far from obstacle walls, m (on top of the drone radius)
CRUISE_HEIGHT = 3.0   # height above ground, m
SPEED = 6.0           # m/s
WAYPOINT_RADIUS = 1.0 # switch to the next waypoint inside this distance, m
STOP_DISTANCE = 2.5   # stop this far (horizontally) from the human, m
TIME_LIMIT = 60.0     # s


class Grid:
    """Top-down occupancy grid covering the terrain."""

    def __init__(self, world):
        terrain = world["terrain"]
        self.x0, self.z0 = terrain["origin"]
        self.columns = int((terrain["columns"] - 1) * terrain["cell_size"] / CELL) + 1
        self.rows = int((terrain["rows"] - 1) * terrain["cell_size"] / CELL) + 1
        self.blocked = [[False] * self.columns for _ in range(self.rows)]

        keep_out = world["drone"]["radius"] + MARGIN
        for box in world["obstacles"]:
            (cx, _, cz), (sx, _, sz) = box["center"], box["size"]
            c_min, r_min = self.cell(cx - sx / 2 - keep_out, cz - sz / 2 - keep_out)
            c_max, r_max = self.cell(cx + sx / 2 + keep_out, cz + sz / 2 + keep_out)
            for r in range(r_min, r_max + 1):
                for c in range(c_min, c_max + 1):
                    self.blocked[r][c] = True

    def cell(self, x, z):
        c = int(round((x - self.x0) / CELL))
        r = int(round((z - self.z0) / CELL))
        return min(max(c, 0), self.columns - 1), min(max(r, 0), self.rows - 1)

    def point(self, c, r):
        return self.x0 + c * CELL, self.z0 + r * CELL

    def free(self, c, r):
        return 0 <= c < self.columns and 0 <= r < self.rows and not self.blocked[r][c]

    def clear_line(self, a, b):
        """True if the straight segment between two cells stays in free cells."""
        (c0, r0), (c1, r1) = a, b
        steps = int(max(abs(c1 - c0), abs(r1 - r0)) * 2) + 1
        for i in range(steps + 1):
            t = i / steps
            if not self.free(int(round(c0 + (c1 - c0) * t)), int(round(r0 + (r1 - r0) * t))):
                return False
        return True


def astar(grid, start, goal):
    """Shortest 8-connected path of cells from start to goal, or None."""
    def estimate(cell):
        dc, dr = abs(cell[0] - goal[0]), abs(cell[1] - goal[1])
        return max(dc, dr) + (math.sqrt(2) - 1) * min(dc, dr)

    moves = [(dc, dr, math.hypot(dc, dr)) for dc in (-1, 0, 1) for dr in (-1, 0, 1) if dc or dr]
    cost = {start: 0.0}
    came_from = {}
    frontier = [(estimate(start), start)]
    done = set()

    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]
        if current in done:
            continue
        done.add(current)

        for dc, dr, step in moves:
            nxt = (current[0] + dc, current[1] + dr)
            if not grid.free(*nxt):
                continue
            # no squeezing diagonally between two blocked cells
            if dc and dr and not (grid.free(current[0] + dc, current[1]) and grid.free(current[0], current[1] + dr)):
                continue
            new_cost = cost[current] + step
            if new_cost < cost.get(nxt, float("inf")):
                cost[nxt] = new_cost
                came_from[nxt] = current
                heapq.heappush(frontier, (new_cost + estimate(nxt), nxt))
    return None


def shorten(grid, path):
    """Keeps only the cells where the path has to turn."""
    result = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        reach = len(path) - 1
        while reach > anchor + 1 and not grid.clear_line(path[anchor], path[reach]):
            reach -= 1
        result.append(path[reach])
        anchor = reach
    return result


def main():
    with DroneSim() as sim:
        world = sim.world
        human = world["human"]["center"]
        max_climb = world["drone"]["max_climb_speed"]

        sim.reset()
        state = sim.read_state()
        start_time = state["time"]

        grid = Grid(world)
        start = grid.cell(state["position"][0], state["position"][2])
        goal = grid.cell(human[0], human[2])
        cells = astar(grid, start, goal)
        if cells is None:
            raise SystemExit("No path found: the human is walled in at this margin.")
        waypoints = [grid.point(c, r) for c, r in shorten(grid, cells)]
        print("Planned %d waypoints: %s" % (len(waypoints), ", ".join("(%.0f, %.0f)" % w for w in waypoints)))

        touched = set()
        travelled = 0.0
        previous = state["position"]
        index = 1  # waypoint 0 is where we are

        while True:
            state = sim.read_state()
            x, y, z = state["position"]
            elapsed = state["time"] - start_time
            touched.update(state["contacts"])
            travelled += math.dist(previous, state["position"])
            previous = state["position"]

            if math.hypot(human[0] - x, human[2] - z) <= STOP_DISTANCE:
                print("Reached the human in %.1f s, flew %.1f m." % (elapsed, travelled))
                break
            if elapsed > TIME_LIMIT:
                print("Gave up after %.0f s, still %.1f m away." % (elapsed, state["human_distance"]))
                break

            # steer at the current waypoint
            wx, wz = waypoints[index]
            dx, dz = wx - x, wz - z
            distance = math.hypot(dx, dz)
            if distance < WAYPOINT_RADIUS and index < len(waypoints) - 1:
                index += 1
                continue
            vx, vz = dx / distance * SPEED, dz / distance * SPEED

            # hold height above the highest ground we are about to cross
            ahead = max(sim.ground_height(x + vx * t, z + vz * t) for t in (0.0, 0.25, 0.5, 0.75))
            vy = 2.0 * (ahead + CRUISE_HEIGHT - y)
            vy = min(max(vy, -max_climb), max_climb)

            sim.send_velocity(vx, vy, vz)

        sim.hover()
        print("Touched on the way:", ", ".join(sorted(touched)) or "nothing")


if __name__ == "__main__":
    main()
