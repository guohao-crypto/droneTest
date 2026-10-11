"""Python side of the Drone Testbed bridge.

Start the Godot scene first, then run a script that uses this module.
Only the Python standard library is needed (Python 3.8 or newer).

    from drone_sim import DroneSim

    with DroneSim() as sim:
        while True:
            state = sim.read_state()            # newest state from Godot
            sim.send_velocity(0.0, 0.0, -3.0)   # fly 3 m/s towards -Z

Coordinates are the Godot scene's: [x, y, z] in metres, Y is up.

sim.world (sent once on connect):
    terrain    columns, rows, cell_size, origin [x, z], heights (flat list)
    obstacles  list of {name, center [x, y, z], size [x, y, z]}
    human      {name, center, size}
    drone      radius, max_speed, max_climb_speed, max_yaw_rate
    sensors    range, ray_angles_deg

state (about 30 times a second):
    time            seconds since the scene started
    position        [x, y, z]
    velocity        [x, y, z] actually achieved, m/s
    yaw             heading in radians; 0 faces -Z, positive is a left turn
    ground_height   terrain height under the drone
    altitude        height above that ground
    ranges          distance seen by each horizontal range sensor, in the
                    order of world["sensors"]["ray_angles_deg"] (degrees
                    clockwise from straight ahead); equals the sensor range
                    when nothing is hit
    range_down      distance to whatever is directly below
    contacts        names of things the drone is touching right now
    human_distance  straight-line distance to the human
    human_in_sight  True if nothing blocks the line to the human
    manual_override True if someone pressed Tab in Godot to fly by hand
"""
import json
import select
import socket

DEFAULT_PORT = 9361


class DroneSim:
    def __init__(self, host="127.0.0.1", port=DEFAULT_PORT, timeout=5.0):
        try:
            self._sock = socket.create_connection((host, port), timeout=timeout)
        except OSError as error:
            raise SystemExit(
                "Could not connect to Godot on %s:%d (%s).\n"
                "Run the Drone Testbed scene in Godot first, and keep its window open."
                % (host, port, error)
            )
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._buffer = b""
        self._pending = []
        self.world = self._next("world")

    # --- reading ---------------------------------------------------------

    def read_state(self):
        """Waits for the next state and returns the newest one available.

        If several states piled up while your code was busy, the older ones
        are skipped so you always act on current data.
        """
        state = self._next("state")
        self._pending.extend(self._receive(wait=False))
        newer = [m for m in self._pending if m.get("type") == "state"]
        self._pending = [m for m in self._pending if m.get("type") != "state"]
        return newer[-1] if newer else state

    def ground_height(self, x, z):
        """Terrain height at world (x, z), from the heights sent on connect."""
        terrain = self.world["terrain"]
        columns, rows = terrain["columns"], terrain["rows"]
        heights = terrain["heights"]
        fx = (x - terrain["origin"][0]) / terrain["cell_size"]
        fz = (z - terrain["origin"][1]) / terrain["cell_size"]
        fx = min(max(fx, 0.0), columns - 1.0)
        fz = min(max(fz, 0.0), rows - 1.0)
        c = min(int(fx), columns - 2)
        r = min(int(fz), rows - 2)
        tx, tz = fx - c, fz - r
        top = heights[r * columns + c] * (1 - tx) + heights[r * columns + c + 1] * tx
        bottom = heights[(r + 1) * columns + c] * (1 - tx) + heights[(r + 1) * columns + c + 1] * tx
        return top * (1 - tz) + bottom * tz

    # --- commanding ------------------------------------------------------

    def send_velocity(self, vx, vy, vz, yaw_rate=0.0):
        """Asks the drone to fly at this world-space velocity (m/s).

        Godot clamps it to the drone's limits. Keep sending: the drone
        hovers if it hears nothing for half a second.
        """
        self._send({"type": "cmd", "velocity": [vx, vy, vz], "yaw_rate": yaw_rate})

    def hover(self):
        self.send_velocity(0.0, 0.0, 0.0)

    def reset(self):
        """Puts the drone back at its start point and waits until it is there."""
        self._send({"type": "reset"})
        self._next("reset_done")

    def close(self):
        try:
            self.hover()
        except OSError:
            pass
        self._sock.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    # --- plumbing --------------------------------------------------------

    def _send(self, message):
        self._sock.sendall((json.dumps(message) + "\n").encode("utf-8"))

    def _next(self, wanted):
        """Blocks until a message of type `wanted` arrives and returns it."""
        while True:
            for index, message in enumerate(self._pending):
                if message.get("type") == wanted:
                    del self._pending[: index + 1]
                    return message
            self._pending = self._receive(wait=True)

    def _receive(self, wait):
        """Returns the complete messages that have arrived.

        With wait=True, blocks until there is at least one.
        """
        messages = []
        while True:
            while b"\n" in self._buffer:
                line, self._buffer = self._buffer.split(b"\n", 1)
                if not line.strip():
                    continue
                message = json.loads(line)
                if message.get("type") == "error":
                    print("Godot rejected a message:", message.get("message"))
                else:
                    messages.append(message)
            readable = select.select([self._sock], [], [], 0)[0]
            if not readable and (messages or not wait):
                return messages
            try:
                chunk = self._sock.recv(65536)
            except socket.timeout:
                raise SystemExit("Godot stopped sending data. Is the scene still running?")
            if not chunk:
                raise SystemExit("Godot closed the connection (scene stopped).")
            self._buffer += chunk
