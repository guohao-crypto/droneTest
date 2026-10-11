# Drone Testbed

A small Godot 4.7 scene for testing drone navigation. Godot handles the 3D world and the physics. The navigation code is written in Python and controls the drone through a local socket.

What's in the scene:

- heightmap terrain, generated from `terrain/heightmap.png`
- the drone (blue sphere)
- three box obstacles
- the human (pink glowing box with a label over it)

## Setup

You need Godot 4.7 (the standard version, not .NET) and Python 3.8 or newer. There are no Python packages to install.

## Running

1. Open Godot, import `project.godot`, and press Run.
2. The top left corner of the game should say `Python: waiting on 127.0.0.1:9361`.
3. Leave the game running and open a terminal:

```
cd python
python3 fly_astar.py
```

The drone plans a path around the obstacles and flies to the human. When it gets there the script prints the time it took and anything it touched.

`fly_straight.py` does the same thing without any planning. It flies straight at the human and runs into the obstacle on the way, so we use it as a baseline to compare against.

## Manual controls

- W / S: forward and back
- A / D: left and right
- Q / E: turn
- Space / Shift: up and down
- Tab: switch between manual and autonomous

If the keys don't do anything, check that the toolbar above the game window is set to Input and not 2D or 3D.

## Python API

Everything goes through the `DroneSim` class in `python/drone_sim.py`.

```python
import math
from drone_sim import DroneSim

with DroneSim() as sim:
    gx, gy, gz = sim.world["human"]["center"]
    sim.reset()
    while True:
        state = sim.read_state()
        x, y, z = state["position"]
        if math.hypot(gx - x, gz - z) < 2.5:
            break
        sim.send_velocity(gx - x, 0.0, gz - z)
```

- `sim.world` is sent once when you connect. It has the terrain heights, the obstacle boxes, the human's position and the drone's speed limits.
- `sim.read_state()` returns the latest state, about 30 times a second: position, velocity, yaw, altitude, 8 range sensor readings, what the drone is touching, and the distance to the human.
- `sim.send_velocity(vx, vy, vz)` sets the velocity you want in m/s.
- `sim.reset()` moves the drone back to the start.
- `sim.ground_height(x, z)` gives the terrain height at a point.

Some notes:

- Coordinates are the same as in Godot. Units are meters and Y is up.
- Speed is capped at 10 m/s horizontally and 6 m/s vertically.
- If the script stops sending commands for half a second the drone hovers in place.
- When the script disconnects, control goes back to the keyboard.

The full list of fields is in the comment at the top of `drone_sim.py`.

## Editing the scene

The obstacles and the human can be moved around in the Godot editor. They get placed on the ground when the game starts, so only their X and Z position matters. To add another obstacle, duplicate one of the existing ones.

To change the terrain, replace `terrain/heightmap.png` with another grayscale image (white is high, black is low, one pixel is one meter). `terrain/make_heightmap.py` can generate a random one but it needs numpy and pillow.

## Files

```
main.tscn                  the scene
scripts/terrain.gd         builds the terrain mesh and collision from the heightmap
scripts/drone.gd           drone movement
scripts/chase_camera.gd    camera that follows the drone
scripts/main.gd            places objects on the ground, updates the HUD
scripts/python_bridge.gd   Godot side of the Python connection
python/drone_sim.py        Python side of the connection
python/fly_straight.py     baseline example
python/fly_astar.py        A* example
```