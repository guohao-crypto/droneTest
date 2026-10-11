# Drone Testbed

Minimal Godot 4.7 scene for trying out autonomous-navigation ideas.
Godot draws the world and runs the physics; the navigation code is written
in Python and talks to Godot over a local connection.

| Requirement | Node in `main.tscn` | Notes |
|---|---|---|
| Basic heightmap terrain | `Terrain` | Built at load time from `terrain/heightmap.png` (mesh + collision) |
| Sphere representing a drone | `Drone` | Blue sphere, radius 0.5 m, with a small dark nose showing its heading |
| At least one box obstacle | `Obstacle1`, `Obstacle2`, `Obstacle3` | Grey-blue boxes; `Obstacle1` sits between the drone and the human |
| Human as a highlighted box | `Human` | Glowing magenta box with a "HUMAN" label and a pink light on the ground |

## 快速开始（中文）

1. 用标准版 Godot 4.7 打开项目管理器，点"导入"，选本文件夹里的 `project.godot`。
2. 点编辑器右上角的 ▶ 运行。画面左上角会显示 `Python: waiting on 127.0.0.1:9361`。
3. 游戏窗口保持打开，另开一个终端：

   ```
   cd drone-testbed/python
   python3 fly_astar.py
   ```

4. 无人机会自己绕开障碍物飞到人旁边，终端里打印用时和有没有碰到东西。
5. 自己写导航：复制 `fly_astar.py` 改就行，接口说明在 `drone_sim.py` 开头。

游戏窗口工具栏要选"输入"（不是 2D / 3D），键盘才能操控无人机。

## Run it

1. Godot Project Manager -> **Import** -> pick `project.godot`.
2. Press the **Run** button (top right). The HUD shows
   `Python: waiting on 127.0.0.1:9361`.
3. Leave that window open and, in a terminal:

   ```
   cd drone-testbed/python
   python3 fly_straight.py   # baseline: straight at the human, bumps into things
   python3 fly_astar.py      # plans around the obstacles, touches nothing
   ```

The Python side needs Python 3.8+ and nothing else (standard library only).
The Godot side is GDScript only, so the standard (non-.NET) editor is enough.

## Writing navigation code in Python

```python
import math
from drone_sim import DroneSim

with DroneSim() as sim:
	gx, gy, gz = sim.world["human"]["center"]
	sim.reset()                                # same start point every run
	while True:
		state = sim.read_state()               # newest state from Godot
		x, y, z = state["position"]
		if math.hypot(gx - x, gz - z) < 2.5:   # close enough, seen from above
			break
		sim.send_velocity(gx - x, 0.0, gz - z) # m/s, world space; Godot caps the speed
```

That is a complete (and bad) controller: it holds its height and flies
straight at the human, scraping past whatever is in the way.

What you get (full list at the top of `python/drone_sim.py`):

- `sim.world` - sent once: terrain height grid, obstacle boxes, the human,
  drone limits, sensor layout.
- `sim.read_state()` - about 30 times a second: `position`, `velocity`, `yaw`,
  `altitude`, `ranges` (8 horizontal range sensors, 30 m reach), `range_down`,
  `contacts` (what the drone is touching), `human_distance`, `human_in_sight`.
- `sim.send_velocity(vx, vy, vz, yaw_rate=0.0)` - desired velocity. Godot
  clamps it to 10 m/s horizontal and 6 m/s vertical.
- `sim.reset()` - back to the start point, for repeated trials.
- `sim.ground_height(x, z)` - terrain height from the grid in `sim.world`.

Coordinates are Godot's: metres, Y up, the drone starts facing -Z.

Things worth knowing:

- Connecting switches the drone to autonomous; disconnecting gives control
  back to the keyboard. Tab in the Godot window takes over by hand at any time.
- If Python stops sending commands for 0.5 s the drone hovers.
- A planner that knows the map can use `sim.world["obstacles"]`. One that has
  to find obstacles itself should use only `state["ranges"]`.
- Keep the Godot window visible while a script runs; macOS can slow down
  apps whose windows are fully hidden.

## Flying by hand

| Key | Action |
|---|---|
| W / S | forward / back |
| A / D | strafe left / right |
| Q / E | turn left / right |
| Space / Shift | climb / descend |
| Tab | switch manual <-> autonomous |

In the editor's embedded game window the toolbar must be on **Input**
(not 2D / 3D), otherwise the game does not receive the keyboard.

## Changing the scene

- **Move or add obstacles / the human:** drag them in the editor. Anything in
  the `on_terrain` group is dropped onto the ground when the scene starts, so
  only X and Z matter. To add an obstacle, duplicate an existing one. Python
  receives the new layout automatically on the next connect.
- **Different terrain:** replace `terrain/heightmap.png` with any greyscale
  image (black = low, white = high; one pixel = one metre), or run
  `python3 terrain/make_heightmap.py <seed>` (needs numpy and pillow).
- **Taller or flatter hills:** select `Terrain` and change `Max Height`.
- **Camera:** select `Drone/Camera3D` and change `Offset` or `Field Of View`
  in the Inspector. The camera is positioned from code, so dragging it in
  the viewport has no effect; the drone likewise ignores any tilt or scale.

## Files

```
project.godot
main.tscn                  the scene
scripts/main.gd            ground snapping + HUD
scripts/terrain.gd         heightmap -> mesh + collision
scripts/drone.gd           drone movement (keyboard or commanded)
scripts/chase_camera.gd    camera that follows the drone
scripts/python_bridge.gd   the Godot end of the Python link
terrain/heightmap.png      129 x 129 greyscale heightmap
terrain/make_heightmap.py  optional generator for the PNG
python/drone_sim.py        the Python end of the link (import this)
python/fly_straight.py     example: baseline
python/fly_astar.py        example: A* path planning
```
