extends CharacterBody3D
## Sphere drone.
##
## Manual mode (default): fly with the keyboard.
##   W / S        forward / back
##   A / D        strafe left / right
##   Q / E        turn left / right
##   Space/Shift  climb / descend
##   Tab          toggle manual <-> autonomous
##
## Autonomous mode: the drone ignores the keyboard and flies at
## `command_velocity` (world space, m/s), turning at `command_yaw_rate`
## (rad/s, positive = left). The Python bridge sets these; so can any
## GDScript controller.

@export var manual_control := true
@export var max_speed := 10.0          # horizontal, m/s
@export var max_climb_speed := 6.0     # vertical, m/s
@export var turn_speed := 1.8          # rad/s
@export var acceleration := 30.0       # m/s^2

## Desired world-space velocity when manual_control is false.
var command_velocity := Vector3.ZERO
## Desired turn rate when manual_control is false.
var command_yaw_rate := 0.0
## Names of whatever the drone touched during the last physics step.
var contacts: PackedStringArray = []

var _home := Transform3D.IDENTITY


func _ready() -> void:
	# Keep only the heading. This drops any tilt or stretch the node picked
	# up from being dragged around in the editor.
	var forward := -global_basis.orthonormalized().z
	global_basis = Basis(Vector3.UP, atan2(-forward.x, -forward.z))
	_home = global_transform


func _physics_process(delta: float) -> void:
	var target := Vector3.ZERO
	var yaw_rate := 0.0
	if manual_control:
		yaw_rate = _axis(KEY_E, KEY_Q) * turn_speed
		var move := Vector3(_axis(KEY_A, KEY_D), 0.0, _axis(KEY_W, KEY_S))
		target = global_basis * (move.limit_length(1.0) * max_speed)
		target.y = _axis(KEY_SHIFT, KEY_SPACE) * max_climb_speed
	else:
		yaw_rate = clampf(command_yaw_rate, -turn_speed, turn_speed)
		var horizontal := Vector3(command_velocity.x, 0.0, command_velocity.z).limit_length(max_speed)
		target = horizontal
		target.y = clampf(command_velocity.y, -max_climb_speed, max_climb_speed)

	rotate_y(yaw_rate * delta)
	velocity = velocity.move_toward(target, acceleration * delta)
	move_and_slide()

	contacts.clear()
	for i in get_slide_collision_count():
		var other := get_slide_collision(i).get_collider() as Node
		if other and not contacts.has(String(other.name)):
			contacts.append(String(other.name))


func _unhandled_key_input(event: InputEvent) -> void:
	if event.is_pressed() and not event.is_echo() and event.physical_keycode == KEY_TAB:
		manual_control = not manual_control
		stop()


## Remembers the current pose as the place reset() returns to.
func set_home() -> void:
	_home = global_transform


## Puts the drone back at its start pose, standing still.
func reset() -> void:
	global_transform = _home
	velocity = Vector3.ZERO
	stop()


## Clears the autonomous command so the drone hovers.
func stop() -> void:
	command_velocity = Vector3.ZERO
	command_yaw_rate = 0.0


## Heading in radians. 0 = facing -Z, positive = turned left.
func get_yaw() -> float:
	return global_rotation.y


## -1 when `negative` is held, +1 when `positive` is held, 0 otherwise.
func _axis(negative: Key, positive: Key) -> float:
	return float(Input.is_physical_key_pressed(positive)) \
			- float(Input.is_physical_key_pressed(negative))
