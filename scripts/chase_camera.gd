extends Camera3D
## Chase camera: sits behind and above its parent and looks at it.
##
## Position, aim and lens are all set from code, so nothing done to this
## node in the editor viewport (moving, rotating, dragging the FOV handle)
## can change what the game shows. Tune it with the two values below.

## Where the camera sits relative to the parent: X right, Y up, Z behind.
@export var offset := Vector3(0.0, 3.5, 8.0)
## Vertical field of view in degrees.
@export var field_of_view := 75.0


func _ready() -> void:
	top_level = true  # do not inherit the parent's rotation or scale
	projection = PROJECTION_PERSPECTIVE
	fov = field_of_view
	h_offset = 0.0
	v_offset = 0.0
	_follow()


func _process(_delta: float) -> void:
	_follow()


func _follow() -> void:
	var target := get_parent() as Node3D
	if target == null:
		return
	var heading := Basis(Vector3.UP, target.global_rotation.y)
	global_position = target.global_position + heading * offset
	look_at(target.global_position + Vector3.UP * 0.5, Vector3.UP)
