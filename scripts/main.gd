extends Node3D
## Scene setup and HUD.
##
## Anything in the "on_terrain" group is dropped onto the ground at start-up,
## so obstacles and the human can be moved around in the editor (X/Z only)
## without having to line up their height by hand.

const Terrain := preload("res://scripts/terrain.gd")

## Drone starting height above the ground, in metres.
@export var start_altitude := 3.0

@onready var terrain: Terrain = $Terrain
@onready var drone: CharacterBody3D = $Drone
@onready var human: Node3D = $Human
@onready var hud: Label = $HUD/Label
@onready var bridge: Node = $PythonBridge


func _ready() -> void:
	for node in get_tree().get_nodes_in_group("on_terrain"):
		_place_on_ground(node)
	drone.global_position.y = terrain.get_height(drone.global_position) + start_altitude
	drone.set_home()


func _process(_delta: float) -> void:
	var altitude := drone.global_position.y - terrain.get_height(drone.global_position)
	var distance := drone.global_position.distance_to(human.global_position)
	var lines := [
		"Mode: %s  (Tab to switch)" % ("MANUAL" if drone.manual_control else "AUTONOMOUS"),
		"Python: %s" % _bridge_status(),
		"Altitude above ground: %.1f m" % altitude,
		"Speed: %.1f m/s" % drone.get_real_velocity().length(),
		"Distance to human: %.1f m" % distance,
		"",
		"W/S forward-back   A/D strafe   Q/E turn   Space/Shift up-down",
	]
	if not drone.contacts.is_empty():
		lines.insert(5, "Touching: " + ", ".join(drone.contacts))
	hud.text = "
".join(lines)


func _bridge_status() -> String:
	if bridge.is_client_connected():
		return "connected"
	if bridge.is_listening():
		return "waiting on 127.0.0.1:%d" % bridge.port
	return "off (port %d is busy)" % bridge.port


## Rests a box on the terrain. Uses the lowest ground point under its
## footprint so no corner is left floating on a slope.
func _place_on_ground(node: Node3D) -> void:
	var size := _box_size(node)
	var half := size * 0.5
	var ground := terrain.get_height(node.global_position)
	for corner in [Vector3(-half.x, 0, -half.z), Vector3(half.x, 0, -half.z),
			Vector3(-half.x, 0, half.z), Vector3(half.x, 0, half.z)]:
		ground = minf(ground, terrain.get_height(node.global_transform * corner))
	node.global_position.y = ground + half.y


func _box_size(node: Node3D) -> Vector3:
	for child in node.get_children():
		if child is MeshInstance3D and child.mesh is BoxMesh:
			return child.mesh.size
	return Vector3.ZERO
