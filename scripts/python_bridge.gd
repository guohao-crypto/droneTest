extends Node
## Lets a Python program fly the drone.
##
## Listens on 127.0.0.1 (this computer only). Messages are JSON, one per
## line, in both directions:
##
##   Godot -> Python
##     {"type": "world", ...}   once, right after connecting: terrain heights,
##                              obstacle boxes, the human, drone limits
##     {"type": "state", ...}   `state_rate_hz` times a second: position,
##                              velocity, heading, range sensors, contacts
##     {"type": "reset_done"}   answer to a reset; states after it are fresh
##     {"type": "error", ...}   a message from Python could not be used
##
##   Python -> Godot
##     {"type": "cmd", "velocity": [x, y, z], "yaw_rate": r}
##     {"type": "reset"}        put the drone back at its start point
##
## Vectors are [x, y, z] in metres, Y up, same axes as the Godot scene.
## See python/drone_sim.py for the client side.

const Terrain := preload("res://scripts/terrain.gd")

@export var port := 9361
## How often the drone state is sent.
@export var state_rate_hz := 30.0
## The drone hovers if no command arrives for this many seconds.
@export var command_timeout := 0.5
## Maximum distance the range sensors can see, in metres.
@export var sensor_range := 30.0
## Horizontal sensor directions in degrees, clockwise from straight ahead
## when seen from above (90 = right, 180 = behind, 270 = left).
@export var ray_angles_deg: PackedFloat32Array = [0, 45, 90, 135, 180, 225, 270, 315]

var _server := TCPServer.new()
var _peer: StreamPeerTCP
var _inbox := PackedByteArray()
var _outbox := PackedByteArray()
var _clock := 0.0
var _next_state_time := 0.0
var _last_command_time := -1.0

@onready var _drone: CharacterBody3D = get_tree().get_first_node_in_group("drone")
@onready var _human: Node3D = get_tree().get_first_node_in_group("human")
@onready var _terrain: Terrain = get_tree().get_first_node_in_group("terrain")


func _ready() -> void:
	var error := _server.listen(port, "127.0.0.1")
	if error != OK:
		push_warning("Python bridge: could not listen on port %d (error %d). Is another copy of the game running?" % [port, error])


## True while a Python program is connected.
func is_client_connected() -> bool:
	return _peer != null


func is_listening() -> bool:
	return _server.is_listening()


func _physics_process(delta: float) -> void:
	_clock += delta
	_accept()
	if _peer == null:
		return

	_peer.poll()
	if _peer.get_status() != StreamPeerTCP.STATUS_CONNECTED:
		_disconnect()
		return

	_receive()
	if _peer == null:
		return

	# Safety: a silent client must not leave the drone flying blind.
	if not _drone.manual_control and _clock - _last_command_time > command_timeout:
		_drone.stop()

	# Only queue a new state once the previous one has gone out, so a slow
	# client gets fresh data instead of a growing backlog.
	if _clock >= _next_state_time and _outbox.is_empty():
		_next_state_time = _clock + 1.0 / maxf(state_rate_hz, 1.0)
		_send(_state_message())
	_flush()


func _accept() -> void:
	if not _server.is_connection_available():
		return
	if _peer != null:
		_disconnect()  # newest client wins
	_peer = _server.take_connection()
	_peer.set_no_delay(true)
	_inbox.clear()
	_outbox.clear()
	_last_command_time = _clock
	_next_state_time = _clock
	_drone.stop()
	_drone.manual_control = false
	_send(_world_message())
	print("Python bridge: client connected.")


func _disconnect() -> void:
	if _peer != null:
		_peer.disconnect_from_host()
	_peer = null
	_drone.stop()
	_drone.manual_control = true
	print("Python bridge: client disconnected.")


func _receive() -> void:
	var available := _peer.get_available_bytes()
	if available <= 0:
		return
	var result := _peer.get_data(available)
	if result[0] != OK:
		_disconnect()
		return
	_inbox.append_array(result[1])

	while true:
		var newline := _inbox.find(10)
		if newline == -1:
			break
		var line := _inbox.slice(0, newline).get_string_from_utf8().strip_edges()
		_inbox = _inbox.slice(newline + 1)
		if not line.is_empty():
			_handle(line)


func _handle(line: String) -> void:
	var json := JSON.new()
	var message = json.data if json.parse(line) == OK else null
	if typeof(message) != TYPE_DICTIONARY:
		_send({"type": "error", "message": "not a JSON object: " + line.left(80)})
		return

	match message.get("type", ""):
		"cmd":
			var v = message.get("velocity", [0, 0, 0])
			if typeof(v) != TYPE_ARRAY or v.size() != 3:
				_send({"type": "error", "message": "velocity must be [x, y, z]"})
				return
			_drone.command_velocity = Vector3(float(v[0]), float(v[1]), float(v[2]))
			_drone.command_yaw_rate = float(message.get("yaw_rate", 0.0))
			_last_command_time = _clock
		"reset":
			_drone.reset()
			_last_command_time = _clock
			_send({"type": "reset_done"})
		_:
			_send({"type": "error", "message": "unknown message type: " + str(message.get("type"))})


func _send(message: Dictionary) -> void:
	_outbox.append_array((JSON.stringify(message) + "\n").to_utf8_buffer())


func _flush() -> void:
	if _outbox.is_empty():
		return
	var result := _peer.put_partial_data(_outbox)
	if result[0] != OK:
		_disconnect()
		return
	_outbox = _outbox.slice(result[1])


func _world_message() -> Dictionary:
	var size := _terrain.get_size()
	var obstacles := []
	for node in get_tree().get_nodes_in_group("obstacle"):
		obstacles.append(_box(node))
	return {
		"type": "world",
		"terrain": {
			"columns": _terrain.get_columns(),
			"rows": _terrain.get_rows(),
			"cell_size": 1.0,
			# world x, z of heights[0]
			"origin": [_r(_terrain.global_position.x - size.x * 0.5), _r(_terrain.global_position.z - size.y * 0.5)],
			# row-major: heights[row * columns + column], row along +Z, column along +X
			"heights": _terrain.get_world_heights(),
		},
		"obstacles": obstacles,
		"human": _box(_human),
		"drone": {
			"radius": 0.5,
			"max_speed": _drone.max_speed,
			"max_climb_speed": _drone.max_climb_speed,
			"max_yaw_rate": _drone.turn_speed,
		},
		"sensors": {"range": sensor_range, "ray_angles_deg": Array(ray_angles_deg)},
		"state_rate_hz": state_rate_hz,
	}


func _state_message() -> Dictionary:
	var position := _drone.global_position
	var ground := _terrain.get_height(position)
	var to_human := _human.global_position - position
	var heading := Basis(Vector3.UP, _drone.get_yaw())

	var ranges := []
	for angle in ray_angles_deg:
		var a := deg_to_rad(angle)
		ranges.append(_r(_ray(position, heading * Vector3(sin(a), 0.0, -cos(a)))))

	return {
		"type": "state",
		"time": _r(_clock),
		"position": _v(position),
		"velocity": _v(_drone.get_real_velocity()),
		"yaw": _r(_drone.get_yaw()),
		"ground_height": _r(ground),
		"altitude": _r(position.y - ground),
		"ranges": ranges,
		"range_down": _r(_ray(position, Vector3.DOWN)),
		"contacts": Array(_drone.contacts),
		"human_distance": _r(to_human.length()),
		"human_in_sight": _sees(_human),
		"manual_override": _drone.manual_control,
	}


## Distance to the first thing hit along `direction`, or sensor_range.
func _ray(from: Vector3, direction: Vector3) -> float:
	var query := PhysicsRayQueryParameters3D.create(from, from + direction * sensor_range)
	query.exclude = [_drone.get_rid()]
	var hit := _drone.get_world_3d().direct_space_state.intersect_ray(query)
	return from.distance_to(hit.position) if hit else sensor_range


## True if nothing blocks the straight line from the drone to `target`.
func _sees(target: Node3D) -> bool:
	var query := PhysicsRayQueryParameters3D.create(_drone.global_position, target.global_position)
	query.exclude = [_drone.get_rid()]
	var hit := _drone.get_world_3d().direct_space_state.intersect_ray(query)
	return hit.is_empty() or hit.collider == target


func _box(node: Node3D) -> Dictionary:
	var size := Vector3.ZERO
	for child in node.get_children():
		if child is MeshInstance3D and child.mesh is BoxMesh:
			size = child.mesh.size
	return {"name": String(node.name), "center": _v(node.global_position), "size": _v(size)}


func _v(v: Vector3) -> Array:
	return [_r(v.x), _r(v.y), _r(v.z)]


func _r(value: float) -> float:
	return snappedf(value, 0.001)
