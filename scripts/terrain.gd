@tool
extends StaticBody3D
## Heightmap terrain.
##
## Reads a greyscale image and builds both the visible mesh and the collision
## shape from it. One pixel = one grid vertex, spaced 1 m apart, so a 129x129
## image gives a 128 m x 128 m terrain centred on this node.
## Black = 0 m, white = max_height.

## Greyscale heightmap image.
@export var heightmap: Texture2D:
	set(value):
		heightmap = value
		if is_node_ready():
			rebuild()

## World height (metres) of a pure white pixel.
@export var max_height := 12.0:
	set(value):
		max_height = value
		if is_node_ready():
			rebuild()

## Ground colours, blended from lowest to highest ground.
@export var low_color := Color(0.16, 0.30, 0.13)
@export var mid_color := Color(0.33, 0.36, 0.18)
@export var high_color := Color(0.44, 0.41, 0.36)

var _width := 0   # samples along X
var _depth := 0   # samples along Z
var _heights := PackedFloat32Array()
var _generated: Array[Node] = []


func _ready() -> void:
	rebuild()


## Rebuilds mesh and collision from the current heightmap.
func rebuild() -> void:
	for node in _generated:
		if is_instance_valid(node):
			node.queue_free()
	_generated.clear()

	if heightmap == null:
		push_warning("Terrain: no heightmap assigned.")
		return

	var image := heightmap.get_image()
	if image.is_compressed():
		image.decompress()
	_width = image.get_width()
	_depth = image.get_height()

	_heights.resize(_width * _depth)
	for z in _depth:
		for x in _width:
			_heights[z * _width + x] = image.get_pixel(x, z).r * max_height

	_build_collision()
	_build_mesh()


## Ground height (world Y) under a world-space position.
## Positions outside the terrain are clamped to its edge.
func get_height(world_position: Vector3) -> float:
	if _heights.is_empty():
		return global_position.y
	var local := to_local(world_position)
	var fx := clampf(local.x + (_width - 1) * 0.5, 0.0, _width - 1.0)
	var fz := clampf(local.z + (_depth - 1) * 0.5, 0.0, _depth - 1.0)
	var x0 := mini(int(fx), _width - 2)
	var z0 := mini(int(fz), _depth - 2)
	var tx := fx - x0
	var tz := fz - z0
	var top := lerpf(_sample(x0, z0), _sample(x0 + 1, z0), tx)
	var bottom := lerpf(_sample(x0, z0 + 1), _sample(x0 + 1, z0 + 1), tx)
	return global_position.y + lerpf(top, bottom, tz)


## Terrain footprint in metres (X, Z).
func get_size() -> Vector2:
	return Vector2(_width - 1, _depth - 1)


## Number of height samples along X.
func get_columns() -> int:
	return _width


## Number of height samples along Z.
func get_rows() -> int:
	return _depth


## Every height sample as a world Y value, row by row (row along +Z,
## column along +X), rounded to the centimetre.
func get_world_heights() -> Array:
	var heights := []
	heights.resize(_heights.size())
	for i in _heights.size():
		heights[i] = snappedf(global_position.y + _heights[i], 0.01)
	return heights


func _sample(x: int, z: int) -> float:
	return _heights[z * _width + x]


func _vertex(x: int, z: int) -> Vector3:
	return Vector3(x - (_width - 1) * 0.5, _sample(x, z), z - (_depth - 1) * 0.5)


func _build_collision() -> void:
	var shape := HeightMapShape3D.new()
	shape.map_width = _width
	shape.map_depth = _depth
	shape.map_data = _heights

	var collider := CollisionShape3D.new()
	collider.name = "TerrainCollision"
	collider.shape = shape
	add_child(collider)
	_generated.append(collider)


func _build_mesh() -> void:
	var surface := SurfaceTool.new()
	surface.begin(Mesh.PRIMITIVE_TRIANGLES)

	# One texel per vertex, coloured by height.
	var colors := Image.create(_width, _depth, false, Image.FORMAT_RGB8)
	for z in _depth:
		for x in _width:
			colors.set_pixel(x, z, _color_for(_sample(x, z) / maxf(max_height, 0.001)))
			surface.set_uv((Vector2(x, z) + Vector2(0.5, 0.5)) / Vector2(_width, _depth))
			surface.add_vertex(_vertex(x, z))

	for z in _depth - 1:
		for x in _width - 1:
			var a := z * _width + x
			var b := a + 1
			var c := a + _width
			var d := c + 1
			surface.add_index(a)
			surface.add_index(b)
			surface.add_index(d)
			surface.add_index(a)
			surface.add_index(d)
			surface.add_index(c)

	surface.generate_normals()

	var material := StandardMaterial3D.new()
	material.albedo_texture = ImageTexture.create_from_image(colors)
	material.roughness = 1.0

	var mesh_instance := MeshInstance3D.new()
	mesh_instance.name = "TerrainMesh"
	mesh_instance.mesh = surface.commit()
	mesh_instance.material_override = material
	add_child(mesh_instance)
	_generated.append(mesh_instance)


func _color_for(t: float) -> Color:
	if t < 0.5:
		return low_color.lerp(mid_color, t * 2.0)
	return mid_color.lerp(high_color, (t - 0.5) * 2.0)
