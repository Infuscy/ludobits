local M = {}

local function multiply(v, n)
	return vmath.vector3(v.x * n, v.y * n, v.z * n)
end

local function divide(v, n)
	return vmath.vector3(v.x / n, v.y / n, v.z / n)
end

local function apply_force(o, f)
	o.velocity = o.velocity + multiply(f, o.inv_mass)
end

local function positional_correction(a, b, penetration, normal)
	local slop = 0.01
	local percent = 0.4
	local correction_mag = penetration > 0 and math.max(penetration - slop, 0) or math.min(penetration + slop, 0)
	local correction = multiply(normal, (correction_mag / (a.inv_mass + b.inv_mass)) * percent)

	go.set_position(go.get_position(a.id) - multiply(correction, a.inv_mass), a.id)
	go.set_position(go.get_position(b.id) + multiply(correction, b.inv_mass), b.id)
end


local function fix_collision(a, b, penetration, normal)
	local relative_velocity = b.velocity - a.velocity
	local velocity_along_normal = math.abs(vmath.dot(relative_velocity, normal))
	local j = velocity_along_normal / (a.inv_mass + b.inv_mass)

	local impulse = multiply(normal, j)
	apply_force(a, multiply(impulse, -1))
	apply_force(b, impulse)

	positional_correction(a, b, penetration, normal)
end

local function string_joint(a, b, distance)
	local j = {}
	
	function j.update()
		local relative_position = go.get_world_position(a.id) - go.get_world_position(b.id)
		local relative_distance = vmath.length(relative_position)
		if relative_distance > distance then
			fix_collision(a, b, relative_distance - distance, vmath.normalize(relative_position))
		end
		--print(go.get_world_position(a.url), go.get_world_position(b.url))
		msg.post("@render:", "draw_line", { start_point = go.get_world_position(a.id), end_point = go.get_world_position(b.id), color = vmath.vector4(1,1,1,1) })
	end
	
	return j
end


local function point_joint(a, p)
	local j = {}
	
	function j.update()
		a.velocity = vmath.vector3()
		go.set_position(p, a.id)
	end
	
	return j
end


local gravity = vmath.vector3(0, -1000, 0)
local air_friction = 2

local objects = {}

local joints = {}

function M.add_object(id, mass)
	assert(id)
	assert(mass and mass > 0)
	local o = { id = id, velocity = vmath.vector3(), mass = mass, inv_mass = 1 / mass }
	objects[id] = o
	return o
end

function M.add_string_joint(a, b, distance)
	table.insert(joints, string_joint(a, b, distance))
end

function M.add_point_joint(a, p)
	table.insert(joints, point_joint(a, p))
end

function M.update(dt)
	for id,o in pairs(objects) do
		apply_force(o, divide(gravity, o.inv_mass) * dt)
		apply_force(o, multiply(o.velocity, air_friction * dt))
		--go.set_position(go.get_position(id) + o.velocity * dt, id)
	end
	
	for _,j in pairs(joints) do
		j.update()
	end
	
	for id,o in pairs(objects) do
		go.set_position(go.get_position(id) + o.velocity * dt, id)
	end
end

function M.collision(id, message)
	positional_correction(objects[message.other_id], objects[id], message.distance, message.normal)
end


return M