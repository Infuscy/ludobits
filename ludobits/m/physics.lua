local M = {}

M.MAX_LINEAR_CORRECTION = 0.2

local function clamp(v, min, max)
	v = math.max(v, min)
	v = math.min(v, max)
	return v
end

local function multiply(v, n)
	return vmath.vector3(v.x * n, v.y * n, v.z * n)
end

local function divide(v, n)
	return vmath.vector3(v.x / n, v.y / n, v.z / n)
end

local function add_force(o, f)
	o.impulse = o.impulse + f
	o.velocity.x = o.velocity.x + f.x * o.inv_mass
	o.velocity.y = o.velocity.y + f.y * o.inv_mass
	o.velocity.z = o.velocity.z + f.z * o.inv_mass
	--o.velocity = o.velocity + f
end

local function sub_force(o, f)
	o.impulse = o.impulse - f
	o.velocity.x = o.velocity.x - f.x * o.inv_mass
	o.velocity.y = o.velocity.y - f.y * o.inv_mass
	o.velocity.z = o.velocity.z - f.z * o.inv_mass
	--o.velocity = o.velocity - f
end

local function resolve_collision(a, b, normal, penetration)
	local relative_velocity = b.velocity - a.velocity
	local velocity_along_normal = math.abs(vmath.dot(normal, relative_velocity))
	local restitution = math.min(a.restitution, b.restitution)
	local j = (1 + restitution) * velocity_along_normal
	j = j / (a.inv_mass + b.inv_mass)
	
	local impulse = normal * j
	sub_force(a, impulse)
	add_force(b, impulse)
end

local function positional_correction(a, b, normal, penetration)
	local percent = 0.4
	local slop = 0.01
	local correction_mag = (penetration > 0) and math.max(penetration - slop, 0) or math.min(penetration + slop, 0);
	local correction = normal * ((correction_mag / (a.inv_mass + b.inv_mass)) * percent)
	go.set_position(go.get_position(a.id) - correction * a.inv_mass, a.id)
	go.set_position(go.get_position(b.id) + correction * b.inv_mass, b.id)
end

local function string_joint(a, b, distance)
	local j = {}
	
	local inv_k = 1 / (a.inv_mass + b.inv_mass)
	
	function j.solve_velocity()
		local relative_position = go.get_world_position(a.id) - go.get_world_position(b.id)
		local relative_distance = vmath.length(relative_position)
		local penetration = relative_distance - distance
		if penetration > 0 then
			local normal = vmath.normalize(relative_position)
			resolve_collision(a, b, normal, penetration)
		end
	end
	
	function j.solve_position()
		local relative_position = go.get_world_position(a.id) - go.get_world_position(b.id)
		local relative_distance = vmath.length(relative_position)
		local penetration = relative_distance - distance
		
		if penetration > 0 then
			local normal = vmath.normalize(relative_position)
			positional_correction(a, b, normal, penetration)
			
			msg.post("@render:", "draw_line", { start_point = go.get_world_position(a.id), end_point = go.get_world_position(b.id), color = vmath.vector4(1,1,1,1) })
			msg.post("@render:", "draw_line", { start_point = go.get_world_position(a.id) + vmath.vector3(10, 0, 0), end_point = go.get_world_position(a.id) + vmath.vector3(10, -20, 0), color = vmath.vector4(1,1,1,1) })
			return penetration <= 0.005
		else
			msg.post("@render:", "draw_line", { start_point = go.get_world_position(a.id), end_point = go.get_world_position(b.id), color = vmath.vector4(1,1,1,1) })
			return true
		end

	end
	
	return j
end


local function point_joint(a, p)
	local j = {}
	
	function j.solve_velocity()
		a.velocity = vmath.vector3()
	end
	
	function j.solve_position()
		go.set_position(p, a.id)
	end
	
	return j
end


local gravity = vmath.vector3(0, -100, 0)
local air_friction = 25

local objects = {}

local joints = {}

function M.add_object(id, mass, restitution)
	assert(id)
	assert(mass)
	if objects[id] then
		return objects[id]
	end
	local o = {
		id = id,
		velocity = vmath.vector3(),
		mass = mass,
		inv_mass = (mass > 0) and (1 / mass) or 0,
		restitution = restitution or 0,
		impulse = vmath.vector3(),
	}
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
	local ITERATIONS = 1

	-- warm start
	for i,o in pairs(objects) do
		local old_impulse = vmath.vector3(o.impulse)
		o.impulse = vmath.vector3(0)
		add_force(o, old_impulse)
	end

	local gravity_force = gravity * dt
	for i,o in pairs(objects) do
		add_force(o, gravity_force * (1 / o.inv_mass))
		add_force(o, o.velocity * (air_friction * dt))
	end

	for solver_iterations=1,ITERATIONS + 1 do
		for _,j in pairs(joints) do
			j.solve_velocity()
		end
	end

	for i,o in pairs(objects) do
		go.set_position(go.get_position(o.id) + o.velocity * dt, o.id)
	end

	for solver_iterations=1,ITERATIONS + 1 do
		local joints_solved = true
		for _,j in pairs(joints) do
			local joint_solved = j.solve_position()
			--print("solved", joint_solved)
			joints_solved = joints_solved and joint_solved
		end
		
		if joints_solved then break end
	end
end

function M.collision(id, message)
	local a = objects[message.other_id]
	local b = objects[id]
	local distance = message.distance
	local normal = message.normal
	
	resolve_collision(a, b, normal, distance)
	positional_correction(a, b, normal, distance)
end


return M