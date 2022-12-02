module AABBCollisions

import SimpleDraw as SD
# import SaferIntegers as SI

struct Point{I}
    x::I
    y::I
end

# struct Line{I}
    # point1::I
    # point2::I
# end

struct AABB{I}
    position::Point{I}
    x_width::I
    y_width::I
end

@enum BodyType begin
    STATIC = 1
    DYNAMIC
end

get_relative_aabb(aabb1::AABB, aabb2::AABB) = AABB(Point(aabb2.position.x - aabb1.x_width, aabb2.position.y - aabb1.y_width), aabb2.x_width + aabb1.x_width, aabb2.y_width + aabb1.y_width)

# @enum InteractionType begin
    # NONE = 1
    # CONTACT
    # NORMAL_PENETRATION
    # ANGULAR_PENETRATION
    # SLIDING
# end

@enum AABBIntersectionType begin
    NO_INTERSECTION = 1
    POINT_INTERSECTION
    LINE_INTERSECTION
    REGION_INTERSECTION
end

struct PhysicsBody
end

get_x_min(point::Point) = point.x
get_x_max(point::Point) = point.x

get_y_min(point::Point) = point.y
get_y_max(point::Point) = point.y

# get_x_min(line::Line) = min(line.point1.x, line.point2.x)
# get_x_max(line::Line) = max(line.point1.x, line.point2.x)

# get_y_min(line::Line) = min(line.point1.y, line.point2.y)
# get_y_max(line::Line) = min(line.point1.y, line.point2.y)

get_x_min(aabb::AABB) = aabb.position.x
get_x_max(aabb::AABB) = aabb.position.x + aabb.x_width

get_y_min(aabb::AABB) = aabb.position.y
get_y_max(aabb::AABB) = aabb.position.y + aabb.y_width

get_x_extrema(shape) = (get_x_min(shape), get_x_max(shape))
get_y_extrema(shape) = (get_y_min(shape), get_y_max(shape))

is_valid(aabb::AABB) = (aabb.x_width >= zero(aabb.x_width)) || (aabb.y_width >= zero(aabb.y_width))

# is_reducible(line::Line) = line.point1 == line.point2
is_reducible(aabb::AABB) = iszero(aabb.x_width) || iszero(aabb.y_width)

function get_intersection_type(point::Point, aabb::AABB)
    @assert is_valid(aabb)
    @assert !is_reducible(aabb)

    x = point.x
    y = point.y
    x_min, x_max = get_x_extrema(aabb)
    y_min, y_max = get_y_extrema(aabb)

    if (x < x_min) || (x > x_max)
        return NO_INTERSECTION
    elseif (x == x_min) || (x == x_max)
        if (y < y_min) || (y > y_max)
            return NO_INTERSECTION
        elseif (y == y_min) || (y == y_max)
            return POINT_INTERSECTION
        else
            return LINE_INTERSECTION
        end
    else
        if (y < y_min) || (y > y_max)
            return NO_INTERSECTION
        elseif (y == y_min) || (y == y_max)
            return LINE_INTERSECTION
        else
            return REGION_INTERSECTION
        end
    end
end

function get_intersection_type(aabb1::AABB, aabb2::AABB)
    aabb2_expanded = AABB(Point(aabb2.position.x - aabb1.x_width, aabb2.position.y - aabb1.y_width), aabb2.x_width + aabb1.x_width, aabb2.y_width + aabb2.y_width)
    return get_intersection_type(aabb1.position, aabb2_expanded)
end

function collide(point::Point, aabb::AABB, v_x, v_y)
    @assert is_valid(aabb)
    @assert !is_reducible(aabb)

    intersection_type = get_intersection_type(point, aabb)
    @assert intersection_type != REGION_INTERSECTION

    x_0 = point.x
    y_0 = point.y

    x_min, x_max = get_x_extrema(aabb)
    y_min, y_max = get_y_extrema(aabb)

    hit_dimension = 0
    hit_time = 0 // 1

    if (iszero(v_x) && (iszero(v_y))) || (iszero(v_x) && ((x_0 <= x_min) || (x_0 >= x_max))) || (iszero(v_y) && ((y_0 <= y_min) || (y_0 >= y_max)))
        return hit_dimension, hit_time
    end

    t_x_min = (x_min - x_0) // v_x
    t_x_max = (x_max - x_0) // v_x
    t_x_entry, t_x_exit = minmax(t_x_min, t_x_max)

    t_y_min = (y_min - y_0) // v_y
    t_y_max = (y_max - y_0) // v_y
    t_y_entry, t_y_exit = minmax(t_y_min, t_y_max)

    if (t_x_entry < t_y_exit) && (t_y_entry < t_x_exit)
        entry_times = (t_x_entry, t_y_entry)
        velocity = (v_x, v_y)
        hit_dimension = argmax(entry_times)
        hit_time = entry_times[hit_dimension]
    end

    return hit_dimension, hit_time
end

# dynamic body vs. dynamic body
function get_normal_impulse(inv_mass_a, inv_mass_b, initial_velocity_ao, initial_velocity_bo, e, normal_o)
    initial_velocity_ba = initial_velocity_bo .- initial_velocity_ao
    initial_velocity_ba_normal_o = LA.dot(initial_velocity_ba, normal_o)
    final_velocity_ba_normal_o = -e * initial_velocity_ba_normal_o
    final_velocity_ba_normal_o = max(zero(final_velocity_ba_normal_o), final_velocity_ba_normal_o)
    j_ao_normal_o = (initial_velocity_ba_normal_o - final_velocity_ba_normal_o) / (inv_mass_a + inv_mass_b)
    j_bo_normal_o = -j_ao_normal_o
    return j_ao_normal_o, j_bo_normal_o
end

# dynamic body vs. kinematic body
get_normal_impulse(inv_mass_b, initial_velocity_ao, initial_velocity_bo, e, normal_o) = get_normal_impulse(zero(inv_mass_b), inv_mass_b, initial_velocity_ao, initial_velocity_bo, e, normal_o)

# dynamic body vs. static body
get_normal_impulse(inv_mass_b, initial_velocity_bo, e, normal_o) = get_normal_impulse(zero(inv_mass_b), inv_mass_b, zero(Vector2D), initial_velocity_bo, e, normal_o)

end
