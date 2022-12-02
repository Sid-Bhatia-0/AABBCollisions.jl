import AABBCollisions as AABBC
import SimpleDraw as SD
import StaticArrays as SA

function SD.Rectangle(aabb::AABBC.AABB)
    return SD.Rectangle(SD.Point(Int(aabb.position.x) + 1, Int(aabb.position.y) + 1), Int(aabb.x_width), Int(aabb.y_width))
end

function start()
    T = Rational{Int}
    image = falses(64, 64)
    num_aabbs = 6
    aabbs = Vector{AABBC.AABB{T}}(undef, num_aabbs)
    body_types = Vector{AABBC.BodyType}(undef, num_aabbs)
    velocities = Vector{SA.SVector{2, T}}(undef, num_aabbs)
    collisions = Vector{Tuple{Int, Int, Int, T}}()

    aabbs[1] = AABBC.AABB(AABBC.Point(0 // 1, 0 // 1), 64 // 1, 8 // 1)
    body_types[1] = AABBC.STATIC
    velocities[1] = SA.SVector(zero(T), zero(T))

    aabbs[2] = AABBC.AABB(AABBC.Point(0 // 1, (64 - 8) // 1), 64 // 1, 8 // 1)
    body_types[2] = AABBC.STATIC
    velocities[2] = SA.SVector(zero(T), zero(T))

    aabbs[3] = AABBC.AABB(AABBC.Point(0 // 1, 8 // 1), 8 // 1, (64 - 2 * 8) // 1)
    body_types[3] = AABBC.STATIC
    velocities[3] = SA.SVector(zero(T), zero(T))

    aabbs[4] = AABBC.AABB(AABBC.Point((64 - 8) // 1, 8 // 1), 8 // 1, (64 - 2 * 8) // 1)
    body_types[4] = AABBC.STATIC
    velocities[4] = SA.SVector(zero(T), zero(T))

    aabbs[5] = AABBC.AABB(AABBC.Point(28 // 1, 28 // 1), 8 // 1, 8 // 1)
    body_types[5] = AABBC.STATIC
    velocities[5] = SA.SVector(zero(T), zero(T))

    aabbs[6] = AABBC.AABB(AABBC.Point(12 // 1, 12 // 1), 8 // 1, 8 // 1)
    body_types[6] = AABBC.DYNAMIC
    velocities[6] = SA.SVector(2 * one(T), one(T))

    @show aabbs
    println()
    @show body_types
    println()
    @show velocities

    for i in 1 : length(aabbs) - 1
        body_type1 = body_types[i]
        aabb1 = aabbs[i]
        velocity1 = velocities[i]

        dt = 1

        for j in i + 1 : length(aabbs)
            body_type2 = body_types[j]
            if !((body_type1 == AABBC.STATIC) && (body_type2 == AABBC.STATIC))
                aabb2 = aabbs[j]
                velocity2 = velocities[j]

                velocity12 = velocity1 - velocity2
                aabb12 = AABBC.get_relative_aabb(aabb1, aabb2)

                hit_dimension, hit_time = AABBC.collide(aabb1.position, aabb12, velocity12[1], velocity12[2])

                if !iszero(hit_dimension) && zero(hit_time) <= hit_time
                    @info "Collision occurred"

                    @show body_type1
                    @show aabb1
                    @show velocity1

                    @show body_type2
                    @show aabb2
                    @show velocity2

                    @show (i, j, hit_dimension, hit_time)
                    push!(collisions, (i, j, hit_dimension, hit_time))
                end
            end
        end
    end

    @show collisions

    fill!(image, false)
    for aabb in aabbs
        SD.draw!(image, SD.Rectangle(aabb), true)
    end

    SD.visualize(image)

    # sort and resolve collisions
    if isempty(collisions)
        # integrate the system for dt
    else
        sort!(collisions)
        i, j, hit_dimension, hit_time = first(collisions)
        # evolve the system for hit_time
        # resolve collision
    end

    return nothing
end

start()
