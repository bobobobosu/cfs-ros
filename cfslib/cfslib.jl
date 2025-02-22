# using Pkg
# Pkg.add("RigidBodyDynamics")
# Pkg.add("Serialization")

using RigidBodyDynamics
using Serialization

struct RobotSetup{T}
    # Robot
    model::Mechanism{T}
end

function robotsetup_setup(urdfstring)
    # write urdfstring to temporary file
    temp_file = tempname()
    open(temp_file, "w") do f
        write(f, urdfstring)
    end
    model = parse_urdf(temp_file, remove_fixed_tree_joints=false)
    rm(temp_file)
    return RobotSetup(model)
end

function plan_motion_joint(
    setup::RobotSetup, 
    start_joint_names,
    start_joint_positions,
    goal_joint_names,
    goal_joint_positions,
    )
    # joint_names = string.(joint_names)
    # joint_positions = Float64.(joint_positions)

    # joint_names |> println
    # joint_positions |> println

    return nothing
end

function plan_motion_cart(
    setup::RobotSetup, 
    start_joint_names,
    start_joint_positions,
    goal_link_name,
    goal_pos,
    goal_ori,
)
    # link_name = string(link_name)
    # position_offset = Float64.(position_offset)
    # orientation = Float64.(orientation)

    # link_name |> println
    # position_offset |> println
    # orientation |> println

    ts = range(0, 10, length=100)
    trajj = zeros(6, length(ts))
    for i in 1:length(ts)
        trajj[:, i] .= start_joint_positions .+ 0.01 * (i - 1)
    end
    ts, trajj
end

# function configidx(s::MechanismState, jointdof::AbstractVector{<:Joint})
#     c = configuration(s)
#     reduce(vcat, (x -> (c[x].indices |> first)).(jointdof); init=Vector{Int}())
# end

# function configidx(model::Mechanism, jointdof::AbstractVector{<:Joint})
#     rngs = SegmentedVector(
#         Vector{Float64}(undef, num_positions(model)),
#         tree_joints(model),
#         num_positions
#     ) |> RigidBodyDynamics.ranges
#     reduce(vcat, map(x -> rngs[x], jointdof); init=Vector{Int}())
# end

# let
#     urdfstring = read("test.urdf", String)
#     setup = robotsetup_setup(urdfstring)
#     motion_plan_request = deserialize("testmpr.jld2")
#     start_joint_names = motion_plan_request.start_state.joint_state.name |> collect
#     start_joint_positions = motion_plan_request.start_state.joint_state.position |> collect

#     dof_joints = Vector{Joint}(filter(x ->
#             x.joint_type isa Revolute, tree_joints(setup.model)))


#     dof_joints = (x -> findjoint(setup.model, x)).(string.(start_joint_names))
#     qidx = configidx(setup.model, dof_joints)

#     # check goal type
#     if isempty(motion_plan_request.goal_constraints)
#         return nothing
#     else
#         if !isempty(motion_plan_request.goal_constraints[0].joint_constraints)
#             # joint constraints

#         elseif !isempty(motion_plan_request.goal_constraints[0].position_constraints)
#             # position constraints
#             pos_constr = motion_plan_request.goal_constraints[0].position_constraints[0].target_point_offset |> x -> [x.x, x.y, x.z]
#             pos_constr[1] |> Float32
#         else
#             return nothing
#         end
#     end

#     return nothing
# end
