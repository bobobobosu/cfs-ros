include("CFSRos.jl")
using .CFSRos: robotsetup_setup
setup = robotsetup_setup("/mnt/storage/cfs-planner/cfs-ros/test.urdf")
setup.dof_joints

using BoTrajOpt.Serialization
plan_motion_cart_req = deserialize("../plan_motion_cart.jld2")

using BoTrajOpt.RigidBodyDynamics
using BoTrajOpt.StaticArrays
using BoTrajOpt.KernelAbstractions
using BoTrajOpt: set_configurations!, t3d2posang, RobotCaches, RigidBodyCaches, MeshGraphCaches, rrtcfscart, distgoalf, t3d2poszdir, rrtcfsjoint, configidx, Trajectory
using BoTrajOpt.Rotations
using BoTrajOpt.ForwardDiff
using BoTrajOpt.Serialization
trjy = let
    start_joint_names = plan_motion_cart_req["start_joint_names"]
    start_joint_positions = plan_motion_cart_req["start_joint_positions"]
    frame_id = plan_motion_cart_req["frame_id"]
    goal_link_name = plan_motion_cart_req["goal_link_name"]
    goal_ori = plan_motion_cart_req["goal_ori"]
    goal_pos = plan_motion_cart_req["goal_pos"]

    q = map(x -> start_joint_positions[findfirst(==(x), start_joint_names)], start_joint_names)
    s = MechanismState(setup.model)
    set_configurations!(s, setup.dof_joints, q)
    c = s |> configuration |> x -> SVector(x...)

    target_f = (relative_transform(
            s,
            default_frame(findbody(setup.model, goal_link_name)),
            default_frame(findbody(setup.model, frame_id))
        ) * Transform3D(
            default_frame(findbody(setup.model, goal_link_name)),
            default_frame(findbody(setup.model, goal_link_name)),
            QuatRotation(goal_ori...),
            SVector(goal_pos...)
        ), :t3d2poszdir)

    caches = RobotCaches(
        [RigidBodyCaches(setup.model) for _ in 1:Threads.nthreads()],
        MeshGraphCaches(setup.meshgraphs_cvx, CPU())
    )
    dof = setup.dof_joints
    meshgraphs = setup.meshgraphs_cvx
    collision_pairs = setup.collision_pairs
    dyn_constr = setup.dyn_constr
    safety_margin = 0.01

    N = length(c)
    T = eltype(c)

    pthcart = rrtcfscart(
        caches,
        dof,
        c,
        target_f,
        meshgraphs,
        collision_pairs,
        dyn_constr,
        safety_margin,
        6.28,
        1.0e-3,
        4000
    )

    # upscale
    upscale_tasks = Vector{NTuple{2,SVector{N,T}}}()
    for i in 2:lastindex(pthcart)
        push!(upscale_tasks, (pthcart[i-1], pthcart[i]))
    end

    upscale_pths = Vector{Vector{SVector{N,T}}}(undef, length(upscale_tasks))
    for i in 1:lastindex(upscale_tasks)
        upscale_pths[i] = rrtcfsjoint(
            caches,
            dof,
            first(upscale_tasks[i]),
            last(upscale_tasks[i]),
            meshgraphs,
            collision_pairs,
            dyn_constr,
            safety_margin,
            0.1,
            1.0e-4,
            4000
        )
    end

    qidx = configidx(setup.model, setup.dof_joints)
    trajj = let
        trajj = [pthcart[1]]
        map(x -> append!(trajj, x[2:end]), upscale_pths)
        trajj
    end
    ts = let
        vel_limit = dyn_constr[2] .|> x -> minimum(abs.(x))
        dts = diff(trajj) .|> x -> max(1.0e-3, maximum(x[qidx] ./ vel_limit))
        range(0.0, sum(dts), length=length(trajj)) |> collect
    end
    target_fs = let
        target_fs = Vector{Union{Nothing,Tuple{Transform3D{T},Symbol}}}(nothing, length(trajj))
        target_fs[end] = target_f
        target_fs
    end
    target_cs = Vector{Union{Nothing,eltype(trajj)}}(nothing, length(trajj))
    safety_margins = Vector{Union{Nothing,eltype(ts)}}(fill(safety_margin, length(trajj)))

    trjy = Trajectory(
        setup.model,
        setup.dof_layouts,
        setup.dof_joints,
        setup.collision_pairs,
        setup.dyn_constr,
        setup.meshgraphs_cvx,
        ts,
        trajj,
        target_fs,
        target_cs,
        safety_margins,
        CPU()
    )
end;

trjy.ts

trjy.cs

trjy.target_fs

solve_cfsmotion(trjy);

findjoint(setup.model, "base_joint")


setup.dof_joints[1].name

function configidx2(model::Mechanism, jointdof::AbstractVector{<:Joint})
    rngs = SegmentedVector(
        Vector{Float64}(undef, num_positions(model)),
        tree_joints(model),
        num_positions
    ) |> RigidBodyDynamics.ranges
    reduce(vcat, map(x -> rngs[x], jointdof); init=Vector{Int}())
end
ros_joints = (x -> findjoint(setup.model, String(x))).(plan_motion_cart_req["start_joint_names"])
qidx = configidx2(setup.model, ros_joints)
joints(setup.model)[10] |> typeof
ros_joints[1] |> typeof
AbstractVector{Joint}
setup.dof_joints[1] |> typeof

setup.dof_joints |> typeof
ros_joints |> typeof

setup.dof_joints |> typeof <: AbstractVector{Joint}
ros_joints |> typeof <: AbstractVector{<:Joint}
setup.meshgraphs_cvx .|> first .|> x -> x.geometry

setup.dyn_constr