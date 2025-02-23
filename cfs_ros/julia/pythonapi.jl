using BoTrajOpt.RigidBodyDynamics
using BoTrajOpt.StaticArrays
using BoTrajOpt.KernelAbstractions
using BoTrajOpt: set_configurations!, t3d2posang, RobotCaches, RigidBodyCaches, MeshGraphCaches, rrtcfscart, distgoalf, t3d2poszdir, rrtcfsjoint, configidx, Trajectory
using BoTrajOpt.Rotations
using BoTrajOpt.ForwardDiff
using BoTrajOpt.Serialization

function plan_motion_joint(
    setup::RobotSetup,
    start_joint_names,
    start_joint_positions,
    goal_joint_names,
    goal_joint_positions,
)
    q_start = map(x ->
            start_joint_positions[findfirst(==(String(x)), start_joint_names)],
        (x -> x.name).(setup.dof_joints)
    )
    q_goal = map(x ->
            goal_joint_positions[findfirst(==(String(x)), goal_joint_names)],
        (x -> x.name).(setup.dof_joints)
    )
    s = MechanismState(setup.model)
    set_configurations!(s, setup.dof_joints, q_start)
    c_start = s |> configuration |> x -> SVector(x...)
    set_configurations!(s, setup.dof_joints, q_goal)
    c_goal = s |> configuration |> x -> SVector(x...)

    caches = RobotCaches(
        [RigidBodyCaches(setup.model) for _ in 1:Threads.nthreads()],
        MeshGraphCaches(setup.meshgraphs_cvx, CPU())
    )
    dof = setup.dof_joints
    meshgraphs = setup.meshgraphs_cvx
    collision_pairs = setup.collision_pairs
    dyn_constr = setup.dyn_constr
    safety_margin = 0.001

    N = length(c_start)
    T = eltype(c_start)

    trajj = rrtcfsjoint(
        caches,
        dof,
        c_start,
        c_goal,
        meshgraphs,
        collision_pairs[1:0],
        dyn_constr,
        safety_margin,
        0.1,
        1.0e-4,
        4000
    )

    qidx = configidx(setup.model, setup.dof_joints)
    ts = let
        vel_limit = dyn_constr[2] .|> x -> minimum(abs.(x))
        dts = diff(trajj) .|> x -> max(1.0e-3, maximum(abs.(x[qidx]) ./ vel_limit))
        range(0.0, max(1.0e-3, sum(dts)), length=length(trajj)) |> collect
    end
    target_fs = Vector{Union{Nothing,Tuple{Transform3D{T},Symbol}}}(nothing, length(trajj))
    target_cs = Vector{Union{Nothing,eltype(trajj)}}(nothing, length(trajj))
    target_cs[1] = c_start
    target_cs[end] = c_goal
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

    if length(trajj) > 2
        trjy = solve_cfsmotion(trjy)
    end

    ts = trjy.ts
    trajj = let
        ros_joints = (x -> findjoint(setup.model, String(x))).(start_joint_names)
        qidx = configidx(setup.model, ros_joints)
        reduce(hcat, (x -> x[qidx]).(trajj))
    end
    ts |> display
    trajj |> display
    ts, trajj
end

function plan_motion_cart(
    setup::RobotSetup,
    start_joint_names,
    start_joint_positions,
    frame_id,
    goal_link_name,
    goal_pos,
    goal_ori,
)
    q = map(x ->
            start_joint_positions[findfirst(==(String(x)), start_joint_names)],
        (x -> x.name).(setup.dof_joints)
    )
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
        ), :t3d2posang)

    caches = RobotCaches(
        [RigidBodyCaches(setup.model) for _ in 1:Threads.nthreads()],
        MeshGraphCaches(setup.meshgraphs_cvx, CPU())
    )
    dof = setup.dof_joints
    meshgraphs = setup.meshgraphs_cvx
    collision_pairs = setup.collision_pairs
    dyn_constr = setup.dyn_constr
    safety_margin = 0.001

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
        1.0,
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
        dts = diff(trajj) .|> x -> max(1.0e-3, maximum(abs.(x[qidx]) ./ vel_limit))
        range(0.0, sum(dts), length=length(trajj)) |> collect
    end
    target_fs = let
        target_fs = Vector{Union{Nothing,Tuple{Transform3D{T},Symbol}}}(nothing, length(trajj))
        target_fs[end] = target_f
        target_fs
    end
    target_cs = Vector{Union{Nothing,eltype(trajj)}}(nothing, length(trajj))
    target_cs[1] = c
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

    if length(trajj) > 2
        trjy = solve_cfsmotion(trjy)
    end

    ts = trjy.ts
    trajj = let
        ros_joints = (x -> findjoint(setup.model, String(x))).(start_joint_names)
        qidx = configidx(setup.model, ros_joints)
        reduce(hcat, (x -> x[qidx]).(trajj))
    end
    ts, trajj
end
