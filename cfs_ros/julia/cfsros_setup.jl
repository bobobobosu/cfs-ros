using BoTrajOpt: load_mesh_tuples, t3d2cfg, set_configurations!, meshgraph

using BoTrajOpt.RigidBodyDynamics
using BoTrajOpt.MechanismGeometries
using BoTrajOpt.GeometryBasics
using BoTrajOpt.StaticArrays
using BoTrajOpt.Rotations
using BoTrajOpt.Serialization
using BoTrajOpt.LazySets
using BoTrajOpt: RobotCaches, MeshGraphCaches, RigidBodyCaches
using BoTrajOpt.KernelAbstractions

function robot_get_state(
    setup::RobotSetup,
    g::AbstractVector{T},
    q::AbstractVector{T}) where {T}
    s = MechanismState(setup.model)
    set_configurations!(s, setup.dof_layouts, g)
    set_configurations!(s, setup.dof_joints, q)
    s
end

function robotsetup_setup(urdf_path::String)
    model = parse_urdf(urdf_path, remove_fixed_tree_joints=false)
    meshes = load_mesh_tuples(visual_elements(model, URDFVisuals(urdf_path, tag="collision")))
    meshes_cvx = load_mesh_tuples(visual_elements(model, URDFVisuals(urdf_path, tag="collision")))

    dof_layouts = Vector{Joint}(filter(x ->
            (x.joint_type isa Planar) ||
                (x.joint_type isa Prismatic), tree_joints(model)))[1:0]
    dof_joints = Vector{Joint}(filter(x ->
            x.joint_type isa Revolute, tree_joints(model)))
    dyn_constr = let
        pos = SVector((dof_joints |>
                       x -> map(y -> y.position_bounds, x) |>
                            Iterators.flatten |> collect .|>
                            x -> (x.lower, x.upper))...)
        vel = SVector(map(x -> (-0.5, 0.5), pos))
        acc = SVector(map(x -> (-1.0, 1.0), pos))
        jerk = SVector(map(x -> (-1.0, 1.0), pos))
        eff = SVector((dof_joints |>
                       x -> map(y -> y.effort_bounds, x) |>
                            Iterators.flatten .|>
                            x -> (x.lower, x.upper))...)
        (pos, vel, acc, jerk, eff)
    end

    meshgraphs_cvx = map(x -> (x[1], meshgraph(x[2])...), meshes_cvx)

    collision_idxs = Vector{Tuple{Int,Int}}()
    append!(collision_idxs, Iterators.product(2:3, 5:7))
    append!(collision_idxs, Iterators.product(4:4, 7:7))
    append!(collision_idxs, Iterators.product(2:7, 8:lastindex(meshes_cvx)))
    collisions_bodies = (x -> (y -> body_fixed_frame_to_body(model, first(meshes_cvx[y]).frame)).(x)).(collision_idxs)
    collisions_paths = (x -> path(model, x[1], x[2])).(collisions_bodies)
    collision_pairs = map(x -> (x[1], x[2]), zip(collision_idxs, collisions_paths))

    RobotSetup{Float64,6}(
        model, meshes, meshes_cvx, meshgraphs_cvx, dyn_constr,
        dof_joints, dof_layouts,
        collision_pairs
    )
end;