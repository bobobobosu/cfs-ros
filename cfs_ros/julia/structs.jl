using BoTrajOpt.RigidBodyDynamics
using BoTrajOpt.MechanismGeometries
using BoTrajOpt.GeometryBasics
using BoTrajOpt.StaticArrays
using BoTrajOpt.SparseArrays
using BoTrajOpt.LazySets
using BoTrajOpt.RigidBodyDynamics: StateCache, WrenchesCache
using BoTrajOpt: RobotCaches

# RobotSetup
struct RobotSetup{T,D}
    # Robot
    model::Mechanism{T}
    meshes::Vector{Tuple{VisualElement,AbstractMesh}}
    meshes_cvx::Vector{Tuple{VisualElement,AbstractMesh}}
    meshgraphs_cvx::Vector{Tuple{VisualElement,SparseMatrixCSC{Bool,Int},Vector{SVector{3,T}}}}

    dyn_constr::NTuple{5,SVector{D,Tuple{T,T}}}

    dof_joints::Vector{Joint}
    dof_layouts::Vector{Joint}

    # Collisions
    collision_pairs::Vector{Tuple{Tuple{Int,Int}, RigidBodyDynamics.Graphs.TreePath}} # indexes to meshgraphs_cvx
end
