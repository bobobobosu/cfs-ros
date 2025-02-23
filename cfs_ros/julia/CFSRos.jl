module CFSRos

include("structs.jl")
include("cfsros_setup.jl")
include("cfsmotion.jl")
include("pythonapi.jl")

using PrecompileTools: @setup_workload, @compile_workload

@setup_workload begin
    @compile_workload begin
        # include("precompile_workload.jl")
    end
end

end # module CFSRos