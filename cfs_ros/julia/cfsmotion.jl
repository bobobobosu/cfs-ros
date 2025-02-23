using BoTrajOpt.Clarabel
using BoTrajOpt.JuMP
using BoTrajOpt.StaticArrays
using BoTrajOpt.RigidBodyDynamics
using BoTrajOpt.Rotations
using BoTrajOpt: t3d2posang, t3d2allposang, t3d2posxdir, t3d2posydir, t3d2poszdir, velocityidx, packvecs, set_velocities!, set_configurations!, configidx
using BoTrajOpt: cfstrajkinsetup!, cfstrajdynsetup!, cfstrajkinodynsetup!, cfschecktrajj, update_ys!, update_d0s_dgrads!
using BoTrajOpt: Trajectory
using BoTrajOpt.TimerOutputs
using BoTrajOpt: to

function validate_motion(
    caches::RobotCaches,
    dof_layouts::AbstractVector{Joint},
    dof_joints::AbstractVector{Joint},
    meshgraphs::Vector{Tuple{VisualElement,SparseMatrixCSC{Bool,Int},Vector{SVector{3,T}}}},
    collision_pairs::Vector{Tuple{Tuple{Int,Int},RigidBodyDynamics.Graphs.TreePath}},
    dyn_constr::NTuple{5,AbstractArray{Tuple{T,T}}},
    betterkeys::Vector{Symbol}=[:joint, :cart, :dyn, :coll]
) where {T}
    function checkfunc(trjy)
        vios = cfschecktrajj(
            caches, dof_layouts, dof_joints, meshgraphs, collision_pairs, dyn_constr,
            trjy.ts, trjy.cs,
            trjy.target_fs, trjy.target_cs, trjy.safety_margins
        )
        Dict(
            :duration => sum(diff(trjy.ts)),
            :joint => vios[:joint] |> sum,
            :cart => vios[:cart] |> sum,
            :dyn => vios[:dyn] |> sum,
            :coll => vios[:coll] |> sum,
        )
    end
    function bestfilter(
        thistraj, thisvio,
        thattraj, thatvio)
        # The criteria of "better":
        # Ordered by betterkeys (priority)
        # lexicographically comparing
        better = true
        for k in betterkeys
            if thisvio[k] < thatvio[k]
                better = true
                break
            elseif thisvio[k] > thatvio[k]
                better = false
                break
            else
                continue
            end
        end

        println("duration: ", thisvio[:duration])
        if better
            println("better: ", [
                thisvio[:joint], thisvio[:cart],
                thisvio[:dyn], thisvio[:coll]
            ])
            thistraj, thisvio
        else
            println("not better: ", [
                thisvio[:joint], thisvio[:cart],
                thisvio[:dyn], thisvio[:coll]
            ])
            thattraj, thatvio
        end
    end

    function processtraj(thistraj, thattraj, thatvio)
        if isnothing(thattraj)
            thattraj = thistraj
        end
        if isnothing(thatvio)
            thatvio = Dict(
                :duration => Inf,
                :joint => Inf,
                :cart => Inf,
                :dyn => Inf,
                :coll => Inf
            )
        end
        thisvio = checkfunc(thistraj)
        bettertraj, bettervio = bestfilter(
            thistraj, thisvio,
            thattraj, thatvio)

        bettertraj |> deepcopy,
        bettervio |> deepcopy,
        thisvio |> deepcopy
    end
end


function solve_cfsmotion(
    trjy::Trajectory
)
    function kinvars(m::Model, trjy::Trajectory)
        g = @variable(m, [1:lastindex(trjy.gidx)])
        set_start_value.(g, trjy.cs[1][trjy.gidx])

        q = Matrix{VariableRef}(undef, lastindex(trjy.qidx), lastindex(trjy.ts))
        for i in 1:lastindex(trjy.ts)
            q[:, i] = @variable(m, [1:lastindex(trjy.qidx)])
            set_start_value.(q[:, i], trjy.cs[i][trjy.qidx])
        end
        return g, q
    end

    function dynvars(m::Model, trjy::Trajectory)
        dt = @variable(m, [1:lastindex(diff(trjy.ts))])
        set_start_value.(dt, diff(trjy.ts))
        de = @variable(m, _)
        set_start_value.(de, 1.0e-3)
        return dt, de
    end

    function endpoints_constr(m::Model, q, dt)
        vel_lim = 1.0e-1
        @constraint(m, (q[:, 2] .- q[:, 1]) .>= dt[1] * -vel_lim)
        @constraint(m, (q[:, 2] .- q[:, 1]) .<= dt[1] * vel_lim)
        @constraint(m, (q[:, end-1] .- q[:, end]) .>= dt[end] * -vel_lim)
        @constraint(m, (q[:, end-1] .- q[:, end]) .<= dt[end] * vel_lim)
    end

    function update_tr(Δx::Union{AbstractVector{T},T}, ρs::Union{AbstractVector{T},T}; bounds::Tuple{T,T}=(-Inf, Inf)) where {T<:Real}
        Δx = ifelse.(ρs .> 0.75, Δx * 1.5, ifelse.(ρs .< 0.5, Δx * 0.5, Δx))
        clamp.(Δx, bounds...)
    end

    # validate
    validatef = validate_motion(
        trjy.caches, trjy.dof_layouts, trjy.dof_joints, trjy.meshgraphs_cvx, trjy.collision_pairs, trjy.dyn_constr,
        [:joint, :cart, :coll]
    )
    best, bestvio, _ = validatef(trjy, nothing, nothing)

    # kinematics optimization
    begin
        objective_val, objective_val_new = nothing, nothing
        Δdg = 1.0e-3
        Δdq = fill(1.0e-3, lastindex(trjy.cs))
        for iter in 1:50
            m = Model(() -> Clarabel.Optimizer())
            set_optimizer_attribute(m, "tol_gap_abs", 1.0e-9)
            set_optimizer_attribute(m, "tol_gap_rel", 1.0e-9)
            set_optimizer_attribute(m, "max_iter", 100000)
            set_silent(m)

            @variable(m, Δslack)
            g, q = kinvars(m, trjy)

            # update precomputation
            @timeit to "update_d0s_dgrads!" update_d0s_dgrads!(trjy)

            @timeit to "setup kin" begin
                # set trust region
                @views @constraint(m, g .- trjy.cs[1][trjy.gidx] .<= Δdg + Δslack)
                @views @constraint(m, g .- trjy.cs[1][trjy.gidx] .>= -Δdg - Δslack)
                @timeit to "trust region" begin
                    n = lastindex(trjy.cs)
                    @views @constraint(m, [i = 1:n], q[:, i] .- trjy.cs[i][trjy.qidx] .<= Δdq[i] + Δslack)
                    @views @constraint(m, [i = 1:n], q[:, i] .- trjy.cs[i][trjy.qidx] .>= -Δdq[i] - Δslack)
                end

                cartobj = zero(QuadExpr)
                pathlenobj = zero(QuadExpr)
                @timeit to "cfstrajkinsetup!" begin
                    cartobj_, update_func, dg_ρ, dq_ρ = cfstrajkinsetup!(
                        trjy.caches, trjy.dof_layouts, trjy.dof_joints, trjy.meshgraphs_cvx, trjy.collision_pairs, trjy.dyn_constr,
                        m, g, q,
                        trjy.ts, trjy.cs,
                        trjy.target_fs, trjy.target_cs, trjy.safety_margins,
                        trjy.d0s, trjy.dgrads
                    )
                    for i in 2:lastindex(trjy.ts)
                        add_to_expression!(pathlenobj, sum(abs2.(q[:, i] .- q[:, i-1])))
                    end
                    add_to_expression!(cartobj, (1.0 ./ max(1.0, sum(.!isnothing(trjy.target_fs)))) .* cartobj_)
                end

                begin
                    obj = zero(QuadExpr)
                    add_to_expression!(obj, 1.0e5 * cartobj)
                    add_to_expression!(obj, pathlenobj)
                    add_to_expression!(obj, 1.0e3 * (Δslack^2))
                    @objective(m, Min, obj)
                end
            end
            @timeit to "optimize! kin" optimize!(m)

            if !JuMP.is_solved_and_feasible(m)
                trajs_fail = trajs_view
                objective_val_new = nothing
                @warn "kinematics solve failed"
                # return (trajs_fail, sched)
            end

            println("cartobj: ", value(cartobj))
            println("pathlenobj: ", value(pathlenobj))
            println("slackobj: ", value(Δslack))
            println("objective_val: ", objective_value(m))
            objective_val_new = objective_value(m)
            g_result = value.(g)
            q_result = value.(q)
            @time begin
                apply_func = update_func
                apply_func()
            end

            begin
                # update model
                Δdq .= update_tr(Δdq, dq_ρ; bounds=(1.0e-3, 1.0e1))
                Δdg = update_tr(Δdg, minimum(reduce(min, dg_ρ)); bounds=(1.0e-3, 1.0e1))

                trjy.cs .= map(x -> let
                        c = trjy.cs[x] |> collect
                        c[trjy.gidx] .= g_result
                        c[trjy.qidx] .= q_result[:, x]
                        SVector(c...)
                    end, 1:lastindex(trjy.cs))
            end

            best, bestvio, thisvio = validatef(trjy, best, bestvio)

            # Early termination
            if !isnothing(objective_val) && objective_val_new > objective_val - 1.0e-4
                break
            else
                objective_val = objective_val_new
            end
        end
    end
    trjy = best

    # dynamics optimization
    begin
        println("dynamics optimization")
        objective_val, objective_val_new = nothing, nothing
        Δdt = fill(1.0e-3, lastindex(diff(trjy.ts)))
        for iter in 1:20
            m = Model(() -> Clarabel.Optimizer())
            set_optimizer_attribute(m, "tol_gap_abs", 1.0e-9)
            set_optimizer_attribute(m, "tol_gap_rel", 1.0e-9)
            set_optimizer_attribute(m, "max_iter", 100000)
            set_silent(m)

            @variable(m, Δslack)
            dt, de = dynvars(m, trjy)
            endpoints_constr(m, reduce(hcat, (x -> x[trjy.qidx]).(trjy.cs)), dt)

            # update precomputation
            @timeit to "update ys" update_ys!(trjy)

            @timeit to "setup dyns" begin
                # set trust region
                for i in 1:lastindex(dt)
                    @views @constraint(m, dt[i] .- diff(trjy.ts)[i] .<= Δdt[i] + Δslack)
                    @views @constraint(m, dt[i] .- diff(trjy.ts)[i] .>= -Δdt[i] - Δslack)
                end

                slackobj = zero(QuadExpr)
                durationobj = zero(QuadExpr)
                slackobj_, update_func, dt_ρ = cfstrajdynsetup!(
                    trjy.caches, trjy.dof_joints, trjy.dyn_constr,
                    m, dt, de,
                    trjy.ts, trjy.cs,
                    trjy.ya, trjy.ya_jac,
                    trjy.yj, trjy.yj_jac,
                    trjy.yτe, trjy.yτe_jac
                )
                add_to_expression!(durationobj, (1.0 / lastindex(trjy.ts)) * sum(dt))
                add_to_expression!(slackobj, (1.0 / lastindex(trjy.ts)) * slackobj_)

                obj = zero(QuadExpr)
                add_to_expression!(obj, 1.0e3 * (Δslack^2))
                add_to_expression!(obj, durationobj)
                @objective(m, Min, obj)
            end

            @timeit to "optimize! dyn" optimize!(m)
            println(objective_value(m))
            if !JuMP.is_solved_and_feasible(m)
                objective_val_new = objective_val
                @warn "dynamics solve failed"
                # return (trajs_fail, sched)
            end
            dts_result = value.(dt)
            println("durationobj: ", value(durationobj))
            println("objective_val: ", objective_value(m))
            println("Δslack: ", value(Δslack))

            # update trajs
            trjy.ts .= cumsum([0.0; dts_result])
            @timeit to "update ys" update_ys!(trjy)
            # update model
            @timeit to "collect update funcs" apply_func = update_func()
            @timeit to "apply update funcs" apply_func()

            # update trust region
            Δdt .= update_tr(Δdt, dt_ρ; bounds=(1.0e-3, 1.0e1))

            # update model and resolve
            objective_val_update = -1.0e6
            @timeit to "incremental solve dyn" for _ in 1:20
                @timeit to "optimize! incremental dyn" optimize!(m)
                termination_status(m) |> println
                if !JuMP.is_solved_and_feasible(m)
                    objective_val_new = objective_val
                    break
                end
                dts_result = value.(dt)
                println("durationobj: ", value(durationobj))
                println("Δslack: ", value(Δslack))
                println("objective_val: ", objective_value(m))
                println("objective_val_update: ", objective_val_update)
                # update trajs
                trjy.ts .= cumsum([0.0; dts_result])
                objective_val_new = objective_value(m)
                if abs(objective_value(m) - objective_val_update) > 1.0e-4
                    objective_val_update = objective_value(m)
                else
                    break
                end
                # update precomputation
                @timeit to "update ys" update_ys!(trjy)
                @timeit to "collect update funcs" apply_func = update_func()
                @timeit to "apply update funcs" apply_func()
            end

            best, bestvio, thisvio = validatef(trjy, best, bestvio)

            # Early termination
            if !isnothing(objective_val) && objective_val_new > objective_val - 1.0e-4
                break
            else
                objective_val = objective_val_new
            end
        end
    end
    trjy = best
end