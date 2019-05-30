include("fracionario.jl")

function saturador!(data, limite)
    state = data .> limite
    data[state] .= 0
    data[state] .= limite[state]
    return nothing
end


# função auxiliar do fracionário
  # remove o primeiro termo do vetor 'vec' e adiciona 'new' ao final dele      
function change!(vec, new)
    popfirst!(vec)
    push!(vec, new )
end        
                
# OBS: aqui coloquei para o tempo do controlador ser 10x menor que o tempo de captura dos dados             
# essa função simula o sistema com um PD analógico e retorna as resposta do sistema
# e o torque máximo em módulo com controlador fracionário
function simulationPDDigitalFractional(kp::Vector{Float64}, kv::Vector{Float64}, lambda::Vector{Float64},
    positions::Vector{T}, velocitys::Vector{T}, acelerations::Vector{T},
    jerks::Vector{T}, mechanism, state::MechanismState, Δt::Float64 = 1e-2,
    stime = 5.; showtorque = false, maxTorque = fill(Inf,num_positions(state)) ) where {T<:Function}
    # prepara ganhos do controlador PID fracionário
    kp = diagm(0 => kp)
    kv = diagm(0 => kv)
    max_torque = zeros(num_positions(state))
    jounts_quantity = num_positions(state)
    data_log = [[0.], [0.]]
    # cria controlador para o sistema
    step_controller = Δt / 10.
    function control!(τ::AbstractVector, t, state::MechanismState)
        e = map(x->x(t), positions) - configuration(state)
        de = zeros(jounts_quantity)
        for i=1:jounts_quantity
            push!(data_log[i], e[i])
            de[i] = glfdiff(data_log[i], step_controller, lambda[i])[end]
        end
        τ[:] = kp * e + kv * de
        saturador!(τ, maxTorque)    
        max_torque[:]= map( (a,b)-> max(a,abs(b)),max_torque[:],τ[:] )
        τ[:]
    end
    # cria variável de torques iniciais (todos nulos)
    # e faz com que o controlador seja discreto.
    τ = similar(configuration(state)) .* 0.0
    controller = PeriodicController(τ, step_controller, control!)
    digital_system = Dynamics(mechanism, controller)
    problem = ODEProblem(digital_system, state, (0., stime))
    solution = solve(problem, Tsit5(), saveat = Δt, maxiters = 1e9, force_dtmin=true, reltol=1e-5, abstol=1e-5)
    # organiza a saída para ser os erros
    data = solution.u
    time_out = solution.t[1:end-1]

    data = vcat((data')...)
    ex = Vector{Float64}[]
    ev = Vector{Float64}[]
    ea = Vector{Float64}[]
    ej = Vector{Float64}[]
    for i = 1:jounts_quantity
        position = data[:,i]
        velocity = data[:,(i+jounts_quantity)] 
        push!(ex, position[1:end-1])
        push!(ev, velocity[1:end-1])
    end
    ta = diff(time_out)
    tj = ta[2:end] 
    ea = map(x -> x./ta, diff.(ev)) 
    ej = map(x -> x./tj, diff.(ea))
    ta = time_out[1:length(ea[1])]
    tj = time_out[1:length(ej[1])]
    # retorna posião, velocidade, aceleração e jerk

    if showtorque
        return     ex, ev, ea, ej, time_out, ta, tj, max_torque
    else
        return ex, ev, ea, ej, time_out, ta, tj        
    end
end;

function erroPDDigitalFractional(kp::Vector{Float64}, kv::Vector{Float64}, lambda::Vector{Float64},
                    positions::Vector{T}, velocitys::Vector{T}, acelerations::Vector{T},
                    jerks::Vector{T}, mechanism, state::MechanismState, Δt::Float64 = 1e-2,
                    stime = 5.; maxTorque = fill(Inf,num_positions(state))  ) where {T<:Function}

    ex, ev, ea, ej, time_out, ta, tj = simulationPDDigitalFractional(kp, kv, lambda, positions, velocitys, acelerations,
                            jerks, mechanism, state, Δt, stime, maxTorque = maxTorque)
    for i = 1:num_positions(state)
        ex[i] =  [ positions[i](x) for x in time_out ] - ex[i]
        ev[i] =  [ velocitys[i](x) for x in time_out ] - ev[i]
        ea[i] =  [ acelerations[i](x) for x in ta ] - ea[i]
        ej[i] =  [ jerks[i](x) for x in tj ] - ej[i]
    end
    # retorna erro de posição, velocidade, aceleração e jerk
    ex, ev, ea, ej, time_out, ta, tj     
end;
                
########## PERTUBAÇÂO
# OBS: aqui coloquei para o tempo do controlador ser 10x menor que o tempo de captura dos dados             
# essa função simula o sistema com um PD analógico e retorna as resposta do sistema
# e o torque máximo em módulo com controlador fracionário
function simulationPDDigitalFractionalPert(kp::Vector{Float64}, kv::Vector{Float64}, lambda::Vector{Float64},
    positions::Vector{T}, velocitys::Vector{T}, acelerations::Vector{T},
    jerks::Vector{T}, mechanism, state::MechanismState, pert::Function , Δt::Float64 = 1e-2,
    stime = 5.; showtorque = false, maxTorque = fill(Inf,num_positions(state)) ) where {T<:Function}
    # prepara ganhos do controlador PID fracionário
    kp = diagm(0 => kp)
    kv = diagm(0 => kv)
    max_torque = zeros(num_positions(state))
    jounts_quantity = num_positions(state)
    data_log = [[0.], [0.]]
    # cria controlador para o sistema
    step_controller = Δt / 10.
    function control!(τ::AbstractVector, t, state::MechanismState)
        e = map(x->x(t), positions) - configuration(state)
        de = zeros(jounts_quantity)
        for i=1:jounts_quantity
            push!(data_log[i], e[i])
            de[i] = glfdiff(data_log[i], step_controller, lambda[i])[end]
        end
        τ[:] = kp * e + kv * de
        saturador!(τ, maxTorque)
        τ[:] = τ[:] .- pert(state, t)
        max_torque[:]= map( (a,b)-> max(a,abs(b)),max_torque[:],τ[:] )
        τ[:]
    end
    # cria variável de torques iniciais (todos nulos)
    # e faz com que o controlador seja discreto.
    τ = similar(configuration(state)) .* 0.0
    controller = PeriodicController(τ, step_controller, control!)
    digital_system = Dynamics(mechanism, controller)
    problem = ODEProblem(digital_system, state, (0., stime))
    solution = solve(problem, Tsit5(), saveat = Δt, maxiters = 1e9, force_dtmin=true, reltol=1e-5, abstol=1e-5)
    # organiza a saída para ser os erros
    data = solution.u
    time_out = solution.t[1:end-1]

    data = vcat((data')...)
    ex = Vector{Float64}[]
    ev = Vector{Float64}[]
    ea = Vector{Float64}[]
    ej = Vector{Float64}[]
    for i = 1:jounts_quantity
        position = data[:,i]
        velocity = data[:,(i+jounts_quantity)] 
        push!(ex, position[1:end-1])
        push!(ev, velocity[1:end-1])
    end
    ta = diff(time_out)
    tj = ta[2:end] 
    ea = map(x -> x./ta, diff.(ev)) 
    ej = map(x -> x./tj, diff.(ea))
    ta = time_out[1:length(ea[1])]
    tj = time_out[1:length(ej[1])]
    # retorna posião, velocidade, aceleração e jerk

    if showtorque
        return     ex, ev, ea, ej, time_out, ta, tj, max_torque
    else
        return ex, ev, ea, ej, time_out, ta, tj        
    end
end;

function erroPDDigitalFractionalPert(kp::Vector{Float64}, kv::Vector{Float64}, lambda::Vector{Float64},
                    positions::Vector{T}, velocitys::Vector{T}, acelerations::Vector{T},
                    jerks::Vector{T}, mechanism, state::MechanismState,pert::Function , Δt::Float64 = 1e-2,
                    stime = 5.; maxTorque = fill(Inf,num_positions(state))  ) where {T<:Function}

    ex, ev, ea, ej, time_out, ta, tj = simulationPDDigitalFractionalPert(kp, kv, lambda, positions, velocitys, acelerations,
                            jerks, mechanism, state, pert, Δt, stime, maxTorque = maxTorque)
    for i = 1:num_positions(state)
        ex[i] =  [ positions[i](x) for x in time_out ] - ex[i]
        ev[i] =  [ velocitys[i](x) for x in time_out ] - ev[i]
        ea[i] =  [ acelerations[i](x) for x in ta ] - ea[i]
        ej[i] =  [ jerks[i](x) for x in tj ] - ej[i]
    end
    # retorna erro de posição, velocidade, aceleração e jerk
    ex, ev, ea, ej, time_out, ta, tj     
end;                
                
