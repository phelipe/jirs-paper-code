
"""
Numerical Solutions with Grünwald–Letnikov Definition
"""
function glfdiff(y::Vector, t::Vector, gam)
    h = t[2] - t[1]
    w = Float64[]
    push!(w, 1.)
    a0 = y[1]
    dy = []
    if (a0 != 0.) & (gam > 0.)
        push!(dy, sign(a0)*Inf)
    else
        push!(dy,0.)
    end
    for j in 2 : length(t)
        push!(w, last(w) * (1. - (gam + 1.)/ (j - 1.) ))   
    end
    for j in 2 : length(t)
        temp = (t[j] - t[j-1])^gam
        if temp == 0.
            temp = 1e-8
        end        
        push!(dy, (w[1:j]' * y[j:-1:1]) / (temp)) 
    end
    return dy    
end

function glfdiff(y::Vector, h::Float64, gam)
    w = Float64[]
    push!(w, 1.)
    a0 = y[1]
    dy = []
    if (a0 != 0.) & (gam > 0.)
        push!(dy, sign(a0)*Inf)
    else
        push!(dy,0.)
    end
    for j in 2 : length(y)
        push!(w, last(w) * (1. - (gam + 1.)/ (j - 1.) ))   
    end
    for j in 2 : length(y)
        push!(dy, (w[1:j]' * y[j:-1:1]) / (h^gam)) 
    end
    return dy    
end;    
    
