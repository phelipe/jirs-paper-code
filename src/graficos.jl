
function plot_xv(t, xdata, xdata2, xdata3, xref, vdata, vdata2, vdata3,vref, joint )
    p1 = plot(t, xdata[joint] , label = "PD", title = "Position Joint $(joint)", line=(2))
    p1 = plot!(t,xdata2[joint] , label = "OPD", line=(2,:dash))
    p1 = plot!(t,xdata3[joint] , label = "OFPD", line=(2,:dash))
    p1 = plot!(t, [xref[joint](j) for j in t], label = "Desired",line=(2, :dot))

    p2 = plot(t,vdata[joint], label = "PD", title = "Velocity Joint $(joint)", line=(2))
    p2 = plot!(t,vdata2[joint] , label = "OPD", line=(2,:dash))
    p2 = plot!(t,vdata3[joint] , label = "OFPD", line=(2,:dash))
    p2 = plot!(t, [vref[joint](j) for j in t], label = "Desired",line=(2, :dot))
    
    plot(p1, p2)
end

function plot_aj(ta, tj, adata, adata2, adata3, aref, jdata,jdata2,jdata3,  jref, joint )
    p1 = plot(ta,adata[joint] , label = "PD", title = "Aceleration Joint $(joint)", line=(2))
    p1 = plot!(ta,adata2[joint], label = "OPD", line=(2,:dash))
    p1 = plot!(ta,adata3[joint], label = "OFPD", line=(2,:dash))
    p1 = plot!(ta, [aref[joint](j) for j in ta], label = "Desired",line=(2, :dot))
    
    p2 = plot(tj,jdata[joint], label = "PD", title = "Jerk Joint $(joint)", line=(2))
    p2 = plot!(tj,jdata2[joint], label = "OPD", line=(2,:dash))
    p2 = plot!(tj,jdata3[joint], label = "OFPD", line=(2,:dash))
    p2 = plot!(tj, [jref[joint](j) for j in tj], label = "Desired",line=(2, :dot))
    
    plot(p1, p2)
end;


function relatorio(ex, exo, exf, ev, evo, evf, ea, eao,eaf, ej, ejo, ejf, joint)
    println("IAE PD   - Position (Joint $(joint)) :      $( iae((abs.(ex[joint]))) )")
    println("IAE OPD  - Position (Joint $(joint)) :      $( iae((abs.(exo[joint]))) )")
    println("IAE OFPD - Position (Joint $(joint)) :      $( iae((abs.(exf[joint]))) )")
    println("---------------------------------------------------------------------------------------------")

    println("IAE PD   - Velocity (Joint $(joint)) :      $( iae((abs.(ev[joint]))) )")
    println("IAE OPD  - Velocity (Joint $(joint)) :      $( iae((abs.(evo[joint]))) )")
    println("IAE OFPD - Velocity (Joint $(joint)) :      $( iae((abs.(evf[joint]))) )")
    println("---------------------------------------------------------------------------------------------")

    println("IAE PD   - Aceleration (Joint $(joint)) :     $( iae((abs.(ea[joint]))) )")
    println("IAE OPD  - Aceleration (Joint $(joint)) :     $( iae((abs.(eao[joint]))) )")
    println("IAE OFPD - Aceleration (Joint $(joint)) :     $( iae((abs.(eaf[joint]))) )")
    println("---------------------------------------------------------------------------------------------")

    println("IAE PD   - Jerk (Joint $(joint)) :      $( iae((abs.(ej[joint]))) )")
    println("IAE OPD  - Jerk (Joint $(joint)) :      $( iae((abs.(ejo[joint]))) )")
    println("IAE OFPD - Jerk (Joint $(joint)) :      $( iae((abs.(ejf[joint]))) )")
end
